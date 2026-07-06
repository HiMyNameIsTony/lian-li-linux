//! RGB controller: manages LED effects for all RGB-capable devices.
//!
//! Coordinates between native config effects and OpenRGB overrides.
//! Wired devices use the `RgbDevice` trait. Wireless devices stream
//! compressed per-LED frames via the `WirelessController`.

mod direct_color;
mod wireless;

pub use direct_color::{start_direct_color_writer, DirectColorBuffer};

use lianli_devices::traits::RgbDevice;
use lianli_devices::wireless::{WirelessController, WirelessFanType};
use lianli_shared::rgb::{
    RgbAppConfig, RgbDeviceCapabilities, RgbEffect, RgbMode, RgbPresetZone, RgbZoneInfo,
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};
use wireless::WirelessRgbState;

/// Minimum interval between drift-triggered re-uploads of a bank's RGB state.
/// Echo-latency false positives are already filtered upstream by the
/// post-send grace window in `WirelessController::drifted_macs`, so any
/// drift that reaches us is real; this floor only guards against an RF
/// upload storm if a bank's firmware echoes garbage indefinitely.
const WIRELESS_RESYNC_MIN_INTERVAL: Duration = Duration::from_secs(20);

/// Header repeats for one-shot RGB uploads (mode changes, IPC direct sets,
/// drift resyncs). The RF protocol has no acks; L-Connect's reliability
/// strategy (2026-05 usbmon capture) is 8–12 header repeats at ~21 ms gaps
/// (~200 ms per write). Our previous 2–4 repeats under-delivered on marginal
/// links — partially-received uploads are the prime suspect for banks
/// crashing back to firmware defaults ~1 min after an upload. The streaming
/// direct-color path keeps low repeats: the next frame is the retry.
const ONE_SHOT_HEADER_REPEATS: u8 = 8;

pub struct RgbController {
    /// Wired RGB devices keyed by device_id.
    wired: HashMap<String, Box<dyn RgbDevice>>,
    /// Wireless controller for RF-based LED control.
    wireless: Option<Arc<WirelessController>>,
    /// Wireless device state keyed by device_id ("wireless:xx:xx:xx:xx:xx:xx").
    wireless_state: HashMap<String, WirelessRgbState>,
    /// Current RGB config (from AppConfig).
    config: Option<RgbAppConfig>,
    /// Cached presets for restoring active preset LED colors.
    presets: Vec<lianli_shared::rgb::RgbPreset>,
    /// When true, OpenRGB has active control — suppress native config application.
    openrgb_active: bool,
    /// Last drift-triggered resync per wireless bank, for rate limiting.
    wireless_resync_at: HashMap<[u8; 6], Instant>,
}

impl RgbController {
    pub fn new(
        wired: HashMap<String, Box<dyn RgbDevice>>,
        wireless: Option<Arc<WirelessController>>,
    ) -> Self {
        let mut wireless_state = HashMap::new();

        if let Some(ref w) = wireless {
            for dev in w.devices() {
                let device_id = format!("wireless:{}", dev.mac_str());
                wireless_state.insert(
                    device_id,
                    WirelessRgbState::new(dev.mac, dev.fan_count, dev.fan_type),
                );
            }
        }

        info!(
            "RGB controller: {} wired device(s), {} wireless device(s)",
            wired.len(),
            wireless_state.len()
        );

        Self {
            wired,
            wireless,
            wireless_state,
            config: None,
            presets: Vec::new(),
            openrgb_active: false,
            wireless_resync_at: HashMap::new(),
        }
    }

    /// Apply an RGB config. Called on config load/change.
    pub fn apply_config(
        &mut self,
        config: &RgbAppConfig,
        presets: &[lianli_shared::rgb::RgbPreset],
    ) {
        self.config = Some(config.clone());
        self.presets = presets.to_vec();

        if !config.enabled {
            info!("RGB control disabled in config");
            return;
        }

        if config.openrgb_server {
            debug!("Skipping native RGB config — OpenRGB server is enabled");
            return;
        }

        if self.openrgb_active {
            debug!("Skipping native RGB config — OpenRGB has active control");
            return;
        }

        for dev_cfg in &config.devices {
            for zone_cfg in &dev_cfg.zones {
                if let Err(e) =
                    self.set_effect(&dev_cfg.device_id, zone_cfg.zone_index, &zone_cfg.effect)
                {
                    warn!(
                        "Failed to apply RGB effect to {} zone {}: {e}",
                        dev_cfg.device_id, zone_cfg.zone_index
                    );
                }
                if zone_cfg.swap_lr || zone_cfg.swap_tb {
                    if let Err(e) = self.set_fan_direction(
                        &dev_cfg.device_id,
                        zone_cfg.zone_index,
                        zone_cfg.swap_lr,
                        zone_cfg.swap_tb,
                    ) {
                        warn!(
                            "Failed to apply fan direction to {} zone {}: {e}",
                            dev_cfg.device_id, zone_cfg.zone_index
                        );
                    }
                }
            }

            if let Some(ref preset_name) = dev_cfg.active_preset {
                if let Some(preset) = presets
                    .iter()
                    .find(|p| &p.name == preset_name && p.device_id == dev_cfg.device_id)
                {
                    for zone_entry in &preset.zones {
                        if !zone_entry.colors.is_empty() {
                            if let Err(e) = self.set_direct_colors(
                                &dev_cfg.device_id,
                                zone_entry.zone,
                                &zone_entry.colors,
                            ) {
                                warn!(
                                    "Failed to restore preset '{}' zone {}: {e}",
                                    preset_name, zone_entry.zone
                                );
                            }
                        }
                    }
                    debug!(
                        "Restored active preset '{}' for {}",
                        preset_name, dev_cfg.device_id
                    );
                }
            }
        }
    }

    pub fn set_effect(
        &mut self,
        device_id: &str,
        zone: u8,
        effect: &RgbEffect,
    ) -> anyhow::Result<()> {
        if let Some(dev) = self.wired.get(device_id) {
            dev.set_zone_effect(zone, effect)?;
            debug!(
                "Set RGB effect on {device_id} zone {zone}: {:?}",
                effect.mode
            );
            return Ok(());
        }

        if let (Some(ref wireless), Some(state)) =
            (&self.wireless, self.wireless_state.get_mut(device_id))
        {
            apply_zone_effect_state(state, zone, effect)?;
            upload_wireless_state(wireless, state, device_id)?;
            return Ok(());
        }

        anyhow::bail!("RGB device not found: {device_id}");
    }

    /// Apply one effect to multiple zones of a device with a single RF
    /// upload at the end.
    ///
    /// OpenRGB UpdateMode is device-wide; calling `set_effect` per zone
    /// would upload the full bank state once per sub-zone (15× the RF
    /// traffic on a 3-fan SL-Infinity bank — enough congestion to corrupt
    /// discovery polls and garble the uploads themselves).
    ///
    /// For wired devices and rejected zones, falls back to per-zone calls.
    /// Returns the first error encountered, but attempts every zone.
    pub fn set_effect_zones(
        &mut self,
        device_id: &str,
        zones: &[u8],
        effect: &RgbEffect,
    ) -> anyhow::Result<()> {
        if self.wired.contains_key(device_id) {
            // Wired path has no shared bank state — just delegate per-zone.
            let mut first_err = None;
            for zone in zones {
                if let Err(e) = self.set_effect(device_id, *zone, effect) {
                    if first_err.is_none() {
                        first_err = Some(e);
                    }
                }
            }
            return first_err.map(Err).unwrap_or(Ok(()));
        }

        if let (Some(ref wireless), Some(state)) =
            (&self.wireless, self.wireless_state.get_mut(device_id))
        {
            let mut applied_any = false;
            let mut first_err = None;
            for zone in zones {
                match apply_zone_effect_state(state, *zone, effect) {
                    Ok(()) => applied_any = true,
                    Err(e) => {
                        if first_err.is_none() {
                            first_err = Some(e);
                        }
                    }
                }
            }
            if applied_any {
                upload_wireless_state(wireless, state, device_id)?;
            }
            return first_err.map(Err).unwrap_or(Ok(()));
        }

        anyhow::bail!("RGB device not found: {device_id}");
    }

    pub fn set_direct_colors(
        &mut self,
        device_id: &str,
        zone: u8,
        colors: &[[u8; 3]],
    ) -> anyhow::Result<()> {
        if let Some(dev) = self.wired.get(device_id) {
            dev.set_direct_colors(zone, colors)?;
            return Ok(());
        }

        if let (Some(ref wireless), Some(state)) =
            (&self.wireless, self.wireless_state.get_mut(device_id))
        {
            let zone_idx = zone as usize;
            let total_zones = wireless_total_zones(state);
            let (slice_start, slice_len) = match wireless_zone_slice(state, zone_idx) {
                Some(s) => s,
                None => anyhow::bail!(
                    "Zone {zone} out of range (device has {total_zones} zones, fan_type={:?}, fan_count={})",
                    state.fan_type, state.fan_count
                ),
            };
            let copy_len = colors.len().min(slice_len);
            state.led_state[slice_start..slice_start + copy_len]
                .copy_from_slice(&colors[..copy_len]);
            // Direct mode supersedes any composite animation on this zone.
            state.sub_zone_effects.remove(&zone);

            let idx = effect_index_from_state(&state.led_state);
            wireless.send_rgb_direct(&state.mac, &state.led_state, &idx, ONE_SHOT_HEADER_REPEATS)?;
            return Ok(());
        }

        anyhow::bail!("RGB device not found: {device_id}");
    }

    /// Apply direct-color updates for multiple zones of a single device, then
    /// send the resulting full LED state in ONE RF transmission.
    ///
    /// `set_direct_colors` always sends the full bank state (because the RF
    /// protocol is per-bank, not per-zone), so calling it once per zone for an
    /// SL-Infinity bank with 3 fans triples the RF traffic for the same
    /// visual result. Batching here cuts that overhead.
    ///
    /// For wired devices and rejected zones, falls back to per-zone calls.
    /// Returns the first error encountered, but attempts every zone.
    /// `one_shot` selects delivery effort: an isolated apply (single click)
    /// has no follow-up frame to paper over RF loss, so it gets the full
    /// header-repeat treatment; mid-stream the next frame is the retry, so
    /// repeats stay minimal to preserve airtime.
    pub fn apply_direct_zones(
        &mut self,
        device_id: &str,
        zones: &[(u8, Vec<[u8; 3]>)],
        one_shot: bool,
    ) -> anyhow::Result<()> {
        if self.wired.contains_key(device_id) {
            // Wired path has no shared bank state — just delegate per-zone.
            let mut first_err = None;
            for (zone, colors) in zones {
                if let Err(e) = self.set_direct_colors(device_id, *zone, colors) {
                    if first_err.is_none() {
                        first_err = Some(e);
                    }
                }
            }
            return first_err.map(Err).unwrap_or(Ok(()));
        }

        if let (Some(ref wireless), Some(state)) =
            (&self.wireless, self.wireless_state.get_mut(device_id))
        {
            let total_zones = wireless_total_zones(state);
            let mut applied_any = false;
            for (zone, colors) in zones {
                let zone_idx = *zone as usize;
                let (slice_start, slice_len) = match wireless_zone_slice(state, zone_idx) {
                    Some(s) => s,
                    None => {
                        debug!(
                            "Skipping zone {zone} for {device_id}: out of range (total={total_zones}, fan_count={})",
                            state.fan_count
                        );
                        continue;
                    }
                };
                let copy_len = colors.len().min(slice_len);
                state.led_state[slice_start..slice_start + copy_len]
                    .copy_from_slice(&colors[..copy_len]);
                // Direct overrides any composite animation on this zone.
                state.sub_zone_effects.remove(zone);
                applied_any = true;
            }

            if applied_any {
                let idx = effect_index_from_state(&state.led_state);
                let repeats = if one_shot { ONE_SHOT_HEADER_REPEATS } else { 2 };
                wireless.send_rgb_direct(&state.mac, &state.led_state, &idx, repeats)?;
            }
            return Ok(());
        }

        anyhow::bail!("RGB device not found: {device_id}");
    }

    /// Re-send the cached RGB state to wireless banks whose firmware reset
    /// its lighting (drift detected via effect_index mismatch in discovery).
    ///
    /// Deliberately bypasses the openrgb_server / openrgb_active guards that
    /// make `apply_config` a no-op for OpenRGB users: the cached `led_state`
    /// is whatever was last applied, regardless of source, so re-sending it
    /// is always the right recovery.
    ///
    /// Rate-limited per bank (`WIRELESS_RESYNC_MIN_INTERVAL`) because drift
    /// stays flagged for 10–35 s after a send (firmware echo latency) and a
    /// false positive would otherwise re-upload full bank state at 1 Hz.
    /// Returns how many banks were actually re-sent.
    pub fn resync_wireless_state(&mut self, macs: &[[u8; 6]]) -> usize {
        let Some(ref wireless) = self.wireless else {
            return 0;
        };
        let now = Instant::now();
        let mut resent = 0;
        for (device_id, state) in &self.wireless_state {
            if !macs.contains(&state.mac) {
                continue;
            }
            // Nothing has been applied to this bank yet (fresh daemon start);
            // an all-black re-send would just turn its LEDs off.
            if state.sub_zone_effects.is_empty()
                && state.led_state.iter().all(|c| *c == [0, 0, 0])
            {
                continue;
            }
            let rate_limited = self
                .wireless_resync_at
                .get(&state.mac)
                .is_some_and(|t| now.duration_since(*t) < WIRELESS_RESYNC_MIN_INTERVAL);
            if rate_limited {
                continue;
            }
            // Recorded even on failure so a wedged TX isn't hammered at 1 Hz.
            self.wireless_resync_at.insert(state.mac, now);
            match upload_wireless_state(wireless, state, device_id) {
                Ok(()) => {
                    resent += 1;
                    info!("Re-sent cached RGB state to drifted bank {device_id}");
                }
                Err(e) => warn!("Failed to re-send RGB state to {device_id}: {e}"),
            }
        }
        resent
    }

    /// Current colors of a wireless zone, from the cached full-bank LED state.
    /// Wired devices don't cache LED state — returns None.
    pub fn zone_colors(&self, device_id: &str, zone: u8) -> Option<Vec<[u8; 3]>> {
        let state = self.wireless_state.get(device_id)?;
        let (start, len) = wireless_zone_slice(state, zone as usize)?;
        state.led_state.get(start..start + len).map(|s| s.to_vec())
    }

    /// The animated mode currently running on a wireless device, if any.
    /// Static/Direct state is baked into `led_state` (no `sub_zone_effects`
    /// entry) and reports None — callers should present that as Direct.
    pub fn active_animated_mode(&self, device_id: &str) -> Option<RgbMode> {
        self.wireless_state
            .get(device_id)?
            .sub_zone_effects
            .values()
            .next()
            .map(|e| e.mode)
    }

    pub fn capabilities(&self) -> Vec<RgbDeviceCapabilities> {
        let mut caps = Vec::new();

        for (device_id, dev) in &self.wired {
            caps.push(RgbDeviceCapabilities {
                device_id: device_id.clone(),
                device_name: dev.device_name(),
                supported_modes: dev.supported_modes(),
                zones: dev.zone_info(),
                supports_direct: dev.supports_direct(),
                supports_mb_rgb_sync: dev.supports_mb_rgb_sync(),
                total_led_count: dev.total_led_count(),
                supported_scopes: dev.supported_scopes(),
                supports_direction: dev.supports_direction(),
            });
        }

        for (device_id, state) in &self.wireless_state {
            let mut zones: Vec<RgbZoneInfo> = Vec::new();

            if let Some(total) = state.fan_type.total_led_count_override() {
                let zone_name = match state.fan_type {
                    WirelessFanType::Lc217 => "Case Ring",
                    WirelessFanType::Led88 => "Screen Ring",
                    _ => "LED Strip",
                };
                zones.push(RgbZoneInfo {
                    name: zone_name.to_string(),
                    led_count: total,
                });
            } else {
                if state.fan_type.is_aio() {
                    zones.push(RgbZoneInfo {
                        name: "Pump Head".to_string(),
                        led_count: state.fan_type.pump_led_count() as u16,
                    });
                }
                let sub_zones = state.fan_type.sub_zones();
                for fan_i in 0..state.fan_count {
                    for (sub_name, count) in &sub_zones {
                        zones.push(RgbZoneInfo {
                            name: if sub_zones.len() == 1 {
                                format!("Fan {}", fan_i + 1)
                            } else {
                                format!("Fan {} {}", fan_i + 1, sub_name)
                            },
                            led_count: *count as u16,
                        });
                    }
                }
            }

            let total_leds: u16 = zones.iter().map(|z| z.led_count).sum();

            caps.push(RgbDeviceCapabilities {
                device_id: device_id.clone(),
                device_name: state.fan_type.display_name().to_string(),
                supported_modes: vec![RgbMode::Static, RgbMode::Direct, RgbMode::Breathing],
                zones,
                supports_direct: true,
                supports_mb_rgb_sync: false,
                total_led_count: total_leds,
                supported_scopes: vec![],
                supports_direction: false,
            });
        }

        caps
    }

    pub fn set_mb_rgb_sync(&self, device_id: &str, enabled: bool) -> anyhow::Result<()> {
        if let Some(dev) = self.wired.get(device_id) {
            if !dev.supports_mb_rgb_sync() {
                anyhow::bail!("Device {device_id} does not support MB RGB sync");
            }
            dev.set_mb_rgb_sync(enabled)?;
            info!(
                "MB RGB sync {}: {device_id}",
                if enabled { "enabled" } else { "disabled" }
            );
            return Ok(());
        }
        anyhow::bail!("RGB device not found: {device_id}");
    }

    pub fn set_fan_direction(
        &self,
        device_id: &str,
        zone: u8,
        swap_lr: bool,
        swap_tb: bool,
    ) -> anyhow::Result<()> {
        if let Some(dev) = self.wired.get(device_id) {
            if !dev.supports_direction() {
                anyhow::bail!("Device {device_id} does not support fan direction");
            }
            dev.set_fan_direction(zone, swap_lr, swap_tb)?;
            debug!(
                "Set fan direction on {device_id} zone {zone}: swap_lr={swap_lr} swap_tb={swap_tb}"
            );
            return Ok(());
        }
        anyhow::bail!("RGB device not found: {device_id}");
    }

    /// Called when OpenRGB connects — suppress native config.
    pub fn set_openrgb_active(&mut self, active: bool) {
        if self.openrgb_active != active {
            self.openrgb_active = active;
            if active {
                info!("OpenRGB took control — suppressing native RGB config");
            } else {
                info!("OpenRGB released control");
                // Only restore native config if the OpenRGB server is disabled;
                // when the server is enabled, leave LEDs as-is so OpenRGB state persists.
                let server_enabled = self
                    .config
                    .as_ref()
                    .map(|c| c.openrgb_server)
                    .unwrap_or(false);
                if !server_enabled {
                    info!("Restoring native RGB config");
                    if let Some(config) = self.config.clone() {
                        let presets = self.presets.clone();
                        self.apply_config(&config, &presets);
                    }
                }
            }
        }
    }

    /// Compute zone count and LEDs-per-zone for a wireless device state.
    /// Override-based devices (V150, Strimer, LC217, Led88) are single-zone
    /// with all LEDs in one flat buffer.
    fn zone_layout(state: &WirelessRgbState) -> (usize, usize) {
        if state.fan_type.total_led_count_override().is_some() {
            return (1, state.led_state.len());
        }
        let total_zones = if state.fan_type.is_aio() {
            state.fan_count as usize + 1
        } else {
            state.fan_count as usize
        };
        (total_zones, state.leds_per_fan as usize)
    }

    pub fn get_zone_colors(&self, device_id: &str, zone: u8) -> Option<Vec<[u8; 3]>> {
        let state = self.wireless_state.get(device_id)?;
        let (_, leds_in_zone) = Self::zone_layout(state);
        let start = zone as usize * leds_in_zone;
        let end = (start + leds_in_zone).min(state.led_state.len());
        if start >= state.led_state.len() {
            return None;
        }
        Some(state.led_state[start..end].to_vec())
    }

    pub fn get_all_zone_colors(&self, device_id: &str) -> Option<Vec<RgbPresetZone>> {
        let state = self.wireless_state.get(device_id)?;
        let (total_zones, leds_in_zone) = Self::zone_layout(state);
        let mut zones = Vec::new();
        for z in 0..total_zones {
            let start = z * leds_in_zone;
            let end = (start + leds_in_zone).min(state.led_state.len());
            if start < state.led_state.len() {
                zones.push(RgbPresetZone {
                    zone: z as u8,
                    colors: state.led_state[start..end].to_vec(),
                    effect: None,
                });
            }
        }
        Some(zones)
    }

    pub fn is_wireless(&self, device_id: &str) -> bool {
        self.wireless_state.contains_key(device_id)
    }

    pub fn set_wireless(&mut self, wireless: Option<Arc<WirelessController>>) {
        self.wireless = wireless;
    }

    pub fn refresh_wireless_devices(&mut self) {
        if let Some(ref w) = self.wireless {
            let mut new_state = HashMap::new();
            for dev in w.devices() {
                let device_id = format!("wireless:{}", dev.mac_str());
                let led_state = self
                    .wireless_state
                    .get(&device_id)
                    .map(|s| s.led_state.clone());

                let mut state = WirelessRgbState::new(dev.mac, dev.fan_count, dev.fan_type);
                if let Some(leds) = led_state {
                    if leds.len() == state.led_state.len() {
                        state.led_state = leds;
                    }
                }

                new_state.insert(device_id, state);
            }
            self.wireless_state = new_state;
        }
    }
}

fn effect_index_from_state(led_state: &[[u8; 3]]) -> [u8; 4] {
    let mut h: u32 = 0x811c_9dc5;
    for px in led_state {
        for &b in px {
            h ^= b as u32;
            h = h.wrapping_mul(0x0100_0193);
        }
    }
    if h == 0 {
        h = 1;
    }
    h.to_be_bytes()
}

/// Render a solid color array for a single zone from an RgbEffect.
/// Whether a mode produces a multi-frame animation (and so should
/// participate in composition). Static, Off, and Direct don't.
fn pattern_is_animated(mode: RgbMode) -> bool {
    matches!(mode, RgbMode::Breathing)
}

/// Global frame budget for composite uploads. 480 frames × 25 ms = 12 s loop.
///
/// 25 ms (~40 fps) sits just above the firmware's ~20 ms playback floor —
/// fast enough for visually smooth fades, slow enough to leave headroom
/// before the firmware clamps. The daemon-side interval encoder pre-scales
/// by 8/5 to compensate for the firmware's internal 0.625× factor, so
/// `COMPOSITE_INTERVAL_MS` is in real milliseconds.
///
/// 12 s is wide enough that slow patterns like Breathing can give properly
/// relaxed cadences (1 cycle / 12s ≈ 5 BPM, like a meditative exhale) at
/// the slowest speed. LZO compresses the highly repetitive breathing/pulse
/// frames very efficiently so even hundreds of frames stay within a few
/// KB compressed.
const COMPOSITE_FRAMES: usize = 480;
const COMPOSITE_INTERVAL_MS: u16 = 25;

/// Render a composite animation covering every animated sub-zone of a
/// wireless device. Non-animated zones contribute their current `led_state`
/// values as a static layer.
///
/// The frame budget is fixed (`COMPOSITE_FRAMES` × `COMPOSITE_INTERVAL_MS`).
/// Each animated sub-zone's natural cycle is fitted into this window — at
/// the cost of slightly off-period playback for patterns whose natural
/// cycles don't divide the window evenly. Worth it: one upload covers
/// every zone simultaneously, regardless of how many or what mix of
/// patterns are active.
fn render_composite_frames(state: &WirelessRgbState) -> (Vec<Vec<[u8; 3]>>, u16) {
    let mut frames: Vec<Vec<[u8; 3]>> = (0..COMPOSITE_FRAMES)
        .map(|_| state.led_state.clone())
        .collect();

    for (zone_idx, effect) in &state.sub_zone_effects {
        let Some((slice_start, slice_len)) = wireless_zone_slice(state, *zone_idx as usize) else {
            continue;
        };
        match effect.mode {
            RgbMode::Breathing => paint_breathing(&mut frames, effect, slice_start, slice_len),
            _ => {}
        }
    }

    (frames, COMPOSITE_INTERVAL_MS)
}

/// Paint a breathing pattern over `[slice_start..slice_start+slice_len)` in
/// each frame, scaled by `effect.brightness` and modulated by a sine wave
/// whose period is determined by `effect.speed` relative to the composite
/// window length.
fn paint_breathing(
    frames: &mut [Vec<[u8; 3]>],
    effect: &RgbEffect,
    slice_start: usize,
    slice_len: usize,
) {
    let scale = (effect.brightness as f32 / 4.0).clamp(0.0, 1.0);
    // Speed → integer cycles-per-window. Must be integer so the animation
    // wraps cleanly at frame N (otherwise the loop restarts mid-pulse and
    // looks broken). Window is COMPOSITE_FRAMES × COMPOSITE_INTERVAL_MS
    // = 12 s, so frequency_Hz = cycles / 12.
    let cycles = breathing_cycles_per_window(effect.speed) as f32;
    let n = frames.len() as f32;
    // Capture the slice's per-LED base colors from frame 0 (which still
    // reflects the device's state.led_state at composition time). This lets
    // each LED breathe its own color — set per-LED via Direct first, then
    // pick Breathing, and the colors persist while only brightness pulses.
    let base_colors: Vec<[u8; 3]> = frames[0][slice_start..slice_start + slice_len].to_vec();
    for (i, frame) in frames.iter_mut().enumerate() {
        let t = (i as f32 * cycles) / n; // 0..cycles
        let phase = t.fract(); // 0..1 within current cycle
        let factor = (std::f32::consts::PI * phase).sin() * scale;
        for (j, slot) in frame[slice_start..slice_start + slice_len]
            .iter_mut()
            .enumerate()
        {
            let base = base_colors[j];
            *slot = [
                (base[0] as f32 * factor) as u8,
                (base[1] as f32 * factor) as u8,
                (base[2] as f32 * factor) as u8,
            ];
        }
    }
}

/// Speed → integer breathing cycles per ~12-second composite window.
/// Must be integer (and ideally evenly divide COMPOSITE_FRAMES = 480) so
/// the animation wraps cleanly at the frame boundary.
///
/// Mapped onto natural breathing/pulse cadences:
///
///   speed 0 → 1 cycle / 12s   ≈ 5 BPM  (meditative)
///   speed 1 → 2 cycles        ≈ 10 BPM (slow, relaxed)
///   speed 2 → 4 cycles        ≈ 20 BPM (default — brisk)
///   speed 3 → 6 cycles        ≈ 30 BPM (rapid)
///   speed 4 → 12 cycles       ≈ 60 BPM (pulse-like)
fn breathing_cycles_per_window(speed: u8) -> u32 {
    match speed.min(4) {
        0 => 1,
        1 => 2,
        2 => 4,
        3 => 6,
        _ => 12,
    }
}

/// Resolve an OpenRGB zone index into a (start, length) slice within the
/// bank's full `led_state` buffer. Returns None if the zone is out of range.
///
/// Layout per fan type:
/// - rgb-only: zone 0 covers all LEDs
/// - AIO: zone 0 = Pump Head, then per-fan sub-zones
/// - regular fan banks: per-fan sub-zones (1 zone for most, 5 zones for SL-Infinity)
/// Update a zone's `led_state` slice and `sub_zone_effects` entry for an
/// effect, without uploading. Callers decide when to upload (per-zone for
/// `set_effect`, once per bank for `set_effect_zones`).
fn apply_zone_effect_state(
    state: &mut WirelessRgbState,
    zone: u8,
    effect: &RgbEffect,
) -> anyhow::Result<()> {
    let zone_idx = zone as usize;
    let total_zones = wireless_total_zones(state);
    let (slice_start, slice_len) = match wireless_zone_slice(state, zone_idx) {
        Some(s) => s,
        None => anyhow::bail!(
            "Zone {zone} out of range (device has {total_zones} zones, fan_type={:?}, fan_count={})",
            state.fan_type,
            state.fan_count
        ),
    };

    // For non-animated modes (Static, Off): always overwrite the
    // slice with the rendered uniform color — that's the whole
    // point of Static.
    //
    // For animated modes (Breathing, …): preserve any existing
    // per-LED colors so the "set per-LED via Direct, then switch
    // to Breathing" workflow works (each LED breathes its own
    // color, matching how RAM controllers behave). But if the
    // slice is currently all-black (no Direct setup yet), seed
    // it with the effect's base color so the user sees SOMETHING
    // when they pick Breathing on a fresh device.
    if pattern_is_animated(effect.mode) {
        let slice = &state.led_state[slice_start..slice_start + slice_len];
        let slice_is_blank = slice.iter().all(|c| *c == [0, 0, 0]);
        if slice_is_blank {
            let zone_color = render_zone_color(effect, slice_len);
            state.led_state[slice_start..slice_start + slice_len].copy_from_slice(&zone_color);
        }
    } else {
        let zone_color = render_zone_color(effect, slice_len);
        state.led_state[slice_start..slice_start + slice_len].copy_from_slice(&zone_color);
    }

    // Track this effect for composition, OR remove it (Static / Off
    // are baked into led_state and don't need per-frame animation).
    if pattern_is_animated(effect.mode) {
        state.sub_zone_effects.insert(zone, effect.clone());
    } else {
        state.sub_zone_effects.remove(&zone);
    }

    Ok(())
}

/// Upload a bank's current state: one direct frame if nothing is animated,
/// otherwise one composite frame sequence covering all animated sub-zones.
fn upload_wireless_state(
    wireless: &WirelessController,
    state: &WirelessRgbState,
    device_id: &str,
) -> anyhow::Result<()> {
    let idx = effect_index_from_state(&state.led_state);

    if state.sub_zone_effects.is_empty() {
        wireless.send_rgb_direct(&state.mac, &state.led_state, &idx, ONE_SHOT_HEADER_REPEATS)?;
        debug!(
            "Set wireless RGB on {device_id}: {} LEDs (no animation)",
            state.led_state.len()
        );
    } else {
        let (frames, interval_ms) = render_composite_frames(state);
        wireless.send_rgb_frames(&state.mac, &frames, interval_ms, &idx, ONE_SHOT_HEADER_REPEATS)?;
        // Don't poison led_state with rendered frame contents — it
        // would cascade-corrupt subsequent set_effect calls (the
        // animation's "middle" frame can be all-black depending on
        // cycle parity, then the next zone sees a blank slice and
        // re-seeds it with the mode's default color, looking like
        // colors mysteriously reset). led_state stays as the
        // unmodulated base colors that the composite paints from.
        info!(
            "Uploaded composite ({} animated zone(s)) to {device_id}: {} frames @ {}ms",
            state.sub_zone_effects.len(),
            frames.len(),
            interval_ms
        );
    }
    Ok(())
}

fn wireless_zone_slice(state: &WirelessRgbState, zone_idx: usize) -> Option<(usize, usize)> {
    if state.fan_type.is_rgb_only() {
        return if zone_idx == 0 {
            Some((0, state.led_state.len()))
        } else {
            None
        };
    }
    let pump_count = if state.fan_type.is_aio() {
        state.fan_type.pump_led_count() as usize
    } else {
        0
    };
    if state.fan_type.is_aio() && zone_idx == 0 {
        return Some((0, pump_count));
    }
    let fan_zone_idx = if state.fan_type.is_aio() {
        zone_idx - 1
    } else {
        zone_idx
    };
    let sub_zones = state.fan_type.sub_zones();
    let zones_per_fan = sub_zones.len();
    let fan_idx = fan_zone_idx / zones_per_fan;
    let sub_idx = fan_zone_idx % zones_per_fan;
    if fan_idx >= state.fan_count as usize {
        return None;
    }
    let leds_per_fan = state.leds_per_fan as usize;
    let mut sub_start = 0usize;
    for (_, count) in &sub_zones[..sub_idx] {
        sub_start += *count as usize;
    }
    let len = sub_zones[sub_idx].1 as usize;
    let start = pump_count + fan_idx * leds_per_fan + sub_start;
    Some((start, len))
}

fn wireless_total_zones(state: &WirelessRgbState) -> usize {
    if state.fan_type.is_rgb_only() {
        return 1;
    }
    let zones_per_fan = state.fan_type.sub_zones().len();
    let pump_zones = if state.fan_type.is_aio() { 1 } else { 0 };
    pump_zones + state.fan_count as usize * zones_per_fan
}

fn render_zone_color(effect: &RgbEffect, led_count: usize) -> Vec<[u8; 3]> {
    let color = match effect.mode {
        RgbMode::Off => [0, 0, 0],
        _ => {
            let base = effect.colors.first().copied().unwrap_or([255, 255, 255]);
            let scale = (effect.brightness as f32 / 4.0).clamp(0.0, 1.0);
            [
                (base[0] as f32 * scale) as u8,
                (base[1] as f32 * scale) as u8,
                (base[2] as f32 * scale) as u8,
            ]
        }
    };
    vec![color; led_count]
}
