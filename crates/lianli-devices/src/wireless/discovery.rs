use super::transport::with_transport_recovery;
use super::{WirelessFanType, RX_IDS, USB_CMD_SEND_RF};
use anyhow::{bail, Context, Result};
use lianli_transport::usb::{UsbTransport, USB_TIMEOUT};
use parking_lot::Mutex;
use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

/// Highest rx endpoint the bind flow will ever assign (see get_rx_unused).
/// Anything above this in a device record is corruption, not a re-assignment
/// (observed live 2026-07-02: rx=202 for 5 minutes under 60 fps RF load —
/// every unicast to that bank was silently misaddressed).
const RX_TYPE_MAX: u8 = 14;

/// A wireless device discovered via the RX GetDev command.
/// Parsed from the 42-byte device record in the response.
#[derive(Debug, Clone)]
pub struct DiscoveredDevice {
    pub mac: [u8; 6],
    pub master_mac: [u8; 6],
    pub channel: u8,
    pub rx_type: u8,
    pub device_type: u8,
    pub fan_count: u8,
    pub fan_types: [u8; 4],
    pub fan_rpms: [u16; 4],
    pub current_pwm: [u8; 4],
    pub cmd_seq: u8,
    pub fan_type: WirelessFanType,
    pub list_index: u8,
    /// Coolant temperature in °C (WaterBlock/WaterBlock2 only, from byte 27)
    pub coolant_temp_c: Option<u8>,
    /// Effect index the device firmware is currently running. Drifts to
    /// device-default if the firmware resets idle; compare against the desired
    /// effect_index to detect that and re-send the RGB packet.
    pub effect_index: [u8; 4],
}

impl DiscoveredDevice {
    pub fn mac_str(&self) -> String {
        format!(
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            self.mac[0], self.mac[1], self.mac[2], self.mac[3], self.mac[4], self.mac[5],
        )
    }

    pub fn is_aio(&self) -> bool {
        self.fan_type.is_aio()
    }

    pub fn pump_rpm(&self) -> Option<u16> {
        if self.is_aio() {
            Some(self.fan_rpms[3])
        } else {
            None
        }
    }
}

impl fmt::Display for DiscoveredDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mac = self.mac_str();
        if self.fan_type.is_aio() {
            let temp_str = self
                .coolant_temp_c
                .map(|t| format!(", coolant={t}°C"))
                .unwrap_or_default();
            write!(
                f,
                "{} ({:?}, {} fans, pump={}rpm{temp_str}, ch={}, rx={})",
                mac, self.fan_type, self.fan_count, self.fan_rpms[3], self.channel, self.rx_type,
            )
        } else {
            write!(
                f,
                "{} ({:?}, {} fans, ch={}, rx={})",
                mac, self.fan_type, self.fan_count, self.channel, self.rx_type,
            )
        }
    }
}

/// Parse a 42-byte device record from GetDev response.
///
/// Record layout:
/// ```text
/// [0-5]   Device MAC (6 bytes)
/// [6-11]  Master MAC (6 bytes)
/// [12]    RF Channel
/// [13]    RX Type (radio endpoint)
/// [14-17] System time (ms * 0.625)
/// [18]    Device type (0=fan, 65=LC217, 255=master)
/// [19]    Fan count
/// [20-23] Effect index (4 bytes)
/// [24-26] Fan type bytes (3 bytes, per-slot)
/// [27]    Coolant temperature °C (WaterBlock/WaterBlock2 only)
/// [28-35] Fan speeds (4x u16 big-endian RPM)
/// [36-39] Current PWM (4 bytes)
/// [40]    Command sequence number
/// [41]    Validation marker (must be 0x1C = 28)
/// ```
pub(super) fn parse_device_record(data: &[u8], list_index: u8) -> Option<DiscoveredDevice> {
    if data.len() < 42 {
        return None;
    }

    if data[41] != 0x1C {
        debug!(
            "  Device record {list_index}: invalid marker 0x{:02x} (expected 0x1C)",
            data[41]
        );
        return None;
    }

    let device_type = data[18];

    if device_type == 0xFF {
        debug!("  Device record {list_index}: skipping master device");
        return None;
    }

    let mut mac = [0u8; 6];
    mac.copy_from_slice(&data[0..6]);

    let mut master_mac = [0u8; 6];
    master_mac.copy_from_slice(&data[6..12]);

    let channel = data[12];
    let rx_type = data[13];
    let reported_fan_count = data[19].min(4);

    let mut fan_types = [0u8; 4];
    fan_types.copy_from_slice(&data[24..28]);

    // data[19] is unreliable on wireless banks: most banks report 4 regardless
    // of how many fans are physically paired. fan_types[i] == 0 is a reliable
    // "empty slot" signal, so prefer counting non-zero slots and fall back to
    // the dongle-reported value only if fan_types is all-zero (transient probe).
    let detected = fan_types.iter().filter(|&&b| b != 0).count() as u8;
    let fan_count = if detected > 0 { detected } else { reported_fan_count };

    let fan_rpms = [
        u16::from_be_bytes([data[28], data[29]]),
        u16::from_be_bytes([data[30], data[31]]),
        u16::from_be_bytes([data[32], data[33]]),
        u16::from_be_bytes([data[34], data[35]]),
    ];

    let mut current_pwm = [0u8; 4];
    current_pwm.copy_from_slice(&data[36..40]);

    let cmd_seq = data[40];

    let fan_type = match device_type {
        10 => WirelessFanType::WaterBlock,
        11 => WirelessFanType::WaterBlock2,
        1..=9 => WirelessFanType::Strimer(device_type),
        65 => WirelessFanType::Lc217,
        66 => WirelessFanType::V150,
        88 => WirelessFanType::Led88,
        _ => fan_types
            .iter()
            .find(|&&b| b != 0)
            .map(|&b| WirelessFanType::from_fan_type_byte(b))
            .unwrap_or(WirelessFanType::Unknown),
    };

    let coolant_temp_c = if fan_type.is_aio() && data[27] > 0 {
        Some(data[27])
    } else {
        None
    };

    let mut effect_index = [0u8; 4];
    effect_index.copy_from_slice(&data[20..24]);

    Some(DiscoveredDevice {
        mac,
        master_mac,
        channel,
        rx_type,
        device_type,
        fan_count,
        fan_types,
        fan_rpms,
        current_pwm,
        cmd_seq,
        fan_type,
        list_index,
        coolant_temp_c,
        effect_index,
    })
}

/// Polls the RX device for the current device list.
///
/// Sends GetDev command (0x10, page=1) and parses the response into
/// full 42-byte device records.
pub(super) fn poll_and_discover(
    rx: &Arc<Mutex<UsbTransport>>,
    discovered_devices: &Arc<Mutex<Vec<DiscoveredDevice>>>,
    mobo_pwm: &Arc<AtomicU16>,
    master_mac: &Arc<Mutex<[u8; 6]>>,
    master_channel: &Arc<Mutex<u8>>,
    pending_addr: &Arc<Mutex<HashMap<[u8; 6], (u8, u8)>>>,
    limbo_since: &Arc<Mutex<HashMap<[u8; 6], Instant>>>,
) -> Result<()> {
    let mut cmd = vec![0u8; 64];
    cmd[0] = USB_CMD_SEND_RF;
    cmd[1] = 0x01;

    with_transport_recovery(rx, &RX_IDS, "RX", |handle| {
        handle.read_flush();
        handle
            .write(&cmd, USB_TIMEOUT)
            .context("sending GetDev command")?;
        Ok(())
    })?;
    let handle = rx.lock();

    let mut response = [0u8; 512];
    match handle.read(&mut response, Duration::from_millis(200)) {
        Ok(len) if len >= 4 => {
            if response[0] != USB_CMD_SEND_RF {
                info!(
                    "GetDev: unexpected response 0x{:02x}, will retry",
                    response[0]
                );
                bail!("GetDev: unexpected response 0x{:02x}", response[0]);
            }

            let device_count = response[1] as usize;

            // Mobo PWM extraction. High bit of byte[2] = unavailable flag.
            // When clear: off_time = byte[2] & 0x7F, on_time = byte[3]
            //   pwm = 255 * on_time / (on_time + off_time)
            let indicator = response[2];
            if indicator >> 7 == 1 {
                mobo_pwm.store(0xFFFF, Ordering::Relaxed);
            } else {
                let off_time = (indicator & 0x7F) as u16;
                let on_time = response[3] as u16;
                let denominator = off_time + on_time;
                if denominator > 0 {
                    let pwm = (255u16 * on_time / denominator).min(255);
                    mobo_pwm.store(pwm, Ordering::Relaxed);
                } else {
                    mobo_pwm.store(0xFFFF, Ordering::Relaxed);
                }
            }

            debug!("GetDev: {device_count} device(s) reported");

            if device_count == 0 || device_count > 12 {
                return Ok(());
            }

            let mut found = Vec::new();
            let mut offset = 4;

            for idx in 0..device_count {
                if offset + 42 > len {
                    debug!("GetDev: response truncated at device {idx}");
                    break;
                }

                // The master's own record (device_type 0xFF) is skipped by
                // parse_device_record, but its byte 12 is the master's LIVE
                // channel — the authoritative source (decompiled L-Connect
                // reads it from exactly this record; the GetMac reply's
                // channel is often stale, observed wrong in both directions).
                let rec = &response[offset..offset + 42];
                if rec[41] == 0x1C && rec[18] == 0xFF && rec[0..6] == *master_mac.lock() {
                    let ch = rec[12];
                    if ch != 0 {
                        let mut master_ch = master_channel.lock();
                        if *master_ch != ch {
                            warn!(
                                "Master dongle record reports channel {ch} (tracked {}) — following it",
                                *master_ch
                            );
                            *master_ch = ch;
                        }
                    }
                }

                if let Some(device) = parse_device_record(&response[offset..offset + 42], idx as u8)
                {
                    debug!(
                        "  [{}] {} type=0x{:02x} fans={} RPM=[{},{},{},{}] PWM=[{},{},{},{}]",
                        idx,
                        device,
                        device.device_type,
                        device.fan_count,
                        device.fan_rpms[0],
                        device.fan_rpms[1],
                        device.fan_rpms[2],
                        device.fan_rpms[3],
                        device.current_pwm[0],
                        device.current_pwm[1],
                        device.current_pwm[2],
                        device.current_pwm[3],
                    );
                    found.push(device);
                }

                offset += 42;
            }

            let mut devices = discovered_devices.lock();
            if !found.is_empty() {
                // Master and banks share one RF channel, and the network can
                // re-form on a different channel at runtime (interference
                // hop, dongle reset — observed live 2026-07-02: ch 8 -> 2).
                // Per-device sends follow the records automatically, but
                // broadcasts (master-clock heartbeat, SaveCfg) use
                // master_channel — left stale, the banks silently stop
                // hearing the heartbeat and fail-safe/reset. Track it.
                {
                    let local_mac = *master_mac.lock();
                    let mut bound = found.iter().filter(|d| d.master_mac == local_mac);
                    if let Some(first) = bound.next() {
                        let ch = first.channel;
                        if bound.all(|d| d.channel == ch) {
                            let mut master_ch = master_channel.lock();
                            if *master_ch != ch {
                                warn!(
                                    "Wireless network moved: channel {} -> {ch} — updating broadcast channel",
                                    *master_ch
                                );
                                *master_ch = ch;
                            }
                        }
                    }
                }
                // Sanitize against the last-accepted records: under heavy RF
                // load the dongle returns records with corrupt fields
                // (fan_count=0 for 80 s, rx=202, ch=40 — observed 2026-07-02
                // during a 60 fps stress run). Blindly adopting them starves
                // banks of PWM keepalives (fan_count=0 skip guard) or
                // misaddresses every unicast (RGB + PWM) until the next
                // clean poll — banks then watchdog-reset. Rules:
                //   - fan_count collapsing to 0 → keep the cached record;
                //   - rx above RX_TYPE_MAX → keep the cached record;
                //   - bound bank off the consensus network channel → cached;
                //   - an otherwise valid rx/channel change → adopt only when
                //     two consecutive polls agree (real re-forms persist;
                //     one-poll glitches don't get to redirect traffic).
                let found = {
                    let local_mac = *master_mac.lock();
                    let master_ch = *master_channel.lock();
                    let mut pending = pending_addr.lock();
                    let mut limbo = limbo_since.lock();
                    found
                        .into_iter()
                        .filter_map(|d| {
                            // Channel-0 is a real state, not corruption: the
                            // bank fell off the network and sits unjoined.
                            // Adopt the raw record so sends address the bank
                            // where it actually is — the 1 Hz PWM keepalive
                            // carries target rx/channel bytes (like
                            // L-Connect's bind command, decompiled
                            // SyncControlInfo) and re-admits it within
                            // seconds. Hiding ch=0 behind the cached copy
                            // redirected every send to a channel the bank
                            // couldn't hear (2026-07-05: 30+ min at 100%
                            // fans; only daemon restarts — which adopt the
                            // raw record at first sighting — ever healed it).
                            if d.master_mac == local_mac {
                                if d.channel == 0 {
                                    if let std::collections::hash_map::Entry::Vacant(e) =
                                        limbo.entry(d.mac)
                                    {
                                        e.insert(Instant::now());
                                        info!(
                                            "{}: off-network (channel 0) — addressing keepalives to its limbo channel to re-admit it",
                                            d.mac_str()
                                        );
                                    }
                                    pending.remove(&d.mac);
                                    return Some(d);
                                }
                                if limbo.remove(&d.mac).is_some() {
                                    info!(
                                        "{}: rejoined the network on channel {}",
                                        d.mac_str(),
                                        d.channel
                                    );
                                }
                            }
                            let Some(old) = devices.iter().find(|o| o.mac == d.mac) else {
                                // First sighting: no baseline to fall back on,
                                // accept unless the addressing is nonsense.
                                return (d.rx_type <= RX_TYPE_MAX).then_some(d);
                            };
                            if d.fan_count == 0 && old.fan_count > 0 {
                                debug!(
                                    "{}: record lost its fans (fan_count=0), keeping cached",
                                    d.mac_str()
                                );
                                return Some(old.clone());
                            }
                            if d.rx_type > RX_TYPE_MAX {
                                debug!(
                                    "{}: corrupt rx={} in record, keeping cached rx={}",
                                    d.mac_str(),
                                    d.rx_type,
                                    old.rx_type
                                );
                                return Some(old.clone());
                            }
                            if d.master_mac == local_mac && d.channel != master_ch {
                                // trace: fires every poll for the duration of
                                // a limbo episode; the actionable signal is
                                // the fan controller's limbo-rescue warn.
                                tracing::trace!(
                                    "{}: record channel {} != network channel {master_ch}, keeping cached",
                                    d.mac_str(),
                                    d.channel
                                );
                                return Some(old.clone());
                            }
                            let addr = (d.rx_type, d.channel);
                            if addr != (old.rx_type, old.channel) {
                                if pending.get(&d.mac) == Some(&addr) {
                                    pending.remove(&d.mac);
                                    return Some(d);
                                }
                                debug!(
                                    "{}: unconfirmed addressing change rx={} ch={} -> rx={} ch={}, keeping cached until confirmed",
                                    d.mac_str(),
                                    old.rx_type,
                                    old.channel,
                                    d.rx_type,
                                    d.channel
                                );
                                pending.insert(d.mac, addr);
                                return Some(old.clone());
                            }
                            pending.remove(&d.mac);
                            Some(d)
                        })
                        .collect::<Vec<_>>()
                };

                // RX-slot assignments can change when the network re-forms,
                // and the master has been observed piling several banks onto
                // one slot (2026-07-02: three banks on rx=0, two of which
                // stopped receiving unicast RGB entirely). Log assignment
                // changes; flag shared slots as a delivery-reliability risk.
                {
                    let local_mac = *master_mac.lock();
                    let rx_map = |list: &[DiscoveredDevice]| {
                        let mut m: Vec<([u8; 6], u8)> = list
                            .iter()
                            .filter(|d| d.master_mac == local_mac)
                            .map(|d| (d.mac, d.rx_type))
                            .collect();
                        m.sort_unstable();
                        m
                    };
                    let old_map = rx_map(&devices);
                    let new_map = rx_map(&found);
                    if !old_map.is_empty() && old_map != new_map {
                        let desc = new_map
                            .iter()
                            .map(|(mac, rx)| {
                                format!("{:02x}:{:02x}=rx{rx}", mac[0], mac[1])
                            })
                            .collect::<Vec<_>>()
                            .join(", ");
                        let mut slots: Vec<u8> =
                            new_map.iter().map(|(_, rx)| *rx).collect();
                        slots.sort_unstable();
                        if slots.windows(2).any(|w| w[0] == w[1]) {
                            warn!(
                                "RX slot assignment changed: {desc} — banks share a slot, unicast delivery may be unreliable"
                            );
                        } else {
                            info!("RX slot assignment changed: {desc}");
                        }
                    }
                }
                let old_count = devices.len();
                *devices = found;
                if old_count != devices.len() {
                    let local_mac = *master_mac.lock();
                    let bound = devices.iter().filter(|d| d.master_mac == local_mac).count();
                    let unbound = devices.len() - bound;
                    info!(
                        "Discovered {} wireless device(s) ({bound} bound, {unbound} unbound)",
                        devices.len()
                    );
                    for d in devices.iter().filter(|d| d.master_mac != local_mac) {
                        info!(
                            "  {} ({}) not bound to this dongle",
                            d.mac_str(),
                            d.fan_type.display_name()
                        );
                    }
                }
            }
        }
        Ok(_) => {}
        Err(lianli_transport::TransportError::Usb(rusb::Error::Timeout)) => {}
        Err(err) => {
            debug!("GetDev error: {err}");
        }
    }

    Ok(())
}
