use super::discovery::{poll_and_discover, DiscoveredDevice};
use super::transport::{open_any, with_transport_recovery};
use super::{
    CMD_GET_MAC_WAKE, CMD_RX_LCD_MODE, CMD_RX_QUERY_34, CMD_RX_QUERY_37, CMD_VIDEO_START,
    RF_CHUNKS, RF_CHUNK_SIZE, RF_DATA_SIZE, RF_SAVE_CFG, RF_SELECT, RX_IDS, TX_IDS,
    USB_CMD_GET_MAC, USB_CMD_RESET_ANOTHER, USB_CMD_SEND_RF,
};
use anyhow::{bail, Context, Result};
use lianli_transport::usb::{UsbTransport, USB_TIMEOUT};
use parking_lot::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

const TX_FAILURE_THRESHOLD: u32 = 5;

/// How long after an RGB send to suppress drift checks for that bank.
/// The firmware takes 10–35 s (measured on SL-INF banks, 2026-07-02) to echo
/// a freshly-sent effect_index back through discovery records; comparing
/// during that window reads as spurious drift and would trigger a redundant
/// re-upload after every send. Cost: a real firmware reset landing inside
/// the window takes up to this long to detect.
// At the 60 ms frame-loop interval the firmware refreshes its discovery-record
// echo within ~10 s of a send (measured 2026-07-02; zero false positives all
// day at 10 s). Shorter grace = faster rescue of a bank that missed a send.
const RGB_ECHO_GRACE: Duration = Duration::from_secs(10);

pub struct WirelessController {
    pub(super) tx: Option<Arc<Mutex<UsbTransport>>>,
    pub(super) rx: Option<Arc<Mutex<UsbTransport>>>,
    pub(super) poll_stop: Arc<AtomicBool>,
    pub(super) poll_thread: Option<JoinHandle<()>>,
    pub(super) video_mode_active: Arc<AtomicBool>,
    pub(super) master_mac: Arc<Mutex<[u8; 6]>>,
    pub(super) master_channel: Arc<Mutex<u8>>,
    pub(super) discovered_devices: Arc<Mutex<Vec<DiscoveredDevice>>>,
    /// Motherboard PWM duty cycle (0-255) extracted from RX GetDev response bytes [2:3].
    /// 0xFFFF means unavailable/not yet read.
    pub(super) mobo_pwm: Arc<AtomicU16>,
    pub(super) tx_failures: Arc<AtomicU32>,
    /// Per-bank effect_index of the last RGB upload, plus when it was sent
    /// (for the [`RGB_ECHO_GRACE`] window in [`Self::drifted_macs`]).
    pub(super) desired_effects: Arc<Mutex<std::collections::HashMap<[u8; 6], ([u8; 4], Instant)>>>,
    /// Per-bank unconfirmed (rx, channel) addressing change awaiting a second
    /// consecutive poll before adoption (see poll_and_discover debounce).
    pub(super) pending_addr: Arc<Mutex<std::collections::HashMap<[u8; 6], (u8, u8)>>>,
    /// Bound banks whose raw discovery records report channel 0 (off-network
    /// limbo: unreachable by any send, firmware fail-safe at 100% fans),
    /// keyed to when limbo was first observed. See [`Self::limbo_macs`].
    pub(super) limbo_since: Arc<Mutex<std::collections::HashMap<[u8; 6], Instant>>>,
}

impl Clone for WirelessController {
    fn clone(&self) -> Self {
        Self {
            tx: self.tx.clone(),
            rx: self.rx.clone(),
            poll_stop: Arc::clone(&self.poll_stop),
            poll_thread: None,
            video_mode_active: Arc::clone(&self.video_mode_active),
            master_mac: Arc::clone(&self.master_mac),
            master_channel: Arc::clone(&self.master_channel),
            discovered_devices: Arc::clone(&self.discovered_devices),
            mobo_pwm: Arc::clone(&self.mobo_pwm),
            tx_failures: Arc::clone(&self.tx_failures),
            desired_effects: Arc::clone(&self.desired_effects),
            pending_addr: Arc::clone(&self.pending_addr),
            limbo_since: Arc::clone(&self.limbo_since),
        }
    }
}

impl WirelessController {
    pub fn new() -> Self {
        Self {
            tx: None,
            rx: None,
            poll_stop: Arc::new(AtomicBool::new(false)),
            poll_thread: None,
            video_mode_active: Arc::new(AtomicBool::new(false)),
            master_mac: Arc::new(Mutex::new([0u8; 6])),
            master_channel: Arc::new(Mutex::new(8)),
            discovered_devices: Arc::new(Mutex::new(Vec::new())),
            mobo_pwm: Arc::new(AtomicU16::new(0xFFFF)),
            tx_failures: Arc::new(AtomicU32::new(0)),
            desired_effects: Arc::new(Mutex::new(std::collections::HashMap::new())),
            pending_addr: Arc::new(Mutex::new(std::collections::HashMap::new())),
            limbo_since: Arc::new(Mutex::new(std::collections::HashMap::new())),
        }
    }

    pub fn connect(&mut self) -> Result<()> {
        let mut tx = None;
        let max_retries = 3;

        for attempt in 1..=max_retries {
            match open_any(&TX_IDS) {
                Ok(device) => {
                    tx = Some(device);
                    break;
                }
                Err(e) if attempt < max_retries => {
                    debug!("TX device not found (attempt {attempt}/{max_retries}): {e}");
                    thread::sleep(Duration::from_millis(1000 * attempt as u64));
                }
                Err(e) => {
                    return Err(e).context("opening wireless TX dongle");
                }
            }
        }

        let mut tx = tx.context("TX device failed to open after retries")?;
        tx.detach_and_configure("TX")?;
        let tx_arc = Arc::new(Mutex::new(tx));

        let rx_arc = match open_any(&RX_IDS) {
            Ok(mut rx) => {
                rx.detach_and_configure("RX")?;
                rx.read_flush();
                Some(Arc::new(Mutex::new(rx)))
            }
            Err(_) => {
                warn!("RX dongle not found – telemetry disabled");
                None
            }
        };

        self.tx = Some(tx_arc);
        self.rx = rx_arc;
        self.tx_failures.store(0, Ordering::Relaxed);

        self.discover_master_mac()?;
        Ok(())
    }

    /// Discovers master MAC address and channel by querying TX with USB_GetMac.
    /// Tries the default channel first, then scans even, then odd as fallback.
    fn discover_master_mac(&self) -> Result<()> {
        let tx = self.tx.as_ref().context("TX device not available")?;
        info!("Discovering master MAC address and wireless channel...");

        let channels_to_try: Vec<u8> = std::iter::once(8u8)
            .chain((2..=38).filter(|&ch| ch != 8 && ch % 2 == 0))
            .chain((1..=39).filter(|&ch| ch % 2 == 1))
            .collect();

        for channel in channels_to_try {
            let mut cmd = vec![0u8; 64];
            cmd[0] = USB_CMD_GET_MAC;
            cmd[1] = channel;

            let handle = tx.lock();
            if handle.write(&cmd, USB_TIMEOUT).is_err() {
                drop(handle);
                continue;
            }

            let mut response = [0u8; 64];
            let len = match handle.read(&mut response, Duration::from_millis(500)) {
                Ok(len) => len,
                Err(_) => {
                    drop(handle);
                    continue;
                }
            };
            drop(handle);

            if len >= 7 && response[0] == USB_CMD_GET_MAC {
                let mut mac = self.master_mac.lock();
                mac.copy_from_slice(&response[1..7]);
                if mac.iter().any(|&b| b != 0) {
                    *self.master_channel.lock() = channel;
                    info!(
                        "Master MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} channel={}",
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel
                    );
                    if len >= 13 {
                        let fw_ver = u16::from_be_bytes([response[11], response[12]]);
                        debug!("Master firmware version: {fw_ver}");
                    }
                    return Ok(());
                }
            }
        }

        bail!("Failed to discover master MAC on any channel (tried 1-39)");
    }

    pub fn start_polling(&mut self) -> Result<()> {
        let tx = self
            .tx
            .as_ref()
            .cloned()
            .context("TX device must be connected before polling")?;
        let rx = self
            .rx
            .as_ref()
            .cloned()
            .context("RX device must be connected for device discovery")?;

        {
            let handle = tx.lock();
            handle
                .write(&CMD_GET_MAC_WAKE, USB_TIMEOUT)
                .context("sending TX wake probe")?;
        }

        thread::sleep(Duration::from_millis(500));

        self.video_mode_active.store(false, Ordering::Release);
        self.poll_stop.store(false, Ordering::SeqCst);

        let stop_flag = self.poll_stop.clone();
        let discovered_devices = Arc::clone(&self.discovered_devices);
        let mobo_pwm = Arc::clone(&self.mobo_pwm);
        let master_mac = Arc::clone(&self.master_mac);
        let master_channel = Arc::clone(&self.master_channel);
        let pending_addr = Arc::clone(&self.pending_addr);
        let limbo_since = Arc::clone(&self.limbo_since);

        let discovery_done = Arc::new(AtomicBool::new(false));
        let discovery_signal = discovery_done.clone();

        self.poll_thread = Some(thread::spawn(move || {
            let mut found_devices = false;
            let mut consecutive_errors = 0u32;
            let mut consecutive_successes = 0u32;
            let mut total_resets = 0u32;
            const MAX_RESETS: u32 = 3;
            while !stop_flag.load(Ordering::SeqCst) {
                if let Err(err) = poll_and_discover(
                    &rx,
                    &discovered_devices,
                    &mobo_pwm,
                    &master_mac,
                    &master_channel,
                    &pending_addr,
                    &limbo_since,
                ) {
                    consecutive_errors += 1;
                    consecutive_successes = 0;
                    info!("RX polling ({consecutive_errors}): {err:?}, continuing");
                    if consecutive_errors >= 5 {
                        total_resets += 1;
                        if total_resets > MAX_RESETS {
                            error!(
                                "RX dongle unresponsive after {MAX_RESETS} resets, \
                                 stopping wireless polling"
                            );
                            break;
                        }
                        warn!(
                            "5 consecutive RX errors, sending RX reset ({total_resets}/{MAX_RESETS})"
                        );
                        let handle = rx.lock();
                        let mut reset_cmd = vec![0u8; 64];
                        reset_cmd[0] = USB_CMD_RESET_ANOTHER;
                        if handle.write(&reset_cmd, USB_TIMEOUT).is_ok() {
                            let mut resp = [0u8; 64];
                            let _ = handle.read(&mut resp, Duration::from_millis(2000));
                        }
                        drop(handle);
                        thread::sleep(Duration::from_millis(500));
                        consecutive_errors = 0;
                        continue;
                    }
                    let backoff = if consecutive_successes == 0
                        && !discovery_signal.load(Ordering::Acquire)
                    {
                        Duration::from_millis(200)
                    } else {
                        Duration::from_secs((1 << consecutive_errors.min(5)).min(30))
                    };
                    thread::sleep(backoff);
                    continue;
                }
                consecutive_errors = 0;
                consecutive_successes += 1;
                total_resets = 0;
                if consecutive_successes >= 2 && !discovery_signal.load(Ordering::Acquire) {
                    discovery_signal.store(true, Ordering::Release);
                }
                if !found_devices && !discovered_devices.lock().is_empty() {
                    found_devices = true;
                }
                thread::sleep(Duration::from_millis(500));
            }
        }));

        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            if discovery_done.load(Ordering::Acquire) {
                info!("Wireless discovery stable, proceeding with device list");
                break;
            }
            if std::time::Instant::now() >= deadline {
                warn!("Wireless discovery timed out (5s) — will retry in background");
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        Ok(())
    }

    pub fn ensure_video_mode(&self) -> Result<()> {
        if self.video_mode_active.load(Ordering::Acquire) {
            return Ok(());
        }

        if self.tx.is_some() {
            let device_count = self.discovered_devices.lock().len().max(1);
            let master_ch = *self.master_channel.lock();
            self.tx_recover(|handle| {
                handle
                    .write(&CMD_VIDEO_START, USB_TIMEOUT)
                    .context("sending TX video start")?;
                thread::sleep(Duration::from_millis(2));
                for device_idx in 0..device_count {
                    let mut cmd = vec![0u8; 64];
                    cmd[0] = USB_CMD_SEND_RF;
                    cmd[1] = device_idx as u8;
                    cmd[2] = master_ch;
                    cmd[3] = 0xFF;
                    handle
                        .write(&cmd, USB_TIMEOUT)
                        .context("sending TX prep command")?;
                    thread::sleep(Duration::from_millis(1));
                }
                Ok(())
            })?;
            self.video_mode_active.store(true, Ordering::Release);
            info!("Video mode activated with {device_count} device(s)");
        }
        Ok(())
    }

    /// Broadcast a "master clock" sync packet (RF sub-command 0x14) carrying
    /// 220 bytes of CPU/GPU info. L-Connect sends this once per second; missing
    /// it appears to put the fan firmware into an autonomous fallback that
    /// occasionally spikes RPM. We send all-zero info bytes — the firmware
    /// only seems to need the heartbeat itself.
    pub fn send_master_clock(&self) -> Result<()> {
        let master_mac = *self.master_mac.lock();
        let master_ch = *self.master_channel.lock();

        let mut rf_data = vec![0u8; RF_DATA_SIZE];
        rf_data[0] = RF_SELECT;
        rf_data[1] = 0x14;
        rf_data[8..14].copy_from_slice(&master_mac);
        // rf_data[14..234] = cpuInfoParam (220 bytes, leave zero)

        self.tx_recover(|handle| {
            for chunk_idx in 0..RF_CHUNKS as u8 {
                let mut packet = vec![0u8; 64];
                packet[0] = USB_CMD_SEND_RF;
                packet[1] = chunk_idx;
                packet[2] = master_ch;
                packet[3] = 0xFF;

                let start = chunk_idx as usize * RF_CHUNK_SIZE;
                let end = start + RF_CHUNK_SIZE;
                packet[4..64].copy_from_slice(&rf_data[start..end]);

                handle
                    .write(&packet, USB_TIMEOUT)
                    .context("sending master clock packet")?;
                thread::sleep(Duration::from_millis(1));
            }
            Ok(())
        })?;
        Ok(())
    }

    /// Poll the TX dongle for liveness, mirroring L-Connect's 1 Hz
    /// QuerryMasterMac. The GetMac reply carries the master MAC, firmware
    /// version, and the dongle's TMOS system clock (0.625 ms ticks) —
    /// returns that clock in ms. A clock that goes backwards between polls
    /// means the dongle silently rebooted (USB brownout etc.) and likely
    /// dropped RF state; callers should treat that as a recovery trigger.
    pub fn query_master(&self) -> Result<u32> {
        let tx = self.tx.as_ref().context("TX device not connected")?;
        let channel = *self.master_channel.lock();

        let mut cmd = vec![0u8; 64];
        cmd[0] = USB_CMD_GET_MAC;
        cmd[1] = channel;

        let handle = tx.lock();
        handle.read_flush();
        handle
            .write(&cmd, USB_TIMEOUT)
            .context("sending GetMac liveness probe")?;
        let mut resp = [0u8; 64];
        let len = handle
            .read(&mut resp, Duration::from_millis(500))
            .context("reading GetMac liveness reply")?;
        drop(handle);

        if len < 13 || resp[0] != USB_CMD_GET_MAC {
            bail!("unexpected GetMac reply (len={len}, cmd=0x{:02x})", resp[0]);
        }

        // Reply layout (L-Connect QuerryMasterMac): [1..7] master MAC,
        // [7..11] TMOS clock BE, [11..13] firmware version BE.
        let tmos = u32::from_be_bytes([resp[7], resp[8], resp[9], resp[10]]);
        let clock_ms = (tmos as u64 * 5 / 8) as u32;

        let reported = &resp[1..7];
        if reported.iter().any(|&b| b != 0) {
            let mut mac = self.master_mac.lock();
            if *mac != *reported {
                warn!(
                    "Master MAC changed: {:02x?} -> {:02x?} (dongle re-paired?)",
                    *mac, reported
                );
                mac.copy_from_slice(reported);
            }
        }

        Ok(clock_ms)
    }

    /// Broadcast RF_SaveCfg: every bank persists its CURRENT lighting/fan
    /// state to flash as its power-on default, so a later firmware reset
    /// (idle watchdog, brownout) reverts to that state instead of factory
    /// rainbow. Mirrors L-Connect's MasterDevice.SaveConfig — broadcast MAC
    /// ff:…:ff / rx_type 0xFF, repeated `tries`× with 200 ms gaps.
    ///
    /// Callers are responsible for flash-friendly pacing (L-Connect: once
    /// after 1 h, then every 3 h, only when effects have settled) and for
    /// not persisting while a bank is drifted.
    pub fn save_config(&self, tries: u8) -> Result<()> {
        let master_mac = *self.master_mac.lock();
        let master_ch = *self.master_channel.lock();

        let mut rf_data = vec![0u8; RF_DATA_SIZE];
        rf_data[0] = RF_SELECT;
        rf_data[1] = RF_SAVE_CFG;
        rf_data[2..8].copy_from_slice(&[0xFF; 6]);
        rf_data[8..14].copy_from_slice(&master_mac);
        rf_data[14] = 0xFF;

        let tries = tries.max(1);
        self.tx_recover(|handle| {
            for attempt in 0..tries {
                for chunk_idx in 0..RF_CHUNKS as u8 {
                    let mut packet = vec![0u8; 64];
                    packet[0] = USB_CMD_SEND_RF;
                    packet[1] = chunk_idx;
                    packet[2] = master_ch;
                    packet[3] = 0xFF;

                    let start = chunk_idx as usize * RF_CHUNK_SIZE;
                    let end = start + RF_CHUNK_SIZE;
                    packet[4..64].copy_from_slice(&rf_data[start..end]);

                    handle
                        .write(&packet, USB_TIMEOUT)
                        .context("sending SaveCfg packet")?;
                    thread::sleep(Duration::from_micros(100));
                }
                if attempt + 1 < tries {
                    thread::sleep(Duration::from_millis(200));
                }
            }
            Ok(())
        })?;

        info!("Broadcast SaveCfg to wireless banks ({tries}x)");
        Ok(())
    }

    /// When the most recent RGB upload (to any bank) was sent, if ever.
    pub fn last_rgb_send(&self) -> Option<Instant> {
        self.desired_effects
            .lock()
            .values()
            .map(|(_, sent_at)| *sent_at)
            .max()
    }

    pub fn send_rx_sequence(&self) -> Result<()> {
        if let Some(rx) = &self.rx {
            for (cmd, capture) in [
                (&*CMD_RX_QUERY_34, true),
                (&*CMD_RX_QUERY_37, true),
                (&*CMD_RX_LCD_MODE, false),
            ] {
                with_transport_recovery(rx, &RX_IDS, "RX", |handle| {
                    handle
                        .write(cmd, USB_TIMEOUT)
                        .context("sending RX command")?;
                    Ok(())
                })?;
                thread::sleep(Duration::from_millis(2));
                if capture {
                    let mut buf = [0u8; 64];
                    let handle = rx.lock();
                    if let Ok(len) = handle.read(&mut buf, USB_TIMEOUT) {
                        debug!("RX resp: {:02x?}", &buf[..len.min(8)]);
                    }
                }
            }
        }
        Ok(())
    }

    pub fn soft_reset(&mut self) -> bool {
        if self.tx.is_none() {
            if let Ok(mut transport) = open_any(&TX_IDS) {
                if transport.detach_and_configure("TX").is_ok() {
                    self.tx = Some(Arc::new(Mutex::new(transport)));
                }
            }
        }
        self.reset_dongle()
    }

    /// Send the GetMac wake probe (0x11 0x08) that the connect path fires
    /// before discovery. Empirically this is what re-admits banks stuck in
    /// channel-0 limbo: a daemon restart (which sends it) healed a limbo
    /// bank in ~15 s twice on 2026-07-05/06, while USB_ResetAnother fired
    /// and changed nothing. Non-disruptive to healthy banks — every daemon
    /// boot sends it while they keep running.
    pub fn wake_network(&self) -> bool {
        let Some(tx) = &self.tx else {
            return false;
        };
        let handle = tx.lock();
        handle.read_flush();
        if handle.write(&CMD_GET_MAC_WAKE, USB_TIMEOUT).is_err() {
            return false;
        }
        let mut resp = [0u8; 64];
        let _ = handle.read(&mut resp, Duration::from_millis(500));
        true
    }

    /// Reset the connected TX dongle (USB_ResetAnother) and re-enter video
    /// mode. Escalation if [`Self::wake_network`] doesn't re-admit a limbo
    /// bank. Unlike [`Self::soft_reset`] this does not attempt to (re)open
    /// the transport, so it works through a shared reference (the
    /// fan-controller heartbeat holds an `Arc`).
    pub fn reset_dongle(&self) -> bool {
        if let Some(tx) = &self.tx {
            {
                // Send the REAL dongle reset (USB_ResetAnother). This code
                // used to send CMD_GET_MAC_WAKE here believing it was a
                // reset — a read-only MAC query, so recovery was a placebo.
                let handle = tx.lock();
                let mut reset_cmd = vec![0u8; 64];
                reset_cmd[0] = USB_CMD_RESET_ANOTHER;
                if handle.write(&reset_cmd, USB_TIMEOUT).is_err() {
                    return false;
                }
                let mut resp = [0u8; 64];
                let _ = handle.read(&mut resp, Duration::from_millis(2000));
            }
            self.video_mode_active.store(false, Ordering::Release);
            thread::sleep(Duration::from_millis(50));
            return self.ensure_video_mode().is_ok();
        }

        false
    }

    pub fn is_connected(&self) -> bool {
        self.tx.is_some() && self.tx_failures.load(Ordering::Relaxed) < TX_FAILURE_THRESHOLD
    }

    /// MACs of wireless devices whose currently-running effect_index drifted
    /// away from what we last sent. Indicates the device firmware reset its
    /// lighting state (e.g. idle watchdog) and we should re-send that bank's
    /// RGB state.
    ///
    /// Two classes of false positive are filtered out:
    /// - records with fan_count=0 (under heavy RF load the dongle returns
    ///   partially-zeroed device records — see set_fan_speeds_by_mac — whose
    ///   zeroed effect_index would read as spurious drift);
    /// - banks sent to within [`RGB_ECHO_GRACE`] (the firmware's echo lags
    ///   the send, so a fresh send always looks drifted for a while).
    pub fn drifted_macs(&self) -> Vec<[u8; 6]> {
        let desired = self.desired_effects.lock();
        if desired.is_empty() {
            return Vec::new();
        }
        let now = Instant::now();
        let devices = self.discovered_devices.lock();
        devices
            .iter()
            .filter(|d| {
                d.fan_count != 0
                    && desired.get(&d.mac).is_some_and(|(want, sent_at)| {
                        let drifted = d.effect_index != *want
                            && now.duration_since(*sent_at) >= RGB_ECHO_GRACE;
                        if drifted {
                            debug!(
                                "drift {}: want={:02x?} echoed={:02x?} cmd_seq={} sent {:.0?} ago",
                                d.mac_str(),
                                want,
                                d.effect_index,
                                d.cmd_seq,
                                now.duration_since(*sent_at),
                            );
                        }
                        drifted
                    })
            })
            .map(|d| d.mac)
            .collect()
    }

    /// Bound banks stuck in channel-0 limbo for at least `min_age`.
    /// Such a bank hears nothing we send (unicast or broadcast) and sits in
    /// firmware fail-safe; the only software remedy is forcing the master to
    /// re-form its network (see the fan controller's limbo rescue).
    pub fn limbo_macs(&self, min_age: Duration) -> Vec<[u8; 6]> {
        let now = Instant::now();
        self.limbo_since
            .lock()
            .iter()
            .filter(|(_, since)| now.duration_since(**since) >= min_age)
            .map(|(mac, _)| *mac)
            .collect()
    }

    pub(super) fn tx_recover<F, R>(&self, op: F) -> Result<R>
    where
        F: FnMut(&UsbTransport) -> Result<R>,
    {
        let tx = self.tx.as_ref().context("TX device not connected")?;
        let result = with_transport_recovery(tx, &TX_IDS, "TX", op);
        match &result {
            Ok(_) => self.tx_failures.store(0, Ordering::Relaxed),
            Err(_) => {
                let n = self.tx_failures.fetch_add(1, Ordering::Relaxed) + 1;
                if n == TX_FAILURE_THRESHOLD {
                    warn!("Wireless TX: {n} consecutive failures, marking disconnected");
                }
            }
        }
        result
    }

    pub fn has_discovered_devices(&self) -> bool {
        !self.discovered_devices.lock().is_empty()
    }

    pub fn discovered_device_count(&self) -> usize {
        self.discovered_devices.lock().len()
    }

    /// Snapshot of devices bound to this PC's dongle.
    pub fn devices(&self) -> Vec<DiscoveredDevice> {
        let local_mac = *self.master_mac.lock();
        self.discovered_devices
            .lock()
            .iter()
            .filter(|d| d.master_mac == local_mac)
            .cloned()
            .collect()
    }

    /// Snapshot of devices NOT bound to this dongle.
    pub fn unbound_devices(&self) -> Vec<DiscoveredDevice> {
        let local_mac = *self.master_mac.lock();
        self.discovered_devices
            .lock()
            .iter()
            .filter(|d| d.master_mac != local_mac && d.device_type != 255)
            .cloned()
            .collect()
    }

    pub fn device_by_mac(&self, mac: &[u8; 6]) -> Option<DiscoveredDevice> {
        self.discovered_devices
            .lock()
            .iter()
            .find(|d| &d.mac == mac)
            .cloned()
    }

    /// Current motherboard PWM duty cycle (0-255), or `None` if unavailable.
    /// Extracted from RX GetDev response bytes [2:3] during polling.
    pub fn motherboard_pwm(&self) -> Option<u8> {
        match self.mobo_pwm.load(Ordering::Relaxed) {
            0xFFFF => None,
            v => Some(v as u8),
        }
    }

    /// Send a 240-byte RF packet as 4× 64-byte USB chunks.
    pub(super) fn send_rf_packet(
        &self,
        handle: &UsbTransport,
        device: &DiscoveredDevice,
        rf_data: &[u8],
    ) -> Result<()> {
        for chunk_idx in 0..RF_CHUNKS as u8 {
            let mut packet = vec![0u8; 64];
            packet[0] = USB_CMD_SEND_RF;
            packet[1] = chunk_idx;
            packet[2] = device.channel;
            packet[3] = device.rx_type;

            let start = chunk_idx as usize * RF_CHUNK_SIZE;
            let end = start + RF_CHUNK_SIZE;
            packet[4..64].copy_from_slice(&rf_data[start..end]);

            handle
                .write(&packet, USB_TIMEOUT)
                .context("sending RGB RF packet")?;
            // 100µs — empirical pacing to avoid the dongle's USB endpoint
            // buffer overflowing on back-to-back chunks. Reducing from the
            // historical 1ms gives ~10x headroom on RF send rate, which
            // matters for keeping master-clock heartbeats, PWM updates, and
            // device-state polling from being starved when OpenRGB clients
            // flood UPDATE_LEDS at 60+ fps per device.
            thread::sleep(Duration::from_micros(100));
        }
        Ok(())
    }

    pub(super) fn next_seq_index(&self, device: &DiscoveredDevice) -> u8 {
        let devices = self.discovered_devices.lock();
        let master_mac = *self.master_mac.lock();
        devices
            .iter()
            .filter(|d| d.master_mac == master_mac && d.device_type != 0xFF)
            .position(|d| d.mac == device.mac)
            .map(|i| (i + 1) as u8)
            .unwrap_or(1)
    }

    pub(super) fn device_by_mac_snapshot(&self, mac: &[u8; 6]) -> Result<DiscoveredDevice> {
        let devices = self.discovered_devices.lock();
        devices
            .iter()
            .find(|d| &d.mac == mac)
            .cloned()
            .with_context(|| {
                format!(
                    "Device MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} not found in discovery",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                )
            })
    }

    pub fn stop(&mut self) {
        if self.poll_thread.is_some() {
            self.poll_stop.store(true, Ordering::SeqCst);
            if let Some(handle) = self.poll_thread.take() {
                let _ = handle.join();
            }
        }
        self.tx.take();
        self.rx.take();
    }
}

impl Default for WirelessController {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for WirelessController {
    fn drop(&mut self) {
        self.stop();
    }
}
