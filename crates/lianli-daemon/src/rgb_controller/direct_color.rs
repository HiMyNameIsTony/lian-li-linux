use super::RgbController;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tracing::debug;

/// Minimum time between flush cycles (~30 fps). OpenRGB clients can push
/// 60+ fps, but RF airtime is shared with the 1 Hz PWM/master-clock
/// keepalives that keep bank firmware out of its "host gone" fail-safe
/// (100% fans). Newest-wins coalescing in the buffer means capping here
/// drops intermediate frames, not adds lag.
const FLUSH_INTERVAL: Duration = Duration::from_millis(33);

/// Guaranteed RF-idle gap after every flush cycle, even when the cycle ran
/// longer than FLUSH_INTERVAL (a multi-bank flush can exceed the budget,
/// and a budget-only cap then degrades to zero sleep). Without it,
/// keepalives sit behind queued RGB in the dongle, arrive late, and banks
/// hit fan fail-safe.
const FLUSH_IDLE_FLOOR: Duration = Duration::from_millis(15);

/// Buffers per-device, per-zone direct color updates for async flushing.
///
/// The OpenRGB TCP handler writes latest colors here (fast, no device I/O).
/// A writer thread flushes dirty devices at ~30fps, dropping intermediate frames.
pub struct DirectColorBuffer {
    pending: HashMap<String, HashMap<u8, Vec<[u8; 3]>>>,
}

impl DirectColorBuffer {
    pub fn new() -> Self {
        Self {
            pending: HashMap::new(),
        }
    }

    /// Store colors for a device zone (overwrites any previous pending value).
    pub fn set(&mut self, device_id: String, zone: u8, colors: Vec<[u8; 3]>) {
        self.pending
            .entry(device_id)
            .or_default()
            .insert(zone, colors);
    }

    /// Patch a single LED inside an already-pending zone update.
    /// Returns false when no pending vec covers that LED — the caller must
    /// seed the zone from current device state instead.
    pub fn patch_led(&mut self, device_id: &str, zone: u8, led: usize, color: [u8; 3]) -> bool {
        match self
            .pending
            .get_mut(device_id)
            .and_then(|zones| zones.get_mut(&zone))
        {
            Some(colors) if led < colors.len() => {
                colors[led] = color;
                true
            }
            _ => false,
        }
    }

    /// Take all pending updates, clearing the buffer.
    pub fn take_all(&mut self) -> HashMap<String, HashMap<u8, Vec<[u8; 3]>>> {
        std::mem::take(&mut self.pending)
    }
}

/// Spawns a background thread that flushes buffered direct colors.
///
/// Wired devices are processed first for lowest latency.
/// Wireless devices use single-frame direct sends.
pub fn start_direct_color_writer(
    rgb: Arc<Mutex<RgbController>>,
    buffer: Arc<Mutex<DirectColorBuffer>>,
    stop_flag: Arc<AtomicBool>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        debug!("Direct color writer started");

        // Per-device timestamp of the last flush. A device not flushed
        // within the last second is getting an isolated apply, not a stream
        // frame — no follow-up frame will retry RF loss, so it gets full
        // one-shot effort. Per device because OpenRGB splits a multi-device
        // apply into separate messages across flush cycles; a global
        // timestamp lets siblings downgrade each other to lossy stream
        // effort.
        let mut last_flush: HashMap<String, Instant> = HashMap::new();

        loop {
            if stop_flag.load(Ordering::Relaxed) {
                break;
            }

            let updates = buffer.lock().take_all();

            if !updates.is_empty() {
                let flush_start = Instant::now();
                let mut wired = Vec::new();
                let mut wireless = Vec::new();
                {
                    let rgb = rgb.lock();
                    for (device_id, zones) in updates {
                        if rgb.is_wireless(&device_id) {
                            wireless.push((device_id, zones));
                        } else {
                            wired.push((device_id, zones));
                        }
                    }
                }

                if !wired.is_empty() {
                    let mut rgb = rgb.lock();
                    for (device_id, zones) in wired {
                        for (zone, colors) in zones {
                            if let Err(e) = rgb.set_direct_colors(&device_id, zone, &colors) {
                                debug!("Wired flush error for {device_id} zone {zone}: {e}");
                            }
                        }
                    }
                }

                if !wireless.is_empty() {
                    let mut rgb = rgb.lock();
                    for (device_id, zones) in wireless {
                        let one_shot = last_flush
                            .get(&device_id)
                            .is_none_or(|t| t.elapsed() > Duration::from_secs(1));
                        debug!("flushing {device_id}, one_shot={one_shot}");
                        let zones_vec: Vec<(u8, Vec<[u8; 3]>)> = zones.into_iter().collect();
                        if let Err(e) = rgb.apply_direct_zones(&device_id, &zones_vec, one_shot) {
                            debug!("Wireless flush error for {device_id}: {e}");
                        }
                        last_flush.insert(device_id, Instant::now());
                    }
                }

                // Enforce the ~30 fps cap the buffer was designed around,
                // with a hard idle floor so a slow cycle can't erase the
                // sleep entirely and starve keepalive airtime.
                let remaining = FLUSH_INTERVAL
                    .checked_sub(flush_start.elapsed())
                    .unwrap_or(Duration::ZERO);
                thread::sleep(remaining.max(FLUSH_IDLE_FLOOR));
            } else {
                thread::sleep(Duration::from_millis(5));
            }
        }

        debug!("Direct color writer stopped");
    })
}
