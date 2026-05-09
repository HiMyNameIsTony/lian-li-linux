# Wireless RGB pattern renderers — plan

## Goal

Implement each native RGB mode for SL-Infinity wireless fans as a host-rendered
multi-frame animation, uploaded once to the dongle, then played back
autonomously by the firmware. Replaces the current "single static frame"
behavior for all non-Direct modes.

## Pick up here tomorrow

**Visual quality is good**; the remaining UX issue is upload latency. When
OpenRGB's `UpdateMode` packet arrives, `handle_update_mode` in
`crates/lianli-daemon/src/openrgb_server.rs` loops over every zone of the
device and calls `RgbController::set_effect` per zone. Each call rebuilds and
uploads the whole 480-frame composite — so a 5-sub-zone bank becomes 5 full
uploads back-to-back. User reported "having to wait a few seconds for the
daemon to compile and send over the whole animation" and diagnosed it as
"sending each zone as it compiles a new one, instead of sending [once at the
end]".

**Fix**: add a batched `apply_effects(&mut self, device_id, &[(u8, RgbEffect)])`
method on `RgbController` (parallel to the existing `apply_direct_zones`),
which:
1. For wireless: updates `state.sub_zone_effects` and `state.led_state` for
   every (zone, effect) pair in one pass, then calls
   `render_composite_frames` + `send_rgb_frames` exactly once.
2. For wired: falls back to per-zone `set_effect` (no batching benefit
   since wired protocols address zones directly).

Then change `handle_update_mode` in `openrgb_server.rs` to collect all
`(zone_idx, effect)` pairs into a `Vec` and call `apply_effects` once,
instead of looping `set_effect`.

I started writing this in the rgb_controller `mod.rs` (between `set_effect`
and `set_direct_colors`) and reverted it — start fresh tomorrow.

## Context — what already works

- **Breathing**: per-LED-color-preserving sine modulation, rendered as a
  global composite (480 frames × 25ms = 12s loop @ ~40 fps). Verified
  smooth on hardware across all five speeds.
- **Composite renderer** (formerly section 7 below): `render_composite_frames`
  in `rgb_controller/mod.rs` paints all active sub-zones into one frame
  sequence and uploads once. Sub-zones with no effect contribute their
  static `led_state` slice; non-animated zones (Static / Off) bake into
  `led_state` and don't enter the composite.
- **Per-LED color preservation across mode changes**: `set_effect` for
  animated modes preserves any non-blank `led_state` slice (so "set per-LED
  via Direct, then switch to Breathing" makes each LED breathe its own
  color). Only seeds with the effect's default color when the slice is
  all-black.
- **Sub-zones**: each fan exposes 5 zones to OpenRGB
  (Blades / Left Outer / Left Inner / Right Outer / Right Inner). LED slot
  ranges per fan: `[0..8) [8..18) [18..26) [26..36) [36..44)`.
- **Mode metadata fix**: `colors_min > 0` modes now advertise 1 default
  color so the GUI doesn't crash.
- **v4 protocol** confirmed sufficient. v5 doesn't add per-segment effects;
  alt names are for keyboard localization; zone flags are unrelated.

## Discovered firmware quirks

- **Interval scaling factor 0.625**: the firmware plays animations at
  `interval_ms × 0.625` per frame, regardless of value. Verified across
  three (frame_count, interval_ms) pairs that all produced identical 7.5s
  loops — math: `frames × interval × 0.625`. Cause unknown; one "interval
  unit" is effectively ~0.625 ms instead of 1 ms.
  **Daemon-side fix** (already shipped, see `wireless/rgb.rs`): pre-multiply
  `interval_ms` by 8/5 = 1.6 before encoding. Callers can now pass real
  milliseconds and get the playback rate they expect.
- **Interval floor ~20 ms**: pushing below this seems to cap rather than
  scale further; 25 ms is the chosen safe minimum for smooth animations.
- **Composite frame budget**: 480 frames × 25 ms = 12s loop, ~40 fps. Wide
  enough that speed=0 = 1 cycle / 12s ≈ 5 BPM (meditative); fast enough
  that fades look continuous. LZO compresses the highly-repetitive
  breathing frames very efficiently — even hundreds of frames compress to
  a few KB.
- **Speed → cycles mapping for Breathing** (must integer-divide 480):
  - speed 0 → 1 cycle / window  ≈ 5 BPM
  - speed 1 → 2 cycles          ≈ 10 BPM
  - speed 2 → 4 cycles          ≈ 20 BPM
  - speed 3 → 6 cycles          ≈ 30 BPM
  - speed 4 → 12 cycles         ≈ 60 BPM

## Approach — implement one pattern at a time

For each pattern, the cycle is:

1. Implement renderer using the unified signature
2. Update `set_effect` dispatch to use it
3. Update `build_modes` flags / colors metadata for that mode
4. Push to the experimental branch (`fix/wireless-multiframe-breathing-test`,
   rename when graduating from "test")
5. Rebuild, set the mode via OpenRGB GUI/CLI
6. Verify visually — capture surprises (firmware quirks, wrong intervals,
   weird LZO behavior, RPM impact)
7. Update this doc with anything learned, then move to the next pattern

## Renderer interface

Single signature all patterns implement:

```rust
fn render_frames(
    effect: &RgbEffect,            // mode, colors, speed, brightness, direction
    base_state: &[[u8; 3]],        // current full-bank state (for sub-zone preservation)
    slice_start: usize,            // where in led_state this sub-zone begins
    slice_len: usize,              // length of the slice
) -> (Vec<Vec<[u8; 3]>>, u16 /* interval_ms */)
```

Each renderer mutates only `[slice_start..slice_start + slice_len)`; other LEDs
in each frame are copied from `base_state`.

## Speed → cycles-per-window table (per pattern)

With the composite renderer settled at a fixed 480 × 25ms = 12s window, all
patterns now express speed as **integer cycles per window** (not interval).
Speed is 0–4 (0 = slowest, 4 = fastest); cycles must integer-divide 480 so
the loop wraps cleanly at the frame boundary.

| Pattern | s=0 | 1 | 2 | 3 | 4 |
|---|---|---|---|---|---|
| Breathing | 1 | 2 | 4 | 6 | 12 |
| Flashing | 2 | 4 | 8 | 12 | 24 |
| Rainbow / Morph / Cycle | 1 | 2 | 4 | 6 | 12 |
| Chase / ChaseFade | TBD — natural cycle = slice_len, multiple repeats per window |
| Random Flicker | random — different design |

## v4 mode metadata per pattern

Map of `RgbMode` → `(flags, color_mode, colors_min, colors_max)`:

```rust
match mode {
    Static          => (HAS_BRIGHTNESS | HAS_MODE_SPECIFIC_COLOR,
                        COLOR_MODE_MODE_SPECIFIC, 1, 1),
    Breathing       => (HAS_SPEED | HAS_BRIGHTNESS | HAS_MODE_SPECIFIC_COLOR,
                        COLOR_MODE_MODE_SPECIFIC, 1, 1),
    Flashing        => (HAS_SPEED | HAS_BRIGHTNESS | HAS_MODE_SPECIFIC_COLOR,
                        COLOR_MODE_MODE_SPECIFIC, 1, 1),
    Rainbow         => (HAS_SPEED | HAS_BRIGHTNESS | HAS_DIRECTION_LR,
                        COLOR_MODE_NONE, 0, 0),
    RainbowMorph
    | ColorCycle
    | SpectrumCycle => (HAS_SPEED | HAS_BRIGHTNESS,
                        COLOR_MODE_NONE, 0, 0),
    Chase
    | ChaseFade     => (HAS_SPEED | HAS_BRIGHTNESS | HAS_DIRECTION_LR
                          | HAS_MODE_SPECIFIC_COLOR,
                        COLOR_MODE_MODE_SPECIFIC, 1, 1),
    RandomFlicker   => (HAS_SPEED | HAS_BRIGHTNESS,
                        COLOR_MODE_NONE, 0, 0),
}
```

Notes: drop `HAS_DIRECTION_UD` everywhere (these are 1-D LED strips per
sub-zone — no up/down concept).

## Implementation order

Each step ends with a hardware-verify pause + this doc update.

### 0. Batched effect upload (queued — see "Pick up here tomorrow")

Add `apply_effects` to `RgbController`, switch `handle_update_mode` to use
it. Cuts upload latency by Nx where N = number of sub-zones receiving the
mode. Test: switching to Breathing should produce ONE upload log line
instead of five.

### 1. Refactor existing Breathing onto renderer-interface signature

Just to establish the dispatch pattern. No behavior change. Test: confirm
breathing still pulses identically.

### 2. Flashing

Easiest new pattern (4 frames: on, off, on, off). Test: pick Flashing
red on a sub-zone, confirm blink rate matches speed slider.
**Learn**: does the firmware honor very-small interval values? L-Connect
uses `interval_ms = 5000` as a default in the existing code path —
firmware may have a minimum. Empirical lower bound TBD.

### 3. Rainbow Morph (and alias Spectrum Cycle / Color Cycle to same renderer)

Uniform color across the slice, hue cycles through 360°. 60 frames at
33ms = 2-second cycle at speed=2.
HSV → RGB conversion: `hsv((t/N) * 360, 1.0, brightness/4)`.
**Learn**: does the firmware preserve color accuracy across LZO compression
on the long animation? Color count budget — 60 frames × 44 LEDs × 3 bytes
= ~7.9KB raw. After LZO probably fits in 4–6 RF data chunks.

### 4. Rainbow (wave)

Same hue cycle but offset per LED position so the rainbow scrolls across
the slice. For LED i at frame f:
`hue = (i / slice_len) * 360 + (f / N) * 360`.
Direction LR reverses sign of position term.
**Learn**: with 5 sub-zones running independent hue waves, the composite
upload per fan will be the most data-dense yet. Worth measuring RF cost.

### 5. Chase

A single lit position sweeps the slice. Frame i lights LED `i % slice_len`
(or `slice_len - 1 - (i % slice_len)` if direction = LR-reverse).
Total frames = slice_len, interval_ms varies with speed.
For multi-fan banks: each sub-zone's chase loops independently; needs
composition.
**Learn**: how does it look when slice_len is small (e.g. Blades = 8)?
May need a tail (fading) to avoid stutter.

### 6. Chase Fade

Chase with an exponential decay tail.
`brightness(led, frame) = max(0, decay^|led - head_pos|)`.
**Learn**: the LZO-compressed frame size gets bigger because more LEDs
have non-zero values — verify upload fits.

### 7. Per-sub-zone composition — DONE

Implemented as `render_composite_frames` + `state.sub_zone_effects`
HashMap. Each `set_effect` updates the map and re-renders the global 480 ×
25ms composite. Each sub-zone's natural cycle is fitted into the 12s
window (cycles per window per the speed table above) so all animations
loop cleanly at the frame boundary.

Cost: every `set_effect` rebuilds + uploads the full composite. Mitigated
by step 0 (batched upload) for the multi-zone-update case.

### 8. Random Flicker

Lowest priority. Random LEDs in the slice flash at random colors.
Total frames could be longer (e.g. 30 frames @ 50ms = 1.5s loop with a
distinct pattern that doesn't look obviously periodic).

## Open questions to resolve as we go

- **Firmware interval_ms minimum**: empirical. L-Connect uses 5000ms as the
  default for one-shot direct sends. Multi-frame animations need much
  smaller intervals. Find the floor where playback still looks smooth.
- **Firmware total_frame maximum**: how many frames can the dongle store and
  loop? L-Connect's longer animations might hint at the limit.
- **LZO efficiency at scale**: a 60-frame animation across 44 LEDs is ~7.9KB
  raw. LZO compression of repetitive patterns (rainbow morph = same hue
  for all 44 LEDs) should be excellent. Test compressed size in the
  daemon's debug log.
- **RF cost of upload**: each upload is N data chunks. usbmon should show
  the burst on each mode change. Expect: small spike on mode change,
  then 0 RF for the duration of playback.
- **Composition vs. per-sub-zone tradeoff**: if composing all 5 sub-zones'
  animations into one frame sequence is expensive (e.g. blows up the LZO
  size), maybe accept that animation periods get rounded to a common
  divisor.
- **Effect dedup**: when OpenRGB's GUI re-fires `UPDATE_MODE` on every
  slider tweak, we re-render and re-upload. Cheap individually but adds
  up. Detect "frames identical to last upload" and skip the actual send.
  Already noted from Breathing testing — fans re-uploaded every ~8s while
  fiddling with sliders.

## What this is NOT

- This plan does not address OpenRGB Effects plugin (audio-reactive,
  ambilight) which is fundamentally per-frame Direct streaming. The
  pre-rendered upload approach has no value for those use cases.
- Wired RGB devices are out of scope here. Their `set_zone_effect` already
  works through `RgbDevice::set_zone_effect` which the wired drivers
  implement natively.
