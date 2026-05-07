# Wireless RGB pattern renderers — plan

## Goal

Implement each native RGB mode for SL-Infinity wireless fans as a host-rendered
multi-frame animation, uploaded once to the dongle, then played back
autonomously by the firmware. Replaces the current "single static frame"
behavior for all non-Direct modes.

## Context — what already works

- Breathing: rendered as 30-frame sine modulation, uploaded via
  `send_rgb_frames` at 33ms interval. Verified pulsing on hardware.
- Sub-zones: each fan exposes 5 zones to OpenRGB
  (Blades / Left Outer / Left Inner / Right Outer / Right Inner). LED slot
  ranges per fan: `[0..8) [8..18) [18..26) [26..36) [36..44)`.
- Mode metadata fix: `colors_min > 0` modes now advertise 1 default color so
  the GUI doesn't crash.
- v4 protocol confirmed sufficient. v5 doesn't add per-segment effects, alt
  names are for keyboard localization, zone flags are unrelated.

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

## Speed → interval table (per pattern)

Speed is 0–4 (0 = slowest, 4 = fastest):

| Pattern | s=0 | 1 | 2 | 3 | 4 | total_frames |
|---|---|---|---|---|---|---|
| Breathing | 67 | 50 | 33 | 22 | 17 | 30 |
| Flashing | 500 | 250 | 125 | 67 | 33 | 4 |
| Rainbow / Morph / Cycle | 50 | 40 | 33 | 25 | 17 | 60 |
| Chase / ChaseFade | dur/N×slice_len pacing | – | – | – | – | slice_len × 2 |
| Random Flicker | 100 | 67 | 50 | 33 | 17 | 30 |

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

### 7. Per-sub-zone composition

Currently each `set_effect` call per sub-zone re-renders + uploads. With
multiple sub-zones in different modes, each upload only animates ONE
sub-zone (others stay frozen).

Goal: render one composite frame sequence covering all sub-zones'
animations simultaneously, upload once.

Approach: keep a per-bank `EffectMap = HashMap<sub_zone_idx, RgbEffect>`.
On any `set_effect`, update the map, then re-render the composite (60 frame
shared loop, each sub-zone's animation fitted into that window with
period-extension or repetition). Upload once.

Tricky bit: different patterns have different natural cycle lengths
(Breathing 1s, Rainbow 2s, Chase = slice_len). Need to pick a global LCM
period, then each sub-zone repeats its cycle within it. Or accept some
patterns running at non-natural speeds for compositional purposes.

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
