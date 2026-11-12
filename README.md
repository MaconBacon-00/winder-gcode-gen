# windgen

Makes G-code wind files for the NU Rocketry filament winder. It follows
`winder-firmware/docs/WINDER_GCODE_INTERFACE.md`. Uses only the Python standard library and supports cylindrical mandrels only.

```
python windgen.py example_tube.json -o HELIX45.gcode
python windgen.py example_tube.json --dry-run
```

## Web version

`index.html` runs the same `windgen.py` in the browser with [Pyodide](https://pyodide.org). Nothing needs installing and nothing is uploaded. The page has a config editor, a summary, a download button and an unrolled preview of the fibre path.

- Local: run `python -m http.server` in this folder, then open http://localhost:8000. Opening the file directly will not work.
- Hosted: push the repo to GitHub, then go to Settings → Pages → Deploy from a branch → `main` / root. Changes to `windgen.py` go live on the next push.

## Config (JSON)

Required: `mandrel_diameter`, `band_width`, `x_start`, `x_end` (all in mm), and `layers`.

Layers:
- `{"type": "hoop", "passes": 2}`: one band width of carriage travel per mandrel revolution. Each pass runs to the other end of the tube.
- `{"type": "helical", "angle": 45, "skip": 1}`: one full ±angle coverage. The circuit count is `ceil(pi*D*cos(a)/band_width)`. By default each circuit starts `skip*360/N` degrees further round. `skip` must be coprime with N.
  - `"pattern": P` (optional, like Cyclone's `patternNumber`): uses P evenly spaced start positions. N is rounded up to a multiple of P. The P positions are visited in `skip` order, then the whole set shifts by one band, and this repeats N/P times. `skip` must then be coprime with P. A small P gives a coarse diamond pattern, and P = N (the default) gives the finest.
  - `"min_dwell"`, `"lead_in_mm"` and `"lead_out_deg"` can be set per layer. If left out, the global value is used.
- Any layer can also take `"thickness"` (mm added to the radius after the layer, which feeds into the next layer's D).

Optional settings (defaults are in `DEFAULTS` in `windgen.py`): `carriage_speed`, `mandrel_speed`, `head_speed`, `travel_speed`, `accel`, `min_dwell`, `lead_in_mm`, `lead_out_deg`, `lead_segments`, `hoop_dwell`, `layer_thickness`, `head_enabled`, `head_gain`, `pause_to_align`, `pause_at_end`, `name`.

## Ends of helical passes
- **Lock** (`min_dwell`, like Cyclone's `lockDegrees`): the smallest mandrel rotation with the carriage stopped at each end. The actual lock is between `min_dwell` and `min_dwell + 180`, whatever is needed to land the next circuit on its start position. Each circuit's second lock is calculated from the actual position, so rounding never builds up.
- **Lead-in** (`lead_in_mm`): over the first part of a pass, the wind angle ramps from almost hoop down to the wind angle, in `lead_segments` steps.
- **Lead-out** (`lead_out_deg`): over the last part of a pass, the wind angle ramps from the wind angle up to almost hoop. The length is given in mandrel degrees.
- The ramps ease the carriage's stop and reversal, so the fibre wraps over the end instead of turning a sharp corner. Both are 0 (off) by default. The example uses 30 mm and 90°. With `head_enabled`, the head stays at ±angle during the ramps and flips during the lock.

## How moves are made
- A helical pass is one `G1` for the body, plus `lead_segments` `G1`s for each ramp that is turned on. Hoop passes are a single `G1`. Each lock or turnaround is one mandrel-only move, plus a head flip if `head_enabled` is on.
- `F = 60*L/t`. `t` is the shortest time that keeps every axis under both its configured cap and the firmware maximum. F is rounded down.
- Output is checked against the firmware rules: lines of at most 96 characters, ASCII only, no E words, and none of the banned M or G codes. The run fails if the total rotation reaches about 1,000,000°.

## Check on the machine before trusting a pattern
- Which way +Y turns the mandrel (this sets the handedness).
- The sign convention for the head (Z). `head_enabled` is off by default.
- Whether the fibre holds at the tube ends during the turnaround dwell. Cylinders without domes usually need pin rings.
