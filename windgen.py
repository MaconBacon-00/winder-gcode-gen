#!/usr/bin/env python3
"""
windgen - G-code generator for the NU Rocketry filament winder.

Targets the interface in winder-firmware/docs/WINDER_GCODE_INTERFACE.md:
  X = carriage (mm, 0..1250), Y = mandrel (deg), Z = delivery head (deg).
  F = units/min over the combined Euclidean length, with degrees counted as mm.

Usage:
  python windgen.py example_tube.json -o HELIX45.gcode
  python windgen.py example_tube.json --dry-run      # summary only

Stdlib only. Cylindrical mandrels only (no domes).
"""
import argparse
import json
import math
import sys

# ---------------------------------------------------------------- machine ---
# From Marlin/Configuration.h. Update if the firmware changes.
MACHINE = {
    "steps_per_unit": {"X": 80.0, "Y": 8.889, "Z": 8.889},
    "max_feedrate": {"X": 200.0, "Y": 720.0, "Z": 720.0},   # units/s
    "max_accel": {"X": 500.0, "Y": 1000.0, "Z": 1000.0},    # units/s^2
    "x_min": 0.0,
    "x_max": 1250.0,
    "max_line": 96,
    "min_steps_per_segment": 6,
    "min_segment_time_s": 0.020,
    "max_abs_rotation_deg": 1_000_000.0,
}

# Generator defaults. Every one can be overridden in the JSON config.
DEFAULTS = {
    "name": "WIND",
    "mandrel_diameter": None,      # mm, required
    "band_width": None,            # mm, required
    "x_start": None,               # mm, required (tube start along X)
    "x_end": None,                 # mm, required (tube end along X)
    "layer_thickness": 0.0,        # mm added to radius per layer (0 = ignore growth)
    "carriage_speed": 50.0,        # mm/s, max carriage speed during passes
    "mandrel_speed": 360.0,        # deg/s, max mandrel speed (any move)
    "head_speed": 360.0,           # deg/s, max head speed
    "travel_speed": 100.0,         # mm/s, carriage-only positioning moves
    "accel": 1000,                 # M204 P/T value
    "min_dwell": 180.0,            # deg, minimum mandrel rotation (end lock) at each turnaround
    "lead_in_mm": 0.0,             # mm of carriage travel ramping in from the end (0 = off)
    "lead_out_deg": 0.0,           # deg of mandrel rotation ramping out to the end (0 = off)
    "lead_segments": 4,            # G1 segments per lead-in / lead-out ramp
    "hoop_dwell": 360.0,           # deg, rotation at the end of a hoop pass
    "head_enabled": False,         # rotate delivery head (Z) to follow wind angle
    "head_gain": 1.0,              # Z degrees per degree of wind angle
    "pause_to_align": False,       # M0 before G92 so operator can align mandrel/head
    "pause_at_end": True,          # M0 at end ("tie off fibre")
    "layers": [],
}

LAYER_DEFAULTS = {
    "hoop": {"type": "hoop", "passes": 1},
    "helical": {"type": "helical", "angle": None, "skip": 1, "pattern": None,
                "min_dwell": None, "lead_in_mm": None, "lead_out_deg": None},
}
# Per-layer keys that fall back to the global setting when left out.
LAYER_INHERIT = ("min_dwell", "lead_in_mm", "lead_out_deg")


class WindError(Exception):
    pass


# ---------------------------------------------------------------- planner ---
class Program:
    """Accumulates G-code lines while tracking absolute machine position."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lines = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.time_s = 0.0          # no-accel estimate (move time at commanded F)
        self.time_accel_s = 0.0    # conservative estimate: stop at every move end
        self.max_abs_y = 0.0
        self.max_abs_z = 0.0
        self.clamped = 0

    def emit(self, text=""):
        self.lines.append(text)

    def comment(self, text):
        self.emit("; " + text)

    def move(self, x=None, y=None, z=None, rates=None, label=None):
        """Coordinated G1 to absolute (x, y, z).

        rates: per-axis speed caps (units/s) chosen by the pattern. The move
        time is the smallest t that keeps every axis under its cap and under
        the machine limit; F is then 60*L/t.
        """
        nx = self.x if x is None else x
        ny = self.y if y is None else y
        nz = self.z if z is None else z
        # round to what we will actually write
        nx, ny, nz = round(nx, 3), round(ny, 3), round(nz, 3)
        d = {"X": nx - self.x, "Y": ny - self.y, "Z": nz - self.z}

        if not (MACHINE["x_min"] - 1e-9 <= nx <= MACHINE["x_max"] + 1e-9):
            raise WindError(f"X={nx} outside 0..{MACHINE['x_max']} mm")

        steps = {a: abs(d[a]) * MACHINE["steps_per_unit"][a] for a in d}
        if all(s < MACHINE["min_steps_per_segment"] for s in steps.values()):
            if any(s > 0 for s in steps.values()):
                # Too small: firmware would merge it. Skip; the next move absorbs it.
                pass
            return

        rates = rates or {}
        t = 0.0
        for a in d:
            if d[a] == 0:
                continue
            cap = min(rates.get(a, MACHINE["max_feedrate"][a]), MACHINE["max_feedrate"][a])
            t = max(t, abs(d[a]) / cap)
        L = math.sqrt(d["X"] ** 2 + d["Y"] ** 2 + d["Z"] ** 2)
        t = max(t, MACHINE["min_segment_time_s"])
        F = 60.0 * L / t
        F = math.floor(F * 10) / 10  # round down so no axis exceeds its cap
        if F <= 0:
            raise WindError("zero feedrate move")
        t = L / (F / 60.0)

        words = ["G1"]
        if d["X"]:
            words.append(f"X{nx:.3f}")
        if d["Y"]:
            words.append(f"Y{ny:.3f}")
        if d["Z"]:
            words.append(f"Z{nz:.3f}")
        words.append(f"F{F:.1f}")
        line = " ".join(words)
        if label:
            line += " ; " + label
        if len(line) > MACHINE["max_line"]:
            line = " ".join(words)
        self.emit(line)

        self.time_s += t
        self.time_accel_s += _trapezoid_time(L, F / 60.0, self._move_accel(d, L))
        self.x, self.y, self.z = nx, ny, nz
        self.max_abs_y = max(self.max_abs_y, abs(ny))
        self.max_abs_z = max(self.max_abs_z, abs(nz))

    def _move_accel(self, d, L):
        """Combined acceleration Marlin will use: M204 T, cut so no axis exceeds its max."""
        a = float(self.cfg["accel"])
        for ax, dv in d.items():
            if dv:
                a = min(a, MACHINE["max_accel"][ax] * L / abs(dv))
        return a


def _trapezoid_time(L, v, a):
    """Time for a rest-to-rest move of length L, cruise v, accel a."""
    if L <= 0:
        return 0.0
    d_acc = v * v / a
    if d_acc >= L:
        return 2.0 * math.sqrt(L / a)
    return 2.0 * v / a + (L - d_acc) / v


# --------------------------------------------------------------- patterns ---
def deg_per_mm(angle_deg, D):
    """Mandrel degrees per mm of carriage travel for wind angle from the axis."""
    return 360.0 * math.tan(math.radians(angle_deg)) / (math.pi * D)


def head_angle(cfg, signed_wind_angle):
    if not cfg["head_enabled"]:
        return None
    return cfg["head_gain"] * signed_wind_angle


def pass_rates(cfg, k):
    """Per-axis caps for a pass with k deg/mm, keeping X and Y in ratio."""
    vx = min(cfg["carriage_speed"], cfg["mandrel_speed"] / k if k > 0 else 1e9)
    return {"X": vx, "Y": vx * k, "Z": cfg["head_speed"]}


def dwell_rates(cfg):
    return {"Y": cfg["mandrel_speed"], "Z": cfg["head_speed"]}


def other_end(cfg, x):
    xs, xe = cfg["x_start"], cfg["x_end"]
    return xe if abs(x - xs) <= abs(x - xe) else xs


def wind_hoop(p, cfg, layer, D, info):
    bw = cfg["band_width"]
    angle = math.degrees(math.atan(math.pi * D / bw))
    k = 360.0 / bw  # one band width per revolution
    for i in range(layer["passes"]):
        target = other_end(cfg, p.x)
        direction = 1 if target > p.x else -1
        z = head_angle(cfg, direction * angle)
        dx = abs(target - p.x)
        # turnaround / head flip before the pass
        if i > 0 or z is not None:
            p.move(y=p.y + (cfg["hoop_dwell"] if i > 0 else 0), z=z, rates=dwell_rates(cfg),
                   label="hoop turnaround" if i > 0 else "head to hoop angle")
        p.move(x=target, y=p.y + dx * k, rates=pass_rates(cfg, k),
               label=f"hoop pass {i + 1}/{layer['passes']}")
    info.append(f"hoop: angle {angle:.2f} deg, pitch {bw:.3f} mm/rev, {layer['passes']} pass(es)")


def ramp_angles(alpha, nseg):
    """Segment mid-angles stepping from alpha toward 90 deg (never reaching it)."""
    return [alpha + (90.0 - alpha) * (i + 0.5) / nseg for i in range(nseg)]


def helical_pass_plan(cfg, layer, alpha, D):
    """Split one end-to-end pass into (dx_mm, dy_deg, tag) segments.

    lead-in : first lead_in_mm of carriage travel, angle ramps from ~90 down to alpha
    body    : constant alpha
    lead-out: last lead_out_deg of mandrel rotation, angle ramps from alpha up to ~90
    The ramps soften the carriage stop/reverse at each end so the fibre wraps
    over the end instead of being yanked round a sharp corner.
    """
    length = abs(cfg["x_end"] - cfg["x_start"])
    lead_in, lead_out, nseg = layer["lead_in_mm"], layer["lead_out_deg"], cfg["lead_segments"]
    if lead_in < 0 or lead_out < 0:
        raise WindError("lead_in_mm and lead_out_deg must be >= 0")
    segs = []
    if lead_in > 0:
        for ang in reversed(ramp_angles(alpha, nseg)):
            dx = lead_in / nseg
            segs.append((dx, dx * deg_per_mm(ang, D), "lead-in"))
    out = []
    if lead_out > 0:
        for ang in ramp_angles(alpha, nseg):
            dy = lead_out / nseg
            out.append((dy / deg_per_mm(ang, D), dy, "lead-out"))
    body = length - sum(s[0] for s in segs) - sum(s[0] for s in out)
    if body <= 0.1 * length:
        raise WindError(f"helical {alpha:g} deg: lead-in/lead-out use {length - body:.1f} mm of "
                        f"the {length:.1f} mm tube; shrink lead_in_mm or lead_out_deg")
    segs.append((body, body * deg_per_mm(alpha, D), "body"))
    return segs + out


def run_pass(p, cfg, segs, target, label):
    direction = 1 if target > p.x else -1
    for i, (dx, dy, tag) in enumerate(segs):
        x = target if i == len(segs) - 1 else p.x + direction * dx  # land exactly on the end
        p.move(x=x, y=p.y + dy, rates=pass_rates(cfg, dy / dx),
               label=label if tag == "body" else tag)


def wind_helical(p, cfg, layer, D, info):
    alpha = layer["angle"]
    if alpha is None or not (1.0 <= alpha <= 89.0):
        raise WindError(f"helical angle must be 1..89 deg, got {alpha}")
    bw = cfg["band_width"]
    circ = math.pi * D
    n = max(1, math.ceil(circ * math.cos(math.radians(alpha)) / bw))

    # Pattern: `pattern` evenly spaced start positions, each visited m times with
    # a one-band shift between visits (Cyclone's patternNumber). Default: pattern = n,
    # i.e. every circuit gets its own start position.
    pat = layer["pattern"] or n
    if pat < 1:
        raise WindError("helical pattern must be >= 1")
    n = math.ceil(n / pat) * pat
    m = n // pat
    skip = layer["skip"]
    if math.gcd(skip, pat) != 1:
        raise WindError(f"helical skip {skip} shares a factor with pattern {pat}; "
                        f"bands would repeat. Pick a skip coprime with {pat}.")

    def slot(c):  # start position of circuit c, in units of 360/n
        return ((c % pat) * skip % pat) * m + c // pat

    bw_eff = circ * math.cos(math.radians(alpha)) / n
    segs = helical_pass_plan(cfg, layer, alpha, D)
    pass_deg = sum(s[1] for s in segs)
    min_dwell = layer["min_dwell"]

    # Each circuit = pass, lock, pass back, lock. The two locks are split evenly
    # so the next circuit starts on its slot (mod 360). The second lock is
    # recomputed from the actual position so rounding never accumulates.
    start = p.x
    far = other_end(cfg, start)
    out_dir = 1 if far > start else -1
    y0 = p.y
    dwells = []
    for c in range(n):
        nxt = slot(c + 1) if c + 1 < n else n  # n == slot 0 one turn on
        delta = (nxt - slot(c)) * 360.0 / n
        d1 = ((delta - 2 * pass_deg) / 2.0) % 180.0
        while d1 < min_dwell:
            d1 += 180.0

        p.emit(f"M117 {cfg['_layer_tag']} C{c + 1}/{n}")
        z_out = head_angle(cfg, out_dir * alpha)
        z_back = head_angle(cfg, -out_dir * alpha)
        if c == 0 and z_out is not None:
            p.move(z=z_out, rates=dwell_rates(cfg), label="head to wind angle")
        run_pass(p, cfg, segs, far, f"circuit {c + 1} out")
        p.move(y=p.y + d1, z=z_back, rates=dwell_rates(cfg), label="lock")
        run_pass(p, cfg, segs, start, f"circuit {c + 1} back")
        target = y0 + nxt * 360.0 / n
        d2 = (target - p.y) % 360.0
        while d2 < min_dwell - 0.01:
            d2 += 360.0
        p.move(y=p.y + d2, z=z_out, rates=dwell_rates(cfg), label="lock")
        dwells += [d1, d2]

    lead = ""
    if layer["lead_in_mm"] or layer["lead_out_deg"]:
        lead = f", lead-in {layer['lead_in_mm']:g} mm, lead-out {layer['lead_out_deg']:g} deg"
    info.append(
        f"helical {alpha:g} deg: {n} circuits (pattern {pat} x {m}), skip {skip}, "
        f"effective band {bw_eff:.3f} mm (overlap {100 * (1 - bw_eff / bw):.1f}%), "
        f"pass {pass_deg:.1f} deg, lock {min(dwells):.1f}-{max(dwells):.1f} deg{lead}"
    )


PATTERNS = {"hoop": wind_hoop, "helical": wind_helical}


# --------------------------------------------------------------- top level ---
def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return parse_config(json.load(f))


def parse_config(user):
    """Merge a config dict (already parsed from JSON) with defaults and check it."""
    if not isinstance(user, dict):
        raise WindError("config must be a JSON object")
    cfg = dict(DEFAULTS)
    unknown = set(user) - set(DEFAULTS)
    if unknown:
        raise WindError(f"unknown config keys: {', '.join(sorted(unknown))}")
    cfg.update(user)
    for key in ("mandrel_diameter", "band_width", "x_start", "x_end"):
        if cfg[key] is None:
            raise WindError(f"config needs '{key}'")
    if not cfg["layers"]:
        raise WindError("config needs at least one layer")
    xs, xe = cfg["x_start"], cfg["x_end"]
    for v in (xs, xe):
        if not (MACHINE["x_min"] <= v <= MACHINE["x_max"]):
            raise WindError(f"x_start/x_end must be within 0..{MACHINE['x_max']} mm")
    if abs(xe - xs) < 1.0:
        raise WindError("x_start and x_end are too close")
    for key, axis in (("carriage_speed", "X"), ("travel_speed", "X"),
                      ("mandrel_speed", "Y"), ("head_speed", "Z")):
        if not (0 < cfg[key] <= MACHINE["max_feedrate"][axis]):
            raise WindError(f"{key} must be >0 and <= {MACHINE['max_feedrate'][axis]}")
    layers = []
    for i, layer in enumerate(cfg["layers"]):
        t = layer.get("type")
        if t not in LAYER_DEFAULTS:
            raise WindError(f"layer {i + 1}: type must be one of {list(LAYER_DEFAULTS)}")
        merged = dict(LAYER_DEFAULTS[t])
        extra = set(layer) - set(merged) - {"thickness"}
        if extra:
            raise WindError(f"layer {i + 1}: unknown keys {sorted(extra)}")
        merged.update(layer)
        merged.setdefault("thickness", cfg["layer_thickness"])
        for key in LAYER_INHERIT:
            if key in merged and merged[key] is None:
                merged[key] = cfg[key]
        layers.append(merged)
    cfg["layers"] = layers
    return cfg


def generate(cfg, source_name="config"):
    p = Program(cfg)
    n_layers = len(cfg["layers"])
    p.comment("Generated by windgen.py v1.1")
    p.comment(f"Config: {source_name}")
    p.comment(f"Mandrel OD: {cfg['mandrel_diameter']} mm, band width: {cfg['band_width']} mm, "
              f"layers: {n_layers}")
    p.comment(f"Tube X range: {cfg['x_start']} - {cfg['x_end']} mm")
    for i, L in enumerate(cfg["layers"]):
        desc = L["type"] + (f" {L['angle']}deg skip {L['skip']}" if L["type"] == "helical"
                            else f" x{L['passes']}")
        p.comment(f"  layer {i + 1}: {desc}")
    p.emit("G90 ; absolute")
    p.emit("M220 S100 ; no feedrate override")
    p.emit(f"M204 P{cfg['accel']} T{cfg['accel']}")
    p.emit("M17 ; motors on")
    p.emit("G28 X ; home carriage")
    if cfg["pause_to_align"]:
        p.emit("M117 Align mandrel+head")
        p.emit("M0 ; align, then click")
    p.emit("G92 Y0 Z0 ; mandrel/head zero here")
    p.x, p.y, p.z = 0.0, 0.0, 0.0
    p.move(x=cfg["x_start"], rates={"X": cfg["travel_speed"]}, label="to tube start")
    p.emit("M117 Attach fibre")
    p.emit("M0 ; attach fibre, then click")

    info = []
    D = cfg["mandrel_diameter"]
    for i, layer in enumerate(cfg["layers"]):
        tag = f"L{i + 1}/{n_layers}"
        cfg["_layer_tag"] = tag
        p.emit("")
        p.comment(f"===== Layer {i + 1}/{n_layers}: {layer['type']} (D = {D:.3f} mm) =====")
        p.emit(f"M117 {tag} {layer['type']}")
        before = len(info)
        PATTERNS[layer["type"]](p, cfg, layer, D, info)
        info[before] = f"layer {i + 1} (D {D:.2f}): " + info[before]
        D += 2.0 * layer["thickness"]

    p.emit("")
    p.emit("M400 ; wait for motion")
    p.emit("M117 Wind complete")
    if cfg["pause_at_end"]:
        p.emit("M0 ; tie off fibre, then click")
    p.comment("motors left on deliberately to hold tension")

    if p.max_abs_y >= MACHINE["max_abs_rotation_deg"] or p.max_abs_z >= MACHINE["max_abs_rotation_deg"]:
        raise WindError(f"total rotation {max(p.max_abs_y, p.max_abs_z):.0f} deg exceeds "
                        f"~{MACHINE['max_abs_rotation_deg']:.0f} deg float precision limit; "
                        "split the wind into several files")
    return p, info


def validate(lines):
    """Check the output against the firmware rules. Returns a list of problems."""
    problems = []
    banned = ("M104", "M140", "M109", "M190", "M500", "M106", "M107", "G2 ", "G3 ")
    for n, raw in enumerate(lines, 1):
        if len(raw) > MACHINE["max_line"]:
            problems.append(f"line {n}: longer than {MACHINE['max_line']} chars")
        if not raw.isascii():
            problems.append(f"line {n}: non-ASCII")
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        if any(w.startswith("E") for w in code.split()[1:]):
            problems.append(f"line {n}: E word")
        if any((code + " ").startswith(b) for b in banned):
            problems.append(f"line {n}: disallowed command {code.split()[0]}")
    return problems


def fmt_time(s):
    h, rem = divmod(int(round(s)), 3600)
    m, sec = divmod(rem, 60)
    return f"{h}h {m:02d}m {sec:02d}s"


def summary(prog, info):
    """Human-readable summary lines, shared by the CLI and the web page."""
    moves = sum(1 for ln in prog.lines if ln.startswith("G1"))
    return list(info) + [
        f"{moves} moves, {len(prog.lines)} lines",
        f"est. time {fmt_time(prog.time_s)} at commanded speed, "
        f"{fmt_time(prog.time_accel_s)} worst case with acceleration",
        f"final Y {prog.y:.1f} deg ({prog.y / 360:.1f} rev), max |Z| {prog.max_abs_z:.1f} deg",
    ]


def run_json(text, source_name="config.json"):
    """Web entry point: JSON text in, plain dict out (never raises)."""
    try:
        cfg = parse_config(json.loads(text))
        prog, info = generate(cfg, source_name=source_name)
    except json.JSONDecodeError as e:
        return {"ok": False, "error": f"JSON error: {e}"}
    except WindError as e:
        return {"ok": False, "error": str(e)}
    problems = validate(prog.lines)
    return {
        "ok": not problems,
        "error": "; ".join("validation: " + pr for pr in problems),
        "name": cfg["name"],
        "summary": summary(prog, info),
        "gcode": "\n".join(prog.lines) + "\n",
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate winder G-code from a JSON config.")
    ap.add_argument("config", help="JSON wind config")
    ap.add_argument("-o", "--output", help="output .gcode (default: <name>.gcode)")
    ap.add_argument("--dry-run", action="store_true", help="print summary, write nothing")
    args = ap.parse_args(argv)

    try:
        cfg = load_config(args.config)
        prog, info = generate(cfg, source_name=args.config.replace("\\", "/").split("/")[-1])
    except (WindError, OSError, json.JSONDecodeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    problems = validate(prog.lines)
    for line in summary(prog, info):
        print(line)
    if problems:
        for pr in problems:
            print("validation: " + pr, file=sys.stderr)
        return 2

    if args.dry_run:
        return 0
    out = args.output or f"{cfg['name']}.gcode"
    with open(out, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(prog.lines) + "\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
