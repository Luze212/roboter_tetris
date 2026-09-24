#!/usr/bin/env python3
"""B23/B21 evaluation: base camera vs. robot, from paired touch points.

Fahrplan Block 2 (`docs/archiv/fahrplan-aufbau.md`). At several places
along the belt a resting block is measured by ``base_cam`` (S1) and then
touched on the centre of its top face with the closed gripper (flange pose).
Each place gives one pair. From the pairs this tool answers:

* **B23** -- do both systems agree? It tests the plain identity, the
  suspected 180° turn about the vertical axis (UR ``base`` vs ``base_link``,
  Nachtrag 8 / F1) and fits a rigid 2D transform (rotation + translation)
  with its residual per point.
* **B21** -- is the base camera's scatter the same inside and outside the old
  measuring region (y = +500 ... +1000 mm in ``world``, K5)?
* **Height** -- block height from the robot, ``z_flange - 298.56 mm``
  (flange height when touching the bare belt, M9). The 245 mm tool length
  cancels out, so this is independent of its ±5 mm uncertainty.

This is a CHECK, not a calibration: the extrinsics come from the calibration
project (C3). Pure Python, runs on the host.

Usage -- the summary lines come from ``signal_reader.py ... --summary``::

    <reader objects   --duration 10 --summary> | tail -1 | python3 b23_compare.py add pts.json P1 cam
    <reader cartesian --duration 3  --summary> | tail -1 | python3 b23_compare.py add pts.json P1 rob
    python3 b23_compare.py eval pts.json
"""

import json
import math
import os
import sys

#: Flange z when the closed jaws touch the bare belt, mean of M9 (mm).
FLANGE_Z_ON_BELT_MM = 298.56
#: Tracker measuring region as base_cam reports it (mm). Since 22.09.2026 the
#: calibration is turned into ``world`` (Nachtrag 12 / K5), which mirrors the old
#: region -1000 ... -500 (UR frame ``base``) to +500 ... +1000.
OLD_REGION_Y_MM = (500.0, 1000.0)
#: Acceptance limit for B23 (Fahrplan Block 2), mm.
B23_LIMIT_MM = 10.0


def load(path):
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {}


def cmd_add(path, label, part):
    if part not in ("cam", "rob"):
        sys.exit("part muss cam oder rob sein")
    line = sys.stdin.read().strip().splitlines()
    if not line:
        sys.exit("keine Eingabe auf stdin")
    try:
        data = json.loads(line[-1])
    except json.JSONDecodeError:
        sys.exit(f"letzte Zeile ist kein JSON: {line[-1][:120]}")
    pts = load(path)
    pts.setdefault(label, {})[part] = data
    with open(path, "w") as f:
        json.dump(pts, f, indent=2, sort_keys=True)
    warn = ""
    if part == "rob" and data.get("frozen"):
        warn = "  ⚠ Roboterdaten eingefroren -- Pendant-Programm läuft?"
    if part == "rob" and data.get("tilt_deg", 0) > 2.0:
        warn += f"  ⚠ Werkzeug {data['tilt_deg']:.1f}° schräg -- Berührpunkt nicht unter dem Flansch"
    if part == "cam" and data.get("ids_seen", 1) > 1:
        warn += f"  ⚠ {data['ids_seen']} IDs gesehen -- genommen wurde die häufigste ({data['id']})"
    print(f"{label}.{part}: x {data['x']:.1f}  y {data['y']:.1f}  z {data['z']:.1f} mm{warn}")


def rigid_fit(src, dst):
    """Least-squares rotation+translation mapping src -> dst (2D)."""
    n = len(src)
    cs = (sum(p[0] for p in src) / n, sum(p[1] for p in src) / n)
    cd = (sum(p[0] for p in dst) / n, sum(p[1] for p in dst) / n)
    num = den = 0.0
    for (sx, sy), (dx, dy) in zip(src, dst):
        ax, ay, bx, by = sx - cs[0], sy - cs[1], dx - cd[0], dy - cd[1]
        num += ax * by - ay * bx
        den += ax * bx + ay * by
    th = math.atan2(num, den)
    c, s = math.cos(th), math.sin(th)
    tx = cd[0] - (c * cs[0] - s * cs[1])
    ty = cd[1] - (s * cs[0] + c * cs[1])
    return th, (tx, ty)


def apply(th, t, p):
    c, s = math.cos(th), math.sin(th)
    return (c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1])


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


def cmd_eval(path):
    pts = load(path)
    pairs = [(k, v["cam"], v["rob"]) for k, v in sorted(pts.items()) if "cam" in v and "rob" in v]
    single = [k for k, v in pts.items() if not ("cam" in v and "rob" in v)]
    if single:
        print(f"unvollständig (ignoriert): {', '.join(sorted(single))}")
    if not pairs:
        sys.exit("keine vollständigen Paare")

    print(f"\n{'Punkt':6s} {'Kamera x':>9s} {'y':>8s} {'σy':>5s} | {'Roboter x':>9s} {'y':>8s} | "
          f"{'h Kamera':>8s} {'h Roboter':>9s}")
    for k, c, r in pairs:
        print(f"{k:6s} {c['x']:9.1f} {c['y']:8.1f} {c['sy']:5.2f} | {r['x']:9.1f} {r['y']:8.1f} | "
              f"{c['h']:8.1f} {r['z'] - FLANGE_Z_ON_BELT_MM:9.1f}")

    cam = [(c["x"], c["y"]) for _, c, _ in pairs]
    rob = [(r["x"], r["y"]) for _, _, r in pairs]

    print("\n-- B23: Hypothesen (Abstand Kamera→Roboter je Punkt, mm)")
    for name, f in (("Identität        ", lambda p: p),
                    ("180° um z (−x,−y)", lambda p: (-p[0], -p[1]))):
        d = [math.dist(f(c), r) for c, r in zip(cam, rob)]
        print(f"  {name}  " + "  ".join(f"{v:7.1f}" for v in d) + f"   RMS {rms(d):7.1f}")

    if len(pairs) >= 2:
        th, t = rigid_fit(cam, rob)
        res = [math.dist(apply(th, t, c), r) for c, r in zip(cam, rob)]
        print(f"\n-- Starre 2D-Ausgleichsrechnung Kamera → Roboter ({len(pairs)} Paare)")
        print(f"  Drehung      {math.degrees(th):+8.2f}°")
        print(f"  Verschiebung ({t[0]:+.1f}, {t[1]:+.1f}) mm")
        print("  Restfehler   " + "  ".join(f"{k}:{v:.1f}" for (k, _, _), v in zip(pairs, res))
              + f"   RMS {rms(res):.1f} mm")
        if len(pairs) == 2:
            print("  ⚠ Zwei Paare bestimmen die Transformation exakt -- Restfehler ohne Aussage. ≥ 3 messen.")
        else:
            verdict = "bestanden" if max(res) <= B23_LIMIT_MM else "NICHT bestanden"
            print(f"  Mit dieser Korrektur: max. Restfehler {max(res):.1f} mm → {verdict} "
                  f"(Grenze {B23_LIMIT_MM:.0f} mm, Fahrplan Block 2)")
            d_id = max(math.dist(c, r) for c, r in zip(cam, rob))
            print(f"  Ohne Korrektur (so wie base_cam heute meldet): max. {d_id:.1f} mm → "
                  + ("Basiskamera darf an den Follower" if d_id <= B23_LIMIT_MM
                     else "Basiskamera bleibt vom Follower abgeklemmt (bis C3)"))

    print("\n-- B21: Streuung der Längsposition (Kamera, σy)")
    inside = [c["sy"] for _, c, _ in pairs if OLD_REGION_Y_MM[0] <= c["y"] <= OLD_REGION_Y_MM[1]]
    outside = [c["sy"] for _, c, _ in pairs if not OLD_REGION_Y_MM[0] <= c["y"] <= OLD_REGION_Y_MM[1]]
    for name, vals in (("innerhalb der alten Region", inside), ("außerhalb", outside)):
        if vals:
            print(f"  {name:28s} n {len(vals)}  σy max {max(vals):.2f}  Median {sorted(vals)[len(vals)//2]:.2f} mm")
        else:
            print(f"  {name:28s} keine Punkte")
    print("  Bezug M5 (ruhend, innerhalb): σ 0,2 … 0,6 mm")

    hs = [(c["h"], r["z"] - FLANGE_Z_ON_BELT_MM) for _, c, r in pairs]
    print("\n-- Höhe: Kamera gegen Roboter (Roboter unabhängig von den 245 mm)")
    print("  Differenz je Punkt " + "  ".join(f"{a - b:+.1f}" for a, b in hs) + " mm")


def main():
    if len(sys.argv) >= 5 and sys.argv[1] == "add":
        cmd_add(sys.argv[2], sys.argv[3], sys.argv[4])
    elif len(sys.argv) == 3 and sys.argv[1] == "eval":
        cmd_eval(sys.argv[2])
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
