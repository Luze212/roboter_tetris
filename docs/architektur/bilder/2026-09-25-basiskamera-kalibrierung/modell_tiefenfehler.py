"""Depth error of the L515 against its colour image: model fit (A2, 27.09.2026).

Truth per pixel: z of the board from the colour pose (PnP with distortion, undistorted
ray). Measurement: the colour-aligned depth pixel. Error e = z_depth - z_true [mm].

Data
  A  the 13 board images of this folder (belt 0.85 m, block 0.75 m, gripper 0.53 m), dense
  B  run 3: the 28 gripper poses with a depth raster (0.49-0.59 m)

Model M1s (chosen, entscheidungen.md L27), in what base_cam has at hand -- pixel (u, v),
measured depth z [m], rays without undistortion:
    X = (u - cx) / fx * z,   Y = (v - cy) / fy * z
    e [mm] = c0 + c1 * X + c2 * Y + c3 * z        z_corrected = z - e / 1000
Also printed for comparison: M0 (offset), M1 (offset + tilt), M2 (M1s + quadratic image pattern).

Checks
  leave-one-out   every image of A left out of the fit once
  belt            run 3 belt depth, corrected, under the colour pose: still tilted?
  belt profile    the 9 flat board images in world under the colour pose -- colour only,
                  independent of depth, camera height and board thickness

Usage from source/roboter_tetris:
    PYTHONPATH=. python3 ../../docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/modell_tiefenfehler.py
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_argv, sys.argv = sys.argv, sys.argv[:1]   # the evaluation script loops over its arguments
import auswertung_farbe_gegen_tiefe as af   # noqa: E402  detect_small, all_corners, K, D
sys.argv = _argv

from roboter_tetris.basecam_extrinsics import fit_plane, matrix_from_cal, solve_pnp  # noqa: E402

K, D = af.K, af.D
S = HERE + "/"
SHOTS = ["greifer_start", "band_roboterseite", "band_mitte", "band_gegenseite",
         "band_a1", "band_a2", "band_a3", "band_a4", "band_a5", "band_a6",
         "band_b1", "band_b2", "band_b3"]
FLAT = [n for n in SHOTS if n.startswith("band_") and not n.startswith("band_b")]
ROI = (342, 60, 618, 580)   # base_cam roi_x, roi_y, roi_width, roi_height
MODELS = ("M0", "M1", "M1s", "M2")


def true_z(cam_T_board, uv):
    """z of the board point under pixel uv from the colour pose (undistorted ray), m."""
    n, p0 = cam_T_board[:3, 2], cam_T_board[:3, 3]
    xy = cv2.undistortPoints(np.asarray(uv, float).reshape(-1, 1, 2), K, D).reshape(-1, 2)
    return (n @ p0) / (np.c_[xy, np.ones(len(xy))] @ n)


def colour_pose(name):
    g0 = cv2.imread(S + f"{name}_grau.png", cv2.IMREAD_GRAYSCALE)
    obj, img, T, _ = af.all_corners(g0, af.detect_small(g0))
    return img, T


def rows_from_shot(name, stride=3):
    d = cv2.imread(S + f"{name}_tiefe_mm.png", cv2.IMREAD_UNCHANGED).astype(float)
    img, T = colour_pose(name)
    mask = np.zeros(d.shape, np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(np.round(img).astype(np.int32)), 1)
    mask = cv2.erode(mask, np.ones((9, 9), np.uint8))
    vs, us = np.nonzero(mask[::stride, ::stride])
    vs, us = vs * stride, us * stride
    z = d[vs, us]
    ok = z > 0
    us, vs, z = us[ok].astype(float), vs[ok].astype(float), z[ok]
    return dict(name=name, u=us, v=vs, zm=z, zt=1000 * true_z(T, np.c_[us, vs]))


def rows_from_run3():
    raw = json.load(open(S + "lauf3_rohdaten.json"))
    out = []
    for role, items in raw["samples"].items():
        for i, s in enumerate(items):
            g = np.array(s["depth_grid"], float)
            if len(g) == 0:
                continue   # top layer (board ~0.44 m): no depth over the board
            T, _ = solve_pnp(np.array(s["obj"]), np.array(s["img"]), K, D)
            out.append(dict(name=f"{role}{i}", u=g[:, 0], v=g[:, 1], zm=g[:, 2],
                            zt=1000 * true_z(T, g[:, :2])))
    return out


def features(r, model):
    xn = (r["u"] - K[0, 2]) / K[0, 0]
    yn = (r["v"] - K[1, 2]) / K[1, 1]
    z = r["zm"] / 1000.0
    cols = [np.ones_like(z)]
    if model != "M0":
        cols += [xn * z, yn * z]
    if model in ("M1s", "M2"):
        cols += [z]
    if model == "M2":
        cols += [xn ** 2, yn ** 2, xn * yn]
    return np.column_stack(cols)


def fit(rows, model):
    """Least squares, every image / pose weighted equally."""
    A = np.vstack([features(r, model) for r in rows])
    e = np.concatenate([r["zm"] - r["zt"] for r in rows])
    w = np.concatenate([np.full(len(r["zm"]), 1 / np.sqrt(len(r["zm"]))) for r in rows])
    c, *_ = np.linalg.lstsq(A * w[:, None], e * w, rcond=None)
    return c


def median_rest(r, model, c):
    return float(np.median(r["zm"] - r["zt"] - features(r, model) @ c))


def colour_calibration():
    res = json.load(open(S + "lauf3_ergebnis_farbkamera.json"))
    return matrix_from_cal(*[res["cal"][k] for k in
                             ("cal_x", "cal_y", "cal_z", "cal_roll", "cal_pitch", "cal_yaw")])


def belt_check(A, B, X):
    raw = json.load(open(S + "lauf3_rohdaten.json"))
    g = np.array(raw["belt_depth_grid"], float)
    g = g[np.abs(g[:, 2] - np.median(g[:, 2])) < 80]
    r = dict(u=g[:, 0], v=g[:, 1], zm=g[:, 2])
    print("\nBand (Lauf 3) unter der Farb-Lage — Steigung in world [Grad] (negativ in x: steigt zu −x an):")
    for label, model in [("Tiefe roh", None)] + [(f"korrigiert {m}", m) for m in MODELS]:
        zc = r["zm"] if model is None else r["zm"] - features(r, model) @ fit(A + B, model)
        z = zc / 1000
        P = np.c_[(r["u"] - K[0, 2]) / K[0, 0] * z, (r["v"] - K[1, 2]) / K[1, 1] * z, z]
        n = X[:3, :3] @ fit_plane(P).normal
        n = n * np.sign(n[2])   # upwards: slope dz/dx = -n_x / n_z
        print(f"  {label:15s} quer (x) {np.degrees(np.arctan(-n[0] / n[2])):+.2f}   "
              f"längs (y) {np.degrees(np.arctan(-n[1] / n[2])):+.2f}")


def belt_profile(X):
    """Board centres of the flat images in world, colour only."""
    P = []
    for name in FLAT:
        _, T = colour_pose(name)
        P.append((X @ T @ np.array([0.143, 0.091, 0.0, 1.0]))[:3])
    P = np.array(P)
    c, *_ = np.linalg.lstsq(np.c_[np.ones(len(P)), P[:, 0], P[:, 1]], P[:, 2], rcond=None)
    rest = P[:, 2] - np.c_[np.ones(len(P)), P[:, 0], P[:, 1]] @ c
    print(f"\nBandprofil nur aus dem Farbbild ({len(P)} Board-Lagen, x {P[:, 0].min():.3f}…"
          f"{P[:, 0].max():.3f} m, y {P[:, 1].min():.3f}…{P[:, 1].max():.3f} m):")
    print(f"  Steigung in x {np.degrees(np.arctan(c[1])):+.2f} Grad (negativ: steigt zu −x an), "
          f"in y {np.degrees(np.arctan(c[2])):+.2f} Grad, Rest {1000 * np.sqrt(np.mean(rest ** 2)):.2f} mm rms")
    print(f"  Höhe der Board-Mitten relativ: {1000 * (P[:, 2].min() - P[:, 2].mean()):+.1f} … "
          f"{1000 * (P[:, 2].max() - P[:, 2].mean()):+.1f} mm")


def main():
    A = [rows_from_shot(n) for n in SHOTS]
    B = rows_from_run3()
    print(f"A: {len(A)} Aufnahmen, {sum(len(r['zm']) for r in A)} Pixel; "
          f"B: {len(B)} Posen, {sum(len(r['zm']) for r in B)} Pixel")

    print("\nMedian-Rest je Aufnahme, jeweils ohne sie angepasst (A+B) [mm]:")
    print(f"  {'Aufnahme':18s} {'roh':>6s} " + " ".join(f"{m:>6s}" for m in MODELS))
    table = []
    for i, r in enumerate(A):
        pool = A[:i] + A[i + 1:] + B
        row = [float(np.median(r["zm"] - r["zt"]))] + [median_rest(r, m, fit(pool, m)) for m in MODELS]
        table.append(row)
        print(f"  {r['name']:18s} " + " ".join(f"{x:+6.1f}" for x in row))
    t = np.abs(np.array(table))
    print(f"  {'max |.|':18s} " + " ".join(f"{x:6.1f}" for x in t.max(0)))
    print(f"  {'rms':18s} " + " ".join(f"{x:6.1f}" for x in np.sqrt((t ** 2).mean(0))))

    c = fit(A + B, "M1s")
    print("\nM1s auf A+B (e [mm] = c0 + c1*X + c2*Y + c3*z; X, Y, z in m):")
    print(f"  c = [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}, {c[3]:.3f}]")
    print(f"  = Versatz {c[0]:.2f} mm, Neigung um Bild-y {np.degrees(np.arctan(c[1] / 1000)):.2f} Grad, "
          f"um Bild-x {np.degrees(np.arctan(c[2] / 1000)):.2f} Grad, Maßstab {c[3]:.2f} mm/m")
    q = dict(u=np.array([ROI[0], ROI[0] + ROI[2] / 2, ROI[0] + ROI[2]] * 3, float),
             v=np.repeat([ROI[1], ROI[1] + ROI[3] / 2, ROI[1] + ROI[3]], 3).astype(float),
             zm=np.full(9, 800.0))
    corr = features(q, "M1s") @ c
    print("  Korrektur bei z = 0,80 m an den Ecken / Mitten des Bildausschnitts [mm]:")
    for k in range(3):
        print("   " + "  ".join(f"{x:5.1f}" for x in corr[3 * k:3 * k + 3]))

    X = colour_calibration()
    belt_check(A, B, X)
    belt_profile(X)


if __name__ == "__main__":
    main()
