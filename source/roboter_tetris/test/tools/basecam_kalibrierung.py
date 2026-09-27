#!/usr/bin/env python3
"""Calibration file for base_cam from the raw data of a stage 1 run (L27).

The calibration component writes this file itself since 27.09.2026. This tool
does the same from the raw data it leaves beside it
(``/tmp/base_cam_extrinsics_rohdaten.json``) -- for runs made before, for a
second look, or with another depth model. It solves stage 1 again, turns the
colour pose into the pose for base_cam and compares against calibration files.

Nothing is written unless ``--out`` is given; the calibration in force is never
touched.

Usage from source/roboter_tetris (needs numpy and OpenCV)::

    PYTHONPATH=. python3 test/tools/basecam_kalibrierung.py ROHDATEN.json \\
        [--out NEU.json] [--vergleich DATEI.json ...]

Without ``--vergleich`` it compares against the file shipped in the package
(Extrinsics/base_cam_extrinsics.json, the hand calibration L6).
"""

import argparse
import datetime
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

from roboter_tetris.basecam_extrinsics import (  # noqa: E402
    DEFAULT_CALIBRATION_FILE, DepthErrorModel, ExtrinsicsRecord, Sample, basecam_pose,
    belt_sample_points, cal_from_matrix, fit_plane, invert, load_camera_calibration,
    pixel_rays, resolve_calibration_path, rotation_angle_deg, save_record, solve_stage1,
)

BELT_Z_M = 0.0536
ROI = (342, 60, 618, 580)


def belt_plane_from_raw(raw, K):
    """The belt in the depth image at the start of the run (every 16th pixel)."""
    g = np.asarray(raw.get("belt_depth_grid") or [], dtype=np.float64)
    if len(g) < 200:
        raise SystemExit("Rohdaten ohne Band im Tiefenbild (belt_depth_grid) - Lauf ohne Tiefe?")
    P = pixel_rays(g[:, :2], K) * (g[:, 2:3] / 1000.0)
    P = P[np.abs(P[:, 2] - np.median(P[:, 2])) < 0.08]   # the belt, not arm or board
    return fit_plane(P)


def compare(label, T_new, T_old, K, size):
    belt = belt_sample_points(T_old, K, size, BELT_Z_M, roi=ROI)
    moved = (T_new @ invert(T_old) @ np.c_[belt, np.ones(len(belt))].T)[:3].T
    d = 1000.0 * np.linalg.norm(moved - belt, axis=1)
    angle = rotation_angle_deg(T_new[:3, :3] @ T_old[:3, :3].T)
    print(f"  gegen {label}: Verschiebung auf dem Band im Bildausschnitt mittel {d.mean():.1f} mm, "
          f"größte {d.max():.1f} mm; Drehung {angle:.3f} Grad")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("rohdaten")
    ap.add_argument("--out", help="Kalibrierdatei hierhin schreiben")
    ap.add_argument("--vergleich", nargs="*", default=None,
                    help="Kalibrierdateien zum Vergleich (Standard: die im Paket)")
    args = ap.parse_args()

    raw = json.load(open(args.rohdaten, encoding="utf-8"))
    if raw.get("mode") != "stufe1":
        raise SystemExit(f"Rohdaten aus Betriebsart {raw.get('mode')!r}, gebraucht: stufe1")
    K = np.asarray(raw["camera"]["K"], dtype=np.float64)
    D = np.asarray(raw["camera"]["D"], dtype=np.float64)
    size = tuple(raw["camera"]["size"])
    samples = [Sample(np.asarray(s["flange"]), np.asarray(s["obj"]), np.asarray(s["img"]))
               for s in raw["samples"]["kalibrieren"]]
    plane = belt_plane_from_raw(raw, K)
    try:
        result = solve_stage1(samples, K, D, prior=np.asarray(raw["reference_T"]),
                              camera_z_m=raw.get("camera_z_from_depth"))
    except ValueError as exc:
        raise SystemExit(f"Stufe 1 nicht lösbar: {exc}")
    model = DepthErrorModel()
    pose = basecam_pose(result.world_T_cam, K, D, plane, BELT_Z_M, ROI, model)
    T = pose.world_T_cam

    cal = cal_from_matrix(T)
    colour = cal_from_matrix(result.world_T_cam)
    print(f"Rohdaten {args.rohdaten} vom {raw.get('created', '?')}: {len(samples)} Posen, "
          f"Bildfehler {result.rms_px:.2f} px")
    print("Kalibrierung für base_cam: " + ", ".join(f"{k} {v:.4f}" for k, v in cal.items()))
    print("Lage der Farbkamera:       " + ", ".join(f"{k} {v:.4f}" for k, v in colour.items()))
    print(f"Rest der starren Lage über Bildausschnitt und Klotzhöhen: {pose.rest_mm_rms:.1f} mm rms, "
          f"{pose.rest_mm_max:.1f} mm größter (Grenze des unveränderten base_cam, auch bei L6)")
    files = args.vergleich if args.vergleich is not None else [DEFAULT_CALIBRATION_FILE]
    for f in files:
        if not os.path.exists(resolve_calibration_path(f)) and os.path.exists(
                resolve_calibration_path(os.path.join("Extrinsics", f))):
            f = os.path.join("Extrinsics", f)   # a bare name: the files in the package
        record, line = load_camera_calibration(f)
        if record is None:
            print(f"  {line}")
            continue
        compare(f"{f} ({record.method})", T, record.world_T_cam, K, size)

    if args.out:
        record = ExtrinsicsRecord(
            world_T_cam=T, method="stufe1_basecam",
            created=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            quality={"rohdaten": os.path.basename(args.rohdaten),
                     "rohdaten_vom": raw.get("created", ""),
                     "posen": len(samples) - result.poses_dropped,
                     "bildfehler_px": round(result.rms_px, 3),
                     "basecam_rest_mm": round(pose.rest_mm_rms, 2),
                     "basecam_rest_mm_max": round(pose.rest_mm_max, 2),
                     "farbkamera": {k: round(v, 6) for k, v in colour.items()},
                     "tiefenmodell": model.to_dict(),
                     "hinweis": "Kalibrierung für base_cam aus den Rohdaten eines Laufs, "
                                "nachgerechnet mit test/tools/basecam_kalibrierung.py (L27)."},
            grid=raw.get("grid") or {})
        save_record(args.out, record)
        print(f"geschrieben: {args.out}")


if __name__ == "__main__":
    main()
