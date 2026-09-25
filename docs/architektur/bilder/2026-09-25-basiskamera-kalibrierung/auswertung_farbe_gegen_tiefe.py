"""Board plane from the colour image (all grid corners) against the depth image.

Usage from source/roboter_tetris:
    PYTHONPATH=. python3 <this file> band_mitte band_a1 greifer_start ...
"""
import sys, cv2, numpy as np
from roboter_tetris.basecam_extrinsics import *
import os
S = os.path.dirname(os.path.abspath(__file__)) + "/"
K = np.array([[897.831, 0, 647.054], [0, 897.539, 362.306], [0, 0, 1.0]]); D = np.array([0.129869, -0.440544, -0.000885, -0.00064, 0.402135])
spec = GridSpec()
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
def detect_small(g0):
    """Union over a few settings (small tags at belt distance): median corners per id."""
    import itertools
    union = {}
    for scale, amt, sig, thr in itertools.product([2, 3], [2, 3, 4], [1.0, 2.0], [7, 13]):
        g = cv2.resize(g0, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        b = cv2.GaussianBlur(g, (0, 0), sig * scale); g = cv2.addWeighted(g, 1 + amt, b, -amt, 0)
        p = cv2.aruco.DetectorParameters_create(); p.perspectiveRemovePixelPerCell = 20
        p.perspectiveRemoveIgnoredMarginPerCell = 0.4; p.adaptiveThreshConstant = thr
        c, ids, _ = cv2.aruco.detectMarkers(g, dic, parameters=p)
        if ids is None: continue
        for i, q in zip(ids.ravel(), c):
            if i < 77: union.setdefault(int(i), []).append(q.reshape(4, 2) / scale)
    return {i: np.median(v, axis=0) for i, v in union.items()}
def all_corners(g0, det):
    obj, img = grid_points(spec, det)
    T, _ = solve_pnp(obj, img, K, D)
    for _ in range(2):   # grow: project every corner, refine, re-solve
        allobj = np.vstack([spec.tag_corners(t) for t in range(77)])
        guess, _ = cv2.projectPoints(allobj, matrix_to_rotvec(T[:3, :3]), T[:3, 3], K, D)
        ref = guess.astype(np.float32).copy()
        cv2.cornerSubPix(g0, ref, (2, 2), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.005))
        ok = np.linalg.norm(ref.reshape(-1, 2) - guess.reshape(-1, 2), axis=1) < 1.0
        T, err = solve_pnp(allobj[ok], ref.reshape(-1, 2)[ok].astype(float), K, D)
    return allobj[ok], ref.reshape(-1, 2)[ok].astype(float), T, err
for n in sys.argv[1:]:
    g0 = cv2.imread(S + f"{n}_grau.png", cv2.IMREAD_GRAYSCALE)
    d = cv2.imread(S + f"{n}_tiefe_mm.png", cv2.IMREAD_UNCHANGED).astype(float) / 1000
    det = detect_small(g0)
    if len(det) < 3:
        print(f"{n}: nur {len(det)} Tags"); continue
    obj, img, T, err = all_corners(g0, det)
    nc = -T[:3, 2] if T[2, 2] > 0 else T[:3, 2]
    pl = board_depth_plane(d, K, img)
    ang = np.degrees(np.arccos(abs(pl.normal @ nc)))
    axis = np.cross(pl.normal, nc)
    print(f"{n}: {len(det)} Tags gelesen, {len(obj)} Ecken fein ({err:.2f} px), Mitte im Bild {np.round(img.mean(axis=0)).astype(int).tolist()} | "
          f"Abstand Farbe {abs(nc @ T[:3,3])*1000:.1f} / Tiefe {abs(pl.offset)*1000:.1f} mm | Neigung Farbe gegen Tiefe {ang:.2f} Grad "
          f"(um x {np.degrees(axis[0]):+.2f}, um y {np.degrees(axis[1]):+.2f})")
