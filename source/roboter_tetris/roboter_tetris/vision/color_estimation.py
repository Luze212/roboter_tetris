"""HSV color classification with patch voting.

Port of ``FuE_Greifen-main/cameras/color_estimation.hpp`` (read-only reference).
Pure numpy/cv2 — no ROS imports, unit-testable with synthetic images.

Color ids match the C++ ``ColorClass`` enum and the component's output contract:
0=Red, 1=Yellow, 2=Green, 3=Blue, 4=White, 5=Black, 6=Unknown.
"""

import cv2

COLOR_RED = 0
COLOR_YELLOW = 1
COLOR_GREEN = 2
COLOR_BLUE = 3
COLOR_WHITE = 4
COLOR_BLACK = 5
COLOR_UNKNOWN = 6
_NUM_CLASSES = 7

COLOR_NAMES = ["Red", "Yellow", "Green", "Blue", "White", "Black", "Unknown"]


def classify_hsv(h: int, s: int, v: int) -> int:
    """Classify a single HSV pixel (OpenCV ranges: H 0-179, S/V 0-255)."""
    if v < 45:
        return COLOR_BLACK
    if s < 35 and v > 170:
        return COLOR_WHITE
    if h <= 10 or h >= 170:
        return COLOR_RED
    if h <= 35:
        return COLOR_YELLOW
    if h <= 85:
        return COLOR_GREEN
    if h <= 135:
        return COLOR_BLUE
    return COLOR_UNKNOWN


def estimate_object_color_id(hsv_img, contour, center_xy, patch_radius_px: int = 2) -> int:
    """Majority vote over a small patch around ``center_xy``, restricted to pixels
    inside ``contour``. Falls back to the (clamped) exact center pixel if no patch
    pixel lands inside the contour — identical to the C++ behaviour, including the
    first-maximum tie-break.
    """
    rows, cols = hsv_img.shape[:2]
    cx, cy = int(center_xy[0]), int(center_xy[1])

    votes = [0] * _NUM_CLASSES
    for y in range(cy - patch_radius_px, cy + patch_radius_px + 1):
        for x in range(cx - patch_radius_px, cx + patch_radius_px + 1):
            if x < 0 or y < 0 or x >= cols or y >= rows:
                continue
            if cv2.pointPolygonTest(contour, (float(x), float(y)), False) < 0.0:
                continue
            h, s, v = hsv_img[y, x]
            votes[classify_hsv(int(h), int(s), int(v))] += 1

    if all(v == 0 for v in votes):
        fx = min(max(cx, 0), cols - 1)
        fy = min(max(cy, 0), rows - 1)
        h, s, v = hsv_img[fy, fx]
        return classify_hsv(int(h), int(s), int(v))

    best_idx = COLOR_UNKNOWN
    best_votes = -1
    for i, count in enumerate(votes):
        if count > best_votes:
            best_votes = count
            best_idx = i
    return best_idx
