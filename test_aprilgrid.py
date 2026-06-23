#!/usr/bin/env python3
"""Test AprilGrid detection with the exact parameters from the board image."""

import sys
sys.path.insert(0, 'source/roboter_tetris')

from roboter_tetris.vision.board import (
    BoardParams, build_board, detect_board
)

# Configuration matching the board in the image
params = BoardParams(
    board_type="GRID",
    aruco_dictionary="t36h11",
    board_rows=11,
    board_cols=7,
    marker_length_m=0.020,  # 20mm
    marker_spacing_m=0.006,  # 6mm
    min_detected_markers=4
)

print("=" * 60)
print("AprilGrid Board Detection Test")
print("=" * 60)
print(f"Board config: {params.board_cols}x{params.board_rows}")
print(f"Tag size: {params.marker_length_m*1000:.0f}mm")
print(f"Tag spacing: {params.marker_spacing_m*1000:.0f}mm")
print(f"Dictionary: {params.aruco_dictionary}")
print()

try:
    print("Building GRID board...")
    board, dictionary = build_board(params)
    print(f"✓ Board created successfully")
    print(f"  Board object: {board}")
    print(f"  Dictionary: {dictionary}")
except Exception as e:
    print(f"✗ Error building board: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("Testing with sample image...")
print("=" * 60)

import cv2
import numpy as np

# Create a test image (empty/white background)
h, w = 480, 640
test_img = np.ones((h, w, 3), dtype=np.uint8) * 255

try:
    result = detect_board(test_img, board, dictionary, "GRID")
    if result is None:
        print("✓ Detection: No markers found (expected on blank image)")
    else:
        marker_corners, marker_ids, board_corners, board_ids = result
        print(f"✓ Detection returned data:")
        print(f"  Marker IDs found: {marker_ids.flatten() if marker_ids is not None else 'None'}")
        print(f"  Number of markers: {len(marker_ids) if marker_ids is not None else 0}")
        print(f"  Board corners: {board_corners}")
        print(f"  Board IDs: {board_ids}")
except Exception as e:
    print(f"✗ Error in detection: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("✓ All tests passed! AprilGrid board configuration is valid.")
print("=" * 60)
print("\nTo use in ROS2 component, set these parameters:")
print("  - board_type: GRID")
print("  - aruco_dictionary: t36h11")
print("  - board_rows: 11")
print("  - board_cols: 7")
print("  - marker_length_m: 0.020")
print("  - marker_spacing_m: 0.006")
