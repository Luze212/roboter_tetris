#!/usr/bin/env python3
"""Quick test for board detection with GRID (AprilGrid)."""

import cv2
import numpy as np
import sys
sys.path.insert(0, 'source/roboter_tetris')

from roboter_tetris.vision.board import (
    BoardParams, build_board, detect_board, draw_board_debug
)

# Test parameters for AprilGrid
params = BoardParams(
    board_type="GRID",
    aruco_dictionary="DICT_APRILTAG_36H11",
    board_rows=5,
    board_cols=7,
    marker_spacing_m=0.06,  # 60mm spacing between markers
    marker_length_m=0.05,   # 50mm markers
    min_detected_markers=4
)

print(f"Building GRID board with {params.board_cols}x{params.board_rows} markers...")
try:
    board, dictionary = build_board(params)
    print(f"✓ Board created successfully")
    print(f"  Board type: {params.board_type}")
    print(f"  Dictionary: {params.aruco_dictionary}")
    print(f"  Marker size: {params.marker_length_m}m, Spacing: {params.marker_spacing_m}m")
except Exception as e:
    print(f"✗ Error building board: {e}")
    sys.exit(1)

# Test on a dummy image
h, w = 480, 640
test_img = np.ones((h, w, 3), dtype=np.uint8) * 200

print(f"\nTesting detection on dummy image ({h}x{w})...")
try:
    result = detect_board(test_img, board, dictionary, params.board_type)
    if result is None:
        print("✓ No board detected (expected for dummy image)")
    else:
        marker_corners, marker_ids, board_corners, board_ids = result
        print(f"✓ Detection returned data:")
        print(f"  Markers found: {len(marker_ids)}")
        print(f"  Board corners: {board_corners}")
        print(f"  Board IDs: {board_ids}")
except Exception as e:
    print(f"✗ Error in detection: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n✓ All tests passed!")
