"""
Example configuration for board detection with different board types.

Usage:
  CHARUCO boards:
    - board_type: CHARUCO
    - aruco_dictionary: DICT_6X6_250 (or other ArUco dictionaries)
    - marker_spacing_m: should be > marker_length_m (adds margin to marker size)
    
  GRID/AprilGrid boards:
    - board_type: GRID
    - aruco_dictionary: t36h11 or DICT_APRILTAG_36H11 (AprilTag dictionaries)
    - marker_spacing_m: spacing between marker centers (not including marker size)
"""

# Example 1: Charuco board 5x7 with 35mm checker squares and 26mm markers
CHARUCO_5x7 = {
    "board_type": "CHARUCO",
    "aruco_dictionary": "DICT_5X5_250",
    "board_rows": 5,
    "board_cols": 7,
    "marker_length_m": 0.026,  # Marker size 26mm
    "marker_spacing_m": 0.009,  # Checker square = marker + spacing = 35mm
    "min_detected_markers": 4,
}

# Example 2: AprilGrid 4x6, tag size 35mm, spacing 11mm
APRILGRID_4x6 = {
    "board_type": "GRID",
    "aruco_dictionary": "t36h11",
    "board_rows": 4,
    "board_cols": 6,
    "marker_length_m": 0.035,   # Tag size 35mm
    "marker_spacing_m": 0.011,  # Spacing between tag edges 11mm
    "min_detected_markers": 4,
}

# Example 3: AprilGrid 36h10
APRILGRID_36H10 = {
    "board_type": "GRID",
    "aruco_dictionary": "DICT_APRILTAG_36H10",
    "board_rows": 4,
    "board_cols": 6,
    "marker_length_m": 0.03,   # Marker size 30mm
    "marker_spacing_m": 0.04,  # Spacing 40mm
    "min_detected_markers": 3,
}

print("Available board configurations:")
print("\n1. Charuco 6x6 (default):")
for k, v in CHARUCO_6x6.items():
    print(f"   {k}: {v}")

print("\n2. AprilGrid 36h11:")
for k, v in APRILGRID_36H11.items():
    print(f"   {k}: {v}")

print("\n3. AprilGrid 36h10:")
for k, v in APRILGRID_36H10.items():
    print(f"   {k}: {v}")
