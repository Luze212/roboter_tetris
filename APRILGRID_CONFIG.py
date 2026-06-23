#!/usr/bin/env python3
"""
Configuration for the AprilGrid board shown in the image.
Tag Size: 20 mm, Tag Spacing: 6 mm, Dictionary: 36h11 (7x11 layout)
"""

# AprilGrid parameters for the board in the image
APRILGRID_CONFIG = {
    "board_type": "GRID",
    "aruco_dictionary": "t36h11",  # or DICT_APRILTAG_36H11
    "board_rows": 11,      # Number of rows (vertical)
    "board_cols": 7,       # Number of columns (horizontal)
    "marker_length_m": 0.020,    # Tag size: 20 mm
    "marker_spacing_m": 0.006,   # Tag spacing: 6 mm
    "min_detected_markers": 4,
}

print("AprilGrid Configuration:")
print(f"  Type: {APRILGRID_CONFIG['board_type']}")
print(f"  Dictionary: {APRILGRID_CONFIG['aruco_dictionary']}")
print(f"  Layout: {APRILGRID_CONFIG['board_cols']}x{APRILGRID_CONFIG['board_rows']}")
print(f"  Tag size: {APRILGRID_CONFIG['marker_length_m']*1000:.0f} mm")
print(f"  Tag spacing: {APRILGRID_CONFIG['marker_spacing_m']*1000:.0f} mm")
print(f"  Min markers for detection: {APRILGRID_CONFIG['min_detected_markers']}")
