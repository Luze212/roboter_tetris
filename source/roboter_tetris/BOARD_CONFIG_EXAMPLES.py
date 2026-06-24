"""Example configuration for ChArUco board detection.

These keys match the parameters exposed in the AICA component description.
There is no board-type parameter; the board detection node only uses ChArUco.
"""

# ChArUco board 5x7 with 35 mm checker squares and 26 mm markers.
CHARUCO_5x7 = {
    "aruco_dictionary": "DICT_5X5_250",
    "board_rows": 5,
    "board_cols": 7,
    "marker_length_m": 0.026,
    "marker_spacing_m": 0.009,
    "min_detected_markers": 4,
}


print("Available board configurations:")
print("\n1. ChArUco 5x7:")
for key, value in CHARUCO_5x7.items():
    print(f"   {key}: {value}")
