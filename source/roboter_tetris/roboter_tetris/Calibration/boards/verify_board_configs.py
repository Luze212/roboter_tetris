#!/usr/bin/env python3
"""Validate the calibration-board definitions by constructing them with OpenCV."""

import json
import sys
from pathlib import Path


CONFIG_DIR = Path(__file__).resolve().parent
PACKAGE_SOURCE = CONFIG_DIR.parents[2]
sys.path.insert(0, str(PACKAGE_SOURCE))

from roboter_tetris.vision.board import BoardParams, build_board  # noqa: E402


def load_params(config_path: Path) -> BoardParams:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    return BoardParams(
        aruco_dictionary=config["aruco_dictionary"],
        board_rows=config["board_rows"],
        board_cols=config["board_cols"],
        marker_length_m=config["marker_size_m"],
        marker_spacing_m=config["checker_size_m"] - config["marker_size_m"],
        min_detected_markers=config["min_detected_markers"],
    )


def main() -> None:
    for config_path in sorted(CONFIG_DIR.glob("*.json")):
        board, dictionary = build_board(load_params(config_path))
        assert board is not None and dictionary is not None
        print(f"OK: {config_path.name}")


if __name__ == "__main__":
    main()
