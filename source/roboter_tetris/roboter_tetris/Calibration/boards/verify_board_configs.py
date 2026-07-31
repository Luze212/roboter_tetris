#!/usr/bin/env python3
"""Validate the calibration-board definitions by constructing them with OpenCV."""

import json
import sys
from pathlib import Path


CONFIG_DIR = Path(__file__).resolve().parent
PACKAGE_SOURCE = CONFIG_DIR.parents[2]
sys.path.insert(0, str(PACKAGE_SOURCE))

from roboter_tetris.vision.board import BoardParams, build_board  # noqa: E402


PARAMETER_KEYS = (
    "board_type", "aruco_dictionary", "board_rows", "board_cols",
    "marker_length_m", "marker_spacing_m", "min_detected_markers",
)


def load_params(config_path: Path) -> BoardParams:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    return BoardParams(**{key: config[key] for key in PARAMETER_KEYS})


def main() -> None:
    for config_path in sorted(CONFIG_DIR.glob("*.json")):
        board, dictionary = build_board(load_params(config_path))
        assert board is not None and dictionary is not None
        print(f"OK: {config_path.name}")


if __name__ == "__main__":
    main()
