"""ChArUco board-detection utilities for the roboter_tetris project."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np



@dataclass
class BoardParams:
    aruco_dictionary: str = "DICT_6X6_250"
    board_rows: int = 5
    board_cols: int = 7
    marker_spacing_m: float = 0.04
    marker_length_m: float = 0.02
    min_detected_markers: int = 4


def _get_aruco_dictionary(name: str) -> cv2.aruco_Dictionary:
    name = name.strip().upper()
    if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"Unsupported ArUco dictionary {name!r}.")
    return cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))


def build_board(params: BoardParams) -> Tuple[object, cv2.aruco_Dictionary]:
    dictionary = _get_aruco_dictionary(params.aruco_dictionary)
    board = cv2.aruco.CharucoBoard_create(
        params.board_cols,
        params.board_rows,
        float(params.marker_length_m + params.marker_spacing_m),
        float(params.marker_length_m),
        dictionary,
    )
    return board, dictionary


def detect_board(
        image_bgr: np.ndarray,
        board: object,
        dictionary: cv2.aruco_Dictionary,
) -> Optional[Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    detector_params = cv2.aruco.DetectorParameters_create()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=detector_params)

    if marker_ids is None or len(marker_ids) == 0:
        return None

    _, board_corners, board_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board)
    if board_ids is None or board_corners is None or len(board_ids) == 0:
        return None
    return marker_corners, marker_ids, board_corners.reshape(-1, 2), board_ids.flatten().astype(int)


def draw_board_debug(
        debug_img: np.ndarray,
        marker_corners: Optional[np.ndarray],
        marker_ids: Optional[np.ndarray],
        board_corners: Optional[np.ndarray],
        board_ids: Optional[np.ndarray],
) -> np.ndarray:
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(debug_img, marker_corners, marker_ids)

    if board_corners is not None and len(board_corners) > 0:
        for idx, corner in zip(board_ids, board_corners):
            x, y = int(round(corner[0])), int(round(corner[1]))
            cv2.circle(debug_img, (x, y), 4, (0, 255, 0), -1)
            cv2.putText(debug_img, str(int(idx)), (x + 3, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return debug_img
