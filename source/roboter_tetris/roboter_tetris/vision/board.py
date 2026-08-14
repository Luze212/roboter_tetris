"""ChArUco board-detection utilities for the roboter_tetris project.

Supports both legacy (OpenCV < 4.7) and modern (OpenCV >= 4.7) ArUco APIs.
"""

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


def _get_aruco_dictionary(name: str) -> object:
    name = name.strip().upper()
    if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"Unsupported ArUco dictionary {name!r}.")
    dict_val = getattr(cv2.aruco, name)

    # OpenCV 4.7+ uses getPredefinedDictionary, OpenCV < 4.7 uses Dictionary_get
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_val)
    elif hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dict_val)
    elif hasattr(cv2.aruco, "Dictionary"):
        return cv2.aruco.Dictionary(dict_val)
    raise AttributeError("No suitable ArUco dictionary getter found in cv2.aruco.")


def build_board(params: BoardParams) -> Tuple[object, object]:
    dictionary = _get_aruco_dictionary(params.aruco_dictionary)
    square_length = float(params.marker_length_m + params.marker_spacing_m)
    marker_length = float(params.marker_length_m)

    # OpenCV 4.7+ uses CharucoBoard constructor, OpenCV < 4.7 uses CharucoBoard_create
    if hasattr(cv2.aruco, "CharucoBoard"):
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            board = cv2.aruco.CharucoBoard_create(
                params.board_cols,
                params.board_rows,
                square_length,
                marker_length,
                dictionary,
            )
        else:
            board = cv2.aruco.CharucoBoard(
                (params.board_cols, params.board_rows),
                square_length,
                marker_length,
                dictionary,
            )
    else:
        board = cv2.aruco.CharucoBoard_create(
            params.board_cols,
            params.board_rows,
            square_length,
            marker_length,
            dictionary,
        )
    return board, dictionary


def detect_board(
        image_bgr: np.ndarray,
        board: object,
        dictionary: object,
) -> Optional[Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2.aruco, "DetectorParameters"):
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            detector_params = cv2.aruco.DetectorParameters_create()
        else:
            detector_params = cv2.aruco.DetectorParameters()
    else:
        detector_params = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers (OpenCV 4.7+ ArucoDetector vs legacy detectMarkers)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    elif hasattr(cv2.aruco, "detectMarkers"):
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=detector_params)
    else:
        return None

    if marker_ids is None or len(marker_ids) == 0:
        return None

    # Interpolate Charuco Corners
    if hasattr(cv2.aruco, "interpolateCornersCharuco"):
        _, board_corners, board_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board)
    elif hasattr(cv2.aruco, "CharucoDetector"):
        charuco_detector = cv2.aruco.CharucoDetector(board)
        board_corners, board_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)
    else:
        return None

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
