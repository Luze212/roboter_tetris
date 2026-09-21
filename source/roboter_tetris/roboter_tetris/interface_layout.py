"""Layout behind `interface_streamer` -- one overview image for RViz.

Two parts, both without ROS: the text (plain Python, testable on its own) and
the composition (numpy + OpenCV, like the vision code). The component only
converts messages and publishes.

The text is ASCII on purpose: OpenCV's built-in Hershey fonts have no umlauts,
no degree sign and no Greek letters -- "laengs", "Grad", "dv".

    +-----------------------+-----------------------+
    |  base camera debug    |  robot camera debug   |
    +-----------------------+-----------------------+
    |  follower: state, target, w, errors           |
    |  belt: pooled velocity, direction, count      |
    +-----------------------------------------------+
    |  one row per block from world_state           |
    +-----------------------------------------------+
"""

import math
from typing import List, NamedTuple, Optional, Sequence

import cv2
import numpy as np

from .contracts import (
    COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_RED, COLOR_UNKNOWN,
    COLOR_WHITE, COLOR_YELLOW, FOLLOWER_STATES, TRACK_FINAL, TRACK_SETTLING,
    FollowerStatus, WorldStateMsg,
)

COLOR_NAMES = {
    COLOR_RED: "rot", COLOR_YELLOW: "gelb", COLOR_GREEN: "gruen",
    COLOR_BLUE: "blau", COLOR_WHITE: "weiss", COLOR_BLACK: "schwarz",
    COLOR_UNKNOWN: "unbekannt",
}

LINE_HEIGHT = 24
STATUS_LINES = 3
LIST_ROWS = 8
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.55
BACKGROUND = (32, 32, 32)
NO_IMAGE = (80, 80, 80)
TEXT = (235, 235, 235)
DIMMED = (130, 130, 130)
WARNING = (60, 170, 255)            # orange, BGR


class Row(NamedTuple):
    text: str
    dimmed: bool = False            # done or no longer in the image
    warning: bool = False


# -- Text ----------------------------------------------------------------------------

def status_lines(status: Optional[FollowerStatus], world: Optional[WorldStateMsg],
                 follower_stale: bool = False, world_stale: bool = False) -> List[Row]:
    """The three lines under the camera images."""
    rows = []
    if status is None:
        rows.append(Row("Follower: keine Daten", warning=True))
        rows.append(Row(""))
    else:
        state = FOLLOWER_STATES.get(int(status.state), f"? ({status.state:.0f})")
        target = f"{status.target_id:.0f}" if status.target_id else "-"
        suffix = "   (veraltet)" if follower_stale else ""
        rows.append(Row(f"Zustand: {state}   Ziel: {target}   "
                        f"w: {status.w_effective:.2f}{suffix}", warning=follower_stale))
        rows.append(Row(f"Abweichung  laengs {status.err_long * 1000:+.1f} mm   "
                        f"quer {status.err_lat * 1000:+.1f} mm   "
                        f"z {status.err_z * 1000:+.1f} mm"))
    rows.append(belt_line(world, world_stale))
    return rows


def belt_line(world: Optional[WorldStateMsg], stale: bool = False) -> Row:
    """Pooled belt velocity (project goal 3) -- or why there is none yet."""
    if world is None:
        return Row("Band: keine Daten", warning=True)
    if world.n_pool < 1:
        return Row("Band: noch keine Schaetzung (kein Klotz eingeschwungen)")
    speed = math.hypot(world.v_belt_x, world.v_belt_y)
    heading = math.degrees(math.atan2(world.v_belt_y, world.v_belt_x))
    suffix = "   (veraltet)" if stale else ""
    return Row(f"Band  v = {speed * 1000:.0f} mm/s   Richtung {heading:+.1f} Grad   "
               f"({world.n_pool:.0f} Kloetze){suffix}", warning=stale)


def object_row(entry) -> Row:
    """One block of S10. Frozen follow-up entries (present = 0) never show a
    position, as if the block were still there (Nachtrag 7 / T1)."""
    color = COLOR_NAMES.get(int(entry.color), "?")
    head = f"ID {entry.id:>4.0f}  {color:<9}"
    if entry.picked:
        return Row(f"{head} gepickt", dimmed=True)
    if entry.out_of_bounds:
        return Row(f"{head} hinter der Greifebene", dimmed=True)
    if not entry.present:
        return Row(f"{head} nicht mehr im Bild", dimmed=True)
    if entry.status == TRACK_FINAL:
        speed = math.hypot(entry.vx, entry.vy)
        return Row(f"{head} final           v = {speed * 1000:.0f} mm/s   "
                   f"h {entry.height * 1000:.0f} mm")
    if entry.status == TRACK_SETTLING:
        return Row(f"{head} einschwingend   dv = {entry.v_change * 1000:.0f} mm/s")
    return Row(f"{head} Status {entry.status:.0f}", warning=True)


def object_rows(world: Optional[WorldStateMsg], max_rows: int = LIST_ROWS) -> List[Row]:
    """The block list; the newest entries if not all fit."""
    if world is None or not world.entries:
        return [Row("keine Kloetze", dimmed=True)]
    rows = [object_row(e) for e in world.entries]
    if len(rows) <= max_rows:
        return rows
    hidden = len(rows) - (max_rows - 1)
    return [Row(f"... {hidden} aeltere", dimmed=True)] + rows[-(max_rows - 1):]


# -- Composition ----------------------------------------------------------------------

def panel_height(image_height: int, show_list: bool) -> int:
    rows = STATUS_LINES + (LIST_ROWS if show_list else 0)
    return image_height + rows * LINE_HEIGHT + LINE_HEIGHT // 2


def fit_image(image: Optional[np.ndarray], width: int, height: int,
              label: str) -> np.ndarray:
    """Scale into width x height keeping the aspect ratio; grey with a label
    when there is no image -- the display must never fail because a source
    is silent."""
    area = np.full((height, width, 3), NO_IMAGE, dtype=np.uint8)
    if image is None or image.size == 0:
        cv2.putText(area, f"{label}: keine Daten", (12, height // 2), FONT,
                    FONT_SCALE, TEXT, 1, cv2.LINE_AA)
        return area
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    scale = min(width / image.shape[1], height / image.shape[0])
    w = max(1, int(round(image.shape[1] * scale)))
    h = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
    x0, y0 = (width - w) // 2, (height - h) // 2
    area[y0:y0 + h, x0:x0 + w] = resized
    return area


def compose(base_image: Optional[np.ndarray], robot_image: Optional[np.ndarray],
            status: Sequence[Row], objects: Sequence[Row], width: int,
            image_height: int, show_list: bool) -> np.ndarray:
    """The whole panel; its size depends only on the parameters."""
    canvas = np.full((panel_height(image_height, show_list), width, 3), BACKGROUND,
                     dtype=np.uint8)
    half = width // 2
    canvas[:image_height, :half] = fit_image(base_image, half, image_height, "Basiskamera")
    canvas[:image_height, half:] = fit_image(robot_image, width - half, image_height,
                                             "Roboterkamera")
    y = image_height + LINE_HEIGHT
    rows = list(status) + (list(objects)[:LIST_ROWS] if show_list else [])
    for index, row in enumerate(rows):
        if index == STATUS_LINES:
            cv2.line(canvas, (0, y - LINE_HEIGHT + 6), (width, y - LINE_HEIGHT + 6),
                     DIMMED, 1)
        color = WARNING if row.warning else DIMMED if row.dimmed else TEXT
        cv2.putText(canvas, row.text, (12, y), FONT, FONT_SCALE, color, 1, cv2.LINE_AA)
        y += LINE_HEIGHT
    return canvas
