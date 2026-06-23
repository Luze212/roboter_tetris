# Board Detection Configuration Guide

## Charuco Board (funktioniert ✓)
Die aktuell funktionierende Konfiguration zeigt ein klassisches **Charuco 6x6 250** Board.

**Parameter:**
```
board_type: CHARUCO
aruco_dictionary: DICT_6X6_250
board_rows: 5
board_cols: 7
marker_length_m: 0.02        # 20mm marker size
marker_spacing_m: 0.04       # spacing adds to marker (total square = 6cm)
min_detected_markers: 4
```

---

## AprilGrid 36h11 (7x11 Board - zu konfigurieren)
Das zweite Board ist ein **AprilGrid 36h11** mit 20mm Tags und 6mm Spacing.

**Parameter für dein AprilGrid:**
```
board_type: GRID
aruco_dictionary: t36h11  (oder DICT_APRILTAG_36H11)
board_rows: 11
board_cols: 7
marker_length_m: 0.020    # 20mm tag size
marker_spacing_m: 0.006   # 6mm spacing
min_detected_markers: 4
```

---

## Wie man die Parameter ändert:

### Option 1: ROS2 Parameter beim Start setzen
```bash
ros2 run roboter_tetris board_detection_node \
  --ros-args -p board_type:=GRID \
             -p aruco_dictionary:=t36h11 \
             -p board_rows:=11 \
             -p board_cols:=7 \
             -p marker_length_m:=0.020 \
             -p marker_spacing_m:=0.006
```

### Option 2: Parameter-YAML Datei erstellen
Erstelle `board_detection.yaml`:
```yaml
board_detection_node:
  ros__parameters:
    board_type: "GRID"
    aruco_dictionary: "t36h11"
    board_rows: 11
    board_cols: 7
    marker_length_m: 0.020
    marker_spacing_m: 0.006
    min_detected_markers: 4
    debug_enable: true
```

Dann starten:
```bash
ros2 launch roboter_tetris board_detection.launch.py config:=board_detection.yaml
```

### Option 3: Hardcoded in Launch-File
In der Launch-Datei `board_detection.launch.py` die Parameter setzen.

---

## Unterschied Charuco vs AprilGrid:

| Feature | Charuco | AprilGrid |
|---------|---------|-----------|
| Marker-Typ | ArUco Marker | AprilTag Marker |
| Board-Type | CHARUCO | GRID |
| Dictionary | DICT_*X*_* | t36h11, DICT_APRILTAG_* |
| Ecken-Interpolation | Ja (genauer) | Nein (nur Marker-Center) |
| Parameter | `marker_length_m + marker_spacing_m = square` | `marker_spacing_m = center-to-center distance` |

---

## Debugging:

Setze `debug_enable: true` um die erkannten Marker visuell zu sehen:
```bash
ros2 topic echo /board_detection/debug_image
```

Das sollte dir zeigen:
- **Charuco**: Grüne Punkte auf den interpolierten Board-Ecken
- **AprilGrid**: Grüne Marker mit den detektierten Tag-IDs
