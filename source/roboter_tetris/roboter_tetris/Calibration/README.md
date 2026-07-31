# Extrinsische Kalibrierung

Dieser Ordner ist bewusst von der Objekterkennung, dem Tracking und der Greifaufgabe getrennt. Er enthält die geometrische Beziehung zwischen der fest montierten Basis-Kamera und dem Roboter. Die Kalibrierung ist eine Voraussetzung für die Aufgabe, aber nicht Teil ihrer Laufzeitlogik.

## Quelle der Kalibrierwerte

`calibration.json` ist die versionierbare Ablage für die Kamera-zu-Roboter-Transformation. Sie beschreibt die aktive Konvention eindeutig:

```
p_robot = T_robot_camera @ p_camera
```

`source_frame` ist der Kamera-Frame, `target_frame` der Roboter-Base-Frame. Translationen stehen in Metern. Die Orientierung ist zusätzlich als Roll, Pitch und Yaw in Grad abgelegt und wird mit `Rz(yaw) @ Ry(pitch) @ Rx(roll)` zur Rotationsmatrix aufgebaut. Die Matrix ist zeilenweise gespeichert.

Die Datei enthält derzeit die bisherigen Aufbauwerte und ist ausdrücklich mit `legacy_initial_values` markiert. Sie ist erst nach einer dokumentierten Messung als validierte Kalibrierung zu verwenden.

## Vorgehen

1. Roboter und Kamera mechanisch fixieren; den Roboter-Base-Frame als Referenz prüfen.
2. Mehrere eindeutig messbare Punkte oder ein geeignetes Kalibriertarget im Arbeitsraum erfassen. Die Punkte müssen jeweils im Kamera- und Base-Frame vorliegen.
3. Die starre Transformation `T_robot_camera` bestimmen und in der JSON-Datei sowohl als Translation/ZYX-Eulerwinkel als auch als 4×4-Matrix eintragen.
4. Unter `validation` Messdatum, Methode, Anzahl der Punkte und Positions-/Orientierungsfehler dokumentieren. `status` erst nach erfolgreicher Prüfung auf `validated` setzen.
5. Die sechs Werte in AICA in der Komponente **BaseCam** (`cal_x`, `cal_y`, `cal_z`, `cal_roll`, `cal_pitch`, `cal_yaw`) übernehmen. Diese Laufzeitparameter entsprechen exakt den JSON-Feldern und bleiben für ein kontrolliertes Fein-Tuning verfügbar.
6. Mit unabhängigen Prüfpunkten validieren. Ein Punkt darf nicht zugleich zur Bestimmung und alleinigen Bewertung der Transformation dienen.

## Abgrenzung

Intrinsik, Tiefenskalierung, Förderbandhöhe und affine Korrekturen sind keine Extrinsik und werden hier nicht verwaltet. Die Extrinsik wird neu bestimmt, wenn Kamera, Roboterbase oder deren starre Verbindung bewegt wurde.

## Änderungsregel

Keine Werte ohne Messprotokoll überschreiben. Bei jeder Änderung `measured_at`, `operator`, `method`, Fehlerwerte und eine kurze Notiz ergänzen. Dadurch bleibt nachvollziehbar, welcher Aufbau mit welcher AICA-Konfiguration betrieben wurde.


## ChArUco-Board-Erkennung in AICA

`board_detection.py` ist die AICA-Komponente für die reine Erkennung von **ChArUco-Boards**. Sie unterstützt ausschließlich ChArUco-Boards. Für jedes verwendete Board werden die folgenden dynamischen AICA-Parameter gesetzt:

| Parameter | Bedeutung |
| --- | --- |
| `aruco_dictionary` | ArUco-Dictionary, z. B. `DICT_5X5_250` |
| `board_rows` / `board_cols` | Anzahl der Marker in Y- bzw. X-Richtung |
| `checker_size_m` | Kantenlänge eines Schachbrettfelds in Metern |
| `marker_size_m` | Kantenlänge eines ArUco-Markers in Metern |

`checker_size_m` muss größer als `marker_size_m` sein. Der für OpenCV benötigte Abstand wird intern als `checker_size_m - marker_size_m` berechnet.

## Pose-Schätzung

Wenn `color_camera_info` gültige Kamera-Intrinsics und Verzerrungskoeffizienten liefert, bestimmt die Komponente zusätzlich die Pose mit `cv2.aruco.estimatePoseCharucoBoard`. Die Ausgabe `board_pose` ist `[tx, ty, tz, rx, ry, rz]`: Translation in Metern und Rodrigues-Rotationsvektor in Radiant für die Transformation **Board → Kamera**. Der Ursprung des Board-Koordinatensystems liegt an der ersten Ecke des ChArUco-Boards. `has_pose` ist nur bei gültiger Schätzung gesetzt. Bei aktiviertem Debug wird ein Achsenkreuz mit drei Checker-Feldlängen gezeichnet.
