# Extrinsische Kalibrierung & Test Drive (`roboter_tetris`)

Dieser Ordner enthält die Komponenten zur automatisierten extrinsischen **Eye-in-Hand-Kalibrierung** und zur Messung des **Förderband-Koordinatensystems (`conveyor_frame`)** für Roboter und Kameras.

Die Kalibrierung ist Voraussetzung für die präzise Objekterkennung und Greifaufgabe, läuft jedoch als eigenständige Modulo/AICA-Lifecycle-Komponente ab.

---

## 1. Systemübersicht & Komponenten

Das Kalibrier-Subsystem besteht aus vier Hauptkomponenten:

* **`board_detection.py` (`BoardDetection`)**: Erkennt ChArUco-Boards im Kamerabild und berechnet die Pose des Board-Ursprungs in der Kamera (`[tx, ty, tz, rx, ry, rz]`). Dient als führende Instanz für die physikalischen Board-Parameter.
* **`auto_calibration.py` (`AutoCalibration`)**: Steuert eine automatische Orbit-Trajektorie an, erfasst Aufnahmen, führt die Tsai-Hand-Eye-Kalibrierung aus und ermittelt das Förderband-Koordinatensystem (`conveyor_frame`).
* **`extrinsic_calibration.py`**: Mathematik-Bibliothek für `solve_eye_in_hand`, Quaternionen-Normierung und YAML/JSON-Export.
* **`test_drive.py` (`CalibrationTestDrive`)**: Validierungskomponente, die das Kamera-Fadenkreuz präzise über dem Board-Zentrum ausrichtet und eine Testfahrt entlang der Förderband-Y-Achse ausführt.

---

## 2. Parameter-Konfiguration & Vererbung

Um Redundanzen zu vermeiden, werden die physikalischen Board-Eigenschaften **nur einmal** in der Komponente `board_detection` (z. B. `board_detection_2`) festgelegt:

### Board-Detection Parameter (`board_detection.json`)
| Parameter | Typ | Standard | Beschreibung |
| --- | --- | --- | --- |
| `aruco_dictionary` | string | `"DICT_6x6_250"` | ArUco-Dictionary für das ChArUco-Board |
| `board_rows` | int | `5` | Anzahl der Zeilen (Checker-Felder in Y-Richtung) |
| `board_cols` | int | `7` | Anzahl der Spalten (Checker-Felder in X-Richtung) |
| `checker_size_mm` | double | `35.0` | Kantenlänge eines Schachfeld-Quadrat-Feldes in mm |
| `marker_size_mm` | double | `26.0` | Kantenlänge eines ArUco-Markers in mm |
| `min_detected_markers` | int | `4` | Mindestanzahl erkannter Marker für eine gültige Pose |

### Dynamische Parameterabfrage in AutoCalibration
`AutoCalibration` fragt beim Ausführen im Zustand `SOLVING` die Parameter `board_rows`, `board_cols` und `checker_size_mm` dynamisch über den ROS 2 Service `/{board_detection_node_name}/get_parameters` ab. Lokale Fallback-Parameter greifen automatisch, falls der Service nicht erreichbar ist.

---

## 3. Geometrische Konventionen & Conveyor Frame

### Board-Ursprung (Marker ID 0)
Der mathematische Ursprung $(0,0,0)$ des ChArUco-Boards liegt an der **äußersten linken oberen Ecke des ersten Markers (Marker ID 0)**.

### Conveyor Frame Transformation
Das Förderband-Koordinatensystem (`conveyor_frame`) wird wie folgt definiert:
* **$+Y$ (Flow)**: Entlang der Förderrichtung des Bands.
* **$+X$ (Width)**: Nach rechts in Förderrichtung gesehen.
* **$+Z$ (Height)**: Vertikal nach oben aus der Förderbandebene.

Da das ChArUco-Board spiegelbildlich im Arbeitsraum liegt, wird das Board-KS mit einer $180^\circ$-Rotation um die Y-Achse ($R_y(180^\circ)$) in das `conveyor_frame` überführt.

### Berechnung des Board-Zentrums im Conveyor Frame
Ausgehend vom gemessenen Ursprung (Ecke Marker ID 0) berechnet `AutoCalibration` das Board-Zentrum wie folgt:

$$\text{board\_width\_m} = \text{board\_cols} \cdot \text{checker\_size\_m}$$
$$\text{board\_height\_m} = \text{board\_rows} \cdot \text{checker\_size\_m}$$

$$\text{center}_x = \text{conveyor\_offset\_x\_m} - \frac{\text{board\_width\_m}}{2.0}$$
$$\text{center}_y = \text{conveyor\_offset\_y\_m} + \frac{\text{board\_height\_m}}{2.0}$$
$$\text{center}_z = \text{conveyor\_offset\_z\_m}$$

Dieses Zentrum wird zusammen mit der Hand-Eye-Matrix in der Kalibrierungsdatei (`/tmp/calibration.yaml`) abgespeichert.

---

## 4. Ablauf der Kalibrierung (`AutoCalibration`)

1. **Vorbereitung:** Kamera mit Fadenkreuz grob über dem Board-Zentrum ausrichten.
2. **Start:** Auslösen über AICA Studio Button / Service `start_calibration`.
3. **Orbit-Trajektorie:** Der Roboter fährt automatisch $N$ Wegpunkte auf einem Halbkugelsegment ab, schwenkt die Kamera auf das Board ein und sammelt gemittelte Bildsamples.
4. **Lösung (`SOLVING`):** 
   * Berechnung der Eye-in-Hand Transformation $T_{\text{ee\_robot\_cam}}$ via `solve_eye_in_hand`.
   * Berechnung von $T_{\text{robot\_conveyor}}$ und `board_center_conveyor_mm`.
   * Speichern der Ergebnisse nach `/tmp/calibration.yaml` und `/tmp/calibration.json`.

---

## 5. Validierung via Test Drive (`CalibrationTestDrive`)

`CalibrationTestDrive` liest sowohl $T_{\text{robot\_conveyor}}$ als auch die Eye-in-Hand Transformation $T_{\text{ee\_robot\_cam}}$ aus der generierten `calibration.yaml` ein:

* **Kamera-Fadenkreuz-Zentrierung (Phase 0 & 1):** 
  Der Roboter kompensiert den physischen Abstand von $100\text{ mm}$ zwischen Kamera-Optik und Flansch ($T_{\text{ee\_robot\_cam}}$). Dadurch fährt **das Fadenkreuz der Kamera im Debug Image** exakt zentriert über die Mitte des ChArUco-Boards.
* **Trajektorie (Phase 2 & 3):** 
  Der Roboter fährt $+200\text{ mm}$ entlang der $Y$-Achse des Förderbands vorwärts und anschließend wieder zurück auf die Startposition.
* **Erfolgskriterium:** 
  Das Fadenkreuz bleibt beim Zentrieren auf der Board-Mitte stehen, und der Roboter bewegt sich visuell exakt parallel zur Förderbandkante.

---

## 6. Kalibrierungsdatei Format (`calibration.yaml`)

Auszug der generierten Kalibrierungsstruktur:

```yaml
transformations:
  T_ee_robot_cam:
    source_frame: robot_cam
    target_frame: end_effector
    homogeneous_matrix: [...]
  T_robot_conveyor:
    source_frame: conveyor_frame
    target_frame: robot_base
    homogeneous_matrix: [...]

board_center_conveyor_mm:
  x: -252.5
  y: 330.5
  z: 0.0

validation:
  position_rmse_mm: 1.25