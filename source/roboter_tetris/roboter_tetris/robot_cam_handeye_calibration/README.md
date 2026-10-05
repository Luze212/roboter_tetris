# Robot-Kamera-Hand-Auge-Kalibrierung

Dieses Paket ist das zweite, unabhängige Kalibrierverfahren im Projekt. Es kalibriert
nicht die laufende Pick-Anwendung und überschreibt keine Daten des bestehenden
Verfahrens `BaseCamCalibration`.

| Verfahren | AICA-Komponente | Ergebnisdatei | Verwendung |
|---|---|---|---|
| Basiskamera-Kalibrierung | `BaseCamCalibration` | `Extrinsics/base_cam_extrinsics.json` | Bestehende `base_cam`-Pick-Anwendung |
| Robot-Kamera-Hand-Auge-Kalibrierung | `RobotCamHandEyeCalibration` | `/data/robot_cam_handeye_calibration.json` | Eigenständige Kalibrierung und Diagnose |

## Komponenten

- `RobotCamBoardDetection`: ChArUco-Erkennung für die RealSense am Flansch.
- `StaticBaseCamBoardDetection`: ChArUco-Erkennung für die fest montierte
  Basiskamera.
- `RobotCamHandEyeCalibration`: führt die Orbit-Bewegung aus, löst die
  Hand-Auge-Transformation und speichert ihr Ergebnis.
- `RobotCamHandEyeTestDrive`: prüft das Ergebnis mit einer Fahrt entlang der
  diagnostischen Conveyor-Y-Achse.

Die Komponenten wurden absichtlich nach Kamera-Rolle benannt. Dadurch können die
beiden Board-Beobachtungen im AICA-Graphen nicht verwechselt werden.

## Bezugssysteme und Datenvertrag

- `world` ist `ur_base_link` am Roboterfuß.
- `robot_flange_state` ist die gemessene Flanschpose `world_T_ur_tool0` aus dem
  Hardware-State. Sie darf keine TCP- oder Greifpunktpose sein.
- Das ChArUco-Board bleibt während **eines** Kalibrierlaufs ruhig liegen.
- Die Robot-Kamera hängt am Flansch; ihre Transformation wird als
  `T_flange_robot_cam` gespeichert.
- Die berechnete statische Basiskamera-Pose ist ein Ergebnis dieser Methode. Sie
  wird noch nicht in `base_cam` übernommen und nicht mit
  `Extrinsics/base_cam_extrinsics.json` kombiniert.
- `T_world_conveyor` dient nur der Testfahrt und Diagnose. Die Hauptanwendung
  bleibt im globalen Frame `world`.

## AICA-Verbindungen

```text
Robot-Kamera RGB/Info/Tiefe ──> RobotCamBoardDetection
  robot_cam_board_pose ─────────┐
  robot_cam_board_observation_id │
  robot_cam_board_depth           ├─> RobotCamHandEyeCalibration
  robot_cam_board_geometry ───────┘                    │
                                                       ├─> Zielpose Flansch ──> Attractor
Statische Basiskamera RGB/Info/Tiefe -> StaticBaseCamBoardDetection
  static_base_cam_board_pose ──────────────────────────> RobotCamHandEyeCalibration
  static_base_cam_board_observation_id ────────────────> RobotCamHandEyeCalibration

robot_state_broadcaster/cartesian_state ───────────────> robot_flange_state
```

`RobotCamHandEyeTestDrive` erhält ebenfalls `robot_flange_state` und
`robot_cam_board_geometry`; seine Ausgabe
`robot_cam_handeye_test_target_flange_pose` wird an den Attractor angeschlossen.

## Bedienung und Ergebnisdatei

Beide bewegenden Komponenten können beim Aktivieren starten; zusätzlich stellen sie
die Services `start_robot_cam_handeye_calibration` und
`start_robot_cam_handeye_test_drive` bereit. Das ist eine Roboterbewegung und wird
nur in einer eigenen Kalibrieranwendung verwendet.

Der Parameter heißt in beiden Komponenten
`robot_cam_handeye_file_path` und muss auf
`/data/robot_cam_handeye_calibration.json` stehen. Die Datei ist die einzige
Laufzeitquelle für dieses Verfahren. Nach einem erfolgreichen Lauf wird sie als
JSON atomar ersetzt. Für die Versionshistorie wird sie anschließend in
`calibration_history/` kopiert und dort mit dem Quellcode versioniert.
