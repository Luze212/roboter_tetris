# Robot-Kamera-Hand-Auge-Kalibrierung

Dieses Paket ist das zweite, unabhängige Kalibrierverfahren im Projekt. Es kalibriert
nicht die laufende Pick-Anwendung und überschreibt keine Daten des bestehenden
Verfahrens `BaseCamCalibration`.

| Verfahren | AICA-Komponente | Ergebnisdatei | Verwendung |
|---|---|---|---|
| Basiskamera-Kalibrierung | `BaseCamCalibration` | `Extrinsics/base_cam_extrinsics.json` | Bestehende `base_cam`-Pick-Anwendung |
| Robot-Kamera-Hand-Auge-Kalibrierung | `RobotCamHandEyeCalibration` | `/data/robot_cam_handeye_calibration.json` | Eigenständige Kalibrierung und Diagnose; optional als Basiskamera-Extrinsik für `BaseCam` auswählbar |
| Robot-Kamera-Hand-Auge-Kalibrierung mit drei Board-Lagen | `RobotCamHandEyeThreeBoardPositions` | `/data/robot_cam_handeye_3positions_calibration.json` | Drei getrennte Messabschnitte und eine gemeinsame, separat gespeicherte Basiskamera-Pose |

## Komponenten

- `RobotCamBoardDetection`: ChArUco-Erkennung für die RealSense am Flansch.
- `StaticBaseCamBoardDetection`: ChArUco-Erkennung für die fest montierte
  Basiskamera.
- `RobotCamHandEyeCalibration`: führt die Orbit-Bewegung aus, löst die
  Hand-Auge-Transformation und speichert ihr Ergebnis.
- `RobotCamHandEyeThreeBoardPositions`: wiederholt den Orbit an drei nacheinander
  vom Bediener umgesetzten Board-Lagen. Nach jedem der ersten beiden Orbits fährt
  der Roboter zur Startpose zurück und wartet dort auf eine ausdrückliche
  Fortsetzung. Erst nach der dritten Lage wird eine Ergebnisdatei geschrieben.
- `RobotCamHandEyeTestDrive`: prüft das Ergebnis mit einer Fahrt entlang der
  diagnostischen Conveyor-Y-Achse.

Die Komponenten wurden absichtlich nach Kamera-Rolle benannt. Dadurch können die
beiden Board-Beobachtungen im AICA-Graphen nicht verwechselt werden.

### Variante mit drei Board-Lagen

Die AICA-Vorlage liegt unter
`docs/uebersicht/anwendung-calibration-tobi-3-board-lagen.yaml`. Sie ist eine
eigene Anwendung: Die alte Einzellauf-Anwendung bleibt unverändert und beide
Komponenten sind im selben gebauten Paket verfügbar. In der neuen Anwendung
läuft nur die Drei-Lagen-Komponente am Bewegungsziel des Attractors.

Aktivieren der neuen Komponente fährt den Roboter **nicht**. Der Dienst
`start_three_board_calibration` startet Lage 1. Erst wenn der Roboter zur
Startpose zurückgekehrt ist, kann das Board für Lage 2 umgesetzt werden; der
Log und `waiting_for_board` zeigen die Pause an. Mit
`continue_after_board_move` beginnt Lage 2, später Lage 3. Beide Dienstaufrufe
lösen Roboterbewegung aus. Während einer Lage darf das Board nicht bewegt
werden; zwischen den Lagen muss es in beiden Kameras sichtbar und nachweislich
anders positioniert sein. Bei fehlender oder alter Flanschpose, fehlenden
Board-Erkennungen, instabilem Board oder zu stark voneinander abweichenden
Einzelergebnissen endet die Sitzung ohne neue Ergebnisdatei.

Die drei Abschnitte werden **einzeln** gelöst. Alle 27 Samples in den Solver des
Einzellaufs zu geben wäre falsch: dieser setzt eine einzige konstante Board-Pose
voraus. Nach den drei Einzelprüfungen werden die drei Flansch-Kamera-Posen und
die drei Basiskamera-Posen mit gleichen Gewichten gemittelt (Position linear,
Rotation auf SO(3)). Die größte Differenz zwischen den Einzelresultaten und
deren Qualitätswerte stehen im JSON unter `multi_board`. Die Datei enthält
weiterhin die BaseCam-kompatiblen Felder `schema`, `version`, `matrix` und `cal`.
Die älteren `validation`-RMSE-Felder enthalten bei dieser Variante den jeweils
schlechtesten der drei Abschnitte; sie sind kein aus bewegtem Board berechneter
Gesamt-RMSE.
Ein lokaler Link `robot_cam_handeye_3positions_calibration.json` im Projektordner
zeigt nach einem erfolgreichen Lauf auf die Datei im AICA-Datenvolume. Der Link
selbst wird nicht versioniert. Wiederholte Läufe sichern die vorherige
Ergebnisdatei als `_vorher.json`; geprüfte Ergebnisse sollten zusätzlich mit
Zeitstempel in `calibration_history/` übernommen werden.

Es gibt bei drei Board-Lagen keine gemeinsame Board- oder Conveyor-Pose; daher
enthält die neue Ergebnisdatei keine `T_world_conveyor`. Die bestehende
`RobotCamHandEyeTestDrive` darf damit nicht verwendet werden. Auch diese neue
Basiskamera-Pose wird erst im Pick-System wirksam, wenn dort `BaseCam.calibration_file`
bewusst auf `/data/robot_cam_handeye_3positions_calibration.json` gesetzt und
`BaseCam` neu aktiviert wird. Bei der separaten Fusionskomponente muss stattdessen
deren Parameter `robot_cam_file` auf diesen Pfad gesetzt werden.

## Bezugssysteme und Datenvertrag

- `world` ist `ur_base_link` am Roboterfuß.
- `robot_flange_state` ist die gemessene Flanschpose `world_T_ur_tool0` aus dem
  Hardware-State. Sie darf keine TCP- oder Greifpunktpose sein.
- Das ChArUco-Board bleibt während **eines** Kalibrierlaufs ruhig liegen.
- Die Robot-Kamera hängt am Flansch; ihre Transformation wird als
  `T_flange_robot_cam` gespeichert.
- Die berechnete statische Basiskamera-Pose ist ein Ergebnis dieser Methode. Sie
  wird nicht automatisch in `base_cam` übernommen und nicht mit
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

Wenn die statische Basiskamera während des Laufs beobachtet wurde, enthält
dieselbe Datei zusätzlich die von `BaseCam` lesbaren Felder `schema`, `version`
und `matrix`. `matrix` ist ausschließlich `T_world_base_static_cam` im
**Farbkamera-Optik-Frame**; die bewegte Roboterkamera-Pose und der diagnostische
Conveyor-Frame werden dafür nicht verwendet. Die vorhandenen Hand-Auge-Felder
(`schema_version: 3`, `transformations` usw.) bleiben für die Testfahrt erhalten.

Die Pick-Anwendung bleibt standardmäßig bei
`Extrinsics/base_cam_extrinsics.json`. Um die alternative Schätzung bewusst zu
erproben, in der Komponente `BaseCam` den Parameter **Kalibrierdatei**
(`calibration_file`) auf `/data/robot_cam_handeye_calibration.json` setzen und
`BaseCam` neu aktivieren. Im Log muss `robot_cam_handeye_charuco` als geladene
Methode erscheinen. Ein leerer, fehlender oder ungültiger Pfad führt stattdessen
zu den `cal_*`-Rückfallwerten. Der Wechsel ist keine automatische Kombination
beider Kalibrierverfahren; die bestehende Datei bleibt unverändert.
