# Robot-Kamera-Hand-Auge-Kalibrierung

Dieses Paket ist das zweite, unabhängige Kalibrierverfahren im Projekt. Es kalibriert
nicht die laufende Pick-Anwendung und überschreibt keine Daten des bestehenden
Verfahrens `BaseCamCalibration`.

| Verfahren | AICA-Komponente | Aktive Ergebnisdatei | Unveränderliches Archiv | Verwendung |
|---|---|---|---|
| Basiskamera-Kalibrierung | `BaseCamCalibration` | `Extrinsics/base_cam_extrinsics.json` | `/data/calibration_archive/base_cam/` | Bestehende `base_cam`-Pick-Anwendung |
| Robot-Kamera-Hand-Auge-Kalibrierung | `RobotCamHandEyeCalibration` | `/data/robot_cam_handeye_calibration.json` | `/data/calibration_archive/robot_cam_handeye/` | Eigenständige Kalibrierung und Diagnose; optional als Basiskamera-Extrinsik für `BaseCam` auswählbar |
| Robot-Kamera-Hand-Auge-Kalibrierung mit mehreren Board-Lagen | `RobotCamHandEyeThreeBoardPositions` | `/data/robot_cam_handeye_3positions_calibration.json` | `/data/calibration_archive/robot_cam_handeye_multi_board/` | Konfigurierbare, getrennt gelöste Messabschnitte und eine gemeinsam gemittelte Basiskamera-Pose |

## Komponenten

- `RobotCamBoardDetection`: ChArUco-Erkennung für die RealSense am Flansch.
- `StaticBaseCamBoardDetection`: ChArUco-Erkennung für die fest montierte
  Basiskamera.
- `RobotCamHandEyeCalibration`: führt die Orbit-Bewegung aus, löst die
  Hand-Auge-Transformation und speichert ihr Ergebnis.
- `RobotCamHandEyeThreeBoardPositions`: wiederholt den Orbit an einer konfigurierbaren
  Zahl nacheinander vom Bediener umgesetzter Board-Lagen (Standard: drei). Nach jedem
  Orbit außer dem letzten fährt
  der Roboter zur Startpose zurück und wartet dort auf eine ausdrückliche
  Fortsetzung. Vor jedem Orbit werden zehn stabile Basiskamera-Aufnahmen an der
  Startpose gemittelt; während des Orbits ist eine Basiskamera-Sicht auf das
  Board nicht erforderlich. Erst nach der gewählten letzten Lage wird eine Ergebnisdatei
  geschrieben.
- `RobotCamHandEyeTestDrive`: prüft das Ergebnis mit einer Fahrt entlang der
  diagnostischen Conveyor-Y-Achse.

Die Komponenten wurden absichtlich nach Kamera-Rolle benannt. Dadurch können die
beiden Board-Beobachtungen im AICA-Graphen nicht verwechselt werden.

### Variante mit mehreren Board-Lagen

Die AICA-Vorlage liegt unter
`docs/uebersicht/anwendung-calibration-tobi-3-board-lagen.yaml`. Sie ist eine
eigene Anwendung: Die alte Einzellauf-Anwendung bleibt unverändert und beide
Komponenten sind im selben gebauten Paket verfügbar. In der neuen Anwendung
läuft nur die Drei-Lagen-Komponente am Bewegungsziel des Attractors.

### Kalibrierstartpose im AICA-Systembild

Vor einem Drei-Lagen-Lauf wird der Roboter bewusst in die Kalibrierstartpose
gefahren. Dazu muss der Bediener im Systembild noch selbst einen
`TfToSignal`-Block (Anzeige: **Frame to Signal**) und einen Frame
`Kalibrierstart` ergänzen. Der Ausgang `pose` wird auf
`/auto_calibration/target_pose` gelegt, also auf denselben Eingang des
`Signal Point Attractor`, den die Drei-Lagen-Komponente später verwendet.

Der Block darf nur zum Anfahren der Startpose geladen sein. Vor dem Laden oder
Starten von `RobotCamHandEyeThreeBoardPositions` ist er wieder zu entladen,
damit nicht zwei Komponenten gleichzeitig Zielposen auf
`/auto_calibration/target_pose` veröffentlichen.

Als bisherige, erprobte Ausgangspose aus dem AICA-Systembild **Calibration**
ist der Frame `Target` in `world` gespeichert:

```text
Position [m]:     x=-0.563646, y=0.701042, z=0.572258
Orientierung xyzw: x=0.699212, y=0.714887, z=0.006226, w=-0.000126
```

Diese Pose liegt absichtlich außerhalb des Pick-Arbeitsraums aus `main`
(`y_max = 0.48 m`). Sie ist daher Ausgangspunkt für einen getrennt
festzulegenden Kalibrier-Arbeitsraum und darf nicht durch die Pick-Grenzen
ersetzt werden.

### Vorläufiger Kalibrier-Arbeitsraum

Für die automatische Anfahrt nach dem Umsetzen des Boards ist der folgende
Flansch-Arbeitsraum in `world` vorgesehen:

| Achse | Untergrenze | Obergrenze |
|---|---:|---:|
| x | -1.000 m | -0.500 m |
| y | +0.500 m | +0.760 m |
| z | +0.310 m | +0.600 m |

Die bestehende Startpose `Target` bleibt unverändert. Der gegenwärtige Orbit
um diese Pose bleibt ebenfalls unverändert (Mittelpunkt plus 50 mm Radius).
Diese Grenzen sind bis zur Prüfung am realen Aufbau ein **Planungswert** und
werden noch nicht durch die Komponente erzwungen.

Mit `y_max = +0.760 m` liegt der äußerste bestehende Orbit-Wegpunkt
(`y ≈ +0.751 m`) innerhalb der geplanten Grenze. Der Orbit bleibt unverändert.

### Automatische Anfahrt für Board-Lage 2 und 3

Nach der ersten erfolgreich gelösten Lage speichert die Komponente deren
vorläufige Transformationen `world_T_base_cam` und `flange_T_robot_cam` sowie
die damals bewährte relative Sicht `robot_cam_T_board`. Nach dem Umsetzen wird
die neue Boardpose aus zehn Basiskamera-Beobachtungen an der Startpose gemittelt.
Die Komponente überträgt die bewährte relative Sicht auf diese neue Boardpose
und berechnet daraus eine Flansch-Annäherungspose sowie die spätere Sichtpose.

Die automatische Anfahrt ist bewusst nur eine grobe Nachführung: Die
Basiskamera misst die Änderung der Board-Position zu Lage 1. Dieser Versatz
wird mit der vorläufigen BaseCam-Rotation nach `world` übertragen und nur auf
X/Y der bewährten Kalibrierstartpose addiert. Z und Flanschorientierung bleiben
identisch zur Startpose. Damit bewegt sich der Roboter horizontal aus dem Bild
der Basiskamera, ohne eine ungetestete Höhe anzufahren. Die Zielpose und alle
Orbit-Wegpunkte müssen im Kalibrier-Arbeitsraum liegen; der rechteckige Bereich
enthält auch jede lineare Zwischenposition. Erst an der Sichtpose muss die
Roboterkamera das Board wieder selbst erkennen. Sie bestimmt damit die
Feinausrichtung für den Orbit; gelingt das nicht, endet der Lauf ohne Orbit.

`auto_approach_enabled` ist standardmäßig `true`. Vor der ersten Verwendung
müssen die Grenzen am Aufbau abgefahren werden. Mit `false` bleibt der bisherige Ablauf unverändert: Das Board
muss an der Startpose bereits für beide Kameras sichtbar sein. Mit `true`
reicht beim Fortsetzen eine frische Basiskamera-Erkennung; die Roboterkamera
wird erst an der berechneten Sichtpose benötigt.

#### Plan vor der endgültigen Implementierung

1. **Extrempositionen abfahren:** Bei eingeschaltetem Roboter alle sechs
   Flächen des obigen Quaders sowie die Startpose mit langsamer Handführung
   prüfen. Dabei insbesondere Singularitäten, Kollisionen mit der festen
   Basiskamera, Kabelzug und freie Sicht beider Kameras dokumentieren.
2. **Board-Lagen festlegen:** Die gewählte Zahl konkreter, stabiler Board-Positionen innerhalb
   des geprüften Bereichs auswählen. Die Basiskamera muss jede Lage sehen; die
   Roboterkamera muss sie nach der automatischen Anfahrt sehen können.
3. **Automatische Anfahrt implementiert, physisch abnehmen:** Nach Lage 1
   genügt für Lage 2 und 3 zunächst die neue Basiskamera-Beobachtung. Die
   Komponente berechnet Annäherungs- und Sichtpose aus der vorläufigen
   Kalibrierung, prüft beide gegen den Kalibrier-Arbeitsraum und verlangt an
   der Sichtpose wieder die Roboterkamera-Erkennung. Vor der ersten Bewegung
   den Arbeitsraum am Aufbau abfahren.
4. **Abbruchfälle absichern:** Bei fehlender Basiskamera-Beobachtung,
   Zielpose außerhalb des geprüften Bereichs, Zeitüberschreitung oder fehlender
   Roboterkamera-Erkennung bleibt der Roboter stehen; ein Orbit startet nicht.
5. **Automatisierte Tests ergänzen:** Transformation, Grenzprüfung und die
   Zustandsfolge `Warten → Anfahrt → Wiedererkennung → Orbit` ohne Hardware
   testen.
6. **AICA-Systembild vervollständigen:** Den beschriebenen Frame
   `Kalibrierstart`, `Frame to Signal` sowie getrennte Laden-/Entladen-Buttons
   selbst ergänzen. Vor dem Drei-Lagen-Start muss `Frame to Signal` entladen
   sein.
7. **Abnahme am Aufbau:** Erst einen manuellen Mehrlagen-Lauf
   vergleichen, danach die neue Anfahrt mit kleinen Board-Versätzen und erst
   zuletzt mit der gewählten Anzahl Lagen prüfen. Ergebnisdatei, Einzel-RMSE und
   Streuung der Lösungen dokumentieren.

Aktivieren der neuen Komponente fährt den Roboter **nicht**. Der Dienst
`start_three_board_calibration` startet Lage 1. Erst wenn der Roboter zur
Startpose zurückgekehrt ist, kann das Board für die nächste Lage umgesetzt werden; der
Log und `waiting_for_board` zeigen die Pause an. Mit
`continue_after_board_move` beginnt zunächst die Basiskamera-Messreihe an der
Startpose und danach die nächste Lage. Beide Dienstaufrufe können somit
nach der kurzen Messreihe Roboterbewegung auslösen. Während einer Lage darf das Board nicht bewegt
werden; zwischen den Lagen muss es in beiden Kameras sichtbar und nachweislich
anders positioniert sein. Bei fehlender oder alter Flanschpose, fehlenden
Board-Erkennungen, instabilem Board oder zu stark voneinander abweichenden
Einzelergebnissen endet die Sitzung ohne neue Ergebnisdatei.

Die Abschnitte werden **einzeln** gelöst. Alle Samples in den Solver des
Einzellaufs zu geben wäre falsch: dieser setzt eine einzige konstante Board-Pose
voraus. Nach den Einzelprüfungen werden die Flansch-Kamera-Posen und
die Basiskamera-Posen mit gleichen Gewichten gemittelt (Position linear,
Rotation auf SO(3)). Die größte Differenz zwischen den Einzelresultaten und
deren Qualitätswerte stehen im JSON unter `multi_board`. Die Datei enthält
weiterhin die BaseCam-kompatiblen Felder `schema`, `version`, `matrix` und `cal`.
Die älteren `validation`-RMSE-Felder enthalten bei dieser Variante den jeweils
schlechtesten Abschnitt; sie sind kein aus bewegtem Board berechneter
Gesamt-RMSE.

Die Anzahl wird über `board_position_count` vor dem Start festgelegt (1 bis 6,
Standard 3) und für die laufende Sitzung eingefroren. Für jede mögliche Lage
gibt es das Predicate `board_position_1_complete` bis
`board_position_6_complete`. Nur die bis zur gewählten Anzahl benötigten
Predicates können auf `true` wechseln; sie bedeuten, dass die jeweilige Lage
erfolgreich gelöst und in die Ergebnisaggregation aufgenommen wurde.
Die aktive Mehrlagen-Datei liegt unter
`/data/robot_cam_handeye_3positions_calibration.json`. Jeder erfolgreiche Lauf
legt zusätzlich eine unveränderliche Zeitstempelkopie unter
`/data/calibration_archive/robot_cam_handeye_multi_board/` ab. Die vollständige
Archivstruktur steht in
[`docs/uebersicht/kalibrierdateien-ablage.md`](../../../../docs/uebersicht/kalibrierdateien-ablage.md).

Bei mehreren Board-Lagen gibt es keine gemeinsame Board- oder Conveyor-Pose; daher
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
`robot_cam_handeye_file_path`. Beim Einzellauf ist sein Standard
`/data/robot_cam_handeye_calibration.json`, bei mehreren Board-Lagen
`/data/robot_cam_handeye_3positions_calibration.json`. Die aktive Datei wird
nach einem erfolgreichen Lauf atomar ersetzt und zusätzlich unveränderlich im
zugehörigen `/data/calibration_archive/`-Verzeichnis gespeichert.

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
