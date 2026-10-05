# Projektkontext Robotetris

Aufgabe, Ergebnis, Aufbau und Inhalt des Pakets — der Einstieg vor den technischen
Dokumenten.

---

## 1. Was das Projekt leistet

Ein **UR10e** greift farbige Klötze von einem laufenden Förderband — **im Lauf, ohne
dass das Band angehalten wird** — und legt sie in einer Kiste neben dem Band ab. Die
Klötze werden am Bandanfang von Hand aufgelegt: stehend, liegend, flach oder schräg.
Eine feste Kamera über dem Bandanfang erkennt sie; daraus werden Position und
Geschwindigkeit geschätzt, der nächste erreichbare Klotz gewählt und die Bewegung
über die AICA-Bausteine Signal Point Attractor und IK Velocity Controller geführt.

### Die Projektziele und ihr Ergebnis

| Nr. | Ziel | Umsetzung | Ergebnis am Aufbau |
|---|---|---|---|
| 1 | Ansteuerung des UR10e mit AICA | Zielpose → Signal Point Attractor → IK Velocity Controller | erfüllt |
| 2 | Schnelles Kalibrierungsverfahren | automatische Kalibrierung der Basiskamera: Der Roboter hält ein AprilGrid ins Bild (`base_cam_calibration`) | etwa 4 min je Lauf, über drei Tage auf unter 1 mm wiederholbar; Greiflauf 7 von 7, so gut wie die Handkalibrierung, die in Kraft bleibt |
| 3 | Geschwindigkeitsschätzung und Positionsberechnung | `base_cam` (Position), `vectoring` (Geschwindigkeit je Klotz und für das Band) | Position gegen den Roboter höchstens 6 mm; Band geschätzt −127,9 mm/s gegen 125–133 mm/s per Stoppuhr |
| 4 | Priorisierung und Bahnplanung für das kontrollierte Greifen | `priority_handler` (Auswahl, Erreichbarkeit, Greifebene); `object_follower` mit den AICA-Bausteinen | Griff im Lauf mit rund 1 mm Längsfehler; bei dichter Folge etwa ein Klotz je 7 s; flache (ab 20 mm) und gedrehte Klötze; Dauerlauf zuverlässig |

**Zusätzlich:** den Prozess nachvollziehbar darstellen — `data_tracker` (Klotzliste
ohne Rückwirkung auf den Regelpfad) und `interface_streamer` (Übersichtsbild für
RViz).

Zwei Punkte der Aufgabenstellung:

- **Die Bandgeschwindigkeit wird geschätzt, nicht kalibriert.** Ziel 3 verlangt ein
  Verfahren, das sie zur Laufzeit aus den Bilddaten bestimmt.
- **Die Bahnplanung über die AICA-Bausteine** gilt als Lösung der Vorgabe.

Der Kern ist der **Pick im Lauf** — Ziele 3 und 4 zusammen.

## 2. Der Aufbau

| | |
|---|---|
| Roboter | UR10e direkt neben dem Band, etwa auf einem Drittel vom Bandende aus |
| Greifer | Robotiq 2F-140 über USB/Modbus (nicht als ros2_control-Hardware). 3D-Druck-Aufsätze mit Gummi-Grippmatte, Greiffläche 20 mm hoch × 15 mm breit, Öffnungsweite 127 mm. Nutzlast in der UR-Installation 1,3 kg, Schwerpunkt 12 / 24 / 45 mm |
| Basiskamera | RealSense L515 (`serial_no f1370107`) senkrecht über dem Bandanfang, auf einem beweglichen Gestell — daher die wiederholbare Kalibrierung |
| Roboterkamera | RealSense D435i am Flansch; nicht in der Pick-Anwendung. Die getrennte `RobotCamHandEyeCalibration` dient in `calibrateNpick` nur zur Kalibrierung. |
| Band | grün-türkis, konstante Geschwindigkeit ≈ 0,13 m/s, nicht einstellbar, Lauf entlang −y |
| Klötze | rechtwinklig, 25 bis 100 mm Kante, rot, blau, weiß; 3D-gedruckt, Oberseite matt, Seitenflächen spiegelnd |
| Ablage | Pose in der Luft über einer Auffangkiste neben dem Band; der Klotz fällt hinein |
| Rechner | Intel Core 3 100U ohne NVIDIA-Grafik; AICA `v2.0.5-jazzy`, Hardware-Interface 500 Hz |

Die Klötze laufen frei mit dem Band. Ein Klotz kann beim Aufsetzen umkippen; bis er
gleichmäßig läuft, gilt er als einschwingend. Herunterfallen gibt es nur am Bandende,
wenn ein Klotz nicht gegriffen wurde.

Lage und Maße in Zahlen: `einrichtung-projektanwendung.md` §8.

**Drei Maße unter dem Flansch, die man nicht verwechseln darf:** Der TCP der
UR-Steuerung liegt 215 mm unter dem Flansch und dient nur der Umrechnung fremder
TCP-Werte. Die Kette regelt den **Flansch**; der Follower braucht Flansch → Griffpunkt
**235 mm** (`flange_to_grip_point_m`); die geschlossene Backenspitze liegt 245 mm unter
dem Flansch (`entscheidungen.md` §4.2).

## 3. Das Vorgängerprojekt

Eine Vorgängergruppe arbeitete am selben Aufbau, mit demselben Greifer und Band (C++
und ZeroMQ statt AICA). Sie hat die Klötze erkannt und verfolgt — ihre C++-Erkennung
ist die Grundlage der heutigen `base_cam`, nach Python übertragen —, aber **nicht im
Lauf gegriffen**: Der Roboter wartete an einer festen Pickposition am Bandende,
rechnete eine Ankunftszeit aus und griff ohne Rückkopplung zu; eine Zielauswahl gab es
nicht. Für `vectoring`, `priority_handler` und die Regelung im `object_follower` gab es
kein Vorbild. Übernommen wurden die Erkennung und einzelne Messwerte des Aufbaus; ihre
Kamerakalibrierung stand im UR-Rahmen `base` und wurde ersetzt.

## 4. Das Paket

`roboter_tetris` ist ein AICA-Komponentenpaket in Python, gebaut über
`aica-package.toml`.

| Komponente | Aufgabe |
|---|---|
| `base_cam` | erkennt die Klötze im Bild der Basiskamera, Lage in `world` |
| `vectoring` | schätzt Geschwindigkeit je Klotz und für das Band, führt Klötze hinter dem Bild weiter |
| `priority_handler` | wählt den dringendsten erreichbaren Klotz, rechnet die Greifebene |
| `object_follower` | fährt an, folgt, senkt ab, greift mitfahrend, legt ab |
| `robotiq_gripper` | bedient den Greifer, meldet „Bewegung fertig“ und „Objekt gegriffen“ |
| `data_tracker`, `interface_streamer` | Klotzliste und Übersichtsbild |
| `base_cam_calibration` | kalibriert die Basiskamera mit dem Roboter; eigene Anwendung |
| `true_signal`, `toggle_signal` | Schaltsignale für die Kalibrieranwendung (Greifer zu / auf) |

Jede Komponente ist eine dünne Schale um ein Logikmodul ohne ROS; die Signalformate
S1–S10 stehen in `contracts.py`. Graph, Raten und Kopplungen: `systemgraph.md`.
Komponenten, Signale und Abläufe zum Einlesen: der Komponentenplan (Word).

**Dateien mit festen Werten:**

| Datei | Inhalt |
|---|---|
| `roboter_tetris/Extrinsics/base_cam_extrinsics.json` | Kalibrierung der Basiskamera, die `base_cam` liest (Handkalibrierung L6); ein Kalibrierlauf schreibt in dieselbe Datei |
| `roboter_tetris/Safety/workspace_bounds.json` | Arbeitsraum des Flansches; dieselben Werte sind Standard im `object_follower`, ein Test prüft die Gleichheit |

## 5. Tests

- `test_contracts.py` prüft jedes Signalformat.
- Jedes Logikmodul hat eigene Tests ohne ROS, darunter Ende-zu-Ende-Läufe mit
  synthetischen Klötzen aus `test/tools/fake_objects.py` und Läufe des Followers im
  geschlossenen Regelkreis.
- Die Kalibrierung wird mit einer echten Aufnahme des AprilGrid
  (`test/python_tests/data/`) und synthetischen Posen geprüft.
- Konstruktionstests (Fixture `ros_context`) laufen im AICA-Testabbild:
  `docker build -f aica-package.toml --target test .` — am 24.09.2026 vollständig grün.
  `test_base_cam_contract` und `test_interface_streamer` werden dort übersprungen, weil
  `cv_bridge` im Testabbild fehlt.
- Lokal ohne AICA: `python3 -m pytest source/roboter_tetris/test/python_tests` mit
  `numpy<2` und `opencv-contrib-python-headless` 4.7, ohne die Dateien, die
  `state_representation` brauchen (`test_data_tracker`, `test_object_follower`,
  `test_priority_handler`, `test_robotiq_gripper`); Stand 30.09.2026: 269 bestanden,
  8 übersprungen.

**Werkzeuge** unter `test/tools/`: `fake_objects.py` erzeugt synthetische Klötze statt
`base_cam` und treibt die ganze Kette ohne Kamera; `basecam_kalibrierung.py` rechnet
eine Kalibrierung aus den Rohdaten eines Kalibrierlaufs nach.

## 6. Build und Laden

Neubauen: `docker build -f aica-package.toml -t roboter-tetris .` Ein Rebuild allein
genügt nicht: Das Paket wird erst wirksam, wenn danach das **AICA-Systemabbild im
Launcher neu erzeugt** wird; ein Neustart der Anwendung holt das ohne Fehlermeldung
nicht nach. Was beim Anlegen der Anwendung zu setzen ist:
`einrichtung-projektanwendung.md`.

## 7. Farbcode der Dokumentation

Im Komponentenplan durchgängig verwendet:

| Farbe | Hex | Bedeutung |
|---|---|---|
| Hellblau | `00B0F0` | Komponentennamen |
| Rot | `EE0000` | Datensignale (Zahlenfelder) |
| Grau | `808080` | Schaltsignale (Bool) |
| Orange | `FFC000` | Bildsignale |
| Gelb | `FFFF00` | Zielkoordinaten an die Robotersteuerung |
| Grün | `92D050` | Roboterzustand (Koordinaten TCP; genau genommen die Flanschpose) |
