# Projektkontext Robotetris

**Stand 24.09.2026, finaler Build.** Rahmenbedingungen, Aufbau, Abgrenzungen und
Arbeitsweise — der Einstieg vor den technischen Dokumenten.

> Dies ist **nicht** die `CLAUDE.md`. Die existiert separat, ist in `.gitignore`
> eingetragen und damit **pro Rechner eigen** — sie wandert nicht mit dem Push.

---

## 1. Was das Projekt leistet

Ein **UR10e** greift farbige Klötze von einem laufenden Förderband — **im Lauf,
ohne dass das Band angehalten wird** — und legt sie in einer Kiste neben dem Band
ab. Die Klötze werden am Bandanfang von Hand aufgelegt, stehend, liegend, flach
oder schräg.

### Die Projektziele (offizielle Vorgabe)

| Nr. | Ziel | Umsetzung | Ergebnis am Aufbau |
|---|---|---|---|
| 1 | Ansteuerung des UR10e mit AICA | AICA-Kette Signal Point Attractor → IK Velocity Controller | erfüllt |
| 2 | Entwicklung eines schnellen Kalibrierungsverfahrens | getrenntes Projekt eines Kommilitonen (`Calibration/*`); dieses Paket ist Abnehmer der Werte | Basiskamera läuft mit einer Übergangskalibrierung, Abweichung höchstens 6 mm |
| 3 | Verfahren zur **Geschwindigkeitsschätzung** und Positionsberechnung | `base_cam` (Position), `vectoring` (Geschwindigkeit je Klotz und für das Band) | Position gegen den Roboter höchstens 6 mm; Band geschätzt −127,9 mm/s gegen 125–133 mm/s per Stoppuhr |
| 4 | Algorithmus zur **Priorisierung** und **Bahnplanung** für das kontrollierte Greifen | `priority_handler` (Auswahl, Erreichbarkeit, Greifebene); `object_follower` mit den AICA-Bausteinen (Bahn) | greift im Lauf mit rund 1 mm Längsfehler; bei dichter Folge etwa ein Klotz je 7 s; flache und gedrehte Klötze; Dauerlauf zuverlässig |

**Zusätzlich:** den Prozess nachvollziehbar darstellen — `data_tracker` (eine
Klotzliste ohne Rückwirkung auf den Regelpfad) und `interface_streamer` (ein
Übersichtsbild für RViz).

Zwei Punkte der Aufgabenstellung, die leicht übersehen werden:

- **Die Bandgeschwindigkeit wird geschätzt, nicht kalibriert.** Ziel 3 verlangt
  ein Verfahren, das sie aus den Bilddaten bestimmt (`entscheidungen.md`,
  Nachtrag 6).
- **Die Bahnplanung über AICA** — die Nutzung der bereitgestellten Bausteine
  Attractor und IK Velocity Controller — gilt als Lösung der Vorgabe.

Der eigentliche Kern ist der **Pickvorgang im Lauf** — Ziele 3 und 4 zusammen.
Genau daran ist das Vorgängerprojekt gescheitert.

## 2. Das Vorgängerprojekt

Die Vorgängergruppe arbeitete am **selben Aufbau, mit demselben Greifer und
demselben Band** (Archiv `UR10_Pick_ws`, C++ und ZeroMQ statt AICA). Sie hat:

- die Klötze erkannt und verfolgt — ihre C++-Erkennung ist die Grundlage der
  heutigen `base_cam`, nach Python übertragen;
- **nicht** im Lauf gegriffen: Der Roboter wartete an einer festen Pickposition
  am Bandende, rechnete eine Ankunftszeit aus und griff dann ohne Rückkopplung zu.
  Eine Zielauswahl gab es nicht.

Für `vectoring`, `priority_handler` und den Regelteil des `object_follower` gab
es deshalb kein Vorbild. Übernommen wurden Messwerte des Aufbaus; der Abgleich
steht im Archiv (`archiv/vorgaengerprojekt-abgleich.md`).

Zwei Maße, die man nicht verwechseln darf:

- Der **TCP der UR-Steuerung** ist 215 mm. Er dient nur zur Umrechnung fremder
  TCP-Werte.
- Unsere Kette regelt den **Flansch**. Der Follower braucht **Flansch → Griffpunkt
  = 235 mm** (`flange_to_grip_point_m`, gemessen; Nachtrag 6 / Z7).

## 3. Der Stand der Komponenten

Die ganze Kette greift am Aufbau: `base_cam` → `vectoring` → `priority_handler` →
`object_follower` → Attractor → IK Velocity Controller. Gegriffen wird allein mit
der Basiskamera; die Komponenten der Roboterkamera (`robot_cam`, `robot_cam_2`)
liegen im Paket, sind aber nicht eingebunden (Nachtrag 13 / L22). Die Komponenten
im Einzelnen: der Komponentenplan (Word, zum Einlesen) und `systemgraph.md` (Graph,
Raten, Kopplungen).

Neben den Komponenten der Kette enthält das Paket Testhilfen:
`move_to_pose_test`, `true_signal`, `toggle_signal` und
`test/tools/fake_objects.py` — synthetische Klötze statt `base_cam`, die die ganze
Kette ohne Kamera treiben.

## 4. Abgrenzungen — was nicht angefasst wird

### `roboter_tetris/Calibration/*` — fremdes Projekt

Der Ordner gehört dem Kommilitonen, der die Kamerakalibrierung automatisiert. Die
dortigen Komponenten (`board_detection`, `auto_calibration`) und Dateien werden
nicht verändert. Dieses Paket ist **Abnehmer** der Ergebnisse (Intrinsik,
Extrinsik der Basiskamera); die Werte werden als AICA-Parameter gespiegelt, wie in
`Calibration/README.md` beschrieben. Bis dahin trägt `base_cam` die am Aufbau
gemessene Übergangskalibrierung als Standardwert (Nachtrag 13 / L6).

### `roboter_tetris/vision/*` — Bildverarbeitung

| Datei | Regel |
|---|---|
| `vision/board.py` | nicht anfassen — die Kalibrierung nutzt sie |
| `vision/robot_detection*.py` | Erkennung der Roboterkamera; nicht eingebunden |
| `vision/detection.py`, `vision/tracker.py` | Erkennung und Verfolgung der Basiskamera (Nachtrag 6 / Z4, Nachtrag 13 / L6) |

Die Bildverarbeitung ist an der Leistungsgrenze. Zusatzrechnungen gehören deshalb
in andere Komponenten, nicht in die Kamerakomponenten.

### `.init_wizard/` — Vorlagengenerator

Nach der Wizard-Ausführung nicht mehr ändern (`ARCHITECTURE.md`).

### Build und Laden der Anwendung

Neubauen des Pakets und Laden der Anwendung macht der Nutzer. Ein Rebuild allein
genügt nicht: Das neue Paket wird erst wirksam, wenn danach das
**AICA-Systemabbild im Launcher neu erzeugt** wird — ein Neustart der Anwendung
holt das ohne Fehlermeldung nicht nach (`einrichtung-projektanwendung.md` §1).

### Git

Git bedient der Nutzer selbst. Ein Assistent führt keine schreibenden Git-Befehle
aus (`commit`, `push`, `fetch`, `pull`, `merge`, `checkout`, `rebase`, `stash`);
lesende sind in Ordnung.

## 5. Der Aufbau

| | |
|---|---|
| Roboter | UR10e direkt neben dem Band, etwa auf einem Drittel vom Bandende aus |
| Greifer | Robotiq 2F-140 über USB/Modbus (nicht als ros2_control-Hardware). Verschraubte 3D-Druck-Aufsätze mit Gummi-Grippmatte, Greiffläche 20 mm hoch × 15 mm breit, Öffnungsweite 127 mm. Nutzlast in der UR-Installation 1,3 kg, Schwerpunkt 12 / 24 / 45 mm |
| Basiskamera | RealSense L515 senkrecht über dem Bandanfang, auf einem beweglichen Gestell — daher das Kalibrierprojekt |
| Roboterkamera | RealSense D435i am Flansch; nicht eingebunden |
| Band | grün-türkis, konstante Geschwindigkeit ≈ 0,13 m/s, nicht einstellbar, Lauf entlang der y-Achse; Lage und Maße in `einrichtung-projektanwendung.md` §8 |
| Klötze | rechtwinklig, unterschiedlich groß (25 bis 100 mm Kante), rot, blau, weiß; 3D-gedruckt, Oberseite matt, Seitenflächen spiegelnd |
| Ablage | Pose in der Luft über einer Auffangkiste neben dem Band auf der Roboterseite; der Klotz fällt hinein |

Die Klötze laufen frei mit Bandgeschwindigkeit. Ein Klotz kann beim Aufsetzen
umkippen; bis er gleichmäßig läuft, gilt er als einschwingend. Herunterfallen gibt
es nur am Bandende, wenn ein Klotz nicht gegriffen wurde.

## 6. Arbeitsweise

### Das Vorgängerarchiv

Das Archiv liegt unter `/home/tetripick/UR10_Pick_ws` (auf einem mobilen Rechner
als `FuE_Greifen-main/` im Projektroot, in `.gitignore`); die Struktur darunter
ist gleich. Es ist **read-only**: Dort wird nichts verändert, nur ausgelesen.

`.gitignore` hält außerdem `CLAUDE.md`, `GEMINI.md` und
`docs/archiv/2026-06-01-robotiq-gripper-component-design.md` zurück; auf einem
frisch gepullten Rechner fehlen sie.

### AICA-Aufbau

Der `object_follower` gibt die Zielpose an den Signal Point Attractor, der
IK Velocity Controller setzt sie um, der Robot State Broadcaster meldet die
Flanschpose zurück; das Hardware-Interface läuft mit 500 Hz. Der vollständige Graph
steht in `systemgraph.md`, die Werte, die in der Anwendung von Hand gesetzt werden,
in `einrichtung-projektanwendung.md` §2 und §5.

### Entscheidungen mit Begründung

Das Team hatte vorher nicht mit Echtzeitanwendungen gearbeitet. Entscheidungen
wurden deshalb schrittweise erarbeitet und begründet —
`architektur/entscheidungen.md` hält zu jeder Festlegung das *Warum* fest, am
Aufbau jeweils mit Messung.

### Tests

`test_contracts.py` prüft jedes Signalformat. Jede Komponente hat ein Logikmodul
ohne ROS mit eigenen Tests (Ende-zu-Ende-Läufe mit `fake_objects.py`
eingeschlossen) und einen Konstruktionstest, der im AICA-Testabbild läuft
(`docker build -f aica-package.toml --target test .`; Fixture `ros_context` aus
`test/python_tests/conftest.py`). `test_base_cam_contract` und
`test_interface_streamer` werden dort übersprungen, weil `cv_bridge` im
Testabbild fehlt. Lokal: `python3 -m pytest` ohne die Dateien, die
`state_representation` brauchen.

## 7. Dokumentenlandkarte

Die Landkarte samt Vorrangregeln und Kurzregister steht in **`docs/README.md`**.
Das Nötigste:

- **`ARCHITECTURE.md`** (Projektroot) — verbindliche AICA-Regeln
- **`architektur/entscheidungen.md`** und **`architektur/datenvertraege.md`** —
  normativ; bei Widerspruch gelten diese beiden
- **Komponentenplan** (Word) — alle Komponenten und Funktionen zum Einlesen

## 8. Farbcode der Dokumentation

Im Komponentenplan und im Systemgraph durchgängig verwendet:

| Farbe | Hex | Bedeutung |
|---|---|---|
| Hellblau | `00B0F0` | Komponentennamen |
| Rot | `EE0000` | Datensignale (Zahlenfelder) |
| Grau | `808080` | Schaltsignale (Bool) |
| Orange | `FFC000` | Bildsignale |
| Gelb | `FFFF00` | Zielkoordinaten an die Robotersteuerung |
| Grün | `92D050` | Roboterzustand (Koordinaten TCP; genau genommen die Flanschpose) |

## 9. Beobachtungen am Bestand

- `component_descriptions/roboter_tetris_auto_calibration.json` existiert, die
  Klasse `AutoCalibration` ist nicht in `setup.cfg` registriert. Gehört zum
  Kalibrierprojekt.
- `Safety/workspace_bounds.json` ist am Aufbau abgefahren und festgelegt
  (`status: defined`); der Follower trägt dieselben Werte als Standard, ein Test
  prüft die Gleichheit.
- `Calibration/calibration.json` enthält Werte des Vorgängerprojekts, markiert als
  `legacy_initial_values`.
