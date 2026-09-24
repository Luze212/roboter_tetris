# Robotetris — Pick im Lauf mit UR10e und AICA

Ein UR10e greift farbige Klötze von einem laufenden Förderband, **ohne das Band
anzuhalten**, und legt sie in einer Kiste neben dem Band ab. Eine feste Kamera über
dem Bandanfang erkennt die Klötze; daraus werden Position und Geschwindigkeit
geschätzt, der nächste erreichbare Klotz gewählt und die Bewegung über die
AICA-Bausteine Signal Point Attractor und IK Velocity Controller geführt.

Dieses Repository ist das AICA-Paket `roboter_tetris` mit den eigenen Komponenten.

## Ergebnis am Aufbau

- Greifen im Lauf mit rund 1 mm Längsfehler, bei dichter Folge etwa ein Klotz je 7 s
- stehende, liegende, flache (ab 20 mm) und schräg liegende Klötze (Drehung höchstens ±45°)
- Bandgeschwindigkeit aus den Bilddaten geschätzt: −127,9 mm/s gegen 125–133 mm/s per Stoppuhr
- Dauerlauf mit gemischten Klötzen zuverlässig

## Die Kette

```
RealSense L515 → base_cam → vectoring → priority_handler → object_follower
                                                              │        │
                                                   robotiq_gripper   Signal Point Attractor
                                                                       → IK Velocity Controller → UR10e
Anzeige: data_tracker, interface_streamer (RViz)
```

| Komponente | Aufgabe |
|---|---|
| `base_cam` | erkennt die Klötze im Bild der Basiskamera, Lage in `world` |
| `vectoring` | schätzt Geschwindigkeit je Klotz und für das Band, führt Klötze hinter dem Bild weiter |
| `priority_handler` | wählt den dringendsten erreichbaren Klotz, rechnet die Greifebene |
| `object_follower` | fährt an, folgt, senkt ab, greift mitfahrend, legt ab |
| `robotiq_gripper` | bedient den Greifer, meldet „zu“ und „Objekt gegriffen“ |
| `data_tracker`, `interface_streamer` | Klotzliste und Übersichtsbild |

## Dokumentation

Einstieg: **`docs/README.md`** (Landkarte) und `docs/uebersicht/projektkontext.md`.
Alle Komponenten zum Einlesen: `docs/uebersicht/Komponentenplan Robotetris - Stand 2026-09-24.docx`.
Einrichtung der AICA-Anwendung: `docs/uebersicht/einrichtung-projektanwendung.md`.
Verbindliche AICA-Regeln: `ARCHITECTURE.md`.

## Bauen und testen

```bash
docker build -f aica-package.toml -t roboter-tetris .
```

```bash
docker build -f aica-package.toml --target test .
```

Das neue Paket wird erst wirksam, wenn danach das AICA-Systemabbild im Launcher neu
erzeugt wird. Die Logikmodule ohne ROS lassen sich auch lokal testen
(`python3 -m pytest source/roboter_tetris/test/python_tests`, ohne die Dateien, die
`state_representation` brauchen).

## Aufbau des Repositorys

```
aica-package.toml                 Build-Konfiguration (statt Dockerfile)
ARCHITECTURE.md                   AICA-Regeln
docs/                             Dokumentation (uebersicht/, architektur/, archiv/)
source/roboter_tetris/
  component_descriptions/         AICA-Beschreibungen der Komponenten
  roboter_tetris/                 Komponenten und Logikmodule ohne ROS
  roboter_tetris/contracts.py     Signalformate S1–S10
  roboter_tetris/Calibration/     Kalibrierprojekt (getrennt)
  roboter_tetris/Safety/          Arbeitsraumgrenzen
  test/python_tests/              Tests
  test/tools/                     fake_objects.py, Mitlese- und Auswertewerkzeuge
```

Das Paket entstand aus dem
[AICA Package Template](https://github.com/aica-technology/package-template);
Einzelheiten zu `aica-package.toml` und eigenen Komponenten:
<https://docs.aica.tech/docs/category/custom-components>.
