# Projektkontext Robotetris

**Stand 21.09.2026.** Rahmenbedingungen, Abgrenzungen und Arbeitsweise.
Gedacht als Einstieg für jede Sitzung, die ohne Vorkontext startet — vor den
technischen Dokumenten zu lesen.

> Dies ist **nicht** die `CLAUDE.md`. Die existiert separat, ist in `.gitignore`
> eingetragen und damit **pro Rechner eigen** — sie wandert nicht mit dem Push.

---

## 1. Was das Projekt leisten soll

Ein **UR10e** greift farbige Klötze von einem laufenden Förderband — **im Lauf,
ohne dass das Band angehalten wird**. Die Klötze werden vorn von Hand aufgelegt
und durchlaufen den Arbeitsbereich.

### Die Projektziele (offizielle Vorgabe)

| Nr. | Ziel | Umsetzung | Stand |
|---|---|---|---|
| 1 | Ansteuerung des UR10e mit AICA | AICA-Kette Attractor → IK-Velocity-Controller | **erledigt** |
| 2 | Entwicklung eines schnellen Kalibrierungsverfahrens | Kommilitone, **getrenntes Projekt** (`Calibration/*`). Wir sind Abnehmer; offen ist nur die Übergabeform (C6). | läuft |
| 3 | Verfahren zur **Geschwindigkeitsschätzung** und Positionsberechnung der Gegenstände | `base_cam` (Position), `vectoring` (Geschwindigkeit je Klotz und gepoolt) | Position läuft am Aufbau; Schätzung **gebaut**, am Aufbau offen (B21, B23) |
| 4 | Algorithmus zur **Priorisierung** des zuerst zu greifenden Gegenstandes und zur **Bahnplanung** für das kontrollierte Greifen | `priority_handler` (Auswahl, Erreichbarkeit, Greifebene); `object_follower` mit den AICA-Bausteinen (Bahn) | **gebaut**, am Aufbau offen |

**Zusätzlich, als eigenes Ziel:** den Prozess nachvollziehbar darstellen —
`data_tracker` (ein Blatt ohne Rückwirkung auf den Regelpfad) und
`interface_streamer` (ein Übersichtsbild für RViz). **Beide gebaut.**

Den Aufbau des Systems als Graph, die Komponenten und die Kopplungen zwischen
ihren Parametern zeigt **`uebersicht/systemgraph.md`**.

> ⚠️ **Geschwindigkeit ist eine Vorgabe, keine Kalibrierung.** Ziel 3 verlangt ein
> *entwickeltes Verfahren* zur Geschwindigkeitsschätzung aus den Bilddaten. Die Doku
> hatte das bis zum 21.09.2026 anders angenommen — Bandgeschwindigkeit als
> einmalig kalibrierte Konstante — und die Schätzung ausdrücklich gestrichen.
> Korrigiert in `architektur/entscheidungen.md`, **Nachtrag 6**.

> **Bahnplanung über AICA:** Die Nutzung der von AICA bereitgestellten Bausteine
> (Attractor, IK-Velocity-Controller) gilt als Lösung der Vorgabe.

Der eigentliche Kern ist der **Pickvorgang im Lauf** — Ziele 3 und 4 zusammen.
**Genau daran ist das Vorgängerprojekt gescheitert.**

## 2. Was das Vorgängerprojekt erreicht hat

Die Vorgängergruppe arbeitete am **selben Aufbau, mit demselben Greifer und
demselben Band**. Sie hat:

- die Klötze erkannt und verfolgt (die C++-Erkennung ist die Grundlage der
  heutigen `base_cam`, nach Python übertragen)
- **nicht** im Lauf gegriffen: Der Roboter wartete an einer **festen Pickposition
  am Bandende** und griff zu, sobald der Klotz im Bild der Roboterkamera auftauchte
- den RGB-Raum für die Roboterkamera nicht zuverlässig nutzen können. Stattdessen
  ein Behelf: Die Kamera wurde so positioniert, dass die Tiefenkamera das Band
  gerade noch erfassen konnte, alles etwa einen Zentimeter darüber aber aus dem
  Messbereich fiel (Tiefenwert 0). Die Konturen wurden dann über diesen
  Null-Bereich bestimmt.

**Dieser Behelf funktioniert bei uns nur bedingt** — deshalb die beiden
Erkennungsvarianten `robot_cam` (farbbasiert) und `robot_cam_2` (kantenbasiert),
die aktuell gegeneinander getestet werden.

> **Der Grund ist physikalisch und in `docs/architektur/robot-cam-befunde.md`
> ausgeführt:** Die Tiefenkamera sitzt seitlich versetzt und sieht in Greifnähe
> die **spiegelnden Seitenflächen** der Klötze, die ebenfalls Tiefe 0 liefern.
> Der Verzug verschwindet mit größerer Kamerahöhe — daraus folgt die Kopplung
> von B8 (Beobachtungshöhe) und D18. Die Vorgängergruppe ist mit ihrer eigenen
> Lösung selbst unzufrieden (Rücksprache).

> **Präzisierung nach dem Archivabgleich (14.09.2026):** Die zuletzt gebaute
> „Strategie 2" fuhr zwar eine Spurkorrektur in x an, der Griff selbst war aber
> **Koppelnavigation** — der Roboter rechnete eine Ankunftszeit aus und
> schlief bis dahin (`time.sleep`). Es gab keine Rückkopplung während des Griffs
> und **keine Zielauswahl**; verarbeitet wurde immer das erste Objekt der Liste.
> Für `vectoring`, `priority_handler` und den Regelteil des `object_follower`
> gibt es dort also **kein Vorbild**. Einzelheiten:
> `docs/architektur/vorgaengerprojekt-abgleich.md`.

Für uns relevant: Die Vorgängergruppe hat Werte, die wir brauchen. Die
**Hand-Auge-Kalibrierung der Roboterkamera ist damit vollständig geklärt** (C1/C4:
Bezug ist der Flansch). Der **TCP der UR-Steuerung stand dagegen nicht im
Archiv** — er saß in der UR-Installation am Teach-Pendant und wurde am 15.09.2026
direkt aus der Steuerung ausgelesen: **215 mm**, auf der Flanschachse, unverdreht
(C8). ⚠️ **Das ist nicht der Abstand zum Griffpunkt.** Unsere Kette regelt den
Flansch; der Follower braucht **Flansch → Griffpunkt = 235 mm**
(`flange_to_grip_point_m`, gemessen). Die 215 mm dienen nur der Umrechnung fremder
TCP-Werte (`architektur/entscheidungen.md` Nachtrag 6 / Z7). Dass **TCP-Posen kommandiert wurden**, ist bestätigt (C9) — A7
beantwortet das aber nicht, denn im Archiv existiert kein URDF; die Frage ist
inzwischen eigenständig geklärt (kein Greifer im URDF, geregelt wird der Flansch).

## 3. Was heute funktioniert

**Stand 21.09.2026: Alle Komponenten sind gebaut** und lokal getestet (259 Tests,
davon 8 nur in der AICA-Testumgebung lauffähig). In AICA ist von den neuen noch
keine gelaufen — das ist der erste Schritt am Aufbau.

| Komponente | Stand |
|---|---|
| `robotiq_gripper` | **funktionsfähig** am Aufbau; seit 2.3 mit `motion_done`/`has_object` |
| `base_cam` | **funktionsfähig**, erkennt Klötze zuverlässig; seit 2.1/2.4 Vertrag S1 und gemessene Längsposition. ⚠️ Alte Kalibrierwerte passen nicht zu den Antastpunkten des Roboters (B23) |
| `vectoring` | gebaut — Geschwindigkeitsschätzung je Klotz und gepoolt (Ziel 3) |
| `priority_handler` | gebaut — Zielauswahl, Erreichbarkeit, Greifebene (Ziel 4); Greifzone auf Platzhaltern bis B19 |
| `data_tracker` | gebaut — Klotzliste für die Anzeige |
| `object_follower` | gebaut, alle vier Stufen — Start, Folgen, Roboterkamera als Korrektur, Greifzyklus mit Ablage |
| `interface_streamer` | gebaut — Übersichtsbild für RViz |
| `robot_cam` / `robot_cam_2` | implementiert und unit-getestet, **am Aufbau am 15.09.2026 durchgefallen** (B6). `robot_cam` farbbasiert (Band ausmaskieren), `robot_cam_2` kantenbasiert mit Tiefenkanten-Fusion. Identische I/O, im Graphen austauschbar. ⚠️ Beide scheitern **nicht an der Erkennung, sondern an der Auswahl** — Einzelheiten: `architektur/robot-cam-befunde.md` §9. **Auswahlkorrektur seit 2.2 umgesetzt**, am Aufbau noch nicht erprobt. |
| `move_to_pose_test`, `true_signal`, `toggle_signal` | Testhilfen; `toggle_signal` ersetzt am virtuellen Roboter die Greifer-Rückmeldung |
| `test/tools/fake_objects.py` | synthetische Klötze statt `base_cam` — treibt die ganze Kette ohne Kamera, im Robotersystem |
| AICA-Kette Attractor → IK-Velocity-Controller | **getestet**, Roboter folgt einem per Maus verschobenen Frame |

## 4. Abgrenzungen — was nicht angefasst wird

Dies sind harte Vorgaben, keine Empfehlungen.

### `roboter_tetris/Calibration/*` — fremdes Projekt

Der Ordner gehört dem Kommilitonen, der die Kamerakalibrierung automatisiert.
Das läuft als **getrenntes Projekt**. Die dortigen Komponenten
(`board_detection`, `auto_calibration`) und Dateien **dürfen unter keinen
Umständen verändert werden**.

Wir sind lediglich **Abnehmer** der Ergebnisse: Intrinsik beider Kameras,
Extrinsik der Basiskamera zur Roboterbasis. Die Werte werden als AICA-Parameter
gespiegelt, wie in `Calibration/README.md` beschrieben.

### `roboter_tetris/vision/*` — Bildverarbeitung

**Geändert 21.09.2026:** Die Komponenten rund um Kameras und Greifer **dürfen**
geändert werden; tabu ist nur die Kalibrierung. Für `vision/` heißt das:

| Datei | Regel |
|---|---|
| `vision/board.py` | **nicht anfassen** — die Kalibrierung nutzt sie (`Calibration/board_detection.py`) |
| `vision/tracker.py` | ein beschlossener Eingriff: gemessene statt gerechneter Längsposition (Umsetzungsplan 2.4) |
| `vision/robot_detection*.py` | nur die gemeinsame Blob-Auswahl (seit 2.2 umgesetzt); **die beiden Detektionskerne und die Rückprojektion bleiben**, weil der A/B-Test aus B6 die Kerne vergleicht und der Follower die Rückprojektion korrigiert |
| `vision/detection.py`, `color_estimation.py` | kein Anlass zur Änderung |

Geprüft: Die Kalibrierung importiert aus `vision/` ausschließlich `board.py`.

**Grund für die Zurückhaltung, die trotzdem gilt:** Das System ist bei der Bildverarbeitung an der
Leistungsgrenze. Die Kamerakomponenten sind bewusst so gehalten, dass neben der
Bildverarbeitung möglichst wenig gerechnet wird, damit sie ohne spürbare
Verzögerung in Echtzeit laufen. Zusatzrechnungen gehören in andere Komponenten —
deshalb liegt etwa die Umrechnung der Roboterkamera-Messung in Weltkoordinaten
im `object_follower` und nicht in `robot_cam`.

### `.init_wizard/` — Vorlagengenerator

Nach der Wizard-Ausführung nicht mehr ändern (`ARCHITECTURE.md`).

### Rebuild und Laden der Anwendung — Sache des Nutzers

Auf derselben AICA-Installation läuft das **Kalibrierprojekt des Kommilitonen**.
Ein Neubauen des Pakets und Neuladen der Anwendung stört es, deshalb schaut der
Nutzer **kurz vor dem Testen und Laden** nach, ob das gerade passt.

**Das ist eine Handreichung vor Ort, keine Planungsschranke.** Es betrifft weder
die Umsetzung der Komponenten noch das mobile Setup: Die Arbeit findet in der
eigenen Branch statt, die davon unabhängig ist, und vor Ort wird ohnehin immer
diese Branch geladen. Innerhalb der Branch bestehen keine Einschränkungen.

⚠️ **Eine technische Eigenheit ist trotzdem wichtig, wenn geladen wird:** Ein
Rebuild allein genügt nicht — das neue Paket wird erst wirksam, wenn danach das
**AICA-Systemabbild im Launcher neu erzeugt** wird. Ein Neustart der Anwendung
holt es nicht nach, und zwar ohne Fehlermeldung. Gegenprobe und Zwischenlösung
für den `global_time`-Fix in `base_cam`:
`uebersicht/einrichtung-projektanwendung.md` §1.

### GitHub

**Kein Git durch den Assistenten** — kein `commit`, `push`, `fetch`, `pull`,
`merge`, `checkout`, `rebase`, `stash`. Der Nutzer bedient git ausschließlich
selbst. Lesende Befehle (`status`, `log`, `diff`) sind in Ordnung. Ist ein
Git-Schritt nötig, wird er **vorgeschlagen, nicht ausgeführt** — auch dann, wenn
eine Aufgabe dadurch unfertig bleibt.

Dateien werden lokal angelegt und bleiben untracked, bis der Nutzer sie selbst
übernimmt. GitHub ist zugleich der **einzige Austauschweg zwischen den beiden
Setups** (§6) — was nicht gepusht ist, existiert auf dem anderen Rechner nicht.

## 5. Der Aufbau

| | |
|---|---|
| Roboter | UR10e, steht **direkt neben dem Band**, etwa auf einem Drittel vom Bandende aus gerechnet |
| Greifer | Robotiq 2-Finger (2F-140), über USB/Modbus direkt angesteuert — **nicht** als ros2_control-Hardware-Interface. An den letzten Fingergliedern sitzen **verschraubte 3D-Druck-Aufsätze** (Gewindeeinsätze); darauf eine mit Isolierband befestigte Gummi-Grippmatte, Greiffläche **20 mm hoch × 15 mm breit**. Öffnungsweite **127 mm** |
| Basiskamera | RealSense, am Bandanfang auf einem **beweglichen Gestell** — daher die automatisierte Extrinsik-Kalibrierung als Parallelprojekt (C3) |
| Roboterkamera | RealSense, am Arm montiert — **festes Bauteil am Flansch, unverändert seit der Vorgängergruppe**. Deren Hand-Auge-Kalibrierung gilt damit unmittelbar (C1) |
| Band | **grün-türkis** — gemessen **H ≈ 88–90**, nicht die ursprünglich angenommenen 60; der Farbton wandert zudem mit der Belichtungszeit (`architektur/robot-cam-befunde.md` §9.3). Konstante Geschwindigkeit, **nicht einstellbar**; sie wird im Betrieb **geschätzt** (Ziel 3), B1 prüft das nur gegen. Richtung ist praktisch die **y-Achse**; im Robotersystem angetastet bei x ≈ −0,70 … −0,93 m (die Basiskamera sieht es mit alten Kalibrierwerten woanders, B23). Spiegelungen treten **nur hier** auf, nicht auf den Klötzen |
| Klötze | rechtwinklig, **unterschiedlich groß**, von Hand aufgelegt, realistisch 2–3 gleichzeitig. Farben **rot, blau, weiß**. 3D-gedruckt: Oberseite **matt**, Seitenflächen **spiegelnd** (siehe `architektur/robot-cam-befunde.md`) |
| Ablage | seitlich neben dem Band auf der Roboterseite; Pose in der Luft über einer Auffangkiste, der Klotz fällt hinein |
| Freiraum | senkrecht über dem Arbeitsbereich frei; nur die Basiskamera steht am Bandanfang, den der Roboter kaum erreicht |

**Klotzverhalten auf dem Band** (klargestellt 21.09.2026): Die Klötze werden frei
und ungehindert aufgelegt und laufen mit Bandgeschwindigkeit. **Festhängen oder
Anstoßen kommt nicht vor.** Ein Klotz kann aber **beim Aufsetzen umkippen** — bis
er wieder gleichmäßig läuft, gilt er als einschwingend.

**Herunterfallen gibt es nur am Bandende**, wenn ein Klotz nicht rechtzeitig
gegriffen wurde — und das ist aus Position und geschätzter Geschwindigkeit
berechenbar. Von Hand vom Band genommen wird keiner. (`architektur/entscheidungen.md`,
Nachtrag 6 / Z3, Z5)

Die Ablage ist **kein Aufgabenpunkt**. Sie muss nur funktionieren, ohne dass sich
abgelegte Klötze gegenseitig behindern. Der Pickvorgang ist der Fokus.

## 6. Arbeitsweise

### Zwei Setups

| Setup | Rolle |
|---|---|
| **Lokal** — der Rechner am Roboter | Messungen, Tests, Inbetriebnahme, Build. Alles, was Hardware braucht. Hier ist auch das Vorgängerarchiv verfügbar. |
| **Mobil** — Arbeit abseits des Aufbaus | Konzept, Architektur, Code am Schreibtisch, Dokumentation. **Kein Roboter, kein Build.** |

Ein früher genutzter dritter Rechner (Stand-PC, ursprünglich für die
Konzeptarbeit) **entfällt.** Sein Stand ist überholt — die tragenden
Informationen stammen aus den Pushs des lokalen Setups.

Austausch läuft ausschließlich über GitHub, und der Nutzer pusht und pullt selbst
(§4). Daraus folgt die Arbeitsweise: **Alles, was am Schreibtisch entschieden
werden kann, wird vorab entschieden.** Die Umsetzung am Aufbau soll nicht neu
herleiten müssen — deshalb die ausführlichen Specs. Zeit am Aufbau ist die knappe
Ressource, nicht Zeit am Schreibtisch.

### Pfadunterschiede zum Vorgängerprojekt

Das Archiv der Vorgängergruppe liegt auf beiden Setups, aber **unter
verschiedenen Namen**. Die Dokumentation zitiert durchgängig die Variante des
lokalen Setups:

| | Pfad |
|---|---|
| **Lokal** (Name in der Doku) | `/home/tetripick/UR10_Pick_ws` |
| **Mobil** | `FuE_Greifen-main/` im Projektroot, in `.gitignore` |

**Die Struktur darunter ist identisch** — `Robot/`, `cameras/`, `docs/`,
`models/`, `zeroMQ/`. Jede Pfadangabe in der Dokumentation ist relativ zur
Archivwurzel zu lesen; nur die Wurzel unterscheidet sich. Ein Verweis wie
`cameras/tracker.hpp` oder `Robot/pose.yaml` ist also auf beiden Setups ohne
Umrechnung auffindbar.

⚠️ **Read-only, auf beiden Setups.** Im Archiv wird nichts verändert, nur
ausgelesen.

Ein zweiter Unterschied betrifft die Dateien, die `.gitignore` zurückhält und die
deshalb auf einem frisch gepullten Rechner **fehlen**: `CLAUDE.md`, `GEMINI.md`,
das Archiv selbst und `docs/archiv/2026-06-01-robotiq-gripper-component-design.md`.
Letztere wird in mehreren Dokumenten als Formatvorlage genannt — sie ist dort
nicht vorhanden, ohne dass das ein Fehler wäre.

### AICA-Aufbau

Die Kette stammt aus einem AICA-Beispielaufbau mit Frame-Verfolgung:

```
frame_to_signal → signal_point_attractor → ik_velocity_controller
                            ▲
              robot_state_broadcaster (cartesian_state)
```

Hardware-Rate 100 Hz. Im Betrieb ersetzt der `object_follower` den
`frame_to_signal`; der vollständige Graph steht in `uebersicht/systemgraph.md`.

Die Kenntnisse über verfügbare AICA-Controller sind im Team begrenzt — der
Aufbau entstand aus dem, was verstanden wurde. Auf AICA-Seite bestehen keine
Einschränkungen, Ergänzungen sind möglich.

### Vorerfahrung

Das Team hat **noch nicht mit Echtzeitanwendungen gearbeitet**. Entscheidungen
werden deshalb schrittweise erarbeitet und begründet, nicht nur festgelegt —
siehe `architektur/entscheidungen.md`, wo zu jeder Festlegung das *Warum* steht.

## 7. Dokumentenlandkarte

**Die Landkarte steht in `docs/README.md`** — dort, wo sie hingehört, samt
Vorrangregeln und Kurzregister („welche Frage wurde wo entschieden"). Sie wird
hier bewusst **nicht** wiederholt: Mehrfachpflege ist genau der Fehler, den
Nachtrag 3 in `entscheidungen.md` einmal teuer bezahlt hat.

Das Nötigste für den Einstieg:

- **`ARCHITECTURE.md`** (Projektroot) — verbindliche AICA-Regeln, gilt über
  allem in `docs/`
- **`architektur/entscheidungen.md`** und **`architektur/datenvertraege.md`** —
  **normativ.** Bei Widerspruch gelten diese beiden.
- **`uebersicht/uebergabe.md`** — Einstieg beim Rechnerwechsel: Stand, gemessene
  Werte, nächste Schritte
- Alles Weitere: `docs/README.md`

## 8. Farbcode der Dokumentation

Im Word-Dokument und im Systemgraph durchgängig verwendet:

| Farbe | Hex | Bedeutung |
|---|---|---|
| Hellblau | `00B0F0` | Komponentennamen |
| Rot | `EE0000` | Datensignale (Zahlenfelder) |
| Grau | `808080` | Schaltsignale (Bool) |
| Orange | `FFC000` | Bildsignale |
| Gelb | `FFFF00` | Zielkoordinaten an die Robotersteuerung |
| Grün | `92D050` | Roboterzustand (Koordinaten TCP) |

> Anmerkung: `FFFF00` ist auf weißem Grund schwer lesbar. Ein dunkleres Gold
> (etwa `BF8F00`) bliebe vom Orange unterscheidbar. Noch nicht entschieden.

## 9. Beobachtungen am Bestand

Ohne Handlungsbedarf, aber gut zu wissen:

- `component_descriptions/roboter_tetris_auto_calibration.json` existiert, die
  Klasse `AutoCalibration` ist aber **nicht in `setup.cfg` registriert**. Gehört
  zum Kalibrierprojekt — nicht anfassen.
- `Safety/workspace_bounds.json` steht auf `placeholder_not_yet_defined` mit
  lauter `null`. Laut eigenem README darf daraus nichts als Sicherheitsgrenze
  übernommen werden, solange das so ist.
- `Calibration/calibration.json` enthält Legacy-Werte, markiert als
  `legacy_initial_values` — noch nicht validiert.
- **Tests:** Seit 1.1 prüft `test_contracts.py` jedes Signalformat. Jede neue
  Komponente hat ein Logikmodul ohne ROS mit eigenen Tests (Ende-zu-Ende-Läufe mit
  `fake_objects.py` eingeschlossen) und einen Konstruktionstest, der nur in der
  AICA-Testumgebung läuft (`ros_context`). Lokal fehlt `pytest`; die Tests laufen
  in einer venv im Scratchpad mit einem kleinen Runner, der die AICA-Module
  nachbildet.
