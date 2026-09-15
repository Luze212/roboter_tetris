# Projektkontext Robotetris

**Stand 13.09.2026.** Rahmenbedingungen, Abgrenzungen und Arbeitsweise.
Gedacht als Einstieg für jede Sitzung, die ohne Vorkontext startet — vor den
technischen Dokumenten zu lesen.

> Dies ist **nicht** die `CLAUDE.md`. Die existiert separat und ist in
> `.gitignore` eingetragen.

---

## 1. Was das Projekt leisten soll

Ein **UR10e** greift farbige Klötze von einem laufenden Förderband — **im Lauf,
ohne dass das Band angehalten wird**. Die Klötze werden vorn von Hand aufgelegt
und durchlaufen den Arbeitsbereich.

Die Aufgabenstellung umfasst vier Punkte:

| Nr. | Aufgabe | Stand |
|---|---|---|
| 1 | AICA implementieren, Hardware zum Laufen bringen | **erledigt** |
| 2 | Automatische Kamerakalibrierung einrichten | läuft — **anderer Kommilitone, getrenntes Projekt** |
| 3 | **Pickvorgang on-the-fly umsetzen** | Gegenstand dieser Arbeit |
| 4 | Prozess nachvollziehbar darstellen | `interface_streamer`, zuletzt |

Punkt 3 ist der eigentliche Kern. **Genau daran ist das Vorgängerprojekt
gescheitert.**

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
Bezug ist der Flansch). Der **Werkzeugversatz zum Greifpunkt steht dagegen nicht
im Archiv** — er saß in der UR-Installation am Teach-Pendant und muss aus der
Robotersteuerung kommen (C8). Dass **TCP-Posen kommandiert wurden**, ist bestätigt
(C9) — A7 beantwortet das aber nicht, denn im Archiv existiert kein URDF.

## 3. Was heute funktioniert

| Komponente | Stand |
|---|---|
| `robotiq_gripper` | **funktionsfähig** am Aufbau |
| `base_cam` | **funktionsfähig**, erkennt Klötze zuverlässig |
| `robot_cam` / `robot_cam_2` | implementiert und unit-getestet, **am Aufbau noch nicht gelaufen** (B6). `robot_cam` farbbasiert (grünes Band ausmaskieren), `robot_cam_2` kantenbasiert mit Tiefenkanten-Fusion. Identische I/O, im Graphen austauschbar. Hintergrund: `architektur/robot-cam-befunde.md` |
| `move_to_pose_test`, `true_signal`, `toggle_signal` | Testhilfen |
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

Die Algorithmik steht und wird **gesondert behandelt**. Sie ist ausdrücklich
nicht Teil dieser Überarbeitung. Geändert wird ausschließlich, **wie ihre
Ergebnisse ausgegeben werden** — also das Packen der Ausgabearrays in
`base_cam.py`, `robot_cam.py` und `robot_cam_2.py`.

**Grund für die Zurückhaltung:** Das System ist bei der Bildverarbeitung an der
Leistungsgrenze. Die Kamerakomponenten sind bewusst so gehalten, dass neben der
Bildverarbeitung möglichst wenig gerechnet wird, damit sie ohne spürbare
Verzögerung in Echtzeit laufen. Zusatzrechnungen gehören in andere Komponenten —
deshalb liegt etwa die Umrechnung der Roboterkamera-Messung in Weltkoordinaten
im `object_follower` und nicht in `robot_cam`.

### `.init_wizard/` — Vorlagengenerator

Nach der Wizard-Ausführung nicht mehr ändern (`ARCHITECTURE.md`).

### Kein Rebuild der AICA-Anwendung ohne Absprache

Auf derselben AICA-Installation läuft das **Kalibrierprojekt des Kommilitonen**.
Ein Neubauen des Pakets und Neuladen der Anwendung stört es. **Stand 15.09.2026
ist beides nicht möglich.**

Die Folge für die Planung ist erheblich: **Änderungen an Komponenten lassen sich
derzeit am Aufbau nicht erproben.** Sie können geschrieben und ROS-frei getestet
werden — am Aufbau wirksam werden sie erst nach einem abgestimmten Rebuild.
Messungen, die nur die vorhandenen Komponenten beobachten, sind davon nicht
betroffen.

Betroffen ist insbesondere der `global_time`-Fix in `base_cam`. Zwischenlösung:
`uebersicht/einrichtung-projektanwendung.md` §1.

### GitHub

**Es wird nichts committet und nichts gepusht.** Der Nutzer bedient git
ausschließlich manuell. Dateien werden lokal angelegt und bleiben untracked, bis
er sie selbst übernimmt.

## 5. Der Aufbau

| | |
|---|---|
| Roboter | UR10e, steht **direkt neben dem Band**, etwa auf einem Drittel vom Bandende aus gerechnet |
| Greifer | Robotiq 2-Finger (2F-140), über USB/Modbus direkt angesteuert — **nicht** als ros2_control-Hardware-Interface. An den letzten Fingergliedern sitzen **verschraubte 3D-Druck-Aufsätze** (Gewindeeinsätze); darauf eine mit Isolierband befestigte Gummi-Grippmatte, Greiffläche **20 mm hoch × 15 mm breit**. Öffnungsweite **127 mm** |
| Basiskamera | RealSense, am Bandanfang auf einem **beweglichen Gestell** — daher die automatisierte Extrinsik-Kalibrierung als Parallelprojekt (C3) |
| Roboterkamera | RealSense, am Arm montiert — **festes Bauteil am Flansch, unverändert seit der Vorgängergruppe**. Deren Hand-Auge-Kalibrierung gilt damit unmittelbar (C1) |
| Band | **grün**; konstante Geschwindigkeit, **nicht einstellbar**, Wert noch unbekannt. Spiegelungen treten **nur hier** auf, nicht auf den Klötzen |
| Klötze | rechtwinklig, **unterschiedlich groß**, von Hand aufgelegt, realistisch 2–3 gleichzeitig. Farben **rot, blau, weiß**. 3D-gedruckt: Oberseite **matt**, Seitenflächen **spiegelnd** (siehe `architektur/robot-cam-befunde.md`) |
| Ablage | seitlich neben dem Band auf der Roboterseite; Pose in der Luft über einer Auffangkiste, der Klotz fällt hinein |
| Freiraum | senkrecht über dem Arbeitsbereich frei; nur die Basiskamera steht am Bandanfang, den der Roboter kaum erreicht |

Herunterfallende Klötze sind **kein Problem** — sie müssen nur erkannt werden,
damit der Roboter ihnen nicht hinterherfährt. Das leistet die
Plausibilitätsprüfung in `vectoring`.

Die Ablage ist **kein Aufgabenpunkt**. Sie muss nur funktionieren, ohne dass sich
abgelegte Klötze gegenseitig behindern. Der Pickvorgang ist der Fokus.

## 6. Arbeitsweise

### Zwei Systeme

| System | Rolle |
|---|---|
| Heimrechner | Konzeptarbeit, Architektur, Dokumentation (diese Sitzung) |
| Laptop | Umsetzung der Komponenten, Zeit am realen Aufbau begrenzt |

Daraus folgt: **Alles, was am Schreibtisch entschieden werden kann, wird vorab
entschieden.** Die Umsetzungssitzung soll nicht neu herleiten müssen — deshalb
die ausführlichen Specs.

### Ein zweiter Projekt-Chat

Auf dem Laptop existiert eine weitere Sitzung, in der die `robot_cam`-Varianten
entstanden sind. Sie kennt unter anderem den Ablageort der
Attractor-Parameter (`offene-punkte.md`, C7).

### AICA-Aufbau

Die Kette stammt aus einem AICA-Beispielaufbau mit Frame-Verfolgung:

```
frame_to_signal → signal_point_attractor → ik_velocity_controller
                            ▲
              robot_state_broadcaster (cartesian_state)
```

Hardware-Rate 100 Hz. Der `object_follower` ersetzt `frame_to_signal` im Betrieb.

Die Kenntnisse über verfügbare AICA-Controller sind im Team begrenzt — der
Aufbau entstand aus dem, was verstanden wurde. Auf AICA-Seite bestehen keine
Einschränkungen, Ergänzungen sind möglich.

### Vorerfahrung

Das Team hat **noch nicht mit Echtzeitanwendungen gearbeitet**. Entscheidungen
werden deshalb schrittweise erarbeitet und begründet, nicht nur festgelegt —
siehe `architektur/entscheidungen.md`, wo zu jeder Festlegung das *Warum* steht.

## 7. Dokumentenlandkarte

| Dokument | Inhalt |
|---|---|
| `ARCHITECTURE.md` | **verbindliche** AICA-Regeln — Pflichtlektüre |
| `docs/uebersicht/projektkontext.md` | dieses Dokument |
| `docs/Komponentenplan Robotetris - Stand 2026-09-13.docx` | Systembeschreibung für Menschen, mit Farbcode |
| `docs/Komponentenplan Robotetris.docx` | **Original**, unverändert, historischer Stand |
| `docs/architektur/entscheidungen.md` | alle Architekturentscheidungen mit Begründung (Themen 1–7) |
| `docs/architektur/datenvertraege.md` | verbindliche Signalspezifikation |
| `docs/uebersicht/offene-punkte.md` | Arbeitsliste nach Ort und Quelle |
| `docs/uebersicht/systemgraph.md` | Graph und Signalliste |
| `docs/architektur/vorgaengerprojekt-abgleich.md` | **Systematischer Abgleich aller offenen Punkte gegen das Vorgängerarchiv `UR10_Pick_ws`.** Was von dort beantwortet ist (C1/C4 Hand-Auge, A6, B16), was dort *nicht* zu holen ist (C8, A7, B1), Größenordnungen zur Vorbelegung von Parametern, übertragbares Know-how und fünf Fallen. |
| `docs/architektur/robot-cam-befunde.md` | **Szene, Materialeigenschaften und Physik der Roboterkamera.** Warum der Loch-Trick verworfen wurde, warum die Beobachtungshöhe nicht frei wählbar ist, verworfene Wege mit Begründung, Werte aus dem Vorgängerprojekt. Vor jeder Arbeit an `robot_cam` / `robot_cam_2` lesen. |
| `docs/archiv/2026-09-05-konzeptreview-komponentenplan.md` | die ursprüngliche Analyse (61 Befunde) |
| `docs/architektur/specs/2026-09-13-umsetzungsplan-on-the-fly-pick.md` | Reihenfolge, Phasen, Abnahmekriterien |
| in `docs/architektur/specs/`:<br>`2026-09-13-data-tracker-component-design.md`<br>`2026-09-13-interface-streamer-component-design.md`<br>`2026-09-13-object-follower-component-design.md`<br>`2026-09-13-priority-handler-component-design.md`<br>`2026-09-13-vectoring-component-design.md`<br>`2026-09-13-bestandskomponenten-anpassungen.md`<br>`2026-06-01-robotiq-gripper-component-design.md` | Umsetzungsvorlagen je Komponente |

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
- Die bestehenden Tests prüfen ausschließlich die Module unter `vision/`, nicht
  das Packen der Komponentenausgaben. Eine Änderung des Ausgabeformats bricht
  daher keine Tests.
