# Umsetzungsplan: On-the-fly-Pick

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Umsetzungsbereit, mit benannten Vorbedingungen

## Zweck

Rückgrat für die Erstellung aller Komponenten des On-the-fly-Pickvorgangs.
Gedacht für eine Sitzung, die **ohne Vorkontext** startet: jeder Schritt nennt
Voraussetzungen, betroffene Dateien und ein prüfbares Abnahmekriterium.

## Pflichtlektüre vor dem ersten Schritt

| Dokument | Warum |
|---|---|
| `docs/projektkontext.md` | Rahmenbedingungen, Abgrenzungen, was **nicht** angefasst wird. Zuerst lesen. |
| `ARCHITECTURE.md` | Verbindliche AICA-Regeln. Nicht optional. |
| `docs/review/entscheidungen.md` | Alle Architekturentscheidungen mit Begründung (Themen 1–7) |
| `docs/review/datenvertraege.md` | Verbindliche Signalspezifikation |
| `docs/review/offene-punkte.md` | Was noch zu klären/messen ist |
| `docs/review/robot-cam-befunde.md` | Szene, Materialphysik und verworfene Wege der Roboterkamera. **Vor Schritt 2.2 und vor jeder Arbeit an der Beobachtungshöhe (B8) lesen.** |
| `docs/superpowers/specs/2026-06-01-robotiq-gripper-component-design.md` | Format- und Qualitätsvorlage für Komponenten-Specs |

## Verbindliche Rahmenregeln

Aus `ARCHITECTURE.md`, besonders relevant hier:

- `LifecycleComponent`, I/O über `add_input`/`add_output` im `__init__`, kein Pub/Sub
- **Niemals blockieren** in Callbacks, kein `time.sleep()`
- Kein `copy.deepcopy()` auf `state_representation`-Objekte → Copy-Konstruktor
- Python-Modul im gleichnamigen Unterordner, Registrierung in `setup.cfg` mit `::`
- JSON je Komponente in `component_descriptions/` (`registration`/`inherits`)
- Keine erfundenen CMake-Makros; `install(DIRECTORY ./component_descriptions DESTINATION .)`
- Neue Python-Abhängigkeiten in `requirements.txt`

---

## Leitidee der Reihenfolge

1. **Der Datenpfad wird vollständig ohne Roboterbewegung geprüft.** `vectoring`,
   `data_tracker` und `priority_handler` sind bei stehendem Roboter testbar —
   Blöcke auflegen, zuschauen, ob Glättung, Plausibilität und Auswahl stimmen.
   Eine komplette Validierungsphase ohne Bewegungsrisiko.
2. **Der `object_follower` entsteht in vier Stufen.** Die erste bewegt nur
   zwischen zwei stehenden Posen. Das Risiko wächst schrittweise statt auf
   einen Schlag.

---

## Phase 0 — Vorbedingungen klären (kein Code)

Billig, aber blockierend. Ohne diese Werte entstehen Komponenten, die später
umgebaut werden müssen.

| Schritt | Aufgabe | Quelle | Blockiert |
|---|---|---|---|
| 0.1 | **Welches Frame regelt der IK-Velocity-Controller** — Flansch oder Greifpunkt? Steht der Greifer im URDF? (A7/A8) | AICA Studio; zuverlässiger: wie hat die **Vorgängergruppe** kommandiert (C9) — gleicher Aufbau | Phase 4 |
| 0.2 | **Werkzeugversatz Flansch → Greifpunkt** (C8) | Vorgängerprojekt, gleicher Greifer | Phase 4 |
| 0.3 | **Hand-Auge-Kalibrierung Roboterkamera** (C1), Konvention und Bezug klären (C4) | Vorgängerprojekt | Phase 4c |
| 0.4 | **Signaltyp des `attractor`-Eingangs** (`cartesian_state` vs. `cartesian_pose`, A2) | AICA Studio — was liefert `frame_to_signal.pose`? | Phase 4a |
| 0.5 | **Attractor-Parameter ablesen** (A1): `linear gain`, `max linear velocity`, Präzisionsschwellen | AICA Studio | Phase 4b |
| 0.6 | **Uhrendrift prüfen** (B13): `ros_zeit − header_stempel` über einige Minuten | Labor, eine Logzeile | Phase 2 |

> Schritt 0.1 ist der wichtigste. Regelt der Controller den **Flansch** und wir
> geben die Blockposition als Ziel aus, fährt der Greifer rund 20 cm zu tief —
> ins Band. Indiz bisher: In Studio ist nur der Flansch sichtbar, der Greifer
> steht also vermutlich nicht im URDF.

---

## Phase 1 — Fundament

### 1.1 `contracts.py`

| | |
|---|---|
| **Ziel** | Ein gemeinsames Modul mit Kopflängen, Strides, Feldindizes und `pack_*`/`unpack_*` für alle elf Signale |
| **Voraussetzung** | keine |
| **Vorlage** | `docs/review/datenvertraege.md` — vollständig und verbindlich |
| **Dateien** | **neu** `source/roboter_tetris/roboter_tetris/contracts.py`<br>**neu** `source/roboter_tetris/test/python_tests/test_contracts.py` |
| **Abnahme** | Alle `pack_*`/`unpack_*` sind Round-Trip-getestet. `unpack_*` lehnt zu kurze/inkonsistente Arrays ab, statt auf halben Daten zu rechnen. Keine ROS-/cv2-Importe — rein testbar. |

**Regeln:** `unpack_*` prüft die Länge gegen `kopf + n·stride`. `unpack_*` rechnet
nicht um und interpretiert nicht — Extrapolation und Transformation macht der
Verbraucher. Keine Komponente indiziert von Hand in ein fremdes Array.

---

## Phase 2 — Bestehende Komponenten auf die Verträge bringen

Kleine, risikoarme Änderungen an laufendem Code. Jede einzeln testbar.

### 2.1 `base_cam` — Ausgabevertrag

| | |
|---|---|
| **Ziel** | Ausgabe auf `[t, n, v_band] + n×9` umstellen, SI-Einheiten |
| **Änderung** | Kopf mit `t` aus `header.stamp`; mm → m beim Packen; `vy` vom Objektfeld in den Kopf; Rate auf 100 Hz |
| **Nicht ändern** | Die Vision-Module unter `vision/` bleiben unangetastet, sie rechnen weiter in mm |
| **Dateien** | `roboter_tetris/base_cam.py`, `component_descriptions/roboter_tetris_base_cam.json` |
| **Abnahme** | Bestehende Tests laufen unverändert durch (sie prüfen nur `vision/*`). Ein neuer Test prüft das Packen gegen `contracts.py`. |

> `v_band` gehört in den Kopf, weil der Tracker die Geschwindigkeit global allen
> Tracks zuweist (`set_global_velocity`) — als Objektfeld ist der Wert
> irreführend.

### 2.2 `robot_cam` und `robot_cam_2` — Ausgabevertrag

| | |
|---|---|
| **Ziel** | Ausgabe auf `[t, valid, x, y, z_band, orientation]` umstellen, SI-Einheiten |
| **Änderung** | `valid`-Flag ergänzen; Distanz-Gate über neue Parameter `min_belt_distance_m` / `max_belt_distance_m` auf `z_band`; mm → m; Rate 100 Hz |
| **Wichtig** | Bei `valid = 0` läuft `t` **weiter**. Nur so unterscheidet der Empfänger "Kamera arbeitet, sieht nichts" von "Kamera liefert nicht mehr". |
| **Nicht ändern** | Erkennungsalgorithmik in `vision/robot_detection*.py` — **inkl. des gemeinsamen Kerns `localize_largest_blob`**, den sich beide Varianten teilen. Nur so bleibt der A/B-Test aussagekräftig. |
| **Vorher lesen** | `docs/review/robot-cam-befunde.md` — begründet die Defaults und nennt die Fallstricke (Seitenflächen-Physik, Nah-Gate, RealSense-Konfiguration) |
| **Dateien** | `roboter_tetris/robot_cam.py`, `robot_cam_2.py` + beide JSONs |
| **Abnahme** | Beide Varianten liefern denselben Vertrag und sind gegeneinander austauschbar. |

### 2.3 `robotiq_gripper` — zwei Bool-Ausgänge

| | |
|---|---|
| **Ziel** | `is_closed` und `has_object` als `Bool`-Signale ergänzen |
| **Begründung** | Der Plan sah nur "Greifer zu" vor — ein Echo des Eingangs und als Rückmeldung wertlos. Der Follower braucht `has_object`, um einen Fehlgriff zu erkennen. |
| **Nicht ändern** | Worker-Thread, Port-pro-Operation-Muster (`ARCHITECTURE.md` §13), vorhandene Predicates |
| **Dateien** | `roboter_tetris/robotiq_gripper.py`, `component_descriptions/roboter_tetris_robotiq_gripper.json` |
| **Abnahme** | Bestehender Test läuft weiter; neuer Test prüft, dass `has_object` dem Predicate folgt. |

---

## Phase 3 — Datenpfad (**ohne Aufbau und ohne Roboter** testbar)

### 3.0 Synthetischer Signalgeber (zuerst)

| | |
|---|---|
| **Ziel** | Ein kleines Skript, das synthetische `objects`-Arrays veröffentlicht: drei Blöcke, die mit konstanter Geschwindigkeit über ein gedachtes Band wandern |
| **Voraussetzung** | 1.1 |
| **Nutzen** | Treibt `vectoring`, `priority_handler` und den Follower vollständig an — **ohne Kameras, ohne Roboter, ohne Aufbau**. Glättung, Plausibilität, Auswahl, Erreichbarkeit, Ziel-Lock und die gesamte Zustandsmaschine sind damit am Schreibtisch validierbar. |
| **Dateien** | **neu** `source/roboter_tetris/test/tools/fake_objects.py` (kein Komponentenpaket, reines Testwerkzeug) |
| **Abnahme** | Blöcke erscheinen, wandern, verlassen die Zone; ein künstlich angehaltener Block erhält Status 1 |

> Bei knapper Zeit am Aufbau ist das die wertvollste Einzelmaßnahme im Plan.

### 3.1 `vectoring`

| | |
|---|---|
| **Ziel** | Geglättete Objektzustände plus Plausibilitätscode |
| **Voraussetzung** | 1.1, 2.1; Bandrichtung und -geschwindigkeit als Parameter (B1) |
| **Kern** | Querposition und Orientierung mitteln (konstant je Objekt); Längsposition auf die Bandgerade projizieren; Plausibilität gegen die kalibrierte Geschwindigkeit; Verfall ausbleibender IDs |
| **Besonderheit** | Orientierung über den **verdoppelten Winkel** mitteln (`½·atan2(⟨sin2θ⟩, ⟨cos2θ⟩)`). Die Resultantenlänge ist das Gütemaß: unter der Schwelle gilt die Orientierung als unbrauchbar. Ein arithmetischer Mittelwert aus 0° und 90° wäre 45° — genau die Lage, in der der Greifer die Ecken erwischt. |
| **Entfällt** | Parameter `K`, `W`, `J`, `H`, "Richtung halten", "Geschwindigkeit halten", lineare Regression |
| **Abnahme** | Reine Logiktests ohne ROS. Am Aufbau: Blöcke auflegen, Status-Codes im Log prüfen — Code 3 beim Auflegen, dann 0; Block anhalten → Code 1. |

### 3.2 `data_tracker`

| | |
|---|---|
| **Ziel** | Gesamtliste mit `picked` / `out_of_bounds`, rein zur Diagnose |
| **Voraussetzung** | 1.1, 3.1 |
| **Wichtig** | **Blatt im Graphen** — keine Leitung zurück in den Regelpfad. Rate 10 Hz. |
| **Verfallsregel** | Einträge mit `picked` oder `out_of_bounds` verschwinden nach einer Frist. Ohne sie wächst das Signal monoton. |
| **Abnahme** | Liste bildet auflegen/picken/herausfahren korrekt ab; Länge bleibt beschränkt. |

### 3.3 `priority_handler`

| | |
|---|---|
| **Ziel** | Zielauswahl mit Erreichbarkeits- und Greifbarkeitsprüfung, Ziel-Lock |
| **Voraussetzung** | 1.1, 3.1; Greifzone (B19), Greifergeometrie (B15/B16) |
| **Erreichbarkeit** | `t_verfügbar = (zonenende − position)/v_band`<br>`t_benötigt = abstand/v_max + 3/K + absenken + greifen`<br>Kandidat ⟺ `t_verfügbar > faktor · t_benötigt` |
| **Greifbarkeit** | Höhe ≥ `min_greifbare_hoehe`; Abmessung quer zur Backenrichtung ≤ `max_oeffnung − marge` |
| **Auswahl** | Unter den Kandidaten das dringendste (kleinstes `t_verfügbar`) — die Regel aus dem Plan, nur eben unter den *erreichbaren* |
| **Ziel-Lock** | Einmal gewählt bleibt gewählt. Rückzug nur bei: gepickt, ID verschwunden, unplausibel. **Erreichbarkeit ist Auswahl-, kein Abbruchkriterium** — sonst wird ein fast erfolgreicher Griff abgebrochen, wenn der Block beim Greifen die Zonengrenze überschreitet. |
| **Abnahme** | **Bei stehendem Roboter vollständig prüfbar.** Blöcke auflegen, im Log verfolgen, welches Ziel wann gewählt und warum zurückgezogen wird. Zu flache/zu breite Blöcke werden gar nicht erst gewählt. |

> Ende Phase 3: Der gesamte Datenpfad ist validiert, ohne dass sich der Roboter
> je bewegt hat.

---

## Phase 4 — `object_follower` in vier Stufen

> **Die Stufen 4a bis 4c laufen am virtuellen Roboter.** AICA selbst läuft dabei
> real; nur statt des echten UR10e wird ein virtueller Roboter verwendet, der die
> Bewegungen visualisiert. Zusammen mit dem synthetischen Signalgeber aus Phase
> 3.0 läuft damit **die gesamte Kette ohne jede Hardware** — synthetische Klötze,
> echte Komponenten, echter Attractor, echter IK-Controller, sichtbare Bewegung.
>
> Als Ersatz für die Greifer-Rückmeldung dienen die vorhandenen Testkomponenten
> `true_signal` / `toggle_signal` (sie liefern `has_object` von Hand), solange der
> echte Greifer nicht angeschlossen ist.
>
> **Was das nachweist:** Zustandsübergänge, Geometrie, Bahnform, Sicherheitsgate,
> Vorhersage, Kameramischung, Winkelwahl — und, sofern derselbe URDF und
> IK-Controller genutzt wird, auch das Verhalten nahe Singularitäten (B11).
>
> **Was es nicht nachweist:** reale Latenz, den Wert von `lead_offset_m` (B4),
> Kameraverhalten, Greiferzeiten. Diese bleiben dem Aufbau vorbehalten.

Die größte Komponente. **Nicht am Stück bauen.**

### 4a — Gerüst und stehende Posen

| | |
|---|---|
| **Ziel** | Zustandsautomat, Zielposen-Ausgang, Sicherheitsgate, TCP-Ringpuffer. Aktiv nur `WARTEN`, `ABLEGEN`, `ABBRUCH`. |
| **Voraussetzung** | 0.1, 0.2, 0.4; **B8** (Beobachtungshöhe) — 4a fährt sie bereits an, und sie ist an B6 Stufe 3 gekoppelt |
| **Prüft** | Verbindung zum Attractor, Werkzeugversatz, Sicherheitsgate, Startverhalten |
| **Wichtig** | Start im Zustand `ABBRUCH` — senkrecht hoch, dann Beobachtungspose. Damit ist der Start definiert, egal wo der Arm steht. Bei stehenden Zielen `lead_offset_m = 0`, sonst parkt der Roboter dauerhaft daneben. |
| **Abnahme** | Roboter fährt aus beliebiger Ausgangslage sicher zur Beobachtungspose und zur Ablagepose. Zielposen außerhalb des Arbeitsraums werden gedeckelt und gemeldet. |

### 4b — Verfolgen nur mit Basiskamera

| | |
|---|---|
| **Ziel** | `ANFAHREN` und `FOLGEN` mit `w = 0` |
| **Voraussetzung** | 4a, 3.3, 0.5; Bandwerte (B1) |
| **Kern** | Zielpose = Prädiktion + Vorhalt, begrenzt auf die Greifzone. Die Begrenzung erzeugt den Abfangkurs von selbst — Block stromaufwärts heißt Warten am Zonenrand **auf der Querposition des Blocks**. |
| **Hier kalibrieren** | `lead_offset_m` (B4): Block mitfahren lassen, Restabstand ablesen, Vorhalt anpassen bis null |
| **Abnahme** | Der Roboter fährt einem Block über die Greifzone hinterher und hält konstanten Abstand — ohne Absenken, ohne Greifen. |

### 4c — Roboterkamera aufschalten

| | |
|---|---|
| **Ziel** | `w`-Mischung mit Rampe, Rückfall, Identitätsprüfung |
| **Voraussetzung** | 4b, 2.2, 0.3; **B6 abgeschlossen** — 2.2 ist nur der Vertragsumbau, den eine `robot_cam` auch besteht, die dauerhaft `valid = 0` liefert. Ohne belegte Erkennung am Aufbau ist 4c nicht beurteilbar. |
| **Kern** | `korrektur = roboterkamera_welt − basiskamera_prädiktion`; `ziel = prädiktion + w · korrektur_gefiltert`. Die Korrektur ist nahezu konstant und damit stark glättbar — die absolute Position wäre es nicht. |
| **Schutz** | `max_korrektur_m`: zu große Korrektur → Messung verwerfen, `w = 0` (fängt einen falschen Block im Bild ab, R4). Rampe ~0,2 s beim Wechsel. |
| **Abnahme** | `w_wirksam` in `follower_status` zeigt den Quellenwechsel. Bei abgedeckter Roboterkamera läuft die Verfolgung mit `w = 0` weiter. |

### 4d — Voller Greifzyklus

| | |
|---|---|
| **Ziel** | `ABSENKEN`, `GREIFEN`, `HEBEN`, `ABLEGEN`, `LOESEN`, `picked_id` |
| **Voraussetzung** | 4c, 2.3; Greifhöhe (B7/B15/B17), Toleranzen (B18) |
| **Kern** | Greifhöhe `= bandoberflaeche + max(blockhoehe/2, min_greifhoehe)`. Beim Absenken läuft die Querregelung weiter; **wächst die Abweichung, zurück nach `FOLGEN`**. `GREIFEN` ist reines Mitfahren ohne Absenken. `HEBEN` behält das Mitfahren bis zur Freihöhe. |
| **Orientierung** | Einmal festlegen, beim Übergang nach `ABSENKEN` einfrieren. Von zwei gleichwertigen Handgelenkstellungen die nähere wählen. Modus 1 (fester Winkel) ist Spezialfall von Modus 2 — ein Codepfad. |
| **Abnahme** | Vollständiger Zyklus mit Modus 1 (gerade aufgelegte Blöcke), danach Modus 2. Fehlgriff führt zu `outcome = 1` und sauberem Rücksetzen. |

---

## Phase 5 — `interface_streamer`

| | |
|---|---|
| **Ziel** | Ein zusammengesetztes Bild für RViz |
| **Voraussetzung** | alle übrigen |
| **Umfang** | Zwei Debug-Bilder nebeneinander; darunter Zustandsname, Ziel-ID, `w_wirksam`, drei Regelabweichungen; darunter die Objektliste mit Status/gepickt/oob. Rate **5 Hz**. |
| **Begründung der Rate** | Zwei Bilder zu kopieren, zu skalieren und zu beschriften kostet dieselbe CPU, die die Echtzeit-Bildverarbeitung braucht. |
| **Priorität** | **Zuletzt.** Die einzige Komponente, ohne die das System vollständig funktioniert. |

---

## Phase 6 — Inbetriebnahme und Abstimmung

Reihenfolge der Labormessungen: `docs/review/offene-punkte.md`, Abschnitt B.
Kritischer Pfad: B13 (Uhrendrift) → B1 (Bandwerte) → B19 (Greifzone) →
B11 (Singularitäten) → B10 (Arbeitsraum) → B4 (Vorhalt) → B18 (Toleranzen).

---

## Dateien insgesamt

**Neu:**
- `roboter_tetris/contracts.py`
- `roboter_tetris/vectoring.py`
- `roboter_tetris/data_tracker.py`
- `roboter_tetris/priority_handler.py`
- `roboter_tetris/object_follower.py`
- `roboter_tetris/interface_streamer.py`
- je eine JSON in `component_descriptions/`
- je ein Test in `test/python_tests/`

**Geändert:**
- `roboter_tetris/base_cam.py`, `robot_cam.py`, `robot_cam_2.py`, `robotiq_gripper.py`
- die zugehörigen JSONs
- `setup.cfg` (fünf neue Registrierungen)

**Unangetastet:**
- `roboter_tetris/vision/*` — die Bildverarbeitung
- `roboter_tetris/Calibration/*` — anderes Projekt
- `CMakeLists.txt` — bereits korrekt

---

## Bewusste YAGNI-Entscheidungen

- **Kein eigenes Sicherheitsgate als Komponente** — bräuchte dieselben Daten,
  fügte Verzögerung und eine Fehlerquelle hinzu. Als abgegrenzte Funktion im
  Follower derselbe Nutzen.
- **Keine Ablage nach Farbe** — feste Ablagepose über einer Auffangkiste. Die
  Farbe liegt in den Daten, nachrüstbar ohne Schnittstellenänderung.
- **Kein `gripper_change`** (Vorpositionierung der Öffnungsweite) in der ersten
  Fassung. Die Breite liegt im `target`-Vertrag bereit.
- **Kein Wechsel des Zielobjekts** bei Auftauchen eines "besseren" — bei 2–3
  Blöcken kein Gewinn, aber zusätzliche Fehlerquelle.
- **Option C (bewegter Bezugsrahmen)** nicht als Primärweg. `lead_offset_m = 0`
  plus eine Leitung im Graphen genügen zum Umschalten, falls B3 sie bestätigt.
