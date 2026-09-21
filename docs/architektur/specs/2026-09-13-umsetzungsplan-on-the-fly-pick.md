# Umsetzungsplan: On-the-fly-Pick

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Umsetzungsbereit, mit benannten Vorbedingungen

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


## Stand der Umsetzung (21.09.2026)

| Phase | Schritt | Stand |
|---|---|---|
| 0 | Vorbedingungen | ✅ 15.09. |
| 1 | 1.1 `contracts.py` | ✅ — nach Nachtrag 6 auf S3/S4/S7/S10 nachgezogen |
| 2 | 2.1 `base_cam` · 2.2 `robot_cam`/`_2` · 2.3 Greifer · **2.4 Tracker** | ✅ |
| 3 | 3.0 `fake_objects.py` · **3.1 `vectoring`** | ✅ |
| 3 | **3.3 `priority_handler`** — vor 3.2 gebaut, weil er Ziel 4 umsetzt | ✅ |
| 3 | 3.2 `data_tracker` | ✅ |
| 4 | **4a** Gerüst und Start | ✅ |
| 4 | **4b** Verfolgen mit der Basiskamera | ✅ |
| 4 | **4c** Roboterkamera · **4d** Greifzyklus | ✅ |
| 5 | `interface_streamer` | ✅ — **damit sind alle Komponenten gebaut** |
| 6 | Inbetriebnahme | offen — **nächster Schritt, am Aufbau**; kritischer Pfad unten |

✅ heißt: **lokal getestet, am Aufbau noch nicht erprobt.** Nichts davon ist bisher
in AICA gelaufen; die erste Probe gehört an den Anfang der nächsten Sitzung am
Aufbau (`uebersicht/uebergabe.md` §6).

## Zweck

Rückgrat für die Erstellung aller Komponenten des On-the-fly-Pickvorgangs.
Gedacht für eine Sitzung, die **ohne Vorkontext** startet: jeder Schritt nennt
Voraussetzungen, betroffene Dateien und ein prüfbares Abnahmekriterium.

## Pflichtlektüre vor dem ersten Schritt

| Dokument | Warum |
|---|---|
| `docs/uebersicht/projektkontext.md` | Rahmenbedingungen, Abgrenzungen, was **nicht** angefasst wird. Zuerst lesen. |
| `ARCHITECTURE.md` | Verbindliche AICA-Regeln. Nicht optional. |
| `docs/architektur/entscheidungen.md` | Alle Architekturentscheidungen mit Begründung (Themen 1–7). **Nachtrag 3 ist Pflicht** — er korrigiert zwei Messfehler und zwei Planwidersprüche, die sonst erst in Phase 4 auffallen. **Nachtrag 6 ist ebenso Pflicht:** Er setzt die Projektvorgaben um (Geschwindigkeit wird geschätzt, nicht kalibriert) und korrigiert den Werkzeugversatz, der bei flachen Klötzen einen Crash ins Band erzeugt hätte. |
| `docs/architektur/datenvertraege.md` | Verbindliche Signalspezifikation |
| `docs/uebersicht/offene-punkte.md` | Was noch zu klären/messen ist |
| `docs/architektur/robot-cam-befunde.md` | Szene, Materialphysik und verworfene Wege der Roboterkamera. **Vor Schritt 2.2 und vor jeder Arbeit an der Beobachtungshöhe (B8) lesen.** |
| `docs/architektur/vorgaengerprojekt-abgleich.md` | Was aus dem Vorgängerarchiv beantwortet ist und was nicht. **Vor Phase 0 lesen** — es verschiebt die Quellen von 0.2 (C8) und 0.3 (C1/C4). |
| `docs/archiv/2026-06-01-robotiq-gripper-component-design.md` | Format- und Qualitätsvorlage für Komponenten-Specs |

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

> ✅ **Phase 0 ist seit 15.09.2026 vollständig abgeschlossen.** 0.1 (A7/A8: der
> Greifer steht nicht im URDF, geregelt wird der Flansch), 0.2 (C8 / Nachtrag 6
> Z7: **`flange_to_grip_point_m` = 0,235**, Flansch → Griffpunkt, gemessen — die
> 0,215 aus der UR-Steuerung sind deren TCP und dienen nur der Umrechnung; hier
> stand fälschlich 0,215 unter dem Namen `tool_offset_z_m`), 0.3 (C1: Hand-Auge ist
> Flansch → Kamera), 0.4 (A2: `cartesian_pose`), 0.5 (A1: Defaults abgelesen, als
> Anforderung in `uebersicht/einrichtung-projektanwendung.md` §2) und 0.6 (B13:
> Uhrendrift gemessen und behoben). Einzelheiten: `entscheidungen.md`, Nachtrag 4.

| Schritt | Aufgabe | Quelle | Blockiert |
|---|---|---|---|
| 0.1 | **Welches Frame regelt der IK-Velocity-Controller** — Flansch oder Greifpunkt? Steht der Greifer im URDF? (A7/A8) | AICA Studio; zuverlässiger: wie hat die **Vorgängergruppe** kommandiert (C9) — gleicher Aufbau | Phase 4 |
| 0.2 | **Werkzeugversatz Flansch → Greifpunkt** (C8) | Vorgängerprojekt, gleicher Greifer | Phase 4 |
| 0.3 | **Hand-Auge-Kalibrierung Roboterkamera** (C1), Konvention und Bezug klären (C4) | Vorgängerprojekt | Phase 4c |
| 0.4 | **Signaltyp des `attractor`-Eingangs** (`cartesian_state` vs. `cartesian_pose`, A2) | AICA Studio — was liefert `frame_to_signal.pose`? | Phase 4a |
| 0.5 | **Attractor-Parameter ablesen** (A1): `linear gain`, `max linear velocity`, Präzisionsschwellen | AICA Studio | Phase 4b |
| 0.6 | **Uhrendrift prüfen** (B13): `ros_zeit − header_stempel` über einige Minuten | Labor, eine Logzeile | Phase 2 |

> Schritt 0.1 war der wichtigste. Der Controller regelt den **Flansch** (A7/A8
> bestätigt); gäben wir die Blockposition als Ziel aus, führe der Greifer rund
> 23,5 cm zu tief — ins Band.

---

## Phase 1 — Fundament

### 1.1 `contracts.py` ✅

| | |
|---|---|
| **Ziel** | Ein gemeinsames Modul mit Kopflängen, Strides, Feldindizes und `pack_*`/`unpack_*` für alle elf Signale |
| **Voraussetzung** | keine |
| **Vorlage** | `docs/architektur/datenvertraege.md` — vollständig und verbindlich |
| **Dateien** | **neu** `source/roboter_tetris/roboter_tetris/contracts.py`<br>**neu** `source/roboter_tetris/test/python_tests/test_contracts.py` |
| **Abnahme** | Alle `pack_*`/`unpack_*` sind Round-Trip-getestet. `unpack_*` lehnt zu kurze/inkonsistente Arrays ab, statt auf halben Daten zu rechnen. Keine ROS-/cv2-Importe — rein testbar. |

**Regeln:** `unpack_*` prüft die Länge gegen `kopf + n·stride`. `unpack_*` rechnet
nicht um und interpretiert nicht — Extrapolation und Transformation macht der
Verbraucher. Keine Komponente indiziert von Hand in ein fremdes Array.

---

## Phase 2 — Bestehende Komponenten auf die Verträge bringen

Kleine, risikoarme Änderungen an laufendem Code. Jede einzeln testbar.

### 2.1 `base_cam` — Ausgabevertrag ✅

| | |
|---|---|
| **Ziel** | Ausgabe auf `[t, n, v_band] + n×9` umstellen, SI-Einheiten |
| **Änderung** | Kopf mit `t` aus `header.stamp`; mm → m beim Packen; `vy` vom Objektfeld in den Kopf; Rate auf 100 Hz; **`debug_image` auf `publish_on_step=False`** (N5) |
| **Nicht ändern** | Die Vision-Module unter `vision/` bleiben unangetastet, sie rechnen weiter in mm |
| **Dateien** | `roboter_tetris/base_cam.py`, `component_descriptions/roboter_tetris_base_cam.json` |
| **Abnahme** | Bestehende Tests laufen unverändert durch (sie prüfen nur `vision/*`). Ein neuer Test prüft das Packen gegen `contracts.py`. |

> `v_band` gehört in den Kopf, weil der Tracker die Geschwindigkeit global allen
> Tracks zuweist (`set_global_velocity`) — als Objektfeld ist der Wert
> irreführend.

### 2.2 `robot_cam` und `robot_cam_2` — Ausgabevertrag ✅

| | |
|---|---|
| **Ziel** | Ausgabe auf `[t, valid, x, y, z_band, orientation]` umstellen, SI-Einheiten |
| **Änderung** | `valid`-Flag ergänzen; Distanz-Gate über neue Parameter `min_belt_distance_m` / `max_belt_distance_m` auf `z_band`; mm → m; Rate 100 Hz; **`debug_image` auf `publish_on_step=False`** (N5). ⚠️ Die Rückprojektion **nicht** korrigieren — das macht der Follower (N1) |
| **Wichtig** | Bei `valid = 0` läuft `t` **weiter**. Nur so unterscheidet der Empfänger "Kamera arbeitet, sieht nichts" von "Kamera liefert nicht mehr". |
| **Nicht ändern** | Die **Detektionskerne** `belt_candidate_mask` (Farbe) und `edge_candidate_mask` (Kanten) — sie sind das, was der A/B-Test aus B6 vergleicht. Ebenso die **Rückprojektion** (N1, der Follower korrigiert sie; sonst doppelt). ⚠️ **Korrektur 20.09.2026:** Hier stand zusätzlich `localize_largest_blob`. Diese Sperre ist **aufgehoben** — die gemeinsame Auswahl- und Geometriestufe *muss* geändert werden (`robot-cam-befunde.md` §9.7/§9.8), und weil sie für beide Varianten identisch wirkt, bleibt der A/B-Test aussagekräftig. |
| **Vorher lesen** | `docs/architektur/robot-cam-befunde.md` — begründet die Defaults und nennt die Fallstricke (Seitenflächen-Physik, Nah-Gate, RealSense-Konfiguration) |
| **Dateien** | `roboter_tetris/robot_cam.py`, `robot_cam_2.py` + beide JSONs |
| **Abnahme** | Beide Varianten liefern denselben Vertrag und sind gegeneinander austauschbar. |

### 2.3 `robotiq_gripper` — zwei Bool-Ausgänge ✅

| | |
|---|---|
| **Ziel** | `motion_done` und `has_object` als `Bool`-Signale ergänzen |
| **Begründung** | Der Plan sah nur "Greifer zu" vor — ein Echo des Eingangs und als Rückmeldung wertlos. Der Follower braucht `has_object`, um einen Fehlgriff zu erkennen. |
| **Nicht ändern** | Worker-Thread, Port-pro-Operation-Muster (`ARCHITECTURE.md` §13), vorhandene Predicates |
| **Dateien** | `roboter_tetris/robotiq_gripper.py`, `component_descriptions/roboter_tetris_robotiq_gripper.json` |
| **Abnahme** | Bestehender Test läuft weiter; neuer Test prüft, dass `has_object` dem Predicate folgt. |

### 2.4 `base_cam` — Tracker misst die Längsposition (Nachtrag 6 / Z4) ✅

| | |
|---|---|
| **Ziel** | Außerhalb der Messregion die **gemessene** statt einer gerechneten Längsposition verwenden, sobald eine Detektion vorliegt |
| **Warum** | Sonst ist die Geschwindigkeitsschätzung in 3.1 zirkulär — sie bekäme die hineingesteckte Geschwindigkeit zurück —, und ein beim Aufsetzen kippender Klotz erscheint glatt, weil vorn aufgelegt wird |
| **Nicht ändern** | Die **Löschregel** (Z. 233–238). Sie schützt Tracks am Bildrand vor dem Abreißen; das Aufweiten der Region ist deshalb in N2 verworfen |
| **Dateien** | `roboter_tetris/vision/tracker.py` (Z. 168–173), `test/python_tests/test_base_cam_vision.py` |
| **Abnahme** | Ein Test belegt: Mit Detektion außerhalb der Region folgt `y` der Messung; ohne Detektion wird wie bisher fortgeschrieben; die Löschregel verhält sich unverändert. Am Aufbau: **B21**. |

---

## Phase 3 — Datenpfad (**ohne Aufbau und ohne Roboter** testbar)

### 3.0 Synthetischer Signalgeber (zuerst) ✅

| | |
|---|---|
| **Ziel** | Ein kleines Skript, das synthetische `objects`-Arrays veröffentlicht: drei Blöcke, die mit konstanter Geschwindigkeit über ein gedachtes Band wandern |
| **Voraussetzung** | 1.1 |
| **Nutzen** | Treibt `vectoring`, `priority_handler` und den Follower vollständig an — **ohne Kameras, ohne Roboter, ohne Aufbau**. Glättung, Plausibilität, Auswahl, Erreichbarkeit, Ziel-Lock und die gesamte Zustandsmaschine sind damit am Schreibtisch validierbar. |
| **Dateien** | **neu** `source/roboter_tetris/test/tools/fake_objects.py` (kein Komponentenpaket, reines Testwerkzeug) |
| **Abnahme** | Blöcke erscheinen, wandern, verlassen die Zone. ⚠️ **Geändert (Nachtrag 6 / Z3):** Statt eines angehaltenen Blocks — die gibt es nicht — simuliert das Werkzeug einen **beim Aufsetzen kippenden** Klotz (Positionssprung). Er muss in 3.1 so lange Status 3 behalten, bis er wieder gleichmäßig läuft. |

> Bei knapper Zeit am Aufbau ist das die wertvollste Einzelmaßnahme im Plan.

### 3.1 `vectoring` ✅

| | |
|---|---|
| **Ziel** | **Geschwindigkeitsschätzung (Ziel 3)**, Einschwing-Erkennung und geglättete Objektzustände |
| **Voraussetzung** | 1.1, 2.1, **2.4**. ~~B1~~ — die Geschwindigkeit wird hier geschätzt, B1 ist nur noch Gegenprobe |
| **Kern** | Je Objekt ein Geschwindigkeitsvektor aus der Positionshistorie; **Einschwingen**: Status 3, bis zwei aufeinanderfolgende Halbsekundenfenster dieselbe Geschwindigkeit messen, dann Status 0 — ein Status, kein eingefrorener Wert; **Pool** = gemeinsame Ausgleichsrechnung über alle Messungen seit dem Einschwingen aller Klötze des Durchlaufs = Bandgeschwindigkeit (Nachtrag 6 / Z9); Geometrie und Orientierung erst ab „final" mitteln; einzelne Ausreißer verwerfen, anhaltende Abweichung = Umkippen → Neustart; Verfall ausbleibender IDs |
| **Besonderheit** | Orientierung über den **verdoppelten Winkel** mitteln (`½·atan2(⟨sin2θ⟩, ⟨cos2θ⟩)`). Die Resultantenlänge ist das Gütemaß: unter der Schwelle gilt die Orientierung als unbrauchbar. Ein arithmetischer Mittelwert aus 0° und 90° wäre 45° — genau die Lage, in der der Greifer die Ecken erwischt. ⚠️ **Das Gütemaß schlägt bei quadratischen Klötzen nicht an** — deren Winkel friert der Tracker bereits ein. Vor der Festlegung von D11 die Korrektur in der `vectoring`-Spec lesen. |
| **Entfällt** | Parameter `K`, `W`, `J`, `H`, "Richtung halten", "Geschwindigkeit halten". ⚠️ Die **Regression** stand hier ebenfalls — sie ist zurück, denn sie *ist* die Geschwindigkeitsschätzung (Nachtrag 6 / Z2). Ebenso entfallen die Plausibilitätsstatus 1 und 2 (Z3). |
| **Abnahme** | Logiktests ohne ROS **gegen die Ground Truth aus 3.0**: geschätzte Geschwindigkeit trifft die eingestellte in Betrag und Richtung; kippender Klotz (13 und 25 mm) bleibt Status 3 und wird danach mit **korrekter** Geschwindigkeit final — der Regressionstest für den Fehler aus Z9; Pool mittelt über mehrere Klötze; `n_pool = 0` vor dem ersten finalen. Am Aufbau: Code 3 beim Auflegen, dann 0; geschätzte Bandgeschwindigkeit gegen B1 (Stoppuhr). |

### 3.2 `data_tracker` ✅

| | |
|---|---|
| **Ziel** | Gesamtliste mit `picked` / `out_of_bounds`, rein zur Diagnose |
| **Voraussetzung** | 1.1, 3.1 |
| **Wichtig** | **Blatt im Graphen** — keine Leitung zurück in den Regelpfad. Rate 10 Hz. |
| **Verfallsregel** | Einträge mit `picked`, `out_of_bounds` oder `present = 0` verschwinden eine Frist nach der letzten dieser Änderungen. Ohne sie wächst das Signal monoton. **Nachlauf** (`present = 0`), weil `picked_id` erst nach dem Ablegen kommt (Nachtrag 7 / T1). |
| **Abnahme** | Liste bildet auflegen/picken/herausfahren korrekt ab; Länge bleibt beschränkt. |

### 3.3 `priority_handler` ✅

| | |
|---|---|
| **Ziel** | Zielauswahl mit Erreichbarkeits- und Greifbarkeitsprüfung, Ziel-Lock |
| **Voraussetzung** | 1.1, 3.1; Greifzone (B19), Greifergeometrie (B15/B16). Die Geschwindigkeit kommt aus dem S3-Kopf, **kein Parameter** |
| **Erreichbarkeit** | `t_verfügbar = (greifebene − position)/v_belt` — gemessen bis zur **Greifebene**, nicht bis zum Zonenende (Nachtrag 6 / Z11); `v_belt` geschätzt aus S3; bei `n_pool = 0` gibt es noch keinen Schätzwert und damit kein Ziel<br>`t_benötigt = abstand/v_max + 3/K` — Absenken und Greifen stecken jetzt in der Lage der Greifebene<br>Kandidat ⟺ `t_verfügbar > faktor · t_benötigt`<br>**`abstand` = waagerecht Flansch → Anfahrpunkt** (Klotz, stromaufwärts auf `zone_upstream` begrenzt); dafür `cartesian_state` als Eingang (N4, Nachtrag 7 / H1) |
| **Greifbarkeit** | Höhe ≥ `min_greifbare_hoehe`; **Diagonale** ≤ `max_oeffnung − marge` — gilt für jeden Gierwinkel (Nachtrag 7 / H2) |
| **Auswahl** | Unter den Kandidaten das dringendste (kleinstes `t_verfügbar`) — die Regel aus dem Plan, nur eben unter den *erreichbaren* |
| **Zusätzlich** | `zone_upstream` als Feld 12 in S4 ausgeben (N3). ⚠️ **Greifzone muss in der Messregion von `base_cam` liegen** — B19 legt beides gemeinsam fest (N2). |
| **Ziel-Lock** | Einmal gewählt bleibt gewählt. Rückzug nur bei: gepickt oder verpasst (`picked_id`), ID verschwunden, Status zurück auf 3. **Erreichbarkeit ist Auswahl-, kein Abbruchkriterium** — sonst wird ein fast erfolgreicher Griff abgebrochen, wenn der Block beim Greifen die Zonengrenze überschreitet. |
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
> **Was es nicht nachweist:** reale Latenz, den Wert von `lead_time_s` (B4),
> Kameraverhalten, Greiferzeiten. Diese bleiben dem Aufbau vorbehalten.

Die größte Komponente. **Nicht am Stück bauen.**

### 4a — Gerüst und stehende Posen ✅

| | |
|---|---|
| **Ziel** | Zustandsautomat, Zielposen-Ausgang, Sicherheitsgate, TCP-Ringpuffer. Aktiv nur `WARTEN` und `ABBRUCH` ohne Klotz — **`ABLEGEN` kommt mit 4d** (Nachtrag 8 / F2). |
| **Voraussetzung** | 0.1, 0.2, 0.4; **B8** (Beobachtungshöhe) — 4a fährt sie bereits an, und sie ist an B6 Stufe 3 gekoppelt |
| **Prüft** | Verbindung zum Attractor, Sicherheitsgate, Startverhalten. Der Werkzeugversatz braucht Klotzpositionen und wird in 4b geprüft. |
| **Wichtig** | Start im Zustand `ABBRUCH` — senkrecht hoch, dann Beobachtungspose. Damit ist der Start definiert, egal wo der Arm steht. Bei stehenden Zielen ist der Vorhalt null — mit `v = 0` ergibt `v · lead_time_s` das von selbst, ein Sonderfall im Code entfällt. |
| **Abnahme** | Roboter fährt aus beliebiger Ausgangslage erst senkrecht hoch, dann zur Beobachtungspose. Zielposen außerhalb des Arbeitsraums werden gedeckelt und gemeldet. Ohne `ws_*`/`observe_*` lässt sich die Komponente nicht konfigurieren (Nachtrag 8 / F3). Das Log nennt den Bezugsrahmen von `robot_state` — er muss `world` sein. Die Ablagepose wird in 4d abgenommen. |

### 4b — Verfolgen nur mit Basiskamera ✅

| | |
|---|---|
| **Ziel** | `ANFAHREN` und `FOLGEN` mit `w = 0` |
| **Voraussetzung** | 4a, 3.3, 0.5. ~~Bandwerte (B1)~~ — die Geschwindigkeit kommt geschätzt über S4 |
| **Kern** | Zielpose = Prädiktion + Vorhalt auf Beobachtungshöhe — der Werkzeugversatz wirkt nur in z und kommt deshalb mit `ABSENKEN` in 4d (Nachtrag 9 / G1) —, begrenzt gegen `zone_upstream` (S4 Feld 12) — **nur stromaufwärts**, quer und stromabwärts wird nicht begrenzt (N3). Die Begrenzung erzeugt den Abfangkurs von selbst: Block stromaufwärts heißt Warten am Zonenrand **auf der Querposition des Blocks**. |
| **Hier kalibrieren** | `lead_time_s` (B4): Block mitfahren lassen, **`err_laengs` in `follower_status` ablesen** (positiv = Flansch voraus), Vorhaltzeit anpassen bis null; `timeout_track_s` dafür auf ≥ 10 s (Nachtrag 9 / G4, G7). Als **Zeit**, nicht als Strecke — dann bleibt der Vorhalt richtig, wenn die Bandgeschwindigkeit zwischen Durchläufen leicht abweicht (Nachtrag 6 / Z6) |
| **Abnahme** | Der Roboter fährt einem Block über die Greifzone hinterher und hält konstanten Abstand — ohne Absenken, ohne Greifen. |

### 4c — Roboterkamera aufschalten *(Verbesserung, parallel zu 4d — Nachtrag 6 / Z10)* ✅

| | |
|---|---|
| **Ziel** | `w`-Mischung mit Rampe, Rückfall, Identitätsprüfung |
| **Voraussetzung** | 4b, 2.2, 0.3; **B6 abgeschlossen** — 2.2 ist nur der Vertragsumbau, den eine `robot_cam` auch besteht, die dauerhaft `valid = 0` liefert. Ohne belegte Erkennung am Aufbau ist 4c nicht beurteilbar. |
| **Kern** | **Zuerst** die Höhenkorrektur `(x,y) · (z_band − blockhoehe)/z_band` — Pflicht, nicht Feinschliff: 6…39 mm Fehler, je Klotz verschieden, unterhalb von `max_correction_m` (N1). Danach `korrektur = roboterkamera_welt − basiskamera_prädiktion` und `ziel = prädiktion + w · korrektur_gefiltert`. Die Korrektur ist nahezu konstant und damit stark glättbar — die absolute Position wäre es nicht. |
| **Richtung Hand-Auge** | Verbindlich **Flansch → Kamera**. Die ~91,5°-Drehung vertauscht bei falscher Richtung x und y, statt ein Vorzeichen zu drehen — der Fehler sieht plausibel aus und ist es nicht. |
| **Schutz** | `max_korrektur_m`: zu große Korrektur → Messung verwerfen, `w = 0` (fängt einen falschen Block im Bild ab, R4). Rampe ~0,2 s beim Wechsel. |
| **Abnahme** | `w_wirksam` in `follower_status` zeigt den Quellenwechsel. Bei abgedeckter Roboterkamera läuft die Verfolgung mit `w = 0` weiter. |

### 4d — Voller Greifzyklus ✅

| | |
|---|---|
| **Ziel** | `ABSENKEN`, `GREIFEN`, `HEBEN`, `ABLEGEN`, `LOESEN`, `picked_id` |
| **Voraussetzung** | **4b**, 2.3; Greifhöhe (B7/B15/B17), Toleranzen (B18). ⚠️ **Nicht mehr 4c (Nachtrag 6 / Z10):** Der Greifzyklus wird zuerst mit der Basiskamera allein gebaut und getestet (`w = 0`, `require_robot_cam_for_grasp = false`). Sonst wäre ohne funktionierende Roboterkamera (B6) kein einziger Griff testbar. Die Genauigkeit hängt dann an C3; ein Fehlgriff endet mit `outcome = 1`, nicht mit einem Crash. |
| **Kern** | Greifhöhe `= bandoberflaeche + max(blockhoehe/2, min_greifhoehe)`, mit `min_greifhoehe = 0,015` (5 mm Luft, bis die 245 mm beim ersten Testgriff bestätigt sind) und `flange_to_grip_point_m = 0,235`. **`ABSENKEN` beginnt nur, solange der Block vor der Greifebene liegt** (S4 Feld 15); überschreitet er sie in `ANFAHREN` oder `FOLGEN`, Abbruch mit `outcome = 3` (Nachtrag 6 / Z11). Dieses Tor kommt erst mit 4d — in 4b darf der Roboter zum Einmessen des Vorhalts der ganzen Zone folgen. Beim Absenken läuft die Querregelung weiter; **wächst die Abweichung, zurück nach `FOLGEN`**. `GREIFEN` ist reines Mitfahren ohne Absenken. `HEBEN` behält das Mitfahren bis zur Freihöhe. |
| **Orientierung** | Einmal festlegen, beim Übergang nach `ABSENKEN` einfrieren. Von zwei gleichwertigen Handgelenkstellungen die nähere wählen. Modus 1 (fester Winkel) ist Spezialfall von Modus 2 — ein Codepfad. |
| **Abnahme** | Vollständiger Zyklus mit Modus 1 (gerade aufgelegte Blöcke), danach Modus 2. Fehlgriff führt zu `outcome = 1` und sauberem Rücksetzen. **Nach dem Greifen verschwindet die ID des gehobenen Klotzes aus `tracks` — der Zyklus muss trotzdem bis zur Ablage durchlaufen** (Nachtrag 6 / Z12). Ein künstlich ausgelöster Abbruch mit Klotz im Greifer legt ihn in der Kiste ab, statt ihn fallen zu lassen. Beobachten: Bleibt die Ziel-ID während `ABSENKEN` und `GREIFEN` erhalten (B22)? |

---

## Phase 5 — `interface_streamer` ✅

| | |
|---|---|
| **Ziel** | Ein zusammengesetztes Bild für RViz |
| **Voraussetzung** | alle übrigen |
| **Umfang** | Zwei Debug-Bilder nebeneinander; darunter Zustandsname, Ziel-ID, `w_wirksam`, drei Regelabweichungen; darunter die Objektliste mit Status/gepickt/oob. Rate **5 Hz**. |
| **Begründung der Rate** | Zwei Bilder zu kopieren, zu skalieren und zu beschriften kostet dieselbe CPU, die die Echtzeit-Bildverarbeitung braucht. |
| **Priorität** | **Zuletzt.** Die einzige Komponente, ohne die das System vollständig funktioniert. |

---

## Phase 6 — Inbetriebnahme und Abstimmung

**Reihenfolge am Aufbau: `uebersicht/uebergabe.md` §6** — dort Schritt für
Schritt. Die Punkte selbst: `uebersicht/offene-punkte.md`.

Kritischer Pfad: B13 (Uhrendrift, erledigt) → **B23** (Basiskamera und Roboter im
selben System) → **B21** (Tracker misst über den ganzen Sichtbereich) → B19
(Greifzone) → B11 (Singularitäten) → B10 (Arbeitsraum) → B4 (Vorhaltzeit) → D22
(Zeiten des Greifprozesses, trägt die Greifebene) → B18 (Toleranzen). Daneben für
4c: B6 → B8 → B24. **B1 ist nicht mehr auf dem kritischen Pfad** — die
Geschwindigkeit wird geschätzt, B1 prüft den Schätzer nur gegen.

---

## Dateien insgesamt

**Neu — je Komponente eine Schale und ein Logikmodul ohne ROS:**

| Komponente | Schale | Logik | Tests |
|---|---|---|---|
| Verträge | – | `contracts.py` | `test_contracts.py` |
| `vectoring` | `vectoring.py` | `track_estimation.py` | `test_vectoring.py` |
| `priority_handler` | `priority_handler.py` | `target_selection.py` | `test_target_selection.py`, `test_priority_handler.py` |
| `data_tracker` | `data_tracker.py` | `world_bookkeeping.py` | `test_world_bookkeeping.py`, `test_data_tracker.py` |
| `object_follower` | `object_follower.py` | `follower_logic.py` | `test_follower_logic.py`, `test_follower_camera_and_grasp.py`, `test_object_follower.py` |
| `interface_streamer` | `interface_streamer.py` | `interface_layout.py` | `test_interface_layout.py`, `test_interface_streamer.py` |

Dazu je eine JSON in `component_descriptions/` und `test/tools/fake_objects.py`
(synthetische Klötze, `test_fake_objects.py`).

**Geändert:**
- `roboter_tetris/base_cam.py`, `robot_cam.py`, `robot_cam_2.py`, `robotiq_gripper.py`
- `roboter_tetris/vision/tracker.py` (2.4), `vision/robot_detection*.py` (2.2, nur die gemeinsame Blob-Auswahl)
- die zugehörigen JSONs
- `setup.cfg` (fünf neue Registrierungen)

**Unangetastet:**
- die Detektionskerne der Roboterkamera, die Rückprojektion (N1) und `vision/board.py` (von der Kalibrierung genutzt)
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
- **Option C (bewegter Bezugsrahmen)** nicht als Primärweg. `lead_time_s = 0`
  plus eine Leitung im Graphen genügen zum Umschalten, falls B3 sie bestätigt.
