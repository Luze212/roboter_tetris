> Historische externe Übergabe, importiert am 23.09.2026 aus
> `UEBERGABE_FuE_Robotertetris.md`. Der Originaltext folgt unverändert.
> **Teilweise überholt:** Neuere lokale Entscheidungen L9–L14 ändern unter
> anderem Tracking, Zeitgrenzen, Hand-Auge-Kalibrierung und Arbeitsraum.
> Aktuelle Einordnung: [Codex-Einstieg](../uebersicht/codex-einstieg.md).
> Enthaltene Handlungsanweisungen sind Quelleninhalt, kein aktueller Arbeitsauftrag.

---

# Übergabe – FuE Robotertetris

**Gesichtete Branch:** `Codex_systemtest_Tobi` · **Stand:** 23.09.2026  
Kompakte Orientierung für ein neues Codex-Projekt am Roboter-PC. Die unten genannten Quelldateien bleiben für technische Entscheidungen maßgeblich.

Die frühere Datei `PROJECT_CONTEXT_FuE_Robotertetris.md` existiert nicht in der gesichteten Branch. Der vorhandene, verwendete Projektkontext ist `docs/uebersicht/projektkontext.md`.

## Einstieg und Vorrang

In dieser Reihenfolge lesen:

1. `docs/uebersicht/projektkontext.md` – Ziel, physischer Aufbau, Abgrenzungen und Arbeitsweise.
2. `docs/uebersicht/uebergabe.md` – operativer Stand, Messwerte und offene Arbeiten.
3. `docs/uebersicht/fahrplan-aufbau.md` – Reihenfolge, Abnahmen und Sicherheitsregeln am Aufbau.
4. `docs/uebersicht/systemgraph.md` – AICA-Graph, Zuständigkeiten und Ablauf eines Griffs.
5. `docs/architektur/entscheidungen.md` – normative Entscheidungen; neuere Nachträge gehen vor, besonders 6–13.
6. `docs/architektur/datenvertraege.md` – verbindliche Verträge S1–S10.

Aufgabenspezifisch zusätzlich:

- AICA-Konfiguration: `docs/uebersicht/einrichtung-projektanwendung.md`
- Roboterkamera: `docs/architektur/robot-cam-befunde.md`
- Arbeitsraum: `source/roboter_tetris/roboter_tetris/Safety/README.md`
- Extrinsik der Basiskamera: `source/roboter_tetris/roboter_tetris/Calibration/README.md`
- AICA-/Paketregeln: `ARCHITECTURE.md`

Die Dokumentenlandkarte steht in `docs/README.md`. Bei Widerspruch gelten `docs/architektur/entscheidungen.md` und `docs/architektur/datenvertraege.md`.

## Projektziel und Systembild

Das Projekt soll erreichen, dass ein **UR10e farbige, unterschiedlich große Klötze von einem laufenden Förderband greift, ohne das Band anzuhalten**. Die Klötze werden vorn von Hand aufgelegt und durchlaufen den Arbeitsbereich. Der Kern ist damit nicht die Ablage, sondern ein nachvollziehbarer, kontrollierter Pickvorgang während der Bewegung. Quelle: `docs/uebersicht/projektkontext.md`, §1 und §5.

Die offiziellen Ziele sind:

1. Ansteuerung des UR10e mit AICA.
2. Entwicklung eines schnellen Kalibrierverfahrens als getrenntes Projekt eines Kommilitonen; dieses Projekt ist Abnehmer der Ergebnisse.
3. Laufzeit-Schätzung von Objekt- und Bandgeschwindigkeit sowie Positionsberechnung aus Bilddaten.
4. Priorisierung des als Nächstes zu greifenden Objekts und Bahnplanung für kontrolliertes Greifen.

Zusätzlich soll der Prozess live nachvollziehbar dargestellt werden. Quelle: `docs/uebersicht/projektkontext.md`, §1; `docs/architektur/entscheidungen.md`, Nachtrag 6 / Z1–Z5, Z8.

Der reale Aufbau besteht aus einem UR10e neben dem Band, einem direkt per USB/Modbus gesteuerten Robotiq-2F-140-Greifer, einer L515-Basiskamera am Bandanfang auf beweglichem Gestell und einer D435i-Roboterkamera fest am Flansch. Das Band ist grün-türkis, läuft mit konstanter, nicht einstellbarer Geschwindigkeit und die Klötze sind rot, blau oder weiß; ihre matten Oberseiten und spiegelnden Seitenflächen erschweren besonders die Roboterkamera. Quelle: `docs/uebersicht/projektkontext.md`, §5; `docs/architektur/robot-cam-befunde.md`.

Das Vorgängerprojekt erkannte und verfolgte Klötze, griff aber nicht rückgekoppelt im Lauf: Es wartete auf eine berechnete Ankunftszeit. Das aktuelle Projekt baut deshalb ausdrücklich Geschwindigkeitsschätzung, Zielauswahl und einen regelnden Follower neu auf. Quelle: `docs/uebersicht/projektkontext.md`, §§1–2; `docs/architektur/vorgaengerprojekt-abgleich.md`, §1.

Regelpfad: L515-Basiskamera → `base_cam` → `vectoring` → `priority_handler` → `object_follower` → AICA-Attractor/IK-Controller → UR10e. Die D435i-Roboterkamera korrigiert optional den Follower. `robotiq_gripper` bedient den Greifer; `data_tracker` und `interface_streamer` sind Diagnoseblätter ohne Rückwirkung. Details: `docs/uebersicht/systemgraph.md`; Signaldefinitionen: `docs/architektur/datenvertraege.md`.

Wichtige Zusammenhänge:

- Laufzeitpositionen verwenden `world` (= Roboterbasis); die Roboterpose ist der Flansch `ur_tool0`, nicht der TCP der Robotersteuerung. Quellen: `docs/uebersicht/systemgraph.md`, „Bezugssysteme“; `docs/architektur/entscheidungen.md`, Thema 3.
- `vectoring` schätzt Bandgeschwindigkeit aus Bilddaten; die Messung von etwa 0,13 m/s ist nur Gegenprobe. Quelle: `docs/architektur/entscheidungen.md`, Nachtrag 6 / Z2 und Nachtrag 13 / L7.
- Gekoppelte Parameter (Attractor/Vorhalt, Geschwindigkeitsgrenzen, Sinkzeit/Greifebene, Greifhöhe) müssen gemeinsam passen. Tabelle: `docs/uebersicht/systemgraph.md`.

## Entwicklungsstand und abgeschlossene Arbeiten

- Alle vorgesehenen eigenen Komponenten sind gebaut und in AICA geladen; der Datenpfad bis `data_tracker` wurde am Aufbau geprüft. Quelle: `docs/uebersicht/systemgraph.md`.
- `base_cam`, `vectoring`, `priority_handler`, `data_tracker`, Robotiq-Greifer sowie Attractor-/IK-Kette laufen am Aufbau. Follower und Roboterkamera sind gebaut, aber noch nicht als vollständiger Bewegungsablauf am realen Roboter abgenommen. Quellen: `docs/uebersicht/uebergabe.md`, Abschnitt 2; `docs/uebersicht/fahrplan-aufbau.md`, „Stand nach Termin B“.
- B23 ist erledigt: Basiskamera und Roboter liegen mit einer Übergangskalibrierung im selben `world`-System; dokumentierte Positionsabweichung höchstens 6 mm, auch bei hohen Klötzen. Quelle: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L6.
- Die Basiskamera wurde hinsichtlich Zeitdomäne, Warteschlangen und Last verbessert: reduzierter Aufbau plus Queue-Tiefe 1 ergab rund 7,1 neue Messungen/s, 139 ms medianes Alter bei Empfang und bis 506 ms vor der nächsten Messung. Quelle: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L2.
- Alle Python-Komponenten teilen sich einen Prozess; unnötige Blöcke, hohe Raten und paralleles Mitlesen senken die Messrate. Quellen: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L2–L3; `docs/uebersicht/fahrplan-aufbau.md`.

### Komponentenstand und Rollen

| Bereich | Erledigter, belegter Stand | Für Details lesen |
|---|---|---|
| Objekterfassung und Tracking | `base_cam` liefert Objekte; `vectoring` schätzt Geschwindigkeiten pro Objekt und als Pool. Ein Objekt ist erst nach dem Einschwingen ein auswählbarer Kandidat. | `docs/architektur/specs/2026-09-13-vectoring-component-design.md`; `docs/architektur/datenvertraege.md`, S1 und S3 |
| Auswahl | `priority_handler` prüft Greifbarkeit und Erreichbarkeit, wählt das dringendste erreichbare Ziel und hält es bis zum Abschluss fest. Die Greifebene verhindert zu spät gestartete Griffe. | `docs/architektur/specs/2026-09-13-priority-handler-component-design.md`; `docs/architektur/entscheidungen.md`, Nachtrag 6 / Z11 und Nachtrag 7 |
| Bewegungsablauf | `object_follower` ist lokal getestet und in den Stufen 4a–4d umgesetzt: Anfahren, Folgen, Absenken, Greifen, Heben, Ablegen und Lösen; beim Start und bei Problemen nutzt er den Abbruchpfad. | `docs/architektur/specs/2026-09-13-object-follower-component-design.md`; `docs/architektur/entscheidungen.md`, Nachträge 8–10 |
| Greifer | `robotiq_gripper` liefert zusätzlich zu seiner Ansteuerung `motion_done` und `has_object`; der Greifer selbst gehört nicht zum UR10e-URDF. | `docs/uebersicht/systemgraph.md`; `docs/uebersicht/offene-punkte.md`, A7/A8 |
| Diagnose | `data_tracker` führt nur Anzeige-/Buchhaltungszustand, `interface_streamer` erzeugt ein RViz-Übersichtsbild. Beide sind absichtlich keine Regelpfad-Abhängigkeiten. | `docs/architektur/specs/2026-09-13-data-tracker-component-design.md`; `docs/architektur/specs/2026-09-13-interface-streamer-component-design.md` |

Die Komponenten-Spezifikationen sind abgeleitet, nicht normativ. Sie sind laut `docs/README.md` nach erfolgreichem Aufbau-Lauf der jeweiligen Komponente zum Löschen vorgesehen; für aktuelle Verträge stets `entscheidungen.md` und `datenvertraege.md` verwenden.

### Gemessene, aber vor Ort zu prüfende Aufbauwerte

| Größe | Dokumentierter Wert / Status | Quelle |
|---|---|---|
| Bandoberfläche in `world` | 53,6 mm; Ebenheit ±1 mm, systematische Unsicherheit ±5 mm | `docs/uebersicht/uebergabe.md`, Abschnitt 3 |
| Flansch → Backenspitze / Griffpunkt | 245 mm / 235 mm. Der Wert 215 mm ist nur der TCP der UR-Steuerung und kein Griffpunkt-Parameter. | `docs/uebersicht/uebergabe.md`, Abschnitte 2–3; `docs/architektur/entscheidungen.md`, Nachtrag 6 / Z7 |
| Transferhöhe | 0,49 m Flanschmaß; berücksichtigt den gehaltenen Klotz | `docs/uebersicht/uebergabe.md`, Abschnitt 2; `docs/architektur/entscheidungen.md`, Nachtrag 10 / J1 |
| Ablagepose | Flansch: x −316,49 mm, y +476,21 mm, z +419,71 mm; vor fester Verwendung bei laufendem Roboterprogramm gegenprüfen | `docs/uebersicht/uebergabe.md`, Abschnitt 3; `docs/uebersicht/einrichtung-projektanwendung.md`, Abschnitt 8 |
| Band | etwa 0,13 m/s; in `world` ungefähr von y +1,08 bis −0,375 m | `docs/architektur/entscheidungen.md`, Nachtrag 13 / L7 |
| Basiskamerabild | ungefähr y +0,46 bis +1,03 m; Greifzone liegt bewusst dahinter | `docs/architektur/entscheidungen.md`, Nachtrag 13 / L4 und L7 |
| Basiskamera-Extrinsik | dokumentierte Übergangswerte: x −0,7787, y 0,7934, z 0,9163 m; Roll 179,46°, Pitch 0,45°, Yaw 179,76° | `docs/architektur/entscheidungen.md`, Nachtrag 13 / L6 |

Alle Höhen in dieser Tabelle sind Flanschmaße. Vor Ort keine TCP-Werte der UR-Steuerung als Ersatz einsetzen.

## Daten- und Bewegungslogik, die nicht versehentlich geändert werden sollte

- S1–S10 sind eine gemeinsame Schnittstelle. Feldreihenfolge, Strides, Einheiten und gemeinsame Hilfsregeln liegen ausschließlich in `docs/architektur/datenvertraege.md` und `contracts.py`; bei Signaländerungen zuerst den Vertrag ändern, dann den Code nachziehen.
- Die Datenflüsse sind bewusst azyklisch. `picked_id` ist nur ein diskretes Ereignis vom Follower an Auswahl und Buchhaltung; `data_tracker` darf nie wieder eine Voraussetzung für Auswahl oder Tracking werden. Quelle: `docs/uebersicht/systemgraph.md`; `docs/architektur/entscheidungen.md`, Thema 4.
- `target_pose` ist in AICA `cartesian_pose`, nicht `cartesian_state`. Die in ROS gleiche zugrunde liegende Nachricht verhindert keinen AICA-Typfehler beim Verdrahten. Quelle: `docs/uebersicht/offene-punkte.md`, A2; `docs/architektur/datenvertraege.md`, S6.
- Der Follower folgt einer Zielpose, kein direkt ausgegebenem Twist. Der Attractor erzeugt den Twist; der Vorhalt ist deshalb eine Zeit (`lead_time_s`), die zur Attractor-Verstärkung passen muss. Quelle: `docs/architektur/specs/2026-09-13-object-follower-component-design.md`, „Regelkette“; `docs/architektur/entscheidungen.md`, Thema 1 und Nachtrag 6 / Z6.
- Eine Roboterkamera-Messung wird als gefilterte Korrektur auf die Basiskamera-Prädiktion eingeblendet, nicht als ungefilterte absolute Position. Während Absenken und Greifen wird diese Korrektur eingefroren. Quelle: `docs/architektur/specs/2026-09-13-object-follower-component-design.md`, „Kameramischung“; `docs/architektur/entscheidungen.md`, Nachtrag 10 / J3.
- Sobald der Greifer einen Klotz hält, darf ein Abbruch ihn nicht öffnen und fallen lassen: Der Follower hebt an, fährt zur Ablage und öffnet dort. Quelle: `docs/architektur/specs/2026-09-13-object-follower-component-design.md`, „Zustandsautomat“; `docs/architektur/entscheidungen.md`, Nachtrag 6 / Z12.

## Einschränkungen, Risiken und offene Fragen

- **C3 / endgültige Extrinsik:** Die aktiven Werte sind nur Übergangskalibrierung. `Calibration/calibration.json` bleibt `legacy_initial_values`; Validierung mit unabhängigen Prüfpunkten fehlt. Quellen: `source/roboter_tetris/roboter_tetris/Calibration/README.md`; `docs/architektur/entscheidungen.md`, Nachtrag 13 / L6.
- **Roboterkamera:** Nur `robot_cam_2` (Kantenvariante) wird weiterverfolgt. Banddistanz wird aus Bildmedian bestimmt, ist am Aufbau aber noch ungeprüft. Die bekannte Hauptursache ist falsche Konturauswahl bzw. fehlende ROI. Quellen: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L5; `docs/architektur/robot-cam-befunde.md`, §9.8.
- **Datenlücke vor dem Griff:** Greifzone und Wartebereich liegen absichtlich außerhalb des Basiskamerabildes, damit offene Backen nicht als Objekte erkannt werden. Track-Fortführung und Timeouts/Extrapolation müssen dafür noch robust angepasst werden. Quelle: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L1 und L4.
- **Follower-Parameter:** `max_extrapolation_s` (0,2 s) und mehrere 0,5-s-Grenzen decken den gemessenen Datenhorizont nicht sicher ab. Vor Bewegungsinbetriebnahme an reale Rate/Latenz und `fake_objects.py` anpassen. Quellen: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L1; `docs/uebersicht/fahrplan-aufbau.md`.
- **Arbeitsraum:** Die versionierte Sicherheitsquelle enthält nur Platzhalter. Virtuelle Werte dürfen nicht am echten Roboter verwendet werden. Quellen: `source/roboter_tetris/roboter_tetris/Safety/README.md`; `docs/uebersicht/fahrplan-aufbau.md`, Abschnitt 2.
- **Höhenkorrektur:** `top_depth_bias_mm = 11,5` ist umgesetzt, aber nach dem nächsten Build an flachem und hohem Klotz zu bestätigen. Quelle: `docs/architektur/entscheidungen.md`, Nachtrag 13 / L6 und L8.

## Besonderheiten für Tests am Roboter

- Nur nach dem Ablauf und den Abnahmekriterien aus `docs/uebersicht/fahrplan-aufbau.md`, Abschnitte 2–4, arbeiten. Reale Bewegung erst mit dokumentiertem Arbeitsraum; beim Erstlauf ist Not-Aus-Bereitschaft vorgesehen.
- Neue Paket-Standardwerte gelten nur für neu eingefügte AICA-Blöcke. Nach Build prüfen, ob tatsächlich der neue Stand geladen ist. Quellen: `docs/uebersicht/einrichtung-projektanwendung.md`, Einleitung und §1; `docs/architektur/entscheidungen.md`, Nachtrag 13 / L8.
- RealSense-Zeitdomäne prüfen: Bei der Basiskamera müssen `rgb_camera.global_time_enabled` und `depth_module.global_time_enabled` wirksam `true` sein. Knotennamen anhand der Seriennummer bestimmen, nicht aus einer anderen Anwendung übernehmen. Details: `docs/uebersicht/einrichtung-projektanwendung.md`, §1.
- Roboterkamera vor dem Test mit deaktivierter Belichtungsautomatik betreiben. Quellen: `docs/uebersicht/einrichtung-projektanwendung.md`, §1; `docs/architektur/robot-cam-befunde.md`, §9.3.
- Anwendung schlank halten: keine unkonfigurierten Kamerablöcke, `interface_streamer` nur bei Bedarf und nur ein Leseprozess gleichzeitig. Quellen: `docs/uebersicht/fahrplan-aufbau.md`; `docs/architektur/entscheidungen.md`, Nachtrag 13 / L2–L3.

### Vor-Ort-Abnahmen und Reihenfolge

| Priorität | Noch zu klären bzw. abzunehmen | Bedeutung und dokumentierter Anhaltspunkt | Maßgebliche Quelle |
|---|---|---|---|
| Rot | C3: endgültige Extrinsik der Basiskamera | Die Übergangskalibrierung überbrückt C3 nur, solange Kamera, Roboterbasis und deren starre Beziehung unverändert bleiben. | `docs/uebersicht/offene-punkte.md`; `source/roboter_tetris/roboter_tetris/Calibration/README.md` |
| Hoch | Höhe nach `top_depth_bias_mm` | Nach dem Build mit 25- und 100-mm-Klotz bestätigen; der aktuelle Korrekturwert 11,5 mm ist noch nicht abschließend abgenommen. | `docs/architektur/entscheidungen.md`, Nachtrag 13 / L6; `docs/uebersicht/fahrplan-aufbau.md` |
| Hoch | B21 und Block 3 | Tracker im gesamten Sichtbereich prüfen und die gepoolte Laufzeit-Schätzung bei laufendem Band gegen die Stoppuhr-Gegenprobe abnehmen. | `docs/uebersicht/offene-punkte.md`, B1/B21; `docs/uebersicht/fahrplan-aufbau.md`, Block 3 |
| Hoch | Datenweg hinter dem Basiskamerabild | Extrapolation, Track-Fortführung und Zeitgrenzen korrigieren, bevor ein echter Follower-Lauf darauf vertraut. | `docs/architektur/entscheidungen.md`, Nachtrag 13 / L1 und L4 |
| Mittel | B6/B8/B24: Roboterkamera | `robot_cam_2` über ruhendem Klotz testen, Beobachtungshöhe bestimmen und Zeitdomäne der D435i prüfen. Erst danach die Kamera-Korrektur in Stufe 4c schrittweise gewichten. | `docs/uebersicht/fahrplan-aufbau.md`, Blöcke 6 und 8; `docs/uebersicht/offene-punkte.md` |
| Mittel | B10/B11/B19: Arbeitsraum, Singularitäten, Greifzone | Den Arm am Aufbau manuell entlang Band/Beobachtungs-/Greifhöhe führen, Grenzen dokumentieren und erst dann reale Follower-Bewegungen zulassen. | `docs/uebersicht/fahrplan-aufbau.md`, Block 5; `source/roboter_tetris/roboter_tetris/Safety/README.md` |
| Mittel | B4 und Realroboter-Stufen 4a/4b/4d | Zunächst virtueller Roboter mit `fake_objects.py`; danach reale Bewegung stufenweise. `lead_time_s` über `err_laengs` einmessen. | `docs/uebersicht/fahrplan-aufbau.md`, Blöcke 4 und 7; `docs/uebersicht/offene-punkte.md`, B4 |
| Mittel | B9/B15/D22/D23 | Ablagepose bei laufendem Programm prüfen, ersten Griff für Greifhöhen bestätigen, Ablaufzeiten und Backenwinkel messen. | `docs/uebersicht/uebergabe.md`, Abschnitt 6; `docs/uebersicht/offene-punkte.md` |

### AICA-Konfiguration mit hoher Fehlerwirkung

- Für die Basiskamera sind `rgb_camera.global_time_enabled` und `depth_module.global_time_enabled` kritisch. Die L515 kann sonst in einer abweichenden bzw. driftenden Zeitdomäne stempeln; `base_cam` verwirft anschließend Frames. `camera_node` muss dem per Seriennummer ermittelten Basiskamera-Knoten entsprechen. Quelle: `docs/uebersicht/einrichtung-projektanwendung.md`, §1.
- Die Serials sind Basiskamera L515 `f1370107` und Roboterkamera D435i `241122074842`; Knotennamen wechseln je AICA-Anwendung. Quelle: `docs/uebersicht/einrichtung-projektanwendung.md`, §1.
- Die bindende lineare Geschwindigkeitsgrenze ist der IK-Velocity-Controller. Ein gedrosselter Erstlauf mit 0,10 m/s kann ein Band mit etwa 0,13 m/s nicht einholen; dafür zuerst synthetische, langsamere Ziele verwenden. Quelle: `docs/uebersicht/fahrplan-aufbau.md`, Abschnitt 2; `docs/uebersicht/einrichtung-projektanwendung.md`, §2.
- Follower-Arbeitsraum (`ws_*`) und Beobachtungspose (`observe_*`) sind Pflichtparameter ohne reale Defaultwerte. Platzhalter aus der Einrichtungsdokumentation sind ausdrücklich nur für den virtuellen Roboter. Quelle: `docs/uebersicht/einrichtung-projektanwendung.md`, §§1 und 9.
- Die Parameter des `priority_handler` für Attractor-Gain und Maximalgeschwindigkeit müssen dem tatsächlich verdrahteten Attractor und IK-Controller entsprechen, sonst ist die Erreichbarkeitsprüfung falsch. Quelle: `docs/uebersicht/einrichtung-projektanwendung.md`, §2; `docs/uebersicht/systemgraph.md`.

### Umgang mit dem Vorgängerarchiv

`UR10_Pick_ws` ist eine reine Informationsquelle und kein Architekturvorbild: Es verwendet eine blockierende, zeitgesteuerte Zustandsmaschine statt des AICA-Regelpfads. Insbesondere dürfen seine `time.sleep()`-basierte Ablaufsteuerung, die Auswahl nur des ersten Listenobjekts und seine TCP-Posen nicht in die aktuelle Lösung zurückübertragen werden. Quelle: `docs/architektur/vorgaengerprojekt-abgleich.md`, §§1 und 5.

Belegt und nutzbar sind dagegen die Hand-Auge-Konvention **Flansch → Kamera** mit x = 0,1087 m, y = −0,03436 m, z = −0,05987 m und rpy ungefähr (1,66°, 1,56°, 91,5°), sowie der Greiferhub 0–130 mm als Startwert. Quelle: `docs/architektur/vorgaengerprojekt-abgleich.md`, §2. Die Richtung der Hand-Auge-Transformation ist kritisch; eine Vertauschung wirkt wegen der Drehung um rund 90° plausibel, ist aber falsch.

Der frühere Arbeitsraum darf nicht direkt übernommen werden: Er bezog sich auf den TCP und möglicherweise auf einen um 180° gedrehten UR-Frame, während die aktuelle Sicherheitsquelle Flanschgrenzen in `world` verlangt. Quelle: `docs/architektur/vorgaengerprojekt-abgleich.md`, §6, Fallen 5 und 5b; `source/roboter_tetris/roboter_tetris/Safety/README.md`.

### Dokumentierte Betriebs- und Änderungsgrenzen

Die operative Übergabe der Branch (`docs/uebersicht/uebergabe.md`, Abschnitt 1) beschreibt folgende Arbeitsgrenzen: Das Vorgängerarchiv `UR10_Pick_ws` ist nur lesbar zu benutzen; Builds, Images, AICA-Laden und Git-Operationen bleiben beim Nutzer. `ARCHITECTURE.md` ist beim Ändern eigener Komponenten bindend. `source/roboter_tetris/roboter_tetris/Calibration/` und `vision/board.py` gehören zum fremden Kalibrierprojekt und dürfen nicht ohne ausdrückliche neue Freigabe geändert werden.

## Empfohlene nächsten Schritte

Die verbindliche Detailreihenfolge steht in `docs/uebersicht/fahrplan-aufbau.md`. Daraus folgt:

1. Nach dem Build Basiskamera neu einfügen bzw. aktive Werte prüfen; Höhen an 25- und 100-mm-Klotz gegenprüfen.
2. Lastarme Anwendung aufbauen und Zeitstempel, Bildraten sowie Datenpfad kontrollieren.
3. Bei laufendem Band die Geschwindigkeitsschätzung gegen etwa 0,13 m/s prüfen.
4. `robot_cam_2` über einem ruhenden Klotz außerhalb des Basiskamerabildes prüfen.
5. Vor Follower-/Realrobotertests Latenz-/Extrapolationsgrenzen, Datenführung außerhalb des Basiskamerabilds, Greifzone und `fake_objects.py` aktualisieren.
6. Arbeitsraum und Singularitäten nach `Safety/README.md` dokumentieren; erst danach Follower-Stufen am echten Roboter ausführen.

Diese Schritte sind aus dem aktuellen Fahrplan abgeleitet; sie bestätigen nicht, dass die jeweiligen Voraussetzungen bereits erfüllt sind.
