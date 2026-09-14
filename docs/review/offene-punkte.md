# Offene Punkte, Tests und Vor-Ort-Aufgaben

Arbeitsliste für alles, was **nicht** am Schreibtisch entschieden werden kann.
Wächst mit jedem bearbeiteten Thema. Rein lokal.

**Legende Status:** `offen` · `in Arbeit` · `erledigt`
**Legende Dringlichkeit:** 🔴 blockiert die Umsetzung · 🟡 blockiert die Inbetriebnahme · 🟢 Feinschliff

---

## A — In AICA Studio ablesen (kein Test, nur nachschauen)

| ID | Punkt | Warum | Dringl. | Status |
|---|---|---|---|---|
| A1 | Parameterwerte des Signal Point Attractors: `linear gain`, `angular gain`, `max linear velocity`, `max angular velocity`, `linear precision threshold`, `angular precision threshold` | `linear gain` geht direkt in die Vorhaltrechnung (`lead_offset = v_band / K`). Ohne den Wert kann der Vorhalt nicht vorbelegt werden. | 🔴 | offen |
| A2 | Exakter Signaltyp des `attractor`-Eingangs und des `pose`-Ausgangs von `frame_to_signal` — `cartesian_state` oder `cartesian_pose`? | Legt fest, welchen Typ `object_follower` ausgeben muss, damit sich die Leitung im Graphen überhaupt verbinden lässt. AICA prüft Typgleichheit. | 🔴 | offen |
| A3 | Lässt sich am `base_frame`-Eingang des Attractors ein **Signal** anschließen, oder nur ein statischer Frame? | Entscheidet, ob Option C (bewegter Bezugsrahmen) überhaupt möglich ist. | 🟡 | offen |
| A4 | Sind die drei linearen Gains einzeln gesetzt oder gleich? | Bei ungleichen Gains wird der Vorhalt achsabhängig und bei schräger Bandrichtung zur Matrixrechnung. Empfehlung: linear isotrop halten. | 🟡 | offen |
| A5 | Publiziert der `robot_state_broadcaster` neben `cartesian_state` auch TF für die Roboterglieder? | Alternative zur eigenen TCP-Historie im `object_follower`. Nur relevant, falls wir doch auf `tf2`-Lookup gehen wollen. | 🟢 | offen |
| A6 | Welche Rate hat die Kamera-Komponente tatsächlich (Bilder/s bei Farbe und Tiefe)? | Geht in das Latenzbudget und in die Wahl der Follower-Rate ein. | 🟡 | offen |
| A7 | **Steht der Robotiq-Greifer im URDF?** Welches URDF ist im Hardware-Interface eingetragen? | Entscheidet, ob der IK-Controller den Flansch oder den Greifpunkt regelt. Regelt er den Flansch und wir geben die Blockposition als Ziel aus, fährt der Greifer ~20 cm zu tief — ins Band. **Indiz: In Studio ist nur der Flansch zu sehen, also vermutlich nicht im URDF.** | 🔴 | offen |
| A8 | Welches Frame meldet der `robot_state_broadcaster` als `cartesian_state`? | Muss dasselbe sein wie das geregelte Frame, sonst ist der Regelkreis inkonsistent und der TCP-Ringpuffer speichert die falsche Pose. | 🔴 | offen |
| A9 | Hat der IK-Velocity-Controller eine Dämpfung nahe Singularitäten und/oder eine Kommando-Zeitüberwachung? | Zusätzliche Reserve. Das Konzept kommt ohne aus, aber gut zu wissen. | 🟢 | offen |
| A11 | ~~Kann AICA mit simulierter Hardware laufen?~~ | **beantwortet:** AICA selbst läuft real, aber statt des echten Roboters kann ein **virtueller Roboter** verwendet werden, der die Bewegungen visualisiert. Damit sind die Follower-Stufen 4a–4c am Schreibtisch prüfbar. | – | erledigt |
| A10 | Gelenkgeschwindigkeitsgrenzen im Hardware-Interface | Nichts angepasst → vermutlich Herstellergrenzen des UR10e, also vorhanden, aber nicht auf den Aufbau eingeschränkt. Unterste Absicherung. | 🟡 | offen |

---

## B — Im Labor messen / testen

| ID | Punkt | Verfahren | Dringl. | Status |
|---|---|---|---|---|
| B1 | **Bandgeschwindigkeit + Bandrichtung** bestimmen | **Kein eigener Kalibriervorgang nötig.** Einmal das `objects`-Signal während eines normalen Testlaufs mitschneiden (mehrere Blöcke, volle Durchquerung des Sichtfelds), Gerade und Geschwindigkeit am Schreibtisch auswerten. Genauigkeit bei ~500 mm Bahnlänge und 30–50 Messungen: Richtung deutlich unter 0,5°. Gegenprobe der Geschwindigkeit mit Markierung auf dem Band und Stoppuhr. | 🔴 | offen |
| B3 | **Option C verifizieren** (5-Minuten-Test) | Roboter still, Attractor auf eine feste Pose, `base_frame` mit einem Signal versorgen, das eine Position **und eine lineare Geschwindigkeit** trägt. Bewegt sich der Roboter → Option C bestätigt, `lead_offset_m` kann auf 0. Bewegt er sich nicht → bei Option A bleiben. | 🟡 | offen |
| B4 | **Vorhalt kalibrieren** (nur bei Option A) | Block mitfahren lassen, Restabstand TCP↔Block im Debug-Bild ablesen, `lead_offset_m` anpassen bis der Abstand null ist. Ersetzt die Kenntnis des exakten Gains. | 🟡 | offen |
| B5 | **Kameralatenz** bestimmen | Verfahren siehe Thema 2. Ergebnis ist ein konstanter Offset in Sekunden. | 🟡 | offen |
| B6 | `robot_cam` / `robot_cam_2` am realen Aufbau durchtesten | Beide sind implementiert und unit-getestet, aber **noch nie gegen echte Klötze gelaufen**. A/B-Test: Farbe (`robot_cam`) gegen Kante (`robot_cam_2`), identische I/O. **Vorgehen in sechs Stufen: `robot-cam-befunde.md` §7** (Signalweg → Farbmaske → Nah-Gate → **Höhe** → Geometrie → A/B). Stufe 3 (Höhe) ist der Kernpunkt und liefert zugleich B8 und D18. Hintergrund: ebd. §2. | 🟡 | in Arbeit |
| B7 | **Greifhöhe und Eintauchtiefe** ermitteln | Bei welcher z-Höhe fasst der Greifer den Block sicher, ohne aufs Band zu drücken? Pro Blocktyp einmal. | 🟡 | offen |
| B8 | **Beobachtungshöhe festlegen** | Orientierung ist entschieden: **senkrecht** (Thema 6). Offen bleibt nur die Höhe. Sie ist **nicht frei wählbar** — sie muss dort liegen, wo `robot_cam` zuverlässig misst (Kopplung an B6 und die Min-/Max-Distanz-Parameter). Gilt für Warten und Suchen gleichermaßen (ein Parameter, nicht zwei). **Grund für die Untergrenze:** zu tief ⇒ die Tiefenkamera sieht die spiegelnden Seitenflächen, die Geometrie verzieht sich horizontal (`robot-cam-befunde.md` §2). Obergrenze: zu wenig Auflösung auf dem Klotz. Anhaltspunkt Vorgängergruppe: TCP-Z = 0,1 m, aber geneigt — **nicht übertragbar**, muss eingemessen werden. | 🟡 | offen |
| B15 | **Backenhöhe und `min_greifhoehe`** bestimmen | Halbe Backenhöhe plus Luft. Legt fest, ab welcher Blockhöhe sicher gegriffen werden kann. | 🟡 | offen |
| B16 | **Greiferöffnung und Marge** bestimmen | Für die Greifbarkeitsprüfung im `priority_handler` (zu breite Blöcke gar nicht erst anfahren). | 🟡 | offen |
| B17 | **Bandoberflächenhöhe** in world bestimmen | Bezug für die Greifhöhe. Fester Wert, einmal vermessen. | 🟡 | offen |
| B18 | **Toleranzen für die Greif-Freigabe** einmessen | Drei getrennte Werte: quer zur Backenbewegung (eng), längs (großzügig), Höhe. | 🟡 | offen |
| B9 | **Ablageposition festlegen** | Position in der Luft über der Auffangkiste. Muss außerhalb des Bandes und innerhalb des Arbeitsraums liegen. | 🟡 | offen |
| B10 | **Arbeitsraumgrenzen festlegen** | `Safety/workspace_bounds.json` ist Platzhalter (alle Werte `null`). Ohne dokumentierte Festlegung darf laut eigenem README kein Wert als Sicherheitsgrenze übernommen werden. | 🟡 | offen |
| B12 | **Zeitstempel-Plausibilität prüfen** | Zeitstempel eines Bildes ausgeben lassen: nicht null, nicht konstant, monoton steigend. Eine Logzeile. | 🔴 | offen |
| B13 | **Uhrendrift prüfen (Kamera vs. ROS)** | `ros_zeit_jetzt − bild_header_stempel` über einige Minuten mitloggen. Konstant (egal wie groß) = unproblematisch. Wandert der Wert = echte Drift, dann Gegenmaßnahme nötig. RealSense-Kameras stempeln je nach Konfiguration in einer `HARDWARE_CLOCK`-Domäne, die nur einmal beim ersten Bild gegen die Rechneruhr synchronisiert wird. Bei 0,3 s Versatz und 0,2 m/s wären das 60 mm Positionsfehler, mit Drift über die Session wachsend. | 🔴 | offen |
| B14 | **Zeitdomäne des AICA-RealSense-Blocks** | Lässt sich die Zeitdomäne auf Systemzeit stellen? Nur relevant, falls B13 Drift zeigt. | 🟡 | offen |
| B19 | **Greifzone festlegen** | Drei Kriterien für die stromaufwärtige Grenze gemeinsam: Reichweite, Singularitäten (B11), Abstand zur Basiskamera-Halterung am Bandanfang. Stromabwärts: Reichweite und verbleibendes Zeitbudget. **Nicht zu verwechseln mit B10** (Sicherheitsraum) — die Greifzone liegt darin. | 🟡 | offen |
| B11 | **Singularitäten prüfen** | **Teilweise am virtuellen Roboter möglich**, sofern er dasselbe URDF und denselben IK-Controller nutzt — die Gelenkgeschwindigkeiten verhalten sich dort gleich. Fährt der Roboter beim Verfolgen entlang des Bandes durch eine Handgelenk-Singularität? Dort explodieren die Gelenkgeschwindigkeiten bei IK-Velocity-Regelung. | 🟡 | offen |

---

## C — Aus anderen Quellen einholen

| ID | Punkt | Quelle | Dringl. | Status |
|---|---|---|---|---|
| C1 | **Hand-Auge-Kalibrierung Roboterkamera → TCP/Flansch** | **Werte gefunden:** `camera_mount_to_camera` x=0,1087 y=−0,03436 z=−0,05987 m + ~90°-Drehung, in `FuE_Greifen-main/Robot/Calibration_results_final.yaml` (auf dem Projektrechner). Einheit m, Format YAML — damit geklärt. **Rest offen:** ob „mount" der Flansch oder der Greifpunkt ist (hängt an C8). Siehe `robot-cam-befunde.md` §6. | 🔴 | teilweise |
| C2 | **Intrinsik beider Kameras** | Kommilitone (laufendes Kalibrierprojekt) bzw. Vorgängergruppe | 🔴 | offen |
| C3 | **Extrinsik Basiskamera → Roboterbasis** | Kommilitone. Ist-Stand: Legacy-Werte in `Calibration/calibration.json`, markiert als `legacy_initial_values` — noch nicht validiert. | 🔴 | offen |
| C4 | **Extrinsik Roboterkamera — Bezug klären** | Zur Basis oder zum TCP? Beim Vorgängerprojekt lag eine Kamera→TCP-Kalibrierung vor. **Bestätigt:** Das Feld heißt `camera_mount_to_camera`, ist also **mitbewegt** (kamerafest zur Montagestelle am Arm), nicht basisfest. Werte siehe C1 / `robot-cam-befunde.md` §6. | 🔴 | teilweise |
| C5 | **Bandgeschwindigkeit aus dem Vorgängerprojekt** | Gleiches Band — evtl. bereits dokumentiert. Ersetzt B1 nicht, aber gibt einen Erwartungswert zur Gegenprobe. | 🟢 | offen |
| C6 | **Übergabeform der Kalibrierwerte** | Kommilitone. JSON zum manuellen Übertragen in AICA-Parameter (wie bisher) oder als TF? | 🟡 | offen |
| C7 | Attractor-Parameterwerte | 2. Projekt-Chat auf dem mobilen System kennt den Ablageort | 🔴 | offen |
| C8 | **Werkzeugversatz Flansch → Greifpunkt** aus dem Vorgängerprojekt auslesen | Gleicher Greifer, gleicher Aufbau. Die Gruppe hatte laut Aussage eine Kalibrierung für den Greifpunkt. Liefert denselben Wert, den wir als `tool_offset_z_m` brauchen. Beim Bau der Komponenten auf dem mobilen System zu klären. | 🔴 | offen |
| C9 | **Wie hat die Vorgängergruppe gegriffen?** (welches Frame sie kommandiert haben) | Beantwortet A7/A8 indirekt und zuverlässig, da gleicher Aufbau. **Indiz gefunden:** In `FuE_Greifen-main/Robot/move_handler_strat2.py` werden **TCP-Posen** kommandiert (`target_tcp`). ⚠️ Das ist aus dem Code **erschlossen, nicht bestätigt** — A7 (Greifer im URDF?) bleibt eigenständig zu prüfen. Siehe `robot-cam-befunde.md` §6/§8. | 🔴 | teilweise |

---

## D — Festzulegende Werte (Entscheidung, keine Messung)

| ID | Wert | Abhängig von | Status |
|---|---|---|---|
| D1 | `lead_offset_m` | B1, A1 (bzw. direkt über B4 kalibriert) | offen |
| D2 | Rate des `object_follower` | Thema 2 | offen |
| D3 | Toleranz für die Greif-Freigabe (mm) | B7, Greifergeometrie | offen |
| D4 | ~~Sicherheitsfaktor im Erreichbarkeitskriterium~~ → siehe D17 | – | ersetzt |
| D5 | Timeouts je Zustand des `object_follower` | Thema 6 — Richtwerte stehen im Protokoll | offen |
| D10 | `require_robot_cam_for_grasp` (Umsetzung von D9) | Betriebsentscheidung nach der Kalibrierung | offen |
| D11 | Schwelle der Orientierungsgüte (Resultantenlänge) für "Winkel unbrauchbar" | Thema 6 | offen |
| D12 | Transferhöhe über dem Band | B17 + höchster erwarteter Block | offen |
| D13 | Ablagepose (in der Luft über der Auffangkiste) | B9 | offen |
| D14 | `max_extrapolation_s` (Deckel der Vorhersage) | Thema 7 | offen |
| D15 | `max_ziel_sprung_m` (Sprungerkennung im Sicherheitsgate) | Thema 7 | offen |
| D16 | `max_korrektur_m` (Identitätsprüfung der Roboterkamera, R4) | Thema 7 — Größenordnung: wenige cm | offen |
| D17 | Sicherheitsfaktor im Erreichbarkeitskriterium (P1) | Nachtrag Thema 7 | offen |
| D18 | `min_belt_distance_m` / `max_belt_distance_m` für `robot_cam` (R3) | hängt an B6 und B8. Untergrenze = wo der Seitenflächen-Verzug einsetzt, Obergrenze = wo die Auflösung nicht mehr reicht (`robot-cam-befunde.md` §2) | offen |
| D19 | `tol_yaw_rad` — Winkeltoleranz der Greif-Freigabe (Nachtrag 2 / F1) | B18 | offen |
| D6 | Maximale Messungsalter-Grenze der `robot_cam`-Messung vor Rückfall auf `w=0` | Thema 2 | offen |
| D7 | `latency_compensation_s` | B4 / B13 | offen |
| D8 | Rampendauer beim Ein-/Ausblenden von `w` (Vorschlag: ~0,2 s) | Thema 6 | offen |
| D9 | Darf ein Griff komplett ohne `robot_cam` durchlaufen, oder Abbruch nach Zeit X? | Thema 6, hängt an der Güte der Basiskamera-Extrinsik | offen |

---

## E — Noch zu klärende Designfragen

| ID | Frage | Behandelt in | Status |
|---|---|---|---|
| E1 | Zeitstempelquelle und Latenzkompensation | Thema 2 | in Arbeit |
| E2 | Frames und Einheiten, wo welche Kalibrierung eingeht | Thema 3 | offen |
| E3 | Datenverträge (Strides) aller Signale | Thema 4 | offen |
| E4 | Komponentenschnitt, Auflösung des Zyklus | Thema 5 | offen |
| E5 | Zustandsautomat, Orientierungsmodi, Toleranzen, Timeouts | Thema 6 | offen |
| E6 | Sicherheitskonzept (Watchdog, Limits, Arbeitsraum, Singularitäten) | Thema 7 | offen |
| E7 | Umschaltung Option A ↔ C sauber implementieren | **erledigt** — Greif-Freigabe rechnet der Follower selbst; `is_in_range` nur für statische Ziele (Thema 6) | erledigt |
| E8 | Strengere Regel für die Greif-Freigabe als fürs Folgen | **erledigt** — Parameter `require_robot_cam_for_grasp` (Thema 6) | erledigt |
| E9 | Sicherheitskonzept | **erledigt** — Thema 7. Kernpunkt: Die Attractor-Kette schließt den klassischen Weglauf-Fehler aus; das Risiko sind falsche Posen, nicht fehlende. | erledigt |
