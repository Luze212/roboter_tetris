# Offene Punkte, Tests und Vor-Ort-Aufgaben

Arbeitsliste für alles, was **nicht** am Schreibtisch entschieden werden kann.
Wächst mit jedem bearbeiteten Thema. Rein lokal.

**Legende Status:** `offen` · `in Arbeit` · `erledigt`
**Legende Dringlichkeit:** 🔴 blockiert die Umsetzung · 🟡 blockiert die Inbetriebnahme · 🟢 Feinschliff

---

## A — In AICA Studio ablesen (kein Test, nur nachschauen)

| ID | Punkt | Warum | Dringl. | Status |
|---|---|---|---|---|
| A1 | Parameterwerte des Signal Point Attractors | **Abgelesen 14.09.2026** (Komponentenbeschreibung, Defaults — die Teststand-Anwendung setzt keine): `linear_gains` `[1.0]`, `angular_gains` `[1.0]`, `max_linear_velocity` 0,5, `max_angular_velocity` 0,5, `linear_precision` 0,01, `angular_precision` 0,1. ⚠️ **Diese Defaults taugen nicht:** Bei K = 1 ist der Vorhalt die volle Bandgeschwindigkeit und die Einschwingzeit `3/K` **3 s**. Wird damit zur Anforderung an die Projekteinrichtung → `uebersicht/einrichtung-projektanwendung.md` §2. **Zusatzbefund:** Die bindende Geschwindigkeitsgrenze ist die des **IK-Controllers** (Default 0,25 m/s), nicht die des Attractors. | 🔴 | **erledigt** |
| A2 | Exakter Signaltyp des `attractor`-Eingangs und des `pose`-Ausgangs von `frame_to_signal` | **Beantwortet 14.09.2026: `cartesian_pose`** — beide. `datenvertraege.md` S6 ist korrigiert (stand dort fälschlich als `cartesian_state`). Auf ROS-Ebene tragen beide dieselbe Nachricht `EncodedState`; der Unterschied existiert nur in der AICA-Typprüfung beim Verdrahten. | 🔴 | **erledigt** |
| A3 | Lässt sich am `base_frame`-Eingang des Attractors ein **Signal** anschließen? | **Ja — aber vermutlich nutzlos.** Er ist ein Signal-Eingang, allerdings vom Typ **`cartesian_pose`**. Eine Pose trägt keine Geschwindigkeit, und genau die ist die Grundlage von Option C (beim Rücktransformieren wird `v_base` addiert). Die Erwartung an B3 ist damit umzudrehen. | 🟡 | **erledigt**, B3 klärt endgültig |
| A4 | Sind die drei linearen Gains einzeln gesetzt oder gleich? | **Beantwortet:** `linear_gains` ist ein **Vektor mit einem Wert** (`[1.0]`), wirkt also isotrop. Empfehlung erfüllt, solange niemand drei Einträge daraus macht. | 🟡 | **erledigt** |
| A5 | Publiziert der `robot_state_broadcaster` auch TF? | **Ja**, Parameter `tf_publish_frequency`, Default **20 Hz**. Zu grob, um die `robot_cam`-Messung zeitrichtig zuzuordnen — der eigene TCP-Ringpuffer im Follower bleibt nötig. | 🟢 | **erledigt** |
| A6 | Welche Rate hat die Kamera-Komponente tatsächlich? | **Gemessen 14.09.2026: ~29,7 Hz** bei beiden Kameras, Farbe und aligned Depth. Basiskamera 1280×720 (Tiefe nativ 640×480), Roboterkamera 848×480. Einzelheiten: `architektur/entscheidungen.md`, Nachtrag 4 / M1. | 🟡 | **erledigt** |
| A7 | **Steht der Robotiq-Greifer im URDF?** | **Nein.** Die Anwendungen referenzieren `urdf: Universal Robots 10e`, eine AICA-Standarddefinition; die Hardware-Tabelle der Launcher-Datenbank ist leer, es gibt keine eigene Hardwaredefinition. **Der IK-Controller regelt damit den Flansch** → `tool_offset_z_m` wird gebraucht und ist ungleich null. | 🔴 | **erledigt** |
| A8 | Welches Frame meldet der `robot_state_broadcaster` als `cartesian_state`? | **Denselben Flansch.** Der Broadcaster hat keinen Frame-Parameter und meldet das Endframe desselben URDF wie A7. Regelgröße und Rückmeldung sind konsistent. | 🔴 | **erledigt** |
| A9 | Hat der IK-Velocity-Controller eine Dämpfung nahe Singularitäten und/oder eine Kommando-Zeitüberwachung? | **Dämpfung ja, Zeitüberwachung nein.** Parameter `pinv_damping`, Default **0,0** — also vorhanden und ausgeschaltet. Relevant für B11. Eine Kommando-Zeitüberwachung führt der Controller nicht. | 🟢 | **erledigt** |
| A11 | ~~Kann AICA mit simulierter Hardware laufen?~~ | **beantwortet:** AICA selbst läuft real, aber statt des echten Roboters kann ein **virtueller Roboter** verwendet werden, der die Bewegungen visualisiert. Damit sind die Follower-Stufen 4a–4c am Schreibtisch prüfbar. | – | erledigt |
| A10 | Gelenkgeschwindigkeitsgrenzen im Hardware-Interface | Nichts angepasst → vermutlich Herstellergrenzen des UR10e, also vorhanden, aber nicht auf den Aufbau eingeschränkt. Unterste Absicherung. | 🟡 | offen |

---

## B — Im Labor messen / testen

| ID | Punkt | Verfahren | Dringl. | Status |
|---|---|---|---|---|
| B1 | **Bandgeschwindigkeit + Bandrichtung** bestimmen | **Kein eigener Kalibriervorgang nötig.** Einmal das `objects`-Signal während eines normalen Testlaufs mitschneiden (mehrere Blöcke, volle Durchquerung des Sichtfelds), Gerade und Geschwindigkeit am Schreibtisch auswerten. Genauigkeit bei ~500 mm Bahnlänge und 30–50 Messungen: Richtung deutlich unter 0,5°. Gegenprobe der Geschwindigkeit mit Markierung auf dem Band und Stoppuhr. **Archivabgleich ergab keine Zahl** — die Vorgängergruppe bezog die Geschwindigkeit zur Laufzeit aus dem Tracker und schrieb sie nie fest (`architektur/vorgaengerprojekt-abgleich.md` §3, C5). B1 bleibt damit vollständig eigene Arbeit. ⚠️ **Vorab prüfen:** Der Tracker verwirft Geschwindigkeiten unter **30 mm/s** und setzt sie auf 0 — läuft das Band langsamer, meldet `base_cam` dauerhaft `v_band = 0`. Außerdem ist `v_band_gemessen` die **y-Komponente**, nicht der Betrag: für B1 die Positionen über die Zeit auswerten, nicht dieses Feld (`datenvertraege.md` S1). | 🔴 | offen |
| B3 | **Option C verifizieren** (5-Minuten-Test) | Roboter still, Attractor auf eine feste Pose, `base_frame` mit einem Signal versorgen, das eine Position **und eine lineare Geschwindigkeit** trägt. Bewegt sich der Roboter → Option C bestätigt, `lead_offset_m` kann auf 0. Bewegt er sich nicht → bei Option A bleiben. ⚠️ **Erwartung nach A3 umgedreht:** Der `base_frame`-Eingang ist zwar ein Signal, aber vom Typ `cartesian_pose` und trägt damit keine Geschwindigkeit — genau die Größe, auf der Option C beruht. Der Test klärt endgültig, das erwartete Ergebnis ist aber „bewegt sich nicht“. | 🟡 | offen |
| B4 | **Vorhalt kalibrieren** (nur bei Option A) | Block mitfahren lassen, Restabstand TCP↔Block im Debug-Bild ablesen, `lead_offset_m` anpassen bis der Abstand null ist. Ersetzt die Kenntnis des exakten Gains. | 🟡 | offen |
| B5 | **Kameralatenz** bestimmen | Verfahren siehe Thema 2. Ergebnis ist ein konstanter Offset in Sekunden. | 🟡 | offen |
| B6 | `robot_cam` / `robot_cam_2` am realen Aufbau durchtesten | Beide sind implementiert und unit-getestet, aber **noch nie gegen echte Klötze gelaufen**. A/B-Test: Farbe (`robot_cam`) gegen Kante (`robot_cam_2`), identische I/O. **Vorgehen in sechs Stufen: `robot-cam-befunde.md` §7** (Signalweg → Farbmaske → Nah-Gate → **Höhe** → Geometrie → A/B). Stufe 3 (Höhe) ist der Kernpunkt und liefert zugleich B8 und D18. Hintergrund: ebd. §2. | 🟡 | in Arbeit |
| B7 | **Greifhöhe und Eintauchtiefe** ermitteln | Bei welcher z-Höhe fasst der Greifer den Block sicher, ohne aufs Band zu drücken? Pro Blocktyp einmal. ✅ **Höhenmessung am 14.09.2026 verifiziert:** Ein 50×50×100-Klotz wurde zu 100,0 mm gemeldet. `HEIGHT_BIAS_MM = 20` kompensiert dabei exakt eine um 20 mm zu weit gelesene Deckflächen-Tiefe — der Wert ist belegt, kein Fudge-Faktor. ⚠️ Er hängt aber 1:1 an `conveyor_z_dist`: wer an einem dreht, muss den anderen mitprüfen. | 🟡 | offen |
| B8 | **Beobachtungshöhe festlegen** | Orientierung ist entschieden: **senkrecht** (Thema 6). Offen bleibt nur die Höhe. Sie ist **nicht frei wählbar** — sie muss dort liegen, wo `robot_cam` zuverlässig misst (Kopplung an B6 und die Min-/Max-Distanz-Parameter). Gilt für Warten und Suchen gleichermaßen (ein Parameter, nicht zwei). **Grund für die Untergrenze:** zu tief ⇒ die Tiefenkamera sieht die spiegelnden Seitenflächen, die Geometrie verzieht sich horizontal (`robot-cam-befunde.md` §2). Obergrenze: zu wenig Auflösung auf dem Klotz. Anhaltspunkt Vorgängergruppe: TCP-Z = 0,1 m, aber geneigt — **nicht übertragbar**, muss eingemessen werden. | 🟡 | offen |
| B15 | **Backenhöhe und `min_greifhoehe`** bestimmen | Halbe Backenhöhe plus Luft. Legt fest, ab welcher Blockhöhe sicher gegriffen werden kann. | 🟡 | offen |
| B16 | **Greiferöffnung und Marge** bestimmen | Für die Greifbarkeitsprüfung im `priority_handler`. **Startwerte:** Die Vorgängergruppe kalibrierte den Hub auf **0…130 mm** und positionierte auf **Blockbreite + 30 mm** vor. ⚠️ **Die gemeldete Grundfläche ist systematisch zu groß** — am 14.09.2026 gemessen: +8,8 mm auf der zur Kamera zeigenden Seite, +2,3 mm quer (Nachtrag 4 / M5). Die Prüfung liegt damit ohnehin zur sicheren Seite; `gripper_margin_m` sollte diesen Anteil nicht noch einmal aufschlagen. | 🟡 | offen |
| B17 | **Bandoberflächenhöhe** in world bestimmen | Bezug für die Greifhöhe. Fester Wert, einmal vermessen. ⚠️ Siehe B7: Bandoberflächenhöhe, `conveyor_z_dist` und `HEIGHT_BIAS_MM` nur gemeinsam anfassen. | 🟡 | offen |
| B18 | **Toleranzen für die Greif-Freigabe** einmessen | Drei getrennte Werte: quer zur Backenbewegung (eng), längs (großzügig), Höhe. | 🟡 | offen |
| B9 | **Ablageposition festlegen** | Position in der Luft über der Auffangkiste. Muss außerhalb des Bandes und innerhalb des Arbeitsraums liegen. **Anhaltspunkte aus dem Vorgängerprojekt** (`Robot/pose.yaml`, geteachte TCP-Posen): `Move_to_Place` [0,063 · −0,78 · 0,22], `Place_blue` [0,086 · −0,602 · 0,200]. Beide innerhalb des dortigen Arbeitsraums. ⚠️ TCP-Bezug, siehe B10. | 🟡 | offen |
| B10 | **Arbeitsraumgrenzen festlegen** | `Safety/workspace_bounds.json` ist Platzhalter (alle Werte `null`). Ohne dokumentierte Festlegung darf laut eigenem README kein Wert als Sicherheitsgrenze übernommen werden. **Anhaltspunkt:** Die Vorgängergruppe prüfte jede Bewegung gegen ein Rechteck (`Robot/save_pos.py`) — dieselbe Form wie unsere Gate-Prüfung 5. Werte in `Robot/pose.yaml`: X −0,05…1,05 · Y −0,8…0,3 · Z 0,095…0,37 m. ⚠️ **Diese Grenzen gelten für den TCP, unsere für den Flansch** — Übernahme nur mit Umrechnung um den Werkzeugversatz (C8) und dokumentierter Begründung (`architektur/vorgaengerprojekt-abgleich.md` Falle 5). | 🟡 | offen |
| B12 | **Zeitstempel-Plausibilität prüfen** | **Gemessen 14.09.2026.** Roboterkamera: gefüllt, monoton, nicht konstant — in Ordnung. Basiskamera: zunächst monoton, dann **eingefroren** — siehe B13. | 🔴 | **erledigt** |
| B13 | **Uhrendrift prüfen (Kamera vs. ROS)** | **Gemessen 14.09.2026: Drift bestätigt und behoben.** Die Basiskamera (L515) stempelte in der Hardwareuhr — fremde Epoche (Jahr 2006), **+4,05 ms/s** Drift, danach Stillstand des Stempels, wodurch `base_cam` jedes Bild verwarf und die Objektliste leer blieb. Nach Aktivierung von `global_time_enabled`: **+0,022 ms/s**, stabil. Die Roboterkamera war durchgehend unauffällig. Vollständig: `architektur/entscheidungen.md`, Nachtrag 4 / M2. | 🔴 | **erledigt** |
| B14 | **Zeitdomäne des AICA-RealSense-Blocks** | **Beantwortet:** Ja, über `rgb_camera.global_time_enabled` und `depth_module.global_time_enabled`. ⚠️ Der AICA-Block **exponiert sie nicht** (25 Parameter, diese nicht dabei) — deshalb erzwingt `base_cam` sie jetzt selbst über den Parameter `camera_node` (Nachtrag 4 / M3). | 🟡 | **erledigt** |
| B19 | **Greifzone festlegen** | Drei Kriterien für die stromaufwärtige Grenze gemeinsam: Reichweite, Singularitäten (B11), Abstand zur Basiskamera-Halterung am Bandanfang. Stromabwärts: Reichweite und verbleibendes Zeitbudget. **Nicht zu verwechseln mit B10** (Sicherheitsraum) — die Greifzone liegt darin. **Anhaltspunkt zur Bandgeometrie** (gleiches Roboter-Frame): Der Tracker der Vorgängergruppe löschte Objekte außerhalb **y = +375 … −1080 mm** und maß Geschwindigkeit nur im Fenster **y = −1000 … −500 mm** (`cameras/tracker.hpp`, `architektur/vorgaengerprojekt-abgleich.md` §3). ⚠️ **Viertes Kriterium (Nachtrag 3 / N2): Die Greifzone muss innerhalb der Messregion des Trackers liegen.** `track_velocity_region_y_min`/`_max` von `base_cam` werden hier mitfestgelegt (Region = Greifzone plus Rand). Außerhalb koppelt der Tracker die Längsposition statt sie zu messen und löscht Tracks nicht bei ausbleibender Detektion — dort greifen weder die Plausibilitätsprüfung noch die Abbruchregel des Follower. | 🟡 | offen |
| B11 | **Singularitäten prüfen** | **Teilweise am virtuellen Roboter möglich**, sofern er dasselbe URDF und denselben IK-Controller nutzt — die Gelenkgeschwindigkeiten verhalten sich dort gleich. Fährt der Roboter beim Verfolgen entlang des Bandes durch eine Handgelenk-Singularität? Dort explodieren die Gelenkgeschwindigkeiten bei IK-Velocity-Regelung. | 🟡 | offen |

---

## C — Aus anderen Quellen einholen

| ID | Punkt | Quelle | Dringl. | Status |
|---|---|---|---|---|
| C1 | **Hand-Auge-Kalibrierung Roboterkamera → Flansch** | **Vollständig geklärt: die Kalibrierung ist Flansch → Kamera.** Belegt über drei unabhängige Stellen (Posenaufnahme mit `getActualToolFlangePose()` in `ros2_ws/src/get_pose.py:54`, Weiterverarbeitung `convert_poses.py` → `industrial_calibration`, Launch-Default `camera_mount_frame=tool0`). Beweiskette: `architektur/vorgaengerprojekt-abgleich.md` §2. Direkt als `handeye_*` verwendbar: x = 0,1087 · y = −0,03436 · z = −0,05987 m; rpy = (0,02898 · 0,02722 · 1,597) rad ≈ (1,66° · 1,56° · 91,5°). Quelle: `Robot/Calibration_results_final.yaml`. ⚠️ **Beim Einsetzen:** Die Werte sind **Flansch → Kamera**. Die ~91,5°-Drehung vertauscht bei falscher Richtung x und y, statt nur ein Vorzeichen zu drehen — das Ergebnis sieht plausibel aus und ist falsch (Nachtrag 3 / N1). | 🔴 | **erledigt** |
| C2 | **Intrinsik beider Kameras** | Kommilitone (laufendes Kalibrierprojekt) bzw. Vorgängergruppe. ⚠️ **Nicht aus dem Vorgängerarchiv übernehmen:** die Werte in `ros2_ws/src/config.yaml` sind vertauscht beschriftet (fx/fy tragen den Hauptpunkt, cx/cy die Brennweite) und dienten dort nur als Startschätzung — `architektur/vorgaengerprojekt-abgleich.md` Falle 4. | 🔴 | offen |
| C3 | **Extrinsik Basiskamera → Roboterbasis** | Kommilitone. Ist-Stand: Legacy-Werte in `Calibration/calibration.json`, markiert als `legacy_initial_values` — noch nicht validiert. | 🔴 | offen |
| C4 | **Extrinsik Roboterkamera — Bezug klären** | **Erledigt mit C1.** Das Feld heißt `camera_mount_to_camera`, ist also mitbewegt (kamerafest zur Montagestelle am Arm) — und die Montagestelle ist der **Flansch** (`tool0`), nicht der Greifpunkt. | 🔴 | **erledigt** |
| C5 | **Bandgeschwindigkeit aus dem Vorgängerprojekt** | Gleiches Band. **Ergebnis des Archivabgleichs: keine Zahl vorhanden.** Die Geschwindigkeit wurde dort zur Laufzeit aus dem Tracker (`vy`) bezogen und nie festgeschrieben. Einzige Eingrenzung: Objekte unter 0,05 m/s galten als „läuft nicht“, die Bandgeschwindigkeit liegt also darüber (`architektur/vorgaengerprojekt-abgleich.md` §3). **B1 wird dadurch nicht entlastet.** | 🟢 | erledigt (nichts vorhanden) |
| C6 | **Übergabeform der Kalibrierwerte** | Kommilitone. JSON zum manuellen Übertragen in AICA-Parameter (wie bisher) oder als TF? | 🟡 | offen |
| C7 | ~~Attractor-Parameterwerte über den 2. Projekt-Chat~~ | **Entfallen.** Die Werte sind am 14.09.2026 direkt aus der Komponentenbeschreibung im AICA-Image abgelesen — siehe A1. | 🔴 | **erledigt** |
| C8 | **Werkzeugversatz Flansch → Greifpunkt** | ⚠️ **Aus dem Vorgängerarchiv nicht zu holen.** Die Gruppe fuhr über `getActualTCPPose()`/`moveL()`; der Versatz saß damit in der **UR-Installation am Teach-Pendant**, nicht im Code — im ganzen Archiv gibt es kein `set_tcp`/`setTcp`/`setPayload` (`architektur/vorgaengerprojekt-abgleich.md` §4). **Neue Quelle: die Werkzeugkonfiguration der Robotersteuerung selbst.** | 🔴 | offen, Quelle gewechselt |
| C9 | **Wie hat die Vorgängergruppe gegriffen?** | **Bestätigt:** kommandiert wurden **TCP-Posen** (`getActualTCPPose` / `moveL`, `Robot/Robot.py:109`). A7 ist inzwischen **eigenständig beantwortet** (kein Greifer im URDF, siehe dort) — der Umweg über das Vorgängerprojekt wird nicht mehr gebraucht. | 🔴 | **erledigt** |

---

## D — Festzulegende Werte (Entscheidung, keine Messung)

| ID | Wert | Abhängig von | Status |
|---|---|---|---|
| D1 | `lead_offset_m` | B1, A1 (bzw. direkt über B4 kalibriert) — Größenordnung aus dem Vorgängerprojekt: dort deckte **eine** empirische Konstante von **0,7 s** den gesamten Vorhalt ab (`architektur/vorgaengerprojekt-abgleich.md` §3) — bei völlig anderer Kette, also nur als Hausnummer. | offen |
| D2 | Rate des `object_follower` | Thema 2 | offen |
| D3 | Toleranz für die Greif-Freigabe (mm) | B7, Greifergeometrie | offen |
| D4 | ~~Sicherheitsfaktor im Erreichbarkeitskriterium~~ → siehe D17 | – | ersetzt |
| D5 | Timeouts je Zustand des `object_follower` | Thema 6 — Richtwerte stehen im Protokoll | offen |
| D10 | `require_robot_cam_for_grasp` (Umsetzung von D9) | Betriebsentscheidung nach der Kalibrierung | offen |
| D11 | Schwelle der Orientierungsgüte (Resultantenlänge) für "Winkel unbrauchbar" | Thema 6 — ⚠️ **Die Schwelle lässt sich nicht am quadratischen Klotz einstellen:** Der Tracker friert dessen Orientierung bereits ein, das Gütemaß schlägt dort nie an. Es wirkt gegen Rauschen, Teilverdeckung und Objekte am ROI-Rand. Begründung und Rechenweg: `architektur/vorgaengerprojekt-abgleich.md` §7; die Specs sind entsprechend korrigiert. ⚠️ **Zusaetzlich (Nachtrag 4 / M5):** Die Schwelle `NEAR_SQUARE_ASPECT = 0,92` ist zu streng — ein exakt quadratischer Klotz wurde zu 0,889 vermessen. Startwert fuer die abgeleitete Pruefung **0,85**, an weiteren Kloetzen zu bestaetigen. | offen |
| D12 | Transferhöhe über dem Band | B17 + höchster erwarteter Block | offen |
| D13 | Ablagepose (in der Luft über der Auffangkiste) | B9 | offen |
| D14 | `max_extrapolation_s` (Deckel der Vorhersage) | Thema 7 | offen |
| D15 | `max_ziel_sprung_m` (Sprungerkennung im Sicherheitsgate) | Thema 7 | offen |
| D16 | `max_korrektur_m` (Identitätsprüfung der Roboterkamera, R4) | Thema 7 — Größenordnung: wenige cm | offen |
| D17 | Sicherheitsfaktor im Erreichbarkeitskriterium (P1) | Nachtrag Thema 7 | offen |
| D18 | `min_belt_distance_m` / `max_belt_distance_m` für `robot_cam` (R3) | hängt an B6 und B8. Untergrenze = wo der Seitenflächen-Verzug einsetzt, Obergrenze = wo die Auflösung nicht mehr reicht (`robot-cam-befunde.md` §2) | offen |
| D19 | `tol_yaw_rad` — Winkeltoleranz der Greif-Freigabe (Nachtrag 2 / F1) | B18 | offen |
| D6 | Maximale Messungsalter-Grenze der `robot_cam`-Messung vor Rückfall auf `w=0` | Thema 2 | offen |
| D7 | `latency_compensation_s` | B4 / B13 — siehe D1: 0,7 s Gesamtvorhalt im Vorgängerprojekt, dort nicht in Latenz- und Regleranteil getrennt. | offen |
| D8 | Rampendauer beim Ein-/Ausblenden von `w` (Vorschlag: ~0,2 s) | Thema 6 | offen |
| D9 | Darf ein Griff komplett ohne `robot_cam` durchlaufen, oder Abbruch nach Zeit X? | Thema 6, hängt an der Güte der Basiskamera-Extrinsik | offen |

---

## E — Noch zu klärende Designfragen

| ID | Frage | Behandelt in | Status |
|---|---|---|---|
| E1 | Zeitstempelquelle und Latenzkompensation | **erledigt** — Thema 2 entschieden, B13 am Aufbau gemessen und behoben (Nachtrag 4 / M2) | erledigt |
| E2 | Frames und Einheiten, wo welche Kalibrierung eingeht | **erledigt** — Thema 3 | erledigt |
| E3 | Datenverträge (Strides) aller Signale | **erledigt** — Thema 5, vollständig in `architektur/datenvertraege.md` | erledigt |
| E4 | Komponentenschnitt, Auflösung des Zyklus | **erledigt** — Thema 4 | erledigt |
| E5 | Zustandsautomat, Orientierungsmodi, Toleranzen, Timeouts | **erledigt** — Thema 6, ergänzt durch Nachtrag 2 (F1–F3) und Nachtrag 3 (N1, N3) | erledigt |
| E6 | Sicherheitskonzept (Watchdog, Limits, Arbeitsraum, Singularitäten) | **erledigt** — Thema 7; die Werte selbst bleiben als B10/B11/B19 offen | erledigt |
| E7 | Umschaltung Option A ↔ C sauber implementieren | **erledigt** — Greif-Freigabe rechnet der Follower selbst; `is_in_range` nur für statische Ziele (Thema 6) | erledigt |
| E8 | Strengere Regel für die Greif-Freigabe als fürs Folgen | **erledigt** — Parameter `require_robot_cam_for_grasp` (Thema 6) | erledigt |
| E9 | Sicherheitskonzept | **erledigt** — Thema 7. Kernpunkt: Die Attractor-Kette schließt den klassischen Weglauf-Fehler aus; das Risiko sind falsche Posen, nicht fehlende. | erledigt |
