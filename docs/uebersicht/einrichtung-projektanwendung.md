# Einrichtung der Projektanwendung in AICA

**Stand 24.09.2026, finaler Build.** Was beim Anlegen der AICA-Anwendung für den
Pick im Lauf gesetzt werden muss — und warum die AICA-Defaults nicht taugen.
Kopplungen zwischen Parametern: `systemgraph.md`.

Alle Parameter der **eigenen** Komponenten stehen mit den am Aufbau gemessenen
Werten als Standardwert im Paket. Nach dem Build gelten sie auch in bestehenden
Blöcken, sofern der Parameter dort auf dem Standardwert steht; ein von Hand
gesetzter Wert bleibt. Von Hand zu setzen sind nur die Werte der **AICA-Bausteine**,
die **Raten** und die **UR-Nutzlast** (§2, §5).

Grundlage: Messungen am Aufbau (14.–24.09.2026) und die Komponentenbeschreibungen
im AICA-Image (`v2.0.5-jazzy`, core v5.0.0).

---

## 1. Pflicht — ohne diese Werte funktioniert es nicht

| Was | Wert | Warum |
|---|---|---|
| **`rgb_camera.global_time_enabled`** der Basiskamera | **`true`** | Steht er auf `false`, stempelt die L515 in ihrer Hardwareuhr: fremde Epoche, Drift von 3,9 ms/s gegen die ROS-Zeit, und nach wenigen Minuten bleibt der Stempel stehen. Dann verwirft `base_cam` jedes Bild nach dem ersten, und die Objektliste bleibt leer. |
| **`depth_module.global_time_enabled`** der Basiskamera | **`true`** | dito |
| `object_follower`: `ws_*`, `observe_*` | Standardwert (§9) | Arbeitsraum und Beobachtungspose sind Pflichtparameter; leer scheitert `configure` mit einer Liste der fehlenden (Nachtrag 8 / F3). |

Der AICA-RealSense-Block bietet die beiden `global_time_enabled` nicht als
Parameter an; sie lassen sich nur zur Laufzeit setzen. **`base_cam` erzwingt sie
deshalb selbst:** Ihr Parameter `camera_node` nennt den Knoten der Basiskamera
(Standard `/realsense_camera`), und bei jeder Aktivierung setzt die Komponente dort
beide Werte. Leer schaltet die Funktion ab.

**Knotennamen über `serial_no` prüfen.** Welcher RealSense-Knoten welche Kamera ist,
hängt an der Anwendung. Maßgeblich ist der Serial: **Basiskamera `f1370107` (L515)**,
Roboterkamera `241122074842` (D435i). Den Namen der Basiskamera trägt `camera_node`.

**Neues Paket wirksam?** Ein Rebuild genügt nicht — das Paket wird erst wirksam,
wenn danach das **AICA-Systemabbild im Launcher neu erzeugt** wird; ein Neustart der
Anwendung holt es ohne Fehlermeldung nicht nach. Gegenprobe: Neue Parameter oder
Komponenten sind in der AICA-Bibliothek sichtbar.

Zeitstempel prüfen, nicht annehmen: Die Hardwareuhr wird beim Start einmal
synchronisiert, der Versatz sieht direkt danach klein aus und wächst erst später.
`ros_zeit − header_stempel` über mehrere Minuten mitloggen — konstant ist richtig.

---

## 2. Regelung — von Hand zu setzen

| Baustein | Parameter | AICA-Default | **Wert** | Warum |
|---|---|---|---|---|
| Signal Point Attractor | `linear_gains` | `[1.0]` | **`[5.0]`** | Bei K = 1 läuft der Flansch einem 0,13-m/s-Band 70 mm hinterher, und das Einschwingen (3/K) dauert 3 s. Mit K = 5 und Vorhalt 0,24 s rund 1 mm Längsfehler (Nachtrag 13 / L18). Isotrop halten — ein Wert. |
| Signal Point Attractor | `angular_gains` | `[1.0]` | **`[5.0]`** | Sonst dauert das Eindrehen auf gedrehte Klötze rund 3 s (L26). |
| Signal Point Attractor | `max_linear_velocity` | 0,5 m/s | **0,5** | |
| Signal Point Attractor | `max_angular_velocity` | 0,5 rad/s | **1,0** | 45° Drehung in rund 0,9 s statt 1,5 s (L26). |
| Signal Point Attractor | `rate` | 10 Hz | **50** | glatte Bewegung beim Folgen |
| IK Velocity Controller | `max_linear_velocity` | 0,25 m/s | **0,5** | **Die bindende Grenze** — sie klemmt unabhängig vom Attractor. 0,5 m/s verkürzt Ablegen und Rückweg um rund 1 s (L24). |
| IK Velocity Controller | `max_angular_velocity` | 0,25 rad/s | **1,0** | wie am Attractor |
| IK Velocity Controller | `command_rate_limit` | ∞ | **2,0** | Gelenkbeschleunigung in rad/s². Ohne Grenze springt der Arm beim Anfahren aus dem Stand — Schutzstopp am Bandrand (L20). |
| UR-Steuerung (Pendant, Installation) | Nutzlast | 2,8 kg | **1,3 kg**, Schwerpunkt 12 / 24 / 45 mm | gemessen mit dem Assistenten der Steuerung (L20) |

**Im `priority_handler` passen die Standardwerte dazu:** `attractor_gain` 5 =
`linear_gains`, `attractor_v_max_mps` 0,5 = das kleinere `max_linear_velocity` aus
Attractor und IK-Controller. Er rechnet damit die Anfahrzeit; werden die Bausteine
anders eingestellt, müssen diese beiden Werte mitgehen.

Gedrosselte Läufe (IK-Controller langsamer): Die Grenze gilt für den Betrag der
Geschwindigkeit — dann auch langsamer absenken (`descend_speed_mps` 0,05,
`t_descend_s` 3,0) und `attractor_v_max_mps` auf den kleineren Wert setzen. Das
echte Band (0,13 m/s) ist mit 0,10 m/s nicht einzuholen; gedrosselt nur mit
`fake_objects.py` fahren.

---

## 3. Kameras

Gebraucht wird nur die **Basiskamera**; die Roboterkamera und ihr RealSense-Block
gehören nicht in die Anwendung (Rechenzeit, Nachtrag 13 / L22).

| | Basiskamera |
|---|---|
| Typ | RealSense L515 |
| `serial_no` | `f1370107` |
| `rgb_camera.profile` | `1280x720x15` |
| `depth_module.profile` | `640x480x30` (die L515 kann Tiefe nur mit 30) |
| `align_depth.enable` | `true` |
| `global_time_enabled` | `true` (§1) |

15 Bilder je Sekunde genügen: `base_cam` schafft ohnehin rund 8,6 Messungen je
Sekunde, die halbe Rate entlastet die Treiber (L16). Einen Grafikprozessor hat der
Rechner nicht. **Profile immer ausdrücklich setzen** — ohne Angabe gilt ein
Treiber-Default, und `base_cam` klemmt einen zu großen Bildausschnitt ohne
Fehlermeldung auf das Bild. Bildraten über die `camera_info`-Topics messen.

---

## 4. Hardware-Interface

| Parameter | Wert | Anmerkung |
|---|---|---|
| `urdf` | `Universal Robots 10e` | AICA-Standarddefinition. Der Greifer ist darin nicht enthalten — IK-Controller und Robot State Broadcaster arbeiten am **Flansch**. Deshalb braucht der Follower `flange_to_grip_point_m` (0,235 m). |
| `rate` | 500 | |
| `robot_ip` | 192.168.96.221 | Rechner statisch 192.168.96.10/24 |
| `tf_publish_frequency` des Broadcasters | 20 (Default) | Der Follower liest die Flanschpose direkt aus `cartesian_state`. |

---

## 5. Eigene Komponenten

**`rate` am Block setzen.** Der Parameter ist von `LifecycleComponent` geerbt
(Default 10 Hz) und wird nur beim Erzeugen gelesen. Er gehört **nicht** in die
eigenen `component_descriptions` — das Duplikat lässt die Oberfläche bei jedem
Klick ein weiteres Rate-Feld anlegen (Nachtrag 12 / K2).

**Alle Python-Komponenten laufen in einem Prozess** und teilen sich wegen der GIL
praktisch einen Kern (Nachtrag 13 / L2). Jede unnötig hohe Rate und jeder
überzählige Block kostet `base_cam` Messrate; auch unkonfigurierte Kamerablöcke
empfangen ihre Bilder.

| Komponente | Rate | Anmerkung |
|---|---|---|
| `base_cam` | 10 Hz (Default) | rund 8–9 Messungen je Sekunde |
| `vectoring` | **20 Hz** | |
| `priority_handler` | **20 Hz** | |
| `object_follower` | **50 Hz** | glatte Zielpose für den Attractor |
| `robotiq_gripper` | **20 Hz** | Takt, mit dem `motion_done`/`has_object` gelesen werden |
| `data_tracker` | **2 Hz** | nur Anzeige |
| `interface_streamer` | **10 Hz** | flüssiges Bild für den Vortrag; bei Rechenzeitmangel zuerst senken |

**`debug_enable`** steht bei `base_cam` als Standard an, weil der
`interface_streamer` ihr Debug-Bild zeigt. Das Debug-Bild wird mit der Kamerarate
verschickt, nicht mit der Komponentenrate (Nachtrag 3 / N5).

**Tracker- und Messregion von `base_cam`** liegen in `world` und umfassen das
ganze Band von Rolle zu Rolle (y −375 … +1080 mm). Hinter dem Bild der Basiskamera
(y ≈ 460 … 1030 mm) führt `vectoring` die Klötze mit der gemessenen
Bandgeschwindigkeit weiter (Status 4). Die Greifzone liegt bewusst **außerhalb**
des Bildes, damit der Greifer nicht als Klotz erkannt wird (Nachtrag 13 / L4, L10).

---

## 6. Verdrahtung

Die vollständige Signalliste steht in `systemgraph.md`. Worauf es ankommt:

- **`target_pose` des `object_follower` → Eingang `attractor` des Signal Point
  Attractors**, nicht an den IK-Controller. Typ `cartesian_pose`.
- **`cartesian_state` des Robot State Broadcasters** → `robot_state` von
  `object_follower` **und** `priority_handler` (der braucht den Anfahrweg).
- `target` des `priority_handler` → `target` des Followers.
- `gripper_close` → `robotiq_gripper`; `motion_done` und `has_object` zurück an
  den Follower.
- **`picked_id`** → `priority_handler` und `data_tracker`.
- **`interface_streamer`:** `base_debug_image` ← `debug_image` von `base_cam`,
  `world_state` ← `data_tracker`, `follower_status` ← `object_follower`;
  `interface_image` → RViz.
- `frame_to_signal` aus dem AICA-Beispielaufbau entfällt; der Follower tritt an
  seine Stelle.

**Vor dem Start von AICA den Arm freifahren:** Der Attractor fährt beim Start auf
die zuletzt gespeicherte Zielpose. Der Follower selbst startet mit einem
senkrechten Anheben und fährt dann in die Beobachtungspose.

---

## 7. Abnahme nach dem Anlegen

1. **Zeitstempel** — `ros_zeit − header_stempel` der Basiskamera über mindestens
   5 Minuten konstant.
2. **Bildraten** — 15 Bilder je Sekunde Farbe.
3. **`objects` nicht leer**, sobald ein Klotz im Bild liegt, und die Liste bleibt
   gefüllt. Leeres Band: keine Detektion.
4. **Debug-Bild** der Basiskamera: Klötze eingerahmt, Band und Greifer nicht.
5. **Bandgeschwindigkeit** in `tracks` bei laufendem Band um −0,128 m/s.

---

## 8. Gemessene Werte des Aufbaus

Herleitung und Unsicherheiten: `architektur/entscheidungen.md`, Nachträge 5 und 13.

**Alle Höhen sind Flanschmaße** (`ur_tool0`), nicht TCP-Maße der UR-Steuerung. Der
Greifer steht nicht im URDF; wer hier einen TCP-Wert einträgt, liegt um 215 mm
daneben.

| Wert | Zahl | Herkunft |
|---|---|---|
| Bandoberfläche in `world` | **53,6 mm** (eben auf ±1 mm) | drei Antastpunkte mit geschlossenem Greifer |
| Flansch → Backenspitze (geschlossen) | **245 mm** | Maßstab; beim ersten Griff bestätigt |
| Flansch → Auflagenmitte (Griffpunkt) = **`flange_to_grip_point_m`** | **235 mm** | Backenspitze − halbe Auflagenhöhe; Follower-Parameter (Nachtrag 6 / Z7) |
| TCP der UR-Steuerung | 215 mm | kein Parameter der Kette, nur zur Umrechnung fremder TCP-Werte |
| Greifhöhe Flansch, 100-mm-Klotz stehend | **≈ 339 mm** | 53,6 + 50 + 235 |
| Freihöhe Flansch (Transfer, Abbruch) = `transfer_height_m` | **490 mm** | Band + stehender Klotz + gehaltener Klotz + Luft (Nachtrag 10 / J1) |
| Ablagepose Flansch | x −316,49 · y +476,21 · z +419,71 mm, Gier 94,2° | am Aufbau angefahren, im Betrieb bestätigt |
| Arbeitsraum `ws_*` | x −1,0 … −0,30 · y −0,32 … +0,48 · z 0,3036 … 0,60 m | abgefahren (L14), z min L26; `Safety/workspace_bounds.json` |
| Greifzone (`priority_handler`) | x −1,0 … −0,53 · y +0,445 … −0,32 m | der Arbeitsraum auf dem Band (L21) |
| Bildausschnitt `base_cam` | `roi_x` 342, `roi_width` 618, `roi_y` 60, `roi_height` 580 | Bandrand am Roboter bis x −1,0; der Greifer ragt nicht ins Bild (L21) |
| Bandrichtung | praktisch die y-Achse, Lauf nach −y | |
| Bandebenheit | quer 0,39°, längs 0,01° | |
| Band von Rolle zu Rolle | y ≈ +1,08 … −0,375 m | Nachtrag 13 / L7 |
| Bandgeschwindigkeit | geschätzt −127,9 mm/s; per Stoppuhr 125–133 mm/s | Ziel 3 (L9, L10) |
| UR-Nutzlast | 1,3 kg, Schwerpunkt 12 / 24 / 45 mm | L20 |
| Extrinsik Basiskamera | −0,7787 · 0,7934 · 0,9163 · 179,46° · 0,45° · 179,76° | Übergangskalibrierung in `world` (L6); steht in `Extrinsics/base_cam_extrinsics.json` (Parameter „Kalibrierdatei“) und als Rückfall in `cal_*`; Abweichung höchstens 6 mm |
| `belt_surface_z_mm` / `top_depth_bias_mm` (`base_cam`) | 53,6 / 11,5 | Standardwerte (L6) |
| Bild der Basiskamera | y ≈ 0,46 … 1,03 m | L7 |
| Prozesszeiten | Einschwingen 0,23–0,33 s · Absenken 0,78–0,93 s (0,25 m/s) · Greifen 0,63–0,83 s · Ablegen 1,3–2,2 s (0,5 m/s) | `priority_handler` `t_settle_s` 0,4, `t_descend_s` 0,9, `t_grasp_s` 0,8, Faktor 1,0; Greifebene y ≈ +0,02 (L24) |
| Flacher Klotz 50 × 75 × 25 mm | gemessen 23,8 mm; Oberseite roh 11–15 mm über dem Band | erkannt über `min_contour_area` 1000 bei `min_obj_height` 15; greifbar ab 20 mm (L24) |
| Klotzwinkel der Basiskamera | im Lauf kein Wechsel der langen Kante, Güte 1,00, auch bei fast quadratischer Oberseite | L26 |
| Takt bei dichter Folge | etwa ein Klotz je 7 s | L24 |

**Greifhöhe:** `flansch_z_greifen = 53,6 + max(klotzhoehe/2, 16) + 235` [mm]. Die
Untergrenze 16 mm (`min_grip_height_m`) hält die geschlossene Backenspitze 6 mm
über dem Band, 1 mm über der Arbeitsraumgrenze `ws_z_min` 0,3036. Klötze ab 20 mm
Höhe werden gegriffen, bis 32 mm an der Untergrenze, darüber auf halber Höhe.
**Greifhöhe min und `ws_z_min` nur gemeinsam ändern** — sonst bricht das Gate jeden
flachen Griff ab.

---

## 9. Werte für den `object_follower`

Alle Werte sind Standardwert der Komponente. Der Arbeitsraum entspricht
`source/roboter_tetris/roboter_tetris/Safety/workspace_bounds.json`; ein Test prüft
die Gleichheit (Herleitung `entscheidungen.md` Nachtrag 13 / L14, L26).

Die Follower-Parameter heißen in der Oberfläche „Gruppe: Klartext
[parameter_name]“ in sieben Gruppen (1 Arbeitsraum … 7 Aufbau); AICA sortiert sie
selbst, die Gruppen stehen nur im Namen. Die Parameter des `priority_handler`
tragen den Namen ebenfalls in Klammern.

| Parameter | Wert | Herleitung |
|---|---|---|
| `ws_x_min` / `ws_x_max` | −1,000 / −0,300 | abgefahren bis −0,530; erweitert für die Ablage (x −0,316) |
| `ws_y_min` / `ws_y_max` | −0,320 / +0,480 | Bandende / abgefahren bis +0,445, erweitert für die Ablage (y +0,476) |
| `ws_z_min` / `ws_z_max` | 0,3036 / 0,600 | Backenspitze 5 mm über dem Band / darüber Singularität |
| `observe_x` / `observe_y` | −0,816 / +0,35 | Bandmitte am Anfang der Greifzone; beim Anfahren wartet der Arm am Zonenanfang |
| `observe_z` | 0,45 | Folgehöhe |
| `observe_yaw_deg` | 90° | Backen quer zur Bandrichtung (Grundstellung), nahe der Ablage-Orientierung |
| `lead_time_s` | 0,24 | Vorhalt ≈ 1/K, eingemessen über `err_laengs` (L18) |
| `max_extrapolation_s` / `target_timeout_s` | 1,0 / 1,5 | Deckel der Vorhersage über dem Alter der Messung (L23) |
| `descend_speed_mps` | 0,25 | passt zu `t_descend_s` 0,9 im `priority_handler` (Nachtrag 10 / J2, L24) |
| `min_grip_height_m` | 0,016 | siehe Greifhöhe (§8) |
| `use_block_orientation` | an | im Winkel des Klotzes greifen (L25, L26) |
| `max_yaw_deviation_deg` | 50 | höchstens ±45° aus der Grundstellung, Hysterese bis 50° (L25) |
| `gripper_yaw_offset_deg` | 0 | die Grundstellung greift richtig |

Zum Einmessen des Vorhalts `err_laengs` in `follower_status` mitlesen: Im Mittel
null heißt, `lead_time_s` passt zur Verstärkung des Attractors.

## 10. Kalibrierung der Basiskamera

Eigene Anwendung, getrennt vom normalen Programm: Inhalt von
`docs/uebersicht/anwendung-kalibrierung-basiskamera.yaml` in eine neue AICA-Anwendung
(Code-Ansicht) einfügen und speichern. Hintergrund und Stand: `entscheidungen.md` L27.
Der Lauf schreibt die **Kalibrierung für `base_cam`** (die Lage der Farbkamera mit dem
Tiefenfehler der L515 umgerechnet) seit 28.09.2026 **direkt in die Datei, die `base_cam`
liest**. In Kraft ist L6; die Kalibrierung vom 28.09. ist abgenommen, aber nicht übernommen
(L27).

1. Anwendung starten, **Greiferbacken frei**: Der Greifer fährt beim Laden einmal auf und zu.
2. Knopf 1 fährt die Startpose an (Frame „Kalibrierstart“: Werkzeug waagerecht, Board-Mitte
   unter der Kamera auf z 0,40); Knopf 2 hält sie.
3. Board mit Gummihülle am **kurzen Rand** flach zwischen die Backen halten, bedruckte Seite
   oben, Knopf 3 (Greifer zu). Knopf 4 öffnet.
4. Knopf 5 startet Stufe 1: etwa 2 s Prüfung ohne Bewegung (Board, Kamerahöhe, Plan), dann
   45 Posen in 3–4 min, zurück in die Startpose. Knopf 6 bricht ab, der Roboter hält die
   letzte Zielpose.
5. Das Ergebnis ersetzt `roboter_tetris/Extrinsics/base_cam_extrinsics.json` im Container;
   die bisherige Datei bleibt daneben als `base_cam_extrinsics_vorher.json`. `base_cam`
   nutzt die neue ab dem nächsten Aktivieren, ohne Build. Werte und Güte im Log
   (`basecam_gegen_vorher_band_mm_mittel`: Verschiebung gegen die bisherige Kalibrierung),
   Rohdaten in `/tmp/base_cam_extrinsics_rohdaten.json`.
   **Dauerhaft** wird sie erst im Repo: mit `docker cp` aus dem Container nach
   `source/roboter_tetris/roboter_tetris/Extrinsics/` holen, committen, bauen — ein Build
   ohne das bringt die Datei aus dem Repo zurück. **Zurück ohne Build:** im Container die
   `_vorher.json` über die Datei kopieren oder in `base_cam` den Parameter „Kalibrierdatei“
   auf `Extrinsics/base_cam_extrinsics_vorher.json` setzen.
6. Knopf 7 prüft ohne Bewegung, ob sich die Kamera bewegt hat (braucht Referenzmarken).

Standard-Posenplan: nur Verschiebung ±8 cm und Drehung um die Hochachse ±20°, Flansch
höchstens 24 cm von der Startpose; in der Anwendung Untergrenze `min_flange_z_m` 0,34
und Greifkraft 100 %.

