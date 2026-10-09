# Einrichtung der Projektanwendung in AICA

Was beim Anlegen der AICA-Anwendung für den Pick im Lauf gesetzt werden muss, mit
allen Messwerten des Aufbaus, und die Bedienung der Kalibrierung (§10). Kopplungen
zwischen Parametern: `systemgraph.md`. Begründungen: `../architektur/entscheidungen.md`
(Abschnittsnummern in Klammern).

Alle Parameter der **eigenen** Komponenten stehen mit den am Aufbau gemessenen Werten
als Standardwert im Paket. Nach dem Build gelten sie auch in bestehenden Blöcken,
sofern der Parameter dort auf dem Standardwert steht; ein von Hand gesetzter Wert
bleibt. Von Hand zu setzen sind nur die Werte der **AICA-Bausteine**, die **Raten**
und die **UR-Nutzlast** (§2, §5).

Grundlage: Messungen am Aufbau (14.–28.09.2026) und die Komponentenbeschreibungen im
AICA-Image (`v2.0.5-jazzy`, core v5.0.0).

---

## 1. Pflicht — ohne diese Werte funktioniert es nicht

| Was | Wert | Warum |
|---|---|---|
| **`rgb_camera.global_time_enabled`** der Basiskamera | **`true`** | Steht er auf `false`, stempelt die L515 in ihrer Hardwareuhr: fremde Epoche, Drift von rund 4 ms/s gegen die ROS-Zeit, und nach wenigen Minuten bleibt der Stempel stehen. Dann verwirft `base_cam` jedes Bild nach dem ersten, und die Objektliste bleibt leer (§3.2). |
| **`depth_module.global_time_enabled`** der Basiskamera | **`true`** | dito |
| `object_follower`: `ws_*`, `observe_*` | Standardwert (§9) | Arbeitsraum und Beobachtungspose sind Pflichtparameter; leer scheitert `configure` mit einer Liste der fehlenden. |

Der AICA-RealSense-Block bietet die beiden `global_time_enabled` nicht als Parameter
an; sie lassen sich nur zur Laufzeit setzen. **`base_cam` erzwingt sie deshalb
selbst:** Ihr Parameter `camera_node` nennt den Knoten der Basiskamera (Standard
`/realsense_camera`), und bei jeder Aktivierung setzt die Komponente dort beide Werte.
Leer schaltet die Funktion ab.

**Knotennamen über `serial_no` prüfen.** Welcher RealSense-Knoten welche Kamera ist,
hängt an der Anwendung. Maßgeblich ist der Serial: **Basiskamera `f1370107` (L515)**,
Roboterkamera `241122074842` (D435i). Den Namen der Basiskamera trägt `camera_node`.

**Neues Paket wirksam?** Ein Rebuild genügt nicht — das Paket wird erst wirksam, wenn
danach das **AICA-Systemabbild im Launcher neu erzeugt** wird; ein Neustart der
Anwendung holt es ohne Fehlermeldung nicht nach. Gegenprobe: Neue Parameter oder
Komponenten sind in der AICA-Bibliothek sichtbar.

Zeitstempel prüfen, nicht annehmen: Die Hardwareuhr wird beim Start einmal
synchronisiert, der Versatz sieht direkt danach klein aus und wächst erst später.
`ros_zeit − header_stempel` über mehrere Minuten mitloggen — konstant ist richtig.

---

## 2. Regelung — von Hand zu setzen

| Baustein | Parameter | AICA-Default | **Wert** | Warum |
|---|---|---|---|---|
| Signal Point Attractor | `linear_gains` | `[1.0]` | **`[5.0]`** | Bei K = 1 läuft der Flansch einem 0,13-m/s-Band über 100 mm hinterher, und das Einschwingen (3/K) dauert 3 s. Mit K = 5 und Vorhalt 0,24 s rund 1 mm Längsfehler. Isotrop halten — ein Wert (§2.2). |
| Signal Point Attractor | `angular_gains` | `[1.0]` | **`[5.0]`** | Sonst dauert das Eindrehen auf gedrehte Klötze rund 3 s. |
| Signal Point Attractor | `max_linear_velocity` | 0,5 m/s | **0,85** | zusammen mit dem IK-Controller |
| Signal Point Attractor | `max_angular_velocity` | 0,5 rad/s | **1,0** | 45° Drehung in rund 0,9 s |
| Signal Point Attractor | `rate` | 10 Hz | **50** | glatte Bewegung beim Folgen |
| IK Velocity Controller | `max_linear_velocity` | 0,25 m/s | **0,85** | **Die bindende Grenze** — sie klemmt unabhängig vom Attractor. Darüber nicht (UR10e typisch 1 m/s, Sicherheitsgrenzen der Steuerung). |
| IK Velocity Controller | `max_angular_velocity` | 0,25 rad/s | **1,0** | wie am Attractor |
| IK Velocity Controller | `command_rate_limit` | ∞ | **3,0** | Gelenkbeschleunigung in rad/s². Ohne Grenze springt der Arm beim Anfahren aus dem Stand — Schutzstopp am Bandrand (§9.5). Bei einem Stopp (C157A2) zuerst die Geschwindigkeit zurücknehmen. |
| UR-Steuerung (Pendant, Installation) | Nutzlast | 2,8 kg | **1,3 kg**, Schwerpunkt 12 / 24 / 45 mm | gemessen mit dem Assistenten der Steuerung, Greifer leer (§9.5) |

**Im `priority_handler` passen die Standardwerte dazu:** `attractor_gain` 5 =
`linear_gains`, `attractor_v_max_mps` 0,85 = das kleinere `max_linear_velocity` aus
Attractor und IK-Controller. Er rechnet damit die Anfahrzeit; werden die Bausteine
anders eingestellt, müssen diese beiden Werte mitgehen.

**Gedrosselte Läufe** (IK-Controller langsamer): Die Grenze gilt für den Betrag der
Geschwindigkeit, Band und Absenken addieren sich — dann auch langsamer absenken
(`descend_speed_mps` 0,05, `t_descend_s` 3,0) und `attractor_v_max_mps` auf den
kleineren Wert setzen. Das echte Band (0,13 m/s) ist mit 0,10 m/s nicht einzuholen;
gedrosselt nur mit `fake_objects.py` fahren.

---

## 3. Kameras

Gebraucht wird nur die **Basiskamera**; die Roboterkamera und ihr RealSense-Block
gehören nicht in die Anwendung (Rechenzeit, §3.5).

| | Basiskamera |
|---|---|
| Typ | RealSense L515 |
| `serial_no` | `f1370107` |
| `rgb_camera.profile` | `1280x720x15` |
| `depth_module.profile` | `640x480x30` (die L515 kann Tiefe nur mit 30) |
| `align_depth.enable` | `true` |
| `global_time_enabled` | `true` (§1) |

15 Bilder je Sekunde genügen: `base_cam` schafft rund 8,6 Messungen je Sekunde, die
halbe Rate entlastet die Treiber. **Profile immer ausdrücklich setzen** — ohne Angabe
gilt ein Treiber-Default, und `base_cam` klemmt einen zu großen Bildausschnitt ohne
Fehlermeldung auf das Bild. Bildraten über die `camera_info`-Topics messen; `ros2
topic hz` auf Bild-Topics misst sich selbst.

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

**`rate` am Block setzen.** Der Parameter ist von `LifecycleComponent` geerbt (Default
10 Hz) und wird nur beim Erzeugen gelesen. Er gehört **nicht** in die eigenen
Komponentenbeschreibungen — das Duplikat lässt die Oberfläche bei jedem Klick ein
weiteres Rate-Feld anlegen (§3.4).

**Alle Python-Komponenten laufen in einem Prozess** und teilen sich wegen der GIL
praktisch einen Kern. Jede unnötig hohe Rate und jeder überzählige Block kostet
`base_cam` Messrate; auch unkonfigurierte Kamerablöcke empfangen ihre Bilder (§3.3).

| Komponente | Rate | Anmerkung |
|---|---|---|
| `base_cam` | **15 Hz** | = Farbbildrate der Kamera |
| `vectoring` | **15 Hz** | wie `base_cam`; Glättungsfenster und Halbfenster zählen Messungen |
| `priority_handler` | **20 Hz** | |
| `object_follower` | **50 Hz** | glatte Zielpose für den Attractor |
| `robotiq_gripper` | **20 Hz** | Takt, mit dem `motion_done`/`has_object` gelesen werden |
| `data_tracker` | **2 Hz** | nur Anzeige |
| `interface_streamer` | **10 Hz** | flüssiges Bild; bei Rechenzeitmangel zuerst senken |

**`debug_enable`** steht bei `base_cam` als Standard an, weil der `interface_streamer`
ihr Debug-Bild zeigt. Das Debug-Bild wird nur verschickt, wenn ein neues gerendert
wurde, nicht in jedem Takt.

**Tracker- und Messregion von `base_cam`** liegen in `world` und umfassen das ganze
Band von Rolle zu Rolle (y −375 … +1080 mm). Hinter dem Bild der Basiskamera
(y ≈ 460 … 1030 mm) führt `vectoring` die Klötze mit der geschätzten
Bandgeschwindigkeit weiter (Status 4). Die Greifzone liegt bewusst **außerhalb** des
Bildes, damit der Greifer nicht als Klotz erkannt wird (§5.2, §6.4).

**Rechenlast im Betrieb:** `rviz2` und zusätzliche Browser-Ansichten schließen, keine
Mitlese-Prozesse parallel. Reicht das nicht, `base_cam` auf Rate 12 (§3.5).

---

## 6. Verdrahtung

Die vollständige Signalliste steht in `systemgraph.md`. Worauf es ankommt:

- **`target_pose` des `object_follower` → Eingang `attractor` des Signal Point
  Attractors**, nicht an den IK-Controller. Typ `cartesian_pose`.
- **`cartesian_state` des Robot State Broadcasters** → `robot_state` von
  `object_follower` **und** `priority_handler` (der braucht den Anfahrweg).
- `target` des `priority_handler` → `target` des Followers.
- `gripper_close` → `robotiq_gripper`; `motion_done` → `gripper_motion_done`,
  `has_object` → `gripper_has_object` des Followers.
- **`picked_id`** → `priority_handler` und `data_tracker`.
- `tracks` → `priority_handler` und `data_tracker`; `not_pickable` → `data_tracker`.
- **`interface_streamer`:** `base_debug_image` ← `debug_image` von `base_cam`,
  `world_state` ← `data_tracker`, `follower_status` ← `object_follower`;
  `interface_image` → RViz.

**Vor dem Start von AICA den Arm freifahren:** Der Attractor fährt beim Start auf die
zuletzt gespeicherte Zielpose. Der Follower selbst startet mit einem senkrechten
Anheben und fährt dann in die Beobachtungspose.

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

**Alle Höhen sind Flanschmaße** (`ur_tool0`), nicht TCP-Maße der UR-Steuerung. Der
Greifer steht nicht im URDF; wer hier einen TCP-Wert einträgt, liegt um 215 mm
daneben.

| Wert | Zahl | Herkunft |
|---|---|---|
| Bandoberfläche in `world` | **53,6 mm** (eben auf ±1 mm) | drei Antastpunkte mit geschlossenem Greifer (§4.3) |
| Querneigung des Bandes | 0,39–0,64°, robotnahe Seite tiefer | Antasten, Farbbild (§4.3) |
| Flansch → Backenspitze (geschlossen) | **245 mm** | Maßstab; beim ersten Griff bestätigt |
| Flansch → Auflagenmitte (Griffpunkt) = **`flange_to_grip_point_m`** | **235 mm** | Backenspitze − halbe Auflagenhöhe (§4.2) |
| TCP der UR-Steuerung | 215 mm | kein Parameter der Kette, nur zur Umrechnung fremder TCP-Werte |
| Greifhöhe Flansch, 100-mm-Klotz stehend | **≈ 339 mm** | 53,6 + 50 + 235 |
| Freihöhe Flansch (Transfer, Abbruch) = `transfer_height_m` | **490 mm** | Band + stehender Klotz + gehaltener Klotz + Luft (§8.7) |
| Ablagepose Flansch | x −316,49 · y +476,21 · z +419,71 mm, Gier 94,2° | geteacht, im Betrieb bestätigt |
| Arbeitsraum `ws_*` | x −1,0 … −0,30 · y −0,32 … +0,48 · z 0,2986 … 0,60 m | abgefahren (§9.3); `Safety/workspace_bounds.json` |
| Greifzone (`priority_handler`) | x −1,0 … −0,53 · y +0,445 … −0,32 m | der Arbeitsraum auf dem Band (§7.1) |
| Greifebene | y ≈ −0,01 m bei 0,128 m/s | gerechnet (§7.2) |
| Bildausschnitt `base_cam` | `roi_x` 342, `roi_width` 618, `roi_y` 60, `roi_height` 580 | Bandrand am Roboter bis x −1,0; der Greifer ragt nicht ins Bild (§5.1) |
| Bandrichtung | praktisch die y-Achse, Lauf nach −y | |
| Band von Rolle zu Rolle | y ≈ +1,08 … −0,375 m | |
| Bild der Basiskamera | y ≈ +0,46 … +1,03 m | |
| Bandgeschwindigkeit | geschätzt −127,9 mm/s; per Stoppuhr 125–133 mm/s | Ziel 3 (§6.5) |
| UR-Nutzlast | 1,3 kg, Schwerpunkt 12 / 24 / 45 mm | §9.5 |
| Extrinsik Basiskamera | −0,7787 · 0,7934 · 0,9163 m · 179,46° · 0,45° · 179,76° | Handkalibrierung L6 in `world`, in Kraft; steht in `Extrinsics/base_cam_extrinsics.json` (Parameter „Kalibrierdatei“), als Rückfall in `cal_*`; Abweichung höchstens 6 mm (§5.3) |
| `belt_surface_z_mm` / `top_depth_bias_mm` (`base_cam`) | 53,6 / 11,5 | §5.1 |
| Prozesszeiten | Einschwingen 0,23–0,33 s · Absenken 0,78–0,93 s (bei 0,25 m/s) · Greifen 0,63–0,83 s · Heben im Mitfahren 0,5–0,7 s · Ablegen 1,3–2,2 s (bei 0,5 m/s) | `priority_handler`: `t_settle_s` 0,4, `t_descend_s` 0,7 (für 0,35 m/s gerechnet), `t_grasp_s` 0,8, `t_lift_s` 0,5, Faktor 1,0 (§7.6) |
| Flacher Klotz 50 × 75 × 25 mm | gemessen 23,8 mm; Oberseite roh 11–15 mm über dem Band | erkannt über `min_contour_area` 1000 bei `min_obj_height` 10; greifbar ab 20 mm (§5.1) |
| Klotzwinkel der Basiskamera | statisch ±1,9°, im Lauf kein Wechsel der langen Kante, Güte 1,00 | §8.6 |
| Takt bei dichter Folge | etwa ein Klotz je 7 s | §13 |

**Greifhöhe:** `flansch_z_greifen = 53,6 + max(klotzhoehe/2, 11) + 235` [mm]. Die
Untergrenze 11 mm (`min_grip_height_m`) hält die geschlossene Backenspitze 1 mm über
dem Band und damit 1 mm über der Arbeitsraumgrenze `ws_z_min` 0,2986. Klötze ab 20 mm
Höhe werden gegriffen, bis 22 mm an der Untergrenze, darüber auf halber Höhe.
**Greifhöhe min und `ws_z_min` nur gemeinsam ändern** — sonst bricht das Gate jeden
flachen Griff ab.

---

## 9. Werte für den `object_follower`

Alle Werte sind Standardwert der Komponente. Der Arbeitsraum entspricht
`source/roboter_tetris/roboter_tetris/Safety/workspace_bounds.json`; ein Test prüft die
Gleichheit (§9.3).

Die Follower-Parameter heißen in der Oberfläche „Gruppe: Klartext [parameter_name]“ in
sieben Gruppen (1 Arbeitsraum … 7 Aufbau); AICA sortiert sie selbst, die Gruppen
stehen nur im Namen. Die Parameter des `priority_handler` tragen den Namen ebenfalls in
Klammern.

| Parameter | Wert | Herleitung |
|---|---|---|
| `ws_x_min` / `ws_x_max` | −1,000 / −0,300 | abgefahren bis −0,530; erweitert für die Ablage (x −0,316) |
| `ws_y_min` / `ws_y_max` | −0,320 / +0,480 | Bandende / abgefahren bis +0,445, erweitert für die Ablage (y +0,476) |
| `ws_z_min` / `ws_z_max` | 0,2986 / 0,600 | Backenspitze auf Bandhöhe / darüber Singularität |
| `observe_x` / `observe_y` | −0,816 / +0,35 | Bandmitte am Anfang der Greifzone; beim Anfahren wartet der Arm am Zonenanfang |
| `observe_z` | 0,45 | Folgehöhe |
| `observe_yaw_deg` | 90° | Backen quer zur Bandrichtung (Grundstellung) |
| `lead_time_s` | 0,24 | Vorhalt ≈ 1/K, eingemessen über `err_laengs` (§2.2) |
| `max_extrapolation_s` / `target_timeout_s` | 1,0 / 1,5 | Deckel der Vorhersage über dem Alter der Messung (§8.7) |
| `descend_speed_mps` | 0,35 | passt zu `t_descend_s` 0,7 im `priority_handler` (§7.6) |
| `min_grip_height_m` | 0,011 | siehe Greifhöhe (§8) |
| `use_block_orientation` | an | im Winkel des Klotzes greifen (§8.6) |
| `orientation_quality_min` | 0,4 | kleine hochkant stehende Klötze blieben unter 0,7 |
| `max_yaw_deviation_deg` | 50 | höchstens ±45° aus der Grundstellung, Hysterese bis 50° |
| `gripper_yaw_offset_deg` | 0 | die Grundstellung greift richtig |
| `transfer_height_m` | 0,49 | Freihöhe (§8) |
| `place_*` | −0,31649 / +0,47621 / 0,41971, Gier 94,2° | Ablagepose (§8) |

Zum Einmessen des Vorhalts `err_laengs` in `follower_status` mitlesen: Im Mittel null
heißt, `lead_time_s` passt zur Verstärkung des Attractors.

---

## 10. Kalibrierung der Basiskamera

Eigene AICA-Anwendung, getrennt von der Greifanwendung: Inhalt von
`docs/uebersicht/anwendung-kalibrierung-basiskamera.yaml` in eine neue Anwendung
(Code-Ansicht) einfügen und speichern. Der Lauf schreibt die **Kalibrierung für
`base_cam`** direkt in die Datei, die `base_cam` liest. Verfahren, Abnahme und
Grenzen: `entscheidungen.md` §5.5. In Kraft ist die Handkalibrierung L6.

**Ablauf** (Band aus):

1. Anwendung starten, **Greiferbacken frei**: Der Greifer fährt beim Laden einmal auf
   und zu.
2. Knopf 1 fährt die Startpose an (Frame „Kalibrierstart“: Werkzeug waagerecht,
   Board-Mitte unter der Kamera auf z 0,40); Knopf 2 hält sie.
3. Board mit Gummihülle am **kurzen Rand** flach zwischen die Backen halten, bedruckte
   Seite oben, Knopf 3 (Greifer zu).
4. Knopf 5 startet Stufe 1: etwa 2 s Prüfung ohne Bewegung (Board, Kamerahöhe, Plan),
   dann 45 Posen in 3–4 min, zurück in die Startpose. Knopf 6 bricht ab, der Roboter
   hält die letzte Zielpose. Danach Board festhalten, Knopf 4 (Greifer auf).
5. Log prüfen (Zeile „Ergebnis: …“): `bildfehler_px` ≤ 0,5, `pruefposen_mm_mittel` ≤ 1,
   `rutschen_mm` ≤ 0,5; `basecam_gegen_vorher_band_mm_mittel` ist die Verschiebung
   gegen die bisherige Kalibrierung (gegen L6 rund 3 mm). Diese Richtwerte sind
   strenger als die Gütegrenzen der Komponente (Bildfehler ≤ 1 px, Prüfposen ≤ 2 mm,
   Rutschen ≤ 0,5 mm / 0,2°): Außerhalb der Gütegrenzen wird nichts geschrieben;
   liegt ein Lauf dazwischen, ist er geschrieben, aber schlechter als die Abnahme vom
   28.09. (0,34 px, 0,54 mm, 0,04 mm) — dann besser wiederholen. Die Rohdaten liegen in
   jedem Fall in `/tmp/base_cam_extrinsics_rohdaten.json`.

**Was der Lauf schreibt:** Das Ergebnis ersetzt die aktive Datei
`Extrinsics/base_cam_extrinsics.json` im Container und legt gleichzeitig eine
unveränderliche Zeitstempelkopie unter `/data/calibration_archive/base_cam/` ab.
`base_cam` nutzt die aktive Datei ab dem nächsten Aktivieren, ohne Build.
**Zurück ohne Build:** in `base_cam` den Parameter „Kalibrierdatei“ auf die
gewünschte konkrete Archivdatei setzen. Die vollständige Ablage steht in
[`kalibrierdateien-ablage.md`](kalibrierdateien-ablage.md). **Dauerhaft** wird
eine bewusst ausgewählte Fassung erst im Repo — ein Build ohne das bringt die
Datei aus dem Repo zurück:

```bash
C=$(docker ps --format '{{.Names}}' | grep aica-launcher | head -1)
docker cp $C:/ws/install/roboter_tetris/lib/python3.12/site-packages/roboter_tetris/Extrinsics/base_cam_extrinsics.json source/roboter_tetris/roboter_tetris/Extrinsics/
```

Danach committen, bauen, Systemabbild neu erzeugen, und
`test_shipped_file_is_the_l6_calibration` anpassen (der Test schützt bis dahin genau
die L6-Datei).

**Nachrechnen** aus den Rohdaten eines Laufs (aus `source/roboter_tetris`, mit numpy
und OpenCV): `PYTHONPATH=. python3 test/tools/basecam_kalibrierung.py ROHDATEN.json
[--out NEU.json] [--vergleich DATEI.json …]`. Ohne `--out` wird nichts geschrieben.

**Kamera als Frame:** Beim Start lädt die Anwendung den Kalibrierblock in der
Betriebsart `anzeigen` und einen Signal-to-Frame-Block. Die 3D-Ansicht zeigt die
Basiskamera als Frame `basiskamera` an `world` (Farbbild-Rahmen der L515: z in
Blickrichtung, x nach rechts, y nach unten im Bild), gelesen aus der Kalibrierdatei.
Überschreibt ein Lauf die Datei, springt der Frame innerhalb einer Sekunde auf die neue
Lage.

**Standardwerte:** Posenplan nur Verschiebung ±8 cm und Drehung um die Hochachse ±20°,
Flansch höchstens 24 cm von der Startpose; in der Anwendung gesetzt sind die
Untergrenze `min_flange_z_m` 0,34 (Handgelenk) und die Greifkraft 100 %. Knopf 7
(`pruefen`) braucht Referenzmarken am Bandgestell, die nicht angebracht sind.
