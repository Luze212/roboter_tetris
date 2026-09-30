# Entwurfsentscheidungen Robotetris

Die geltenden Festlegungen des Systems mit Begründung und den Messwerten, auf denen
sie beruhen. **Stand: finaler Build 24.09.2026, Nachjustierung und Kalibrierung der
Basiskamera 28.09.2026.**

Das Dokument ist nach Themen geordnet. Die Kennungen in Klammern hinter den
Überschriften (etwa „L6“, „Z7“, „Nachtrag 13“) sind die Namen, unter denen Code,
Komponentenbeschreibungen und Messdaten auf eine Entscheidung verweisen. Wo eine
Kennung steht, führt das [Register am Ende](#register-der-kennungen) zum Abschnitt.
Die Entstehung der Entscheidungen ist in der Git-Historie erhalten (Tag
`vor-bereinigung`).

Die Feldbelegung aller Signale steht in `datenvertraege.md`, die Werte zum Einrichten
der Anwendung in `../uebersicht/einrichtung-projektanwendung.md`.

---

## 1. Ziele und Ergebnis (Z1, Z8)

| Nr. | Ziel (Vorgabe) | Umsetzung | Ergebnis am Aufbau |
|---|---|---|---|
| 1 | Ansteuerung des UR10e mit AICA | Zielpose → Signal Point Attractor → IK Velocity Controller (§2) | erfüllt |
| 2 | Schnelles Kalibrierungsverfahren | automatische Kalibrierung der Basiskamera mit dem Roboter, `base_cam_calibration` (§5.5) | etwa 4 min je Lauf, über drei Tage auf unter 1 mm wiederholbar, im Greiflauf 7 von 7 |
| 3 | Geschwindigkeitsschätzung und Positionsberechnung | `base_cam` (Position), `vectoring` (Geschwindigkeit je Klotz und für das Band) (§5, §6) | Position gegen den Roboter höchstens 6 mm; Band −127,9 mm/s gegen 125–133 mm/s per Stoppuhr |
| 4 | Priorisierung und Bahnplanung für das kontrollierte Greifen | `priority_handler` (§7), `object_follower` mit den AICA-Bausteinen (§8) | Griff im Lauf mit rund 1 mm Längsfehler; bei dichter Folge etwa ein Klotz je 7 s; flache und gedrehte Klötze; Dauerlauf zuverlässig |
| – | Den Prozess nachvollziehbar darstellen | `data_tracker`, `interface_streamer` (§11) | Übersichtsbild in RViz |

**Bahnplanung (Z8):** Die Bewegung erzeugen die AICA-Bausteine Signal Point Attractor
und IK Velocity Controller aus der Zielpose des `object_follower`. Die Nutzung der
bereitgestellten Werkzeuge gilt als Lösung der Vorgabe.

**Die Geschwindigkeit wird geschätzt, nicht kalibriert (Z2).** Ziel 3 verlangt ein
Verfahren, das die Bandgeschwindigkeit zur Laufzeit aus den Bilddaten bestimmt. Eine
Stoppuhrmessung dient nur als unabhängige Gegenprobe (B1).

---

## 2. Bewegungsarchitektur (Thema 1, Z6, L18)

### 2.1 Zielpose statt Geschwindigkeit

Der `object_follower` gibt eine **Zielpose des Flansches** aus (`cartesian_pose`,
`world`), die an den Eingang `attractor` des Signal Point Attractors geht. Der
Attractor rechnet daraus eine Geschwindigkeit, der IK Velocity Controller setzt sie
in Gelenkbewegungen um, das Hardware-Interface läuft mit 500 Hz.

Eine eigene Geschwindigkeitsvorgabe an den IK-Controller wäre regelungstechnisch
gleichwertig gewesen (der Attractor ist ein P-Regler), hätte aber einen neuen
Signalweg an einer getesteten Kette bedeutet. Die Zielpose hat außerdem eine
wichtige Sicherheitseigenschaft: Bleibt das Kommando aus, hält der Attractor die
letzte Pose, der Roboter fährt sie an und bleibt stehen (§9.1).

### 2.2 Vorhalt als Zeit (Z6)

Ein Point Attractor erzeugt Geschwindigkeit nur bei Abstand zum Ziel:
`v = K · (x_ziel − x_flansch)`. Einem fahrenden Klotz läuft der Flansch deshalb
dauerhaft um `v_band / K` hinterher — bei K = 1 und 0,13 m/s rund 130 mm, bei K = 5
rund 26 mm. Der Follower legt die Zielpose deshalb um einen **Vorhalt** vor den Klotz:

```
zielpose = vorhergesagter Klotz + v_band · (lead_time_s + latency_compensation_s)
```

Der Vorhalt ist eine **Zeit**, keine Strecke: Er bleibt richtig, auch wenn die
geschätzte Bandgeschwindigkeit leicht schwankt, und er wird bei einem stehenden Ziel
von selbst null (Warte- und Ablagepose). Theoretisch ist `lead_time_s ≈ 1/K`;
eingemessen am Roboter **0,24 s bei K = 5** (L18, L19). `latency_compensation_s`
bleibt 0 — die Kameralatenz steckt in der Vorhersage auf den aktuellen Zeitpunkt
(§3.1).

**Einmessen (G7):** Im eingeschwungenen `FOLGEN` ist `err_laengs` in S8 der
Restabstand zwischen Flansch und vorhergesagtem Klotz (positiv = Flansch voraus).
`lead_time_s` passt, wenn er im Mittel null ist. Der Follower überwacht das selbst:
Nach 2 s in `FOLGEN` warnt er einmal je Versuch, wenn das gleitende Mittel
(τ = 0,5 s) 10 mm übersteigt. Das fängt eine im Studio geänderte Verstärkung ab, zu
der der Vorhalt nicht mehr passt.

### 2.3 Einstellungen der AICA-Bausteine

| Baustein | Wert | Grund |
|---|---|---|
| Attractor `linear_gains` / `angular_gains` | [5.0] / [5.0] | K = 1 (Standard) ergibt 3 s Einschwingen und 130 mm Nachlauf; mit K = 5 und Vorhalt rund 1 mm Längsfehler. Linear isotrop halten, sonst wird der Vorhalt achsabhängig. Drehung ebenfalls 5, sonst dauert das Eindrehen auf gedrehte Klötze rund 3 s (L26). |
| Attractor `rate` | 50 Hz | glatte Bewegung beim Folgen |
| Attractor und IK `max_linear_velocity` | 0,85 m/s | deutlich über der Bandgeschwindigkeit; UR10e typisch 1 m/s; der Gewinn über 0,85 läge auf 0,5 m Weg unter 0,15 s (L29) |
| Attractor und IK `max_angular_velocity` | 1,0 rad/s | 45° in rund 0,9 s (L26) |
| IK `command_rate_limit` | 3,0 rad/s² | ohne Beschleunigungsgrenze springt der Arm beim Anfahren aus dem Stand → Schutzstopp (L20, §9.5) |

Die Grenze des IK-Controllers ist die bindende: Sie wirkt unabhängig davon, was der
Follower berechnet. Sie gilt für den **Betrag** der Geschwindigkeit — Band und
Absenken addieren sich (L18).

**`is_in_range` des Attractors taugt nicht als Greif-Freigabe:** Der Vorhalt hält
den Abstand absichtlich ungleich null. Die Freigabe rechnet der Follower selbst
(§8.4). Auch „angekommen“ an Warte- und Ablagepose prüft er selbst gegen
`pose_tolerance_m` (0,01 m), weil ein Predicate keiner Komponente als Signal zur
Verfügung steht.

**Nicht verfolgt:** ein mitbewegter Bezugsrahmen am Eingang `base_frame` des
Attractors (Option C). Der Eingang ist `cartesian_pose` und trägt keine
Geschwindigkeit; der Vorhalt erreicht rund 1 mm (V4).

---

## 3. Zeit, Latenz und Rechenleistung (Thema 2, M2, M3, K2, L2, L3, L16, L29)

### 3.1 Zeitstempel aus dem Bild

Jedes Signal trägt als `t` den **Header-Stempel des Kamerabildes**, nicht die Zeit der
Verarbeitung. Eine konstante Verzögerung ist harmlos, eine schwankende nicht; eine
Zeitabfrage im Step-Callback hätte bis zu einer Taktperiode Jitter in Schätzung und
Vorhersage getragen.

Der Follower rechnet die Position auf den aktuellen Zeitpunkt vor:
`p = p₀ + v_band · (t_jetzt − t₀)`, begrenzt auf `[0, max_extrapolation_s]` (G8).
Das überbrückt die Latenz (Bild bis Ankunft im Median 139–161 ms) und die Lücken
zwischen zwei Messungen (§3.3).

### 3.2 Zeitdomäne der L515 (M2, M3)

Mit `global_time_enabled = false` (Treiber-Standard der L515) stempelte die
Basiskamera in ihrer Hardwareuhr: Epoche 2006, Drift +4,05 ms/s, nach einigen Minuten
blieb der Stempel ganz stehen. `base_cam` verwarf dann jedes Bild nach dem ersten.
Mit `true` driftet der Stempel +0,02 ms/s; der Versatz von rund 45–50 ms ist die
Laufzeit von der Belichtung bis zur Ankunft.

Der AICA-RealSense-Block bietet den Parameter nicht an. **`base_cam` erzwingt ihn
deshalb selbst**: Über den Parameter `camera_node` (Standard `/realsense_camera`)
setzt sie bei jeder Aktivierung `rgb_camera.global_time_enabled` und
`depth_module.global_time_enabled` über den ROS-Parameterdienst — nicht blockierend,
mit Wiederholung, falls der Kameraknoten später startet.

### 3.3 Ein Prozess für alle Python-Komponenten (L2)

AICA lädt alle Python-Komponenten in **einen** Prozess (`component_container_mt`).
Wegen der GIL teilen sie sich praktisch einen Kern. Jede zusätzliche Komponente und
jede unnötig hohe Rate kostet `base_cam` Messrate; auch unkonfigurierte Kamerablöcke
abonnieren ihre Bilder.

Zweiter Befund: modulo legt jeden Eingang mit einer Warteschlange der Tiefe 10 an.
Die Bilder stauten sich, `base_cam` rechnete auf rund 0,4 s alten Bildern. **`base_cam`
abonniert ihre Bildeingänge mit Tiefe 1** (`QoSProfile(depth=1)`); das Alter bei
Ankunft sank von 409 auf 139 ms.

| Messgröße (Betrieb) | Wert |
|---|---|
| neue Messungen von `base_cam` | rund 8,6 je Sekunde |
| Alter einer Messung bei Ankunft | Median 139–161 ms |
| Alter vor der nächsten Messung | Median 263 ms, 95 % 357 ms |
| Alter des Ziels (S4) im Follower | Median 0,52 s, höchstens 0,93 s (L23) |

### 3.4 Raten (K2, L29)

`rate` ist ein Parameter der Basisklasse `LifecycleComponent` (Standard 10 Hz), wird
nur beim Erzeugen gelesen und **am Block in der Oberfläche** gesetzt. In den eigenen
Komponentenbeschreibungen darf er nicht wiederholt werden — das Duplikat lässt die
Oberfläche bei jedem Klick ein weiteres Rate-Feld anlegen.

| Komponente | Rate |
|---|---|
| `base_cam`, `vectoring` | 15 Hz (= Farbbildrate der Kamera) |
| `priority_handler`, `robotiq_gripper` | 20 Hz |
| `object_follower` | 50 Hz |
| `interface_streamer` | 10 Hz |
| `data_tracker` | 2 Hz |

### 3.5 Grenzen des Rechners (L3, L16, L29)

Der Rechner hat keine NVIDIA-Grafik (Intel Core 3 100U, 2 Performance- und 4
Effizienzkerne). Die Kameratreiber und der 500-Hz-Regelkreis laufen im selben Prozess
(`event_engine`); bei Überlast fällt die Schleife zeitweise auf 54–89 %, im
schlimmsten Fall läuft die RTDE-Verbindung über und die UR-Steuerung stoppt External
Control. Entlastet wird deshalb: Farbbild der Basiskamera mit 15 Bildern/s, die
Roboterkamera nicht in der Anwendung (§12), im Betrieb `rviz2` und zusätzliche
Browser-Ansichten schließen, keine Mitlese-Prozesse parallel. Reicht das nicht,
`base_cam` auf Rate 12.

---

## 4. Bezugssysteme und Maße (Thema 3, M8–M11, Z7, K5, L7)

### 4.1 `world` und der Flansch

- **Alle Positionen in `world`**, identisch mit `ur_base_link`, dem System, in dem
  AICA regelt. Jede `CartesianPose` trägt ihren `reference_frame` ausdrücklich.
- Der UR-Rahmen `base` ist dagegen **um 180° um z gedreht** (`tf2_echo ur_base_link
  ur_base`: exakt 180°, keine Verschiebung). Werte aus RTDE oder vom Pendant stehen in
  `base` — daher lag die Kalibrierung des Vorgängerprojekts auf der anderen Seite des
  Roboters (K5).
- **Der Greifer steht nicht im URDF.** Der IK-Controller regelt den Flansch
  `ur_tool0`, der `robot_state_broadcaster` meldet den Flansch. Jede kommandierte und
  gemessene Pose ist eine **Flanschpose**. Das URDF bleibt unverändert, der Follower
  rechnet den Versatz selbst.
- **SI-Einheiten an allen Signalgrenzen** (m, m/s, rad, s). Die Bildverarbeitung
  rechnet intern in mm; umgerechnet wird genau einmal, beim Packen.

### 4.2 Drei Abstände unter dem Flansch (Z7, M8)

| Abstand | Wert | Verwendung |
|---|---|---|
| konfigurierter TCP der UR-Steuerung | 215 mm | **nur** Umrechnung fremder TCP-Werte: `flansch_z = tcp_z + 0,215` |
| Flansch → Griffpunkt (Mitte der Auflagen) | **235 mm** | Follower-Parameter **`flange_to_grip_point_m`** |
| Flansch → geschlossene Backenspitze | 245 mm (±5 mm, mit Maßstab) | Höhen am Aufbau |

Der Griffpunkt liegt auf der Flanschachse (TCP der Steuerung x = y = 0, keine
Verdrehung), der Versatz ist damit exakt eine Zahl in z. Die Verwechslung mit den
215 mm hätte die Backenspitze bei einem 25-mm-Klotz 17,5 mm ins Band gefahren; der
Parameter heißt deshalb bewusst nicht „tool offset“.

### 4.3 Aufbau in Zahlen

| Größe | Wert | Herkunft |
|---|---|---|
| Bandoberfläche | z = **53,6 mm** in `world`, eben auf ±1 mm | drei Antastpunkte, geschlossener Greifer (B17, M9) |
| Querneigung des Bandes | 0,39° (Antasten), 0,64° quer / 0,25° längs (Farbbild, L27); robotnahe Seite tiefer | bei x −0,70 rund 52,9 mm, bei x −0,93 rund 54,5 mm |
| Bandrichtung | praktisch −y (quer < 1,1 mm/s) | M10, L9 |
| Band von Rolle zu Rolle | y ≈ +1,08 … −0,375 m | L7 |
| Band quer | x ≈ −0,49 … −1,25 m im Bild; Greifbereich x −0,53 … −1,0 | L21 |
| Bild der Basiskamera | y ≈ +0,46 … +1,03 m | L7 |
| Bandgeschwindigkeit | −127,9 mm/s geschätzt; 125–133 mm/s per Stoppuhr | L9, L10 |
| Ablagepose (Flansch) | x −316,49 · y +476,21 · z +419,71 mm, Gier 94,2°, 0,94° aus dem Lot | geteacht (B9, M11), im Betrieb bestätigt |
| Greiferöffnung | 127 mm | B16 |
| Greifauflage | 20 mm hoch, 15 mm breit | B15 |
| UR-Nutzlast | 1,3 kg, Schwerpunkt 12 / 24 / 45 mm | Assistent der Steuerung (L20) |

---

## 5. Basiskamera: Erkennung und Kalibrierung (Z4, M5, L4, L6, L21, L24, L27, L28)

### 5.1 Erkennung (L6)

`base_cam` erkennt die Klötze im ausgerichteten Tiefenbild der L515 (Grundlage: die
C++-Erkennung des Vorgängerprojekts, nach Python übertragen), die Farbe kommt aus dem
Farbbild.

- **Tiefe der Oberseite = Median** der Tiefe über den Umriss. Alle vier Ecken werden
  mit dieser einen Tiefe umgerechnet. Vorher las jede Ecke die Tiefe ihres eigenen
  Kantenpixels — also das Band —, und die Oberseite eines 100-mm-Klotzes lag um
  16 mm versetzt (Parallaxe zum Bildrand).
- **Höhe = z der Oberseite in `world` − Bandoberfläche** (`belt_surface_z_mm` 53,6).
  `z` in S1 ist die Klotzmitte.
- **`top_depth_bias_mm` = 11,5:** Die L515 liest Klotzoberseiten um 11,5 ± 1,6 mm zu
  tief (der LiDAR dringt in den Kunststoff ein), das Band nicht. Der Wert wird von der
  Oberseitentiefe abgezogen; danach flacher Klotz 23,8 mm (echt 24,2), hoher 96,2 mm
  (echt 100,4).
- **Erkennungsschwellen (L24, L28):** `min_obj_height` 10 mm wirkt auf die rohe Tiefe
  vor der Korrektur — ein flacher 25-mm-Klotz liegt roh nur 11–15 mm über dem Band,
  auf der tieferen, robotnahen Bandseite noch weniger. `min_contour_area` 1000 px
  erfasst den Teil der Oberseite über der Schwelle. Mit 9 mm gab es Fehlerkennungen.
- **Bildausschnitt (L21):** `roi_x` 342, `roi_width` 618, `roi_y` 60, `roi_height`
  580 — vom Bandrand am Roboter bis x −1,0 (Grenze des Arbeitsraums; u 960 entspricht
  x −1,03 auf Höhe einer 100-mm-Oberkante). Klötze jenseits von x −1,0 sind nicht
  erreichbar und fallen heraus.
- **Tracker (Z4, L10):** Bei vorhandener Detektion gilt immer die **gemessene**
  Position — der übernommene Code rechnete außerhalb einer Messregion die
  Längsposition aus der Geschwindigkeit fort, was die Geschwindigkeitsschätzung
  zirkulär gemacht und ein Umkippen verborgen hätte. Die Messregion umfasst das ganze
  Band (y −375 … +1080 mm); ein Track wird nach drei Fehlbildern gelöscht. S1 enthält
  damit nur Messungen, das Weiterführen hinter dem Bild macht `vectoring` (§6.4).

**Einzelbilder tragen keine Geometrie (M5).** Am ruhenden Klotz über 299 Messungen:
Position σ 0,2–0,6 mm, Höhe σ 0,6 mm — aber die Grundfläche streut über 26 mm (Länge
49–75 mm bei echt 50), weil die Tiefenkontur unterschiedlich weit auf die
Seitenflächen greift. Abmessungs- und Orientierungsprüfungen gehören deshalb auf die
geglätteten Werte aus S3.

### 5.2 Greifer außerhalb des Bildes (L4)

Steht der Greifer tief über dem Band im Bild, erkennt `base_cam` die offenen Backen
als zwei Klötze — mit neuen IDs, falschen Geschwindigkeiten und Geistertracks.
**Greifzone und Wartebereich liegen deshalb außerhalb des Bildes der Basiskamera**
(Bild endet bei y ≈ +0,46, Zone ab y +0,445). Die Strecke zwischen Bildrand und Griff
überbrückt `vectoring` mit der gepoolten Bandgeschwindigkeit (§6.4).

### 5.3 Die gültige Kalibrierung: L6

In Kraft ist die **Handkalibrierung L6** in
`roboter_tetris/Extrinsics/base_cam_extrinsics.json` (`method: uebergang_L6`), die
`base_cam` über den Parameter „Kalibrierdatei“ liest; ist die Datei leer oder
unlesbar, gelten die gleichen Werte aus `cal_*`.

| `cal_x` | `cal_y` | `cal_z` | `cal_roll` | `cal_pitch` | `cal_yaw` |
|---|---|---|---|---|---|
| −0,7787 m | 0,7934 m | 0,9163 m | 179,46° | 0,45° | 179,76° |

Konvention `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, Kamera (Farbbild-Rahmen) → `world`.

Herleitung: **Neigung** aus einer Ebene, angepasst an ein Tiefenbild des Bandes
(457 000 Punkte, Rest-σ 1,3 mm) — die Kamera schaut auf 0,7° senkrecht aufs Band;
**Gier und x/y** aus fünf Punkten, an denen Kamera und Roboter (Antasten mit
geschlossenem Greifer) denselben Klotz maßen; **z** so, dass das Band bei 53,6 mm
liegt. Rest über die fünf Punkte ≤ 3,7 mm; Gegenprobe nach dem Build 1,5 mm (flacher
Klotz) und 5,6 mm (100-mm-Klotz). Rohdaten: `bilder/2026-09-22-b23-punkte.json`,
`bilder/2026-09-23-b23-punkte.json`.

Die vorher vorhandene Kalibrierung stammte aus dem Vorgängerprojekt, stand im
UR-Rahmen `base` (180°, §4.1) und trug 13° Gier und 9° Neigung, die es am Aufbau nicht
gab (Abweichung 47–98 mm, K5, K6).

### 5.4 Das Tiefenbild der L515 ist gegen das Farbbild verkippt (L27)

Dieselbe Board-Ebene, über die Marker im Farbbild und über das Tiefenbild gemessen
(13 Aufnahmen, 0,53–0,85 m Abstand): **1,0–1,8° Verkippung um die waagerechte
Bildachse**, am Ort des Klotzes 5,5–13,5 mm zu tief, mit der Bildzeile wachsend.
Ausgeschlossen: Maßstab von Board oder Brennweite (s = 0,9991), Helligkeit
(≤ 0,5 mm), ein gedrehtes Tiefenbild (Umriss 2–7 px versetzt, eine Drehung ergäbe
22 px). Der Fehler steckt im Tiefenwert.

**Modell M1s:** `e = c0 + c1·X + c2·Y + c3·z` im Kamerarahmen, ohne Entzerrung (das,
was `base_cam` hat); c = 3,715 mm · 3,359 · 21,812 · 4,427 mm/m. Kreuzvalidiert je
Aufnahme höchstens 1,5 mm, rms 0,7 mm (ohne Modell 5,5–13,4 mm).

Folge: Die Lage der **Farbkamera** ist nicht die Kalibrierung für `base_cam`. Mit ihr
wären die Klotzhöhen am unteren Bildrand um bis zu 13 mm falsch. L6 hat Verkippung
der Tiefe und Bandneigung über die Bandebene im Tiefenbild mit aufgenommen.

Messdaten und Auswerteskripte: `bilder/2026-09-25-basiskamera-kalibrierung/`.

### 5.5 Automatische Kalibrierung (L27)

Ein eigenes Verfahren in diesem Paket, **nur für die Basiskamera**: Komponente
`base_cam_calibration` (Ablauf `calibration_run.py`, Mathematik `basecam_extrinsics.py`,
ohne ROS), eigene AICA-Anwendung `../uebersicht/anwendung-kalibrierung-basiskamera.yaml`.

**Stufe 1 (`stufe1`):** Der Greifer hält ein AprilGrid (calib.io, 7 × 11, Tag 20 mm,
Abstand 6 mm, 36h11) flach ins Bild. Der Block fährt 40 Posen, 4 Prüfposen und zum
Schluss die erste noch einmal (Rutschprüfung), etwa 4 min. Gelöst werden Farbkamera →
`world` und Board → Flansch gemeinsam (Startwert aus `cv2.calibrateHandEye` oder der
bisherigen Kalibrierung, dann Ausgleich über alle Tag-Ecken). Daraus baut
`basecam_pose` die **Lage für die Sicht von `base_cam`** — wie L6, nur automatisch:

- **Neigung und Höhe** aus der Bandebene im Tiefenbild,
- **Gier und x/y** aus Punktpaaren auf Arbeitshöhe (Band, 25, 50, 100 mm) im
  Bildausschnitt: wo die Farb-Lage einen Punkt sieht, gegen wo `base_cam` ihn mit der
  vom Modell M1s vorhergesagten Tiefe hinlegt. Angepasst wird auf Arbeitshöhe — auf
  Greiferhöhe lag der Fehler auf dem Band bei bis zu 12 mm.

Geschrieben wird nur innerhalb der Gütegrenzen (Bildfehler ≤ 1 px, Prüfposen ≤ 2 mm,
Rutschen ≤ 0,5 mm / 0,2°), direkt in die Datei, die `base_cam` liest; die bisherige
bleibt als `…_vorher.json`. Rohdaten schreibt jeder Lauf. Betriebsart `anzeigen` gibt
die Kamera aus der Kalibrierdatei als Pose aus (Frame `basiskamera` über
`SignalToTf`) und folgt der Datei.

**Randbedingungen am Aufbau:**
- Die Greifer-Hardware darf nicht verändert werden (auch von anderen genutzt). Eine
  Gummihülle am Board klemmt es; gekippt bewegt es sich trotzdem im Griff (0,5°).
  Der Plan **verschiebt nur (±8 cm) und dreht um die Hochachse (±20°)**; die so nicht
  beobachtbare Kamerahöhe kommt aus dem Band im Tiefenbild.
- OpenCV liest die Tags des Boards um 180° gedreht; die Ecken werden nach Schärfen
  gelesen und über die Board-Lage nachgesetzt (0,2 px statt 2 px).
- Flansch höchstens 24 cm von der Startpose, Untergrenze z 0,34 m (Handgelenk),
  Greifkraft 100 % — in der Anwendung gesetzt.

**Offline geprüft** an 12 Board-Aufnahmen auf Band und Klotz: Lage 3,3 mm im Mittel,
höchstens 7,0 mm (L6: 4,3 / 8,4 mm); Board-Höhe über das Band auf 1,8 mm gleichmäßig
(L6: 1,3), 100-mm-Klotz 100,0 mm (L6: 99,9). In den unteren Ecken des Bildausschnitts
bleiben bei beiden rund 11 mm — die Grenze einer starren Lage ohne Entzerrung und
Tiefenkorrektur in `base_cam`.

**Abnahme am 28.09.2026** (Kriterium: mindestens so gut wie L6): Kalibrierlauf mit
40 Posen, 0,34 px, Prüfposen 0,54 mm, Rutschen 0,04 mm; gegen L6 3,1 mm im Mittel,
3,8 mm höchstens auf dem Band. Gegen den Lauf vom 25.09. 0,3 / 0,5 / 0,8 mm in x/y/z
und ≤ 0,03°, auf dem Band 0,9 mm im Mittel — **über drei Tage wiederholbar**.
Greiflauf mit dieser Kalibrierung in `base_cam`: **7 von 7** abgelegt, kein Fehlgriff.
Das geplante Antasten entfiel aus Zeitgründen.

**Entscheidung:** `base_cam` bleibt unverändert — Ziel 2 verlangt ein schnelles,
wiederholbares Verfahren, nicht mehr Genauigkeit als L6, und das greifende System wird
nicht angefasst. **In Kraft bleibt L6** (seit dem finalen Build erprobt). Die
Kalibrierung vom 28.09. liegt als Messdatum in
`bilder/2026-09-28-basiskamera-kalibrierung/`.

Bedienung, Übernahme ins Repo und Rückweg: `einrichtung-projektanwendung.md` §10.

---

## 6. Geschwindigkeitsschätzung — `vectoring` (Z2, Z3, Z9, Z13, L9, L10, D20, D21)

### 6.1 Zwei Stufen

1. **Je Klotz ein Geschwindigkeitsvektor** `(vx, vy)` aus der eigenen
   Positionshistorie — Richtung und Betrag zugleich, ohne Bandmodell.
2. **Ein Pool** über alle Klötze des Durchlaufs, die final sind und sich bewegen: eine
   gemeinsame Ausgleichsrechnung über **alle ihre Messungen seit dem Einschwingen** —
   gemeinsame Steigung, je Klotz ein eigener Achsenabschnitt. Das ist die
   **Bandgeschwindigkeit**, mit der Vorhersage und Priorisierung rechnen. Sie wird mit
   jeder Messung genauer.

Der Pool ist richtig, weil das Band mit konstanter Geschwindigkeit läuft und die
Klötze frei mitfahren: Alle finalen Klötze messen dieselbe Größe. Über die volle Bahn
gemessen ist die Schätzung 160- bis 280-mal genauer als ein Schnappschuss je Klotz
(Simulation, Z9). Zeiten werden relativ zum Abschnittsbeginn summiert (Zeitstempel um
1,7·10⁹ s würden sonst auslöschen).

**Nur bewegte Abschnitte (L9):** `pool_min_speed_mps` = 0,05. Stehende Objekte — auch
erkannte Greiferbacken — begruben sonst die bewegten Abschnitte, der Pool stand auf 0.

Ein **Durchlauf** ist eine Aktivierung der Komponente; beim Aktivieren wird der Pool
geleert. Solange er leer ist, meldet S3 `n_pool = 0` — kein stilles `0,0` (die Falle
von `v_band` aus `base_cam`, das beim Start 0 meldet, als stünde das Band).

### 6.2 Einschwingen (Z3, Z9)

Die Klötze werden frei aufgelegt und laufen mit dem Band; festhängende gibt es nicht.
Ein Klotz kann aber **beim Aufsetzen umkippen**. Jeder Track durchläuft deshalb zwei
Phasen:

| Status | Bedeutung | Geometrie |
|---|---|---|
| 3 einschwingend | Geschwindigkeit noch nicht konstant gemessen | letzte Einzelmessung, nur Anzeige |
| 0 final | konstant gemessen; wählbar, nicht mehr in Frage gestellt | ab hier gemittelt |

**Kriterium:** Die Geschwindigkeit der jüngeren Halbsekunde stimmt mit der älteren
überein (`v_change ≤ settle_v_tolerance` 0,010 m/s, Halbfenster `settle_half_window`
= 5 Messungen). Ein kippender Klotz springt im Schwerpunkt; solange der Sprung in einem
der beiden Fenster liegt, weichen sie ab. Das zuerst erwogene Kriterium
„Standardfehler der Steigung klein“ versagte genau beim Umkippen: Eine Gerade durch
einen Sprung schluckt ihn als Steigung (in der Simulation 100 % fälschlich final, bis
31 mm/s Fehler; mit den zwei Halbfenstern 0 %, ≤ 1,7 mm/s).

Position, Abmessungen und Orientierung werden **erst ab „final“** gemittelt — sonst
entstünde aus stehend und liegend ein Mischwert, und die Höhe bestimmt die Greifhöhe.
Am Band werden Klötze nach rund 1 s final.

**Ausreißer (D21):** Liegt eine Messung mehr als `outlier_distance_m` (0,02 m) neben
der Vorhersage, wird sie verworfen, der Track bleibt. Hält die Abweichung
`outlier_persist_frames` (3) Bilder an, ist sie eine echte Lageänderung: Die
Schätzung beginnt neu (Status 3), ein offener Abschnitt bleibt im Pool.

### 6.3 Orientierung (Z13)

Die Orientierung ist achsenartig ([0, π)). Gemittelt wird **über den doppelten
Winkel**: `½ · atan2(Σ sin 2θ, Σ cos 2θ)` — ein arithmetisches Mittel aus 0° und 90°
ergäbe 45°, genau die Lage, in der der Greifer die Ecken erwischt. Die Länge des
resultierenden Vektors ist die **Güte** `ori_quality` (0…1). Sie reist in S3 und S4
mit; **bewertet wird sie beim Verbraucher** (Follower, §8.6). Am Band liegt sie bei
1,00, auch bei fast quadratischen Oberseiten (L26).

Glättungsfenster `smoothing_window` = 20 Messungen (bei 15 Hz rund 1,3 s; 30 kosteten
bei schnellerem Takt zu viel Zeit im Bild, L29).

### 6.4 Weiterführen hinter dem Bild (L10)

Die Basiskamera sieht nur y ≈ +1,03 … +0,46; die Greifzone liegt dahinter (§5.2).
Ein **finaler** Track, der im letzten Bild fehlt, wird mit der gepoolten
Geschwindigkeit fortgeschrieben — **Status 4 „vorhergesagt“**, höchstens
`predict_max_s` = 8 s (Bildrand bis Bandende bei 0,13 m/s ≈ 7 s). Er ist Kandidat wie
Status 0, trägt aber nichts zum Pool bei. Taucht innerhalb von `handover_distance_m`
(0,05 m) eine neue Messung auf, endet die Vorhersage — der Klotz ist unter neuer ID
wieder im Bild. Einschwingende Tracks werden nicht vorhergesagt, sondern nach
`track_expiry_s` (1,0 s) vergessen.

### 6.5 Ergebnis am Band (L9, L10)

150 s, vier Klötze (hoch und flach): **Pool −127,9 mm/s**, quer −0,1 mm/s, ab dem
ersten finalen Klotz stabil (−127,8 … −128,1). Stoppuhr: 125–133 mm/s. Je Klotz
−126,6 … −128,0 mm/s. Alle Klötze nach 0,9–1,3 s final und ab dem Bildrand als Status 4
bis hinter das Bandende geführt, ohne doppelte IDs. Vorhersagefehler in der Greifzone
nach Rechnung ≤ 3 mm (Pool-Streuung < 0,5 mm/s × ≤ 6 s); die Griffe bestätigen rund
1 mm Längsfehler.

---

## 7. Zielauswahl — `priority_handler` (P1, P4, E10, Z11, N3, N4, H1–H5, L21, L23, L24)

### 7.1 Greifzone (L21, B19)

Die Greifzone ist ein Rechteck in `world` und gehört allein dem `priority_handler`:
**x −1,0 … −0,53, y −0,32 … +0,445** — der abgefahrene Arbeitsraum auf dem Band, ohne
eigenen Sicherheitsabstand. y +0,445 ist der Rand, an dem der Greifer gerade nicht im
Bild ist, y −0,32 das Bandende (danach senkt sich das Band).

Die Längsgrenzen gelten entlang der geschätzten Bandrichtung (`contracts.along_belt`,
§10.3); bei schrägem Band gelten die beiden mittleren der vier Eckprojektionen. Unter
0,01 m/s gilt das Band als stehend — dann gibt es keine Richtung und kein Ziel.

### 7.2 Die Greifebene (Z11, E10)

Die letzte Position entlang des Bandes, von der aus der ganze Greifprozess noch vor
dem Zonenende fertig wird:

```
s_greifebene   = s_zonenende − |v_belt| · t_greifprozess · grasp_time_margin
t_greifprozess = t_descend_s + t_grasp_s + t_lift_s          = 0,7 + 0,8 + 0,5 s
```

Mit `grasp_time_margin` 1,2 und 0,128 m/s liegt sie rund 0,31 m vor dem Zonenende, bei
y ≈ −0,01 m. Sie wird gerechnet und nicht eingetragen, weil sie an der geschätzten
Geschwindigkeit hängt. Sie reist als S4 Feld 15 zum Follower; dort ist sie **ein Tor
für den Beginn des Absenkens**, keine Grenze für einen laufenden Griff (§8.3).

Ein Ziel darf **stromaufwärts der Zone** gewählt werden (E10): Der Follower wartet dann
am Zonenanfang auf der Spur des Klotzes. Liegt die Greifebene stromaufwärts des
Zonenanfangs, ist die Zone für diese Geschwindigkeit zu kurz; Predicate
`is_zone_feasible` meldet das.

### 7.3 Kandidaten

Ein Track ist Kandidat, wenn alle Punkte gelten:

1. Status 0 oder 4 (`contracts.TRACK_SELECTABLE`), noch nicht versucht.
2. **Greifbar (H2, L24):** Höhe ≥ `min_graspable_height_m` (0,02 m) und
   Diagonale `√(l² + b²) ≤ max_gripper_opening_m − gripper_margin_m` (0,127 − 0,01 =
   0,117 m). Die Diagonale passt bei jedem Gierwinkel; den Winkel entscheidet der
   Follower, der `priority_handler` kennt ihn nicht. Ein nicht greifbarer Klotz wird
   einmal ins Log geschrieben.
3. **Auf einer Spur durch die Zone (H3):** Die Stelle, an der er die Greifebene
   erreicht, liegt in der Zone. Ein Klotz neben der Zone wird nie gewählt, einer davor
   schon.
4. **Erreichbar (P1, N4, H1, L23, L24):**

```
t_verfügbar = (s_greifebene − s_klotz) / |v_belt|
t_benötigt  = waagerechter Abstand Flansch → Anfahrpunkt / attractor_v_max_mps
              + 3 / attractor_gain + t_settle_s
Kandidat  ⟺  t_verfügbar > reach_safety_factor · t_benötigt
```

- Anfahrpunkt ist der Klotz selbst oder, solange er vor der Zone liegt, der Punkt auf
  seiner Spur an `zone_upstream`. Der Abstand ist **waagerecht** — die Höhe ändert sich
  erst beim Absenken, dessen Zeit in der Greifebene steckt.
- `3/K` ist das Einschwingen des Attractors, `t_settle_s` (0,4 s) das Einschwingen des
  Followers bis zur Greif-Freigabe (gemessen 0,23–0,33 s).
- Die Flanschpose kommt als `robot_state` vom Robot State Broadcaster (kein Zyklus: das
  Signal kommt aus der Hardware). Steht der Roboter weit weg, gilt korrekt nichts als
  erreichbar.
- `attractor_gain` 5 und `attractor_v_max_mps` 0,85 müssen zu den Einstellungen der
  AICA-Bausteine passen (§2.3).

**Sicherheitsfaktor 1,0 (L24):** Gemessen liegt die Grenze bei Faktor ≈ 1,0 — mit 0,6
kamen alle aus der Ablagepose gewählten Klötze zu spät (Verhältnis verfügbar/benötigt
0,61–0,91), mit 1,5 und `t_settle_s` 1,2 wurde aus der Ablagepose kein Klotz in der
Zone mehr gewählt.

Unter den Kandidaten wird der **dringendste** gewählt: der, der die Greifebene als
nächstes erreicht.

### 7.4 Ziel-Lock (P4)

Einmal gewählt, bleibt ein Ziel gewählt. Ein später auftauchender Klotz löst keinen
Wechsel aus. Zurückgezogen (`has_target = 0`) wird es nur, wenn

- `picked_id` für diese ID eintrifft (gleich welches Ergebnis),
- die ID aus `tracks` verschwindet,
- der Track auf Status 3 zurückfällt (neu einschwingend),
- keine Bandschätzung mehr vorliegt,
- `tracks` länger als 1,0 s stillsteht.

Ohne `robot_state` wird nichts Neues gewählt (die Erreichbarkeit ist dann unbekannt),
ein bestehendes Ziel bleibt.

**Erreichbarkeit ist kein Rückzugsgrund** — sonst bräche ein fast gelungener Griff ab,
sobald der Klotz die Zonengrenze überschreitet. Ob ein Griff noch beginnt, entscheidet
der Follower an der Greifebene. So braucht der `priority_handler` keine Rückmeldung
des Follower-Zustands.

### 7.5 `not_pickable` und Predicates (H5)

`not_pickable` (S5) ist eine aktuelle Liste: alle Tracks hinter der Greifebene außer
dem Ziel und bereits versuchten IDs. Das Festhalten übernimmt der `data_tracker`.
`zone_empty` ist räumlich (kein Klotz im Rechteck, gleich welcher Status).

### 7.6 Prozesszeiten (L24, L29, D22)

| Zeit | Parameter | gemessen |
|---|---|---|
| Einschwingen des Followers bis zur Freigabe | `t_settle_s` 0,4 s | 0,23–0,33 s |
| Absenken | `t_descend_s` 0,7 s | 0,78–0,93 s bei 0,25 m/s; bei 0,35 m/s gerechnet 0,11–0,15 m / 0,35 m/s + Einschwingen |
| Greifen (Schließen bis `motion_done`) | `t_grasp_s` 0,8 s | 0,63–0,83 s |
| Mitfahren beim Heben | `t_lift_s` 0,5 s | 0,5–0,7 s |
| Aufschlag | `grasp_time_margin` 1,2 | – |

`t_descend_s` hängt an der Folgehöhe und der Sinkgeschwindigkeit des Followers:
`t_descend_s ≈ (observe_z − Greifhöhe) / descend_speed_mps` + Einschwingen (J2).

---

## 8. Greifen — `object_follower` (Thema 6, F3–F6, G1–G8, J1, J2, J4–J7, Z12, L23, L25, L26, L28)

### 8.1 Zustände

| Code | Zustand | Verhalten | weiter bei |
|---|---|---|---|
| 0 | `WARTEN` | Beobachtungspose, stehendes Ziel | Ziel in S4 → `ANFAHREN` |
| 1 | `ANFAHREN` | vorhergesagter Klotz + Vorhalt, längs auf `zone_upstream` begrenzt | Klotz erreicht `zone_upstream` → `FOLGEN` |
| 2 | `FOLGEN` | über dem Klotz mit Vorhalt, auf Folgehöhe `observe_z`; Greiforientierung einstellen | 10 Takte in allen vier Toleranzen und Klotz vor der Greifebene → `ABSENKEN` |
| 3 | `ABSENKEN` | wie `FOLGEN`, dazu Sinken mit `descend_speed_mps`; Gier eingefroren | Greifhöhe erreicht → `GREIFEN` |
| 4 | `GREIFEN` | Mitfahren auf Greifhöhe, Greifer schließt | `has_object` → `HEBEN` |
| 5 | `HEBEN` | mit der letzten Bandgeschwindigkeit weiter bis `lift_clearance_m` (0,1 m) über der Greifhöhe, dann senkrecht auf die Freihöhe | Freihöhe → `ABLEGEN` |
| 6 | `ABLEGEN` | Ablagepose auf Freihöhe anfahren | angekommen → `LOESEN` |
| 7 | `LOESEN` | Greifer öffnen, `picked_id` mit `outcome = 0` | Öffnungsbewegung gemeldet → `WARTEN` |
| 8 | `ABBRUCH` | siehe §8.5; auch der **Start** | → `WARTEN` |

**Ergebnisse** (`outcome` in S7): 0 abgelegt · 1 Fehlgriff · 2 Klotz verloren ·
3 Greifebene überschritten, bevor abgesenkt wurde · 4 vorher abgebrochen (Ziel
zurückgezogen oder gewechselt, Zeitüberschreitung, Eingang steht, Datenfehler,
Arbeitsraum; Grund im Log). Jeder Versuch endet mit genau einer Meldung; der Follower
nimmt eine abgeschlossene ID nicht wieder an (G2).

Nach dem Ablegen fährt der Follower ein bereits gewähltes Ziel direkt aus der Ablage
an, nicht über die Wartepose.

### 8.2 Anfahren und Folgen (Thema 6, N3, G1, G3, G6)

- **Abfangen = Begrenzen:** In `ANFAHREN` wird die Zielpose nur **längs** auf
  `zone_upstream` (S4 Feld 12) begrenzt, quer nicht. Liegt der Klotz noch vor der
  Zone, wartet der Roboter am Zonenanfang auf seiner Querposition; erreicht der Klotz
  die Zone, beginnt das Folgen nahtlos. Kein zweiter Codepfad.
- Stromabwärts wird nicht begrenzt: Ein laufender Griff darf dem Klotz über das
  Zonenende hinaus folgen, begrenzt nur durch den Arbeitsraum.
- Gefolgt wird auf **Folgehöhe** `observe_z` (0,45 m); der Versatz Flansch → Griffpunkt
  wirkt erst beim Absenken.
- Die Zeitgrenze von `ANFAHREN` ist relativ: erwartete Ankunft des Klotzes an der Zone
  plus `timeout_approach_s` (2 s). `FOLGEN` hat `timeout_track_s` (3 s) bis zum
  Absenken.
- Die Vorhersage stützt sich auf die letzte Messung und die Bandgeschwindigkeit aus S4;
  ihr Horizont ist auf `max_extrapolation_s` = **1,0 s** begrenzt (§8.7).

### 8.3 Absenken, Greifen, Heben (Z11, J2, J5, J6)

- **Tor an der Greifebene:** `FOLGEN` → `ABSENKEN` nur, solange der Klotz vor der
  Greifebene liegt. Überschreitet er sie vorher — auch nach einem Rücksprung aus
  `ABSENKEN` —, ist der Griff nicht mehr zu schaffen: `ABBRUCH` mit `outcome = 3`.
  **Hat das Absenken begonnen, gilt die Ebene nicht mehr.**
- `ABSENKEN`: Die Zielhöhe sinkt mit `descend_speed_mps` (0,35 m/s) von der Folgehöhe
  zur Greifhöhe, seitlich wird mit Vorhalt weiter gefolgt. Wächst die Abweichung über
  das **Doppelte** der Toleranz, geht es zurück nach `FOLGEN` **auf Folgehöhe** (dort
  beginnt der nächste Versuch unter denselben Bedingungen). Zeitgrenze: Absenkdauer +
  `timeout_grasp_s`.
- `GREIFEN`: `has_object` wird **vor** den Abbruchgründen geprüft — meldet der Greifer
  den Klotz im selben Takt, in dem sich sonst etwas ändert, gilt der Griff.
  **Fehlgriff** (`outcome = 1`): Die Bewegung hat begonnen und ist fertig, ohne
  `has_object`, oder `timeout_grasp_s` (2 s) läuft ab.
- `HEBEN`: Erst mitfahren, bis der Klotz das Band verlassen hat — hielte der Roboter an,
  während der Klotz noch aufliegt, zerrte das Band daran. Dafür braucht es keine
  Blockvorhersage mehr, der Klotz ist im Greifer. **Fällt `has_object` länger als
  0,1 s weg**, gilt der Klotz als verloren (`outcome = 2`); kürzeres Flackern wird
  überbrückt.
- `LOESEN` endet über den Abschluss der Öffnungsbewegung, nicht über `has_object = 0`:
  Der Robotiq-Status meldet ein Objekt auch, wenn die Backen **beim Öffnen** anstoßen.
  Nach `timeout_release_s` (2 s) endet es mit Warnung; der Klotz ist über der Kiste
  gelöst (`outcome = 0`).

### 8.4 Greif-Freigabe (F1 in Nachtrag 2, J4, D3, B18)

Vier Toleranzen, gemessen **entlang und quer zum Band** wie `err_laengs`/`err_quer`,
über `stable_cycles` = 10 Takte gehalten:

| längs | quer | Höhe | Gier |
|---|---|---|---|
| 5 mm | 5 mm | 10 mm | 0,05 rad (modulo 180°) |

Die Gier gehört dazu, weil der Roboter positionsmäßig sauber über dem Klotz stehen
kann, während das Handgelenk noch dreht. Bemessen nach dem ungünstigsten Klotz: quer
bleiben beim 25-mm-Klotz mit 15 mm breiter Auflage nur ±5 mm.

### 8.5 Abbruch und Start (Thema 7, Z12, F4, J6, J7)

**Bevor der Klotz gehalten wird** führen zum Abbruch: Ziel zurückgezogen oder
gewechselt, Zeitüberschreitung, ein stehender Zielsatz (`target_timeout_s` 1,5 s),
das Sicherheitsgate (§9.2). Der Abbruchpfad:
Hat der Follower den Greifer geschlossen, hält er die Pose, öffnet und wartet auf die
Öffnungsbewegung — sonst zöge er halb geschlossene Backen am Klotz nach oben. Dann
**senkrecht hoch** auf `max(z, transfer_height_m)`, Orientierung beibehalten, dann
`WARTEN` (Beobachtungspose).

**Mit Klotz im Greifer** (ab `has_object`) sind Zielrückzug, verschwundene ID und
Sprünge der Regelabweichung **keine Abbruchgründe** mehr: Der Klotz soll vom Band
verschwinden, der Follower besitzt ihn jetzt. Sonst hätte der Follower jeden
gelungenen Griff wieder fallen lassen — die Basiskamera verliert den gehobenen Klotz,
der `priority_handler` zieht das Ziel zurück. Abbruchgründe bleiben
Zeitüberschreitung, Sicherheitsgate und der Wegfall von `has_object`. **Bricht der Follower mit Klotz ab, bleibt der Greifer zu:** senkrecht
hoch, zur Ablagepose, dort öffnen (`outcome = 0`, Grund im Log). Scheitert auch das
Ablegen erneut, hält er den Klotz und meldet „Eingriff nötig“, statt ihn irgendwo
fallen zu lassen.

**Start = Abbruchpfad:** Beim Start steht der Roboter irgendwo; der Attractor führe
geradlinig zur Beobachtungspose, womöglich durch das Band. Der Follower **startet im
Zustand `ABBRUCH`**: erst senkrecht hoch auf die Freihöhe, dann zur
Beobachtungspose. Der Raum über dem Arbeitsbereich ist frei.

Ohne frischen `robot_state` (älter als `robot_state_max_age_s`, 0,2 s) gibt der
Follower keine neue Zielpose aus und bleibt im Zustand; der Attractor hält die
letzte — der sichere Fall.

### 8.6 Orientierung (Thema 6, G6, G9, L25, L26, L29)

- Grundstellung (**Modus 1**): Backen quer zur Laufrichtung, Gier aus der
  Bandrichtung, `observe_yaw_deg` 90°. `gripper_yaw_offset_deg` = 0 — die
  Grundstellung greift richtig (D23).
- **Modus 2** (`use_block_orientation`, Standard an): Der Greifer dreht in den Winkel
  des Klotzes, aber **höchstens ±45° aus der Grundstellung**. Ein Rechteck lässt sich
  über jede Seite greifen; zusammen mit der Halbdrehsymmetrie des Greifers zählt der
  Klotzwinkel **modulo 90°** gegen die Bandrichtung (60° → −30°). Bis
  `max_yaw_deviation_deg` (50°) bleibt die zuletzt gewählte Seite (Hysterese), damit
  ein diagonal liegender Klotz den Greifer bei Rauschen nicht kippen lässt.
- **Güte:** Unter `orientation_quality_min` (0,4) greift er in Grundstellung. Mit 0,7
  wurden kleine, hochkant stehende Klötze nicht gedreht, obwohl der gemittelte Winkel
  auf wenige Grad stimmte (L29).
- Von zwei gleichwertigen Stellungen wird die dem zuletzt kommandierten Winkel nähere
  gewählt — keine halbe Umdrehung des Handgelenks.
- Die Gier wird beim Übergang nach `ABSENKEN` **eingefroren**.
- Am Aufbau bestätigt: Winkel der langen Kante in `world` statisch auf ±1,9°, im Lauf
  kein 90°-Wechsel, Vorzeichen + = gegen den Uhrzeigersinn von oben (L26).

### 8.7 Weitere Werte des Followers

| Parameter | Wert | Grund |
|---|---|---|
| Beobachtungspose `observe_*` | x −0,816 (Bandmitte), y +0,35 (Anfang der Greifzone), z 0,45, Gier 90° | Wartepose am Zonenanfang; mit einer Wartepose am Zonenende war kein Ziel mehr erreichbar (L14). Tief genug, dass die Greifebene genug Band lässt. |
| `max_extrapolation_s` | 1,0 s | Das Ziel ist bis 0,93 s alt; mit 0,6 blieb die Vorhersage zu 24 % der Zeit stehen, `err_laengs` lief auf +17 mm, 3 von 7 Griffen gelangen (mit 1,0: 9 von 9, L23). Bleiben die Daten aus, bricht `target_timeout_s` (1,5 s) ab. |
| `max_target_jump_m` | 0,05 m | Sprungprüfung des Gates (D15) |
| **Greifhöhe** | Band + `max(h/2, min_grip_height_m)` + 0,235 m | mittig auf der Seitenfläche, Reserve nach oben und unten; Bezug ist die gemessene Bandoberfläche, nicht die gemessene Oberkante |
| `min_grip_height_m` | 0,011 m | geschlossene Backenspitze 1 mm über dem Band; greift nur bei Klötzen bis 22 mm; ein flacher 24-mm-Klotz wird auf halber Höhe gefasst (L28) |
| `transfer_height_m` (Freihöhe) | 0,49 m | Band 53,6 + stehender 100-mm-Klotz + untere Hälfte des gehaltenen Klotzes 50 + Griffpunkt 235 + Luft 50 mm (J1) — für Transfer und Abbruch |
| Ablagepose `place_*` | x −0,31649, y +0,47621, z 0,41971, Gier 94,2° | §4.3 |
| Zeitgrenzen | Ablegen 8 s, Lösen 2 s | Fahrt vom Band zur Kiste rund 0,8 m |

**Greifhöhe min und `ws_z_min` nur gemeinsam ändern** — sonst liegt die Greifhöhe
flacher Klötze unter der Arbeitsraumgrenze und das Gate bricht jeden flachen Griff ab.

---

## 9. Sicherheit (Thema 7, F6, G5, L14, L15, L20)

### 9.1 Wo die Sicherheitsarbeit hingehört

Die Kette gibt eine **Pose** aus, keine Geschwindigkeit. Bleibt das Kommando aus,
konvergiert der Attractor auf die letzte Pose und hält — unkritisch. Gefährlich bleibt
nur ein **falsches** Kommando. Die Sicherheitsarbeit gehört deshalb in die Prüfung der
Zielpose vor dem Veröffentlichen.

Hauptrisiko ist eine veraltete Vorhersage: `p₀ + v · (t_jetzt − t₀)` wächst
unbegrenzt, wenn `base_cam` oder `vectoring` ausfallen. Dagegen stehen die
Zeitgrenzen der Eingänge und der Deckel der Vorhersage.

### 9.2 Das Sicherheitsgate

Eine Funktion im Follower, **letzter Schritt vor jeder Ausgabe**:

| # | Prüfung | Verstoß |
|---|---|---|
| 1 | alle Werte endlich (kein NaN/Inf) | Abbruch |
| 2 | Zielsatz frisch (`target_timeout_s`) · Flanschpose frisch (`robot_state_max_age_s`) | Abbruch · keine neue Zielpose, der Attractor hält |
| 3 | Vorhersagehorizont in `[0, max_extrapolation_s]` | begrenzen, einmal je Versuch ins Log |
| 4 | Sprung der Zielpose ≤ `max_target_jump_m` | **Abbruch**, nicht begrenzen — ein großer Sprung heißt falsche Daten |
| 5 | Zielpose im Arbeitsraum | begrenzen; in `ANFAHREN`/`FOLGEN` zusätzlich Abbruch (G5) |

Die Sprungprüfung vergleicht **Rohziel mit Rohziel**, das Begrenzen kommt danach
(F6: Gegen die begrenzte Pose verglichen, galt bei einem Start außerhalb des
Arbeitsraums schon der zweite Takt als Sprung). Sie wird bei jedem Zustandswechsel
zurückgesetzt, weil dort legitime Sprünge auftreten. Wird die Beobachtungspose zur
Laufzeit weit verstellt, nimmt der Follower bewusst den Abbruchpfad.

Keine eigene Komponente: Sie bräuchte dieselben Daten und brächte einen Signalsprung
Verzögerung.

### 9.3 Arbeitsraum (B10, L14, L15)

Von Hand abgefahren, sicher und ohne Singularität, außerhalb des Bildes der
Basiskamera: auf dem Band x −1,000 … −0,530, y −0,320 … +0,445. Für die Ablage neben
dem Band erweitert:

| `ws_x_min` / `ws_x_max` | `ws_y_min` / `ws_y_max` | `ws_z_min` / `ws_z_max` |
|---|---|---|
| −1,0 / −0,30 m | −0,32 / +0,48 m | 0,2986 / 0,60 m |

- `z_max` 0,60: darüber Singularität im hinteren Bereich.
- `z_min` 0,2986: geschlossene Backenspitze auf Bandhöhe (53,6 + 245 mm), Marge 0; beim
  Greifen 1 mm darüber (L28). Auf der hohen Bandseite (x ≈ −0,93) liegt das Band rund
  1 mm höher.
- Mit dem Attractor genügt das Begrenzen der Zielpose: Ein System erster Ordnung
  schwingt nicht über, das kleine Überschwingen der Antriebe deckt die Marge.
- Quelle ist `Safety/workspace_bounds.json`; dieselben Werte sind Standard in
  `FollowerParams` und der Komponentenbeschreibung, ein Test prüft die Gleichheit.
  Arbeitsraum und Beobachtungspose sind **Pflichtparameter**: leer → `configure`
  scheitert mit einer Liste der fehlenden. Die Parameter werden als Satz geprüft
  (Beobachtungs-, Ablagepose und Freihöhe im Arbeitsraum).
- Wird Band oder Roboter verschoben, gilt der Standardwert unbemerkt weiter — dann neu
  abfahren und Datei, `FollowerParams` und Komponentenbeschreibung gemeinsam ändern.

### 9.4 Singularitäten (B11)

Im Arbeitsraum treten keine auf (Dauerlauf, L26). Die Ablagepose liegt weit von
Handgelenk-, Ellbogen- und Schultersingularität (`wrist_2` −89,96°, Ellbogen 124°,
571 mm von der Basisachse).

### 9.5 Schutzstopp beim Anfahren (L20)

Am Bandrand löste das Anfahren einen Schutzstopp der UR-Steuerung aus (C157A2 „Roboter
konnte dem Pfad nicht folgen“, C162A0 Nutzlast). Ursachen zusammen: Nutzlast 2,8 kg
eingetragen statt gemessener 1,3 kg, und ein Sprung des Ziels von der Beobachtungspose
zum Anfahrpunkt ohne Beschleunigungsgrenze am IK-Controller. Abhilfe: Nutzlast
1,3 kg mit gemessenem Schwerpunkt, `command_rate_limit` (heute 3,0). Bei einem Stopp
zuerst die Geschwindigkeit zurücknehmen.

### 9.6 Vor dem Start

- Der Attractor fährt beim Start auf die zuletzt gespeicherte Zielpose — **den Arm
  vorher freifahren**, besonders nach einem Absturz.
- Abnehmer von `target_pose` ist der Attractor, nicht der IK-Controller (am Aufbau vor
  dem ersten Lauf durch Mitlesen gefunden).
- Abschalten mit Klotz im Greifer: Der Klotz bleibt im Greifer; beim nächsten Start
  öffnet `robotiq_gripper` im Bring-up, der Klotz fällt dort, wo der Arm steht.

---

## 10. Komponentenschnitt und Datenverträge (Thema 4, Thema 5, H4, T2)

### 10.1 Eigentum

Jede Information hat genau einen Eigentümer; alle anderen bekommen sie von ihm.

| Komponente | besitzt | besitzt nicht |
|---|---|---|
| `base_cam` | Detektionen, IDs, Zeitstempel | Bahnlogik |
| `vectoring` | Geschwindigkeitsschätzung (je Klotz und Pool), geglättete Zustände, Einschwing- und Vorhersagestatus | Auswahl |
| `priority_handler` | Greifzone, Greifebene, Erreichbarkeit, Ziel-Lock | Objektverwaltung |
| `object_follower` | Zustandsautomat, Regelung, Arbeitsraum, Sicherheitsgate | Zielauswahl |
| `robotiq_gripper` | Greiferhardware | Ablauflogik |
| `data_tracker` | Gesamtliste für die Anzeige | nichts Regelrelevantes |
| `interface_streamer` | Darstellung | nichts |

Greifzone (Aufgabenentscheidung, `priority_handler`) und Arbeitsraum
(Kollisionsschutz, Follower) sind verschiedene Größen. Von der Zone bekommt der
Follower genau zwei Zahlen über S4: `zone_upstream` und die Greifebene (N3).

### 10.2 Kein Zyklus

`data_tracker` ist ein **Blatt**: Er wird nur gelesen und führt nichts in den
Regelpfad zurück. Der `priority_handler` braucht ihn nicht — „schon gepickt“ hört er
auf `picked_id`, „außerhalb“ erzeugt er selbst, „erreichbar“ rechnet er. Die einzige
Rückkante im Graphen ist `picked_id` vom Follower an den `priority_handler`, ein
diskretes Ereignis je Versuch.

### 10.3 Grundregeln der Verträge

1. Kopf `[t, n, …]`, Länge `kopf + n · stride` prüfbar.
2. **Nie leer** — auch `[t, 0]` wird gesendet: „sieht nichts“ ist von „sendet nicht
   mehr“ unterscheidbar.
3. `t` nur im Kopf.
4. SI-Einheiten, `world`.
5. **Statusfelder statt stiller Annahmen** (`has_target`, `status`, `n_pool`,
   `present`, `ori_quality`).
6. Alle Indizes in `contracts.py`; keine Komponente indiziert von Hand in ein fremdes
   Array.
7. **`picked_id` mit laufender Nummer** `[seq, id, outcome]` statt eines zeitlich
   begrenzten „True“: Jeder Verbraucher reagiert genau einmal auf jede neue `seq`, nie
   auf 0 (0 = noch kein Versuch seit der Aktivierung). Die Regel steckt einmal in
   `contracts.AttemptWatcher` (T2).
8. **Die Längskoordinate** der S4-Felder 12 und 15 rechnen Erzeuger und Verbraucher mit
   derselben Funktion `contracts.along_belt`: `s = (x·vx + y·vy) / |v|` (H4).

Die Greiferkomponente meldet **zwei** Bool-Signale, `motion_done` und `has_object`: Dass
der Greifer geschlossen hat, sagt nichts darüber, ob er etwas gefasst hat.

---

## 11. Anzeige (T1, V1–V3, I2)

- `data_tracker` hält je Klotz die Vermerke `picked`, `out_of_bounds` und
  **`present`** (S10). Ein gegriffener Klotz verschwindet beim Heben aus dem Bild,
  `picked_id` kommt erst nach dem Ablegen; der Eintrag muss den Track deshalb
  überdauern, und `present = 0` sagt der Anzeige, dass die Werte eingefroren sind.
- **Verfall:** `expiry_after_done_s` (10 s) nach dem Setzen von `picked` oder
  `out_of_bounds`, ohne solchen Vermerk ab dem Verschwinden aus `tracks`. Ein späteres
  Verschwinden startet die Frist nicht neu (ein gegriffener Klotz läuft als Status 4
  weiter). Gezählt wird in S3-Zeit: Steht der Eingang, friert die Anzeige ein.
- `interface_streamer` setzt das Debug-Bild der Basiskamera, Zustand, Ziel,
  Regelabweichungen, Bandgeschwindigkeit und die Klotzliste zu einem Bild zusammen.
  Der Text ist **ASCII** (die Hershey-Schriften von OpenCV kennen keine Umlaute). Ob
  ein Bild noch kommt, zeigt seine **Ankunftszeit** (die Debug-Bilder tragen keinen
  Stempel); nach 2 s wird der Bereich grau. Daten älter als 1 s werden als
  „(veraltet)“ **markiert, nicht versteckt**.
- Die Anzeige ist die einzige Komponente, ohne die das System vollständig arbeitet.
  Jeder Fehler darin wird geloggt und übersprungen.

---

## 12. Roboterkamera — nicht eingebunden (Z10, L11, L13, L22)

Ursprünglich sollte die RealSense D435i am Flansch die Position vor dem Zugriff
korrigieren. **Gegriffen wird mit der Basiskamera allein**; die Komponenten der
Roboterkamera sind nicht mehr im Paket.

**Begründung:**
- 10 von 10 greifbaren Klötzen wurden ohne Roboterkamera abgelegt, `err_laengs` rund
  1 mm (L19–L22); die Gewichte der Korrektur standen immer auf 0.
- Die Erkennung blieb unzuverlässig: Das Band ist stellenweise entsättigt (Farbmaske
  versagt), Glanzstreifen und Tiefenschatten verschmolzen mit dem Klotz, die Mitte
  lag bis 32 mm daneben. Nur eine positive Klotzfarbe hätte offline getragen — für
  Weiß nicht.
- Treiber und Komponente kosten Rechenzeit genau in den Prozessen an der Grenze (§3).

**Ergebnisse, die bleiben:** Die Hand-Auge-Kalibrierung der Vorgängergruppe war im
Versatz falsch (Kamera 7 cm unter, nicht 6 cm über dem Flansch). Neu eingemessen aus
fünf Ansichten eines angetasteten Klotzes: Flansch → Kamera x 78,3 · y −32,6 ·
z 72,0 mm, Roll 4,26° · Pitch 0,08° · Gier 90,95° (R = Rz·Ry·Rx), Rest ≤ 4,4 mm,
Vorhersage einer fünften Ansicht 1,7 mm.

Nachweis beim Ausbau aus dem Follower: 12 Szenarien im geschlossenen Regelkreis,
27 200 Takte vor und nach dem Umbau, 0 Abweichungen. Der Vertrag S2 steht als
stillgelegtes Signal weiter in `contracts.py`.

---

## 13. Grenzen und mögliche Verbesserungen

- **Takt:** Mehr als etwa ein Klotz je 7 s ginge nur über einen kürzeren Rückweg,
  also eine Ablage näher am Band. Klötze kurz hinter einem gerade gegriffenen laufen
  durch (Taktgrenze aus Heben, Ablegen, Lösen und Rückweg, keine Fehlentscheidung).
- **Genauigkeit der Basiskamera:** In den unteren Ecken des Bildausschnitts bleiben
  rund 11 mm. Genauer als L6 wird `base_cam` nur mit Eingriffen in die Erkennung:
  Tiefenkorrektur (M1s), Entzerrung, Bandfläche statt ebener Bandebene.
- **Kalibrierung Stufe 2** (ohne Bewegung, aus Referenzmarken am Bandgestell) ist
  implementiert, aber nicht nutzbar: Die Marken sind nicht angebracht, weil die
  Hardware nicht verändert werden darf. Nachkalibrieren heißt Stufe 1 laufen lassen.
  In der höchsten Greiferlage (0,44 m unter der Kamera) lieferte die L515 über dem
  Board keine Tiefe.
- **Rechenleistung:** Der Rechner ist mit Rate 15, Oberfläche und RViz voll ausgelastet
  (§3.5).
- **Flache Klötze auf der hohen Bandseite:** Dort bleibt an der Greifhöhen-Untergrenze
  kaum Abstand zum Band (§9.3).
- Die UR-Steuerung meldet, dass ihre Werkskalibrierung nicht zur Kinematik passt;
  Flanschposen aus AICA können um wenige mm von der Steuerung abweichen (0,4 mm
  gemessen).

---

## Register der Kennungen

Code, Komponentenbeschreibungen und Messdaten verweisen mit diesen Kennungen auf
Entscheidungen und Messpunkte. „Thema n“ und „Nachtrag n / X“ sind die Gliederung des
früheren Protokolls; B, C, D und E sind Messpunkte der Arbeitsliste aus der
Inbetriebnahme.

| Kennung | Inhalt | hier |
|---|---|---|
| Thema 1 | Bewegungsarchitektur, Vorhalt | §2 |
| Thema 2 | Zeit und Latenz | §3 |
| Thema 3 | Bezugssysteme, Einheiten, Flansch | §4 |
| Thema 4 | Komponentenschnitt, Eigentum | §10 |
| Thema 5 | Datenverträge | §10.3, `datenvertraege.md` |
| Thema 6 | Zustandsautomat des Followers | §8 |
| Thema 7 | Sicherheit | §9 |
| P1, P4 | Erreichbarkeit, Ziel-Lock | §7.3, §7.4 |
| F1–F3 (Nachtrag 2) | Gier in der Freigabe, Rücksprung auf Folgehöhe | §8.4, §8.3 |
| N1 | Höhenfehler der Roboterkamera | §12 |
| N3, N4 | `zone_upstream` in S4; Roboterzustand im `priority_handler` | §8.2, §7.3 |
| N5 | Debug-Bild mit Kamerarate statt Komponentenrate | `datenvertraege.md`, §3 |
| M2, M3 | Zeitdomäne der L515 | §3.2 |
| M5 | Streuung der Geometrie | §5.1 |
| M8, M9, M10, M11 | Flansch, Bandhöhe, Bandrichtung, Ablagepose | §4 |
| Z1, Z8 | Projektziele, Bahnplanung | §1 |
| Z2, Z9 | Geschwindigkeitsschätzung, Pool | §6.1 |
| Z3 | Einschwingen | §6.2 |
| Z4 | Tracker misst | §5.1 |
| Z5 | Klötze fallen nur am Bandende | §7.5, `datenvertraege.md` S10 |
| Z6 | Vorhalt als Zeit | §2.2 |
| Z7 | Flansch → Griffpunkt 0,235 m | §4.2 |
| Z10 | Greifen ohne Roboterkamera | §12 |
| Z11, E10 | Greifebene; Ziel stromaufwärts | §7.2, §8.3 |
| Z12 | nichts fallen lassen | §8.5 |
| Z13 | Orientierungsgüte beim Verbraucher | §6.3, §8.6 |
| H1–H5 | Anfahrweg, Diagonale, Spur, `along_belt`, Kleinigkeiten | §7 |
| T1, T2 | `present` und Verfall; `AttemptWatcher` | §11, §10.3 |
| F1 (Nachtrag 8), K5 | 180° zwischen `base` und `world` | §4.1, §5.3 |
| F3–F6 (Nachtrag 8) | Pflichtparameter, Abbruchpfad, Zustandscodes, Sprungprüfung | §9.3, §8.5, §8.1, §9.2 |
| G1–G8 | Folgehöhe, `outcome = 4`, Anfahren, Zeitgrenzen, Gier, Vorhalt einmessen, Horizont | §8.1–8.2, §8.6, §2.2, §3.1 |
| G9, D23 | Montagewinkel der Backen (0) | §8.6 |
| J1, D12 | Freihöhe 0,49 m | §8.7 |
| J2, D22 | Absenkzeit und Prozesszeiten | §7.6 |
| J4–J7, D3, B18 | Freigabe, Absenken bis Lösen, Abbruch nach eigenem Schließen | §8.3–8.5 |
| V1–V3, I2 | Anzeige | §11 |
| K1, K2 | Kameraknoten, Raten | §3 |
| K6 | Gier der Altkalibrierung | §5.3 |
| L1, L2, L3 | Audit, Datenrate, Hardware-Takt | §3 |
| L4 | Greifer außerhalb des Bildes | §5.2 |
| L6, B23, C3 | Handkalibrierung der Basiskamera, Parallaxe, `top_depth_bias_mm` | §5.1, §5.3 |
| L7 | Bandlage, Bandgeschwindigkeit per Stoppuhr | §4.3 |
| L9, L10 | Pool nur bewegt; Weiterführen hinter dem Bild | §6.1, §6.4 |
| L11–L13, L22 | Roboterkamera | §12 |
| L14, L15, B10 | Arbeitsraum, Beobachtungspose, Standardwerte | §9.3, §8.7 |
| L16 | Kameras 15 Bilder/s | §3.5 |
| L17 | Prüfung eigener Parameter im Follower; Zeitstempel von `fake_objects.py` in ROS-Zeit | Code |
| L18, L19, B4 | erster Lauf, Vorhalt 0,24 s, erste Griffe | §2.2 |
| L20 | Schutzstopp, Nutzlast | §9.5 |
| L21, B19 | Greifzone = Arbeitsraum auf dem Band, Bildausschnitt | §7.1, §5.1 |
| L23, D14 | Deckel der Vorhersage 1,0 s, `t_settle_s` | §8.7, §7.3 |
| L24 | Tempo, Sicherheitsfaktor, flache Klötze | §7.3, §7.6, §5.1 |
| L25, L26 | ±45° aus der Grundstellung; finaler Build | §8.6 |
| L27 | Kalibrierung der Basiskamera | §5.4, §5.5 |
| L28 | flache Klötze tiefer erkannt und gegriffen | §5.1, §8.7 |
| L29 | 0,85 m/s, Absenken 0,35 m/s, Rate 15, Güte 0,4 | §2.3, §3.4, §8.6 |
| A1 | Standardwerte des Attractors | §2.3 |
| B1 | Bandgeschwindigkeit per Stoppuhr (Gegenprobe) | §1, §6.5 |
| B8 | Beobachtungshöhe | §8.7 |
| B9 | Ablagepose | §4.3 |
| B11 | Singularitäten | §9.4 |
| B15, B16, B17 | Auflage 20 mm, Öffnung 127 mm, Bandhöhe 53,6 mm | §4.3 |
| D5 | Zeitgrenzen der Zustände | §8.2, §8.3, §8.7 |
| D7 | `latency_compensation_s` = 0 | §2.2 |
| D11 | Mindestgüte des Winkels | §8.6 |
| D14, D15 | Deckel der Vorhersage, Sprunggrenze | §8.7 |
| D20, D21 | Einschwingen, Ausreißer | §6.2 |
