# Eye-in-Hand-Kalibrierung — Ansatz 1

> **Kontext:** Dies ist Ansatz 1 der extrinsischen Kalibrierung, entwickelt im Branch
> `Calibration-Working`. Im laufenden Greifbetrieb ist **Ansatz 2** aktiv
> (`base_cam_calibration.py`), bei dem das Board vom Roboter gegriffen und unter die
> fest montierte Basiskamera gefahren wird.

---

## Worum geht es?

Damit der Roboter weiß, wo das Förderband und alle Kameras im Raum liegen, müssen
vier Koordinatensysteme miteinander verbunden werden:

- **`robot_base` / `world`** — Ursprung des Roboters
- **`conveyor_frame`** — Förderband-Koordinatensystem
- **`robot_cam_frame`** — Kamera am Roboterflansch
- **`base_cam_frame`** — fest montierte Basiskamera (L515)

Das ChArUco-Board ist der gemeinsame Messpunkt: Sein Ursprung ist von beiden Kameras
aus messbar und dient als Brücke zwischen allen Frames.

Ergebnis der Kalibrierung sind drei Transformationsmatrizen:

- `T_ee_cam` — wo sitzt die Kamera relativ zum Flansch?
- `T_robot_conveyor` — wo liegt das Förderband relativ zur Roboterbasis?
- `T_robot_base_static_cam` — wo sitzt die Basiskamera relativ zur Roboterbasis?

---

## Abgrenzung der beiden Ansätze

**Ansatz 1 — Eye-in-Hand (dieser Ordner)**
- Board liegt fest auf dem Förderband
- Kamera am Roboterflansch wird bewegt
- Kalibriert: `T_ee_cam` + `T_robot_conveyor` + `T_robot_base_static_cam`
- Ergebnisdatei: `calibration.yaml` / `.json`
- AICA-Komponenten: `auto_calibration`, `board_detection`, `calibration_test_drive`

**Ansatz 2 — Basiskamera (`base_cam_calibration.py`, aktiv im Greifbetrieb)**
- Board am Roboterflansch gegriffen, unter die Basiskamera gefahren
- Kalibriert: `T_robot_base_cam`
- Ergebnisdatei: `Extrinsics/base_cam_extrinsics.json`
- AICA-Komponente: `base_cam_calibration`

---

## Aufbau

**ChArUco-Board**

- Liegt am Rand des Förderbands, sodass es von der Roboterkamera und der Basiskamera
  gut sichtbar ist
- Der **Ursprung des Boards** ist die äußerste linke obere Ecke von Marker ID 0 —
  dieser Punkt ist der Messpunkt, über den alle Frames verknüpft werden
- Der **Ursprung des `conveyor_frame`** ist vorab definiert; `conveyor_offset` in den
  Parametern gibt an, an welcher Stelle im Förderband-Frame dieser Board-Ursprung liegt

**Warum ChArUco und nicht AprilGrid?**

Das vorhandene AprilGrid-Board ist für die Basiskamera bei der entstehenden Distanz
zu klein — Auflösung und Erkennungsrate sind unzureichend. ChArUco-Boards sind physisch
größer und können von beiden Kameras zuverlässig erkannt werden.

---

## AICA-Projektstruktur

In AICA gibt es eine **eigene Anwendung „Calibration"**, die von der Hauptanwendung
(Greifbetrieb) vollständig getrennt ist und nur die Kalibrierkomponenten enthält.

**Ablauf:**

1. Kalibrieranwendung starten
2. Kalibrierung ausführen → erzeugt `calibration.yaml`
3. Kalibrieranwendung beenden
4. Hauptanwendung starten — liest die gespeicherte Datei ein

Die Kalibrierung wird **über eine Datei übergeben**, nicht über laufende Signale.
`CalibrationTestDrive` liest dieselbe Datei zur Validierung.

---

## Komponenten

### `BoardDetection` — ChArUco-Board erkennen

Erkennt das ChArUco-Board im Kamerabild. Läuft je einmal für die Roboterkamera und
für die Basiskamera, da jede Instanz eigene Kameraeingaben hat.

Ausgaben:

- **`board_pose`** — Pose des Board-Ursprungs in der Kamera: `[tx, ty, tz, rx, ry, rz]`
- **`board_geometry`** — Abmessungen `[rows, cols, checker_size_mm]`, damit andere
  Komponenten die Boardgröße kennen, ohne Parameter zu duplizieren
- **`board_depth`** — Tiefe des Board-Bereichs aus dem Tiefenbild: `[mean_mm, median_mm, gültige_Pixel]`
- **`board_corners`** — Eckpunkte erkannter Felder (für Debug-Bild)
- **`debug_image`** — Kamerabild mit eingezeichneten Markern und Achsen

Prädikate: `has_board`, `has_pose`, `is_receiving_frames`

Die Board-Parameter (`aruco_dictionary`, `board_rows`, `board_cols`, `checker_size_mm`,
`marker_size_mm`) werden **nur hier** gesetzt — alle anderen Komponenten lesen sie
über das Signal `board_geometry`.

---

### `AutoCalibration` — Automatische Kalibriersequenz

**Vorbereitung (manuell):**

- Roboter mit der Handkamera grob über das Board fahren, sodass es im Bild sichtbar ist
- In AICA Studio: Parameter prüfen und ggf. anpassen
  (`num_waypoints`, `max_orbit_radius_mm`, `conveyor_offset_x/y_mm`)

**Start:** Service `start_calibration` (Button in AICA Studio).

**Automatischer Ablauf:**

1. Fährt die Startposition an (Zentrum der Orbit-Trajektorie)
2. Fährt N−1 gleichmäßig verteilte Punkte auf einem Kreisring — Kamera wird
   an jedem Punkt zum Board-Zentrum geschwenkt
3. An jedem Wegpunkt: warten bis Roboter ausschwingt, dann mehrere Detektionen mitteln
4. Fährt zurück zur Startposition
5. Berechnet die Hand-Eye-Transformation (`cv2.calibrateHandEye`, Tsai-Methode)
6. Berechnet daraus `T_robot_conveyor` und `T_robot_base_static_cam`
7. Speichert alles in `calibration.yaml` und `calibration.json`

Prädikate: `is_running`, `is_calibrated`, `has_failed`

---

### `CalibrationTestDrive` — Kalibrierung grob überprüfen

Liest die erzeugte `calibration.yaml` und fährt eine Validierungssequenz.
Dient als Sichtprüfung beim Debuggen — kein Ersatz für eine Genauigkeitsmessung.

Service: `start_test_drive` (Button in AICA Studio)

**Phase 0 — Zentrieren:**
Roboter fährt so, dass das Kamera-Fadenkreuz im Debug-Bild exakt über der Board-Mitte
steht. Der physische Versatz zwischen Flansch und Kameraoptik (`T_ee_cam`) wird dabei
eingerechnet. Prüfkriterium: Fadenkreuz landet mittig auf dem Board.

**Phase 1 — Pause:**
Roboter steht, Board visuell prüfen.

**Phase 2 — Vorwärts:**
+200 mm entlang der Förderband-Y-Achse (Laufrichtung).
Prüfkriterium: Roboter bewegt sich parallel zur Bandkante, keine seitliche Drift.

**Phase 3 — Rückwärts:**
Zurück zur Ausgangsposition.

**Phase 4 — Heimfahrt:**
Zurück zur ursprünglichen Benutzerpose (nur wenn `center_over_board = true`).

---

### `extrinsic_calibration.py` — Mathematikbibliothek (kein AICA-Node)

ROS-freie Hilfsfunktionen für `AutoCalibration`:

- `solve_eye_in_hand()` — Kern der Kalibrierung, nutzt `cv2.calibrateHandEye`
- Rotationshelfer: Matrix ↔ Quaternion ↔ Rodrigues ↔ RPY
- `save_calibration_json()` / `save_calibration_yaml()` — schreibt das Ergebnis
  gleichzeitig in mehrere Pfade

---

## Signalverbindungen

Beide `BoardDetection`-Instanzen (eine pro Kamera) liefern ihre Ausgaben an `AutoCalibration`.
`robot_ee_pose` kommt von der Roboter-Hardware.

**Eingaben `AutoCalibration`:**
- `robot_cam_board_pose` — Board-Pose aus der Handkamera: `[tx, ty, tz, rx, ry, rz]`
- `base_cam_board_pose` — Board-Pose aus der Basiskamera: `[tx, ty, tz, rx, ry, rz]`
- `board_geometry` — Board-Abmessungen von `BoardDetection`
- `robot_ee_pose` — aktuelle Flansch-Pose vom Roboter

**Ausgaben `AutoCalibration` → `calibration.yaml`:**
- Berechnete Transformationsmatrizen (Datei, keine Live-Signale)
- `target_ee_pose` → Signal Point Attractor (steuert den Roboter während der Trajektorie)

**Eingaben `CalibrationTestDrive`:**
- `calibration.yaml` (Datei, eingelesen beim Start)
- `robot_ee_pose` — aktuelle Flansch-Pose
- `board_geometry` — zum Berechnen der Board-Mitte für die Zentrierung

---

## Ergebnisdatei `calibration.yaml`

```yaml
transformations:
  T_ee_robot_cam:           # Kamera relativ zum Flansch
    source_frame: robot_camera_frame
    target_frame: end_effector
    homogeneous_matrix: [...]   # 4x4, Einheit Meter

  T_robot_conveyor:         # Förderband relativ zur Roboterbasis
    source_frame: conveyor_frame
    target_frame: robot_base
    homogeneous_matrix: [...]

  T_robot_base_static_cam:  # Basiskamera relativ zur Roboterbasis
    source_frame: base_camera_frame
    target_frame: robot_base
    homogeneous_matrix: [...]

validation:
  position_rmse_mm: 1.25
  sample_count: 9
```

Die Datei wird gleichzeitig nach `/tmp/`, auf den AICA-Desktop und an den
konfigurierten Pfad geschrieben.

---

## Parameter

### `BoardDetection`

- **`aruco_dictionary`** (Standard: `DICT_6x6_250`) — ArUco-Dictionary des Boards
- **`board_rows`** (Standard: `5`) — Zeilen des Boards (Y-Richtung)
- **`board_cols`** (Standard: `7`) — Spalten des Boards (X-Richtung)
- **`checker_size_mm`** (Standard: `35.0`) — Kantenlänge eines Schachfelds in mm
- **`marker_size_mm`** (Standard: `26.0`) — Kantenlänge eines ArUco-Markers in mm
- **`min_detected_markers`** (Standard: `4`) — Mindestanzahl erkannter Marker für eine gültige Pose
- **`debug_enable`** (Standard: `true`) — Debug-Bild mit eingezeichneten Markern ausgeben

### `AutoCalibration`

- **`num_waypoints`** (Standard: `9`) — Gesamtanzahl Wegpunkte (1 Zentrum + Kreisring)
- **`max_orbit_radius_mm`** (Standard: `50.0`) — Radius des Kreisrings in mm
- **`settle_time_s`** (Standard: `0.8`) — Wartezeit nach Anfahren vor Bildaufnahme in s
- **`samples_per_waypoint`** (Standard: `5`) — Gemittelte Detektionen pro Wegpunkt
- **`conveyor_offset_x_mm`** (Standard: `-130.0`) — X-Position des Board-Ursprungs (Marker-0-Ecke) im Förderband-Frame
- **`conveyor_offset_y_mm`** (Standard: `243.0`) — Y-Position des Board-Ursprungs im Förderband-Frame
- **`conveyor_offset_z_mm`** (Standard: `0.0`) — Z-Position des Board-Ursprungs (Höhe, normalerweise 0)
- **`calibration_file_path`** — Zielpfad der Ergebnisdatei

### `CalibrationTestDrive`

- **`test_distance_y_mm`** (Standard: `200.0`) — Fahrstrecke entlang Förderband-Y-Achse in mm
- **`drive_speed_m_s`** (Standard: `0.05`) — Fahrgeschwindigkeit in m/s
- **`center_over_board`** (Standard: `true`) — Kamera-Fadenkreuz vor Fahrt über Board-Mitte zentrieren
- **`phase_pause_s`** (Standard: `2.0`) — Pause zwischen den Phasen in s
- **`calibration_file_path`** — Pfad zur einzulesenden Kalibrierungsdatei

---

## Koordinatensystem `conveyor_frame`

Der `conveyor_frame` ist ein **künstlich definiertes Hilfskoordinatensystem**. Er wurde
eingeführt, um dem Nutzer eine anschauliche Referenz zu geben — Positionen auf dem
Förderband lassen sich damit intuitiv in Laufrichtung (Y), Breite (X) und Höhe (Z)
angeben, statt in Roboterbasis-Koordinaten.

- `+Y` — Laufrichtung des Bands
- `+X` — quer, nach rechts in Laufrichtung gesehen
- `+Z` — senkrecht nach oben aus der Bandoberfläche

**Das eigentliche Ergebnis der Kalibrierung** sind die Transformationen zwischen den
physikalischen Objekten: Wo sieht jede Kamera das ChArUco-Board (`board_pose`), und
wie liegt der Flansch gerade im Raum (`robot_ee_pose`)? Aus diesen gemessenen Beziehungen
berechnet `solve_eye_in_hand()` die Kameraposen relativ zur Roboterbasis.

Der `conveyor_frame` wird daraus abgeleitet: Der Board-Ursprung (Marker-0-Ecke) wird
als bekannter Punkt im Förderband-Frame angesetzt — die Position muss einmalig ausgemessen
und als `conveyor_offset` in `AutoCalibration` eingetragen werden. Das Board liegt
spiegelbildlich, daher dreht `AutoCalibration` das Board-Koordinatensystem um 180° um
die Y-Achse, um den `conveyor_frame` herzuleiten.

