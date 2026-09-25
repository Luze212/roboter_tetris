# Messdaten Basiskamera-Kalibrierung, 25.09.2026

Erste Läufe der Komponente `base_cam_calibration` (Stufe 1) am Aufbau und eine Messreihe
zum Versatz zwischen Tiefen- und Farbbild der L515. Kamera: 1280 × 720, Intrinsik aus
`color_camera_info`:

- K = [[897.831, 0, 647.054], [0, 897.539, 362.306], [0, 0, 1]]
- D (plumb_bob) = [0.129869, −0.440544, −0.000885, −0.00064, 0.402135]

Board: calib.io AprilGrid 7 × 11, Tag 20 mm, Abstand 6 mm, t36h11. OpenCV liest die Tags
um 180° gedreht (`GridSpec.tag_rotation_deg`). Im Greifer mit einer Gummihülle an der
Grifffläche.

## Dateien

| Datei | Inhalt |
|---|---|
| `lauf2_rohdaten.json`, `lauf3_rohdaten.json` | alle Posen der Läufe 2 und 3: Flanschpose, Board-Punkte, Ecken im Bild. Lauf 3 zusätzlich je Pose die Tiefen-Ebene des Boards und ein Raster der Tiefenwerte (u, v, mm) sowie das Band am Start |
| `lauf2_ergebnis_farbkamera.json`, `lauf3_ergebnis_farbkamera.json` | gelöste Lage der **Farbkamera** in `world`. **Nicht** als Kalibrierdatei für `base_cam` verwenden, siehe unten |
| `greifer_start_*` | Startpose, Board im Greifer (0,53 m unter der Kamera) |
| `band_{mitte,roboterseite,gegenseite,a1…a6}_*` | Board flach auf dem Band an 9 Stellen (0,85 m) |
| `band_b1…b3_*` | Board erhöht auf einem 100-mm-Klotz an 3 Stellen (0,75 m) |
| `auswertung_farbe_gegen_tiefe.py` | vergleicht je Aufnahme die Board-Ebene aus dem Farbbild (alle Rasterecken) mit der aus dem Tiefenbild |

`*_grau.png` ist das Farbbild als Graubild. `*_tiefe_mm.png` ist das zum Farbbild ausgerichtete
Tiefenbild als 16-Bit-PNG in mm, verlustfrei, bei den Bandaufnahmen der Median aus 15
Bildern. Auswertung aus `source/roboter_tetris`:
`PYTHONPATH=. python3 ../../docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/auswertung_farbe_gegen_tiefe.py band_a1 greifer_start`

## Befunde

- Lauf 1 (mit Kippen, ohne Gummihülle): Board im Griff verrutscht (0,5°). Deshalb kippt der
  Plan seither nicht mehr, und die Kamerahöhe kommt aus dem Tiefenbild.
- Läufe 2 und 3: Bildfehler 0,37–0,38 px, Prüfposen 0,50–0,55 mm, untereinander auf
  Bruchteile eines Millimeters und Tausendstel Grad gleich. Lage auf dem Band gegen L6:
  2–3 mm.
- **Das Tiefenbild ist gegen das Farbbild verkippt.** Dieselbe Board-Ebene, gleichzeitig über
  die Tags und über die Tiefe gemessen:

  | Aufnahme | Board-Mitte im Bild (u, v) | Abstand | Neigung | um Bild-x | um Bild-y | Tiefe zu tief |
  |---|---|---|---|---|---|---|
  | greifer_start | (643, 334) | 0,53 m | 1,18° | −1,09° | +0,45° | 4,7 mm |
  | band_roboterseite | (454, 470) | 0,85 m | 1,55° | −1,55° | +0,11° | 6,6 mm |
  | band_mitte | (820, 458) | 0,85 m | 1,44° | −1,31° | +0,59° | 5,9 mm |
  | band_gegenseite | (852, 347) | 0,85 m | 1,24° | −1,24° | −0,04° | 8,4 mm |
  | band_a1 | (662, 599) | 0,85 m | 1,75° | −1,72° | +0,29° | 6,2 mm |
  | band_a2 | (500, 609) | 0,85 m | 1,56° | −1,55° | +0,08° | 7,2 mm |
  | band_a3 | (828, 609) | 0,85 m | 1,67° | −1,65° | +0,25° | 5,5 mm |
  | band_a4 | (831, 337) | 0,85 m | 1,24° | −1,23° | +0,12° | 8,1 mm |
  | band_a5 | (642, 326) | 0,85 m | 1,71° | −1,68° | +0,34° | 7,5 mm |
  | band_a6 | (499, 328) | 0,85 m | 1,05° | −1,01° | −0,29° | 5,6 mm |
  | band_b1 | (761, 395) | 0,75 m | 1,29° | −1,29° | +0,12° | 6,6 mm |
  | band_b2 | (494, 412) | 0,75 m | 1,11° | −1,09° | −0,20° | 5,5 mm |
  | band_b3 | (819, 398) | 0,75 m | 1,47° | −1,38° | +0,52° | 6,8 mm |

  Überall 1,0–1,8° um die waagerechte Bildachse und 5–8 mm zu tief, dazu ein Muster über
  den Bildort. Hinzu kommen die 45 Posen im Greifer aus Lauf 3 (0,44–0,58 m).

  `base_cam` setzt Klötze aus Farbpixel und Tiefe zusammen. Mit der Lage der Farbkamera
  allein wären die Klotzhöhen am Bildrand bis zu 9 mm falsch. Eine starre Ersatz-Kalibrierung
  und eine über das Bild lineare Tiefenskala reichen nicht (das Band blieb 0,55° bzw. 0,68°
  schief). Nächster Schritt: den Tiefenfehler über Bildort und Abstand aus diesen Daten
  modellieren, dann eine Korrektur in `base_cam`. Bis dahin gilt L6.
