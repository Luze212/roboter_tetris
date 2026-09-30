# Messdaten Basiskamera-Kalibrierung, 25.09.2026

> **Ausgang** (`architektur/entscheidungen.md` §5.4, §5.5): Aus diesen Daten stammt das Modell des
> Tiefenfehlers; die Kalibrierung für `base_cam` wurde am 28.09. am Aufbau abgenommen
> (`../2026-09-28-basiskamera-kalibrierung/`). `base_cam` bleibt unverändert, in Kraft ist L6.

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
| `auswertung_farbe_gegen_tiefe.py` | vergleicht je Aufnahme die Board-Ebene aus dem Farbbild (alle Rasterecken) mit der aus dem Tiefenbild und gibt den Tiefenfehler am Board aus; läuft mit OpenCV 4.6 und 4.7 |
| `lauf3_kalibrierung_basecam.json` | Kalibrierung für `base_cam`, aus den Rohdaten von Lauf 3 nachgerechnet (`test/tools/basecam_kalibrierung.py`, L27): 2,7 mm im Mittel, 3,2 mm höchstens neben L6 auf dem Band. Vergleichswert für den nächsten Lauf; **nicht** die genutzte Kalibrierung |
| `modell_tiefenfehler.py` | passt das Modell des Tiefenfehlers an (M0, M1, M1s, M2), kreuzvalidiert je Aufnahme, prüft das Band nach der Korrektur und misst das Bandprofil allein aus dem Farbbild; ohne Argumente, rund 1 min |

`*_grau.png` ist das Farbbild als Graubild. `*_tiefe_mm.png` ist das zum Farbbild ausgerichtete
Tiefenbild als 16-Bit-PNG in mm, verlustfrei, bei den Bandaufnahmen der Median aus 15
Bildern. Auswertung aus `source/roboter_tetris`:
`PYTHONPATH=. python3 ../../docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/auswertung_farbe_gegen_tiefe.py band_a1 greifer_start`

## Befunde

- Lauf 1 (mit Kippen, ohne Gummihülle): Board im Griff verrutscht (0,5°). Deshalb kippt der
  Plan seither nicht mehr, und die Kamerahöhe kommt aus dem Tiefenbild.
- Läufe 2 und 3: Bildfehler 0,37–0,38 px, Prüfposen 0,50–0,55 mm, untereinander auf
  Bruchteile eines Millimeters und Tausendstel Grad gleich. Lage auf dem Band gegen L6:
  2,2–8,9 mm über den Bildausschnitt von `base_cam`, im Mittel 4,4 mm (der Bericht
  `gegen_vorher_band_mm` 8,86 ist das Maximum; Nachprüfung 27.09.2026). Der Kameraort
  liegt +9 mm in x und +16 mm in y neben L6 — die Verschiebung gleicht die rund 1,1°
  Neigungsunterschied aus, die L6 aus dem Tiefenbild übernommen hat.
- **Das Tiefenbild ist gegen das Farbbild verkippt.** Dieselbe Board-Ebene, gleichzeitig über
  die Tags und über die Tiefe gemessen:

  | Aufnahme | Board-Mitte im Bild (u, v) | Abstand | Neigung | um Bild-x | um Bild-y | Ebenenabstand Tiefe − Farbe | **Tiefe zu tief am Board** |
  |---|---|---|---|---|---|---|---|
  | greifer_start | (643, 334) | 0,53 m | 1,18° | −1,09° | +0,45° | 4,7 mm | **5,5 mm** |
  | band_roboterseite | (454, 470) | 0,85 m | 1,55° | −1,55° | +0,11° | 6,6 mm | **8,9 mm** |
  | band_mitte | (820, 458) | 0,85 m | 1,44° | −1,31° | +0,59° | 5,9 mm | **9,6 mm** |
  | band_gegenseite | (852, 347) | 0,85 m | 1,24° | −1,24° | −0,04° | 8,4 mm | **8,1 mm** |
  | band_a1 | (662, 599) | 0,85 m | 1,75° | −1,72° | +0,29° | 6,2 mm | **12,9 mm** |
  | band_a2 | (500, 609) | 0,85 m | 1,56° | −1,55° | +0,08° | 7,2 mm | **13,5 mm** |
  | band_a3 | (828, 609) | 0,85 m | 1,67° | −1,65° | +0,25° | 5,5 mm | **13,1 mm** |
  | band_a4 | (831, 337) | 0,85 m | 1,24° | −1,23° | +0,12° | 8,1 mm | **7,8 mm** |
  | band_a5 | (642, 326) | 0,85 m | 1,71° | −1,68° | +0,34° | 7,5 mm | **6,1 mm** |
  | band_a6 | (499, 328) | 0,85 m | 1,05° | −1,01° | −0,29° | 5,6 mm | **5,7 mm** |
  | band_b1 | (761, 395) | 0,75 m | 1,29° | −1,29° | +0,12° | 6,6 mm | **7,6 mm** |
  | band_b2 | (494, 412) | 0,75 m | 1,11° | −1,09° | −0,20° | 5,5 mm | **6,8 mm** |
  | band_b3 | (819, 398) | 0,75 m | 1,47° | −1,38° | +0,52° | 6,8 mm | **8,9 mm** |

  Überall 1,0–1,8° um die waagerechte Bildachse, dazu ein Muster über den Bildort.
  **Zwei Spalten „zu tief“** (Nachprüfung 27.09.2026): Der *Ebenenabstand* (5–8 mm) ist
  der Unterschied der beiden Ebenen im Lot von der Kamera, also nahe der optischen Achse.
  Für `base_cam` zählt der Fehler **am Ort des Klotzes**: Tiefenpixel (Median 5 × 5) an
  jeder Rasterecke minus deren z aus der Farb-Pose. Er wächst mit der Bildzeile, weil die
  Verkippung um die waagerechte Bildachse liegt — **5,5–7,8 mm bei v ≈ 330, bis
  13,5 mm bei v ≈ 600**; der Bildausschnitt von `base_cam` reicht bis v = 640.

  Hinzu kommen die Posen im Greifer aus Lauf 3: **28 von 44 mit Tiefe** (Board
  0,49–0,59 m unter der Kamera), Neigung 0,67–1,45°, im Mittel 1,03°. Den 16 Posen der
  obersten Lage (Flansch z 0,473, Board ≈ 0,44 m) fehlt die Tiefe über dem Board ganz —
  am Aufbau klären, warum. Lauf 2 hat keine Tiefe gespeichert.

  Die echte Querneigung des Bandes (0,39°, Einrichtung §8) verfälscht den Befund nicht:
  Unter der Farb-Lage ist das Band im Tiefenbild 1,08° **längs** (entlang y) geneigt und nur
  0,29° quer; unter L6 liegt es eben (0,18°, Höhe 52,1–54,0 mm im Bildausschnitt).

  `base_cam` nimmt Umriss und Tiefe der Klötze aus dem ausgerichteten Tiefenbild (im
  Pixelraster des Farbbilds). Mit der Lage der Farbkamera allein wären die Klotzhöhen am
  unteren Bildrand bis zu rund 13 mm falsch. Die frühere Aussage, eine starre
  Ersatz-Kalibrierung reiche nicht, weil das Band 0,55° bzw. 0,68° schief blieb, ist
  überholt: Der Rest ist die echte Querneigung des Bandes (siehe unten). In Kraft ist L6
  (L27).

## Modell des Tiefenfehlers (A2, 27.09.2026)

Auswertung mit `modell_tiefenfehler.py`. Wahrheit je Pixel ist das z des Boards aus der
Farb-Pose, Messung das Tiefenpixel. Daten: die 13 Aufnahmen oben (dicht) und die 28
Greiferposen mit Tiefe aus Lauf 3.

**Ausgeschlossen:**

- *Maßstab des Boards oder der Brennweite.* Stufe 1 mit freiem Board-Maßstab gelöst:
  s = 0,9991 ± 0,00003, in Lauf 2 und 3 gleich. Die Farbabstände stimmen auf 0,5–0,8 mm;
  der Versatz liegt in der Tiefe.
- *Helligkeit.* Dunkle gegen helle Board-Pixel: höchstens 0,5 mm Unterschied.
- *Gedrehtes Tiefenbild.* Board-Umriss im Tiefenbild gegen den im Farbbild (`band_b1…b3`,
  `greifer_start`): 2–7 px Versatz, etwa 2–6 mm, in v einheitlich rund −3 px. Eine starre
  Drehung um 1,4° ergäbe rund 22 px. Der Fehler steckt im Tiefen*wert*.

**Modell M1s** (gewählt): in dem, was `base_cam` zur Hand hat — Pixel (u, v), gemessene
Tiefe z in m, Strahl ohne Entzerrung:

```
X = (u − cx) / fx · z,   Y = (v − cy) / fy · z
e [mm] = c0 + c1·X + c2·Y + c3·z          z_korrigiert = z − e / 1000
c = [3,715, 3,359, 21,812, 4,427]
```

Das sind 3,7 mm Versatz, 1,25° um die Bild-x- und 0,19° um die Bild-y-Achse, 4,4 mm je
Meter Abstand. Korrektur bei 0,80 m: 0,5–2,3 mm am oberen, 11,7–13,6 mm am unteren Rand
des Bildausschnitts.

Median-Rest je Aufnahme, wenn die Aufnahme beim Anpassen fehlt (roh 5,5–13,4 mm):

| Modell | Parameter | größter Rest | rms |
|---|---|---|---|
| M0 Versatz | 1 | 6,6 mm | 3,2 mm |
| M1 Versatz + Neigung | 3 | 2,1 mm | 1,0 mm |
| **M1s** M1 + Maßstab | 4 | **1,5 mm** | **0,7 mm** |
| M2 M1s + quadratisches Bildmuster | 7 | 1,3 mm | 0,6 mm |

**Gegenproben am Band** (beide unabhängig von der Anpassung):

| | Steigung quer (x) | längs (y) |
|---|---|---|
| Band im Tiefenbild, roh, unter der Farb-Lage | −0,30° | −1,07° |
| Band im Tiefenbild, korrigiert mit M1s | **−0,49°** | **+0,18°** |
| Bandprofil allein aus dem Farbbild (9 flache Board-Lagen) | **−0,64°** | **+0,25°** |
| Antasten B17 (im Greifbereich, y −0,20…+0,07) | −0,39° | 0,01° |

Negativ in x heißt: Das Band steigt zur Gegenseite (−x) an. Korrigierte Tiefe und
Farbbild stimmen auf 0,15° überein. **Die Querneigung ist echt.** Über die Board-Lagen
schwankt die Bandhöhe um −3,4…+2,1 mm; die Board-Neigung selbst reicht von −0,15° bei
x −0,63 bis −1,05° bei x −0,94, das Band ist also auch leicht gewölbt. L6 hat die
Neigung — wie die Verkippung der Tiefe — in die Extrinsik aufgenommen, deshalb liegt das
Band dort eben.

**Nicht mehr gemessen** (`entscheidungen.md` §13): Materialversatz Kunststoff gegen Papier-Board (bisher
`top_depth_bias_mm` 11,5 gegen das Band), Restversatz des Tiefenbilds in v (≈ 3 px) an
Klotzkanten prüfen, fehlende Tiefe bei 0,44 m.
