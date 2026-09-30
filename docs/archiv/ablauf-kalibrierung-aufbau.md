# Ablauf am Aufbau: automatische Kalibrierung der Basiskamera

> **Archiviert.** Termin durchgeführt am 28.09.2026; Ergebnis und Entscheidungen in
> `architektur/entscheidungen.md` L27. Seitdem schreibt der Lauf direkt in die Datei, die
> `base_cam` liest — Schritt 3 und 4 unten gelten so nicht mehr.

**Für den Termin am 28.09.2026** (halber Tag, letzter Termin am Roboter). Ziel: Die
automatische Kalibrierung liefert eine Kalibrierdatei für `base_cam`, die mindestens so gut
ist wie die Handkalibrierung L6 — nachgewiesen durch Antasten und einen Greiflauf.
Hintergrund: `architektur/entscheidungen.md` L27.

**Die genutzte Kalibrierung bleibt unangetastet.** `Extrinsics/base_cam_extrinsics.json`
(L6) wird nicht überschrieben. Die neue Datei wird über den Parameter „Kalibrierdatei“ von
`base_cam` eingestellt — zurück geht es mit demselben Parameter, ohne Build. Übernommen wird
erst, wenn alles getestet ist, und nur von Hand.

---

## Zeitplan

| # | Schritt | Zeit | Ergebnis |
|---|---|---|---|
| 1 | Paket bauen, Systemabbild neu erzeugen | 10 min | neue Parameter sichtbar |
| 2 | Kalibrierlauf | 15 min | `/tmp/base_cam_extrinsics.json` im Container |
| 3 | Dateien sichern | 5 min | Datei und Rohdaten auf dem Rechner |
| 4 | Neue Kalibrierung in `base_cam` einstellen | 10 min | Log: `stufe1_basecam` |
| 5 | Antasten: 6 Stellen, flacher und hoher Klotz | 45 min | Vergleich neu gegen L6 |
| 6 | Entscheiden | 5 min | behalten oder zurück |
| 7 | Greiflauf | 30–60 min | Griffe im Lauf mit der neuen Kalibrierung |
| | **Summe** | **≈ 2–2,5 h** | Rest Puffer |

---

## 1. Paket bauen

Wie immer: Paket bauen, **danach das AICA-Systemabbild im Launcher neu erzeugen** (sonst
wirkt das neue Paket nicht, `einrichtung-projektanwendung.md` §1).

**Gegenprobe:** Im Block „Base Kamera Kalibrierung“ gibt es die Gruppe „9 Tiefenfehler“
mit vier Parametern (3,715 / 3,359 / 21,812 / 4,427). Die Werte bleiben auf Standard.

## 2. Kalibrierlauf

Kalibrieranwendung laden (`anwendung-kalibrierung-basiskamera.yaml`, unverändert) und wie
gewohnt bedienen (`einrichtung-projektanwendung.md` §10): Knopf 1 Startpose, Knopf 2 halten,
Board mit Gummihülle am kurzen Rand einlegen, Knopf 3 Greifer zu, Knopf 5 Stufe 1 (3–4 min).

**Im Log prüfen** (Zeile „Ergebnis: …“):

| Wert | Erwartung | Wenn nicht |
|---|---|---|
| `bildfehler_px` | ≤ 0,5 | Board, Licht prüfen, Lauf wiederholen |
| `pruefposen_mm_mittel` | ≤ 1 | dito |
| `rutschen_mm` | ≤ 0,5 | Board fester einlegen, wiederholen |
| `basecam_gegen_vorher_band_mm_mittel` | ≈ 3 (gegen L6) | deutlich mehr: Kamera seit 23.09. bewegt (angestoßen?) — dann gilt nur noch der neue Lauf, das Antasten entscheidet |
| `basecam_rest_mm_max` | ≈ 11 | Grenze des unveränderten `base_cam` in den Bildecken, bei L6 genauso |

**Wiederholprobe gegen den 25.09.** Die Kamera wurde vor der Handkalibrierung L6
nachgeschraubt und steht seitdem (L6 am 23.09., finaler Build am 24.09., Kalibrierlauf am
25.09.). Der neue Lauf muss deshalb die Datei vom 25.09. treffen:
`docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/lauf3_kalibrierung_basecam.json`

| | cal_x | cal_y | cal_z | roll | pitch | yaw |
|---|---|---|---|---|---|---|
| 25.09. | −0,7735 | 0,7933 | 0,9169 | 179,459 | 0,268 | 179,828 |
| erwartet | ± 0,002 | ± 0,002 | ± 0,002 | ± 0,1 | ± 0,1 | ± 0,1 |

Lauf 2 und Lauf 3 am 25.09. lagen 0,1 mm (x, y), 0,4 mm (z) und 0,004° auseinander; die
Grenzen lassen Luft für einen Tag Abstand. Werte aus der Log-Zeile „Ergebnis geschrieben: … cal_x …“ ablesen. Liegen sie in diesen
Grenzen, ist das Verfahren über zwei Tage auf etwa 2 mm wiederholbar — ein Ergebnis für
Projektziel 2. Liegen sie deutlich daneben: Kamera angestoßen oder Lauf gestört — Board und
Log prüfen, Lauf einmal wiederholen; im Zweifel entscheidet das Antasten.

## 3. Dateien sichern

```bash
C=$(docker ps --format '{{.Names}}' | grep aica-launcher | head -1)
T=source/roboter_tetris
docker cp $C:/tmp/base_cam_extrinsics.json $T/roboter_tetris/Extrinsics/base_cam_extrinsics_2026-09-28.json
mkdir -p docs/architektur/bilder/2026-09-28-basiskamera-kalibrierung
docker cp $C:/tmp/base_cam_extrinsics_rohdaten.json docs/architektur/bilder/2026-09-28-basiskamera-kalibrierung/lauf_rohdaten.json
```

Neuer Dateiname — die genutzte `base_cam_extrinsics.json` bleibt, wie sie ist.

*Optional*, wenn auf dem Rechner numpy und OpenCV da sind — rechnet die Datei aus den
Rohdaten nach und vergleicht mit L6 und dem 25.09.:

```bash
cd source/roboter_tetris
PYTHONPATH=. python3 test/tools/basecam_kalibrierung.py ../../docs/architektur/bilder/2026-09-28-basiskamera-kalibrierung/lauf_rohdaten.json --vergleich Extrinsics/base_cam_extrinsics.json $PWD/../../docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/lauf3_kalibrierung_basecam.json
```

## 4. Neue Kalibrierung in `base_cam` einstellen

Projektanwendung laden. Im Block `base_cam` den Parameter **„Kalibrierdatei“**
(`calibration_file`) setzen — zwei Wege:

| Weg | Wert | wann |
|---|---|---|
| sofort, ohne Build | `/tmp/base_cam_extrinsics.json` | solange der Container seit dem Kalibrierlauf nicht neu gestartet wurde |
| dauerhaft | `Extrinsics/base_cam_extrinsics_2026-09-28.json` | nach einem Build mit der gesicherten Datei aus Schritt 3 (Systemabbild neu!) |

Anwendung starten. **Gegenprobe im Log** beim Aktivieren von `base_cam`:
„Kalibrierung aus …: stufe1_basecam, …“. Steht dort „unbrauchbar“ oder „cal_*“, rechnet
`base_cam` mit L6.

**Zurück zu L6:** Parameter auf `Extrinsics/base_cam_extrinsics.json`.

## 5. Antasten

Wie Block 2 im alten Fahrplan (`archiv/fahrplan-aufbau.md`). Werkzeuge in den Container:

```bash
C=$(docker ps --format '{{.Names}}' | grep aica-launcher | head -1)
T=source/roboter_tetris
docker cp $T/test/tools/signal_reader.py $C:/tmp/ && docker cp $T/roboter_tetris/contracts.py $C:/tmp/
R() { docker exec -u ros2 -e ROS_DOMAIN_ID=0 "$C" bash -lc "python3 /tmp/signal_reader.py $*"; }
```

**Drei Stellen, je flacher Klotz (25 mm, ungerade Nummer) und 100-mm-Klotz (gerade)**,
Klotz ruhend, Band aus:

| Punkt | Lage (world, ungefähr) | warum |
|---|---|---|
| P1 / P2 | x −0,65 · y 0,55 | Roboterseite, Bildanfang |
| P3 / P4 | x −0,93 · y 0,55 | Gegenseite, Bildanfang |
| P5 / P6 | x −0,80 · y 0,85 | Mitte, weit im Bild (Reichweite: bis etwa y 0,9) |

Je Punkt, erst die Kamera, dann der Roboter (geschlossene Backen mittig auf die Oberseite,
Werkzeug senkrecht):

```bash
R objects --duration 10 --summary | tail -1 | python3 $T/test/tools/b23_compare.py add pts.json P1 cam
R cartesian --duration 3 --summary | tail -1 | python3 $T/test/tools/b23_compare.py add pts.json P1 rob
```

Auswerten — ein Durchgang vergleicht beide Kalibrierungen:

```bash
python3 $T/test/tools/b23_compare.py eval pts.json --aktiv base_cam_extrinsics_2026-09-28.json --vergleich base_cam_extrinsics.json
```

`--aktiv` ist die Datei, mit der `base_cam` beim Messen lief — die in Schritt 3 gesicherte;
Dateinamen ohne Pfad sucht das Werkzeug in `roboter_tetris/Extrinsics/`. Die Tabelle „Kalibrierungen im Vergleich“ zeigt je Datei Lagefehler
und Höhenfehler gegen den Roboter.

## 6. Entscheiden

Verglichen wird **Punkt für Punkt gegen L6** — beide Kalibrierungen an denselben
Messungen, das Antastrauschen (1–2 mm auf der Gummimatte) trifft beide gleich. In den
Bildecken haben beide einige Millimeter mehr; das ist die Grenze des unveränderten
`base_cam`, kein Mangel der neuen Datei.

**Behalten, wenn** die neue Kalibrierung gegenüber L6

- beim **Lagefehler** im Mittel höchstens 1 mm schlechter ist und an keinem Punkt mehr
  als 2 mm schlechter, und
- bei der **Höhe** im Mittel innerhalb ±2 mm von L6 liegt.

Sonst: Parameter zurück auf L6. Nichts ist verloren; die Daten gehen in die Doku.

Erwartung aus den Messdaten vom 25.09.: Lage im Mittel etwa 1 mm besser als L6, Höhen gleich.
Die beiden Kontrollpunkte vom 24.09. (flacher und hoher Klotz, L6) rechnen sich mit der
Datei vom 25.09. von 1,5 / 5,6 mm auf 1,5 / 3,0 mm.

## 7. Greiflauf

Mit der neuen Kalibrierung wie gewohnt greifen: mindestens 10 Klötze, flach und hoch, auf
beiden Bandseiten. `err_laengs` in `follower_status` mitlesen (`lead_time_s` bleibt 0,24 —
die Kalibrierung ändert daran nichts). Fehlgriffe mit Klotzlage notieren.

## Wenn es eng wird

| Lage | Vorgehen |
|---|---|
| wenig Zeit | Antasten auf **zwei Stellen** kürzen (P1–P4, ≈ 20 min), dann Greiflauf |
| Kalibrierlauf scheitert an einer Gütegrenze | Rohdaten liegen trotzdem in `/tmp` — herauskopieren (Schritt 3), Board fester einlegen, Lauf einmal wiederholen |
| Kalibrierlauf geht gar nicht (Hardware, Zeit) | Datei vom 25.09. testen: `docker cp docs/architektur/bilder/2026-09-25-basiskamera-kalibrierung/lauf3_kalibrierung_basecam.json $C:/tmp/` und in `base_cam` `/tmp/lauf3_kalibrierung_basecam.json` eintragen. Gilt für die heutige Kameralage (seit dem Nachschrauben vor L6 unverändert), solange die Kamera seitdem nicht angestoßen wurde — das Antasten zeigt es |
| neue Datei fällt beim Antasten durch | zurück auf L6, Greiflauf mit L6. Für die Projektdoku zählt der Vergleich trotzdem: das Verfahren ist gemessen |
| Container startet neu, `/tmp` leer | die in Schritt 3 gesicherte Datei per Build (Weg „dauerhaft“) oder `docker cp` zurück nach `/tmp` |

## Danach

- Übernahme (Nutzer): Parameter dauerhaft auf die neue Datei **oder** Datei nach
  `Extrinsics/base_cam_extrinsics.json` kopieren, bauen. Dann in `entscheidungen.md` L27 den
  Stand nachtragen und `test_shipped_file_is_the_l6_calibration` anpassen (der Test schützt
  bis dahin genau die L6-Datei).
- Rohdaten, `pts.json` und Log-Auszug nach `docs/architektur/bilder/2026-09-28-…/`.
