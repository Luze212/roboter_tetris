# Kalibrierboards

Dieser Ordner bündelt Boarddefinitionen und Hilfsmittel für die Kalibrierung. Er gehört nicht zur Tetris-Aufgabenlogik.

## Dateien

- `charuco_5x7.json`: unterstützte ChArUco-Konfiguration für die AICA-Komponente `BoardDetection`.
- `aprilgrid_36h11.json`: AprilGrid-36h11-Definition für Offline-Kalibrierungswerkzeuge.
- `verify_board_configs.py`: gemeinsamer Ersatz für die früheren Root-Skripte `test_aprilgrid.py` und `test_board_detection.py`.

## Einsatz in AICA

`BoardDetection` erzeugt interpolierte ChArUco-Ecken und unterstützt momentan **nur ChArUco**. Für die aktive AICA-Komponente sind daher ausschließlich die Werte aus `charuco_5x7.json` zu verwenden.

Die Vision-Hilfsbibliothek `roboter_tetris.vision.board` kann zusätzlich `GRID`/AprilGrid aufbauen und Marker detektieren. Das ist für eine spätere Offline-Kalibrierung nützlich, liefert über die aktuelle AICA-Komponente aber keine Board-Ecken und keine Pose. Die frühere Anleitung, `board_type=GRID` an `BoardDetection` zu übergeben, war deshalb irreführend und wurde entfernt.

## Verifikation

Die gemeinsame Prüfung erzeugt beide Boards mit OpenCV und prüft damit Dictionary und Geometrie:

```bash
PYTHONPATH=source/roboter_tetris python3 source/roboter_tetris/roboter_tetris/Calibration/boards/verify_board_configs.py
```
