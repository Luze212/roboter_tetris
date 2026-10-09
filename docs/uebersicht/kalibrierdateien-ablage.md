# Ablage und Archiv der Kalibrierdateien

Jede erfolgreiche Kalibrierung schreibt weiterhin eine **aktive Datei**. Diese
Pfade bleiben absichtlich stabil, damit vorhandene AICA-Systembilder und die
Pick-Komponente `BaseCam` ohne Umverdrahtung funktionieren. Unmittelbar danach
wird derselbe, fertig geschriebene JSON-Datensatz zusätzlich in ein
unveränderliches Archiv kopiert. Die Archivdateien heißen beispielsweise
`base_cam_20261008T154233.123456Z.json`; sie werden nie durch einen späteren
Lauf ersetzt.

| Verfahren | Aktive Datei | Archivverzeichnis |
| --- | --- | --- |
| BaseCamCalibration | `Extrinsics/base_cam_extrinsics.json` | `/data/calibration_archive/base_cam/` |
| RobotCamHandEyeCalibration | `/data/robot_cam_handeye_calibration.json` | `/data/calibration_archive/robot_cam_handeye/` |
| RobotCamHandEyeThreeBoardPositions | `/data/robot_cam_handeye_3positions_calibration.json` | `/data/calibration_archive/robot_cam_handeye_multi_board/` |
| Basiskamera-Kalibrierungen kombinieren | `/data/base_cam_fused_extrinsics.json` | `/data/calibration_archive/base_cam_fusion/` |

`/data` ist das persistente AICA-Datenvolume und bleibt beim Paket-Neubau
erhalten. Die aktive Datei darf sich ändern; sie ist die bewusst ausgewählte
aktuelle Kalibrierung. Für Rückverfolgung, Vergleich und eine spätere Auswahl
werden ausschließlich die zeitgestempelten Dateien im Archiv verwendet.

Für den Pick-Betrieb wird weiterhin **eine** Datei am Parameter
`BaseCam.calibration_file` ausgewählt. Das kann eine aktive Datei oder nach
bewusster Auswahl eine konkrete Archivdatei sein. Die Fusionskomponente liest
zwei explizit angegebene Quelldateien und erzeugt eine neue aktive sowie eine
archivierte Fassung; sie überschreibt ihre Quellen nie.
