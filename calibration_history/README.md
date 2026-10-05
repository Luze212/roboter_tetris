# Historie der Robot-Kamera-Hand-Auge-Kalibrierung

Die Laufzeitdatei der **Robot-Kamera-Hand-Auge-Kalibrierung** liegt im Container
unter `/data/robot_cam_handeye_calibration.json`. Sie wird nicht direkt versioniert,
damit ein Kalibrierlauf die Git-Arbeitskopie nicht verändert.

Nach einem akzeptierten Lauf wird die Ergebnisdatei mit UTC-Zeitstempel in diesen
Ordner kopiert und gemeinsam mit dem Code versioniert. Die bestehende
Basiskamera-Kalibrierung bleibt davon getrennt: Ihre Datei ist
`source/roboter_tetris/roboter_tetris/Extrinsics/base_cam_extrinsics.json`.

`2026-10-05T143927Z.json` ist ein archivierter Lauf der vorigen Benennung
`/data/calibration.json`; sein Inhalt bleibt unverändert als Messhistorie erhalten.
