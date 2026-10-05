# Basiskamera-Kalibrierungen kombinieren

Die eigenständige AICA-Komponente **Basiskamera-Kalibrierungen kombinieren** liest
das erfolgreiche Ergebnis der BaseCamCalibration und das validierte Ergebnis der
Robot-Kamera-Hand-Auge-Kalibrierung. Sie schreibt eine dritte Datei im vorhandenen
BaseCam-JSON-Schema. Die Quelldateien bleiben unverändert. Ein erneutes Schreiben
sichert die vorige kombinierte Datei als `base_cam_fused_extrinsics_vorher.json`.

Die Gewichtsangabe `robot_cam_weight_percent` bedeutet: 0 % übernimmt die
BaseCam-Pose, 100 % die Robot-Kamera-Pose. Dazwischen werden die Positionen in
Metern linear interpoliert. Die Orientierung wird entlang des kürzesten Wegs
zwischen den beiden Rotationen interpoliert; Roll/Pitch/Yaw werden **nicht**
einzeln gemittelt. Die Datei dokumentiert Gewicht, Abstand und Winkel zwischen
beiden Quellen sowie deren Pfade und SHA-256-Prüfsummen. Diese manuelle Gewichtung
ist keine automatisch ermittelte Genauigkeitsverbesserung. Insbesondere kann die
BaseCam-Methode eine Tiefenfehlerkorrektur enthalten, die in der
Robot-Kamera-Methode nicht gleich geschätzt wird. Die Kombination sollte deshalb
an realen Pick-Punkten geprüft werden.

Standardpfade im AICA-Container:

| Parameter | Standardwert |
| --- | --- |
| `base_cam_file` | `Extrinsics/base_cam_extrinsics.json` (relativ zum installierten Paket) |
| `robot_cam_file` | `/data/robot_cam_handeye_calibration.json` |
| `output_file` | `/data/base_cam_fused_extrinsics.json` |
| `robot_cam_weight_percent` | `50.0` |

Alle drei Pfade und das Gewicht sind in AICA änderbar. Der Projektordner enthält
den lokalen Link `base_cam_fused_extrinsics.json` auf die Ergebnisdatei im
AICA-Datenvolume; vor dem ersten erfolgreichen Schreiben ist der Link noch leer.
Die Datei `/data` bleibt beim Neubau des Pakets erhalten. Der Link ist
rechnerspezifisch und wird nicht mit Git versioniert. Für eine dauerhaft
versionierte Kalibrierung muss das konkrete Ergebnis nach einem geprüften Lauf
separat ins Repository übernommen werden.

Die Datei
[`anwendung-calibration-tobi-2-mit-fusion.yaml`](anwendung-calibration-tobi-2-mit-fusion.yaml)
ist eine versionierte Fassung von `Calibration_Tobi_2` mit dem neuen Block und
einem Button **Kalibrierungen kombinieren (keine Bewegung)**. Nach Paketbau und
Neuerzeugen des Systemabbilds kann sie als AICA-Anwendung importiert werden.
Der Block wird beim Start aktiviert; nur der Button ruft den Schreibdienst auf.
Er hat keine Kamera- oder Roboteranschlüsse.

**Vor dem ersten Kombinieren:** Beide Kalibrierungen müssen erfolgreich
abgeschlossen sein. Die aktuelle Übergangsdatei `uebergang_L6` zählt ausdrücklich
nicht als neues BaseCam-Messergebnis. Der zuletzt gemeldete BaseCam-Lauf hatte
die Gütegrenzen verletzt und die Ergebnisdatei daher nicht ersetzt. Der
Kombinationsdienst meldet in diesem Zustand einen Fehler und schreibt nichts.

**Für den Pick-Betrieb:** Die neue Datei wird nicht automatisch wirksam. In
`Finales System` muss am Block `BaseCam` der Parameter `calibration_file` erst
nach der Prüfung auf `/data/base_cam_fused_extrinsics.json` gesetzt und die
Komponente neu aktiviert werden. Die kombinierte Datei enthält keine
Referenzmarken und ist nicht für den Stufe-2-Prüflauf der BaseCamCalibration
gedacht.
