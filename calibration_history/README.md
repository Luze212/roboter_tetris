# Kalibrierungs-Snapshots

`/calibration.json` ist die aktuelle Laufzeitdatei der AICA-Komponenten. Sie ist
eine lokale Verknüpfung auf den persistenten AICA-Datenordner und wird nicht in Git
versioniert.

Jede freigegebene Kalibrierung wird hier als unveränderliche JSON-Kopie mit einem
UTC-Zeitstempel im Dateinamen abgelegt und zusammen mit dem zugehörigen Code
committet. Diese Snapshots dienen der Nachvollziehbarkeit; sie werden von AICA nicht
automatisch eingelesen und ersetzen nicht die Laufzeitdatei unter `/data`.

Für einen neuen Snapshot wird die geprüfte `/data/calibration.json` kopiert, etwa:

```bash
cp calibration.json calibration_history/2026-10-05T143927Z.json
```
