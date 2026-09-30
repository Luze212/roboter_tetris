# Kalibriertermin 28.09.2026

Ergebnis und Entscheidung: `architektur/entscheidungen.md` §5.5 (Abnahme am 28.09.2026).

| Datei | Inhalt |
|---|---|
| `lauf_rohdaten.json` | Kalibrierlauf Stufe 1: 40 Posen mit Tiefe je Pose, Band am Start |
| `kalibrierung_basecam_2026-09-28.json` | die Kalibrierung für `base_cam` aus diesem Lauf (`stufe1_basecam`); abgenommen im Greiflauf, **nicht übernommen** — in Kraft bleibt L6 |
| `greiflauf_log.txt` | Zustandswechsel des `object_follower` im Greiflauf mit dieser Kalibrierung: 7 von 7 abgelegt |
| `pts.json` | Antasten, nur Kamerapunkt P5 (flacher Klotz); das Antasten entfiel danach aus Zeitgründen |

Nachrechnen: `PYTHONPATH=. python3 test/tools/basecam_kalibrierung.py ../../docs/architektur/bilder/2026-09-28-basiskamera-kalibrierung/lauf_rohdaten.json` (aus `source/roboter_tetris`).
