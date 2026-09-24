# Arbeitsraum-Grenzen

Dieser Ordner ist bewusst von der Kalibrierung (`../Calibration/`), der Objekterkennung und der Greifaufgabe getrennt. Er enthält keine Messwerte einer physikalischen Kamera-Roboter-Beziehung, sondern eine sicherheitsrelevante Konfigurationsentscheidung: den erlaubten kartesischen Bewegungsbereich des Roboterflansches.

## Quelle der Grenzwerte

`workspace_bounds.json` ist die versionierbare Ablage für die Arbeitsraum-Grenzen. Sie ist die dokumentierte Quelle der Wahrheit, nicht die Laufzeit-Konfiguration — wirksam zur Laufzeit sind die gespiegelten AICA-Parameter (`ws_x_min`/`ws_x_max`/`ws_y_min`/`ws_y_max`/`ws_z_min`/`ws_z_max` im `object_follower`), analog zu `cal_x`/`cal_y`/`cal_z` aus der Extrinsik-Kalibrierung.

Seit 23.09.2026 `status: "defined"` — am Aufbau abgefahren (`docs/architektur/entscheidungen.md` Nachtrag 13 / L14). Seit 24.09.2026 sind die Werte zugleich **Standardwert** im `object_follower` (`follower_logic.FollowerParams` und `component_descriptions`, L15); `test_follower_logic.py` prüft die Gleichheit mit dieser Datei. Von Hand gesetzte Werte eines Blocks in der AICA-Anwendung gehen dem Standardwert vor.

## Vorgehen

1. Grundlage festlegen: entweder die Hersteller-Reichweite des Roboters (Datenblatt) minus Sicherheitsmarge, oder eine Vermessung am konkreten Aufbau (Förderband-/Ablagebereich, Hindernisse).
2. Die sechs Grenzwerte (`x_min`/`x_max`/`y_min`/`y_max`/`z_min`/`z_max`, Meter, im Frame `reference_frame`) in `workspace_bounds.json` eintragen.
3. Unter `validation` Datum, Verantwortliche:n und die verwendete Sicherheitsmarge dokumentieren. `status` erst nach dieser Festlegung auf `defined` setzen.
4. Die Werte in die AICA-Parameter übernehmen **und** die Standardwerte in `FollowerParams` und `component_descriptions/roboter_tetris_object_follower.json` nachziehen — sonst schlägt der Gleichheitstest fehl.

## Änderungsregel

Keine Werte ohne dokumentierte Begründung überschreiben. Bei jeder Änderung `defined_at`, `operator`, `safety_margin_m` und eine kurze Notiz ergänzen. Dadurch bleibt nachvollziehbar, welche Grenze mit welcher Begründung zu welchem Zeitpunkt galt.
