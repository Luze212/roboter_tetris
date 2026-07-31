# Arbeitsraum-Grenzen

Dieser Ordner ist bewusst von der Kalibrierung (`../Calibration/`), der Objekterkennung und der Greifaufgabe getrennt. Er enthält keine Messwerte einer physikalischen Kamera-Roboter-Beziehung, sondern eine sicherheitsrelevante Konfigurationsentscheidung: den erlaubten kartesischen Bewegungsbereich des Roboterflansches.

## Quelle der Grenzwerte

`workspace_bounds.json` ist die versionierbare Ablage für die Arbeitsraum-Grenzen. Sie ist die dokumentierte Quelle der Wahrheit, nicht die Laufzeit-Konfiguration — wirksam zur Laufzeit sind die gespiegelten AICA-Parameter (`workspace_x_min`/`x_max`/`y_min`/`y_max`/`z_min`/`z_max`) in der jeweiligen bewegungsauslösenden Komponente, analog zu `cal_x`/`cal_y`/`cal_z` aus der Extrinsik-Kalibrierung.

Aktuell ist die Datei mit `status: "placeholder_not_yet_defined"` markiert, alle Werte sind `null`. Solange `basis.method` nicht gesetzt ist, dürfen die Werte **nicht** in eine Komponente als Sicherheitsgrenze übernommen werden.

## Vorgehen

1. Grundlage festlegen: entweder die Hersteller-Reichweite des Roboters (Datenblatt) minus Sicherheitsmarge, oder eine Vermessung am konkreten Aufbau (Förderband-/Ablagebereich, Hindernisse).
2. Die sechs Grenzwerte (`x_min`/`x_max`/`y_min`/`y_max`/`z_min`/`z_max`, Meter, im Frame `reference_frame`) in `workspace_bounds.json` eintragen.
3. Unter `validation` Datum, Verantwortliche:n und die verwendete Sicherheitsmarge dokumentieren. `status` erst nach dieser Festlegung auf `defined` setzen.
4. Die Werte in die entsprechenden AICA-Parameter der bewegungsauslösenden Komponente(n) übernehmen.

## Änderungsregel

Keine Werte ohne dokumentierte Begründung überschreiben. Bei jeder Änderung `defined_at`, `operator`, `safety_margin_m` und eine kurze Notiz ergänzen. Dadurch bleibt nachvollziehbar, welche Grenze mit welcher Begründung zu welchem Zeitpunkt galt.
