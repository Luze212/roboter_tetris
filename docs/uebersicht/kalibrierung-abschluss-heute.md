# Kalibrierung abschließen – Arbeitsplan für heute

Ziel für heute ist ein belastbarer, **getrennter Vergleich** zwischen einer
erfolgreichen BaseCamCalibration und einer erfolgreichen
RobotCamHandEyeThreeBoardPositions mit drei Board-Lagen. Die beiden Ergebnisse
werden zunächst nicht fusioniert und nicht im Pick-System aktiviert.

Die zugehörige AICA-Anwendung heißt `Calibration_Robot_Cam_Triple`. Ihre
gespeicherte, versionierte Vorlage ist
[`anwendung-calibration-robot-cam-triple-mit-fusion.yaml`](anwendung-calibration-robot-cam-triple-mit-fusion.yaml).
Die vorherige AICA-Anwendung ist als
`Calibration_Robot_Cam_Triple_Backup_2026-10-08` erhalten.

## Was vor dem Start gilt

- Das Paket und das AICA-Core-Image sind bereits neu gebaut. Nur wenn heute
  Code geändert wird, ist erneut mit `--no-cache` zu bauen und das Image neu zu
  erzeugen.
- Den Roboter erst einschalten, den Arbeitsraum freigeben und die normale
  Sicherheitsprüfung am Aufbau durchführen. Bis zu diesem Punkt bleibt die
  AICA-Anwendung gestoppt.
- Das ChArUco-Board muss für **beide** Kameras sichtbar sein. Es wird an jeder
  der drei Lagen fest und unverändert gehalten, bis der jeweilige Orbit
  abgeschlossen ist.
- Der heutige Vergleich braucht eine **neu erfolgreich geschriebene**
  BaseCamCalibration der Betriebsart `stufe1` oder `stufe2`. Die vorhandene
  Übergangsdatei `uebergang_L6` und ein abgebrochener Lauf zählen nicht als
  Messresultat.

## 1. BaseCamCalibration erfolgreich erzeugen

Die vorhandene Anleitung in
[`einrichtung-projektanwendung.md`](einrichtung-projektanwendung.md) §10
ausführen. Entscheidend ist der Abschluss ohne `FEHLER` und mit der Meldung,
dass die Kalibrierung geschrieben wurde.

Danach festhalten:

- Pfad der aktiven Ergebnisdatei;
- Pfad der neu angelegten Archivdatei unter
  `/data/calibration_archive/base_cam/`;
- Datum/Uhrzeit, Methode (`stufe1_basecam` oder `stufe2_basecam`) und die
  Qualitätswerte aus dem Log.

Bei verletzten Gütegrenzen wird keine aktive Datei geschrieben. In diesem Fall
den Lauf beheben und wiederholen; nicht mit dem vorherigen L6-Ergebnis
weitermachen.

## 2. Drei-Lagen-Robot-Cam-Kalibrierung fahren

1. AICA-Anwendung starten. Dadurch werden Hardware, beide Kameras,
   ChArUco-Erkennungen, Attractor und die Fusionskomponente geladen. Die
   Triple-Kalibrierung selbst bleibt zunächst entladen.
2. Mit **Startpose anfahren (Roboter bewegt sich)** den Frame
   `Kalibrierstart` anfahren. Er liegt in `world` bei
   `x=-0,563646 m`, `y=0,701042 m`, `z=0,572258 m`.
3. Warten, bis der Roboter an der Startpose steht. Dann
   **Frame-Steuerung aus** drücken. Erst danach
   **3-Board-Komponente laden (vorher Frame-Steuerung aus)** drücken.
4. Prüfen, dass beide ChArUco-Erkennungen gültige Board-Beobachtungen liefern.
   In der Triple-Komponente müssen diese Werte gesetzt bleiben:

   | Parameter | Wert |
   | --- | --- |
   | `board_position_count` | `3` |
   | `auto_approach_enabled` | `true` |
   | `auto_approach_clearance_mm` | `100` |
   | `base_cam_samples_per_board_position` | `10` |
   | Arbeitsraum X | `-0,900` bis `-0,500 m` |
   | Arbeitsraum Y | `0,500` bis `0,760 m` |
   | Arbeitsraum Z | `0,310` bis `0,570 m` |

   `Kalibrierstart` liegt mit `z=0,572258 m` um 2,258 mm oberhalb dieser
   Orbit-Grenze. Das ist als manuell geprüfte, freie Beobachtungspose erlaubt;
   die Komponente akzeptiert für die automatische Rückfahrt höchstens 5 mm
   Abweichung. Zentrierte Sicht- und Orbit-Posen bleiben strikt innerhalb der
   angegebenen Grenzen.

5. Board-Lage 1 in Sicht der Basiskamera platzieren und
   **Start 3 Board-Lagen (Roboter bewegt sich)** drücken. Nach zehn
   Basiskamera-Aufnahmen fährt die Komponente zunächst einen **vorläufigen**
   Orbit an der Kalibrierstartpose. Dieser dient nur dazu, die Boardmitte
   erstmals in `world` bestimmen zu können und wird verworfen. Der Roboter
   fährt danach kamerazentriert über die ChArUco-Boardmitte, blickt senkrecht
   nach unten und zeichnet dort den endgültigen Orbit auf. Danach kehrt er zur
   freien Kalibrierstartpose zurück.
6. Erst wenn `board_position_1_complete=true` und
   `waiting_for_board=true` sind, das Board mindestens 30 mm versetzen. Es
   muss erneut ruhig und vollständig für die Basiskamera sichtbar sein. Dann
   **Board umgesetzt – weiter (Roboter bewegt sich)** drücken.
7. Dasselbe für Lage 3 wiederholen. Für Lage 2 und 3 berechnet die Komponente
   aus der Basiskamera-Messreihe die geometrische Boardmitte in `world`, fährt
   die Roboterkamera direkt darüber und hält ihre Höhe wie bei Kalibrierstart.
   Ihre optische Achse zeigt senkrecht nach unten. Jeder Ziel- und Orbitpunkt
   muss innerhalb des Kalibrier-Arbeitsraums liegen; andernfalls bricht der
   Lauf vor der Bewegung ab. Nach jedem Orbit kehrt der Roboter zur freien
   Kalibrierstartpose zurück.

Ein erfolgreicher Abschluss verlangt:

- `board_position_1_complete`, `board_position_2_complete` und
  `board_position_3_complete` sind `true`;
- `is_calibrated=true` und `has_failed=false`;
- der Log nennt die kombinierte Position-/Rotationsstreuung und die aktive
  Datei `/data/robot_cam_handeye_3positions_calibration.json`;
- zusätzlich existiert eine neue, zeitgestempelte Datei unter
  `/data/calibration_archive/robot_cam_handeye_multi_board/`.

Die Triple-Datei enthält absichtlich keinen Conveyor-Frame. Die alte
Robot-Cam-Testfahrt darf dafür nicht verwendet werden.

### Bei einem abgelehnten Drei-Board-Lauf

Wenn die Konsistenzgrenze überschritten wird, entsteht **keine** aktive
Kalibrierdatei und keine Fusionsquelle. Stattdessen schreibt die Komponente eine
reine Diagnose unter
`/data/calibration_archive/robot_cam_handeye_multi_board/failed_runs/`.
Sie enthält die Einzelmatrizen jeder abgeschlossenen Board-Lage,
Basiskamera-Streuung sowie alle paarweisen Abstände von
`flange_robot_cam` und `world_base_static_cam`. Diese Diagnose wird weder vom
Pick-System noch von der Fusionskomponente gelesen.

## 3. Ergebnisse sichern und vergleichen

Für den Vergleich ausschließlich die beiden soeben erzeugten **Archivdateien**
verwenden. Sie bleiben auch bei einem späteren Wiederholungslauf unverändert.

Die spätere Auswertung soll eine eigene Markdown- und JSON-Datei erzeugen und
mindestens enthalten:

- vollständige Quellpfade, Zeitstempel, Methoden und SHA-256-Prüfsummen;
- `Δx`, `Δy`, `Δz` und den euklidischen Positionsabstand der statischen
  Basiskamera in `world`;
- den echten relativen Drehwinkel in Grad sowie die relative Drehachse;
- zur Lesbarkeit zusätzlich die jeweiligen Roll-, Pitch- und Yaw-Werte, aber
  nicht deren Differenz als alleinige Orientierungsbewertung;
- die Qualitätswerte beider Verfahren: BaseCam-Gütegrenzen und bei Robot-Cam
  Samples, RMSE sowie die Streuung zwischen den drei Board-Lagen;
- eine kurze Einordnung, ob die Abweichung für den Pick-Bereich plausibel ist.

Erst nach diesem Vergleich wird entschieden, ob und mit welchem Gewicht die
Fusionskomponente verwendet wird. Für heute ist der Fusionsbutton daher nicht
Teil des Prüflaufs.

## 4. Später zu erstellen: HKA-Folie

Nach der Auswertung genau **eine PowerPoint-Folie im HKA-Design** erstellen.
Als Gestaltungsquelle die im Projekt verwendete HKA-Vorlage beziehungsweise die
in den bisherigen Projektgesprächen vereinbarten Farben, Schrift und Kopfzeile
nutzen. Die Folie soll zeigen:

- Titel: Vergleich BaseCamCalibration und 3-Lagen-Robot-Cam-Kalibrierung;
- zwei kleine Kamera-Posen oder eine schematische Gegenüberstellung;
- Positionsabweichung in mm und relativen Drehwinkel in Grad als zentrale
  Kennzahlen;
- die wichtigste Qualitätszahl beider Verfahren;
- eine knappe Empfehlung: getrennt behalten, Wiederholung nötig oder
  Fusionsgewicht als nächster Test.

Die Folie und die Auswertung werden erst erstellt, wenn die konkreten
Archivdateien vorliegen.

## Nicht blockierend, aber noch offen

Die ältere Komponente **Robot-Kamera Hand-Auge-Kalibrierung (1 Board-Lage)**
und ihre zugehörige Testfahrt sind nicht Teil des heutigen Ablaufs. Sie starten
bei Aktivierung derzeit noch automatisch und haben nicht die strenge
Frische-/Frame-Prüfung der Triple-Komponente. Das ist ein separater
Nacharbeitspunkt; heute diese Komponenten nicht laden oder verwenden.

Die Triple-Komponente enthält die für heute notwendigen Sicherungen bereits:
kein Autostart beim Aktivieren, frische Flanschpose jünger als 0,5 s,
verbindlich `world`/`ur_tool0`, frische Board-Beobachtungen, stabile
Basiskamera-Messreihe, Mindestversatz zwischen Lagen, Arbeitsraumprüfung für
Annäherung, Sichtpose und alle Orbit-Wegpunkte, getrennte Ergebnisdatei und
Archivkopie.
