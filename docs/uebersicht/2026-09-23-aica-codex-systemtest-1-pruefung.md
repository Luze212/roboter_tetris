# Prüfung Codex_Systemtest_1 gegen L14

Die daraus abgeleitete vollständige Anschlussanleitung steht in
[Codex_Systemtest_1 — Verdrahtung für L14](codex-systemtest-1-verdrahtung-l14.md).

Stand: 23.09.2026. Ausschließlich lesend geprüft; Anwendung nicht gestartet oder
geändert. Quelle: gespeicherte Anwendung aus der AICA-SQLite-Datenbank, geöffnet
mit `mode=ro`, sowie installierte Dateien im laufenden AICA-Systemcontainer.
Anwendungs-ID: `c431f1cf-da7a-4df6-9d55-0deaa9217c13`.
Gespeicherte Änderungszeit: 23.09.2026, 13:15:28 UTC.
Ungespeicherte Studio-Änderungen und effektive ROS-Laufzeitparameter sind damit
nicht geprüft. Die Studio-Oberfläche verlangte im Prüf-Browser eine Anmeldung.

## Ergebnis

**Die gespeicherte Anwendung ist für L14 unvollständig; auch das installierte
Paket enthält noch ältere Follower-/Auswahl-Defaults.** Ein Start der Anwendung
ist für diesen Befund nicht erforderlich.

| Prüfung | Ist-Zustand | Erforderlich |
|---|---|---|
| `object_follower` | fehlt vollständig | einfügen, parametrieren, Lifecycle und Signale verbinden |
| `priority_handler` | vorhanden, Lifecycle-Ereignisse vorhanden, fehlt in `on_start.load` | in vorgesehenen Startablauf aufnehmen |
| Zielquelle Attractor | `/frame_to_signal/pose` | Follower `target_pose` verwenden |
| Greifer-Befehl | `/true_signal/value` | Follower `gripper_close` verwenden |
| Auswahl → Follower | kein `target`-Ausgang im gespeicherten Graphen verbunden | `target` verbinden |
| Greifer-Rückmeldungen | keine Verbindungen | `motion_done`, `has_object` an Follower |
| Abschlussereignis | fehlt | Follower `picked_id` an Auswahl und `data_tracker` |
| Robot State → Follower | fehlt | `cartesian_state` an `robot_state` |
| Steuerungsparameter | keine expliziten Gain-/Geschwindigkeitswerte bei Attractor/IK gespeichert | für Teststufe setzen und mit Auswahl/Vorhalt abstimmen |
| Zusätzliche Last | beide Roboterkamera-Varianten, D435i und `interface_streamer` in `on_start.load` | für ersten Lauf ohne Roboterkamera aus Startpfad nehmen; Farbvariante nicht weiterverwenden |

Bereits vorhanden und verbunden: L515 → `base_kamera` → `vectoring` →
`priority_handler`; Roboterzustand → Auswahl und Attractor; Attractor-Twist →
IK; `tracks` und `not_pickable` → `data_tracker`; dessen `world_state` →
`interface_streamer`. `data_tracker` hat explizit 2 Hz, Anzeige 5 Hz;
`vectoring` und Auswahl haben keine explizite Rate im gespeicherten YAML.

Hardware wird beim Anwendungsstart geladen; ihre Ereignisse laden und aktivieren
Robot State Broadcaster und IK-Velocity-Controller. Der Attractor und der Greifer
werden ebenfalls automatisch konfiguriert und aktiviert. Starten ist somit kein
rein lesender Prüfakt. Die alten Testbuttons laden `frame_to_signal` bzw.
`true_signal`; diese Testpfade dürfen nicht parallel den Follower steuern.

## Installierter Paketstand

Aus `/ws/install/roboter_tetris/component_descriptions/` gelesen:

| Parameter | Installierter Default | L14 / vorgesehene Einrichtung |
|---|---|---|
| Follower `min_grip_height_m` | 0,015 | 0,021 |
| Auswahl `zone_y_min` | −0,45 | −0,22 |
| Auswahl `zone_y_max` | +0,05 | +0,40 |
| Auswahl `t_descend_s` | 2,0 | 1,2 als expliziter Anwendungswert bei `observe_z = 0.45` |

Installiert sind bereits `max_extrapolation_s = 0.6`, `target_timeout_s = 1.0`,
Roboterkamera-Gewichte 0, `top_depth_bias_mm = 11.5` und
`camera_node = /realsense_camera`. SHA-256-Vergleich: `track_estimation.py`,
`base_cam.py`, `contracts.py` stimmen mit dem Workspace überein;
`follower_logic.py`, `target_selection.py`, `object_follower.py` und
`priority_handler.py` unterscheiden sich. Der installierte Stand darf daher
nicht als identisch mit dem lokalen L14-Code gelten.

## Konkrete Ergänzung der Anwendung nach Nutzer-Build

1. Nutzer baut aktuellen Workspace und erneuert das AICA-Systemabbild im Launcher.
   Danach installierte Defaults gegenprüfen, Auswahl neu einfügen bzw. alle
   relevanten Werte ausdrücklich prüfen; Follower neu einfügen.
2. Follower-Pflichtwerte: `ws_x_min/max = -1.0/-0.30`,
   `ws_y_min/max = -0.32/0.48`, `ws_z_min/max = 0.3086/0.60`,
   `observe_x/y/z = -0.816/0.35/0.45`, `observe_yaw_deg = 90`.
   `min_grip_height_m = 0.021`, Gewichte der Roboterkamera 0;
   Auswahl `t_descend_s = 1.2`, L14-Zone und passende Regelungsparameter prüfen.
3. Fehlende Signalverbindungen:
   - `priority_handler.target` → `object_follower.target`
   - `robot_state_broadcaster.cartesian_state` → `object_follower.robot_state`
   - `object_follower.target_pose` → `signal_point_attractor.attractor`
   - `object_follower.gripper_close` → `robotiq_gripper.gripper_close`
   - `robotiq_gripper.motion_done` → `object_follower.gripper_motion_done`
   - `robotiq_gripper.has_object` → `object_follower.gripper_has_object`
   - `object_follower.picked_id` → `priority_handler.picked_id`
   - `object_follower.picked_id` → `data_tracker.picked_id`
4. Start-/Lifecycle-Ablauf für Auswahl und Follower ergänzen; unnötige
   Roboterkamera-/Anzeige-Blöcke für ersten Lauf aus dem Startpfad nehmen.
   Auswahl/Vectoring 20 Hz, Buchhaltung 2 Hz gemäß Fahrplan.
5. Attractor-Gain, Vorhalt und Geschwindigkeitslimits explizit abstimmen.
   Gedrosselter Erstlauf: IK 0,10 m/s, synthetisches Ziel 0,05 m/s;
   `attractor_v_max_mps` muss dem kleineren aktiven Limit entsprechen.
   `attractor_gain` entspricht `linear_gains`, `lead_time_s` zunächst etwa 1/K.
   Für echtes Band später gemäß Fahrplan mindestens 0,30 m/s, neu abgestimmt.
6. Erst anschließend getrennte Laufzeitprüfung nach Fahrplan und konkretem Go.
   Reale Konfiguration, Zeitdomänen, Datenfrische und Lifecycle-Erfolg lassen
   sich durch das Lesen der gespeicherten Anwendung nicht bestätigen.

Optional später: `robot_cam_2.object_position` → Follower `object_position`;
Anzeige mit `follower_status` und beiden Debugbildern vervollständigen.
Diese optionalen Verbindungen sind keine Voraussetzung für den geplanten
ersten Lauf ohne Roboterkamera und ohne `interface_streamer`.

Maßgeblich: `fahrplan-aufbau.md` (aktueller Kopf),
`einrichtung-projektanwendung.md` §§2, 6, 9 und
`../architektur/entscheidungen.md` Nachtrag 13 / L14.

## Nachtrag: Laufdiagnose am 23.09.2026 — korrigierte Einordnung

Die Containerlogs unterscheiden zwei Läufe (Zeiten Europe/Berlin):

- 17:29:36: Follower meldet `ABBRUCH -> WARTEN: angehoben`.
  Danach Warnungen wegen fehlendem frischen Roboterzustand.
- 17:30:52: Anwendung wird gestoppt; erst danach treten die
  `NoneType ... current_state`-Fehler auf. Ursache innerhalb des
  Lifecycle-Abbaus ist ohne Stacktrace noch nicht bestimmt.
- 17:33:56: Im Wiederholungslauf erneut Übergang nach WARTEN sowie
  UR-Meldung `C174A2` (Bedeutung hier nicht verifiziert).
- 17:34:01: IK-Eingang „Command“ verwirft eine EncodedState-Nachricht
  wegen nicht unterstütztem enthaltenen MessageType.
- 17:34:31: Wiederholungslauf wird gestoppt.

Beide Telemetrieversuche lieferten keine verwertbaren Nachrichten. Beim ersten
fehlte die ROS-Python-Umgebung; beim zweiten lief der Prozess, die Datei blieb
aber leer. Die genaue Ursache dieses fehlenden Empfangs ist noch offen.
Ein laufender Recorderprozess ist kein Empfangsnachweis.

Die gespeicherte Regelkette verbindet Object Follower „Zielpose“ mit
Signal Point Attractor „Attractor pose“ und dessen „Output twist“ mit
IK Velocity Controller „Command“. Die wiederverwendeten Topicnamen
`/frame_to_signal/pose` und `/true_signal/value` sind für sich kein Fehler.
Die frühere Empfehlung, sie als Behebung des IK-Fehlers neu zu verdrahten,
ist nicht belegt und wird zurückgenommen. Zusätzliche Sender auf diesen Topics
müssen bei einer Laufzeitprüfung ausgeschlossen werden.

Attractor und IK haben gespeichert jeweils 0,10 m/s als lineares Limit.
Das bestätigt keine effektiven Laufzeitwerte und keine gemessene Geschwindigkeit.
Das beobachtete Hoch-/Herunterfahren passt zum Startpfad mit Transferhöhe 0,49 m
und Beobachtungshöhe 0,45 m; eine Bahn- oder Geschwindigkeitsabnahme fehlt.
Codex hat bei diesen Versuchen keine synthetische Objektquelle gestartet.

Vor dem nächsten Bewegungstest: tatsächlichen Nachrichtenempfang nachweisen,
IK-Nachrichtentyp und wirksame Limits prüfen. Keine weiteren Bewegungsstarts
allein zur Wiederholung des bisherigen Versuchs.
