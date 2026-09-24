# Codex_Systemtest_1 — Verdrahtung für L14

Stand: 23.09.2026. Anleitung für die Änderungen durch den Nutzer in AICA Studio.
Grundlage ist die gespeicherte Anwendung aus der lesenden
[AICA-Prüfung](2026-09-23-aica-codex-systemtest-1-pruefung.md), zuletzt gespeichert
am 23.09.2026 um 13:15:28 UTC. Spätere oder ungespeicherte Änderungen sind in
diesem beobachteten Stand nicht enthalten. Diese Anleitung dokumentiert den
Soll-Aufbau; die beschriebenen Änderungen wurden nicht durch Codex ausgeführt.

## 1. Vorbereiten

Die Anwendung zum Bearbeiten gestoppt lassen. Für das Einfügen, Verdrahten und
Prüfen der gespeicherten Parameter ist kein Bewegungsstart erforderlich.
Beim späteren Start lädt die bisherige Anwendung bereits Hardware, aktiviert
den IK-Controller und aktiviert Attractor sowie Greifer. Der neue Follower
beginnt im Abbruchpfad: ohne gehaltenen Klotz aufrichten/anheben und zur
Beobachtungspose fahren. Auch ohne ausgewähltes Ziel kann damit Bewegung entstehen.

Vor dem Einfügen der Komponenten den aktuellen Workspace bauen und das
AICA-Systemabbild im Launcher erneuern. Bei der Prüfung war der installierte
Stand noch vor L14: Mindestgreifhöhe 0,015 statt 0,021 m und Greifzone
y −0,45 … +0,05 statt −0,22 … +0,40 m. Ein Anwendungsneustart allein aktualisiert
das installierte Paket nicht.

Anschließend laut Fahrplan den `priority_handler` neu einfügen und den bislang
fehlenden `object_follower` einfügen. Beim Ersetzen der Auswahl alle unten
genannten Verbindungen wiederherstellen, auch die zuvor vorhandenen.
Neue Blockinstanzen können andere Namen erhalten; maßgeblich sind Quelle und
Zielport, nicht eine zufällig vergebene Nummer im Instanznamen.

## 2. Welche Blöcke in deiner Anwendung gemeint sind

| Name in der geprüften Anwendung | Funktion / Komponentenklasse | Aktion |
|---|---|---|
| `realsense_camera` — „Base Kamera“ | L515, Serial `f1370107` | für echtes Kamerabild behalten |
| `base_kamera` — „Base Kamera“ | eigene `roboter_tetris::BaseCam` | behalten; nicht mit dem Kameratreiber verwechseln |
| `vectoring` | `roboter_tetris::Vectoring` | behalten |
| `priority_handler` | `roboter_tetris::PriorityHandler` | nach Build neu einfügen und wieder verbinden |
| `object_follower` — „Object Follower“ | `roboter_tetris::ObjectFollower` | neu einfügen; dieser Instanzname wird hier verwendet |
| `signal_point_attractor` | AICA Signal Point Attractor | behalten; Zielquelle ersetzen |
| `hardware` → `robot_state_broadcaster` | Flanschzustand | behalten, zusätzlichen Abzweig zum Follower ziehen |
| `hardware` → `ik_velocity_controller` | Geschwindigkeitsregelung des Arms | behalten |
| `robotiq_gripper` | Robotiq über USB/Modbus | behalten; Befehl ersetzen, Rückmeldungen verbinden |
| `data_tracker` | Buchhaltung | behalten; Abschlussmeldung ergänzen |
| `frame_to_signal` / `true_signal` | alte Pose-/Greifertestquellen | aus dem Follower-Steuerpfad nehmen |
| `realsense_camera_2` | D435i, Serial `241122074842` | für ersten Lauf ohne Roboterkamera nicht laden |
| `roboter_kamera_2_kanten` | `roboter_tetris::RobotCam2` | optionale spätere Korrektur |
| `roboter_kamera` | alte farbbasierte Variante | nicht für den neuen Regelpfad verwenden, aus Startpfad nehmen |
| `interface_streamer` | Diagnosebild | beim ersten Lauf nicht laden; spätere Anschlüsse siehe §7 |

## 3. Zuerst die beiden alten Steuerleitungen ersetzen

| Beobachtete Verbindung | Änderung |
|---|---|
| `frame_to_signal.pose` → `signal_point_attractor.attractor` | alte Verbindung lösen; dort künftig `object_follower.target_pose` anschließen |
| `true_signal.value` → `robotiq_gripper.gripper_close` | alte Verbindung lösen; dort künftig `object_follower.gripper_close` anschließen |

Der gespeicherte Button „Trigger Events Button“ lädt `frame_to_signal`.
„Greifer zu (Testfunktion)“ lädt `true_signal`. Ihre Ladeaktionen für den
Follower-Betrieb entfernen oder die Testbuttons aus dieser Anwendung nehmen.
Die Testquellen dürfen keinen zweiten Befehlsgeber auf den neuen Ziel-/Greifer-
Topics bilden. Der Frame `Steuerung` wird für die Follower-Zielpose nicht benötigt.

## 4. Pflichtverbindungen — Quelle immer links, Eingang rechts

Die folgenden Tabellen bilden den gesamten benötigten Signalpfad ab.
**Behalten** bedeutet: in der untersuchten Anwendung bereits verbunden.
**Neu** bedeutet: damals fehlend. **Ersetzen** bedeutet: alter Eingang aus §3.
Ein Ausgang kann an mehrere Eingänge angeschlossen werden; vorhandene Abzweige
beim Hinzufügen weiterer Empfänger erhalten.

### 4.1 Kameradaten bis zur Zielauswahl

| Nr. | Ausgang der Quelle | Eingang des Empfängers | Aktion |
|---|---|---|---|
| K1 | `realsense_camera.color_image_raw` | `base_kamera.color_image` | behalten |
| K2 | `realsense_camera.color_camera_info` | `base_kamera.color_camera_info` | behalten |
| K3 | `realsense_camera.aligned_depth_to_color_image_raw` | `base_kamera.depth_image` | behalten |
| K4 | `base_kamera.objects` | `vectoring.objects` | behalten für echten Kamerabetrieb; Testvariante siehe §8 |
| K5 | `vectoring.tracks` | `priority_handler.tracks` | behalten / nach Neueinfügen wiederherstellen |
| K6 | `priority_handler.target` | `object_follower.target` | **neu** |

K4 transportiert S1, K5 S3 und K6 S4, jeweils AICA-Typ `double_array`.
`target` wird aus der Auswahl bezogen; rohe `objects` oder `tracks` gehören
nicht direkt an den Follower-Eingang `target`.

### 4.2 Roboterzustand und Bewegung

| Nr. | Ausgang der Quelle | Eingang des Empfängers | Typ | Aktion |
|---|---|---|---|---|
| R1 | `hardware.robot_state_broadcaster.cartesian_state` | `signal_point_attractor.state` | `cartesian_state` | behalten |
| R2 | `hardware.robot_state_broadcaster.cartesian_state` | `priority_handler.robot_state` | `cartesian_state` | behalten / wiederherstellen |
| R3 | `hardware.robot_state_broadcaster.cartesian_state` | `object_follower.robot_state` | `cartesian_state` | **neu**, dritter Abzweig |
| R4 | `object_follower.target_pose` | `signal_point_attractor.attractor` | `cartesian_pose` | **ersetzen**, siehe §3 |
| R5 | `signal_point_attractor.twist` | `hardware.ik_velocity_controller.command` | bestehende Twist-Verbindung | behalten |

R3 liefert die gemessene Flanschpose; R4 liefert die gewünschte Flanschpose.
Beide beziehen sich auf `world` und `ur_tool0`. Den TCP der UR-Steuerung oder
den alten Frame `Steuerung` nicht als Ersatz für R3 verwenden.
**R4 muss in AICA `cartesian_pose` sein**, trotz ähnlicher ROS-Darstellung nicht
`cartesian_state`. Der Follower wird über den Attractor mit dem IK-Controller
verbunden; seine Zielpose geht nicht direkt an dessen `command`.

### 4.3 Greifer und Abschlussereignis

| Nr. | Ausgang der Quelle | Eingang des Empfängers | Typ | Aktion |
|---|---|---|---|---|
| G1 | `object_follower.gripper_close` | `robotiq_gripper.gripper_close` | `bool` | **ersetzen**, siehe §3 |
| G2 | `robotiq_gripper.motion_done` | `object_follower.gripper_motion_done` | `bool` | **neu** |
| G3 | `robotiq_gripper.has_object` | `object_follower.gripper_has_object` | `bool` | **neu** |
| A1 | `object_follower.picked_id` | `priority_handler.picked_id` | `double_array`, S7 | **neu** |
| A2 | `object_follower.picked_id` | `data_tracker.picked_id` | `double_array`, S7 | **neu**, zweiter Abzweig |
| A3 | `vectoring.tracks` | `data_tracker.tracks` | `double_array`, S3 | behalten |
| A4 | `priority_handler.not_pickable` | `data_tracker.not_pickable` | `double_array`, S5 | behalten / wiederherstellen |

`motion_done` meldet das Ende des Öffnens bzw. Schließens; `has_object` meldet
einen tatsächlich gefassten Klotz. Hier die **Signalausgänge** anschließen,
nicht die Greifer-Predicates „Connected“ oder „Object Grasped“.
`gripper_change` am Greifer bleibt für diesen Ablauf unverbunden; der Follower
hat dafür keinen Ausgang. Bei echtem Greifer G2/G3 direkt nutzen, keine
`toggle_signal`-Ersatzwerte aus der virtuellen Testanleitung hinzufügen.

`picked_id` meldet auch abgebrochene Versuche. Deshalb braucht die Auswahl A1,
um das nächste Ziel freizugeben; A2 versorgt die Buchhaltung. Es ist kein
Greifer-Schaltsignal und kein einzelner Integer.

### 4.4 Portbeschriftungen am Object Follower in AICA

| Angezeigter Port | Technischer Name | Verbindung |
|---|---|---|
| Eingang „Roboterzustand“ | `robot_state` | R3 |
| Eingang „Ziel“ | `target` | K6 |
| Eingang „Roboterkamera“ | `object_position` | zunächst frei; optional O4 |
| Eingang „Greifer: Bewegung fertig“ | `gripper_motion_done` | G2 |
| Eingang „Greifer: Klotz gefasst“ | `gripper_has_object` | G3 |
| Ausgang „Zielpose“ | `target_pose` | R4 |
| Ausgang „Greifer schließen“ | `gripper_close` | G1 |
| Ausgang „Gepickt-Meldung“ | `picked_id` | **A1 und A2** |
| Ausgang „Follower-Status“ | `follower_status` | optional D2 oder lesende Diagnose |

Wenn `object_follower` genau so heißt, entstehen typischerweise Topics wie
`/object_follower/target_pose` und `/object_follower/picked_id`. Der beobachtete
Zustandstopic war `/hardware/robot_state_broadcaster/cartesian_state`.
In Studio die Ports verbinden; bei abweichenden Instanznamen keine alten
Topic-Namen blind eintragen.

## 5. Start und Lifecycle zusätzlich verdrahten

Signalverbindungen aus §4 laden oder aktivieren keine Komponente automatisch.
Für den vorgesehenen betriebsbereiten Graphen sind folgende Ereignisse nötig:

| Auslöser | Aktion | Beobachtung / Änderung |
|---|---|---|
| Anwendungsstart `on_start` | `load priority_handler` | **fehlte**, ergänzen |
| `priority_handler.on_load` | Lifecycle `configure priority_handler` | war vorhanden; nach Neueinfügen wieder prüfen |
| `priority_handler.on_configure` | Lifecycle `activate priority_handler` | war vorhanden; nach Neueinfügen wieder prüfen |
| Anwendungsstart `on_start` | `load object_follower` | **neu** |
| `object_follower.on_load` | Lifecycle `configure object_follower` | **neu** |
| `object_follower.on_configure` | Lifecycle `activate object_follower` | **neu** |

Diese Auto-Aktivierung macht einen späteren Anwendungsstart zu einem
Bewegungsstart. Sie erst mit vollständig gesetzten Parametern und für den
vorbereiteten Testlauf verwenden; während der Verdrahtungsarbeit nicht starten.
Der Lifecycle-Fortschritt ist nach dem Start zusätzlich zu prüfen: Eine
vorhandene Ereignisleitung beweist noch kein erfolgreiches `configure`.

Vorhandene Start-/Lifecycle-Verbindungen von `vectoring`, `data_tracker`,
`robotiq_gripper` und `signal_point_attractor` erhalten. Im Kamerabetrieb auch
den L515-Treiber laden und `base_kamera` laden → konfigurieren → aktivieren.
Die Hardware-Verbindungen sind schon vorhanden: `on_start` lädt `hardware`,
deren `on_load` lädt Broadcaster und IK-Controller; beide werden über
`switch_controllers` aktiviert. Controller nicht wie Lifecycle-Komponenten behandeln.

Für den ersten Lauf ohne Roboterkamera `realsense_camera_2`,
`roboter_kamera_2_kanten`, `roboter_kamera` und `interface_streamer` aus der
Start-Ladeliste nehmen. Nur ihre Datenleitungen zu lösen spart die Last des
Ladens bzw. Verarbeitens nicht zuverlässig. Die optionalen Blöcke können zur
späteren Einrichtung im gespeicherten Graphen bleiben.

## 6. Parameter vor dem ersten Start

Zahlen unten mit Dezimalpunkt für die Eingabe. Positionen in Metern,
Winkel mit `_deg` in Grad; Roboter-Zielpositionen sind Flanschpositionen in `world`.

### Object Follower

| Parameter | Einzutragender / zu prüfender Wert | Bedeutung |
|---|---|---|
| `ws_x_min` / `ws_x_max` | `-1.000` / `-0.300` | Pflicht: Arbeitsraum |
| `ws_y_min` / `ws_y_max` | `-0.320` / `0.480` | Pflicht: Arbeitsraum |
| `ws_z_min` / `ws_z_max` | `0.3086` / `0.600` | Pflicht: Arbeitsraum |
| `observe_x` / `observe_y` | `-0.816` / `0.350` | Pflicht: Warte-/Beobachtungspose |
| `observe_z` | `0.450` | Pflicht: Folgehöhe ohne Roboterkamera |
| `observe_yaw_deg` | `90.0` | Pflicht: Orientierung |
| `min_grip_height_m` | `0.021` | L14, alter installierter Default war `0.015` |
| `transfer_height_m` | `0.49` | Transfer- und Abbruchhöhe |
| `belt_surface_z_m` | `0.0536` | Bandoberfläche |
| `flange_to_grip_point_m` | `0.235` | Abstand Flansch → Griffpunkt |
| `descend_speed_mps` | `0.15` | mit Sinkzeit der Auswahl gekoppelt |
| `max_extrapolation_s` / `target_timeout_s` | `0.6` / `1.0` | angepasste Zeitgrenzen |
| `weight_along` / `weight_across` | `0.0` / `0.0` | Roboterkamera-Korrektur zunächst aus |
| `require_robot_cam_for_grasp` | `false` | Kamera ist zunächst keine Greifvoraussetzung |
| `use_block_orientation` | `false` | erster Lauf mit Modus 1 |
| `lead_time_s` | zunächst `0.2` **bei Attractor-Gain 5** | später über `err_laengs` einmessen |

Für den vollständigen Griff zusätzlich die vorhandenen Ablagewerte prüfen:
`place_x = -0.31649`, `place_y = 0.47621`, `place_z = 0.41971`,
`place_yaw_deg = 94.2`. B9 (Gegenprüfung der Ablagepose am Aufbau) bleibt offen.
`gripper_yaw_offset_deg` ist noch nicht eingemessen (D23); den Default nicht als
bestätigten Montagewinkel behandeln.

`timeout_track_s = 10.0` ist die Einstellung **zum Einmessen des Folgens in 4b**,
kein allgemeiner L14-Pflichtwert. Der Paketdefault ist `3.0`.
Eine Verlängerung dieses Timeouts verhindert kein Absenken.
Der Fahrplan fordert für den ersten reinen Folgetest einen hohen `stable_cycles`-
Wert; die konkrete Einstellung ist für den begrenzten Testlauf noch festzulegen.
Der normale Default `10` ist keine Sperre für den Griff. Ein hoher Zählerwert
ist ebenfalls kein eigenständiger sicherer Betriebsmodus „ohne Absenken“.

### Auswahl, Raten und Controller

| Komponente / Parameter | Wert / Kopplung |
|---|---|
| Auswahl `zone_x_min` / `zone_x_max` | `-0.95` / `-0.68` |
| Auswahl `zone_y_min` / `zone_y_max` | `-0.22` / `0.40` |
| Auswahl `t_descend_s` | **`1.2` ausdrücklich setzen**, auch der neue Paketdefault ist `2.0` |
| Auswahl `attractor_gain` | gleich dem tatsächlichen isotropen `linear_gains`-Wert des Attractors; vorgesehener Startwert `5.0` |
| Auswahl `attractor_v_max_mps` | kleineres `max_linear_velocity` aus Attractor und IK |
| `vectoring.rate` / `priority_handler.rate` | `20` Hz / `20` Hz |
| `data_tracker.rate` | `2` Hz, bereits so gespeichert |
| `object_follower.rate` | `100` Hz laut Systemgraph |

Beim gedrosselten Erstlauf den IK auf `0.10` m/s begrenzen; synthetische Ziele
mit Betrag `0.05` m/s. Wenn der Attractor mindestens `0.10` m/s erlaubt, ist
damit `attractor_v_max_mps = 0.10`. Für den späteren Betrieb mit echtem Band
fordert der aktuelle Fahrplan IK mindestens `0.30` m/s; Attractor und Auswahl
dann entsprechend gemeinsam einstellen. Die geprüfte Anwendung speicherte
keine expliziten Gains oder Limits für Attractor/IK, daher diese Werte nicht
aus der bloßen Anwesenheit der Blöcke ableiten.

## 7. Optionale Verbindungen nach dem ersten Lauf

### Roboterkamera — nur die Kantenvariante

| Nr. | Ausgang | Eingang | Beobachteter Stand |
|---|---|---|---|
| O1 | `realsense_camera_2.color_image_raw` | `roboter_kamera_2_kanten.color_image` | vorhanden |
| O2 | `realsense_camera_2.color_camera_info` | `roboter_kamera_2_kanten.color_camera_info` | vorhanden |
| O3 | `realsense_camera_2.aligned_depth_to_color_image_raw` | `roboter_kamera_2_kanten.aligned_depth_image` | vorhanden |
| O4 | `roboter_kamera_2_kanten.object_position` | `object_follower.object_position` | neu, S2 `double_array` |

Beim späteren Zuschalten D435i laden und `roboter_kamera_2_kanten` laden →
konfigurieren → aktivieren. Für die Kantenkomponente fehlten im beobachteten
YAML die Configure-/Activate-Ereignisse. Ihre Erkennung ist noch nicht zuverlässig
abgenommen; O4 zu verbinden ist keine Freigabe, die Kameragewichte zu erhöhen.
Zeitdomäne, Belichtung und Erkennung nach Fahrplan prüfen, bevor 4c aufgeschaltet wird.

### Anzeige — kein Rückkanal zum Regelpfad

| Nr. | Ausgang | Eingang | Aktion |
|---|---|---|---|
| D1 | `data_tracker.world_state` | `interface_streamer.world_state` | bereits vorhanden |
| D2 | `object_follower.follower_status` | `interface_streamer.follower_status` | neu, S8 `double_array` |
| D3 | `base_kamera.debug_image` | `interface_streamer.base_debug_image` | neu, Bildsignal |
| D4 | `roboter_kamera_2_kanten.debug_image` | `interface_streamer.robot_debug_image` | neu, Bildsignal |

Anzeige bei Bedarf laden → konfigurieren → aktivieren; auch dort fehlten im
beobachteten YAML die Configure-/Activate-Ereignisse. `interface_image` in RViz
als Bild anzeigen. Für D3/D4 `debug_enable` an der jeweiligen Kamera aktivieren,
wenn die Bilder benötigt werden; zusätzliche Bildlast beachten. Die Anzeige
ist für Follower und Greifen nicht erforderlich. Ihr Ausgang wird an keinen
Steuereingang zurückgeführt.

## 8. Zielquelle für den ersten synthetischen Lauf

Der erste Lauf laut Fahrplan verwendet `fake_objects.py` anstelle der Basiskamera:

```text
fake_objects.py → vectoring.objects → priority_handler → object_follower
```

Das Skript ist kein zusätzlich einzufügender AICA-Block. Es publiziert S1 auf
das Topic, auf dem `vectoring.objects` lauscht. Im beobachteten Graphen ist das
`/base_kamera/objects`. Für diesen Test `base_kamera` nicht aktiv laden; den
L515-Treiber kann man ebenfalls aus dem Startpfad nehmen. Den Eingangs-Topicnamen
des Vectoring beibehalten, damit das Skript ihn bedienen kann. Es darf dort
nur die synthetische Quelle publizieren, nicht zusätzlich die reale Kamera.

Das vorhandene Skript unterstützt `--topic`, `--velocity`, `--rate` und
`--duration`. Die Bandrichtung ist −y, also entspricht der geplante Betrag
0,05 m/s dem Argument `--velocity -0.05`. Das Skript erst als gesonderten,
vorbereiteten Testschritt starten. Seine Anpassung an reale Latenz bleibt laut
Fahrplan offen; die Verdrahtung allein löst diese Aufgabe nicht.

Beim späteren Wechsel zur realen Basiskamera die synthetische Quelle beenden,
K1–K4 und den Kamera-Startpfad wieder wirksam machen sowie Limits und Auswahl
gemeinsam auf den vorgesehenen Bandbetrieb einstellen.

## 9. Abschlusskontrolle der gespeicherten Anwendung

- [ ] Aktuelles Paket/Systemabbild geladen; L14-Defaults geprüft.
- [ ] Neue Auswahl und Follower vorhanden; alle Pflichtparameter eingetragen.
- [ ] K1–K6 für Kamerabetrieb vollständig, für synthetischen Betrieb §8 umgesetzt.
- [ ] R1–R5 vollständig; Roboterzustand hat drei Empfänger.
- [ ] G1–G3 vollständig; Greifer bekommt seinen Befehl ausschließlich vom Follower.
- [ ] A1–A4 vollständig; `picked_id` hat zwei Empfänger.
- [ ] Alte Frame-/True-Signal-Testpfade getrennt; alte Testbuttons bereinigt.
- [ ] Auswahl und Follower haben die vorgesehenen Load-/Configure-/Activate-Ereignisse.
- [ ] Unbenötigte Roboterkamera-/Anzeige-Blöcke aus dem Startpfad genommen.
- [ ] Gain, Vorhalt, Limits und Auswahlgeschwindigkeit zusammen geprüft.
- [ ] Gewählte Teststufe und deren Einstellungen festgehalten; für reines Folgen
      keine versehentliche Übernahme der normalen Greiffreigabe.
- [ ] Anwendung gespeichert; anschließend gespeicherten Graphen erneut prüfen.

Die Checkliste bestätigt die Konfiguration, nicht die erfolgreiche Bewegung.
Live-Datenfrische, Lifecycle-Erfolg und realer Bewegungsablauf werden erst beim
separaten Test nach dem [Fahrplan](fahrplan-aufbau.md) geprüft.

## Quellen und Vorrang

- [Beobachtungsbericht zu dieser Anwendung](2026-09-23-aica-codex-systemtest-1-pruefung.md).
- [Systemgraph](systemgraph.md), [Einrichtung §§2, 6, 9](einrichtung-projektanwendung.md).
- [Fahrplan, „Weiter am nächsten Termin“](fahrplan-aufbau.md).
- [Entscheidungen, Nachtrag 13 / L14](../architektur/entscheidungen.md) und
  [Datenverträge S1–S10](../architektur/datenvertraege.md) bleiben normativ.
- Portnamen und Anzeigenamen: JSON-Beschreibungen unter
  `source/roboter_tetris/component_descriptions/` im Projektroot.

Bei späteren Änderungen die Anleitung mit dem neu gespeicherten Graphen
abgleichen; die historischen Beobachtungen aus dem Prüfbericht nicht als
unverändert aktuellen Zustand behandeln.
