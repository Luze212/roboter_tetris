# Einrichtung der Projektanwendung in AICA

**Stand 14.09.2026.** Was beim Anlegen der AICA-Anwendung für den On-the-fly-Pick
gesetzt werden muss — und warum die Defaults nicht taugen.

Grundlage: die Messungen am Aufbau vom 14.09.2026 und die Auswertung der
Komponentenbeschreibungen im AICA-Image (`v2.0.5-jazzy`, core v5.0.0).

> **Zweck.** Für den Systemtest existiert noch keine Projektanwendung. Die
> vorhandenen Anwendungen (`Following Frame Builde`, `Robot Move Test`,
> `Calibration`, …) sind Teststände. Diese Liste sagt, was aus ihnen übernommen
> werden kann und was neu entschieden werden muss.
>
> **Regel:** Jeder Wert unten, der nicht gesetzt wird, gilt mit seinem Default —
> und mehrere Defaults sind für diese Aufgabe nachweislich falsch.

---

## 1. Pflicht — ohne diese Werte funktioniert es nicht

| Was | Wert | Warum |
|---|---|---|
| **`rgb_camera.global_time_enabled`** der **Basiskamera** | **`true`** | **Kritisch.** Steht er auf `false`, stempelt die L515 in ihrer Hardwareuhr: fremde Epoche (gemessen: Jahr 2006), Drift von **3,9 ms/s** gegen die ROS-Zeit, und nach wenigen Minuten bleibt der Stempel ganz stehen. Dann verwirft das Frame-Gating in `base_cam` jedes Bild nach dem ersten und die Objektliste bleibt leer. Am 14.09. genau so gemessen. |
| **`depth_module.global_time_enabled`** der Basiskamera | **`true`** | dito |
| Beides bei der **Roboterkamera** | `true` | steht dort bereits richtig — beim Neuanlegen nicht verlieren |

> ⚠️ **Der AICA-RealSense-Block exponiert diese Parameter nicht** (er bietet 25,
> diese sind nicht dabei). Sie lassen sich nur zur Laufzeit über den
> ROS-Parameterdienst setzen und sind nach jedem Start der Anwendung wieder weg.
> **Deshalb erzwingt `base_cam` sie selbst:** Parameter `camera_node` auf den
> Node-Namen der Basiskamera setzen (z. B. `/realsense_camera_2`), dann erledigt
> die Komponente es bei jeder Aktivierung. Leer lassen = Funktion aus.

> **Nach dem Setzen prüfen**, nicht annehmen: Ein Neustart verdeckt den Fehler.
> Die Hardwareuhr startet nahe null und wird **einmal** gegen die Rechneruhr
> synchronisiert, der Versatz sieht also direkt nach dem Start klein aus und
> wächst erst danach. Verfahren: `ros_zeit − header_stempel` über mehrere Minuten
> mitloggen (B13). Konstant = gut, wachsend = Parameter hat nicht gewirkt.

---

## 2. Regelung — die Defaults sind für diese Aufgabe zu schwach

| Komponente | Parameter | Default | Anforderung |
|---|---|---|---|
| `SignalPointAttractor` | `linear_gains` | `[1.0]` | **deutlich höher.** Bei K = 1 ist der Vorhalt `v_band/K` die volle Bandgeschwindigkeit (20 cm bei 0,2 m/s) und die Einschwingzeit `3/K` **3 Sekunden** — mehr, als ein Klotz für die Durchquerung der Greifzone braucht. Die Erreichbarkeitsrechnung (P1) kalkuliert mit K ≈ 5. |
| `SignalPointAttractor` | `angular_gains` | `[1.0]` | darf abweichen; zu klein lässt das Handgelenk beim Absenken nachdrehen (F1) |
| `SignalPointAttractor` | `max_linear_velocity` | 0,5 m/s | 2–3 × Bandgeschwindigkeit |
| **`IKVelocityController`** | **`max_linear_velocity`** | **0,25 m/s** | **Die bindende Grenze.** Sie klemmt unabhängig vom Attractor. Wird nur der Attractor angehoben, bleibt die Anhebung wirkungslos. Bei 0,2 m/s Band wären 0,25 m/s nur das 1,25-fache — der Roboter könnte das Band nie einholen. |
| `IKVelocityController` | `pinv_damping` | 0,0 | Reserve gegen explodierende Gelenkgeschwindigkeiten nahe Singularitäten (B11) |
| `SignalPointAttractor` | `linear_precision` | 0,01 m | Schwelle für `is_in_range`; nur für stehende Ziele relevant (`WARTEN`, `ABLEGEN`) |

**`linear_gains` isotrop halten** — ein Vektor mit einem Wert. Drei ungleiche
Werte machen den Vorhalt achsabhängig und bei schräger Bandrichtung zur
Matrixrechnung.

---

## 3. Kameras — aus den Teststand-Anwendungen übernehmbar

| | Roboterkamera | Basiskamera |
|---|---|---|
| Typ | D435i | L515 |
| `serial_no` | `241122074842` | `f1370107` |
| `rgb_camera.profile` | `848x480x30` | `1280x720x30` |
| `depth_module.profile` | `848x480x30` | `640x480x30` |
| `align_depth.enable` | **`true`** | **`true`** |
| `global_time_enabled` | `true` | **`true` (siehe §1)** |

Gemessen am 14.09.: beide Kameras liefern **~29,7 Bilder/s** für Farbe und
aligned Depth — der in Thema 2 angenommene Kameratakt stimmt (**A6 erledigt**).

**Profile immer explizit setzen.** Ohne Angabe gilt ein Treiber-Default, und
`base_cam` klemmt eine zu große ROI **stillschweigend** auf das Bild, ohne
Fehlermeldung. Bei der Basiskamera ist die Tiefe mit 640×480 nativ nur halb so
fein wie das Farbbild und wird fürs Alignment hochskaliert — bekannt, kein Fehler,
aber bei der Beurteilung der Geometrie zu wissen.

---

## 4. Hardware-Interface

| Parameter | Wert in den Teststand-Anwendungen | Anmerkung |
|---|---|---|
| `urdf` | `Universal Robots 10e` | AICA-Standarddefinition. **Der Robotiq-Greifer ist darin nicht enthalten** — der IK-Controller regelt damit den Flansch, der `RobotStateBroadcaster` meldet denselben. Folge: `tool_offset_z_m` im `object_follower` wird gebraucht und ist ungleich null (**A7/A8 erledigt**). |
| `rate` | 500 | Die Doku sprach von 100 Hz. Bei einer Änderung die Begründung der Follower-Rate mitziehen. |
| `robot_ip` | 192.168.96.221 | |
| `tf_publish_frequency` des Broadcasters | 20 (Default) | TF wird publiziert (**A5 erledigt**), ist mit 20 Hz aber zu grob für die zeitrichtige Zuordnung der `robot_cam`-Messung — der eigene TCP-Ringpuffer im Follower bleibt nötig. |

---

## 5. Eigene Komponenten

| Komponente | Rate | Anmerkung |
|---|---|---|
| `base_cam`, `robot_cam` | 100 Hz | Ist-Stand am Teststand: **10 Hz** — damit wird nur jedes dritte Kamerabild verarbeitet. |
| `vectoring`, `priority_handler`, `object_follower` | 100 Hz | |
| `data_tracker` | 10 Hz | |
| `interface_streamer` | 5 Hz | |
| `robotiq_gripper` | ereignisgetrieben | |

**`debug_enable`** bei den Kamerakomponenten nur zur Inbetriebnahme einschalten —
und vorher `debug_image` auf `publish_on_step=False` umgestellt haben
(`entscheidungen.md` Nachtrag 3 / N5), sonst werden die Bilder mit der
Komponentenrate statt der Kamerarate verschickt.

### Kopplung, die leicht übersehen wird

`track_velocity_region_y_min` / `_max` von `base_cam` sind **regelungsrelevant**
und werden gemeinsam mit der Greifzone festgelegt (B19, Nachtrag 3 / N2).
Außerhalb dieser Region koppelt der Tracker die Längsposition statt sie zu messen
und löscht Tracks nicht bei ausbleibender Detektion. Die Greifzone muss innerhalb
liegen.

Ist-Werte aus dem Teststand: Region `−1000 … −500` mm, Tracker-Grenzen
`−1080 … +375` mm. Ein Klotz auf dem Band wurde am 14.09. bei
**x = 814, y = −874 mm** gemessen — die Bandmitte liegt also etwa dort.

---

## 6. Verdrahtung

Die Signalliste steht in `systemgraph.md`. Zwei Punkte, die aus der Prüfung am
14.09. stammen:

- **`target_pose` des `object_follower` ist `cartesian_pose`**, nicht
  `cartesian_state` — das ist der Typ des `attractor`-Eingangs und dessen, was
  `TfToSignal.pose` liefert (**A2 erledigt**).
- **`cartesian_state` geht zusätzlich an den `priority_handler`** — er braucht den
  Anfahrweg für die Erreichbarkeitsprüfung (Nachtrag 3 / N4). Kein Zyklus, das
  Signal kommt aus der Hardware.
- `frame_to_signal` entfällt im Betrieb; der `object_follower` tritt an seine
  Stelle.

---

## 7. Abnahme nach dem Anlegen

Kurz und in dieser Reihenfolge:

1. **Zeitstempel** — `ros_zeit − header_stempel` beider Kameras über ≥ 5 Minuten.
   Beide konstant? (B12/B13)
2. **Bildraten** — ~30 Hz je Kamera für Farbe und aligned Depth? (A6)
3. **`objects` nicht leer**, sobald ein Klotz im Sichtfeld liegt, und die Liste
   bleibt gefüllt (kein Einfrieren nach einigen Minuten).
4. **Debug-Bild** der Basiskamera: `median` nahe `conveyor_z`, flache Objekte wie
   Papier werden **nicht** erkannt, der Klotz schon.
5. **`v_band`** über der Totzone von 30 mm/s, sobald das Band läuft — darunter
   meldet `base_cam` dauerhaft 0 (B1).

---

## Offen und nicht aus AICA zu beantworten

`tool_offset_z_m` (C8) steht in der Werkzeugkonfiguration der UR-Steuerung, nicht
in AICA und nicht im Vorgängerarchiv. Muss dort ausgelesen werden.
