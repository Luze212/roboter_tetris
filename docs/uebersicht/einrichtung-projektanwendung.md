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

> ### ⚠️ Belichtungsautomatik der Roboterkamera abschalten
>
> `rgb_camera.enable_auto_exposure` untergräbt die Farbmaske von `robot_cam`
> aktiv: Sie überbelichtet das Band, und Überbelichtung frisst genau die
> Sättigung, die die Maske braucht. Am 15.09.2026 gemessen — Automatik aus, bei
> **gleichem Nennwert** `exposure = 166`: Bandsättigung 60 → **118**,
> überbelichtete Pixel auf **0 %**, größte Störkontur 375 842 → **44 287 px**.
>
> ⚠️ Auch diese Parameter exponiert der AICA-RealSense-Block **nicht** — dieselbe
> Lage wie bei `global_time_enabled`. Zur Laufzeit setzbar mit:
>
> ```bash
> ros2 param set <ROBOTERKAMERA-KNOTEN> rgb_camera.enable_auto_exposure false
> ```
>
> Hintergrund und Grenzen: `architektur/robot-cam-befunde.md` §9.3.

> ### ⚠️ Knotennamen nicht annehmen — über `serial_no` prüfen
>
> Welcher RealSense-Knoten welche Kamera ist, **hängt an der Anwendung und wechselt**.
> Im Projekt vom 13.09. war `/realsense_camera_2` die Basiskamera, im Projekt vom
> 15.09. ist es `/realsense_camera`. Wer den Namen aus einer anderen Anwendung
> übernimmt, behandelt die falsche Kamera — und zwar unbemerkt, weil die
> Roboterkamera den Parameter ohnehin richtig stehen hat.
>
> Maßgeblich ist der Serial: **Basiskamera `f1370107` (L515)**, **Roboterkamera
> `241122074842` (D435i)**. Denselben Namen trägt dann auch `camera_node` bei
> `base_kamera`.
>
> ### Zwischenlösung, solange das Image nicht neu gebaut werden kann
>
> Der Fix in `base_cam` ist im Quellcode vorhanden, aber erst nach einem Rebuild
> **und dem anschließenden Neuerzeugen des AICA-Systemabbilds im Launcher** wirksam
> — ein Neustart der Anwendung allein holt das neue Paket nicht nach. Gegenprobe:
> Taucht `camera_node` in der Parameterliste von `base_kamera` auf? Wenn nein,
> läuft noch der alte Komponentenstand. Ist ein
> Rebuild gerade nicht möglich — etwa weil auf derselben AICA-Installation das
> Kalibrierprojekt läuft —, hilft **nach jedem Start der Anwendung** dieser Aufruf:
>
> ```bash
> docker exec -u ros2 -e ROS_DOMAIN_ID=0 $(docker ps -q | head -1) bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 param set <KAMERA-KNOTEN> rgb_camera.global_time_enabled true && ros2 param set <KAMERA-KNOTEN> depth_module.global_time_enabled true'
> ```
>
> Zweimal „Set parameter successful" = hat gegriffen. **Möglichst sofort nach dem
> Start**, weil der Versatz von Beginn an mit rund 4 ms/s wegläuft. Am 15.09.2026
> erprobt: +0,586 s → +0,067 s.

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

## 8. Gemessene Werte des Aufbaus

Diese Werte stammen aus Messungen am Roboter (15.09.2026) und sind beim Anlegen
der Anwendung einzutragen. Herleitung und Unsicherheiten: `architektur/entscheidungen.md`,
Nachtrag 5.

> ⚠️ **Alle Höhen sind Flanschmaße** (`ur_tool0`), nicht TCP-Maße der UR-Steuerung.
> Der Greifer steht nicht im URDF, `robot_state_broadcaster` und IK-Controller
> arbeiten beide am Flansch. Wer hier einen TCP-Wert einträgt, liegt um 215 mm daneben.

| Wert | Zahl | Herkunft |
|---|---|---|
| Bandoberfläche in `world` | **53,6 mm** (±1 mm Ebenheit, ±5 mm systematisch) | B17, drei Antastpunkte |
| Flansch → Backenspitze (geschlossen) | **245 mm** | mit Maßstab gemessen |
| Flansch → Auflagenmitte (Griffpunkt) | **235 mm** | Backenspitze + halbe Auflagenhöhe (B15) |
| `tool_offset_z_m` (TCP der UR-Steuerung) | **0,215 m** | C8 — **nur** zur Umrechnung fremder TCP-Werte |
| Greifhöhe Flansch, 100-mm-Klotz stehend | **≈ 339 mm** | 53,6 + 50 + 235 |
| Transferhöhe Flansch, Referenzklotz | **≈ 445 mm** | D12 |
| Ablagepose Flansch | x = **−316,49** · y = **+476,21** · z = **+419,71** mm | B9 |
| Ablage-Orientierung (w,x,y,z) | 0,006857 · 0,680692 · 0,732524 · −0,004575 | B9 |
| Arbeitsraum Z im Flanschmaß (Anhaltspunkt) | **0,310…0,585 m** | Vorgängerprojekt + 0,215 (B10) |
| Bandrichtung | praktisch die **y-Achse** | M10 |
| Bandebenheit | quer 0,39°, längs 0,01° | M9 |

Allgemeine Greifhöhe: `flansch_z_greifen = 53,6 + klotzhoehe/2 + 235` [mm].

⚠️ Die Ablagepose wurde aus einer Mitschrift gelesen, deren Rohdaten eingefroren
wirkten (Programm auf dem Pendant vermutlich gestoppt). **Einmal bei laufendem
Programm gegenlesen**, bevor sie fest eingetragen wird.

---

## Offen und nicht aus AICA zu beantworten

~~`tool_offset_z_m` (C8)~~ — **erledigt 15.09.2026: 0,215 m.** Aus der
Werkzeugkonfiguration der UR-Steuerung gelesen, über die Vorwärtskinematik
gegengeprüft. Der TCP liegt auf der Flanschachse und ist nicht verdreht.
⚠️ Für Höhen am Aufbau ist **nicht** dieser Wert zu verwenden, sondern die
gemessenen 245 mm (Abschnitt 8).

~~Bandoberflächenhöhe (B17)~~ und ~~Ablagepose (B9)~~ — **erledigt 15.09.2026**,
Werte in Abschnitt 8.

Aus AICA selbst nicht zu beantworten bleiben nur noch die Werte, die am Aufbau
gemessen werden müssen — allen voran **B1** (Bandgeschwindigkeit; die Richtung
ist geklärt) und **C3** (Extrinsik der Basiskamera, läuft beim Kommilitonen).
