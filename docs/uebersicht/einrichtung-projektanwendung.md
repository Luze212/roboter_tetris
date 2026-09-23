# Einrichtung der Projektanwendung in AICA

**Stand 23.09.2026** (angelegt 14.09.). Was beim Anlegen der AICA-Anwendung für
den On-the-fly-Pick gesetzt werden muss — und warum die Defaults nicht taugen.
Verdrahtung der neuen Komponenten: §6; Werte für den virtuellen Roboter: §9;
Reihenfolge am Aufbau: `fahrplan-aufbau.md`; Kopplungen zwischen Parametern:
`systemgraph.md`.

> **Seit 23.09.2026 stehen die gemessenen Werte der Basiskamera als Standardwerte
> im Paket** (Kalibrierung, Trackergrenzen, Messregion, `camera_node`,
> Bandhöhe, Tiefenversatz; `entscheidungen.md` Nachtrag 13 / L6, L8). Sie gelten
> nur für **neu eingefügte** Blöcke — ein bestehender Block behält seine
> gespeicherten Werte.

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
| Beides bei der **Roboterkamera** | `true` | steht dort bereits richtig — beim Neuanlegen nicht verlieren. Der Follower ordnet ihre Bilder über die Bildzeit seinem Ringpuffer zu (B24) |
| **`object_follower`: `ws_*`, `observe_*`** | ohne Default | Arbeitsraum (B10) und Beobachtungspose (B8) — ohne sie scheitert `configure` mit einer Liste der fehlenden. Vorschläge für den virtuellen Roboter: §9 |

> ⚠️ **Der AICA-RealSense-Block exponiert diese Parameter nicht** (er bietet 25,
> diese sind nicht dabei). Sie lassen sich nur zur Laufzeit über den
> ROS-Parameterdienst setzen und sind nach jedem Start der Anwendung wieder weg.
> **Deshalb erzwingt `base_cam` sie selbst:** Parameter `camera_node` auf den
> Node-Namen der Basiskamera setzen, dann erledigt die Komponente es bei jeder
> Aktivierung. Standardwert seit 23.09.2026: `/realsense_camera` (so heißt die
> Basiskamera in der aktuellen Anwendung — über `serial_no` prüfen, s. u.).
> Leer = Funktion aus.

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
| `SignalPointAttractor` | `max_linear_velocity` | 0,5 m/s | 2–3 × Bandgeschwindigkeit — bei gemessenen **≈ 0,13 m/s** (Stoppuhr, Nachtrag 13 / L7) also 0,26–0,39 m/s |
| **`IKVelocityController`** | **`max_linear_velocity`** | **0,25 m/s** | **Die bindende Grenze.** Sie klemmt unabhängig vom Attractor. Wird nur der Attractor angehoben, bleibt die Anhebung wirkungslos. Beim gemessenen Band (≈ 0,13 m/s) sind 0,25 m/s knapp das Doppelte; für das Folgen mit laufendem Band auf ≥ 0,30 m/s. ⚠️ Die Sicherheitsregel „erster Lauf mit 0,10 m/s“ (Fahrplan §2) holt ein 0,13-m/s-Band **nie** ein — gedrosselt nur mit `fake_objects.py` bei kleiner Geschwindigkeit fahren. |
| `IKVelocityController` | `pinv_damping` | 0,0 | Reserve gegen explodierende Gelenkgeschwindigkeiten nahe Singularitäten (B11) |
| `SignalPointAttractor` | `linear_precision` | 0,01 m | Schwelle für `is_in_range`; nur für stehende Ziele relevant (`WARTEN`, `ABLEGEN`) |

**`linear_gains` isotrop halten** — ein Vektor mit einem Wert. Drei ungleiche
Werte machen den Vorhalt achsabhängig und bei schräger Bandrichtung zur
Matrixrechnung.

**Im `priority_handler` dieselben Werte eintragen:** `attractor_gain` = der Wert
aus `linear_gains`, `attractor_v_max_mps` = der **kleinere** der beiden
`max_linear_velocity`. Er rechnet damit die Anfahrzeit; passen die Werte nicht,
wählt er Klötze, die nicht zu schaffen sind, oder verwirft erreichbare. Defaults:
5,0 und 0,25.

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
| `urdf` | `Universal Robots 10e` | AICA-Standarddefinition. **Der Robotiq-Greifer ist darin nicht enthalten** — der IK-Controller regelt damit den Flansch, der `RobotStateBroadcaster` meldet denselben. Folge: `flange_to_grip_point_m` (0,235 m, Nachtrag 6 / Z7) im `object_follower` wird gebraucht und ist ungleich null (**A7/A8 erledigt**). |
| `rate` | 500 | Die Doku sprach von 100 Hz. Bei einer Änderung die Begründung der Follower-Rate mitziehen. |
| `robot_ip` | 192.168.96.221 | |
| `tf_publish_frequency` des Broadcasters | 20 (Default) | TF wird publiziert (**A5 erledigt**), ist mit 20 Hz aber zu grob für die zeitrichtige Zuordnung der `robot_cam`-Messung — der eigene TCP-Ringpuffer im Follower bleibt nötig. |

---

## 5. Eigene Komponenten

> **`rate` am Block setzen.** Der Parameter ist von `LifecycleComponent` geerbt
> (Default **10 Hz**) und wird nur beim Erzeugen gelesen — eine Änderung wirkt erst
> nach Neuladen. Ohne Eintrag läuft jede eigene Komponente mit 10 Hz, auch der
> Follower. ⚠️ **Nicht** zusätzlich in die `component_descriptions` eintragen: Das
> Duplikat lässt die Oberfläche bei jedem Klick ein weiteres Rate-Feld anlegen
> (`entscheidungen.md` Nachtrag 12 / K2).

> ⚠️ **Alle Python-Komponenten laufen in einem einzigen Prozess** und teilen sich
> wegen der GIL praktisch einen Kern (Nachtrag 13 / L2). Jede weitere Komponente
> und jede unnötig hohe `rate` kostet `base_cam` Messrate — gemessen 2,8 statt
> 7,1 Messungen/s. **Nicht gebrauchte Blöcke aus der Anwendung nehmen**; auch
> unkonfigurierte Kamerablöcke empfangen ihre Bilder.

| Komponente | Rate | Anmerkung |
|---|---|---|
| `base_cam`, `robot_cam_2` | 10 Hz (Default); über 30 Hz sinnlos | `base_cam` schafft damit ~7 Messungen/s (23.09.). Nur `robot_cam_2` in die Anwendung, nicht `robot_cam` (Nachtrag 13 / L5) |
| `vectoring`, `priority_handler` | **20 Hz** | Mehr bringt nichts, solange < 10 Messungen/s ankommen. Einschwing-Halbfenster in `vectoring` = **5** (Standardwert seit 23.09.; zählt Messungen) |
| `object_follower` | 100 Hz, bei Rechenzeitmangel 50 Hz | glatte Zielpose für den Attractor (Thema 2) |
| `data_tracker` | 2 Hz | nur Anzeige |
| `interface_streamer` | 5 Hz, nur bei Bedarf in der Anwendung | Bildverarbeitung, kostet Messrate |
| `robotiq_gripper` | ereignisgetrieben | |

**`debug_enable`** bei den Kamerakomponenten nur zur Inbetriebnahme einschalten —
und vorher `debug_image` auf `publish_on_step=False` umgestellt haben
(`entscheidungen.md` Nachtrag 3 / N5), sonst werden die Bilder mit der
Komponentenrate statt der Kamerarate verschickt.

### Kopplung, die leicht übersehen wird

`track_velocity_region_y_min` / `_max` und `track_min_y_mm` / `track_max_y_mm` von
`base_cam` liegen in `world` (Standardwerte seit 23.09.2026): Messregion
**500 … 1000 mm**, Tracker-Grenzen **−375 … +1080 mm** = das Band von Rolle zu
Rolle. Das Bild der Basiskamera deckt y ≈ 460 … 1030 mm ab.

⚠️ **Geändert 23.09.2026 (Nachtrag 13 / L4):** Die Greifzone liegt **außerhalb**
des Bildes der Basiskamera (etwa y 0,30 … −0,30), damit der Greifer nicht als
Klotz erkannt wird. Die frühere Regel „Greifzone innerhalb der Messregion“ (N2)
ist damit aufgegeben. Ihr Grund — ein verschwundenes Ziel in der Zone schnell zu
bemerken — geht auf die Roboterkamera über; hinter dem Bild führt `base_cam` die
Tracks nur rechnerisch weiter (offen: Nachtrag 13 / L1). Die Greifzone endet vor
dem Bandende bei y = −0,375.

> Früher kam ein zweiter Grund hinzu: Außerhalb der Region *rechnete* der Tracker
> die Längsposition, statt sie zu messen. Das entfällt mit Umsetzungsplan 2.4
> (Nachtrag 6 / Z4).

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
- **`interface_streamer`:** `base_debug_image` ← `debug_image` von `base_cam`,
  `robot_debug_image` ← `debug_image` von `robot_cam`, `world_state` ←
  `data_tracker`, `follower_status` ← `object_follower`; `interface_image` → RViz.
  Die Debug-Bilder kommen nur mit `debug_enable` in den Kamerakomponenten.
  Rate 5 Hz — bei Rechenzeitmangel zuerst senken (Spec).
- **`object_follower`, Stufen 4a/4b:** `robot_state` ← `cartesian_state` des
  `robot_state_broadcaster`; `target` ← `target` des `priority_handler`;
  `target_pose` → `attractor` des Signal Point Attractors; `gripper_close` →
  `robotiq_gripper` (bis 4d immer offen); `picked_id` → `priority_handler` und
  `data_tracker`. Die
  **Pflichtparameter** `ws_*` und `observe_*` haben keinen Default — ohne sie lässt
  sich die Komponente nicht konfigurieren (Nachtrag 8 / F3). Vorschläge für den
  virtuellen Roboter: Abschnitt 9.

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
| Flansch → Auflagenmitte (Griffpunkt) = **`flange_to_grip_point_m`** | **235 mm** | Backenspitze − halbe Auflagenhöhe (B15). **Das ist der Follower-Parameter** (Nachtrag 6 / Z7) |
| TCP der UR-Steuerung | 215 mm | C8 — **kein Parameter unserer Kette**, nur zur Umrechnung fremder TCP-Werte. ⚠️ Stand hier als `tool_offset_z_m = 0,215` — falsch, hätte flache Klötze ins Band gefahren |
| Greifhöhe Flansch, 100-mm-Klotz stehend | **≈ 339 mm** | 53,6 + 50 + 235 |
| Transferhöhe Flansch, Referenzklotz | **≈ 490 mm** — `transfer_height_m` | D12; vorher 445, ohne den gehaltenen Klotz (Nachtrag 10 / J1) |
| Ablagepose Flansch | x = **−316,49** · y = **+476,21** · z = **+419,71** mm | B9 |
| Ablage-Orientierung (w,x,y,z) | 0,006857 · 0,680692 · 0,732524 · −0,004575 | B9 |
| Arbeitsraum Z im Flanschmaß (Anhaltspunkt) | **0,310…0,585 m** | Vorgängerprojekt + 0,215 (B10) |
| Bandrichtung | praktisch die **y-Achse** | M10 |
| Bandebenheit | quer 0,39°, längs 0,01° | M9 |
| Band von Rolle zu Rolle in `world` | y ≈ **+1,08 … −0,375 m** (~1,5 m) | Nachtrag 13 / L7 |
| Bandgeschwindigkeit (Stoppuhr, nur Gegenprobe) | **≈ 0,13 m/s** | Nachtrag 13 / L7 |
| **Extrinsik Basiskamera** (`cal_x`, `_y`, `_z`, `_roll`, `_pitch`, `_yaw`) | **−0,7787 · 0,7934 · 0,9163 · 179,46° · 0,45° · 179,76°** (in `world`) | Nachtrag 13 / L6, Standardwert; Übergang bis C3 |
| `belt_surface_z_mm` / `top_depth_bias_mm` (`base_cam`) | **53,6 / 11,5** | B17 / Nachtrag 13 / L6, Standardwerte |
| Bild der Basiskamera | y ≈ 0,46 … 1,03 m | Nachtrag 13 / L7 |

Allgemeine Greifhöhe: `flansch_z_greifen = 53,6 + max(klotzhoehe/2, 15) + 235` [mm].
Die Untergrenze 15 mm (`min_grip_height_m`, 5 mm Luft) schützt das Band bei flachen
Klötzen; darunter gilt ein Klotz als nicht greifbar (B15). Vorher fehlte das
`max()` in dieser Formel.

⚠️ Die Ablagepose wurde aus einer Mitschrift gelesen, deren Rohdaten eingefroren
wirkten (Programm auf dem Pendant vermutlich gestoppt). **Einmal bei laufendem
Programm gegenlesen**, bevor sie fest eingetragen wird.

---

## 9. Vorläufige Werte für den virtuellen Roboter (Stufen 4a–4c)

⚠️ **Nur für den virtuellen Roboter.** Arbeitsraum (B10) und Beobachtungspose (B8)
sind nicht festgelegt, deshalb haben die Parameter keinen Default. Diese Werte
reichen, um die Stufen am virtuellen Roboter zu fahren. Am echten Roboter gelten sie
**nicht**, bis B10 nach `Safety/README.md` dokumentiert ist.

| Parameter | Vorschlag | Herleitung |
|---|---|---|
| `ws_x_min` / `ws_x_max` | −1,10 / −0,20 m | Band bei x = −0,70 … −0,93 (M9), Ablagepose x = −0,316 (B9) |
| `ws_y_min` / `ws_y_max` | −0,60 / +0,60 m | Zonen-Platzhalter −0,45 … +0,05, Ablagepose y = +0,476 |
| `ws_z_min` / `ws_z_max` | 0,30 / 0,80 m | Greifhöhe eines 30-mm-Klotzes 0,304 m im Flanschmaß; Freihöhe 0,49 |
| `observe_x` / `observe_y` | −0,80 / −0,10 m | über der Bandmitte, im Zonen-Platzhalter |
| `observe_z` | 0,60 m | geschätzt; B8 legt sie an der Roboterkamera fest |
| `observe_yaw_deg` | 90° | nahe der geteachten Ablagepose (94°), damit das Handgelenk wenig dreht |

Die Werte stammen aus denselben Zahlen wie die Tests (`test_follower_logic.py`).

**Für Stufe 4d zusätzlich:** Greifer-Rückmeldung von Hand über `toggle_signal` an
`gripper_motion_done` und `gripper_has_object`, solange der echte Greifer fehlt.
Die Ablagepose steht als Default aus B9 bereit. `weight_along`/`weight_across`
bleiben auf 0 (Basiskamera allein, Z10). `gripper_yaw_offset_deg` erst nach D23.
Mit der vorgeschlagenen Beobachtungshöhe 0,60 m passt `t_descend_s` = 2,0 im
`priority_handler` zu `descend_speed_mps` = 0,15 (Nachtrag 10 / J2).

**Für Stufe 4c:** `object_position` von `robot_cam` an den Follower, Gewichte
schrittweise auf 1. `w_wirksam` in `follower_status` zeigt, ob die Kamera wirkt;
bleibt es 0, meldet das Log den Grund (B24: Zeitdomäne, R4: anderer Klotz).

**Für Stufe 4b zusätzlich:** `timeout_track_s` auf 10 s, damit der Roboter der
ganzen Zone folgt (Nachtrag 9 / G4). Als Zielquelle `fake_objects.py` →
`vectoring` → `priority_handler`. Seit B23 erledigt ist (Nachtrag 13 / L6), darf
danach auch die Basiskamera die Zielquelle sein.
`err_laengs` in `follower_status` mitlesen: Am virtuellen Roboter sollte es mit
`lead_time_s` = 1/K im Mittel null sein — nur wenn `linear_gains` des Attractors
wirklich K ist.

---

## Offen und nicht aus AICA zu beantworten

~~Versatz Flansch → Griffpunkt (C8)~~ — **erledigt, korrigiert 21.09.2026:
`flange_to_grip_point_m` = 0,235 m** (Nachtrag 6 / Z7). ⚠️ Hier stand zuvor
„`tool_offset_z_m` — erledigt: 0,215 m". Die 215 mm sind der TCP der UR-Steuerung,
über die Vorwärtskinematik gegengeprüft, auf der Flanschachse und nicht verdreht —
aber **nicht** der Abstand zum Griffpunkt. Für Höhen am Aufbau gelten die gemessenen
245 mm zur Backenspitze, für die Zielpose die 235 mm zum Griffpunkt.

~~Bandoberflächenhöhe (B17)~~ und ~~Ablagepose (B9)~~ — **erledigt 15.09.2026**,
Werte in Abschnitt 8.

Aus AICA selbst nicht zu beantworten bleiben nur noch die Werte, die am Aufbau
gemessen werden müssen — allen voran **B21** (misst der Tracker auch außerhalb der
alten Region sauber?) und **C3** (Extrinsik der Basiskamera, läuft beim
Kommilitonen; bis dahin gilt die Übergangskalibrierung aus §8). **B1** ist keine Voraussetzung mehr: Die Bandgeschwindigkeit wird
geschätzt, B1 prüft den Schätzer nur gegen (Nachtrag 6 / Z2).
