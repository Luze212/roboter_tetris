# Systemaufbau als AICA-Graph

Die Komponenten, ihre Verbindungen und Raten, der Ablauf eines Griffs und die
Parameter, die zusammenpassen müssen. Feldbelegung der Signale:
`../architektur/datenvertraege.md`; Begründungen: `../architektur/entscheidungen.md`
(Abschnittsnummern in Klammern).

Zeichen: `*` eigene Komponente · `o` AICA-Baustein.
Signaltypen: **D** Datensignal (Zahlenfeld) · **B** Schaltsignal (Bool) ·
**I** Bildsignal · **Z** Zielpose · **R** Roboterzustand

---

## Greifanwendung

### Regelpfad — vom Bild zur Bewegung

```
 o RealSense L515 (Basiskamera)
   │ color · depth · info
   ▼
 * base_cam
   │ D objects (S1)
   ▼
 * vectoring
   │ D tracks (S3)
   ▼
 * priority_handler ◀──── D picked_id (S7) ───┐
   │ D target (S4)                            │
   ▼                                          │
 * object_follower                            │
   │   │ B gripper_close  ▲ B motion_done,    │
   │   ▼                  │   has_object      │
   │ * robotiq_gripper ───┘                   │
   │                                          │
   │ D picked_id (S7) ────────────────────────┘
   │ Z target_pose (S6)
   ▼
 o Signal Point Attractor
   │ twist
   ▼
 o IK Velocity Controller ──▶ UR10e
                                │
 o Robot State Broadcaster ◀────┘
   │ R cartesian_state (Flansch)
   └──▶ Signal Point Attractor · object_follower · priority_handler
```

### Diagnosepfad — nur Anzeige

```
 D tracks (S3)        vectoring ─────────┐
 D not_pickable (S5)  priority_handler ──┼──▶ * data_tracker ── D world_state (S10) ──┐
 D picked_id (S7)     object_follower ───┘                                          │
 D follower_status (S8)   object_follower ──────────────────────────────────────────┼──▶ * interface_streamer
 I debug_image        base_cam ─────────────────────────────────────────────────────┘        │
                                                                    I interface_image ──▶ RViz
```

Der Datenfluss ist azyklisch bis auf eine Rückkante: `picked_id` vom Follower an den
`priority_handler`, ein diskretes Ereignis je Versuch mit laufender Nummer.
`data_tracker` und `interface_streamer` sind Blätter; von ihnen führt nichts zurück
in den Regelpfad (§10.2).

---

## Kalibrieranwendung (getrennt)

```
 L515 ─ color, depth, info ─▶ * base_cam_calibration (stufe1) ── Z target_pose ──▶ Signal Point Attractor ─▶ IK ─▶ UR10e
                                 ▲ robot_state (Flansch)          ▲ Frame „Kalibrierstart“ (Knopf 1)
 Kalibrierdatei ─▶ * base_cam_calibration (anzeigen) ── camera_pose ──▶ SignalToTf ─▶ Frame „basiskamera“
 Knöpfe 3/4 ─▶ * true_signal / * toggle_signal ─▶ robotiq_gripper (hält das Board)
```

Eigene AICA-Anwendung (`anwendung-kalibrierung-basiskamera.yaml`), Attractor
gedrosselt (0,1 m/s, 0,3 rad/s). Stufe 1 schreibt in dieselbe Kalibrierdatei, die
`base_cam` in der Greifanwendung liest; die Anzeige folgt ihr. `true_signal` und
`toggle_signal` liefern „Greifer zu“ und „Greifer auf“. Bedienung:
`einrichtung-projektanwendung.md` §10, Verfahren: §5.5.

---

## Die Komponenten

| Komponente | Rate | Eingänge | Ausgänge | Logik ohne ROS |
|---|---|---|---|---|
| `base_cam` | 15 Hz | color, depth, info | `objects` (S1), `debug_image` | `vision/detection.py`, `vision/tracker.py`, `vision/color_estimation.py` |
| `vectoring` | 15 Hz | `objects` | `tracks` (S3) | `track_estimation.py` |
| `priority_handler` | 20 Hz | `tracks`, `picked_id`, `robot_state` | `target` (S4), `not_pickable` (S5) | `target_selection.py` |
| `object_follower` | 50 Hz | `target`, `robot_state`, `gripper_motion_done`, `gripper_has_object` | `target_pose` (S6), `gripper_close`, `picked_id` (S7), `follower_status` (S8) | `follower_logic.py` |
| `robotiq_gripper` | 20 Hz | `gripper_close` | `motion_done`, `has_object` (S9) | `GripperTargetLogic`, `GripperMotionState` im selben Modul |
| `data_tracker` | 2 Hz | `tracks`, `not_pickable`, `picked_id` | `world_state` (S10) | `world_bookkeeping.py` |
| `interface_streamer` | 10 Hz | `base_debug_image`, `world_state`, `follower_status` | `interface_image` | `interface_layout.py` |
| `base_cam_calibration` | 20 Hz, anzeigen 2 Hz | color, depth, info, `robot_state` | `target_pose` (stufe1), `camera_pose` (anzeigen), `debug_image` | `basecam_extrinsics.py`, `calibration_run.py` |
| `true_signal`, `toggle_signal` | – | – | `value` | – |

Jede eigene Komponente ist eine dünne Schale um ein Modul ohne ROS, das die Logik
trägt und für sich getestet ist. `contracts.py` hält Kopflängen, Strides,
Feldindizes und zwei gemeinsame Regeln (`along_belt`, `AttemptWatcher`).

Alle Python-Komponenten laufen in **einem** Prozess und teilen sich praktisch einen
Kern; jede unnötig hohe Rate kostet `base_cam` Messrate (§3.3).

---

## Der Ablauf eines Griffs

1. **`base_cam`** misst die Klötze (Position, Abmessungen, Winkel, Farbe) und vergibt
   IDs.
2. **`vectoring`** schätzt je Klotz die Geschwindigkeit. Ein frisch aufgelegter Klotz
   ist *einschwingend*, bis zwei aufeinanderfolgende Halbsekunden dieselbe
   Geschwindigkeit messen, dann *final*. Aus allen finalen, bewegten Klötzen entsteht
   die **Bandgeschwindigkeit**. Verlässt ein finaler Klotz das Bild, führt
   `vectoring` ihn mit ihr weiter (*vorhergesagt*) — die Greifzone liegt hinter dem
   Bild.
3. **`priority_handler`** wählt unter den finalen, greifbaren, erreichbaren Klötzen
   den dringendsten und hält ihn fest. Er rechnet die **Greifebene**, bis zu der das
   Absenken begonnen haben muss.
4. **`object_follower`** fährt aus der Wartepose an (wartet am Zonenanfang auf der
   Spur des Klotzes), dreht in den Winkel des Klotzes (höchstens ±45°), folgt mit
   Vorhalt, senkt ab, greift mitfahrend, hebt, fährt zur Kiste und öffnet.
5. **`picked_id`** meldet das Ergebnis; der `priority_handler` gibt das nächste Ziel
   frei, der `data_tracker` bucht mit, der `interface_streamer` zeigt alles.

Zustände des Followers: `WARTEN` → `ANFAHREN` → `FOLGEN` → `ABSENKEN` → `GREIFEN`
→ `HEBEN` → `ABLEGEN` → `LOESEN` → `WARTEN`; dazu `ABBRUCH`, der auch der Start ist.
Ergebnisse: 0 abgelegt · 1 Fehlgriff · 2 verloren · 3 Greifebene überschritten ·
4 vorher abgebrochen (§8).

---

## Bezugssysteme

| Größe | System |
|---|---|
| alle Positionen (S1, S3, S4, S6, S10) | `world` = `ur_base_link`, wie Robot State Broadcaster und IK-Controller; der UR-Rahmen `base` ist um 180° um z gedreht |
| Roboterpose | **Flansch** `ur_tool0` — der Greifer steht nicht im URDF |
| Längskoordinate in S4 (Felder 12, 15) | Projektion auf die geschätzte Bandrichtung, `contracts.along_belt` |
| Basiskamera | Kalibrierdatei `Extrinsics/base_cam_extrinsics.json` (Handkalibrierung L6), Abweichung zum Roboter höchstens 6 mm |

---

## Kopplungen, die man beim Einstellen kennen muss

| Wert | muss passen zu | Grund |
|---|---|---|
| `lead_time_s` (Follower, 0,24) | `linear_gains` K des Attractors (5) | Vorhalt ≈ 1/K; eingemessen über `err_laengs` (§2.2) |
| `attractor_gain` (5), `attractor_v_max_mps` (0,85) im `priority_handler` | Attractor-Gain; kleineres `max_linear_velocity` aus Attractor und IK-Controller | Anfahrzeit in der Erreichbarkeit (§7.3) |
| `t_descend_s` (`priority_handler`, 0,7) | `(observe_z − Greifhöhe) / descend_speed_mps` des Followers | Lage der Greifebene (§7.6) |
| `min_graspable_height_m` (`priority_handler`, 0,02) | `min_grip_height_m` des Followers (0,011) | kein Klotz, den der Follower nicht fassen kann |
| `min_grip_height_m` (Follower) | `ws_z_min` (Follower, 0,2986) | sonst liegt die Greifhöhe flacher Klötze unter der Arbeitsraumgrenze und das Gate bricht ab (§8.7) |
| Greifzone `zone_*` (`priority_handler`) | Arbeitsraum `ws_*` (Follower) | Zone liegt im Arbeitsraum, deckungsgleich auf dem Band, hinter dem Bild der Basiskamera (§7.1) |
| Arbeitsraum `ws_*` im Follower | `Safety/workspace_bounds.json` | ein Test prüft die Gleichheit (§9.3) |
| `roi_*` von `base_cam` | Greifzone | der Greifer darf nicht ins Bild ragen (§5.2) |
| `expiry_after_done_s` (`data_tracker`) | Dauer Heben + Transfer + Ablegen | sonst kommt `picked` nicht mehr an (§11) |
| `smoothing_window`, `settle_half_window` (`vectoring`) | Rate von `base_cam` | beide zählen Messungen, nicht Zeit |

Eigentum: Die Greifzone gehört nur dem `priority_handler`, der Arbeitsraum nur dem
Follower. Von der Zone bekommt der Follower genau zwei Zahlen über S4:
`zone_upstream` und die Greifebene.
