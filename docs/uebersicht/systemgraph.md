# Systemaufbau als AICA-Graph

**Stand 28.09.2026: finaler Build und Kalibrierung der Basiskamera.** Gegriffen wird allein mit der Basiskamera; die
Komponenten der Roboterkamera (`robot_cam`, `robot_cam_2`) liegen im Paket, nichts
nimmt ihr S2 ab (Nachtrag 13 / L22). Grundlage: `architektur/entscheidungen.md` (Themen 1–7, Nachträge 1–13),
`architektur/datenvertraege.md` (S1–S10).

> Die Raten unten sind die am Aufbau gesetzten (Nachtrag 13 / L2, L26): Alle
> Python-Komponenten teilen sich einen Prozess, jede unnötig hohe Rate kostet
> `base_cam` Messrate.

Zeichen: `*` eigene Komponente · `o` AICA-Baustein.
Signaltypen: **D** Datensignal (Zahlenfeld) · **B** Schaltsignal (Bool) ·
**I** Bildsignal · **Z** Zielpose · **R** Roboterzustand

---

## Gesamtgraph

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
 o signal_point_attractor
   │ twist
   ▼
 o ik_velocity_controller ──▶ UR10e
                                │
 o robot_state_broadcaster ◀────┘
   │ R cartesian_state (Flansch)
   └──▶ signal_point_attractor · object_follower · priority_handler
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

**Der Datenfluss ist azyklisch** bis auf eine Rückkante: `picked_id` vom Follower
an den `priority_handler` — ein diskretes Ereignis je Versuch mit laufender Nummer,
kein kontinuierlicher Datenring. `data_tracker` und `interface_streamer` sind
Blätter: Von ihnen führt keine Leitung zurück in den Regelpfad.

---

## Getrennte Anwendung: Kalibrierung der Basiskamera

```
 L515 ─ color, depth, info ─▶ * base_cam_calibration (stufe1) ── Z target_pose ──▶ Signal Point Attractor ─▶ IK ─▶ UR10e
                                 ▲ robot_state (Flansch)          ▲ Frame „Kalibrierstart“ (Knopf 1)
 Kalibrierdatei ─▶ * base_cam_calibration (anzeigen) ── camera_pose ──▶ SignalToTf ─▶ Frame „basiskamera“
 Knöpfe 3/4 ─▶ Greifer zu / auf ─▶ robotiq_gripper (hält das Board)
```

Eigene AICA-Anwendung (`anwendung-kalibrierung-basiskamera.yaml`), Attractor gedrosselt
(0,1 m/s, 0,3 rad/s). Stufe 1 schreibt in dieselbe Kalibrierdatei, die `base_cam` im
Greifgraph liest; die Anzeige folgt ihr. Bedienung: `einrichtung-projektanwendung.md` §10.

---

## Die Komponenten

| Komponente | Rate | Eingänge | Ausgänge | Logik ohne ROS | Anmerkung |
|---|---|---|---|---|---|
| `base_cam` | Kamera | color, depth, info | `objects` (S1), `debug_image` | `vision/*` | Debug-Bild an (für den Streamer); Extrinsik aus der Kalibrierdatei (L27) |
| `robot_cam_2`, `robot_cam` | Kamera | color, depth, info | `object_position` (S2), `debug_image` | `vision/robot_detection*` | **nicht eingebunden** (L22) — im Paket, ohne Abnehmer |
| `vectoring` | 20 Hz | `objects` | `tracks` (S3) | `track_estimation.py` | Bandgeschwindigkeit (Ziel 3) |
| `priority_handler` | 20 Hz | `tracks`, `picked_id`, `robot_state` | `target` (S4), `not_pickable` (S5) | `target_selection.py` | Greifzone = Arbeitsraum auf dem Band (L21) |
| `data_tracker` | 2 Hz | `tracks`, `not_pickable`, `picked_id` | `world_state` (S10) | `world_bookkeeping.py` | nur Anzeige |
| `object_follower` | 50 Hz | `target`, `robot_state`, `gripper_motion_done`, `gripper_has_object` | `target_pose` (S6), `gripper_close`, `picked_id` (S7), `follower_status` (S8) | `follower_logic.py` | Zustandsautomat, Winkel höchstens ±45° (L25) |
| `robotiq_gripper` | 20 Hz | `gripper_close` | `motion_done`, `has_object` (S9) | `GripperMotionState` | |
| `interface_streamer` | 10 Hz (Vortrag, L24) | `debug_image` der Basiskamera, `world_state`, `follower_status` | `interface_image` | `interface_layout.py` | nur Anzeige |
| `base_cam_calibration` | 20 Hz, Anzeige 2 Hz | color, depth, info, `robot_state` | `target_pose` (nur `stufe1`), `camera_pose` (nur `anzeigen`), `debug_image` | `basecam_extrinsics.py`, `calibration_run.py` | **eigene Anwendung**, nicht im Greifgraph (L27) |

Jede eigene Komponente ist eine dünne Schale um ein Modul ohne ROS, das die ganze
Logik trägt und für sich getestet ist. `contracts.py` hält Kopflängen, Strides,
Feldindizes und zwei gemeinsame Regeln (`along_belt`, `AttemptWatcher`); keine
Komponente indiziert von Hand in ein fremdes Array.

**Zur Signalliste:** Feldbelegung und Einheiten stehen vollständig in
`architektur/datenvertraege.md` (S1–S10) — hier bewusst nicht wiederholt.

---

## Der Ablauf eines Griffs

1. **`base_cam`** misst die Klötze (Position, Abmessungen, Farbe) und vergibt IDs.
2. **`vectoring`** schätzt je Klotz die Geschwindigkeit. Ein frisch aufgelegter
   Klotz ist *einschwingend*, bis zwei aufeinanderfolgende Halbsekunden dieselbe
   Geschwindigkeit messen, dann *final*. Aus allen finalen Klötzen entsteht die
   gepoolte **Bandgeschwindigkeit** (Ziel 3). Verlässt ein finaler Klotz das Bild,
   führt `vectoring` ihn mit dieser Geschwindigkeit weiter (*vorhergesagt*,
   Status 4) — die Greifzone liegt hinter dem Bild (Nachtrag 13 / L4, L10).
3. **`priority_handler`** wählt unter den finalen, greifbaren, erreichbaren Klötzen
   den dringendsten und hält ihn fest (Ziel 4). Er rechnet die **Greifebene** —
   bis dorthin muss das Absenken begonnen haben.
4. **`object_follower`** wartet über dem Band, fährt an (wartet am Zonenrand auf
   der Spur des Klotzes), dreht in den Winkel des Klotzes (höchstens ±45°), folgt
   mit Vorhalt, senkt ab, greift mitfahrend, hebt, fährt zur Kiste und öffnet. Die
   Position kommt allein aus der Basiskamera.
5. **`picked_id`** meldet das Ergebnis; der `priority_handler` wählt das nächste
   Ziel, der `data_tracker` bucht mit, der `interface_streamer` zeigt alles.

Zustände des Followers: `WARTEN` → `ANFAHREN` → `FOLGEN` → `ABSENKEN` → `GREIFEN`
→ `HEBEN` → `ABLEGEN` → `LOESEN` → `WARTEN`; dazu `ABBRUCH` (auch der Start).
Ergebnisse (`outcome`): 0 abgelegt · 1 Fehlgriff · 2 verloren · 3 Greifebene
überschritten · 4 vorher abgebrochen.

---

## Bezugssysteme

| Größe | System |
|---|---|
| Alle Positionen in S1, S3, S4, S10, S6 | `world` = Roboterbasis, wie `robot_state_broadcaster` und IK-Controller |
| Roboterpose | **Flansch** `ur_tool0` — der Greifer steht nicht im URDF |
| S2 (Roboterkamera) | Kameraframe — seit L22 ohne Abnehmer |
| Längskoordinate in S4 (Felder 12, 15) | Projektion auf die geschätzte Bandrichtung, `contracts.along_belt` |

**Gemeinsames Bezugssystem (Nachtrag 13 / L6):** Basiskamera und Roboter
rechnen im selben System `world`; Abweichung höchstens 6 mm, auch für 100-mm-Klötze
(Kalibrierdatei `Extrinsics/base_cam_extrinsics.json` mit der Handkalibrierung L6, L27). Greifzone und
Wartebereich liegen außerhalb des Bildes der Basiskamera (L4).

---

## Kopplungen, die man beim Einstellen kennen muss

Werte, die in zwei Komponenten zusammenpassen müssen:

| Wert hier | muss passen zu | Grund |
|---|---|---|
| `lead_time_s` (Follower) | `linear_gains` K des Attractors | Vorhalt ≈ 1/K; eingemessen über `err_laengs` (Nachtrag 9 / G7) — am Roboter 0,24 s bei K = 5 (Nachtrag 13 / L18) |
| `attractor_gain`, `attractor_v_max_mps` (`priority_handler`) | Attractor-Gain; kleineres `max_linear_velocity` aus Attractor und IK-Controller (im Betrieb beide 0,5, L24) | Anfahrzeit in der Erreichbarkeitsprüfung |
| `t_descend_s` (`priority_handler`) | `(observe_z − Greifhöhe) / descend_speed_mps` des Followers | Lage der Greifebene (Nachtrag 10 / J2) |
| `min_graspable_height_m` (`priority_handler`) | `min_grip_height_m` des Followers (Greifhöhe min) | kein Klotz, den der Follower nicht fassen kann — 0,02: flache 25-mm-Klötze greift er an der Untergrenze, geschlossene Backen 6 mm über dem Band (L24, L26) |
| Greifzone `zone_*` (`priority_handler`) | Arbeitsraum `ws_*` des Followers | Zone liegt im Arbeitsraum, seit L21 deckungsgleich auf dem Band; hinter dem Bild der Basiskamera (L4) |
| `expiry_after_done_s` (`data_tracker`) | Dauer Heben + Transfer + Ablegen | sonst kommt `picked` nicht mehr an (Nachtrag 7 / T1) |

Eigentum: Die Greifzone gehört nur dem `priority_handler`, der Arbeitsraum nur dem
Follower. Der Follower bekommt von der Zone genau zwei Zahlen über S4:
`zone_upstream` und die Greifebene.

---

## Was sich gegenüber dem ursprünglichen Plan geändert hat

| Was | Vorher | Jetzt |
|---|---|---|
| `frame_to_signal` | lieferte die Zielpose (Maus-Frame) | entfällt im Betrieb — `object_follower` übernimmt |
| `data_tracker` | tragend, wurde von `vectoring` abgefragt | **Blatt**, nur Diagnose — löst den Zyklus auf |
| Rückkanal | Listenvergleich über drei Komponenten | ein Ereignis `picked_id` mit laufender Nummer |
| Greifer-Rückmeldung | nur Predicates | zwei Bool-Signale `motion_done`, `has_object` |
| **Bandgeschwindigkeit** | kalibrierte Konstante (B1) | **in `vectoring` geschätzt** — je Klotz und gepoolt, reist im S3-Kopf und in S4 mit (Nachtrag 6 / Z2) |
| **Greifebene** | nicht vorhanden | letzte Position, ab der kein Griff mehr beginnt; rechnet der `priority_handler`, reist als S4 Feld 15 zum Follower (Nachtrag 6 / Z11) |
| Greifzone | in zwei Komponenten parametriert | nur im `priority_handler`; Arbeitsraum nur im Follower (Nachtrag 3 / N3) |
| `priority_handler` ← `robot_state` | nicht vorgesehen | ergänzt — die Erreichbarkeit braucht den Anfahrweg (Nachtrag 3 / N4) |
| Roboterkamera | absolute Position gemischt | **nicht eingebunden** — gegriffen wird mit der Basiskamera allein (Nachtrag 6 / Z10, Nachtrag 13 / L22) |
| Abbruch mit Klotz | Greifer öffnen | Klotz wird trotzdem in der Kiste abgelegt (Nachtrag 6 / Z12) |

---

## Vorhalt statt mitfahrendem Bezugsrahmen

Der Vorhalt ist eine Zeit: `lead_time_s` 0,24 s ≈ 1/K, Vorhalt = Bandgeschwindigkeit ·
`lead_time_s`; der `base_frame`-Eingang des Attractors bleibt unverdrahtet. Die
Alternative — ein mitfahrender Bezugsrahmen am Attractor — wird nicht verfolgt:
Der Eingang trägt keine Geschwindigkeit, und der Vorhalt erreicht am Aufbau rund
1 mm Längsfehler (Nachtrag 9 / G7, Nachtrag 13 / L18).
