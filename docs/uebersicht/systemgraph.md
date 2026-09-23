# Systemaufbau als AICA-Graph

**Stand 23.09.2026 — alle Komponenten gebaut** und in AICA geladen; Datenpfad bis
`data_tracker` am Aufbau geprüft, Follower und Roboterkamera noch nicht gefahren.
Grundlage: `architektur/entscheidungen.md` (Themen 1–7, Nachträge 1–13),
`architektur/datenvertraege.md` (S1–S10).

> Die Raten unten sind die am Aufbau gesetzten (Nachtrag 13 / L2): Alle
> Python-Komponenten teilen sich einen Prozess, jede unnötig hohe Rate kostet
> `base_cam` Messrate.

Zeichen: `▣` eigene Komponente · `▢` AICA-Baustein.
Signaltypen: **D** Datensignal (Zahlenfeld) · **B** Schaltsignal (Bool) ·
**I** Bildsignal · **Z** Zielpose · **R** Roboterzustand

---

## Gesamtgraph

### Regelpfad — vom Bild zur Bewegung

```
 ▢ RealSense L515 (Basiskamera)               ▢ RealSense D435i (Roboterkamera)
   │ color · depth · info                       │ color · depth · info
   ▼                                            ▼
 ▣ base_cam                                   ▣ robot_cam  (oder robot_cam_2)
   │ D objects (S1)                             │ D object_position (S2)
   ▼                                            │
 ▣ vectoring                                    │
   │ D tracks (S3)                              │
   ▼                                            │
 ▣ priority_handler ◀──── D picked_id (S7) ───┐ │
   │ D target (S4)                            │ │
   ▼                                          │ │
 ▣ object_follower ◀──────────────────────────┼─┘
   │   │ B gripper_close  ▲ B motion_done,    │
   │   ▼                  │   has_object      │
   │ ▣ robotiq_gripper ───┘                   │
   │                                          │
   │ D picked_id (S7) ────────────────────────┘
   │ Z target_pose (S6)
   ▼
 ▢ signal_point_attractor
   │ twist
   ▼
 ▢ ik_velocity_controller ──▶ UR10e
                                │
 ▢ robot_state_broadcaster ◀────┘
   │ R cartesian_state (Flansch)
   └──▶ signal_point_attractor · object_follower · priority_handler
```

### Diagnosepfad — nur Anzeige

```
 D tracks (S3)        vectoring ─────────┐
 D not_pickable (S5)  priority_handler ──┼──▶ ▣ data_tracker ── D world_state (S10) ──┐
 D picked_id (S7)     object_follower ───┘                                          │
 D follower_status (S8)   object_follower ──────────────────────────────────────────┼──▶ ▣ interface_streamer
 I debug_image        base_cam, robot_cam ──────────────────────────────────────────┘        │
                                                                    I interface_image ──▶ RViz
```

**Der Datenfluss ist azyklisch** bis auf eine Rückkante: `picked_id` vom Follower
an den `priority_handler` — ein diskretes Ereignis je Versuch mit laufender Nummer,
kein kontinuierlicher Datenring. `data_tracker` und `interface_streamer` sind
Blätter: Von ihnen führt keine Leitung zurück in den Regelpfad.

---

## Die Komponenten

| Komponente | Rate | Eingänge | Ausgänge | Logik ohne ROS | Stand |
|---|---|---|---|---|---|
| `base_cam` | Kamera | color, depth, info | `objects` (S1), `debug_image` | `vision/*` | läuft am Aufbau; S1 und Tracker-Eingriff 2.4 neu |
| `robot_cam_2` (`robot_cam` nicht mehr) | Kamera | color, depth, info | `object_position` (S2), `debug_image` | `vision/robot_detection*` | Banddistanz aus dem Bildmedian, am Aufbau offen (B6) |
| `vectoring` | 20 Hz | `objects` | `tracks` (S3) | `track_estimation.py` | gebaut |
| `priority_handler` | 20 Hz | `tracks`, `picked_id`, `robot_state` | `target` (S4), `not_pickable` (S5) | `target_selection.py` | gebaut, Greifzone festgelegt (B19) |
| `data_tracker` | 2 Hz | `tracks`, `not_pickable`, `picked_id` | `world_state` (S10) | `world_bookkeeping.py` | gebaut |
| `object_follower` | 100 Hz | `target`, `object_position`, `robot_state`, `gripper_motion_done`, `gripper_has_object` | `target_pose` (S6), `gripper_close`, `picked_id` (S7), `follower_status` (S8) | `follower_logic.py` | gebaut, alle vier Stufen |
| `robotiq_gripper` | ereignisgetrieben | `gripper_close` | `motion_done`, `has_object` (S9) | `GripperMotionState` | läuft am Aufbau; zwei Ausgänge neu |
| `interface_streamer` | 5 Hz | 2 × `debug_image`, `world_state`, `follower_status` | `interface_image` | `interface_layout.py` | gebaut |

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
   der Spur des Klotzes), folgt mit Vorhalt, senkt ab, greift mitfahrend, hebt,
   fährt zur Kiste und öffnet. Die Roboterkamera korrigiert dabei optional die
   Position der Basiskamera.
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
| S2 (Roboterkamera) | Kameraframe; der Follower rechnet über Ringpuffer und Hand-Auge (Flansch → Kamera) nach `world` |
| Längskoordinate in S4 (Felder 12, 15) | Projektion auf die geschätzte Bandrichtung, `contracts.along_belt` |

✅ **B23 erledigt (23.09.2026, Nachtrag 13 / L6):** Basiskamera und Roboter
rechnen im selben System `world`; Abweichung ≤ 6 mm auch für 100-mm-Klötze
(Übergangskalibrierung bis C3). Greifzone und Wartebereich liegen außerhalb des
Bildes der Basiskamera (L4).

---

## Kopplungen, die man beim Einstellen kennen muss

Werte, die in zwei Komponenten zusammenpassen müssen:

| Wert hier | muss passen zu | Grund |
|---|---|---|
| `lead_time_s` (Follower) | `linear_gains` K des Attractors | Vorhalt ≈ 1/K; eingemessen über `err_laengs` (Nachtrag 9 / G7) |
| `attractor_gain`, `attractor_v_max_mps` (`priority_handler`) | Attractor-Gain; kleineres `max_linear_velocity` aus Attractor und IK-Controller | Anfahrzeit in der Erreichbarkeitsprüfung |
| `t_descend_s` (`priority_handler`) | `(observe_z − Greifhöhe) / descend_speed_mps` des Followers | Lage der Greifebene (Nachtrag 10 / J2) |
| `min_graspable_height_m` (`priority_handler`) | `2 · min_grip_height_m` des Followers | kein Klotz, den der Follower nicht fassen kann |
| Greifzone `zone_*` (`priority_handler`) | Messregion von `base_cam`; Arbeitsraum `ws_*` des Followers | Zone liegt in der Region und im Arbeitsraum (B19, N2) |
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
| Roboterkamera | absolute Position gemischt | gefilterte **Korrektur** auf die Vorhersage, je Achse gewichtet; Greifen geht auch ohne (Nachtrag 6 / Z10) |
| Abbruch mit Klotz | Greifer öffnen | Klotz wird trotzdem in der Kiste abgelegt (Nachtrag 6 / Z12) |

---

## Zwei Verdrahtungsvarianten

| | `lead_time_s` | `base_frame` des Attractors |
|---|---|---|
| **Option A** (Primärweg) | `≈ 1/K` (Zeit; Vorhalt = `v · lead_time_s`) | nicht verdrahtet |
| **Option C** (Ausbaustufe) | `0.0` | mit mitfahrendem Bezugsrahmen verdrahtet |

Der Wechsel ist eine Parameteränderung plus eine Leitung — keine Codeänderung.
Voraussetzung für C ist Test B3.
