# Systemaufbau als AICA-Graph

**Endfassung, Stand 13.09.2026.** Ersetzt den bisherigen Arbeitsstand.
Grundlage: `entscheidungen.md` Themen 1–7, `datenvertraege.md`.

Farbcode wie im Word-Dokument:
`▣` eigene Komponente · `▢` AICA-Kernbaustein
Signaltypen: **D** Datensignal (Zahlenfeld) · **B** Schaltsignal (Bool) ·
**I** Bildsignal · **Z** Zielkoordinate · **R** Roboterzustand

---

## Gesamtgraph

```
 ┌───────────────────────┐                    ┌───────────────────────┐
 │ ▢ RealSense Basiscam  │                    │ ▢ RealSense Robotcam  │
 │   color / depth / info│                    │   color / depth / info│
 └───────────┬───────────┘                    └───────────┬───────────┘
             │                                            │
             ▼                                            ▼
 ┌───────────────────────┐                    ┌───────────────────────┐
 │ ▣ base_cam    100 Hz  │                    │ ▣ robot_cam   100 Hz  │
 │                       │                    │  (oder robot_cam_2)   │
 │ D objects             │                    │ D object_position     │
 │ I debug_image         │                    │ I debug_image         │
 └───┬───────────────┬───┘                    └───────────┬───────────┘
     │               │                                    │
     │ D objects     │ D objects                          │
     ▼               │                                    │
 ┌───────────────────┼───┐                                │
 │ ▣ vectoring  100 Hz   │                                │
 │                       │                                │
 │ D tracks              │                                │
 └───┬───────────────┬───┘                                │
     │ D tracks      │ D tracks                           │
     ▼               │                                    │
 ┌───────────────────┼───┐                                │
 │ ▣ priority_handler    │                                │
 │               100 Hz   │                                │
 │ D target              │                                │
 │ D not_pickable ───────┼──────┐                         │
 └───┬───────────────────┘      │                         │
     │ D target                 │                         │
     ▼                          │                         │
 ┌────────────────────────────┐ │                         │
 │ ▣ object_follower   100 Hz │◀┼─────────────────────────┘
 │                            │ │        D object_position
 │  Z target_pose ────────────┼─┼───────────────┐
 │  B gripper_close ──────────┼─┼──────┐        │
 │  D picked_id ──────────────┼─┤      │        │
 │  D follower_status ────────┼─┼──┐   │        │
 └──────────▲───────▲─────────┘ │  │   │        │
            │       │           │  │   │        │
   R cartesian_state│           │  │   │        │
            │  B is_closed      │  │   │        │
            │  B has_object     │  │   │        │
            │       │           │  │   │        │
            │  ┌────┴─────────┐ │  │   │        │
            │  │ ▣ robotiq_   │◀┼──┼───┘        │
            │  │   gripper    │ │  │            │
            │  │  ereignis-   │ │  │            │
            │  │  getrieben   │ │  │            │
            │  └──────────────┘ │  │            │
            │                   │  │            │
            │   ┌───────────────▼──▼──────────┐ │
            │   │ ▣ data_tracker       10 Hz  │ │
            │   │   D world_state ───────────┐│ │
            │   └────────────────────────────┼┘ │
            │                                │  │
            │   ┌────────────────────────────▼─┐│
            │   │ ▣ interface_streamer   5 Hz  ││
            │   │   ← I debug_image ×2         ││
            │   │   ← D follower_status        ││
            │   │   I interface_image → RViz   ││
            │   └──────────────────────────────┘│
            │                                   │
            │   ┌───────────────────────────────▼──┐
            │   │ ▢ signal_point_attractor         │
            │   │   attractor ◀ Z target_pose      │
            │   │   state     ◀ R cartesian_state  │
            │   │   base_frame ◀ (nur Option C)    │
            │   │   twist ─────────────────────┐   │
            │   └──────────────────────────────┼───┘
            │                                  │
            │   ┌──────────────────────────────▼───┐
            │   │ ▢ HARDWARE            100 Hz     │
            │   │   ik_velocity_controller         │
            │   │     command ◀ twist              │
            │   │   robot_state_broadcaster        │
            └───┼──── R cartesian_state ───────────┤
                └──────────────────────────────────┘
```

---

## Signalliste

| Signal | Typ | Von | Nach |
|---|---|---|---|
| `objects` | D | base_cam | vectoring, data_tracker |
| `object_position` | D | robot_cam | object_follower |
| `tracks` | D | vectoring | priority_handler, data_tracker |
| `target` | D | priority_handler | object_follower |
| `not_pickable` | D | priority_handler | data_tracker |
| `target_pose` | **Z** | object_follower | signal_point_attractor |
| `picked_id` | D | object_follower | priority_handler, data_tracker |
| `follower_status` | D | object_follower | interface_streamer |
| `gripper_close` | B | object_follower | robotiq_gripper |
| `is_closed`, `has_object` | B | robotiq_gripper | object_follower |
| `world_state` | D | data_tracker | interface_streamer |
| `debug_image` ×2 | I | base_cam, robot_cam | interface_streamer |
| `interface_image` | I | interface_streamer | RViz |
| `cartesian_state` | **R** | robot_state_broadcaster | object_follower, attractor |
| `twist` | – | attractor | ik_velocity_controller |

---

## Was sich gegenüber dem ursprünglichen Plan geändert hat

| Was | Vorher | Jetzt |
|---|---|---|
| `frame_to_signal` | lieferte die Zielpose (Maus-Frame) | entfällt im Betrieb — `object_follower` übernimmt |
| `data_tracker` | tragend, wurde von `vectoring` abgefragt | **Blatt**, nur Diagnose — löst den Zyklus auf |
| Rückkanal | Listenvergleich über drei Komponenten | ein Ereignis `picked_id` mit laufender Nummer |
| `vectoring` ← `data_tracker` | vorhanden | entfernt |
| Greifer-Rückmeldung | nur Predicates | zwei Bool-Signale |
| Greifzone | in zwei Komponenten parametriert | nur im `priority_handler`; Arbeitsraum nur im `object_follower` |

**Der Datenfluss ist azyklisch.** Die einzige Rückkante ist `picked_id` — ein
diskretes Ereignis pro Pickvorgang, kein kontinuierlicher Datenring.

---

## Zwei Verdrahtungsvarianten

| | `lead_offset_m` | `base_frame` des Attractors |
|---|---|---|
| **Option A** (Primärweg) | `v_band / K` | nicht verdrahtet |
| **Option C** (Ausbaustufe) | `0.0` | mit mitfahrendem Bezugsrahmen verdrahtet |

Der Wechsel ist eine Parameteränderung plus eine Leitung — keine Codeänderung.
Voraussetzung für C ist Test B3.
