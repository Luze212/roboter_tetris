# Datenverträge Robotetris

**Verbindliche Spezifikation aller AICA-Signale zwischen den Komponenten.**
Grundlage für die Implementierung. Änderungen nur hier — und dann in
`roboter_tetris/contracts.py` nachziehen, nie umgekehrt.

Bezug: `entscheidungen.md` Themen 1–5.

---

## Grundregeln

1. **Kopf-Konvention.** Jedes listenartige Signal beginnt mit `[t, n, ...]`.
   `t` = Zeitstempel in Sekunden, `n` = Anzahl Einträge. Länge prüfbar als
   `kopflänge + n · stride`.
2. **Nie leer.** Auch ohne Inhalt wird der Kopf gesendet (`[t, 0]`). Unterscheidet
   "sieht nichts" von "sendet nicht mehr".
3. **`t` nur im Kopf.** Alle Einträge eines Zyklus teilen denselben Zeitpunkt.
4. **SI-Einheiten überall.** Meter, Meter/Sekunde, Radiant, Sekunden.
   Bildverarbeitung rechnet intern in mm, Umrechnung beim Packen.
5. **Frame `world`** (= Roboterbasis) für alle Positionen, sofern nicht anders
   vermerkt.
6. **Statusfelder statt stiller Annahmen.** `valid`, `has_target`, `status`.
7. **Verbraucher gaten auf `t` bzw. `seq`**, bevor sie rechnen. AICA publiziert
   Ausgänge in jedem Schritt — dieselben Daten werden mehrfach gesehen.
8. **Alle Indizes und Schrittweiten in `roboter_tetris/contracts.py`.** Keine
   Komponente indiziert von Hand in ein fremdes Array.
9. **Alle Signale sind `double_array`** (`std_msgs/Float64MultiArray`), außer wo
   ausdrücklich anders angegeben. Ganzzahlen werden als `double` transportiert.

---

## S1 — `objects` · base_cam → vectoring, data_tracker

```
[ t, n, v_band_gemessen,  <obj_0>, <obj_1>, ... ]
   Kopflänge 3                    Stride 9
```

| Kopf | Bedeutung |
|---|---|
| `t` | Zeitstempel des Bildes (`header.stamp`), Sekunden |
| `n` | Anzahl Objekte |
| `v_band_gemessen` | global geschätzte Bandgeschwindigkeit, m/s. **Frame-Größe, kein Objektmerkmal** — der Tracker weist sie allen Tracks gemeinsam zu (`set_global_velocity`). Dient der Kalibrierung (B1) und als Laufkontrolle des Bandes, **nicht** zur Plausibilitätsprüfung einzelner Objekte. |

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | fortlaufend ab 1, pro Programmlauf neu vergeben |
| 1 | `color` | – | 0=Rot 1=Gelb 2=Grün 3=Blau 4=Weiß 5=Schwarz 6=Unbekannt |
| 2 | `x` | m | world |
| 3 | `y` | m | world |
| 4 | `z` | m | world, Oberkante des Blocks |
| 5 | `orientation` | rad | Längsachse, Bereich [0, π) |
| 6 | `length` | m | |
| 7 | `width` | m | |
| 8 | `height` | m | |

**Länge:** `3 + 9·n`

> Änderung gegenüber Ist-Stand: bisher Stride 10 ohne `t` und mit `vy` je Objekt,
> Werte in mm. Neu: Kopf mit `t`, SI-Einheiten, `vy` in den Kopf verschoben.

---

## S2 — `object_position` · robot_cam → object_follower

Feste Länge 6, kein Kopf/Stride (immer höchstens ein Objekt).

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `t` | s | Zeitstempel des Bildes |
| 1 | `valid` | – | 1 = Objekt erkannt **und** Kameraabstand im gültigen Bereich; sonst 0 |
| 2 | `x` | m | **Kameraframe**, nicht world |
| 3 | `y` | m | Kameraframe |
| 4 | `z_band` | m | gemessener Abstand Kamera → Bandoberfläche |
| 5 | `orientation` | rad | [0, π), im Kameraframe |

Bei `valid = 0` sind Felder 2–5 bedeutungslos, **`t` läuft aber weiter**. Damit
unterscheidet der Empfänger "Kamera arbeitet, sieht nichts" (→ Rückfall auf
`w=0`) von "Kamera liefert nicht mehr".

> Die Umrechnung in world macht der `object_follower` über TCP-Ringpuffer und
> Hand-Auge-Kalibrierung (Thema 2/3) — nicht die Kamerakomponente.

---

## S3 — `tracks` · vectoring → priority_handler, data_tracker

```
[ t, n,  <track_0>, <track_1>, ... ]
  Kopf 2            Stride 10
```

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | identisch zur base_cam-ID |
| 1 | `color` | – | durchgereicht |
| 2 | `x` | m | **geglättet**, gültig für `t` im Kopf |
| 3 | `y` | m | geglättet |
| 4 | `z` | m | geglättet |
| 5 | `orientation` | rad | geglättet, [0, π) |
| 6 | `length` | m | geglättet |
| 7 | `width` | m | geglättet |
| 8 | `height` | m | geglättet |
| 9 | `status` | – | siehe Tabelle unten |

**Länge:** `2 + 10·n`

### Statuscodes

| Code | Bedeutung | Folge |
|---|---|---|
| 0 | plausibel, wird verfolgt | Kandidat für `priority_handler` |
| 1 | zu langsam / steht — vermutlich verklemmt oder umgekippt | ausgeschlossen |
| 2 | Sprung / zu schnell — Fehldetektion oder angestoßen | ausgeschlossen |
| 3 | noch zu wenige Messungen, Mittelung nicht eingeschwungen | noch nicht Kandidat |

Code 3 ist kein Fehler, sondern der Normalzustand direkt nach dem Auflegen.
`priority_handler` nimmt ausschließlich Status 0. `data_tracker` und
`interface_streamer` zeigen den Code an, damit **erkennbar ist, warum** ein Block
nicht angefahren wurde.

> Die Position ist auf `t` extrapoliert — der Empfänger rechnet weiter mit
> `p(t') = p + d · v_band · (t' − t)`.

---

## S4 — `target` · priority_handler → object_follower

Feste Länge 12, kein Kopf/Stride (immer genau ein Ziel oder keines).

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `t` | s | Gültigkeitszeitpunkt der Position |
| 1 | `has_target` | – | 1 = Ziel gewählt, 0 = kein Ziel |
| 2 | `id` | – | |
| 3 | `color` | – | |
| 4 | `x` | m | world, gültig für `t` |
| 5 | `y` | m | |
| 6 | `z` | m | Oberkante |
| 7 | `orientation` | rad | [0, π) |
| 8 | `length` | m | |
| 9 | `width` | m | für die Greifer-Vorpositionierung |
| 10 | `height` | m | für die Greifhöhe |
| 11 | `t_rest` | s | geschätzte Restzeit, bis das Objekt die Greifzone verlässt |

Bei `has_target = 0` sind Felder 2–11 bedeutungslos. **Das Zurückziehen des Ziels
ist die Abbruchregel** — wird ein Objekt unerreichbar, gepickt oder unplausibel,
setzt `priority_handler` `has_target = 0` und der Follower bricht ab.

---

## S5 — `not_pickable` · priority_handler → data_tracker

```
[ t, n,  id_0, id_1, ... ]     Stride 1
```
Reine Buchhaltung für die Anzeige. Nicht im Regelpfad.

**Länge:** `2 + n`

---

## S6 — `target_pose` · object_follower → signal_point_attractor

**Typ: `cartesian_state`** (zu bestätigen, A2 — muss dem `attractor`-Eingang
entsprechen, also dem, was `frame_to_signal.pose` heute liefert).

- `reference_frame`: `world`, **explizit gesetzt**
- Inhalt: Zielpose des **Flansches** (nicht des Greifpunkts), einschließlich
  Werkzeugversatz `tool_offset_z_m` und Vorhalt `lead_offset_m`

Ersetzt `frame_to_signal` im Betrieb.

---

## S7 — `picked_id` · object_follower → priority_handler, data_tracker

Feste Länge 3.

| # | Feld | Bemerkung |
|---|---|---|
| 0 | `seq` | zählt bei jedem **abgeschlossenen** Versuch hoch, ab 0 |
| 1 | `id` | betroffene Objekt-ID |
| 2 | `outcome` | 0 = erfolgreich abgelegt · 1 = Fehlgriff · 2 = Objekt verloren |

Verbraucher merken sich die zuletzt gesehene `seq` und reagieren genau einmal.
**Ersetzt das "1 Sekunde lang True"-Muster aus dem Plan** — kein Zeitfenster,
keine Flankenerkennung, kein Verpassen bei Lastspitzen.

---

## S8 — `follower_status` · object_follower → interface_streamer

Feste Länge 7. Reine Diagnose.

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `t` | s | |
| 1 | `state` | – | Zustandscode (Thema 6) |
| 2 | `target_id` | – | 0 = keins |
| 3 | `err_laengs` | m | Regelabweichung in Bandrichtung |
| 4 | `err_quer` | m | Regelabweichung quer |
| 5 | `err_z` | m | Höhenabweichung |
| 6 | `w_wirksam` | – | tatsächlich verwendete Gewichtung 0…1 (nach Rampe und Rückfall) |

`w_wirksam` macht sichtbar, aus welcher Quelle die Zielposition gerade stammt —
der zentrale Wert für euren geplanten Vergleich base_cam ↔ robot_cam.

---

## S9 — Greifersignale

**Bestehend, unverändert:**

| Signal | Richtung | Typ |
|---|---|---|
| `gripper_close` | object_follower → gripper | `Bool` |
| `gripper_change` | (optional) object_follower → gripper | `Int32`, Ziel-Öffnungsweite in mm |

**Neu zu ergänzen:**

| Signal | Richtung | Typ | Bedeutung |
|---|---|---|---|
| `is_closed` | gripper → object_follower | `Bool` | Bewegung abgeschlossen |
| `has_object` | gripper → object_follower | `Bool` | Objekt tatsächlich gefasst |

Die vorhandenen Predicates `is_connected` / `is_object_grasped` bleiben für die
UI erhalten. Der Plan sah nur "Greifer zu" vor — das wäre ein Echo des Eingangs
und als Rückmeldung wertlos. Der Follower wertet `has_object` aus.

`gripper_change` ist optional: Aus `width` (S4, Feld 9) ließe sich der Greifer
vor dem Zugreifen vorpositionieren — schneller und sicherer. Ausbaustufe.

---

## S10 — `world_state` · data_tracker → interface_streamer

```
[ t, n,  <entry_0>, ... ]     Stride 12
```

Felder 0–9 wie `tracks` (S3), zusätzlich:

| # | Feld | Bemerkung |
|---|---|---|
| 10 | `picked` | 1 = erfolgreich abgelegt |
| 11 | `out_of_bounds` | 1 = Greifzone verlassen, ohne gepickt zu werden |

**Länge:** `2 + 12·n`

**Verfallsregel:** Einträge mit `picked = 1` oder `out_of_bounds = 1` verschwinden
nach einer konfigurierbaren Frist aus dem Array. Ohne diese Regel wächst das
Signal über den Programmlauf monoton — bei periodischem Publizieren ein echtes
Problem. Historie gehört ins Log, nicht ins Signal.

---

## `contracts.py` — Aufbau

```python
# Je Signal: Kopflänge, Stride, Feldindizes, pack/unpack.
OBJECTS_HEADER = 3
OBJECTS_STRIDE = 9
OBJ_ID, OBJ_COLOR, OBJ_X, OBJ_Y, OBJ_Z, OBJ_ORI, OBJ_LEN, OBJ_WID, OBJ_HGT = range(9)

def pack_objects(t: float, v_belt: float, objects: list) -> list: ...
def unpack_objects(arr: list) -> tuple[float, float, list]: ...
```

Regeln:
- `unpack_*` prüft die Länge gegen `kopf + n·stride` und wirft bzw. liefert leer
  bei Verstoß — ein Empfänger darf nie auf halben Daten rechnen.
- `unpack_*` rechnet **nicht** um und interpretiert nicht; Extrapolation und
  Transformationen macht der Verbraucher.
- Keine Komponente importiert Feldindizes aus einer anderen Komponente, nur aus
  `contracts.py`.

---

## Offene Punkte in diesem Dokument

| Punkt | Abhängig von |
|---|---|
| Typ von `target_pose` (`cartesian_state` vs. `cartesian_pose`) | A2 |
| Zustandscodes in `follower_status` Feld 1 | Thema 6 |
| Verfallsfrist in `world_state` | Thema 6 |
| Ob `gripper_change` genutzt wird | Ausbaustufe |
