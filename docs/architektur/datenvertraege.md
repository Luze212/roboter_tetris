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
| `v_band_gemessen` | global geschätzte Bandgeschwindigkeit, m/s. **Frame-Größe, kein Objektmerkmal** — der Tracker weist sie allen Tracks gemeinsam zu (`set_global_velocity`). Dient der Kalibrierung (B1) und als Laufkontrolle des Bandes, **nicht** zur Plausibilitätsprüfung einzelner Objekte. ⚠️ Drei Eigenheiten der Quelle, siehe unten. |

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | fortlaufend ab 1, pro Programmlauf neu vergeben |
| 1 | `color` | – | 0=Rot 1=Gelb 2=Grün 3=Blau 4=Weiß 5=Schwarz 6=Unbekannt |
| 2 | `x` | m | world |
| 3 | `y` | m | world |
| 4 | `z` | m | world, Oberkante des Blocks — ⚠️ Semantik am Code nicht nachvollzogen, siehe unten |
| 5 | `orientation` | rad | Längsachse, Bereich [0, π) |
| 6 | `length` | m | |
| 7 | `width` | m | |
| 8 | `height` | m | |

**Länge:** `3 + 9·n`

> Änderung gegenüber Ist-Stand: bisher Stride 10 ohne `t` und mit `vy` je Objekt,
> Werte in mm. Neu: Kopf mit `t`, SI-Einheiten, `vy` in den Kopf verschoben.

### ⚠️ Drei Eigenheiten von `v_band_gemessen`

Der Wert stammt aus dem Tracker von `base_cam` (Port des C++-Originals) und ist
nicht so allgemein, wie der Name nahelegt:

1. **Es ist die y-Komponente, nicht der Betrag.** Der Tracker kennt nur Bewegung
   entlang der Roboter-y-Achse. Bei einer Bandrichtung wenige Grad daneben liegt
   der Unterschied unter 1 % — wer den Wert aber als Kalibrierquelle für B1 nimmt,
   erhält systematisch etwas zu wenig. Für B1 ist die Auswertung der Positionen
   über die Zeit die bessere Quelle; sie liefert Richtung und Betrag zugleich.
2. **Totzone 30 mm/s.** Gemessene Geschwindigkeiten darunter werden verworfen und
   auf 0 gesetzt. **Läuft das Band langsamer als 30 mm/s, meldet `base_cam`
   dauerhaft `v_band = 0`** und die davon abhängige Kette bricht zusammen. Vor
   allem anderen in B1 zu prüfen.
3. **Der Wert wird gehalten, nicht zurückgesetzt.** Aktualisiert wird nur, solange
   sich ein Objekt in der Messregion des Trackers befindet; sonst bleibt der
   letzte Wert stehen. Beim Start ist er **0,0** — bis der erste Klotz die
   Messregion durchquert hat, koppeln alle Tracks mit Geschwindigkeit null und
   stehen scheinbar still.

### ⚠️ Zur Semantik von `z`

`vision/detection.py` bildet `z = center_z + height/2`, wobei `center_z` aus den
rückprojizierten Ecken der **Oberseite** stammt. **Am 14.09.2026 am Aufbau
nachgemessen:** gemeldet wurden 103,1 mm, die unabhängige Rückrechnung über
dieselben Boxecken ergab 53,1 mm — die Differenz ist exakt die halbe gemeldete
Höhe. Das Feld ist damit **weder Oberkante noch Mittelpunkt**, sondern liegt eine
halbe Höhe über der Ecken-Referenz. Regelungsrelevant ist es nicht — die
Greifhöhe wird aus `belt_surface_z_m` und `height` gebildet, nicht aus `z`. **Vor
jeder regelungsrelevanten Verwendung von `z` nachrechnen.**

### Geprüft und bewusst **nicht** aufgenommen: `square`

`vision/detection.py` berechnet je Detektion ein Merkmal `square` (Seitenverhältnis
des Pixel-`minAreaRect` ≥ 0,92), das `vision/tracker.py` nutzt, um die Orientierung
fast quadratischer Objekte einzufrieren. Das Merkmal erreicht den Vertrag heute
nicht. Die Aufnahme als zehntes Feld wurde geprüft und **verworfen**:

1. **Die Information steckt bereits im Vertrag.** `length` und `width` (Felder 6/7)
   liefern dasselbe über einen Seitenverhältnis-Test. **Nicht mit derselben
   Schwelle:** `detection.py` misst am *unerodierten* Pixelrechteck,
   `length`/`width` stammen aus der *erodierten* 3D-Kontur. Ein Startwert von
   **0,85** statt 0,92 ist am Aufbau begründet (Nachtrag 4 / M5) und an weiteren
   Klötzen zu bestätigen.

   > **Wichtig, und der eigentliche Grund, warum der Übertragungsweg egal ist:** Die
   > Grundfläche wird richtungsabhängig **zu groß** gemessen — ein exakt
   > quadratischer 50 × 50-Klotz kam am 14.09.2026 als 58,8 × 52,3 heraus,
   > Seitenverhältnis 0,889. Das unterschreitet die 0,92 und würde den Klotz als
   > *nicht* quadratisch einstufen. Das interne `square`-Flag hätte **denselben
   > Fehler**, weil es aus derselben Tiefenkontur stammt. Die Schwelle ist das
   > Problem, nicht das fehlende Feld.
2. **Kein Verbraucher braucht es.** „Fast quadratisch" heißt per Definition, dass
   sich die beiden Abmessungen um **weniger als 8 %** unterscheiden. Mehr als diese
   8 % kann eine um 90° vertauschte Achszuordnung nicht anrichten — bei Klötzen von
   25…76 mm gegen eine Öffnung von rund 130 mm deckt das die Greifermarge ab.
3. **Der Preis wäre unverhältnismäßig.** Stride 9 → 10 heißt: dieses Dokument,
   `contracts.py`, `base_cam` und **jeder** Verbraucher müssen gemeinsam umgestellt
   werden. Ein Verbraucher, der zurückbleibt, liest ab dem zweiten Objekt Unsinn.

→ Wer die Information braucht, rechnet sie aus `length`/`width` aus. Siehe auch
`vorgaengerprojekt-abgleich.md` §7 und den Hinweis unter S3.

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

### ⚠️ Pflicht des Empfängers: Höhenkorrektur von `x` und `y`

`x` und `y` sind mit der **Banddistanz** zurückprojiziert, der gemessene Punkt
liegt aber auf der **Oberseite** des Klotzes. Beide Werte sind dadurch um den
Faktor `z_band/(z_band − blockhoehe)` zu groß. Der `object_follower` **muss**
deshalb vor jeder weiteren Verarbeitung skalieren:

```
x_korr = x · (z_band − blockhoehe) / z_band          (y analog)
```

`blockhoehe` kommt aus S4 Feld 10. Die Rechnung ist exakt, keine Näherung.
Begründung, Größenordnung (6…39 mm, je nach Klotzhöhe und Distanz) und warum der
Fehler **nicht** wegkonvergiert: `entscheidungen.md`, Nachtrag 3 / N1.

### `orientation` wird vom Regelpfad nicht verwendet

Bewusst: Der kommandierte Gierwinkel stammt aus `base_cam` über S4 Feld 7 und ist
dort über das Mittelungsfenster von `vectoring` geglättet — eine Einzelmessung der
Roboterkamera wäre schlechter. Das Feld bleibt für Diagnose und Debug-Bild
erhalten. Wer eine Verwendung im Regelpfad sucht, sucht vergeblich.

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

> **Hinweis zur Orientierung fast quadratischer Objekte.** Bei
> `min(length, width) / max(…) ≥ 0,85` (Schwelle aus Nachtrag 4 / M5; **nicht** die
> 0,92 aus `detection.py` — die Grundflächenmessung verzieht das Verhältnis) ist die
> Zuordnung der beiden Achsen
> unsicher: Der Tracker in `base_cam` friert für solche Objekte den zuerst
> gemessenen Winkel ein, und der kann Länge und Breite vertauscht haben. Das
> Gütemaß der Winkelmittelung in `vectoring` schlägt dabei **nicht** an, weil der
> Winkel konstant hereinkommt. Verbraucher, für die die Achszuordnung zählt —
> konkret die Greifbarkeitsprüfung im `priority_handler` — prüfen in diesem Fall
> `max(length, width)`. Ausführlich unter S1, „Geprüft und bewusst nicht
> aufgenommen: `square`".

---

## S4 — `target` · priority_handler → object_follower

Feste Länge 13, kein Kopf/Stride (immer genau ein Ziel oder keines).

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
| 12 | `zone_upstream` | m | **stromaufwärtige Längsgrenze der Greifzone**, als Koordinate entlang der Bandrichtung. Der `object_follower` begrenzt seine Zielpose in `ANFAHREN` darauf — und **nur** darauf. Auch bei `has_target = 0` gültig. |

**Zu Feld 12 — warum nur eine Grenze.** Quer wird nicht begrenzt (Thema 6: der
Roboter wartet am Zonenrand „auf der Querposition des Blocks"), stromabwärts darf
nicht begrenzt werden (P4: der Roboter muss dem Block über die Zonengrenze hinaus
folgen dürfen, sonst bricht ein fast gelungener Griff ab), und die Höhe deckt der
Arbeitsraum-Clamp des Sicherheitsgates ab. Die Greifzone bleibt vollständig im
Besitz des `priority_handler` (Thema 4); der Follower wendet nur diesen einen Wert
an. Begründung: `entscheidungen.md`, Nachtrag 3 / N3.

Bei `has_target = 0` sind Felder 2–11 bedeutungslos, **Feld 12 bleibt gültig**. **Das Zurückziehen des Ziels
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

**Typ: `cartesian_pose`** — bestätigt am 14.09.2026 aus den Komponenten­beschreibungen
im AICA-Image: Der `attractor`-Eingang des `SignalPointAttractor` ist
`cartesian_pose`, und `aica_core_components::ros::TfToSignal` gibt `pose` ebenfalls
als `cartesian_pose` aus. **A2 erledigt.**

> Hier stand vorher `cartesian_state` mit dem Vermerk „zu bestätigen". Das war
> falsch. AICA prüft beim Verbinden die Typgleichheit — mit `cartesian_state` ließe
> sich die Leitung im Graphen nicht ziehen. Auf der ROS-Ebene tragen übrigens beide
> dieselbe Nachricht (`modulo_interfaces/msg/EncodedState`); der Unterschied
> existiert nur in der AICA-Typprüfung, fällt also erst beim Verdrahten auf.

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
UI erhalten.

> ⚠️ `is_object_grasped` folgt dem Robotiq-Status `gOBJ` und ist auch dann true,
> wenn der Greifer **beim Öffnen** auf ein Objekt trifft
> (`GOBJ_OBJECT_WHILE_OPENING`). In `LOESEN` kann `has_object` dadurch kurz
> flackern. Die Zustandsmaschine darf `LOESEN` deshalb nicht über `has_object = 0`
> verlassen, sondern über den Abschluss der Öffnungsbewegung. Der Plan sah nur "Greifer zu" vor — das wäre ein Echo des Eingangs
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
| ~~Typ von `target_pose`~~ | **erledigt 14.09.2026: `cartesian_pose`** (A2) |
| Zustandscodes in `follower_status` Feld 1 | Thema 6 |
| Verfallsfrist in `world_state` | Thema 6 |
| Ob `gripper_change` genutzt wird | Ausbaustufe |
