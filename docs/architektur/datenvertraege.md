# Datenverträge Robotetris

**Verbindliche Spezifikation aller AICA-Signale zwischen den Komponenten.**
Grundlage für die Implementierung. Änderungen nur hier — und dann in
`roboter_tetris/contracts.py` nachziehen, nie umgekehrt.

Bezug: `entscheidungen.md` Themen 1–7 und Nachträge 3–11. **Stand 21.09.2026**,
alle Signale in `contracts.py` umgesetzt.

## Übersicht

| | Signal | von → an | Typ | Länge |
|---|---|---|---|---|
| S1 | `objects` | base_cam → vectoring | double_array | `3 + 9·n` |
| S2 | `object_position` | robot_cam → object_follower | double_array | 6 |
| S3 | `tracks` | vectoring → priority_handler, data_tracker | double_array | `5 + 14·n` |
| S4 | `target` | priority_handler → object_follower | double_array | 17 |
| S5 | `not_pickable` | priority_handler → data_tracker | double_array | `2 + n` |
| S6 | `target_pose` | object_follower → signal_point_attractor | cartesian_pose | – |
| S7 | `picked_id` | object_follower → priority_handler, data_tracker | double_array | 3 |
| S8 | `follower_status` | object_follower → interface_streamer | double_array | 7 |
| S9 | `gripper_close` / `motion_done` / `has_object` | object_follower ↔ robotiq_gripper | bool | – |
| S10 | `world_state` | data_tracker → interface_streamer | double_array | `5 + 17·n` |

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
   vermerkt. **Der Bezugspunkt am Roboter ist der Flansch `ur_tool0`**, nicht der
   in der UR-Steuerung konfigurierte TCP: Der Greifer steht nicht im URDF, und
   sowohl `robot_state_broadcaster` als auch der IK-Velocity-Controller arbeiten
   am Flansch (Nachtrag 5 / M8). Feste Höhen in diesem Frame, gemessen am
   15.09.2026: **Bandoberfläche z = 0,0536 m**, Flansch → Backenspitze
   **0,245 m**, Flansch → Griffpunkt **0,235 m**. Die Bandrichtung ist praktisch
   die **y-Achse**.
6. **Statusfelder statt stiller Annahmen.** `valid`, `has_target`, `status`.
7. **Verbraucher gaten auf `t` bzw. `seq`**, bevor sie rechnen. AICA publiziert
   Ausgänge in jedem Schritt — dieselben Daten werden mehrfach gesehen.
8. **Alle Indizes und Schrittweiten in `roboter_tetris/contracts.py`.** Keine
   Komponente indiziert von Hand in ein fremdes Array.
9. **Alle Signale sind `double_array`** (`std_msgs/Float64MultiArray`), außer wo
   ausdrücklich anders angegeben. Ganzzahlen werden als `double` transportiert.

---

## S1 — `objects` · base_cam → vectoring

```
[ t, n, v_band_gemessen,  <obj_0>, <obj_1>, ... ]
   Kopflänge 3                    Stride 9
```

| Kopf | Bedeutung |
|---|---|
| `t` | Zeitstempel des Bildes (`header.stamp`), Sekunden |
| `n` | Anzahl Objekte |
| `v_band_gemessen` | global geschätzte Bandgeschwindigkeit, m/s. **Frame-Größe, kein Objektmerkmal** — der Tracker weist sie allen Tracks gemeinsam zu (`set_global_velocity`). Dient nur noch als **grobe Laufkontrolle** des Bandes. Die maßgebliche Geschwindigkeit schätzt `vectoring` selbst (S3, `entscheidungen.md` Nachtrag 6 / Z2). ⚠️ Drei Eigenheiten der Quelle, siehe unten. |

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | fortlaufend ab 1, pro Programmlauf neu vergeben |
| 1 | `color` | – | 0=Rot 1=Gelb 2=Grün 3=Blau 4=Weiß 5=Schwarz 6=Unbekannt |
| 2 | `x` | m | world |
| 3 | `y` | m | world |
| 4 | `z` | m | world, **gemessen etwa halbe Blockhöhe über dem Band** — nicht die Oberkante, siehe unten |
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
halbe Höhe über der Ecken-Referenz. Weil diese Referenz praktisch auf der
Bandoberfläche lag (53,1 gegen gemessene 53,6 mm, B17), ergibt sich in der Summe
**`z ≈ Bandoberfläche + halbe Blockhöhe`** — für den 100-mm-Klotz 103,6 mm gegen
gemessene 103,1. Eine einzelne Messung; der Befund gilt für diese Szene, nicht als
Garantie. Regelungsrelevant ist es nicht — die
Greifhöhe wird aus `belt_surface_z_m` und `height` gebildet, nicht aus `z`. **Vor
jeder regelungsrelevanten Verwendung von `z` nachrechnen.**

### Geprüft und bewusst **nicht** aufgenommen: `square`

`vision/detection.py` berechnet je Detektion ein Merkmal `square` (Seitenverhältnis
des Pixel-`minAreaRect` ≥ 0,92), das `vision/tracker.py` nutzt, um die Orientierung
fast quadratischer Objekte einzufrieren. Das Merkmal erreicht den Vertrag heute
nicht. Die Aufnahme als zehntes Feld wurde geprüft und **verworfen**:

1. **Die Information steckt bereits im Vertrag.** `length` und `width` (Felder 6/7)
   liefern dasselbe über einen Seitenverhältnis-Test mit der Schwelle **0,92** —
   allerdings **nur auf den geglätteten Werten aus S3**, nicht bildweise. Am
   ruhenden Klotz streut das Verhältnis je Einzelbild von 0,727 bis 0,999
   (Nachtrag 4 / M5); geglättet liegt es stabil bei 0,95.

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
[ t, n, v_belt_x, v_belt_y, n_pool,  <track_0>, <track_1>, ... ]
  Kopf 5                             Stride 14
```

**Kopf — die gepoolte Bandgeschwindigkeit** (`entscheidungen.md` Nachtrag 6 / Z2):

| Kopf | Einheit | Bedeutung |
|---|---|---|
| `t` | s | Zeitstempel, übernommen aus S1 |
| `n` | – | Anzahl Tracks |
| `v_belt_x`, `v_belt_y` | m/s | gemeinsame Ausgleichsrechnung über **alle Messungen seit dem Einschwingen** aller Tracks des laufenden Durchlaufs — gemeinsame Steigung, je Track eigener Achsenabschnitt (`entscheidungen.md` Nachtrag 6 / Z9). Wird mit jeder Messung genauer |
| `n_pool` | – | Anzahl der Tracks, die dazu beitragen (auch bereits verschwundene). **`n_pool = 0` heißt: noch kein Schätzwert** — `v_belt_*` ist dann bedeutungslos und darf nicht verwendet werden |

> `n_pool` ist die ausdrückliche Antwort auf eine Falle von `base_cam`: Dessen
> `v_band` startet stumm bei 0,0 und sieht damit aus wie „Band steht". Hier ist
> „noch nichts geschätzt" vom Wert unterscheidbar.

**Je Track:**

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | identisch zur base_cam-ID |
| 1 | `color` | – | durchgereicht |
| 2 | `x` | m | ab Status 0 **geglättet**, gültig für `t` im Kopf; davor letzte Einzelmessung |
| 3 | `y` | m | dito |
| 4 | `z` | m | dito |
| 5 | `orientation` | rad | dito, [0, π) |
| 6 | `length` | m | ab Status 0 gemittelt, davor letzte Einzelmessung (nur Anzeige) |
| 7 | `width` | m | dito |
| 8 | `height` | m | dito — ein beim Aufsetzen umgekippter Klotz ändert genau diesen Wert |
| 9 | `status` | – | siehe Tabelle unten |
| 10 | `vx` | m/s | **eigene** Geschwindigkeitsschätzung dieses Tracks — ab Status 0 aus allen seinen Messungen seit dem Einschwingen, fortlaufend genauer. Anzeige und Diagnose; gerechnet wird mit dem Pool |
| 11 | `vy` | m/s | dito |
| 12 | `v_change` | m/s | Betrag der Differenz zwischen den Geschwindigkeiten der jüngeren und der älteren Halbsekunde — **das Maß für „konstant gemessen"**; entscheidet den Übergang 3 → 0 |
| 13 | `ori_quality` | – | Güte der Orientierungsmittelung, 0…1 — Resultantenlänge der Winkel über den doppelten Winkel. Klein heißt: Winkel verrauscht, Klotz teilverdeckt oder halb außerhalb der ROI. **Bewertet wird sie beim Verbraucher**, der Follower fällt unter seiner Schwelle auf die Bandrichtung zurück (`entscheidungen.md` Nachtrag 6 / Z13). Vor Status 0: 0

**Länge:** `5 + 14·n`

> Felder 0–9 stehen an denselben Indizes wie vorher. Neu sind nur die vier
> Felder am Ende und der erweiterte Kopf.
>
> Feld 12 hieß in der ersten Fassung `v_sigma` (Streuung der Schätzung). Das
> Kriterium dahinter versagte beim Umkippen — nachgerechnet in Nachtrag 6 / Z9.

### Statuscodes

| Code | Bedeutung | Folge |
|---|---|---|
| 0 | **final** — Geschwindigkeit konstant gemessen; der Klotz wird nicht mehr in Frage gestellt | Kandidat für `priority_handler`; seine Messungen fließen in den Pool |
| 3 | **einschwingend** — gerade aufgelegt, kippt womöglich noch | noch nicht Kandidat |
| ~~1~~, ~~2~~ | **entfallen, werden nicht wiederverwendet** | — |

Code 3 ist kein Fehler, sondern der Normalzustand direkt nach dem Auflegen.
`priority_handler` nimmt ausschließlich Status 0.

> **Warum 1 und 2 entfallen** (Nachtrag 6 / Z3): Sie standen für „steht /
> verklemmt" und „Sprung / angestoßen". Beides gibt es nicht — die Klötze werden
> frei aufgelegt und laufen mit dem Band. Eine Fehldetektion in einem einzelnen
> Bild bekommt keinen Status; `vectoring` verwirft die einzelne Messung. Die
> Nummern bleiben unbelegt, damit eine ältere Notiz über „Status 1" nie etwas
> anderes bedeuten kann als früher.

> Die Position ist auf `t` extrapoliert. Der Empfänger rechnet weiter mit der
> **Pool-Geschwindigkeit** aus dem Kopf: `p(t') = p + v_belt · (t' − t)`, sofern
> `n_pool ≥ 1`.

> **Hinweis zur Orientierung fast quadratischer Objekte.** Bei
> `min(length, width) / max(…) ≥ 0,92` — **auf den hier geglätteten Werten**, nicht
> auf einer Einzelmessung; bildweise streut das Verhältnis zu stark
> (Nachtrag 4 / M5) — ist die Zuordnung der beiden Achsen
> unsicher: Der Tracker in `base_cam` friert für solche Objekte den zuerst
> gemessenen Winkel ein, und der kann Länge und Breite vertauscht haben. Das
> Gütemaß der Winkelmittelung in `vectoring` schlägt dabei **nicht** an, weil der
> Winkel konstant hereinkommt. Die Greifbarkeitsprüfung im `priority_handler` ist
> davon **nicht mehr betroffen**: Sie prüft die Diagonale, und die ändert sich durch
> vertauschte Achsen nicht (`entscheidungen.md` Nachtrag 7 / H2). Ein künftiger
> Verbraucher, für den die Achszuordnung zählt, prüft in diesem Fall
> `max(length, width)`. Ausführlich unter S1, „Geprüft und bewusst nicht
> aufgenommen: `square`".

---

## S4 — `target` · priority_handler → object_follower

Feste Länge 17, kein Kopf/Stride (immer genau ein Ziel oder keines).

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
| 11 | `t_rest` | s | geschätzte Restzeit, bis das Objekt die **Greifebene** erreicht — die Frist, bis zu der das Absenken begonnen haben muss (Feld 15). **Negativ**, sobald das Ziel die Ebene überschritten hat: Das Ziel bleibt gewählt, ob der Griff noch beginnt, entscheidet der Follower |
| 12 | `zone_upstream` | m | **stromaufwärtige Längsgrenze der Greifzone**, als Koordinate entlang der Bandrichtung. Der `object_follower` begrenzt seine Zielpose in `ANFAHREN` darauf — und **nur** darauf. Auch bei `has_target = 0` gültig. |
| 13 | `vx` | m/s | Geschwindigkeit, mit der der Follower das Ziel vorhersagt: die **Pool-Geschwindigkeit** aus dem S3-Kopf |
| 14 | `vy` | m/s | dito |
| 15 | `grasp_plane` | m | **Greifebene** — letzte Längsposition, von der aus der Greifprozess noch vor dem Zonenende fertig wird. Koordinate entlang der Bandrichtung wie Feld 12. Der Follower beginnt `ABSENKEN` nur, solange der Block davor liegt (`entscheidungen.md` Nachtrag 6 / Z11). Auch bei `has_target = 0` gültig. |
| 16 | `ori_quality` | – | Güte der Orientierung des Ziels, durchgereicht aus S3 Feld 13. Der Follower verwendet `orientation` nur, wenn die Güte über `orientation_quality_min` liegt, sonst die Bandrichtung `atan2(vy, vx)` |

**Die Längskoordinate `s` der Felder 12 und 15** ist die Projektion auf die
Bandrichtung aus den Feldern 13/14 **derselben Nachricht**:
`s = (x · vx + y · vy) / |v|`, Ursprung im Ursprung von `world`, wächst
stromabwärts. Erzeuger und Verbraucher rechnen sie über **eine** Funktion,
`contracts.along_belt` — sonst wartet der Roboter woanders, als der
`priority_handler` in seiner Erreichbarkeitsrechnung annimmt (derselbe Grund wie in
Nachtrag 3 / N3).

**Zu Feld 15 — ein Tor, keine Begrenzung.** Die Greifebene begrenzt die Zielpose
nicht; sie entscheidet nur, ob das Absenken noch **beginnen** darf. Die Aussage
zu Feld 12, dass stromabwärts nicht begrenzt wird, bleibt damit richtig: Ein
laufender Griff darf dem Block über das Zonenende hinaus folgen (P4).

**Zu Feld 13/14 — warum der Pool und nicht die eigene Schätzung des Ziels.** Das
Ziel ist immer ein finaler Track (Status 0), gehört also selbst zum Pool — bei
`has_target = 1` ist `n_pool ≥ 1` garantiert. Pool und eigene Schätzung messen
dieselbe Größe, der Pool nur mit mehr Daten. Der Follower bekommt die Geschwindigkeit
über S4, weil er S3 nicht liest (ein Eingang, ein Vertrag).

**Zu Feld 12 — warum nur eine Grenze.** Quer wird nicht begrenzt (Thema 6: der
Roboter wartet am Zonenrand „auf der Querposition des Blocks"), stromabwärts darf
nicht begrenzt werden (P4: der Roboter muss dem Block über die Zonengrenze hinaus
folgen dürfen, sonst bricht ein fast gelungener Griff ab), und die Höhe deckt der
Arbeitsraum-Clamp des Sicherheitsgates ab. Die Greifzone bleibt vollständig im
Besitz des `priority_handler` (Thema 4); der Follower wendet nur diesen einen Wert
an. Begründung: `entscheidungen.md`, Nachtrag 3 / N3.

Bei `has_target = 0` sind die Felder 2–11 und 16 bedeutungslos. **Die Felder 12–15
bleiben gültig, solange eine Bandschätzung vorliegt** — 12 und 15 sind Koordinaten
entlang der Richtung aus 13/14 und ohne sie nicht lesbar. **Ohne Bandschätzung**
(`n_pool = 0` in S3 oder Band unter 0,01 m/s) sind die Felder 12–15 null und
bedeutungslos: Es gibt dann weder eine Richtung noch eine Greifebene, und
`has_target` ist ohnehin 0 (`entscheidungen.md` Nachtrag 7 / H4). **Das Zurückziehen des Ziels
ist die Abbruchregel** — wird ein Objekt gepickt oder verpasst (`picked_id`),
verschwindet seine ID oder fällt sein Status auf 3 zurück, setzt
`priority_handler` `has_target = 0` und der Follower bricht ab.

> ⚠️ Hier stand zusätzlich „unerreichbar" und „unplausibel". **Unerreichbarkeit
> war nie ein Rückzugsgrund** — P4 schließt sie ausdrücklich aus, damit ein fast
> gelungener Griff nicht abbricht. Ob ein Griff noch *beginnen* darf, entscheidet
> seit Nachtrag 6 / Z11 der Follower an der Greifebene (Feld 15). „Unplausibel"
> entfiel mit den Statuscodes 1/2 (Z3).

---

## S5 — `not_pickable` · priority_handler → data_tracker

```
[ t, n,  id_0, id_1, ... ]     Stride 1
```
Reine Buchhaltung für die Anzeige. Nicht im Regelpfad.

Inhalt: die IDs aller **aktuellen** Tracks, die die Greifebene überschritten haben,
jeder Status — ohne das aktuelle Ziel, solange es gewählt ist. Die Liste ist keine
Historie: Sie ist durch die Zahl der Tracks begrenzt, das Festhalten
(`out_of_bounds`) übernimmt der `data_tracker`. Ohne Bandschätzung gibt es keine
Ebene, die Liste ist dann leer.

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
  Versatz Flansch → Griffpunkt `flange_to_grip_point_m` (0,235 m) und Vorhalt
  `v · lead_time_s` (`entscheidungen.md` Nachtrag 6 / Z6, Z7)

Ersetzt `frame_to_signal` im Betrieb.

---

## S7 — `picked_id` · object_follower → priority_handler, data_tracker

Feste Länge 3.

| # | Feld | Bemerkung |
|---|---|---|
| 0 | `seq` | zählt bei jedem **abgeschlossenen** Versuch hoch. **0 = noch kein Versuch seit der Aktivierung** des Followers, der erste Versuch trägt 1 |
| 1 | `id` | betroffene Objekt-ID |
| 2 | `outcome` | 0 = erfolgreich abgelegt · 1 = Fehlgriff · 2 = Objekt verloren · **3 = verpasst** — Greifebene überschritten, bevor abgesenkt wurde (Nachtrag 6 / Z11) · **4 = abgebrochen**, bevor gegriffen wurde, aus einem anderen Grund: Ziel zurückgezogen oder gewechselt, Zeitüberschreitung, Eingang steht still, Datenfehler, Arbeitsraum (Nachtrag 9 / G2). Der Grund steht im Log |

`outcome = 0` heißt: Der Klotz liegt in der Kiste. Das gilt auch, wenn der
Follower mit Klotz im Greifer abbrechen musste — er fährt dann zur Ablage und
öffnet erst dort (`entscheidungen.md` Nachtrag 6 / Z12); der Abbruchgrund steht im
Log, nicht im Signal.

Verbraucher merken sich die zuletzt gesehene `seq` und reagieren genau einmal —
auf jede Änderung außer auf 0. Ein Rücksprung auf 0 heißt, dass der Follower neu
aktiviert wurde; danach zählt wieder jede Änderung. Die Regel steckt in
`contracts.AttemptWatcher`, den alle Verbraucher nutzen.
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
| 6 | `w_wirksam` | – | tatsächlich verwendete Gewichtung 0…1 (nach Rampe und Rückfall): der **Anteil der eingestellten Gewichte** `weight_along`/`weight_across`, der gerade wirkt — 0 = nur Basiskamera (Nachtrag 10 / J3) |

**Zustandscodes (Feld 1)** — Reihenfolge wie im Zustandsautomaten (Thema 6):

| Code | Zustand | Code | Zustand |
|---|---|---|---|
| 0 | `WARTEN` | 5 | `HEBEN` |
| 1 | `ANFAHREN` | 6 | `ABLEGEN` |
| 2 | `FOLGEN` | 7 | `LOESEN` |
| 3 | `ABSENKEN` | 8 | `ABBRUCH` |
| 4 | `GREIFEN` | | |

Die Regelabweichungen (Felder 3–5) beziehen sich auf den Block: Flansch minus
**vorhergesagte** Klotzposition, ohne Vorhalt. `err_laengs` entlang der geschätzten
Bandrichtung, **positiv = Flansch voraus**; `err_quer` senkrecht dazu, **positiv =
links der Laufrichtung**; `err_z` = Flanschhöhe minus kommandierte Höhe. Im
eingeschwungenen `FOLGEN` ist `err_laengs` das Maß für `lead_time_s` (Nachtrag 9 /
G7). In Zuständen ohne Block (`WARTEN`, `ABLEGEN`, `LOESEN`, `ABBRUCH`) sind sie 0,
ebenso `target_id` (Nachtrag 8 / F5).

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
| `motion_done` | gripper → object_follower | `Bool` | Bewegung abgeschlossen (beide Richtungen: Schließen **und** Öffnen) |
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
[ t, n, v_belt_x, v_belt_y, n_pool,  <entry_0>, ... ]
  Kopf 5 wie S3                      Stride 17
```

**Kopf** unverändert aus S3 übernommen — damit die Anzeige die geschätzte
Bandgeschwindigkeit zeigen kann (das eigene Ziel „Prozess nachvollziehbar
darstellen", `entscheidungen.md` Nachtrag 6 / Z1).

Felder 0–13 wie `tracks` (S3), zusätzlich:

| # | Feld | Bemerkung |
|---|---|---|
| 14 | `picked` | 1 = erfolgreich abgelegt |
| 15 | `out_of_bounds` | 1 = **Greifebene** überschritten, ohne gegriffen zu werden — ab dort ist der Klotz nicht mehr zu holen und fällt später am Bandende herunter (Nachtrag 6 / Z5, Z11) |
| 16 | `present` | 1 = steht in den aktuellen `tracks`, Felder 0–13 sind aktuell · 0 = **Nachlauf**: aus `tracks` verschwunden, Felder 0–13 sind die letzten bekannten Werte (Nachtrag 7 / T1) |

**Länge:** `5 + 17·n`

**Warum es den Nachlauf gibt.** Bei jedem Griff verschwindet der Klotz beim Heben
aus dem Bild, `picked_id` mit `outcome = 0` kommt aber erst nach dem Ablegen,
Sekunden später. Ohne Nachlauf wäre der Eintrag dann schon weg und `picked` käme
nie an. `present = 0` sagt der Anzeige, dass die Werte eingefroren sind — der Klotz
liegt im Greifer oder ist verloren, nicht mehr an der gezeigten Stelle.

**Verfallsregel:** Ein Eintrag ist **erledigt**, sobald `picked`, `out_of_bounds`
oder `present = 0` gilt. Er verschwindet eine konfigurierbare Frist nach der
**letzten** dieser Änderungen aus dem Array, gemessen in der S3-Zeit (`t` im
Kopf). Ein Eintrag, der dabei noch in `tracks` steht, kommt nicht wieder. Ohne
diese Regel wächst das Signal über den Programmlauf monoton — bei periodischem
Publizieren ein echtes Problem. Historie gehört ins Log, nicht ins Signal.

---

## `contracts.py` — Aufbau

```python
# Je Signal: Kopflänge, Stride bzw. feste Länge, Feldindizes, NamedTuple, pack/unpack.
OBJECTS_HEADER = 3
OBJECTS_STRIDE = 9
(OBJ_ID, OBJ_COLOR, OBJ_X, OBJ_Y, OBJ_Z, OBJ_ORI, OBJ_LEN, OBJ_WID, OBJ_HGT) = range(9)

def pack_objects(t, v_belt, objects) -> list: ...
def unpack_objects(arr) -> ObjectsMsg | None: ...

# Gemeinsame Vokabeln
TRACK_FINAL, TRACK_SETTLING          # S3 Status (1, 2 stillgelegt)
OUTCOME_PLACED … OUTCOME_ABORTED     # S7 outcome 0–4
STATE_WAIT … STATE_ABORT             # S8 Zustandscodes 0–8
COLOR_RED … COLOR_UNKNOWN            # S1 Farben 0–6

# Zwei gemeinsame Regeln, die jede Seite gleich rechnen muss
def along_belt(x, y, v_belt) -> float: ...   # Längskoordinate der S4-Felder 12, 15
class AttemptWatcher: ...                    # S7: jede neue seq einmal, nie die 0
```

Regeln:
- `unpack_*` liefert `None` für ein leeres Signal (noch nichts empfangen) und wirft
  `ContractError` bei falscher Länge (`kopf + n·stride` bzw. feste Länge) — ein
  Empfänger darf nie auf halben Daten rechnen.
- `unpack_*` rechnet **nicht** um und interpretiert nicht; Extrapolation und
  Transformationen macht der Verbraucher. Die zwei Ausnahmen oben *definieren*
  Vertragssemantik, die Erzeuger und Verbraucher teilen müssen.
- Keine Komponente importiert Feldindizes aus einer anderen Komponente, nur aus
  `contracts.py`. Das Modul ist frei von ROS, cv2 und numpy und wird für sich
  getestet (`test_contracts.py`).

---

## Offene Punkte in diesem Dokument

| Punkt | Abhängig von |
|---|---|
| ~~Typ von `target_pose`~~ | **erledigt 14.09.2026: `cartesian_pose`** (A2) |
| ~~Zustandscodes in `follower_status` Feld 1~~ | **erledigt 21.09.2026** — S8, Nachtrag 8 / F5 |
| ~~Verfallsfrist in `world_state`~~ | **erledigt 21.09.2026** — S10, `expiry_after_done_s` 10 s, Nachtrag 7 / T1 |
| Ob `gripper_change` genutzt wird | Ausbaustufe |
