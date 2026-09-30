# Datenverträge Robotetris

**Verbindliche Spezifikation aller Signale zwischen den Komponenten.** Umgesetzt in
`roboter_tetris/contracts.py`, geprüft von `test_contracts.py`. Änderungen an einem
Signal zuerst hier, dann in `contracts.py` — nie umgekehrt.

Begründungen: `entscheidungen.md` (Abschnittsnummern in Klammern).

## Übersicht

| | Signal | von → an | Typ | Länge |
|---|---|---|---|---|
| S1 | `objects` | base_cam → vectoring | double_array | `3 + 9·n` |
| S2 | `object_position` | stillgelegt, kein Sender im Paket | double_array | 6 |
| S3 | `tracks` | vectoring → priority_handler, data_tracker | double_array | `5 + 14·n` |
| S4 | `target` | priority_handler → object_follower | double_array | 17 |
| S5 | `not_pickable` | priority_handler → data_tracker | double_array | `2 + n` |
| S6 | `target_pose` | object_follower → Signal Point Attractor | cartesian_pose | – |
| S7 | `picked_id` | object_follower → priority_handler, data_tracker | double_array | 3 |
| S8 | `follower_status` | object_follower → interface_streamer | double_array | 6 |
| S9 | `gripper_close` / `motion_done` / `has_object` | object_follower ↔ robotiq_gripper | bool | – |
| S10 | `world_state` | data_tracker → interface_streamer | double_array | `5 + 17·n` |

---

## Grundregeln

1. **Kopf-Konvention.** Jedes listenartige Signal beginnt mit `[t, n, ...]`:
   `t` Zeitstempel in Sekunden, `n` Anzahl Einträge. Länge = `kopflänge + n · stride`.
2. **Nie leer.** Auch ohne Inhalt wird der Kopf gesendet (`[t, 0, …]`). Das
   unterscheidet „sieht nichts“ von „sendet nicht mehr“.
3. **`t` nur im Kopf.** Alle Einträge eines Zyklus teilen denselben Zeitpunkt. `t` ist
   der Header-Stempel des Kamerabildes, aus dem die Werte stammen (§3.1).
4. **SI-Einheiten:** m, m/s, rad, s. Die Bildverarbeitung rechnet intern in mm und
   rechnet beim Packen um.
5. **Bezugssystem `world`** (= `ur_base_link`, die Roboterbasis) für alle Positionen.
   Der Bezugspunkt am Roboter ist der **Flansch `ur_tool0`**, nicht der TCP der
   UR-Steuerung (§4).
6. **Statusfelder statt stiller Annahmen:** `has_target`, `status`, `n_pool`,
   `present`, `ori_quality`.
7. **Verbraucher gaten auf `t` bzw. `seq`.** AICA publiziert Ausgänge in jedem Takt;
   dieselben Daten kommen mehrfach an.
8. **Alle Indizes und Schrittweiten in `contracts.py`.** Keine Komponente indiziert
   von Hand in ein fremdes Array.
9. **Datensignale sind `double_array`** (`std_msgs/Float64MultiArray`); Ganzzahlen
   werden als `double` transportiert.

---

## S1 — `objects` · base_cam → vectoring

```
[ t, n, v_band_gemessen,  <obj_0>, <obj_1>, ... ]
  Kopf 3                  Stride 9
```

| Kopf | Bedeutung |
|---|---|
| `t` | Zeitstempel des Bildes (`header.stamp`), s |
| `n` | Anzahl Objekte |
| `v_band_gemessen` | Bandgeschwindigkeit aus dem Tracker von `base_cam`, m/s — **nur grobe Laufkontrolle**, siehe unten. Maßgeblich ist die Schätzung in `vectoring` (S3). |

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | fortlaufend ab 1, je Programmlauf neu |
| 1 | `color` | – | 0 Rot · 1 Gelb · 2 Grün · 3 Blau · 4 Weiß · 5 Schwarz · 6 unbekannt |
| 2 | `x` | m | Mitte der Oberseite, `world` |
| 3 | `y` | m | dito |
| 4 | `z` | m | **Klotzmitte** = Oberseite − halbe Höhe |
| 5 | `orientation` | rad | Längsachse, [0, π) |
| 6 | `length` | m | |
| 7 | `width` | m | |
| 8 | `height` | m | z der Oberseite in `world` − Bandoberfläche (`belt_surface_z_mm`); Oberseitentiefe = Median über den Umriss, um `top_depth_bias_mm` korrigiert (§5.1) |

S1 enthält nur Messungen dieses Bildes; weitergeführt wird in `vectoring`.

**Einzelne Messungen tragen keine Geometrie:** Position und Höhe streuen um
0,2–0,6 mm, die Grundfläche um über 26 mm (§5.1). Abmessungen und Orientierung werden
deshalb nur auf den geglätteten Werten aus S3 geprüft.

**`v_band_gemessen`** stammt aus dem übernommenen Tracker: Es ist nur die
**y-Komponente**, Werte unter 30 mm/s werden auf 0 gesetzt, und beim Start steht es
auf 0,0, bis der erste Klotz die Messregion durchquert hat. Deshalb schätzt `vectoring`
die Geschwindigkeit selbst aus den Positionen.

`vision/detection.py` berechnet intern ein Merkmal `square` (fast quadratischer
Umriss), mit dem der Tracker die Orientierung solcher Objekte auf den ersten Wert
einfriert. Es ist bewusst nicht Teil von S1: Es streut bildweise wie die Grundfläche,
und kein Verbraucher braucht es — die Greifbarkeit prüft die Diagonale (§7.3), und
die Winkelwahl rechnet modulo 90° (§8.6).

---

## S2 — `object_position` · stillgelegt

Ausgang der früheren Komponenten der Roboterkamera. Sie sind nicht mehr im Paket
(§12); das Signal bleibt in `contracts.py` definiert, damit es mit einer
Roboterkamera wieder aufgenommen werden kann. Feste Länge 6:
`[t, valid, x, y, z_band, orientation]` — `x`, `y`, `orientation` im Kameraframe,
`z_band` Abstand Kamera → Band. `x` und `y` sind mit der Banddistanz zurückprojiziert
und müssen vor der Verwendung mit `(z_band − blockhoehe) / z_band` skaliert werden,
weil der gemessene Punkt auf der Oberseite des Klotzes liegt.

---

## S3 — `tracks` · vectoring → priority_handler, data_tracker

```
[ t, n, v_belt_x, v_belt_y, n_pool,  <track_0>, <track_1>, ... ]
  Kopf 5                             Stride 14
```

| Kopf | Einheit | Bedeutung |
|---|---|---|
| `t` | s | aus S1 übernommen |
| `n` | – | Anzahl Tracks |
| `v_belt_x`, `v_belt_y` | m/s | **gepoolte Bandgeschwindigkeit**: gemeinsame Ausgleichsrechnung über alle Messungen seit dem Einschwingen aller bewegten Tracks des Durchlaufs (§6.1) |
| `n_pool` | – | Anzahl der beitragenden Tracks, auch bereits verschwundener. **`n_pool = 0` heißt: noch keine Schätzung** — `v_belt_*` darf dann nicht verwendet werden |

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `id` | – | ID aus S1 |
| 1 | `color` | – | aus S1 |
| 2 | `x` | m | ab Status 0 geglättet und auf `t` projiziert; davor letzte Einzelmessung |
| 3 | `y` | m | dito |
| 4 | `z` | m | dito |
| 5 | `orientation` | rad | dito, gemittelt über den doppelten Winkel, [0, π) |
| 6 | `length` | m | ab Status 0 gemittelt, davor letzte Einzelmessung |
| 7 | `width` | m | dito |
| 8 | `height` | m | dito — ein beim Aufsetzen umgekippter Klotz ändert genau diesen Wert |
| 9 | `status` | – | siehe unten |
| 10 | `vx` | m/s | **eigene** Geschwindigkeit dieses Tracks aus seinen Messungen seit dem Einschwingen; Anzeige und Diagnose — gerechnet wird mit dem Pool |
| 11 | `vy` | m/s | dito |
| 12 | `v_change` | m/s | Betrag der Differenz zwischen jüngerer und älterer Halbsekunde — entscheidet den Übergang 3 → 0 |
| 13 | `ori_quality` | – | Güte der Winkelmittelung, 0…1 (Länge des resultierenden Vektors); vor Status 0: 0. Bewertet wird sie beim Verbraucher (§6.3) |

### Statuscodes

| Code | Bedeutung | Folge |
|---|---|---|
| 0 | **final** — Geschwindigkeit konstant gemessen | wählbar; Messungen fließen in den Pool |
| 3 | **einschwingend** — gerade aufgelegt, kippt womöglich noch | nicht wählbar; nach `track_expiry_s` ohne Messung vergessen |
| 4 | **vorhergesagt** — war final, wird im Bild nicht mehr gemessen; Position mit der gepoolten Geschwindigkeit auf `t` fortgeschrieben, höchstens `predict_max_s` | wählbar wie 0; kein Beitrag zum Pool |

Die Codes 1 und 2 sind unbelegt und werden nicht vergeben. Der `priority_handler`
wählt aus 0 und 4 (`contracts.TRACK_SELECTABLE`).

Ein Empfänger rechnet eine Position weiter mit dem Pool:
`p(t') = p + v_belt · (t' − t)`, sofern `n_pool ≥ 1`.

---

## S4 — `target` · priority_handler → object_follower

Feste Länge 17 (genau ein Ziel oder keines).

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `t` | s | Gültigkeitszeitpunkt der Position |
| 1 | `has_target` | – | 1 = Ziel gewählt, 0 = kein Ziel |
| 2 | `id` | – | |
| 3 | `color` | – | |
| 4 | `x` | m | `world`, gültig für `t` |
| 5 | `y` | m | |
| 6 | `z` | m | Klotzmitte |
| 7 | `orientation` | rad | [0, π) |
| 8 | `length` | m | |
| 9 | `width` | m | |
| 10 | `height` | m | bestimmt die Greifhöhe |
| 11 | `t_rest` | s | Restzeit, bis das Ziel die **Greifebene** erreicht; negativ, sobald es sie überschritten hat (das Ziel bleibt gewählt, ob ein Griff noch beginnt, entscheidet der Follower) |
| 12 | `zone_upstream` | m | stromaufwärtige Grenze der Greifzone als Längskoordinate. Der Follower begrenzt seine Zielpose in `ANFAHREN` darauf — **nur** darauf (§8.2) |
| 13 | `vx` | m/s | **Pool-Geschwindigkeit** aus dem S3-Kopf, mit der der Follower vorhersagt |
| 14 | `vy` | m/s | dito |
| 15 | `grasp_plane` | m | **Greifebene** als Längskoordinate: `ABSENKEN` beginnt nur, solange das Ziel davor liegt (§7.2, §8.3) |
| 16 | `ori_quality` | – | aus S3 Feld 13. Der Follower nutzt `orientation` nur über `orientation_quality_min`, sonst die Bandrichtung (§8.6) |

**Längskoordinate** der Felder 12 und 15: `s = (x · vx + y · vy) / |v|` mit (vx, vy)
aus den Feldern 13/14 derselben Nachricht, Ursprung im Ursprung von `world`, wächst
stromabwärts. Erzeuger und Verbraucher rechnen sie mit derselben Funktion
`contracts.along_belt`.

**Gültigkeit:** Bei `has_target = 0` sind die Felder 2–11 und 16 bedeutungslos. Die
Felder 12–15 gelten, solange eine Bandschätzung vorliegt; ohne sie (`n_pool = 0` oder
Band unter 0,01 m/s) sind sie 0.

**Rückzug des Ziels** (`has_target` → 0): Ergebnis in `picked_id`, ID nicht mehr in
S3, Status zurück auf 3, keine Bandschätzung, S3 steht still. Der Follower bricht
dann ab — solange er den Klotz noch nicht hält (§8.5).

---

## S5 — `not_pickable` · priority_handler → data_tracker

```
[ t, n,  id_0, id_1, ... ]     Kopf 2, Stride 1
```

Die IDs aller aktuellen Tracks hinter der Greifebene, außer dem gewählten Ziel und
bereits versuchten IDs (ein gegriffener Klotz läuft als Status 4 weiter). Aktuelle
Liste, keine Historie; ohne Bandschätzung leer. Nur Buchhaltung für die Anzeige.

---

## S6 — `target_pose` · object_follower → Signal Point Attractor

Typ **`cartesian_pose`** (AICA prüft den Typ beim Verbinden; der Eingang `attractor`
erwartet `cartesian_pose`).

- `reference_frame`: `world`, ausdrücklich gesetzt.
- Zielpose des **Flansches**: vorhergesagte Klotzposition plus Vorhalt
  `v · lead_time_s`; beim Greifen in der Höhe Greifpunkt plus
  `flange_to_grip_point_m` (0,235 m). Orientierung senkrecht nach unten, Gier nach
  §8.6.

---

## S7 — `picked_id` · object_follower → priority_handler, data_tracker

Feste Länge 3: `[seq, id, outcome]`.

| # | Feld | Bemerkung |
|---|---|---|
| 0 | `seq` | zählt bei jedem abgeschlossenen Versuch hoch. **0 = noch kein Versuch** seit der Aktivierung des Followers; der erste trägt 1 |
| 1 | `id` | betroffene Objekt-ID |
| 2 | `outcome` | 0 abgelegt · 1 Fehlgriff · 2 Objekt verloren · 3 Greifebene überschritten, bevor abgesenkt wurde · 4 abgebrochen, bevor gegriffen wurde (Grund im Log) |

`outcome = 0` heißt: Der Klotz liegt in der Kiste — auch wenn der Follower mit Klotz
im Greifer abbrechen musste und ihn trotzdem abgelegt hat.

Verbraucher reagieren genau einmal auf jede neue `seq`, nie auf 0; ein Rücksprung auf
0 heißt, dass der Follower neu aktiviert wurde. Die Regel steckt in
`contracts.AttemptWatcher`.

---

## S8 — `follower_status` · object_follower → interface_streamer

Feste Länge 6. Reine Diagnose.

| # | Feld | Einheit | Bemerkung |
|---|---|---|---|
| 0 | `t` | s | |
| 1 | `state` | – | Zustandscode, siehe unten |
| 2 | `target_id` | – | 0 = keins |
| 3 | `err_laengs` | m | Flansch minus vorhergesagter Klotz entlang der Bandrichtung, **positiv = Flansch voraus** |
| 4 | `err_quer` | m | quer dazu, **positiv = links der Laufrichtung** |
| 5 | `err_z` | m | Flanschhöhe minus kommandierte Höhe |

| Code | Zustand | Code | Zustand |
|---|---|---|---|
| 0 | `WARTEN` | 5 | `HEBEN` |
| 1 | `ANFAHREN` | 6 | `ABLEGEN` |
| 2 | `FOLGEN` | 7 | `LOESEN` |
| 3 | `ABSENKEN` | 8 | `ABBRUCH` |
| 4 | `GREIFEN` | | |

Die Abweichungen sind ohne Vorhalt gerechnet; im eingeschwungenen `FOLGEN` ist
`err_laengs` das Maß für `lead_time_s` (§2.2). In Zuständen ohne Klotz (`WARTEN`,
`ABLEGEN`, `LOESEN`, `ABBRUCH`) sind die Abweichungen und `target_id` 0.

---

## S9 — Greifersignale

| Signal | Richtung | Typ | Bedeutung |
|---|---|---|---|
| `gripper_close` | object_follower → robotiq_gripper | `Bool` | true = schließen, false = öffnen |
| `motion_done` | robotiq_gripper → object_follower (`gripper_motion_done`) | `Bool` | Bewegung abgeschlossen, beim Schließen **und** beim Öffnen |
| `has_object` | robotiq_gripper → object_follower (`gripper_has_object`) | `Bool` | Objekt tatsächlich gefasst |
| `gripper_change` | → robotiq_gripper | `Int32` | Ziel-Öffnungsweite in mm; im Greifablauf nicht genutzt |

Die Predicates `is_connected` und `is_object_grasped` des Greifers dienen der
Oberfläche. `has_object` folgt dem Robotiq-Status und ist auch true, wenn die Backen
**beim Öffnen** anstoßen; `LOESEN` endet deshalb über `motion_done`, nicht über
`has_object = 0`.

---

## S10 — `world_state` · data_tracker → interface_streamer

```
[ t, n, v_belt_x, v_belt_y, n_pool,  <entry_0>, ... ]
  Kopf 5 wie S3                      Stride 17
```

Kopf aus S3 übernommen (die Anzeige zeigt die geschätzte Bandgeschwindigkeit).
Felder 0–13 wie S3, dazu:

| # | Feld | Bemerkung |
|---|---|---|
| 14 | `picked` | 1 = erfolgreich abgelegt (`outcome = 0`) |
| 15 | `out_of_bounds` | 1 = Greifebene ungegriffen überschritten; der Klotz fällt später am Bandende herunter |
| 16 | `present` | 1 = steht in den aktuellen `tracks` · 0 = **Nachlauf**: Felder 0–13 sind die letzten bekannten Werte |

Der Nachlauf ist nötig, weil ein gegriffener Klotz beim Heben aus dem Bild
verschwindet, `picked_id` aber erst nach dem Ablegen kommt.

**Verfall:** Ein Eintrag verschwindet `expiry_after_done_s` (10 s) nach dem Setzen von
`picked` oder `out_of_bounds`, ohne solchen Vermerk nach dem Verschwinden aus
`tracks`; gezählt in S3-Zeit. Ein Eintrag, der dabei noch in `tracks` steht, kommt
nicht wieder. Das Signal wächst so nicht über den Programmlauf.

---

## `contracts.py`

```python
# Je Signal: Kopflänge, Stride bzw. feste Länge, Feldindizes, NamedTuple, pack/unpack.
OBJECTS_HEADER, OBJECTS_STRIDE = 3, 9
def pack_objects(t, v_belt, objects) -> list: ...
def unpack_objects(arr) -> ObjectsMsg | None: ...

# Gemeinsame Vokabeln
TRACK_FINAL, TRACK_SETTLING, TRACK_PREDICTED, TRACK_SELECTABLE   # S3-Status 0, 3, 4
OUTCOME_PLACED … OUTCOME_ABORTED                                  # S7 outcome 0–4
STATE_WAIT … STATE_ABORT, FOLLOWER_STATES                         # S8 Zustände 0–8
COLOR_RED … COLOR_UNKNOWN                                         # S1 Farben 0–6

# Zwei Regeln, die Erzeuger und Verbraucher gleich rechnen müssen
def along_belt(x, y, v_belt) -> float: ...   # Längskoordinate der S4-Felder 12, 15
class AttemptWatcher: ...                    # S7: jede neue seq einmal, nie die 0
```

- `unpack_*` liefert `None` für ein leeres Signal (noch nichts empfangen) und wirft
  `ContractError` bei falscher Länge — kein Empfänger rechnet auf halben Daten.
- `unpack_*` rechnet nicht um; Vorhersage und Transformationen macht der Verbraucher.
- Das Modul ist frei von ROS, OpenCV und numpy.
