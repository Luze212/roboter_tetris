# Design: Anpassungen an bestehenden Komponenten

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung

## Zweck

Drei bereits funktionierende Komponenten müssen auf die Datenverträge gebracht
werden. Alle Änderungen sind klein, einzeln testbar und **berühren die
Bildverarbeitung nicht**.

> Grundsatz: Die Algorithmik in `roboter_tetris/vision/` bleibt unangetastet.
> Geändert wird ausschließlich, **wie** Ergebnisse ausgegeben werden.

---

## 1. `base_cam`

### Änderung

| Was | Ist-Stand | Soll |
|---|---|---|
| Format | `[id, color, x, y, z, orientation, vy, length, width, height]`, Stride 10, kein Kopf | `[t, n, v_band] + n×9` (S1) |
| Zeitstempel | fehlt in der Ausgabe (intern vorhanden) | im Kopf, aus `header.stamp` |
| Einheiten | mm | **m** |
| `vy` | je Objekt | in den **Kopf** |
| Rate | 50 Hz | **100 Hz** |

### Warum `vy` in den Kopf

Der Tracker weist die Geschwindigkeit **global allen Tracks** zu
(`set_global_velocity` in `on_step_callback`). Als Objektfeld ist der Wert
irreführend — ein festhängender Block bekäme trotzdem die volle
Bandgeschwindigkeit eingetragen. Als Kopffeld ist er nützlich: Quelle für die
Bandkalibrierung (B1) und laufende Kontrolle, ob das Band überhaupt läuft.

Die Plausibilitätsprüfung je Objekt rechnet `vectoring` aus der eigenen
Messhistorie.

### Warum 100 Hz kostenlos ist

Das Frame-Gating in `on_step_callback` (Zeile 261) kehrt sofort zurück, wenn der
Bildstempel unverändert ist. Eine höhere Rate kostet damit nur den
Callback-Aufruf — und halbiert den Jitter zwischen Bildeingang und Verarbeitung.

### Zeitstempel

Der geplante Weg ("ROS-Zeit im Kamerablock abfragen") ist nicht nötig. Die
Bildnachrichten tragen bereits einen Header-Stempel, und die Komponente nutzt ihn
längst — für das Frame-Gating und als Zeitbasis des Trackers.

Dass er läuft, folgt aus der Funktionsfähigkeit: Wäre er konstant, würde das
Gating jedes Bild nach dem ersten verwerfen.

> **Vorbedingung B13:** Prüfen, ob der Kamerastempel gegen die ROS-Zeit driftet.
> RealSense-Kameras stempeln je nach Konfiguration in einer `HARDWARE_CLOCK`-
> Domäne, die nur einmal beim ersten Bild synchronisiert wird. Konstanter Versatz
> ist unproblematisch (steckt in `latency_compensation_s`); **Drift** nicht.
> Notlösung, falls unbrauchbar: in der Komponente selbst stempeln — der
> ursprüngliche Plan. Kostet Jitter (wenige mm) statt Drift (bis 60 mm).

### Dateien
`roboter_tetris/base_cam.py` · `component_descriptions/roboter_tetris_base_cam.json`

### Abnahme
Bestehende Tests laufen unverändert (sie prüfen nur `vision/*`). Neuer Test prüft
das Packen gegen `contracts.py`.

---

## 2. `robot_cam` und `robot_cam_2`

### Änderung

| Was | Ist-Stand | Soll |
|---|---|---|
| Format | `[t, x, y, z, orientation]` | `[t, valid, x, y, z_band, orientation]` (S2) |
| Gültigkeit | leere Liste, wenn nichts erkannt | **`valid`-Flag**, `t` läuft weiter |
| Distanzgate | fehlt | neue Parameter `min_belt_distance_m` / `max_belt_distance_m` |
| Einheiten | mm | **m** |
| Rate | 50 Hz | **100 Hz** |

### Warum das `valid`-Flag wichtig ist

Mit einer leeren Liste lassen sich zwei völlig verschiedene Lagen nicht
unterscheiden: **"Kamera arbeitet, sieht gerade nichts"** und **"Kamera liefert
nicht mehr"**. Im ersten Fall soll der Follower auf `w = 0` zurückfallen und
weiterfahren, im zweiten Fall abbrechen.

Deshalb: Bei `valid = 0` sind die Felder 2–5 bedeutungslos, **`t` läuft aber
weiter**.

### Distanzgate (Befund R3)

Der Plan nannte "Min-Distanz / Max-Distanz" ohne Bezug. Gemeint ist der gemessene
Abstand **Kamera → Bandoberfläche** (`z_band`). Außerhalb des Fensters →
`valid = 0`:

- zu hoch: zu wenig Bildauflösung auf dem Block
- zu tief: Tiefenbereich verlassen oder Block nicht mehr vollständig im Bild

Verknüpft sich mit B8 — die Beobachtungshöhe muss in diesem Fenster liegen.

### Beide Varianten

`robot_cam` (farbbasiert) und `robot_cam_2` (kantenbasiert) liefern **denselben
Vertrag** und bleiben gegeneinander austauschbar. Welche produktiv läuft, wird im
Graphen verdrahtet — der Rest des Systems merkt keinen Unterschied.

### Nicht ändern
`vision/robot_detection.py`, `vision/robot_detection_edge.py`. Die
Kantendetektion wird separat abgestimmt (B6).

### Dateien
`roboter_tetris/robot_cam.py` · `robot_cam_2.py` · beide JSONs

---

## 3. `robotiq_gripper`

### Änderung

Zwei neue `Bool`-**Ausgänge**:

| Signal | Bedeutung |
|---|---|
| `is_closed` | Bewegung abgeschlossen |
| `has_object` | Objekt tatsächlich gefasst |

### Warum

Der Plan sah nur "Greifer zu" vor, beschrieben als *"Ist geschlossen, solange
Greifen = true und gibt dabei selbst auf Greifer zu = true aus."* Das ist ein
**Echo des Eingangs** und als Rückmeldung wertlos — es sagt nichts darüber, ob
der Griff geglückt ist.

Der `object_follower` wertet `has_object` aus. Bleibt es nach dem Schließen aus,
war es ein Fehlgriff → `ABBRUCH` mit `outcome = 1`, statt in `GREIFEN` hängen zu
bleiben.

Die vorhandenen Predicates `is_connected` / `is_object_grasped` bleiben für die
UI erhalten. Die neuen Signale spiegeln dieselbe Information als Datenfluss —
Predicates lassen sich in AICA nicht in einen Daten-Input verdrahten.

### Nicht ändern

- Worker-Thread und das Muster "Port pro Operation öffnen und schließen"
  (`ARCHITECTURE.md` §13) — das ist erprobt und löst ein reales Problem
- Bring-up, Timeouts, Parameter
- Das Öffnen des Greifers beim Bring-up (§13 Regel 6)

### Bekanntes Verhalten

Wird die Anwendung mit einem Block im Greifer gestoppt, bleibt der Block
gegriffen. Beim nächsten Hochfahren öffnet die Komponente im Bring-up, und der
Block fällt dort hin, wo der Arm steht. Kein Fehler, aber vorher zu wissen.

### Dateien
`roboter_tetris/robotiq_gripper.py` · `component_descriptions/roboter_tetris_robotiq_gripper.json`

### Abnahme
Bestehender Test läuft weiter; neuer Test prüft, dass `has_object` dem Predicate
folgt.

---

## Reihenfolge

Alle drei Anpassungen setzen **nur** `contracts.py` voraus (Phase 1) und sind
untereinander unabhängig. Sie können parallel oder in beliebiger Reihenfolge
erfolgen.
