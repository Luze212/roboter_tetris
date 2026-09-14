# Design: Anpassungen an bestehenden Komponenten

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


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

### Warum 100 Hz — und was es kostet

Das Frame-Gating in `on_step_callback` (Zeile 261) kehrt sofort zurück, wenn der
Bildstempel unverändert ist. Die **Verarbeitung** ist damit tatsächlich fast
kostenlos.

> **Korrektur der Begründung.** Ursprünglich stand hier „halbiert den Jitter" mit
> Bezug auf den Zeitstempel. Das galt für die Notlösung, bei der `base_cam` selbst
> stempelt. Seit Thema 2 den Header-Stempel nutzt, trägt der Zeitstempel diesen
> Jitter nicht. Der tatsächliche Gewinn ist kleiner, aber real: Die Wartezeit
> zwischen Bildeingang und Verarbeitung sinkt von 0–20 ms auf 0–10 ms, und dieser
> Anteil steckt im Regelkreis. 100 Hz bleibt richtig.

### ⚠️ Zwingend dazu: `debug_image` nicht mehr je Takt publizieren

AICA verschickt Ausgangsvariablen in **jedem** Takt. `debug_image` ist ein
normaler Output — bei 100 Hz verdreifacht sich der Bildverkehr gegenüber der
Kamerarate. Im Produktivbetrieb folgenlos (bei ausgeschaltetem Debug ist die
Variable ein leeres `Image()`), aber **genau bei der Inbetriebnahme** schädlich:
Die sechs Stufen von B6 arbeiten vollständig mit eingeschaltetem Debug-Bild, und
ausbleibende Frames würde man dort der Erkennung anlasten statt dem Publizieren.

```python
self.add_output("debug_image", "_debug_msg", Image, publish_on_step=False)
...
self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
self.publish_output("debug_image")      # nur nach echtem Rendern
```

Gilt für `base_cam`, `robot_cam` und `robot_cam_2` gleichermaßen
(`entscheidungen.md`, Nachtrag 3 / N5).

### ⚠️ Messregion des Trackers ist ab jetzt regelungsrelevant

`track_velocity_region_y_min` / `_max` waren bisher reine Tracker-Feinheit. Sie
entscheiden aber darüber, ob die Längsposition **gemessen oder gekoppelt** wird und
ob Tracks bei ausbleibender Detektion gelöscht werden. Sie werden deshalb in
**B19 gemeinsam mit der Greifzone** festgelegt — Region = Greifzone plus Rand.
Begründung: `entscheidungen.md`, Nachtrag 3 / N2.

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

### Zusätzlich: Zeitdomäne der Kamera erzwingen (umgesetzt 14.09.2026)

Neuer Parameter **`camera_node`** (string, Default leer). Ist er gesetzt, legt
`on_configure_callback` einen Service-Client auf `<camera_node>/set_parameters` an,
und `on_step_callback` setzt darüber `rgb_camera.global_time_enabled` und
`depth_module.global_time_enabled` auf `true` — nicht blockierend über
`service_is_ready()` und `call_async` mit Done-Callback, mit Wiederholung, weil die
Kamera-Node später hochkommen kann (`ARCHITECTURE.md` §3).

**Warum das sein muss:** Bei der L515 steht die Option per Treiber-Default auf
`false`. Die Kamera stempelt dann in ihrer Hardwareuhr, driftet mit rund 4 ms/s
gegen die ROS-Zeit und bleibt nach einigen Minuten ganz stehen — dann verwirft das
Frame-Gating jedes Bild nach dem ersten und die Objektliste ist still leer. Am
14.09.2026 gemessen und nach dem Setzen behoben (`entscheidungen.md`, Nachtrag 4).
Der AICA-Block exponiert den Parameter nicht, deshalb der Weg über die Komponente.

### Dateien
`roboter_tetris/base_cam.py` · `component_descriptions/roboter_tetris_base_cam.json` · `package.xml` (`rcl_interfaces`)

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

> ⚠️ **Auch nicht die Rückprojektion.** `localize_largest_blob` projiziert `x`/`y`
> mit der Banddistanz zurück, obwohl der Punkt auf der Klotzoberseite liegt —
> beide Werte sind dadurch um `z_band/(z_band − blockhoehe)` zu groß. Das ist ein
> echter Fehler von 6…39 mm, er wird aber **bewusst im `object_follower`
> korrigiert**, nicht hier: Die Komponente kennt die Klotzhöhe nicht, und der
> gemeinsame Kern beider Varianten soll für den A/B-Test unangetastet bleiben.
> Siehe `entscheidungen.md`, Nachtrag 3 / N1 und `datenvertraege.md` unter S2.
> **Nicht „reparieren" — sonst wird doppelt korrigiert.**

### `debug_image` auch hier mit `publish_on_step=False`
Gleiche Begründung wie bei `base_cam` (Nachtrag 3 / N5).

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
