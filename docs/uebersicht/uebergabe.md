# Übergabe — Stand 15.09.2026

Dieses Dokument ist der Einstieg, wenn die Arbeit **auf einem anderen Rechner**
fortgesetzt wird. Es fasst zusammen, was gilt, was gemessen ist und was als
Nächstes ansteht. Es ersetzt keines der Fachdokumente, sondern verweist auf sie.

> ⚠️ **`CLAUDE.md` liegt in `.gitignore`** und kommt deshalb **nicht** mit dem
> Push auf dem anderen System an. Die Arbeitsregeln daraus stehen darum unten in
> Abschnitt 1 noch einmal. Soll `CLAUDE.md` selbst mitwandern, muss sie entweder
> aus `.gitignore` genommen oder von Hand kopiert werden — das ist eine
> bewusste Entscheidung und wurde hier nicht eigenmächtig getroffen.

---

## 1. Arbeitsregeln (gelten unverändert)

| Regel | Bedeutung |
|---|---|
| **Zwei Archive** | `roboter_tetris` ist Haupt- und Arbeitsarchiv. `UR10_Pick_ws` ist **read-only** — dort wird nichts verändert, nur ausgelesen. |
| **Kein Git durch den Assistenten** | Kein `commit`, `push`, `fetch`, `pull`, `merge`, `checkout`. Git bedient ausschließlich der Nutzer. |
| **Kein Bauen durch den Assistenten** | Docker-Images bauen, taggen, entfernen und in AICA laden macht ausschließlich der Nutzer. In diesen Zyklus wird nicht eingegriffen. |
| **`ARCHITECTURE.md` ist bindend** | Die AICA-Regeln dort gelten beim Anlegen *und* beim Pflegen von Komponenten. |
| **Nicht selbstständig weiterarbeiten** | Jeder Umsetzungsschritt wird ausdrücklich vorgegeben. |

**Nicht anfassen:** `roboter_tetris/Calibration/*` (fremdes Projekt eines
Kommilitonen), `roboter_tetris/vision/*` (Bildverarbeitung steht; geändert wird
nur, *wie* Ergebnisse ausgegeben werden), `.init_wizard/`, `CMakeLists.txt`.

---

## 2. Wo das Projekt steht

**Phase 0 des Umsetzungsplans ist abgeschlossen.** Alle vier Punkte, die Phase 4
blockierten, sind geklärt: `target_pose` ist `cartesian_pose` (A2), der Greifer
steht nicht im URDF und geregelt wird `ur_tool0` (A7/A8), der Werkzeugversatz
der Steuerung ist ausgelesen (C8), und der Uhrendrift der Basiskamera ist durch
die Komponente selbst dauerhaft behoben (B12/B13/B14).

**Läuft am Aufbau:** `robotiq_gripper`, `base_cam`, die AICA-Kette
Attractor → IK-Velocity-Controller.
**Implementiert, ungetestet:** `robot_cam` (farbbasiert), `robot_cam_2`
(kantenbasiert) — der A/B-Test B6 ist offen.
**Noch nicht angelegt:** `contracts.py`, `vectoring.py`, `data_tracker.py`,
`priority_handler.py`, `object_follower.py`, `interface_streamer.py`.

### Noch rot

| Punkt | Warum blockiert |
|---|---|
| **B1** — Bandgeschwindigkeit | Das Band konnte nicht laufen (Zettel eines parallelen Kalibrierprojekts liegen darauf). Die **Richtung** ist geklärt: praktisch die y-Achse. |
| **C3** — Extrinsik der Basiskamera | Läuft als Projekt eines Kommilitonen. Die Basiskamera steht auf einem **beweglichen** Gestell, deshalb wird die Bestimmung automatisiert. |

### Der offene Kernpunkt: `robot_cam`

Beide Varianten scheitern **nicht an der Erkennung, sondern an der Auswahl**.
Der Code nimmt die flächengrößte qualifizierte Kontur; die Doku ging von der
bildzentrumsnächsten aus. Die Farbvariante wählt dadurch einen bildfüllenden
Blob aus Glanz plus Klotz, die Kantenvariante die Maschinenstruktur am Bildrand.
Beides erzeugt **plausibel aussehende Fehlwerte**, die `max_korrektur_m` nicht
abfängt.

Drei Gegenmaßnahmen für Phase 2.2, alle in `localize_largest_blob` und der
Parameterliste, ohne die Detektionskerne zu berühren: Auswahl nach Nähe zum
**Erwartungspunkt** (rechenbar, weil die Roboterkamera-Halterung seit der
Vorgängergruppe unverändert ist), eine **ROI**, und eine **`max_contour_area`**.
Einzelheiten: `architektur/robot-cam-befunde.md` §9.

---

## 3. Gemessene Werte des Aufbaus

Vollständige Tabelle mit Herkunft: `uebersicht/einrichtung-projektanwendung.md`
Abschnitt 8. Herleitung und Unsicherheiten: `architektur/entscheidungen.md`
Nachträge 4 und 5.

> ⚠️ **Alle Höhen sind Flanschmaße** (`ur_tool0`), nicht TCP-Maße der
> UR-Steuerung. Wer hier einen TCP-Wert einsetzt, liegt um 215 mm daneben.

| Wert | Zahl |
|---|---|
| Bandoberfläche in `world` | 53,6 mm (±1 mm eben, ±5 mm systematisch) |
| Flansch → Backenspitze (geschlossen) | 245 mm |
| Flansch → Griffpunkt | 235 mm |
| Greifhöhe Flansch, 100-mm-Klotz stehend | ≈ 339 mm |
| Ablagepose Flansch | x −316,49 · y +476,21 · z +419,71 mm |
| Greiferöffnung | 127 mm |
| Backenauflage | 20 mm hoch, 15 mm breit |
| Bandrichtung | praktisch die y-Achse |

Genauigkeit der Basiskamera bei ruhendem Klotz, 299 Messungen: Position
σ = 0,2…0,6 mm, Höhe σ = 0,6 mm. Die Grundfläche streut dagegen über 26 mm —
Grundflächenmaße gehören auf **geglättete** Werte, nie auf Einzelbilder.

---

## 4. Randbedingungen, die leicht untergehen

**Messungen im Container laufen als `ros2`.** FastDDS gibt die Shared-Memory-
Segmente dem Nutzer, dem sie gehören; als root sind die Topics zwar sichtbar,
es kommen aber keine Daten an.

```bash
docker exec -u ros2 -e ROS_DOMAIN_ID=0 <container> bash -lc 'ros2 topic list'
```

Das leere `ROS_DOMAIN_ID` lässt die `ros2`-CLI abstürzen, deshalb `-e ROS_DOMAIN_ID=0`.

**Kamera-Knotennamen wechseln je Anwendung.** Immer über `serial_no` prüfen,
nie über den Knotennamen: Basiskamera `f1370107` (L515), Roboterkamera
`241122074842` (D435i).

**`global_time_enabled` und `enable_auto_exposure` exponiert der AICA-Block nicht.**
Die Zeitdomäne erzwingt `base_cam` inzwischen selbst über den Parameter
`camera_node`. Die Belichtung muss zur Laufzeit gesetzt werden.

---

## 5. Roboterzustand mitlesen (read-only)

Beide Skripte laufen im Container und verändern nichts. `cartesian_state`
liefert die **Flanschpose**, nicht den TCP.

```python
# read_cart.py — Median und Streuung über N Nachrichten
import sys, statistics, rclpy
from rclpy.node import Node
from modulo_interfaces.msg import EncodedState
import clproto

N = int(sys.argv[1]) if len(sys.argv) > 1 else 200

class R(Node):
    def __init__(self):
        super().__init__("cart_reader")
        self.samples = []
        self.create_subscription(EncodedState,
            "/hardware/robot_state_broadcaster/cartesian_state", self.cb, 10)
    def cb(self, msg):
        st = clproto.decode(bytes(msg.data))
        p, q = st.get_position(), st.get_orientation()
        self.samples.append((p[0], p[1], p[2], q.w, q.x, q.y, q.z))
        if len(self.samples) >= N:
            raise SystemExit

rclpy.init(); n = R()
try: rclpy.spin(n)
except SystemExit: pass
for i, l in enumerate(["x", "y", "z", "qw", "qx", "qy", "qz"]):
    v = [r[i] for r in n.samples]
    print(f"{l:3s} median={statistics.median(v): .6f}  sigma={statistics.pstdev(v): .6f}")
```

Für die Gelenke dasselbe mit dem Topic `.../joint_state` und
`st.get_names()` / `st.get_positions()`.

⚠️ **Plausibilitätsprüfung:** Ist die Streuung über alle Nachrichten **exakt
null**, speist der Treiber keine frischen RTDE-Daten mehr ein (Programm auf dem
Pendant gestoppt) — die Werte sind dann ein eingefrorener Altstand.

---

## 6. Was als Nächstes ansteht

**Am Schreibtisch, ohne Hardware:**

1. **`contracts.py`** (Umsetzungsplan Phase 1.1). Hängt an keinem offenen Punkt.
   S4 hat 13 Felder, `target_pose` ist `cartesian_pose`.
2. **Überarbeitung von `robot_cam`** (Phase 2.2) — die drei Gegenmaßnahmen aus
   Abschnitt 2.
3. **`Komponentenplan.docx`** nachziehen. Sechs Passagen sind überholt, unter
   anderem der Signaltyp von `target_pose` und die Beschreibung der Greifhöhe.

**Am Aufbau, sobald das Band verfügbar ist:**

4. **B1** — Bandgeschwindigkeit. Ein normaler Testlauf mit mehreren Klötzen
   reicht; das `objects`-Signal mitschneiden und am Schreibtisch auswerten.
   ⚠️ Der Tracker verwirft Geschwindigkeiten unter 30 mm/s.
5. **B19 + B11** — Greifzone und Singularitäten. Den Arm die Bandstrecke
   abfahren lassen und dabei die Gelenkgeschwindigkeiten mitlesen.
6. **B6 Stufe 3** — A/B-Test der Roboterkamera; Voraussetzung für B8 und D18.
7. **Gegenprobe der Ablagepose** bei laufendem Programm (Abschnitt 3).

---

## 7. Dokumentenlandkarte

Aufteilung und Vorrangregeln stehen in `docs/README.md`. Kurz: `uebersicht/`
für Betrieb und Überblick, `architektur/` für die technische Grundlage,
`archiv/` wird nicht mehr gepflegt. **Normativ sind `entscheidungen.md` und
`datenvertraege.md`**; die Specs sind daraus abgeleitet und werden gelöscht,
sobald die jeweilige Komponente gebaut ist.
