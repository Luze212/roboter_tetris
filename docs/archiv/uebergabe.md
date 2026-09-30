# Übergabe — Stand 24.09.2026

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
Kommilitonen) und `vision/board.py`, das die Kalibrierung nutzt; `.init_wizard/`,
`CMakeLists.txt`.

**Geändert werden dürfen** die Komponenten rund um Kameras und Greifer (Vorgabe
vom 21.09.2026). Die Roboterkamera ist seit 24.09.2026 **nicht mehr eingebunden**
(Nachtrag 13 / L22); ihre Blöcke bleiben im Paket.

---

## 2. Wo das Projekt steht

**Die Kette greift am Aufbau** Klötze vom laufenden Band und legt sie ab (L19,
L20). Stand und nächste Schritte: **`fahrplan-aufbau.md`**
(„Stand nach Termin C“). Wie die Teile zusammenhängen: `uebersicht/systemgraph.md`.

| | Komponente | Stand |
|---|---|---|
| ✅ läuft am Aufbau | `base_cam` (Übergangskalibrierung, B23 erledigt), `vectoring`, `priority_handler`, `data_tracker`, `robotiq_gripper`, Attractor → IK-Velocity-Controller | Datenpfad bis `data_tracker` geprüft (Nachtrag 12 / K4); Schätzung mit laufendem Band bestätigt (L9, L10). Attractor von Hand auf K = 5 / 50 Hz (L18) |
| ✅ greift im Lauf | `object_follower` (4a, 4b, 4d), `fake_objects.py` | Gedrosselt mit Fake bestanden (L18); **erste echte Griffe mit Ablage am 24.09.** (L19), `err_laengs` ≈ +1 mm, 7 von 7 abgelegt (L20). Stufe 4c entfernt (L22). Mit 0,5 m/s: 15 abgelegt, ~1 Klotz je 7 s, flache Klötze greifbar (L24) |
| ✅ läuft am Aufbau | `interface_streamer` | nur mit dem Bild der Basiskamera (L22), `rate` 10 für den Vortrag (L24) |
| ⏸ nicht eingebunden | `robot_cam_2`, `robot_cam` | Im Paket, aber von keiner Komponente mehr abgenommen (L22). Stand bis dahin: Banddistanz und Nah-Gate korrigiert, Hand-Auge neu (L11–L13), Erkennung unzuverlässig |

### ⚠️ Zuerst lesen: was sich am 24.09. verschoben hat

Nachtrag 13 / L15–L24:

- **Tempo ausgereizt, flache Klötze greifbar** (L24): L23 war zu pessimistisch — aus
  der Ablagepose wurde kein Klotz in der Zone mehr gewählt. Gemessen und neu als
  Standardwert: `t_settle_s` 0,4, Faktor 1,0 (darunter Fehlversuche), `t_descend_s` 0,9
  bei 0,25 m/s Absenken, `t_grasp_s` 0,8, Anfahren 0,5 m/s (Attractor und IK von Hand).
  Ergebnis: 15 abgelegt, 0 zu spät, ~1 Klotz je 7 s. Klötze ab 2 cm werden gegriffen
  (`min_graspable_height_m` 0,02, `min_contour_area` 1000); geschlossene Backen dabei
  11 mm über dem Band (seit L26: 6 mm).

- **Vorhersagedeckel 0,6 → 1,0 s, Ziel-Timeout 1,5 s** (L23): Mit 0,6 wurde die
  Vorhersage einen Teil der Zeit eingefroren, das Absenken brach ab. Danach 9 von 9.
  Der `priority_handler` rechnet die Einschwingzeit des Followers mit (`t_settle_s`,
  seit L24 0,4 s) und wählt nur noch erreichbare Klötze.

- **Die Roboterkamera ist nicht mehr eingebunden** (L22): Follower ohne Stufe 4c,
  Streamer ohne Roboterkamerabild, S8 mit 6 Feldern. Das Verhalten des Followers
  ist nachweislich unverändert (27 200 Takte identisch). Nach dem Build Follower
  und Streamer neu einfügen.
- **Greifzone = Arbeitsraum, Bildausschnitt der Basiskamera angepasst** (L21);
  Schutzstopp am Bandrand behoben: UR-Nutzlast 1,3 kg, IK-Beschleunigungsgrenze (L20).

- **Pick-on-the-fly ist am Aufbau bestätigt:** erst zwei von zwei (L19), nach
  Nutzlast- und Beschleunigungskorrektur insgesamt **sieben von sieben
  greifbare Klötze** (L20): Basiskamera, etwa 0,13 m/s, greifen, heben,
  ablegen. Ablagepose und 245 mm sind bestätigt.

- **Der Follower hatte nie Signale** — seine Parameterprüfung scheiterte an den
  Topic-Parametern von modulo (L17). Korrigiert; erst seitdem läuft er verdrahtet.
- **Block 7 gedrosselt bestanden** (L18): voller Zyklus bis zum Fehlgriff mit
  `fake_objects.py` (stempelt jetzt in ROS-Zeit). Der Attractor stand auf dem
  AICA-Standard K = 1 (70 mm Nachlauf) → **K = 5 / 50 Hz von Hand**; `lead_time_s`
  0,24. Die IK-Grenze gilt für den Betrag — gedrosselt nur mit langsamem Absenken.
- **Rechner an der Grenze:** keine NVIDIA-GPU, Kameras auf 15 Bilder/s, die
  500-Hz-Schleife läuft nur mit 84–86 % und brach einmal ein (External Control
  stoppte). Das automatische Abschalten der Infrarotbilder hängte AICA auf (L16).
- Arbeitsraum und Beobachtungspose sind Standardwert (L15); Follower-Parameter in
  neun Gruppen mit Klartextnamen.

### ⚠️ Was sich am 22./23.09. verschoben hat

Nachträge 12 und 13 in `architektur/entscheidungen.md`:

- **`world` und der UR-Rahmen `base` sind um 180° gedreht** (K5). Alle Werte der
  Vorgängergruppe (Kalibrierung, `pose.yaml`) liegen in `base`.
- **Die Basiskamera schaut senkrecht aufs Band**; die Altkalibrierung war in
  Neigung und Gier falsch. **Übergangskalibrierung** als Standardwert, Position
  ≤ 6 mm auch für hohe Klötze (L6). Die Detektion rechnete die Ecken auf Bandhöhe
  (Parallaxe) — behoben; die Höhe kommt jetzt aus der Kalibrierung, dazu
  `top_depth_bias_mm` = 11,5.
- **Alle Python-Komponenten teilen sich einen Prozess** (GIL). Nicht gebrauchte
  Blöcke raus, Rates niedrig; Bild-Warteschlangen jetzt Tiefe 1 → ~7 Messungen/s,
  139 ms Alter (L2).
- **Greifzone und Wartebereich liegen außerhalb des Bildes der Basiskamera**
  (L4) — sonst erkennt sie den Greifer als Klötze. Die Strecke dahinter
  überbrückt die Roboterkamera.
- **Band ≈ 0,13 m/s** (Stoppuhr), von y +1,08 bis −0,375 in `world` (L7).
- **Hinter dem Bild führt `vectoring` weiter** (S3-Status 4 „vorhergesagt“, L10);
  Pool nur aus bewegten Klötzen; Band −127,9 mm/s geschätzt — **Ziel 3 bestätigt**.
  Follower-Deckel 0,6 s, Zeitgrenzen 1,0 s — seit L23 1,0 / 1,5 s.
- **Hand-Auge der Roboterkamera neu eingemessen** (Vorgänger lag in z 13 cm
  daneben, L11); die Erkennung von `robot_cam_2` ist an flachen Klötzen noch
  unzuverlässig (L13) → erster Griff ohne Roboterkamera.
- **Arbeitsraum und Greifzone festgelegt** (L14, `Safety/workspace_bounds.json`),
  `min_grip_height_m` 0,021 (seit L26 0,016); Arbeitsraum und Beobachtungspose seit 24.09. als
  Standardwert im Follower (L15).

### Was sich am 21.09. verschoben hat

### ⚠️ Zuerst lesen: was sich am 21.09. verschoben hat

Die Grundlage hat sich an einem Tag mehrfach bewegt. Das Wichtigste, jeweils mit
Fundstelle in `architektur/entscheidungen.md`:

- **Die Bandgeschwindigkeit wird geschätzt, nicht kalibriert** (Nachtrag 6 / Z2).
  `vectoring` schätzt je Klotz einen Geschwindigkeitsvektor; ein Klotz ist
  *einschwingend* (Status 3), bis zwei aufeinanderfolgende Halbsekunden dieselbe
  Geschwindigkeit messen, dann *final* (Status 0). Die finalen Klötze eines
  Durchlaufs ergeben gemeinsam die Bandgeschwindigkeit. Festhängende Klötze gibt
  es nicht — Status 1 und 2 sind stillgelegt.
- **Flansch → Griffpunkt ist 0,235 m** (`flange_to_grip_point_m`), nicht 0,215
  (Z7). Die 215 mm sind der TCP der UR-Steuerung.
- **Greifen geht ohne Roboterkamera** (Z10) — am Aufbau bestätigt (L19); seit L22
  ist sie gar nicht mehr eingebunden.
- **Die Greifebene** (Z11): Bis dorthin muss das Absenken begonnen haben, sonst
  bricht der Follower mit `outcome = 3` ab. Ziele dürfen stromaufwärts der Zone
  gewählt werden.
- **Mit Klotz im Greifer wird nichts fallen gelassen** (Z12), auch nicht bei einem
  Abbruch.
- ~~Basiskamera und Roboter sehen das Band an verschiedenen Stellen~~ (F1,
  **B23**) — erledigt 23.09.2026 (Nachtrag 13 / L6).
- **Arbeitsraum und Beobachtungspose sind Pflichtparameter** (F3) — seit L15 mit
  den am Aufbau festgelegten Werten als Standard; geleert lässt sich der Follower
  nicht konfigurieren. Werte: Einrichtung §9.
- **Freihöhe 0,49 m** statt 0,445 (Nachtrag 10 / J1) und **`t_descend_s` 2,0 s**,
  gekoppelt an die Sinkgeschwindigkeit 0,15 m/s (J2). Seit 24.09. **1,2 s** für
  `observe_z` 0,45 (L18), seit L24 **0,9 s** bei 0,25 m/s.
- **Neu in den Verträgen:** S3/S4/S10 erweitert, S4 mit Greifebene und Güte,
  S7 `outcome = 4` (vorher abgebrochen), S8 Zustandscodes, S10 `present`.

### Kritisch am Aufbau

| Punkt | Warum |
|---|---|
| ✅ **B21** — misst der Tracker außerhalb der alten Region sauber? | Ja: Messregion über das ganze Band, Pool −127,9 mm/s (Nachtrag 13 / L9, L10). |
| 🔴 **C3** — Extrinsik der Basiskamera | Läuft als Projekt eines Kommilitonen. Die Basiskamera steht auf einem **beweglichen** Gestell, deshalb wird die Bestimmung automatisiert. **Überbrückt** durch die Übergangskalibrierung (Nachtrag 13 / L6) — wird die Kamera bewegt, gilt sie nicht mehr. |
| ✅ **B23** — Basiskamera und Roboter im selben System | Erledigt 23.09.2026: fünf Antastpunkte, Rest ≤ 6 mm (Nachtrag 13 / L6). |
| ✅ **Latenz im Follower** | `max_extrapolation_s` **1,0 s**, `target_timeout_s` 1,5 s (Nachtrag 13 / L23; S4 bis 0,93 s alt). Mit 0,6 brach das Absenken immer wieder ab. |
| 🔴 **Einbrüche der 500-Hz-Schleife** | Rechner zeitweise voll; ein Einbruch stoppt External Control, der Arm bleibt stehen (L18). Oberfläche minimieren, Nebenprogramme beenden, Mitlesen leicht halten. |

> **B1 ist nicht mehr rot.** Die Bandgeschwindigkeit wird geschätzt; B1 ist nur
> noch die Gegenprobe mit der Stoppuhr.

### `robot_cam_2` — nicht mehr eingebunden (L22)

Am 15.09. scheiterten beide Varianten an der **Auswahl**, am 22.09. an der
**Banddistanz** (K3); seit 23.09. ist sie der Median der Tiefe über das Bild (L5).
Hand-Auge neu eingemessen (L11), die Erkennung blieb an flachen Klötzen
unzuverlässig (L13). Da ohne Roboterkamera gegriffen wird, ist sie seit 24.09.
nicht mehr eingebunden. Einzelheiten: `architektur/robot-cam-befunde.md` §9.

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
| Bandrichtung | praktisch die y-Achse; das Band läuft von y +1,08 nach −0,375 (L7) |
| Band im Robotersystem (angetastet) | x −0,70 … −0,93 m, y −0,20 … +0,87 m; im Bild der Basiskamera ~0,8 m breit, x ≈ −0,49 … −1,25 (L21) |
| Bandgeschwindigkeit (Stoppuhr) | ≈ 0,13 m/s |
| Extrinsik Basiskamera (`world`) | −0,7787 · 0,7934 · 0,9163 · 179,46° · 0,45° · 179,76° |
| Bild der Basiskamera | y ≈ 0,46 … 1,03 m |
| Freihöhe Flansch (Transfer, Abbruch) | 0,49 m (J1) |
| Arbeitsraum = Greifzone | x −1,0 … −0,53 · y −0,32 … +0,445 (Arbeitsraum für die Ablage bis x −0,30 / y +0,48), z 0,3036 … 0,60 (L14, L21) |
| UR-Nutzlast | 1,3 kg, Schwerpunkt 12 / 24 / 45 mm (L20) |
| Prozesszeiten (L24) | Einschwingen 0,23–0,33 s · Absenken 0,78–0,93 s bei 0,25 m/s · Greifen 0,63–0,83 s · Ablegen 1,3–2,2 s bei 0,5 m/s; ~1 Klotz je 7 s |
| Flacher Klotz 50 × 75 × 25 mm | gemessen 23,8 mm, greifbar ab 20 mm; geschlossene Backen 6 mm über dem Band (L24, L26) |

Genauigkeit der Basiskamera bei ruhendem Klotz, 299 Messungen: Position
σ = 0,2…0,6 mm, Höhe σ = 0,6 mm. Die Grundfläche streut dagegen über 26 mm —
Grundflächenmaße gehören auf **geglättete** Werte, nie auf Einzelbilder.
Absolut gegen den Roboter (23.09.): ≤ 6 mm. Höhe mit `top_depth_bias_mm` durch die
Greifhöhen der ersten echten Griffe bestätigt (≈ 96 bzw. 74 mm, L19).

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

**Mitlesen belastet den Rechner.** Schon ein zusätzlicher Leseprozess drückt den
500-Hz-Regelkreis zum Roboter (Nachtrag 13 / L3) — immer nur einer gleichzeitig.

**Von Hand gesetzte Blockparameter gehen vor.** Neue Standardwerte des Pakets
übernimmt nach dem Build auch ein bestehender Block — für jeden Parameter, der
dort auf dem Standardwert steht.

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

### Am Schreibtisch — erledigt

| Schritt | Was | Entscheidungen |
|---|---|---|
| 1.1 | `contracts.py` — alle Signale S1–S10 | Nachtrag 6 |
| 2.1–2.4 | `base_cam` (S1, Tracker misst die Längsposition), `robot_cam` (Auswahl), Greifer (`motion_done`, `has_object`) | Nachtrag 6 / Z4 |
| 3.0–3.3 | `fake_objects.py`, `vectoring`, `priority_handler`, `data_tracker` | Nachtrag 7 |
| 4a–4d | `object_follower`: Start, Folgen, Roboterkamera (4c, seit L22 entfernt), Greifzyklus | Nachträge 8–10 |
| 5 | `interface_streamer` | Nachtrag 11 |

Der **Komponentenplan** (`Komponentenplan Robotetris - Stand 2026-09-24.docx`) ist auf
den finalen Stand nachgezogen (L26).

### Am Aufbau — in dieser Reihenfolge

> **Arbeitsfassung mit Rollen, Dauer, Abnahme und Sicherheitsregeln:**
> `fahrplan-aufbau.md`. Dort sind die Stufen unten zu Blöcken für die Termine
> am Roboter zusammengefasst.

Jede Stufe setzt die vorige voraus. Was gemessen wird, ersetzt einen
dokumentierten Startwert; die Punkte stehen in `offene-punkte.md`.

> **Stand 24.09.2026:** Stufen 1, 2 (bis B21), 5 erledigt; Stufe 3 übersprungen —
> Stufe 6 lief am echten Roboter, erst mit `fake_objects.py` (L18), dann mit der
> Basiskamera und echten Klötzen bis zur Ablage (L19–L21). Stufen 4 und 7 entfallen
> (L22). Was offen ist, führt `fahrplan-aufbau.md`.

**1. Laden und Registrierung** ✅ — Branch bauen, Systemabbild im Launcher neu
erzeugen (Einrichtung §1), Anwendung laden: macht der Nutzer. Dann prüfen, ob
`vectoring`, `priority_handler`, `data_tracker`, `object_follower` und
`interface_streamer` in der Bibliothek erscheinen und die geänderten
Bestandskomponenten ihre neuen Parameter und Ausgänge zeigen (`base_cam`:
`camera_node`; `robot_cam`: Auswahlparameter; Greifer: `motion_done`,
`has_object`).

**2. Datenpfad bei stehendem Roboter** ✅ (bis auf B21, B1-Vergleich) — `base_cam` → `vectoring` →
`priority_handler` → `data_tracker` → `interface_streamer` verdrahten,
`robot_state` an den `priority_handler`. Klötze auflegen und mitlesen: `tracks`,
`target`, `world_state`, Wahl und Rückzug im Log, das Übersichtsbild.
- ~~**B23**~~ ✅ erledigt 23.09.2026 (Nachtrag 13 / L6).
- **B21:** ruhender Klotz an mehreren y-Positionen über den ganzen Sichtbereich —
  trägt der Tracker-Eingriff 2.4?
- **B1** nebenbei: Stoppuhr gegen die geschätzte Bandgeschwindigkeit.
- Danach dürfen die Specs von `vectoring`, `priority_handler`, `data_tracker`,
  `interface_streamer` und den Bestandskomponenten gelöscht werden.

**3. Follower am virtuellen Roboter** — Quelle `fake_objects.py` → `vectoring` →
`priority_handler`, Greifer-Rückmeldung über `toggle_signal`, Pflichtparameter aus
Einrichtung §9.
- **4a:** aus mehreren Startlagen aktivieren — erst senkrecht hoch, dann
  Beobachtungspose? Das Log nennt den Bezugsrahmen von `robot_state`; er muss
  `world` sein.
- **4b:** `timeout_track_s` auf 10 s, `err_laengs` mitlesen — mit `lead_time_s` =
  1/K im Mittel null? Folgt der Roboter über die Zone hinaus, bricht er an der
  Arbeitsraumgrenze ab.
- **4d:** voller Zyklus mit `toggle_signal`; ein künstlicher Abbruch mit Klotz im
  Greifer endet in der Kiste.

**4. ~~Roboterkamera~~** — nicht weiter verfolgt, nicht mehr eingebunden (L22).

**5. Greifzone** ✅ — **B19 + B10** am 23.09.2026 von Hand abgefahren: Arbeitsraum
in `Safety/workspace_bounds.json`, Zone (seit 24.09. = Arbeitsraum, L21) als Standardwert im
`priority_handler` — außerhalb des Bildes der Basiskamera (Nachtrag 13 / L4, L14).
B11 nur beobachtet (Gelenke nicht mitgeschrieben).
`is_zone_feasible` zeigt, ob die Zone lang genug ist. **B10:** Arbeitsraum nach
`Safety/README.md` festlegen.

**6. Follower am echten Roboter** — ✅ gedrosselt mit `fake_objects.py` (L18), dann
echte Griffe mit Ablage, 7 von 7 (L19, L20); B4, B9, B15 bestätigt. Offen: D23, B18,
B22 unter Last, mehrere Klötze. Erst 4a, dann 4b mit **B4**
(`lead_time_s` einmessen), **D23** (Montagewinkel der Backen). Dann 4d mit dem
Referenzklotz: **erster Testgriff** bestätigt die 245 mm (B15), **D22** messen
(Absenken, Greifen, Heben) und `t_descend_s` nachziehen, **B22** beobachten (bleibt
die Ziel-ID beim Greifen?), **B18** Toleranzen, **B9** Ablagepose bei laufendem
Programm gegenlesen.

**7. ~~4c aufschalten~~** — entfällt, Stufe 4c ist entfernt (Nachtrag 13 / L22).

---

## 7. Dokumentenlandkarte

Aufteilung und Vorrangregeln stehen in `docs/README.md`. Kurz: `uebersicht/`
für Betrieb und Überblick, `architektur/` für die technische Grundlage,
`archiv/` wird nicht mehr gepflegt. **Normativ sind `entscheidungen.md` und
`datenvertraege.md`**; die Specs sind daraus abgeleitet und werden gelöscht,
sobald die jeweilige Komponente am Aufbau gelaufen ist. Den Systemaufbau zeigt
`uebersicht/systemgraph.md`.
