# Entscheidungsprotokoll Robotetris

Laufendes Protokoll der Architekturentscheidungen. Entsteht Thema für Thema und
ist die Grundlage für die Aktualisierung des Word-Dokuments und den
Umsetzungsplan. Rein lokal, nichts committet.

Bezug: `docs/review/2026-09-05-konzeptreview-komponentenplan.md` (Befundnummern
in eckigen Klammern verweisen dorthin).

---

## Thema 1 — Bewegungsarchitektur

**Status:** entschieden, mit offenen Verifikationspunkten
**Löst Befunde:** O5, O3, O4, O6 (teilweise auch S4/R1)

### Ausgangslage

Bestehender, getesteter Aufbau (aus `applications`-YAML des Beispiels):

```
frame_to_signal (TfToSignal, Frame "target")
        │ pose
        ▼
signal_point_attractor (aica_core_components::motion::SignalPointAttractor)
        │  state     ← /hardware/robot_state_broadcaster/cartesian_state
        │  attractor ← /frame_to_signal/pose
        │  base_frame (unbenutzt)
        │ twist
        ▼
ik_velocity_controller (aica_core_controllers/velocity/IKVelocityController)
```
Hardware-Rate: 100 Hz.

### Problem

Ein Point Attractor rechnet `v = K · (x_attraktor − x_tcp)`. Er erzeugt
Geschwindigkeit nur bei vorhandenem Abstand. Bei einem **fahrenden** Ziel muss
der TCP dauerhaft Geschwindigkeit haben, also bleibt dauerhaft ein Abstand:

```
bleibender Versatz = v_band / K
```

| v_band | K=2 | K=5 | K=10 |
|---|---|---|---|
| 0,1 m/s | 5,0 cm | 2,0 cm | 1,0 cm |
| 0,2 m/s | 10,0 cm | 4,0 cm | 2,0 cm |
| 0,3 m/s | 15,0 cm | 6,0 cm | 3,0 cm |

Der Greifer würde also systematisch hinter dem Block zufassen — und zwar
stabil, weshalb eine Prüfung "Position über V Frames konstant" das nicht
aufdeckt. Höhere Gains lösen es nicht (Rauschverstärkung, Überschwingen).

### Entscheidung

**Der `object_follower` gibt eine Zielpose aus**, die an den `attractor`-Eingang
des Signal Point Attractors geht (ersetzt `frame_to_signal` im Betrieb).
Die Zielpose enthält einen parametrierten Vorhalt entlang der Bandrichtung:

```
ziel_pose = block_position_prädiziert + band_richtung · lead_offset_m
```

Damit sind zwei Varianten ohne Codeänderung austauschbar:

| Variante | `lead_offset_m` | `base_frame` verdrahtet | Status |
|---|---|---|---|
| **A — Vorhalt** | `v_band / K` | nein | **Primärweg**, funktioniert sicher |
| **C — bewegter Bezugsrahmen** | `0.0` | ja | Ausbaustufe, zu verifizieren |

Variante B (Twist direkt aus der Komponente) wurde verworfen: zu viel
Eigenverantwortung für Begrenzung/Rampen/Stabilität bei begrenzter
Echtzeit-Vorerfahrung im Team.

### Begründung Option C als Ausbaustufe

Die Doku des Signal Point Attractors nennt den `base_frame`-Eingang für Fälle,
"where the DS should be expressed in a **moving** reference frame".
`control-libraries` komponiert zwei `CartesianState` nachweislich inklusive
Twist:

```
linear_velocity  = v_base + R_base · v_lokal + ω_base × (R_base · p_lokal)
angular_velocity = ω_base + R_base · ω_lokal
```

Die Geschwindigkeit des Bezugsrahmens wird also beim Zurücktransformieren
addiert — genau die gesuchte Vorsteuerung. In einem mit dem Band mitfahrenden
Bezugsrahmen steht der Klotz still, der Attractor konvergiert exakt, und die
Bandgeschwindigkeit kommt beim Rücktransformieren automatisch dazu.

**Bestätigt:** die Rechenregel in `control-libraries` (Quellcode gelesen:
[`CartesianState.cpp`](https://raw.githubusercontent.com/aica-technology/control-libraries/main/source/state_representation/src/space/cartesian/CartesianState.cpp),
`operator*`).
**Nicht bestätigt:** dass `SignalPointAttractor` seinen `base_frame` über genau
diese Regel führt. Begründete Vermutung, kein Beweis.

### Konsequenzen für die Umsetzung

- Ausgang `object_follower`: Pose an den `attractor`-Eingang. Typ muss dem
  entsprechen, was `frame_to_signal.pose` liefert (Verifikation V2).
- Eingang `object_follower`: `cartesian_state` des `robot_state_broadcaster`
  (liegt in der bestehenden YAML bereits an).
- Neuer Parameter: `lead_offset_m`.
- Der z-Abstieg ist eine sinkende z-Komponente der Zielpose — kein Sonderfall.
- Das Mitfahren während des Greifens ist eine mit `v_band` weiterlaufende
  Zielpose — kein Sonderfall.
- Vorhalt wirkt **nur entlang der Bandrichtung**, nicht pauschal auf x und y.
- Einschwingzeit des Attractors ≈ `3/K` (bei K=5 rund 0,6 s). Wird in Thema 6
  für das Erreichbarkeitskriterium gebraucht.

### Fallstricke (aus der Attractor-Doku abgeleitet)

1. **`is_in_range` taugt mit Option A nicht als Greif-Freigabe.** Der Vorhalt
   hält den Abstand absichtlich ungleich null, das Prädikat wird nie wahr.
   → Greif-Freigabe rechnet der `object_follower` selbst aus TCP-Pose und
   Blockposition. Funktioniert dann in beiden Varianten. Das Prädikat bleibt
   für das Anfahren stehender Ziele (Warteposition, Ablage) nutzbar.
2. **`max_linear_velocity` muss deutlich über `v_band` liegen** (Richtwert 2–3×),
   sonst kann der Roboter das Band prinzipiell nie einholen.
3. **Lineare Gains isotrop halten** (alle drei Achsen gleich), sonst wird der
   Vorhalt achsabhängig und bei schräger Bandrichtung zur Matrixrechnung.
   Der Winkel-Gain darf abweichen.

### Offene Verifikationspunkte

| ID | Punkt | Wo |
|---|---|---|
| V1 | Werte von `linear gain`, `angular gain`, `max linear velocity`, `max angular velocity`, `linear precision threshold` ablesen | AICA Studio |
| V2 | Signaltyp von `attractor`-Eingang und `frame_to_signal.pose` (`cartesian_state` vs. `cartesian_pose`) | AICA Studio |
| V3 | Lässt sich an `base_frame` ein Signal anschließen (nicht nur statischer Frame)? | AICA Studio |
| V4 | Fünf-Minuten-Test Option C: Roboter still, mitbewegter Bezugsrahmen, fester Attractor → bewegt sich der Roboter? | Labor |
| V5 | Bandgeschwindigkeit messen (base_cam `vy` + Stoppuhr-Gegenprobe) | Labor |

---

## Thema 2 — Zeit & Latenz

**Status:** entschieden, mit einem blockierenden Test (B13)
**Löst Befunde:** S4, R1, teilweise O10

### Zeitquelle: der Header-Stempel, nicht `get_clock().now()`

Der geplante Weg ("ROS-Zeit im Kamerablock abfragen und mitgeben") ist nicht
nötig — die Bildnachrichten tragen bereits einen Header-Stempel, und die
Komponenten nutzen ihn:

- `base_cam.py:261` — Frame-Gating über `header.stamp`
- `base_cam.py:291` — Zeitbasis des Trackers
- `robot_cam.py:259` — wird bereits als `t` ausgegeben

Dass der Stempel gefüllt ist und läuft, folgt aus der Funktionsfähigkeit von
`base_cam`: wäre er konstant, würde das Frame-Gating jedes Bild nach dem ersten
verwerfen.

**Grundprinzip dahinter:** Konstante Verzögerung ist harmlos (ein Parameter),
schwankende Verzögerung ist es nicht. Eine Zeitabfrage im Step-Callback hätte
0…1/rate Jitter eingebracht (bei 50 Hz bis 20 ms, zufällig verteilt), der direkt
in Geschwindigkeitsschätzung und Prädiktion wandert. Der Header-Stempel hat
diesen Jitter nicht.

### Prädiktion auf den Wirkzeitpunkt

```
p_ziel = p₀ + d · v · (t_jetzt − t₀ + τ_latenz) + d · lead_offset
```

`d` = Bandrichtung (Einheitsvektor), `v` = Bandgeschwindigkeit,
`p₀`/`t₀` = Position und Zeitstempel der Messung.

Weil das Band konstant läuft, sind `v·τ_latenz` und `lead_offset` im
eingeschwungenen Zustand nicht unterscheidbar. **Die einzelnen Latenzen müssen
also gar nicht bekannt sein** — es wird ein einziger Restabstand im Betrieb
wegkalibriert (B4).

### Zwei getrennte Parameter

| Parameter | Kompensiert | Bei Option C |
|---|---|---|
| `lead_offset_m` | Nachlauf des Attractors (`v/K`) | **0** |
| `latency_compensation_s` | Kamera- und Kommandolatenz | **bleibt** |

Getrennt, weil beim Umschalten auf Option C nur der Attractor-Anteil
verschwindet. In einer gemeinsamen Zahl wäre nach dem Umschalten nicht mehr
nachvollziehbar, welcher Anteil welcher war.

### Warum die Prädiktion trotzdem gebraucht wird

Nicht primär wegen der Latenz, sondern wegen der **unterschiedlichen Takte**:
`base_cam` liefert ~30 Bilder/s, der Follower läuft schneller. Ohne
Extrapolation würde die Zielpose zwischen zwei Bildern stehen und dann springen
— bei 0,2 m/s in 6,7-mm-Stufen. Der Attractor sähe eine Treppe statt einer
Fahrt und erzeugte eine ruckelnde Geschwindigkeit.

→ **Follower-Rate: 100 Hz** (passend zur Hardware-Rate). Die Komponente macht
keine Bildverarbeitung, ist also billig. Zielpose wandert dann in 2-mm-Schritten.

### TCP-Historie statt `tf2`

`robot_cam` misst im Kamerabild, während der Arm fährt. Die Umrechnung braucht
die TCP-Pose **zum Aufnahmezeitpunkt**, nicht die aktuelle (bei 0,2 m/s und
40 ms sind das 8 mm).

Lösung: Der `object_follower` führt einen Ringpuffer `(t, TCP-Pose)` aus dem
`cartesian_state` und interpoliert auf den Bildstempel. ~20 Zeilen, 100 Einträge
für 1 s bei 100 Hz.

Begründung gegen `tf2`-Lookup: keine Abhängigkeit davon, dass der
`robot_state_broadcaster` TF publiziert (offen, A5), und die Transformation
bleibt aus der Bildverarbeitungskomponente heraus — entspricht der Vorgabe,
`base_cam`/`robot_cam` rechenarm zu halten.

### "Objekt verloren" erkennen

- **`base_cam`:** **nicht** über das Messungsalter. Der Tracker lässt Objekte bei
  fehlender Detektion weiterlaufen, sie tragen dann trotzdem den aktuellen
  Frame-Stempel und sehen frisch aus. Das echte Signal ist das **Verschwinden
  der Ziel-ID aus der Liste** (der Tracker löscht nach `track_max_missed_in_region`).
- **`robot_cam`:** hier ist das Alter das richtige Kriterium — direkte Messung,
  kein Coasting.

### Rückfall bei fehlender `robot_cam`-Messung

`w` = Gewichtung zwischen Basiskamera-Prädiktion (`w=0`) und
Roboterkamera-Messung (`w=1`), entspricht den Parametern X/Y aus dem
Komponentenplan.

- Messung zu alt → intern `w=0`, der Roboter folgt weiter per Prädiktion.
  Einfrieren des letzten Messwerts wäre der Abbruch des Griffs.
- **Nicht hart umschalten, sondern rampen** (Vorschlag ~0,2 s). Weichen beide
  Quellen um einige mm ab, springt die Zielpose sonst bei jedem Aussetzer hin
  und her und der Attractor erzeugt Geschwindigkeitsspitzen.
- Für die **Greif-Freigabe** gilt eine strengere Regel als fürs Folgen → Thema 6.
- Offen (D9): Darf ein Griff komplett ohne `robot_cam` durchlaufen? Hängt an der
  Güte der Basiskamera-Extrinsik, also erst nach der Kalibrierung entscheidbar.

### Risiko: Uhrendrift RealSense ↔ ROS (blockierend, Test B13)

RealSense-Kameras können in einer `HARDWARE_CLOCK`-Domäne stempeln, die beim
Einschalten bei null startet und laut Fehlerberichten **nur einmal nach dem
ersten Bild** gegen die Rechneruhr synchronisiert wird. Driften die Uhren,
driften die Stempel mit (gemeldeter Fall: 0,3 s Versatz bei einer D435).

Für den Tracker ist das egal (nur Differenzen zwischen aufeinanderfolgenden
Bildern). Kritisch ist das **Mischen** beider Uhren in `t_jetzt − t₀`:
0,3 s bei 0,2 m/s sind 60 mm, mit Drift über die Session wachsend — die einmal
kalibrierte Latenz würde unbemerkt ungültig.

**Test B13** klärt das. Nur bei nachgewiesener Drift ist eine Gegenmaßnahme
nötig: Zeitdomäne im Treiber auf Systemzeit (B14) oder Versatz in `base_cam`
selbst nachführen (~15 Zeilen).

**Notlösung**, falls der Kamerastempel unbrauchbar ist: `base_cam` stempelt
beim Verarbeiten selbst (der ursprüngliche Plan). Kostet Step-Jitter (wenige mm),
aber deutlich besser als 60 mm systematisch. Der Jitter halbiert sich, wenn
`base_cam` mit 100 statt 50 Hz läuft — durch das Frame-Gating fast kostenlos.

Quellen: [realsense-ros #419](https://github.com/IntelRealSense/realsense-ros/issues/419),
[#1373](https://github.com/IntelRealSense/realsense-ros/issues/1373),
[#2048](https://github.com/IntelRealSense/realsense-ros/issues/2048)

---

## Thema 3 — Frames & Einheiten

**Status:** entschieden, mit Verifikationspunkten A7/A8 und C8/C9
**Löst Befunde:** S3, R2, teilweise R5

### Einheiten: SI an allen Signalgrenzen

Alles, was über ein AICA-Signal geht, ist in **Meter, Meter/Sekunde, Radiant,
Sekunden**. Die Bildverarbeitung rechnet intern weiter in Millimetern
(Portierung aus dem C++-Original, bleibt unangetastet); die Umrechnung passiert
an genau einer Stelle — beim Packen des Ausgabearrays.

Begründung: Ein vergessener Faktor 1000 ist in einem Vision-System ein falscher
Logwert, in einem Robotersystem ein tausendfach zu weit entfernter Zielpunkt —
der Regler gibt Vollgas in die Begrenzung. Die Fehlerklasse verschwindet, wenn
an der Schnittstelle nie eine andere Einheit auftaucht.

Aufwand: eine Multiplikation je Komponente. **Die Tests prüfen nur die
Vision-Module darunter (`detection`, `tracker`, `color_estimation`), nicht das
Packen — es bricht nichts.**

### Bezugssystem: `world`, identisch mit der Roboterbasis

Bestätigt: `world` liegt bei euch auf der Roboterbasis. Alle Positionssignale in
`world`; jede `CartesianPose` trägt ihren `reference_frame` explizit gesetzt,
damit Verwechslungen beim Lesen auffallen statt beim Fahren.

Die Überlegung, `world` an den Bandanfang zu legen, wurde verworfen — richtig so:
die Roboterbasis ist mechanisch eindeutig und der Bezug, in dem der Regler
ohnehin denkt.

### Werkzeugversatz: Flansch bleibt geregeltes Frame

**Indiz:** In AICA Studio ist nur der Flansch sichtbar. Die 3D-Szene wird aus dem
URDF gerendert — der Greifer steht also vermutlich **nicht** darin. Damit regelt
der IK-Velocity-Controller den Flansch und der `robot_state_broadcaster` meldet
den Flansch.

**Entscheidung:** URDF nicht anfassen. Das nachträgliche Einpflegen des Greifers
würde die laufende Hardware-Konfiguration und das geregelte Frame verschieben,
also genau das bereits getestete Verhalten. Stattdessen rechnet der
`object_follower`:

```
ziel_flansch = ziel_greifpunkt + [0, 0, tool_offset_z_m]
```

Das ist **eine einzige Zahl**, weil der Greifer immer senkrecht nach unten zeigt
und der Greifpunkt auf der Flanschachse liegt. Eine Drehung des Handgelenks um
die Hochachse ändert den Versatz nicht. Erst bei gekipptem Greifer würde daraus
eine echte Posenverkettung.

Wert: aus dem Vorgängerprojekt auslesen (C8) — gleicher Greifer, gleicher Aufbau.

### Band-Frame: kein eigener Kalibriervorgang

Zweites Koordinatensystem neben `world`: Ursprung beliebig auf dem Band,
x entlang der Laufrichtung, y quer, z nach oben. Nutzen: die X/Y-Gewichtung
meint genau diese Achsen, der Vorhalt wirkt rein in x, und Option C braucht ein
solches Frame in mitfahrender Form.

**Es braucht keinen physisch vermessenen Ursprung** — für die Rechnung zählt nur
die Richtung.

Sorge des Teams war, dass die Bestimmung über durchlaufende Blöcke ungenau wird.
Das Gegenteil ist der Fall: Über die volle Bahn (~500 mm, 30–50 Messungen, ~3 mm
Rauschen) liegt die Richtungsunsicherheit **deutlich unter 0,5°**. Ungenau wird
eine Richtungsschätzung nur aus einem *kurzen* Bahnstück — genau das machte der
ursprüngliche Plan mit der Regression pro Objekt über N Punkte.

Wie genau es sein muss (seitlicher Versatz = Strecke · sin θ):

| Vorhersagestrecke | 0,5° | 2° | 5° |
|---|---|---|---|
| 20 mm (ein Bild + Latenz) | 0,2 mm | 0,7 mm | 1,7 mm |
| 100 mm | 0,9 mm | 3,5 mm | 8,7 mm |
| 400 mm (Abfangplanung) | 3,5 mm | 14 mm | 35 mm |

Während der Verfolgung wird immer nur ein Kamerabild plus Latenz überbrückt —
dort ist selbst ein grober Richtungsfehler bedeutungslos. Relevant wird die
Richtung nur bei der langen Vorhersage fürs Abfangen, und dort korrigiert die
Roboterkamera anschließend nach.

→ Richtung und Geschwindigkeit fallen aus einem normalen Testlauf ab
(`objects`-Signal mitschneiden, am Schreibtisch auswerten). Kollidiert nicht mit
der laufenden Automatisierung der Kamerakalibrierung.

**Nebeneffekt:** Die Bandrichtung wird aus denselben Koordinaten bestimmt, in
denen später vorhergesagt wird. Ein kleiner Drehfehler der Basiskamera-Extrinsik
steckt damit sowohl in den gemessenen Positionen als auch in der Richtung und
hebt sich für die Vorhersage weitgehend auf — gebraucht wird die *scheinbare*
Bewegungsrichtung im selben System, nicht die wahre physikalische. Nicht auf
heben tut es sich beim Zusammenführen von Basis- und Roboterkamera (verschiedene
Kalibrierketten) — genau dafür ist die X/Y-Gewichtung als Testinstrument da.

### Wo welche Kalibrierung eintritt

| Kalibrierung | Tritt ein in | Herkunft |
|---|---|---|
| Intrinsik Basiskamera | `color_camera_info`-Signal — schon da | Treiber |
| Extrinsik Basiskamera → Roboterbasis | `base_cam`-Parameter `cal_x`…`cal_yaw` — existiert | Kommilitone (C3) |
| Intrinsik Roboterkamera | `color_camera_info`-Signal — schon da | Treiber |
| Hand-Auge Roboterkamera → Flansch | **`object_follower`** (neu) | Vorgängergruppe (C1) |
| Bandrichtung + Bandgeschwindigkeit | `vectoring` + `object_follower` (neu) | eigene Auswertung (B1) |
| Werkzeugversatz Flansch → Greifpunkt | **`object_follower`** (neu) | Vorgängergruppe (C8) |
| Arbeitsraumgrenzen | `object_follower` | eigene Festlegung (B10) |

**Hand-Auge gehört in den `object_follower`, nicht in `robot_cam`** — folgt aus
Thema 2: `robot_cam` gibt Kamerakoordinaten plus Zeitstempel aus, der Follower
rechnet mit seinem TCP-Ringpuffer um. Bildverarbeitung bleibt rechenarm, die
zeitrichtige Zuordnung passiert dort, wo die TCP-Historie liegt.

Umrechnungskette der Roboterkamera:
```
p_world = T_world_flansch(t_bild) · T_flansch_kamera · p_kamera
              │                          │
      TCP-Ringpuffer, interpoliert   Hand-Auge-Kalibrierung
      auf den Bildzeitstempel        (fester Parametersatz)
```

**Jede Kalibrierung bekommt eine versionierte JSON als dokumentierte Quelle, die
zur Laufzeit in AICA-Parameter gespiegelt wird** — dasselbe Muster, das
`Calibration/calibration.json` und `Safety/workspace_bounds.json` bereits
verwenden, inklusive Änderungsregeln im README.

### Orientierungskonvention

`base_cam` liefert die Orientierung als **eine Zahl** in rad [0, π) — den Winkel
der Längsachse. Die vollständige Zielorientierung entsteht daraus:
Greifer senkrecht nach unten, Gierwinkel aus der Blockorientierung.

Zwei Details:
- **`gripper_yaw_offset_deg` als Parameter.** Ob die Greifbacken parallel oder
  senkrecht zur x-Achse des Werkzeugframes stehen, hängt von der Anflanschung ab
  (meist 0° oder 90°). Ohne diesen Wert greift der Greifer konsequent um 90°
  verdreht.
- **Der Bereich [0, π) ist Absicht.** Ein Block bei 10° und einer bei 190° sind
  für einen Parallelgreifer dasselbe. Welcher der gleichwertigen Winkel
  angefahren wird, entscheidet Thema 6.

---

## Thema 4 (getauscht) — Komponentenschnitt & Datenfluss

**Status:** entschieden
**Löst Befunde:** S5, S6, D1, D3, P3, P5, P6, V4, V6, teilweise S7

> Reihenfolge getauscht: Der Schnitt kam vor die Verträge, weil er bestimmt,
> welche Signale es überhaupt gibt.

### Ergebnis

1. **`data_tracker` verlässt den Regelpfad** und wird reiner Buchhalter für
   Diagnose und Anzeige. Der Zyklus ist damit aufgelöst, **ohne Komponenten
   zusammenzulegen** — die Aufteilung aus dem Plan bleibt erhalten.
2. **`vectoring` schrumpft drastisch**, weil Bandrichtung und -geschwindigkeit
   seit Thema 3 Konstanten sind.

Es bleiben **acht Komponenten**, genau wie im Plan. Geändert haben sich nur
Leitungen und Zuständigkeiten.

### `vectoring` neu

Da die Bahn jedes Objekts bis auf zwei Zahlen bekannt ist (Querablage, Phase),
wird aus Fitten ein Mitteln:

| Aufgabe | Verfahren |
|---|---|
| Querposition glätten | laufender Mittelwert je ID (konstant, da kein Querversatz auf dem Band) |
| Orientierung glätten | dito |
| Längsposition + Zeitpunkt | Messungen auf die Bandgerade projizieren, Phase mitteln |
| Plausibilität | bewegt sich das Objekt mit der kalibrierten Geschwindigkeit? |
| Verfall | ausbleibende IDs nach kurzer Frist vergessen |

Genauigkeitsgewinn: bei ~3 mm Rauschen je Bild und 30 Bildern bleiben quer
**0,55 mm** — Faktor 5, und zwar in der Richtung mit dem geringsten Greiferspiel.

**Entfallen ersatzlos:** Parameter `K`, `W`, `J`, `H` sowie die Schalter
"Richtung halten" / "Geschwindigkeit halten". Übrig bleibt im Wesentlichen die
Fensterlänge der Mittelung.

**Neu hinzugekommen:** der Plausibilitätstest. Er ist das Signal für "Block ist
heruntergefallen oder hängengeblieben", das der alte Entwurf nicht hatte.
Entscheidung: solche Objekte werden **ausgeschlossen**, der Grund bleibt über
den Statuscode **sichtbar** (siehe `datenvertraege.md` S3).

### Datenfluss

```
                                    ┌─────────── picked_id ──────────┐
                                    ▼                                │
base_cam ─objects→ vectoring ─tracks→ priority_handler ─target→ object_follower ─pose→ attractor
    │                  │                                        ▲      │
    │                  │               robot_cam ─measurement───┘      │
    │                  │        robot_state_broadcaster ─state────────→│
    │                  │                                               │
    │                  │            robotiq_gripper ←─command──────────┤
    │                  │                            ──feedback────────→│
    │                  │                                               │
    └─objects──────────┴─tracks→ data_tracker ←──── picked_id ─────────┘
                                      │
                                      └→ interface_streamer
```

Von `data_tracker` geht **keine Leitung zurück** in den Regelpfad. Er ist ein
Blatt, kann nichts blockieren und darf mit 10 Hz laufen.

### Warum `priority_handler` nichts von `data_tracker` braucht

Das war die Kante, die den Zyklus schloss. Tatsächlich kann er alles selbst:

- "schon gepickt" — hört direkt auf `picked_id` (bei 2–3 Objekten eine Handvoll IDs)
- "out of bounds" — besitzt die Greifzonengrenzen, **erzeugt** diese Information
- "erreichbar" — aus Position, Bandgeschwindigkeit, Zeitbudget

Die Information floss von Anfang an in die andere Richtung; der Zyklus im Plan
war eine Verdrahtung entgegen der natürlichen Flussrichtung.

### Eigentümerschaft

Regel: **Jede Information hat genau einen Eigentümer.** Alle anderen bekommen sie
von ihm, niemand rechnet parallel nach.

| Komponente | besitzt | besitzt nicht |
|---|---|---|
| `base_cam` | rohe Detektionen, IDs, Zeitstempel | keine Bahnlogik |
| `vectoring` | geglättete Objektzustände, Plausibilität | keine Auswahl, keine Flags |
| `priority_handler` | Greifzone, Erreichbarkeit, **Ziel-Lock** | keine Objektverwaltung |
| `object_follower` | Zustandsautomat, Regelung, Sicherheitsgate, TCP-Historie | keine Zielauswahl |
| `data_tracker` | Gesamtliste mit Flags (Diagnose) | nichts Regelrelevantes |
| `robotiq_gripper` | Greiferhardware | keine Ablauflogik |
| `interface_streamer` | Darstellung | nichts |

### Greifzone ≠ Sicherheitsraum (Auflösung von P3)

Keine Dopplung, sondern zwei nie unterschiedene Größen:

| | Greifzone | Sicherheitsraum |
|---|---|---|
| Was | Bandbereich, in dem ein Griff sinnvoll ist | erlaubter Bewegungsbereich des Roboters |
| Zweck | Aufgabenentscheidung | Kollisionsschutz |
| Eigentümer | `priority_handler` | `object_follower` |
| Quelle | Festlegung am Aufbau (B8/B10) | `Safety/workspace_bounds.json` |

Die Greifzone liegt innerhalb des Sicherheitsraums (dokumentierte Anforderung,
zur Konfigurationszeit prüfbar).

**Konsequenz:** Die ROI-Parameter wandern aus dem `object_follower` heraus. Er
entscheidet nicht mehr über Greifbarkeit. Wird ein Objekt unerreichbar, **zieht
`priority_handler` das Ziel zurück** (`has_target = 0`) und der Follower bricht
ab. Eine Entscheidung, ein Eigentümer.

### Taktraten

| Komponente | Rate | Begründung |
|---|---|---|
| `base_cam` | 100 Hz | Frame-Gating macht leere Durchläufe kostenlos; halbiert den Zeitstempel-Jitter |
| `robot_cam` | 100 Hz | dito |
| `vectoring` | 100 Hz | rechnet nur bei neuem Zeitstempel |
| `priority_handler` | 100 Hz | siehe Nachtrag 2 / K2 — das Zurückziehen des Ziels darf nicht 50 ms brauchen |
| `object_follower` | 100 Hz | Hardware-Rate, glatte Zielpose |
| `data_tracker` | 10 Hz | reine Diagnose |
| `interface_streamer` | 5 Hz | teuerste Komponente, niedrigste Rate |
| `robotiq_gripper` | ereignisgetrieben | unverändert |

---

## Thema 5 (getauscht) — Datenverträge

**Status:** entschieden, Details in `datenvertraege.md`
**Löst Befunde:** S2, B1, V7, D2, G1, G2, teilweise S7

Vollständige Spezifikation: **`docs/review/datenvertraege.md`**

### Fünf Grundregeln

1. **Kopf-Konvention** `[t, n, …]` bei jedem listenartigen Signal; Länge prüfbar
   als `kopf + n·stride`.
2. **Nie leer** — auch `[t, 0]` wird gesendet. Unterscheidet "sieht nichts" von
   "sendet nicht mehr". Im alten Entwurf wären beide Fälle eine leere Liste
   gewesen, und der Roboter hätte im zweiten Fall weiter auf die letzte bekannte
   Position gezielt.
3. **`t` nur im Kopf**, nicht je Objekt.
4. **Statusfelder statt stiller Annahmen** (`valid`, `has_target`, `status`).
5. **Ein gemeinsames `contracts.py`** mit Strides, Feldindizes und
   `pack_*`/`unpack_*`. Keine Komponente indiziert von Hand in ein fremdes Array.

### Zwei Festlegungen aus der Analyse

**`v_band` gehört in den Kopf, nicht zum Objekt.** Im Code weist der Tracker die
Geschwindigkeit global allen Tracks zu (`set_global_velocity`) — ein
festhängender Block bekäme trotzdem die volle Bandgeschwindigkeit eingetragen.
Als Objektfeld ist der Wert irreführend, als Kopffeld nützlich (Kalibrierquelle
B1 und Laufkontrolle des Bandes). Die Plausibilitätsprüfung je Objekt rechnet
`vectoring` deshalb aus der eigenen Messhistorie.

**`picked_id` mit laufender Nummer** `[seq, id, outcome]` ersetzt das
"1 Sekunde lang True"-Muster. Verbraucher merken sich die letzte `seq` und
reagieren genau einmal — kein Zeitfenster, keine Flankenerkennung, kein
Verpassen bei Lastspitzen. `outcome` liefert zusätzlich, ob der Griff geklappt
hat (im Plan gar nicht vorgesehen).

### Einzige Änderung an einer laufenden Komponente

`robotiq_gripper` bekommt **zwei Bool-Ausgänge**: `is_closed` und `has_object`.
Der Plan sah nur "Greifer zu" vor — ein Echo des Eingangs und als Rückmeldung
wertlos. Der Follower wertet `has_object` aus. Die vorhandenen Predicates
bleiben für die UI.

---

## Thema 6 — Zustandsautomat des `object_follower`

**Status:** entschieden
**Löst Befunde:** O1, O2, O3, O4, O7, O8, O9, O10, O11, O12, R5, S8, E7, E8, D9

### Zustände

| Zustand | Regelverhalten | Übergang bei | Timeout → |
|---|---|---|---|
| `WARTEN` | Beobachtungspose, statisches Ziel, **kein Vorhalt** | `has_target = 1` → `ANFAHREN` | keiner |
| `ANFAHREN` | Ziel auf Greifzone begrenzt | Block in der Zone → `FOLGEN` | großzügig → `ABBRUCH` |
| `FOLGEN` | volle Regelung mit `w`-Mischung, Greiforientierung einstellen | Abweichung < Toleranz für V Takte → `ABSENKEN` | ~2–3 s → `ABBRUCH` |
| `ABSENKEN` | wie `FOLGEN`, zusätzlich `vz` abwärts | Greifhöhe erreicht → `GREIFEN` | aus Höhe/Sinkrate; **Abweichung wächst → zurück zu `FOLGEN`** |
| `GREIFEN` | **reines Mitfahren**, kein Absenken, Greifer schließt | `has_object = 1` → `HEBEN` | ~2 s → `ABBRUCH` (`outcome=1`) |
| `HEBEN` | Mitfahren bis Transferhöhe | Transferhöhe erreicht → `ABLEGEN` | berechnet → `ABBRUCH` |
| `ABLEGEN` | statisches Ziel, **kein Vorhalt** | `is_in_range` → `LOESEN` | ~5 s → `ABBRUCH` |
| `LOESEN` | Greifer auf, `picked_id` senden | fertig → `WARTEN` | ~2 s → `ABBRUCH` |
| `ABBRUCH` | TCP-Pose halten, Greifer öffnen, senkrecht hoch, zur Beobachtungspose | → `WARTEN` | – |

**Abbruchgründe aus jedem Zustand:**
- `has_target = 0` — `priority_handler` hat das Ziel zurückgezogen
- Ziel-ID aus `tracks` verschwunden (Thema 2: bei `base_cam` das richtige Kriterium, nicht das Messungsalter)
- unplausibler Sprung der Regelabweichung (Fehldetektion, ID-Verwechslung)

Beim Abbruch wird `picked_id` mit passendem `outcome` gesendet, damit der
`priority_handler` weiß, dass das Objekt erledigt ist, und das nächste wählt.

### Abfangbewegung = Begrenzung auf die Greifzone

```
ziel = begrenze(block_position_prädiziert + vorhalt, greifzone)
```

Die beiden Fälle aus dem Plan (Abfangkurs am Zonenrand vs. direktes Anfahren)
fallen damit zu **einer Regel** zusammen:
- Block stromaufwärts → Begrenzung greift, Roboter wartet am oberen Zonenrand
  **auf der Querposition des Blocks** (die wird nicht begrenzt) = der geplante
  Abfangkurs
- Block in der Zone → Begrenzung greift nicht mehr, Folgen beginnt nahtlos

Kein Umschalten, kein zweiter Codepfad.

### Regelgesetz: Korrektur filtern statt Position mischen

```
korrektur = roboterkamera_position − basiskamera_prädiktion
ziel      = basiskamera_prädiktion + w · korrektur_gefiltert
```

Mathematisch identisch zur wörtlichen X/Y-Mischung, aber **filterbar**: Die
absolute Position wandert mit Bandgeschwindigkeit und lässt sich nicht glätten,
ohne Verzögerung einzubauen. Die Korrektur ist dagegen nahezu konstant (Versatz
zwischen den Kalibrierketten plus Rauschen) und lässt sich stark glätten, ohne
das Folgen träge zu machen.

`w` bedeutet damit anschaulich: *wie viel der Roboterkamera-Korrektur wird
aufgeschaltet.* Rampe ~0,2 s beim Wechsel (Thema 2), Rückfall auf `w=0` bei
`valid = 0`.

**Entfällt:** das "N Koordinaten sammeln + lineare Regression" aus dem Plan. Es
diente der Gewinnung eines Bewegungsvektors aus Roboterkamera-Messungen — der
ist seit Thema 3 bekannt. Übrig bleibt ein gleitender Mittelwert der Korrektur.

### Greif-Freigabe

Zwei Präzisierungen gegenüber "Position über V Frames konstant":

1. **Konstant im mitfahrenden System** — Abweichung zwischen TCP und
   prädizierter Blockposition, nicht absolute Positionsstabilität.
2. **Drei getrennte Toleranzen**, weil die Richtungen unterschiedlich streng sind:

| Richtung | Begrenzt durch | Größenordnung |
|---|---|---|
| quer zur Backenbewegung | Fingerbreite | eng, wenige mm |
| längs der Backenbewegung | Öffnungsweite − Blockbreite | großzügig, >10 mm |
| Höhe | Blockhöhe − Eintauchtiefe | mittel |

Eine gemeinsame Toleranz würde sich immer nach der strengsten richten und
Freigaben verschenken.

Zusätzlich Parameter `require_robot_cam_for_grasp` (Umsetzung von D9): Ist er
falsch, darf ein Griff komplett auf Basiskamera-Prädiktion laufen.

### Greifhöhe

```
greif_z = bandoberflaeche + max( blockhoehe / 2,  min_greifhoehe )
```

Mittig auf der Seitenfläche → nach oben und unten Reserve bei Höhenfehlern.
Die untere Grenze schützt das Band: Die Greifbacken haben selbst Höhe (z. B.
35 mm); bei einem 20 mm flachen Block läge die Backenunterkante rechnerisch
7 mm unter dem Band.

Bezug ist die **Bandoberfläche** (fester, einmal vermessener Wert), nicht die
gemessene Blockoberkante — die hängt an der Extrinsik der Basiskamera.

### Neue Greifbarkeitsprüfung im `priority_handler`

Im Plan nicht vorhanden. Alle Daten liegen vor, bevor angefahren wird:

| Kriterium | Prüfung |
|---|---|
| zu flach | `hoehe ≥ min_greifbare_hoehe` (≈ `2 · min_greifhoehe`) |
| zu breit | Abmessung quer zur Backenrichtung `≤ max_oeffnung − marge` |

Welche Abmessung quer zur Backenrichtung liegt, folgt aus Blockorientierung und
kommandiertem Gierwinkel — gilt damit in beiden Orientierungsmodi. Verhindert
die frustrierendste Fehlerart: sauber anfahren, greifen, passt nicht.

### Beobachtungspose: senkrecht

**Entscheidung: Kamera senkrecht nach unten.** Die Höhe bleibt der einstellbare
Parameter, die Orientierung ist fest (YAML).

Bei Kippung um θ liegt die erkannte Blockoberfläche seitlich versetzt um
`h · tan θ` gegenüber der Standfläche — bei 20° und 40 mm Blockhöhe knapp 15 mm,
**und je Block unterschiedlich**, weil die Höhen variieren. Senkrecht ist der
Versatz null, unabhängig von der Blockhöhe.

Weitere Vorteile:
- Werkzeugversatz bleibt eine Zahl, kein Aufrichten-Zustand vor dem Absenken
- Tiefenmessung aufs Band auf kürzestem Weg = genauester Fall
- **Kamera und Greifer auf derselben Achse** → "Block an dieser Bildposition"
  heißt immer "Block unter dem Greifer" (fester Pixelversatz aus der
  Hand-Auge-Kalibrierung). Im Debug-Bild unmittelbar ablesbar.
- Der seitliche Anteil der Hand-Auge-Kalibrierung lässt sich dadurch ohne
  Messaufbau nachziehen: einmal erfolgreich greifen, Bildposition notieren.

Der einzige Vorteil einer Kippung — den Block früher sehen — wird nicht
gebraucht; dafür ist die Basiskamera da.

### Orientierungsmodi

```
greifer_gierwinkel = gripper_yaw_offset_deg + ( use_block_orientation
                                                ? block_orientierung
                                                : bandrichtungswinkel )
```

Modus 1 (Blöcke gerade aufgelegt) ist damit ein **Spezialfall** von Modus 2, kein
eigener Zweig — wichtig, damit Modus 2 nicht erst am Ende getestet wird.

**Falle bei fast quadratischen Blöcken:** Der Winkel aus `minAreaRect` springt
zwischen ~0° und ~90°. Ein arithmetischer Mittelwert daraus ist 45° — genau die
Lage, in der der Greifer die Ecken erwischt.

Lösung: **Mittelung über den verdoppelten Winkel** (Standard für achsenartige
Größen mit π-Periodizität):

```
mittel = ½ · atan2( Mittel(sin 2θ), Mittel(cos 2θ) )
```

Die Länge des resultierenden Vektors ist gratis ein Qualitätsmaß: nahe 1 =
Messungen einig, nahe 0 = Winkel flattert. Unterschreitet sie eine Schwelle,
gilt die Orientierung als unbrauchbar → Verhalten wie Modus 1 (fester Winkel),
Meldung über den Status. Bei einem quadratischen Block ist das korrekt, dort ist
die Orientierung ohnehin beliebig.

**Winkelwahl:** Orientierung liegt in [0, π), der Greifer ist 180°-symmetrisch —
es gibt immer zwei gleichwertige Handgelenkstellungen. Regel: **die nähere zur
aktuellen wählen**, sonst dreht das Handgelenk gelegentlich eine halbe Umdrehung
und läuft in Gelenkgrenzen.

**Die Greiforientierung wird einmal festgelegt und beim Übergang nach `ABSENKEN`
eingefroren.** Eine Winkeländerung während des Absenkens ist unnötig und
gefährlich.

### Statische Ziele: Vorhalt auf null

`WARTEN` und `ABLEGEN` fahren stehende Ziele an. Dort muss `lead_offset_m` auf 0
gesetzt werden — er kompensiert den Nachlauf bei **bewegtem** Ziel. Bleibt er
stehen, parkt der Roboter dauerhaft einige Zentimeter neben Beobachtungs- und
Ablageposition.

Umgekehrt ist bei stehenden Zielen das Attractor-Prädikat `is_in_range`
brauchbar und darf als Übergangsbedingung dienen — anders als beim Folgen
(Thema 1, Fallstrick 1).

### Transferhöhe

`HEBEN` steigt auf eine Höhe über allem, was auf dem Band stehen kann, **bevor**
die Fahrt zur Ablage beginnt. Der Point Attractor fährt geradlinig zwischen zwei
Punkten — ohne Zwischenschritt streicht die Bahn je nach Höhenverhältnis flach
über das Band.

Aufbau: Ablage seitlich neben dem Band auf der Roboterseite, Pose in der Luft
über einer Auffangkiste, Block fällt hinein. Zwei Parameter: Freihöhe über dem
Band und die Ablagepose.

Zusätzlich bleibt das Mitfahren zu Beginn von `HEBEN` erhalten, bis der Block
die Bandoberfläche verlassen hat — hielte der Roboter an, während der Block noch
aufliegt, zerrte das Band daran.

---

## Thema 7 — Sicherheit

**Status:** entschieden, B10 bleibt blockierend
**Löst Befunde:** S9, E9, R4

### Korrektur zu Teil B des Reviews

In Teil B stand die klassische Warnung vor Geschwindigkeitsregelung: Bleiben die
Kommandos aus, fährt der Roboter mit der letzten Geschwindigkeit weiter.
**Das gilt für eure Kette nicht.** Der `object_follower` gibt eine *Pose* aus.
Hört er auf zu senden, hält der Attractor die letzte Pose, der Roboter fährt sie
an und bleibt stehen — die Geschwindigkeit ist proportional zum Abstand und geht
gegen null.

| | Rohe Geschwindigkeitsregelung | Attractor-Kette |
|---|---|---|
| Kommando bleibt aus | fährt weiter — gefährlich | konvergiert und hält — unkritisch |
| Kommando ist falsch | fährt falsch | fährt falsch — **das bleibt** |

→ Die Sicherheitsarbeit gehört in die **Prüfung der Zielpose vor dem
Veröffentlichen**, nicht in einen Watchdog auf Kommandoebene.

### Hauptrisiko: veraltete Extrapolation

```
ziel = p₀ + bandrichtung · v_band · (t_jetzt − t₀)     ← wächst unbegrenzt
```

Stürzt `base_cam` ab oder hängt `vectoring`, veralten `p₀`/`t₀`, die
Extrapolation läuft weiter. Nach 10 s bei 0,2 m/s liegt das Ziel **2 m** daneben.

Zwei Gegenmaßnahmen, beide nötig:
1. **Zeitstempel der Eingangssignale muss fortschreiten** — steht `t` in `tracks`
   still, sind die Daten tot → Abbruch. (Anderes Kriterium als bei einem einzeln
   verlorenen Objekt in Thema 2: hier geht es um den Ausfall der ganzen Komponente.)
2. **`max_extrapolation_s`** deckelt `(t_jetzt − t₀)` auf wenige Kamerabilder.
   Versagt Prüfung 1, bleibt der Fehler bei wenigen Zentimetern.

### Das Sicherheitsgate

Eine Funktion, als **letzter Schritt** vor der Ausgabe. Nichts umgeht sie.

| # | Prüfung | Bei Verstoß |
|---|---|---|
| 1 | alle Eingangswerte endlich (kein NaN/Inf) | Abbruch |
| 2 | Zeitstempel der Eingangssignale schreiten fort | Abbruch |
| 3 | Extrapolationshorizont ≤ `max_extrapolation_s` | deckeln |
| 4 | Sprung zur vorherigen Zielpose ≤ `max_ziel_sprung_m` | **Abbruch**, nicht deckeln |
| 5 | Zielpose im Arbeitsraum | deckeln **und** Abbruch melden |

Zu 4: Ein großer Sprung heißt, dass die Daten falsch sind (ID-Verwechslung,
Fehldetektion). Deckeln hieße, falschen Daten langsam zu folgen. Die Prüfung gilt
**innerhalb** eines Zustands und wird bei jedem Zustandswechsel zurückgesetzt,
weil dort legitime Sprünge auftreten.

Zu 5: Deckeln hält den Roboter im Moment sicher, löst aber nichts — er hinge am
Rand bis zum Timeout. Deshalb beides. Ein Ziel außerhalb des Arbeitsraums ist ein
Fehler des `priority_handler` und soll sichtbar werden.

**Keine eigene Komponente.** Sie bräuchte dieselben Daten, fügte einen
Signalsprung Verzögerung und eine Fehlerquelle hinzu. Als abgegrenzte Funktion im
Follower derselbe Nutzen.

### Arbeitsraum: Positionsbegrenzung genügt

Auch das war in Teil B anders formuliert (dort: Geschwindigkeitskomponente
Richtung Grenze nullen). **Mit dem Attractor genügt das Begrenzen der Zielpose**,
weil er positionssuchend ist: Ziel innen → Roboter bleibt innen. Ein System
erster Ordnung schwingt nicht über (`x(t) = x* + (x₀−x*)·e^{−Kt}`, monoton); die
reale Antriebsdynamik erzeugt ein kleines Überschwingen, das die Sicherheitsmarge
abdeckt.

Umsetzung: Rechteck-Clamp auf die Zielpose. Voraussetzung bleibt B10 —
`Safety/workspace_bounds.json` steht weiterhin auf `placeholder_not_yet_defined`
mit lauter `null`, und laut eigenem README darf daraus nichts übernommen werden.

### Singularitäten

Bei IK-Geschwindigkeitsregelung werden nahe einer Singularität für eine gewünschte
kartesische Geschwindigkeit sehr große Gelenkgeschwindigkeiten nötig.

Beim Aufbau nicht theoretisch: Der Roboter steht direkt neben dem Band und greift
seitlich hinüber. Kandidaten: **Schultersingularität** (TCP nahe der senkrechten
Achse durch die Schulter — plausibel, weil das Band dicht vorbeiläuft) und
**Ellbogensingularität** (fast gestreckter Arm am fernen Bandende).

Gegenmaßnahme, zuverlässig und billig: **Die Greifzone ist ein Parameter — den
problematischen Abschnitt ausschließen.** Etwas Bandlänge gegen ein Verhalten,
das nirgends kippt.

Bereich finden (B11): TCP auf Beobachtungshöhe von Hand die Greifzone
entlangfahren, Gelenkgeschwindigkeiten beobachten. Wo sie unruhig werden, beginnt
die Sperrzone.

### Grenzen der Greifzone — drei Kriterien gemeinsam festlegen

| Grenze | Bestimmt durch |
|---|---|
| stromaufwärts | Reichweite · Singularitäten · **Abstand zur Basiskamera-Halterung** (steht am Bandanfang) |
| stromabwärts | Reichweite · verbleibendes Zeitbudget für den Griff |
| quer | Bandbreite · Reichweite |

Müssen zusammen festgelegt werden, sonst legt man eine fest und stolpert über die
nächste.

### Start = Abbruchpfad

Beim Anwendungsstart steht der Roboter irgendwo; der Attractor führe **geradlinig**
zur Beobachtungspose — je nach Ausgangslage durch Band oder Vorrichtung.

Der Abbruchpfad aus Thema 6 macht bereits das Richtige (senkrecht hoch, dann
Beobachtungspose). Also: **Der Follower startet im Zustand `ABBRUCH`.** Ein
Codepfad, zwei Zwecke.

Bestätigt: Der Raum senkrecht über dem Arbeitsbereich ist frei; nur die
Basiskamera steht am Bandanfang, den der Roboter kaum erreicht.

### Abschalten mit Block im Greifer

- **Der Roboter fährt das letzte Ziel noch an.** Beim Verfolgen harmlos (Ziel
  liegt Zentimeter voraus), bei der Transferfahrt wird die Ablagepose zu Ende
  angefahren. Begrenzt und vorhersehbar, weil das Gate jede Pose bereits im
  Arbeitsraum gehalten hat.
- **Der Block bleibt im Greifer.** Beim nächsten Hochfahren öffnet die
  Greiferkomponente ihn im Bring-up (`ARCHITECTURE.md` §13 Regel 6). Der Block
  fällt dort hin, wo der Arm steht. Kein Fehler, aber vorher zu wissen.

### Grenzen in der Controller-Schicht

| Grenze | Wo |
|---|---|
| max. kartesische Geschwindigkeit | `max linear velocity` des Attractors |
| max. Winkelgeschwindigkeit | `max angular velocity` des Attractors |
| Gelenkgeschwindigkeiten | Hardware-Interface (unangetastet → Herstellergrenzen des UR10e) |

Der Attractor-Grenzwert ist der wichtigste Einzelparameter für die Sicherheit: Er
wirkt **unabhängig davon, was der Follower berechnet**, also auch bei einem Fehler
dort.

### R4 — Identitätsprüfung der Roboterkamera-Messung

`robot_cam` liefert "das Objekt am nächsten zum Bildzentrum", ohne Prüfung, ob das
auch das Zielobjekt ist. Bei 2–3 Blöcken kann ein zweiter ins Bild geraten.

Lösung fällt aus der Formulierung in Thema 6 ab: Die Korrektur
(`roboterkamera_position − basiskamera_prädiktion`) ist im Normalfall **klein** —
Versatz der Kalibrierketten plus Rauschen, also wenige Zentimeter. Ein anderer
Block liegt bei von Hand aufgelegten Blöcken deutlich darüber.

→ **`max_korrektur_m`**: Wird sie überschritten, Messung verwerfen und für diesen
Takt auf `w = 0` zurückfallen (wie bei `valid = 0`).

Ein Parameter, keine Änderung an der Bildverarbeitung, keine zusätzliche Leitung.
Fängt nebenbei eine grob falsche Hand-Auge-Kalibrierung und Ausreißer der
Kantendetektion ab.

---

## Nachtrag — Restbefunde P1, P2, P4, R3, I2

Abgleich gegen das Review ergab: 44 von 61 Befunden sind in den Themen 1–7
namentlich abgearbeitet, die meisten übrigen implizit (B2 durch die
Frame-Festlegung, B3 durch `v_band` im Kopf, B5 weil das Predicate
"new object base" durch das gegatete `objects`-Signal ersetzt ist, D4 durch die
Verfallsregel, D5 durch die Eigentümerschaft, G3 als Ausbaustufe, I3 durch die
Signal-statt-Predicate-Regel, O13 durch die Klärung zur Ablage, R6 durch das
benannte Feld `z_band`). Die folgenden fünf waren tatsächlich offen.

### P1 — Erreichbarkeitskriterium im `priority_handler`

```
t_verfügbar = (zonenende − blockposition) / v_band

t_benötigt  = anfahrt + absenken + greifen
  anfahrt  ≈ abstand / v_max + 3/K      ← Fahrt plus Einschwingen des Attractors
  absenken =  (z_beobachtung − z_greif) / v_absenk
  greifen  =  Schließzeit des Greifers

kandidat ⟺ t_verfügbar > sicherheitsfaktor · t_benötigt
```

Der Term `3/K` ist die Einschwingzeit aus Thema 1 (bei K=5 rund 0,6 s). Ohne ihn
plant man mit einer Ankunft, die so nie eintritt.

**Die Auswahlregel des Plans bleibt erhalten:** unter den Kandidaten weiterhin das
dringendste (kleinstes `t_verfügbar`) — nur eben unter den *erreichbaren*. Das ist
der Unterschied zwischen "pickt nie etwas" und "funktioniert".

### P4 — Ziel-Lock

Einmal gewählt, bleibt ein Ziel gewählt. Auftauchende "bessere" Objekte lösen
keinen Wechsel aus (verwirft investierte Zeit, bringt bei 2–3 Blöcken nichts).

Zurückgezogen (`has_target = 0`) nur bei:

| Grund | Auslöser |
|---|---|
| gepickt | `picked_id` trifft ein |
| verloren | ID verschwindet aus `tracks` |
| unplausibel | `status ≠ 0` |

**Erreichbarkeit gehört bewusst NICHT dazu** — sie ist Auswahl-, kein
Abbruchkriterium. Sonst würde ein fast erfolgreicher Griff abgebrochen, sobald der
Block beim Greifen die Zonengrenze überschreitet. Die Greifzone ist enger als der
Sicherheitsraum; der Roboter darf dem Block ein Stück darüber hinaus folgen.

Nebeneffekt: Es braucht keine Rückmeldung des Follower-Zustands an den
`priority_handler` — die Entkopplung aus Thema 4 bleibt erhalten.

### P2 — Greifzone ist ein Rechteck

Vier Grenzen, nicht drei. Das im Plan fehlende `y_min` gehört dazu (B19).

### R3 — Min-/Max-Distanz von `robot_cam`

Bezieht sich auf den gemessenen **Abstand Kamera → Bandoberfläche** (`z_band`,
Feld 4 in S2). Außerhalb → `valid = 0`:
- zu hoch: zu wenig Bildauflösung auf dem Block
- zu tief: Tiefenbereich verlassen oder Block nicht mehr vollständig im Bild

Verknüpft sich mit B8 — die Beobachtungshöhe muss in diesem Fenster liegen.

### I2 — Minimalspezifikation `interface_streamer`

| | |
|---|---|
| Eingänge | `debug_image` (base_cam), `debug_image` (robot_cam), `world_state`, `follower_status` |
| Ausgang | ein zusammengesetztes Bild |
| Rate | 5 Hz |
| Layout | zwei Debug-Bilder nebeneinander; darunter Zustandsname, Ziel-ID, wirksames `w`, die drei Regelabweichungen; darunter die Objektliste mit Status / gepickt / out-of-bounds |

Alle Daten liegen in `world_state` und `follower_status` bereits an — keine
zusätzlichen Leitungen. **Wird als letzte Komponente gebaut**: die einzige, ohne
die das System vollständig funktioniert.

---

## Nachtrag 2 — Kritische Durchsicht (13.09.2026)

Abschließende Prüfung aller Festlegungen mit der Absicht, Fehler zu finden.
Ergebnis: drei Konstruktionsfehler, zwei zu vorsichtige Entscheidungen, zwei
Planlücken.

### F1 — Greif-Freigabe prüfte die Orientierung nicht

Festgelegt waren drei Toleranzen (längs, quer, Höhe). Der **Gierwinkel fehlte**.

Im Orientierungsmodus 2 dreht sich das Handgelenk um bis zu 90°, begrenzt durch
`max angular velocity` des Attractors. Der Roboter kann positionsmäßig sauber
über dem Klotz stehen, während das Handgelenk noch dreht — die Freigabe käme zu
früh und der Greifer fährt schräg nach unten. Tritt besonders dann auf, wenn die
Winkelgeschwindigkeit aus Vorsicht konservativ eingestellt ist.

**Korrektur:** vierte Toleranz `tol_yaw_rad`, verpflichtend in der
Freigabebedingung.

### F2 — Roboterkamera fällt beim Absenken aus und verschiebt dabei das Ziel

Zwei einzeln richtige Festlegungen ergaben zusammen einen Fehler:
- `robot_cam` setzt `valid = 0` unterhalb `min_belt_distance_m`
- bei `valid = 0` wird `w` über `w_ramp_s` auf null gerampt

Beim Absenken unterschreitet die Kamera diese Grenze **zwangsläufig**. Genau
während der Greifer um den Klotz herum nach unten fährt, wird die Korrektur
ausgeblendet und die Zielpose wandert seitlich um deren Betrag — typisch einige
Millimeter. Das ist die Querbewegung, die dort am wenigsten erwünscht ist.

**Korrektur:** Beim Übergang `FOLGEN → ABSENKEN` wird die Korrektur
**eingefroren**, nicht ausgeblendet. Ab dort folgt das Ziel der reinen Vorhersage
plus konstantem Versatz. Sachlich richtig, weil die Ausrichtung vor dem Absenken
geprüft wurde und sich danach am Ziel nichts mehr ändern soll.

Der eingefrorene Wert gilt bis einschließlich `GREIFEN`. Nebeneffekt: beseitigt
auch das Zittern, falls `robot_cam` im Grenzbereich zwischen gültig und ungültig
springt.

### F3 — Rücksprung von `ABSENKEN` nach `FOLGEN` war unterspezifiziert

Festgelegt war der Rücksprung bei wachsender Abweichung, nicht aber **auf welcher
Höhe**. Bleibt der Roboter unten, ist die Kamera weiter außerhalb ihres
Messbereichs; der nächste Absenkversuch startet ohne Korrektur und der Vorgang
wiederholt sich.

**Korrektur:** Beim Rücksprung wieder auf Beobachtungshöhe steigen. Dann wird die
Kamera wieder gültig, die Korrektur baut sich neu auf, und der nächste Versuch
startet unter denselben Bedingungen wie der erste.

### K1 — Option B war zu schnell verworfen

Begründung war der Verlust der Stabilitätseigenschaften des dynamischen Systems.
**Das trifft nicht zu:** Ein Point Attractor *ist* ein P-Regler. Ein selbst
gerechnetes `v = v_ff + K·(ziel − tcp)` mit Begrenzung hat dieselben
Eigenschaften plus korrekte Vorsteuerung, ohne Vorhalt und ohne Gain-Kopplung.

Das tragfähige Argument für Option A bleibt: **die Kette ist getestet**. Ein
neuer Signaltyp und eine neue Verbindung zum IK-Controller sind bei knapper
Laborzeit ein unnötiges Risiko. Die Begründung ist damit korrigiert, die
Entscheidung bleibt.

**Zusatz — Selbstüberwachung des Vorhalts:** Die Gain-Kopplung ist gefährlicher
als dargestellt. Wird der Gain im Studio verändert — beim Abstimmen völlig normal
— stimmt `lead_offset_m` nicht mehr und der Griff geht still daneben.

Der Follower kann das selbst erkennen: Er kennt die ausgegebene Zielpose und die
gemeldete TCP-Pose. Im eingeschwungenen Zustand muss deren Differenz dem
eingestellten Vorhalt entsprechen. Dauerhafte Abweichung → Warnung ins Log. Fünf
Zeilen gegen einen Fehler, den man sonst lange sucht.

### K2 — `priority_handler` auf 100 Hz

20 Hz waren mit "Zielauswahl braucht keine Geschwindigkeit" begründet. Für die
Auswahl stimmt das, aber das **Zurückziehen des Ziels** erreicht den Follower
dann bis zu 50 ms später. Fällt das während des Absenkens an, sinkt der Greifer
50 ms auf ein totes Ziel.

Ein Zwölf-Element-Array bei 100 Hz kostet nichts. **Rate auf 100 Hz.**

### L1 — Synthetischer Datenpfad-Test fehlte im Plan

Phase 3 war als "bei stehendem Roboter testbar" beschrieben — das setzt Anwesenheit
am Aufbau voraus. Der Datenpfad braucht aber gar keine Kameras.

**Ergänzung:** Ein kleines Skript, das synthetische `objects`-Arrays
veröffentlicht (drei Blöcke, konstante Geschwindigkeit über ein gedachtes Band),
treibt `vectoring`, `priority_handler` und den Follower vollständig an. Damit
sind Glättung, Plausibilität, Auswahl, Erreichbarkeit, Ziel-Lock und die gesamte
Zustandsmaschine **am Schreibtisch** validierbar.

Bei knapper Laborzeit die wertvollste Einzelmaßnahme im ganzen Plan.

### L2 — Virtueller Roboter (beantwortet)

AICA selbst lässt sich nicht simulieren, aber **statt des echten UR10e kann ein
virtueller Roboter verwendet werden, der die Bewegungen visualisiert.** Damit
laufen die Stufen 4a bis 4c am Schreibtisch.

Zusammen mit dem synthetischen Signalgeber (L1) läuft die **gesamte Kette ohne
Hardware**: synthetische Klötze, echte Komponenten, echter Attractor, echter
IK-Controller, sichtbare Bewegung. Die Greifer-Rückmeldung liefern übergangsweise
die vorhandenen `true_signal` / `toggle_signal`.

**Nachweisbar:** Zustandsübergänge, Geometrie, Bahnform, Sicherheitsgate,
Vorhersage, Kameramischung, Winkelwahl — und bei gleichem URDF und IK-Controller
auch das Verhalten nahe Singularitäten (B11).

**Nicht nachweisbar:** reale Latenz, `lead_offset_m` (B4), Kameraverhalten,
Greiferzeiten. Diese bleiben dem Aufbau vorbehalten.

### Zwei Anmerkungen ohne Handlungszwang

**Parameterzahl.** 43 Parameter am `object_follower` sind viel; etwa ein Drittel
sind Sicherheits- und Filterwerte, die im Betrieb niemand anfasst. Das
JSON-Schema kennt `internal: true` — damit verschwinden sie aus der Oberfläche,
bleiben aber setzbar.

**Mittelung in `vectoring`.** Gleichgewichtetes Fenster ist optimal, solange das
Messrauschen zufällig ist. Bei positionsabhängigem Fehler (kleiner
Kalibrierfehler, der über das Sichtfeld wandert) mittelt man über eine Drift
statt über Rauschen. Vorher nicht entscheidbar. Falls die Quergenauigkeit im
Betrieb enttäuscht: exponentielle Gewichtung als erster Versuch, bevor anderes
angefasst wird.
