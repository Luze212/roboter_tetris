# Entscheidungsprotokoll Robotetris

Laufendes Protokoll der Architekturentscheidungen. Entsteht Thema für Thema und
ist die Grundlage für die Aktualisierung des Word-Dokuments und den
Umsetzungsplan. Rein lokal, nichts committet.

Bezug: `docs/archiv/2026-09-05-konzeptreview-komponentenplan.md` (Befundnummern
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

Vollständige Spezifikation: **`docs/architektur/datenvertraege.md`**

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

> **Nachtrag 14.09.2026 — die Begründung stimmt, das Beispiel nicht.** Der
> beschriebene Sprung erreicht `vectoring` nicht: `vision/detection.py` markiert
> fast quadratische Objekte und `vision/tracker.py` friert deren Orientierung auf
> den zuerst gemessenen Wert ein. Für sie meldet das Gütemaß deshalb dauerhaft
> „einig". Mittelung und Gütemaß bleiben richtig und nötig — das Gütemaß wirkt nur
> gegen andere Ursachen (Rauschen, Teilverdeckung, Objekt am ROI-Rand), nicht gegen
> Quadrate. Geprüft und verworfen wurde, das Merkmal `square` in S1 aufzunehmen;
> Begründung in `datenvertraege.md` unter S1, Hintergrund in
> `vorgaengerprojekt-abgleich.md` §7.

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
auch das Zielobjekt ist.

> ⚠️ **Die Prämisse stimmt nicht (gemessen 15.09.2026).** Der Code wählt die
> **flächengrößte** qualifizierte Kontur (`localize_largest_blob`), nicht die
> bildzentrumsnächste. Am Aufbau führt das dazu, dass die Farbvariante den
> bildfüllenden Blob und die Kantenvariante die Maschinenstruktur am Bildrand
> wählt — in beiden Fällen nicht den Klotz. `max_korrektur_m` fängt das nicht ab,
> weil der gemeldete Versatz klein aussehen kann. Gegenmaßnahmen (Auswahl nach
> Nähe zum Erwartungspunkt, ROI, Flächenobergrenze):
> `robot-cam-befunde.md` §9.7/§9.8. Bei 2–3 Blöcken kann ein zweiter ins Bild geraten.

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

---

## Nachtrag 3 — Kritische Durchsicht gegen den Code (14.09.2026)

Prüfung des Gesamtplans gegen den tatsächlichen Stand von `roboter_tetris/vision/`,
mit den Erkenntnissen aus `vorgaengerprojekt-abgleich.md` und
`robot-cam-befunde.md`. Ergebnis: zwei Messfehler erster Ordnung, zwei
Planwidersprüche, eine Lastfalle. Alle fünf sind entschieden.

### N1 — `robot_cam` misst seitlich höhenabhängig falsch

`vision/robot_detection.py` projiziert den Blobmittelpunkt mit der **Banddistanz**
zurück:

```python
x_mm = (ref_px[0] - cx) * depth_m / fx * 1000.0     # depth_m = Abstand zum Band
```

Der Mittelpunkt liegt aber auf der **Oberseite** des Klotzes, also um dessen Höhe
`h` näher an der Kamera. Jeder Seitenversatz ist damit um `z_band/(z_band − h)` zu
groß; der absolute Fehler beträgt

```
Δ = (seitlicher Abstand vom Bildzentrum) · h / (z_band − h)
```

**Der Fehler konvergiert nicht weg.** Im Bildzentrum wäre er null — dort steht der
Klotz aber gerade nicht. Die Hand-Auge-Kalibrierung weist rund **114 mm seitlichen
Versatz** zwischen Flanschachse und Kamera aus (x = 0,1087, y = −0,0344 m). Liegt
der Klotz korrekt unter dem Greifer, steht er im Bild also weit außen — dort ist
der Fehler maximal.

| Klotzhöhe | z_band = 0,30 m | z_band = 0,50 m |
|---|---|---|
| 25 mm | 10 mm | 6 mm |
| 76 mm | **39 mm** | 20 mm |

Systematisch, richtungsfest und **je Klotz verschieden** — dieselbe Fehlerklasse,
die „Beobachtungspose senkrecht" (Thema 6) eigentlich ausschließen sollte. Er
liegt zudem unter `max_correction_m` (0,05 m) und wird von der Identitätsprüfung
nicht abgefangen.

> Passt zum Fehlerbild der Vorgängergruppe: fester `MANUELLER_X_OFFSET = -0.03`
> plus zwei nie eingestellte Faktoren für eine positionsabhängige „Rand-Korrektur"
> (`vorgaengerprojekt-abgleich.md` §5). Kein Beweis — deren Kette war eine andere.

**Entscheidung: Korrektur im `object_follower`**, unmittelbar bei der Umrechnung
der Messung in Weltkoordinaten:

```
x_korr = x_gemessen · (z_band − blockhoehe) / z_band        (y analog)
```

Die Rechnung ist exakt, keine Näherung — sie kürzt sich algebraisch auf die
richtige Rückprojektion. `z_band` steht in S2 Feld 4, `blockhoehe` in S4 Feld 10;
beides liegt bereits an, es ändert sich kein Datenvertrag und nichts unter
`vision/`. Das entspricht der Regel aus Thema 3, dass Transformationen in den
Follower gehören und die Kamerakomponenten rechenarm bleiben, und hält den
A/B-Test `robot_cam` ↔ `robot_cam_2` unberührt (gemeinsamer Kern unangetastet).

**Reihenfolge beachten:** Die Skalierung wirkt **vor** dem Korrekturfilter und
damit vor dem Einfrieren beim Übergang `FOLGEN → ABSENKEN` (F2). Sonst friert der
unkorrigierte Wert ein.

Verworfen: Korrektur in `robot_cam` (bräuchte die Klotzhöhe als neuen Eingang in
beide Varianten und einen Eingriff in den gemeinsamen Detektionskern) und
Absorption in der Hand-Auge-Kalibrierung (rechnerisch unmöglich — die
Kalibrierung ist starr, der Fehler höhenabhängig; die Spanne beträgt bei euren
Klötzen rund 30 mm).

### N2 — Der Tracker koppelt die Längsposition außerhalb seiner Messregion

`vision/tracker.py`, zwei Stellen:

```python
if not det_in_region and det.vy != 0.0:
    track.object.y = prev_y + det.vy * dt          # y wird INTEGRIERT
...
if velocity_region_y_min <= y <= velocity_region_y_max:
    keep = track.missed <= max_missed_in_region     # streng: 3 verpasste Frames
else:
    keep = min_tracked_y_mm <= y <= max_tracked_y_mm # nur Positionsgrenzen
```

Außerhalb von `track_velocity_region_y_min…max` gilt beides: Die Längsposition
wird aus der global gefilterten Bandgeschwindigkeit fortgeschrieben **statt
gemessen** — auch bei vorliegender frischer Detektion — und ein Track wird für
fehlende Detektionen **nicht gelöscht**, sondern erst, wenn seine gerechnete
Position das Band verlässt.

Drei Folgen:

| Betroffen | Folge |
|---|---|
| Plausibilität in `vectoring` | Status 1 („steht/verklemmt") vergleicht die Längsgeschwindigkeit gegen `belt_speed_mps`. Außerhalb der Region *ist* die Längsbewegung per Konstruktion die Bandgeschwindigkeit — die Prüfung ist tautologisch und kann dort nie auslösen. |
| Abbruchregel des Follower | Thema 2 legt fest: „Objekt verloren" erkennt man am Verschwinden der ID. Außerhalb der Region verschwindet sie nicht mehr. Ein heruntergefallener oder weggenommener Klotz gleitet als **Phantom** mit Bandgeschwindigkeit weiter, mit Status 0, und wird vom `priority_handler` gewählt. |
| Längsgenauigkeit | Ein Fehler in `v_band` wächst außerhalb der Region linear mit der zurückgelegten Strecke. Mitteln hilft nicht — das ist keine zufällige Streuung. |

**Entscheidung: Die Messregion wird an die Greifzone gekoppelt.**
`track_velocity_region_y_min` / `_max` werden bei **B19 gemeinsam mit der
Greifzone** festgelegt: Region = Greifzone plus Rand. Damit ist `y` dort gemessen,
wo gegriffen wird, und ein Phantom stirbt beim Eintritt in die Zone nach drei
verpassten Frames. Stromaufwärts bleibt die weiche Regel — dort dient die Position
nur der Auswahl und der Erreichbarkeitsschätzung, wo Koppelung über kurze Strecken
unkritisch ist.

Das ist **reine Konfiguration**: beides sind vorhandene AICA-Parameter von
`base_cam`, nichts unter `vision/` wird angefasst. **B19 bekommt damit ein viertes
Kriterium.**

> Der ursprüngliche Zweck der Messregion ist für uns ohnehin entfallen: Sie sollte
> einen verlässlichen Ausschnitt für die *Geschwindigkeitsschätzung* liefern. Seit
> Thema 3 ist `belt_speed_mps` eine kalibrierte Konstante; `v_band_gemessen` ist
> nur noch Kalibrierquelle (B1) und Laufkontrolle.

Verworfen: Ausweitung auf das ganze Band (am Rand des Sichtfelds reißen Tracks
dann nach drei Frames ab → neue IDs, `vectoring` beginnt mit Status 3, der
Ziel-Lock geht verloren) und Beibehaltung der geerbten −1000…−500 mm (diese
500 mm stammen aus dem Aufbau der Vorgängergruppe und würden die Greifzone
willkürlich einengen, womöglich im Konflikt mit Reichweite und Singularitäten).

### N3 — Die Greifzonen-Begrenzung im Follower ist **eine** Zahl

Thema 6 verlangt für `ANFAHREN` „Ziel auf Greifzone begrenzt", Thema 4 hat die
Zonenparameter aber ausdrücklich aus dem Follower entfernt. Der Widerspruch löst
sich bei genauem Lesen fast von selbst — es wirken gar nicht vier Grenzen:

- **quer:** Thema 6 selbst schließt es aus — der Roboter wartet am Zonenrand „auf
  der Querposition des Blocks (die wird nicht begrenzt)".
- **stromabwärts:** P4 verbietet es — der Roboter darf dem Block über die
  Zonengrenze hinaus folgen, sonst bricht ein fast gelungener Griff ab.
- **Höhe:** deckt der Arbeitsraum-Clamp des Sicherheitsgates ab.

Übrig bleibt die **stromaufwärtige Längsgrenze**.

**Entscheidung: als 13. Feld im `target`-Satz (S4), feste Länge 12 → 13.** Der
`priority_handler` bleibt alleiniger Eigentümer der Greifzone (Thema 4 unberührt),
der Follower wendet nur an, was er bekommt. S4 ist ein Satz fester Länge ohne
Stride-Arithmetik und hat **genau einen Verbraucher** — der Eingriff berührt
`datenvertraege.md`, `contracts.py`, `priority_handler` und `object_follower`,
sonst nichts.

Verworfen: eigener Parameter im Follower (die Doppelpflege, die Befund P3
aufgelöst hat — stimmen die Werte nicht überein, wartet der Roboter woanders, als
der `priority_handler` für seine Erreichbarkeitsrechnung annimmt) und
Vorberechnung der Warteposition im `priority_handler` (der müsste dann
`lead_offset_m` und `latency_compensation_s` ebenfalls kennen — drei doppelt
gepflegte Parameter statt einem, und die Vorhaltrechnung an zwei Orten).

### N4 — Die Erreichbarkeitsprüfung braucht den Roboterzustand

`t_benötigt = abstand / attractor_v_max_mps + 3/K + t_descend_s + t_grasp_s`
braucht einen Anfahrweg. Der `priority_handler` hatte dafür keine Quelle — seine
Eingänge sind `tracks` und `picked_id`, seine Parameter kennen weder
Roboterzustand noch Beobachtungspose. Der Term ist nicht entbehrlich: Über
Bandbreite und Zonenlänge schwankt der Weg realistisch zwischen 0,3 und 0,8 m,
also rund eine Sekunde bei einem Budget von wenigen Sekunden.

**Entscheidung: `cartesian_state` wird Eingang des `priority_handler`.**

Das erzeugt **keinen Zyklus** — das Signal kommt vom `robot_state_broadcaster`,
also aus der Hardware, und der `object_follower` hört es ohnehin ab. Es ist eine
zusätzliche Abzweigung eines vorhandenen Signals. Die Entkopplung, die Thema 4
schützt, betrifft die Rückmeldung des **Follower-Zustands**, und die bleibt
unberührt.

Nebeneffekt in die richtige Richtung: Steht der Roboter gerade weit weg — etwa
mitten im Abbruchpfad — gilt korrekt nichts als erreichbar, bis er zurück ist.

Verworfen: Beobachtungspose als eigener Parameter (P3-Doppelpflege, und die
Annahme stimmt nur, solange der Roboter wirklich in `WARTEN` steht) und eine
pauschale Anfahrzeit (man müsste auf den ungünstigsten Fall auslegen und
verschenkt systematisch nahe liegende Klötze).

### N5 — Debug-Bilder werden mit der Komponentenrate publiziert

AICA verschickt Ausgangsvariablen in **jedem** Takt. `debug_image` ist ein
normaler Output; die Anhebung von `base_cam` und `robot_cam` auf 100 Hz
verdreifacht damit den Bildverkehr gegenüber der Kamerarate.

Im Produktivbetrieb folgenlos — bei ausgeschaltetem Debug bleibt die Variable ein
leeres `Image()`. Die Last entsteht genau dann, wenn `debug_enable` an ist: **bei
der Inbetriebnahme**, deren sechs Stufen (B6) vollständig auf dem Debug-Bild
aufbauen. Ausbleibende Frames würde man dort der Erkennung anlasten statt dem
Publizieren.

**Entscheidung: `add_output(..., publish_on_step=False)`** für `debug_image` in
`base_cam`, `robot_cam` und `robot_cam_2`, und `publish_output("debug_image")`
nur dann, wenn tatsächlich ein neues Debug-Bild gerendert wurde. Der Mechanismus
ist in `ARCHITECTURE.md` vorgesehen.

**Korrektur einer Begründung nebenbei:** Die 100 Hz waren mit halbiertem
*Zeitstempel*-Jitter begründet. Das galt für die Notlösung, bei der `base_cam`
selbst stempelt. Seit Thema 2 den Header-Stempel nutzt, trägt der Zeitstempel
diesen Jitter nicht. Der Gewinn ist ein anderer und kleinerer: Die Wartezeit
zwischen Bildeingang und Verarbeitung sinkt von 0–20 ms auf 0–10 ms, und dieser
Anteil steckt im Regelkreis. **100 Hz bleibt richtig, die Begründung war es
nicht.**

---

## Nachtrag 4 — Messungen am Aufbau (14.09.2026)

Erste Messsitzung am laufenden System (Teststand-Anwendung, virtueller Roboter
statt UR10e). Klärt A6, B12, B13, B14 und findet einen akuten Fehler.

> Technische Notiz für Wiederholungen: Messungen im AICA-Container **als Benutzer
> `ros2`** ausführen (`docker exec -u ros2 …`). FastDDS tauscht die Daten über
> Shared-Memory-Segmente, die `ros2` gehören; als `root` sieht man über UDP zwar
> alle Topics, bekommt aber **keine Daten** — ein Fehlerbild, das viel Zeit kostet.

### M1 — A6: Bildraten und Auflösungen

| Signal | Auflösung | Encoding | Rate |
|---|---|---|---|
| Basiskamera (L515) Farbe | 1280×720 | rgb8 | 29,7 Hz |
| Basiskamera aligned depth | 1280×720 (nativ 640×480) | 16UC1 | 29,8 Hz |
| Roboterkamera (D435i) Farbe | 848×480 | rgb8 | 29,7 Hz |
| Roboterkamera aligned depth | 848×480 | 16UC1 | 28,5 Hz |

Die in Thema 2 angenommenen ~30 Bilder/s stimmen. **A6 erledigt.** Die Tiefe der
Basiskamera ist nativ nur halb so fein wie ihr Farbbild und wird fürs Alignment
hochskaliert.

Nebenbefund: Die Komponenten liefen mit **10 Hz**, nicht mit den 50 Hz, die der
Ist-Stand annahm — `base_cam` verarbeitete also nur jedes dritte Kamerabild.

### M2 — B13/B14: Die Basiskamera stempelte in einer fremden, driftenden Uhr

**Gemessen:** `ros_zeit − header_stempel`

| | Basiskamera | Roboterkamera |
|---|---|---|
| Epoche des Stempels | 1155960174 (**Jahr 2006**) | ROS-Zeit ✓ |
| Drift | **+4,05 ms/s**, linear über 270 s | keine |
| danach | Stempel **bleibt ganz stehen** (151 Bilder, 1 Zeitstempel) | 181 Bilder, 181 Zeitstempel |

**Folge, und zwar sofort:** `base_cam` gated auf den Zeitstempel. Steht er, wird
jedes Bild nach dem ersten verworfen, `_handle_stale()` räumt nach einer Sekunde
die Ausgabe leer — `objects` war über Minuten leer, während das Debug-Bild
eingefroren den letzten verarbeiteten Frame zeigte (byteweise identisch über acht
Minuten). Thema 2 hatte genau diesen Fall beschrieben und aus der
Funktionsfähigkeit von `base_cam` geschlossen, dass er nicht eintritt. Der
Schluss war richtig — nur galt er zum Zeitpunkt der Beobachtung.

**Ursache und Behebung:** `rgb_camera.global_time_enabled` und
`depth_module.global_time_enabled` standen bei der L515 auf `false`, bei der
D435i auf `true`. Das ist ein Treiber-Default je Gerätefamilie, keine
Fehlkonfiguration. Nach dem Setzen auf `true`:

```
t=340 s   1,7849 s     ← Drift läuft
t=350 s   0,0730 s     ← Parameter gesetzt
```

Bestätigungsmessung über die folgenden 220 s: **Drift +0,022 ms/s** statt
+4,05 ms/s, Versatz stabil bei 0,046…0,093 s — auf dem Niveau der Roboterkamera.
**B13 und B14 erledigt**, die Notlösung aus Thema 2 wird nicht gebraucht.

### M3 — Entscheidung: `base_cam` erzwingt die Zeitdomäne selbst

**Der AICA-RealSense-Block exponiert `global_time_enabled` nicht** — er bietet 25
Parameter, dieser ist nicht dabei. Der Wert lässt sich also nur zur Laufzeit über
den ROS-Parameterdienst setzen und ist nach jedem Start der Anwendung wieder weg.

**Entscheidung:** `base_cam` bekommt den Parameter **`camera_node`** (Default
leer = aus). Ist er gesetzt, legt die Komponente in `on_configure_callback` einen
Service-Client auf `<camera_node>/set_parameters` an und setzt beide Parameter aus
`on_step_callback` heraus — nicht blockierend über `service_is_ready()` und
`call_async` mit Done-Callback, mit Wiederholung, weil die Kamera-Node später
hochkommen kann als die Komponente (`ARCHITECTURE.md` §3).

Begründung gegen die Alternativen: Ein Setzen von Hand nach jedem Start wird genau
einmal vergessen, und der Fehler ist **still** — die Erkennung sieht minutenlang
normal aus, bevor sie einfriert. Ein Exponieren durch AICA wäre der saubere Weg,
ist aber nichts, worauf man wartet.

### M4 — Was die Erkennung dabei gezeigt hat

Mit laufenden Zeitstempeln arbeitet die Kette einwandfrei:

- `n=1`, der Klotz sauber erkannt, `median=862 mm` gegen `conveyor_z_dist=865` —
  die Banddistanz stimmt auf 3 mm
- **ChArUco-Bögen und Zettel auf dem Band werden korrekt nicht erkannt** — die
  Höhenfilterung über das Tiefenfenster arbeitet wie vorgesehen
- `objects` mit Stride 10 in mm, wie dokumentiert; Position im Roboter-Frame
  **x = 814, y = −874 mm** — innerhalb der Tracker-Grenzen und innerhalb der
  Messregion

**Zwei offene Zahlenfragen** aus derselben Messung:

`z` ist **nicht die Oberkante**. Gemeldet wurde 103,1 mm, die Rückrechnung über
die Boxecken ergibt 53,1 mm — also exakt `Eckenmittel + Höhe/2`. `datenvertraege.md`
beschreibt das Feld unter S1 als „Oberkante des Blocks". Regelungsrelevant ist es
nicht (die Greifhöhe kommt aus `belt_surface_z_m`), die Beschreibung ist aber falsch.

Die **Höhe stimmt exakt** — und `HEIGHT_BIAS_MM` ist damit belegt, nicht willkürlich.
Der vermessene Klotz ist **50 × 50 × 100 mm, hochkant stehend** (nachgereicht):

```
gemeldet                            100,0 mm
rückgerechnete Deckflächen-Tiefe    865 + 20 − 100 = 785 mm
geometrisch erwartet                865 − 100      = 765 mm
→ die Tiefenmessung liest die Deckfläche 20 mm zu weit,
  HEIGHT_BIAS_MM = 20 kompensiert genau das
```

Die Gegenprobe aus `robot-cam-befunde.md` §1 reproduziert sich damit. **Der Bias
ist eine gemessene Korrektur der Tiefenkamera, kein Fudge-Faktor** — und er hängt
genau deshalb an `conveyor_z_dist`, wie bei B7/B17 vermerkt.

> Eine frühere Fassung dieses Abschnitts behauptete, die Höhe sei rund 23 mm zu
> groß. Das war falsch: Als Bandreferenz war der `median` aus der Debug-Kopfzeile
> genommen worden — der ist aber der Median über **das ganze Bild** (Band, Tisch,
> Kalibrierbögen), nicht die Banddistanz unter dem Klotz. Richtig ist
> `conveyor_z_dist`.

### M5 — Streuung der Geometrie, über 299 Messungen (korrigierte Fassung)

> **Korrektur, 15.09.2026.** Hier stand zuvor, die Grundfläche werde
> *richtungsabhängig um +8,8 mm zu groß* gemessen. Das war aus **einer einzigen
> Nachricht** geschlossen und hat sich nicht bestätigt: Eine zweite Einzelmessung
> am selben Klotz ergab 51,0 × 51,0 mm, also praktisch exakt. Beide Werte liegen
> in der Streuung, die die Messung unten zeigt. **Aus einem Bild lässt sich hier
> nichts ableiten.**

Ordentliche Messung am ruhenden 50 × 50 × 100-Klotz, **299 Nachrichten über 30 s**:

| Größe | Median | Min | Max | Std | Soll |
|---|---|---|---|---|---|
| Länge | 53,52 | 49,05 | **75,32** | 5,63 | 50 |
| Breite | 50,75 | 47,90 | 61,53 | 2,62 | 50 |
| Höhe | **100,67** | 98,86 | 102,24 | **0,60** | 100 |
| x | 835,04 | 833,90 | 836,29 | **0,55** | – |
| y | −904,92 | −905,60 | −904,31 | **0,22** | – |
| z | 108,45 | 106,39 | 110,49 | 0,75 | – |

**Drei Schlüsse, und sie gehen in verschiedene Richtungen:**

**Position und Höhe sind besser als angenommen.** Die Standardabweichung der
Position liegt bei **0,2 bis 0,6 mm**, die der Höhe bei 0,6 mm. Thema 4 rechnet
für den Genauigkeitsgewinn von `vectoring` mit „~3 mm Rauschen je Bild" — der
Eingang ist also schon deutlich sauberer. Der Gewinn durch Mittelung fällt damit
kleiner aus als gerechnet, aber von einem besseren Ausgangspunkt.

**Die Grundfläche ist das verrauschte Feld.** Länge streut über **26 mm**
(Std 5,63) bei einer wahren Kante von 50 mm, mit Ausreißern bis 75 mm. Der Median
liegt mit +3,5 mm nur leicht zu hoch — das Problem ist die **Streuung**, nicht ein
Versatz. Ursache ist die Tiefenkontur an den Klotzkanten, die von Bild zu Bild
unterschiedlich weit auf die Seitenflächen übergreift.

**Die Orientierung ist über alle 299 Messungen exakt konstant** (2,912 rad). Das
ist die empirische Bestätigung des Befunds aus Nachtrag 3 / §7: Der Tracker friert
den Winkel fast quadratischer Objekte auf den zuerst gemessenen Wert ein. Bisher
war das aus dem Quellcode gelesen, jetzt ist es gemessen.

> Alle Werte gelten für einen **ruhenden** Klotz. Bei laufendem Band kommen
> Bewegungsunschärfe und wechselnder Blickwinkel dazu; die Streuung ist damit eine
> untere Schranke, kein Betriebswert.

**Zwei Konsequenzen:**

**Für B16.** Die Greifbarkeitsprüfung rechnet mit zu breiten Klötzen und liegt damit
zur sicheren Seite — sie verwirft eher, als dass sie einen zu breiten Klotz
durchlässt. Die Marge `gripper_margin_m` sollte diesen systematischen Anteil nicht
noch einmal aufschlagen.

**Für die Quadrat-Erkennung: die Prüfung gehört auf geglättete Werte, nicht auf
ein Einzelbild.** Das Seitenverhältnis desselben, exakt quadratischen Klotzes
streut über die 299 Messungen von **0,727 bis 0,999** (Median 0,952). Je Einzelbild
griffe die Schwelle 0,92 in 208 von 299 Fällen, die Schwelle 0,85 in 265 — **keine
der beiden ist bildweise verlässlich.** Ein Schwellenwert löst das Problem also
nicht.

Was es löst: `priority_handler` liest `length`/`width` aus **S3**, und die sind von
`vectoring` über das Mittelungsfenster geglättet. Auf dem Median über 30 Messungen
liegt das Verhältnis bei 0,95 und damit stabil über 0,92. **Die ursprüngliche
Schwelle 0,92 bleibt also, sie gilt nur ausdrücklich für die geglätteten Werte.**

Für das interne `square`-Flag in `detection.py` ändert das nichts — es arbeitet
bildweise und ist derselben Streuung ausgesetzt. Die Entscheidung, es nicht in S1
aufzunehmen, bleibt damit richtig: Es zu übertragen hätte die Streuung nur
weitergereicht.

### M6 — C8: Werkzeugversatz aus der Steuerung (15.09.2026)

Aus dem Zustandspaket `CARTESIAN_INFO` der UR-Steuerung gelesen (Primary Interface,
Port 30011, rein lesend — `rtde_control` wurde bewusst nicht verwendet):

```
konfigurierter Werkzeugversatz Flansch → TCP
  x =   0,00 mm     y =   0,00 mm     z = 215,00 mm
  Rotationsvektor = 0 / 0 / 0
→ tool_offset_z_m = 0,215
```

Gegenprobe über die Vorwärtskinematik (UR10e-DH aus den Gelenkwinkeln gegen
`getActualTCPPose`): −0,49 / −0,43 / **214,22** mm. Abweichung unter 0,8 mm, das
liegt im Rahmen der DH-Konstanten — der Wert ist bestätigt.

**Zwei Dinge bestätigen sich damit:**

Die Warnung aus Thema 3 und Phase 0.1 war der Größenordnung nach genau richtig —
„fährt der Greifer rund 20 cm zu tief, ins Band". Es sind **21,5 cm**.

Und die Vereinfachung hält: **x = y = 0 und keine Verdrehung.** Der Greifpunkt
liegt exakt auf der Flanschachse, der Versatz ist damit tatsächlich *eine einzige
Zahl*, wie Thema 3 angenommen hatte. `ziel_flansch = ziel_greifpunkt + [0, 0,
tool_offset_z_m]` ist keine Näherung, sondern exakt.

> **Damit ist Phase 0 des Umsetzungsplans vollständig.** A2, A7, A8 und C8 — die
> vier Punkte, die Phase 4 blockierten — sind geklärt.

### M7 — C2 und A10 nebenbei

**C2 — Intrinsik.** Steht in den `camera_info`-Signalen; eine externe Quelle ist
nicht nötig, weil die Komponenten sie ohnehin zur Laufzeit von dort beziehen.
Basiskamera (L515, 1280×720): fx = 897,83 · fy = 897,54 · cx = 647,05 · cy = 362,31,
`plumb_bob` mit echter Verzeichnung. Roboterkamera (D435i, 848×480): fx = 608,41 ·
fy = 608,57 · cx = 417,55 · cy = 253,28, Verzeichnung **null** — der Farbstream der
D400-Serie ist bereits entzerrt.

**A10 — Gelenkgrenzen** aus `/hardware/robot_description`: Schulter 2,094 rad/s
(330 Nm), Ellbogen und Handgelenke 3,142 rad/s (150 bzw. 56 Nm). Herstellergrenzen
des UR10e, nicht auf den Aufbau eingeschränkt.

---

## Nachtrag 5 — Messungen am Roboter (15.09.2026)

Alle Werte dieses Nachtrags stammen aus einer Mitschrift von
`/hardware/robot_state_broadcaster/cartesian_state` und `.../joint_state`,
je 200 bzw. 50 Nachrichten pro Stellung, ausschließlich lesend. Der Roboter
wurde von Hand gefahren; es wurde nichts kommandiert.

### M8 — Der Bezugspunkt ist der Flansch, nicht der TCP der Steuerung

**Das ist die wichtigste Einsicht dieser Sitzung, weil sie eine stillschweigende
Verwechslung auflöst.**

In der UR-Steuerung ist ein TCP mit **215 mm** unter dem Flansch konfiguriert
(C8 / M6). Unsere Regelkette benutzt diesen TCP **nicht**: Der Greifer steht
nicht im URDF (A7), `robot_state_broadcaster` meldet `ur_tool0`, und der
IK-Velocity-Controller regelt ebenfalls den Flansch (A8). Jede Zahl, die wir
kommandieren oder auswerten, ist damit eine **Flanschgröße**.

Daraus folgt eine saubere Trennung, die ab jetzt gilt:

| Zweck | zu verwendender Versatz | Herkunft |
|---|---|---|
| Höhen am Aufbau, Greifhöhe, Bandhöhe | **245 mm** Flansch → Backenspitze | am 15.09.2026 mit dem Maßstab gemessen |
| Umrechnung fremder **TCP**-Werte (Vorgängerprojekt) in unser Flanschmaß | **215 mm** | Konfiguration der UR-Steuerung |

Die 30 mm Differenz sind plausibel: Die Backenauflagen sind 20 mm hoch (B15),
ein TCP in Auflagenmitte statt an der Spitze liegt in dieser Größenordnung höher.
Die Vorgängergruppe hat den TCP offenbar auf ihren Griffpunkt gelegt.

⚠️ **Die 245 mm sind mit ±5 mm behaftet.** Das Greifergehäuse zwischen Flansch
und Greifer hat denselben Durchmesser wie der Flansch; die Flanschebene ist von
außen nicht sicher anzulegen, gemessen wurde bis zum ersten sichtbaren silbernen
Bauteil des Roboters. Der Fehler ist **systematisch** — er verschiebt alle Höhen
gemeinsam und fällt beim ersten Testgriff als konstanter Versatz auf.

**Kein Beleg aus dem Vorgängerprojekt.** Der Gedanke, die Arbeitsraumuntergrenze
`Z = 0,095` aus `Robot/pose.yaml` stütze den einen oder anderen Wert, ist falsch:
Da jene Grenze selbst eine TCP-Größe ist, kürzt sich der Versatz in der Rechnung
heraus und liefert für beide Annahmen dieselbe Bodenfreiheit von 11 mm. Die
245 mm ruhen allein auf der Messung mit dem Maßstab.

### M9 — B17: Bandoberflächenhöhe und Ebenheit

Drei Antastpunkte, Greifer **geschlossen** (Berührpunkt damit auf der
Flanschachse, direkt unter dem Flansch), Werkzeugachse jeweils lotrecht —
bestätigt durch `wrist_2 = −90,00°` und durch die Quaternion, die eine
180°-Drehung um eine Achse in der xy-Ebene beschreibt und Werkzeug-z exakt
auf Welt-−z abbildet. Der Versatz wirkt dadurch als reine z-Differenz.

| Punkt | x [mm] | y [mm] | z Flansch [mm] | Bandhöhe = z − 245 |
|---|---|---|---|---|
| P1 | −733,80 | +73,69 | 298,22 | 53,22 |
| P2 | −701,14 | −201,85 | 297,94 | 52,94 |
| P3 | −931,06 | −58,10 | 299,53 | 54,53 |

Streuung innerhalb einer Stellung: σ ≤ 0,03 mm — reines Zahlenrauschen.

Ebene durch die drei Punkte: Gefälle **quer (x) 0,39°**, **längs (y) 0,01°**.
Über die 230 mm abgetastete Breite sind das 1,6 mm. Ob das eine echte
Querneigung ist oder drei unterschiedlich fest aufgesetzte Berührungen, lässt
sich mit drei Punkten nicht trennen — die Gummiauflage gibt nach, und 1,6 mm
liegen in genau dieser Größenordnung.

> **Ergebnis B17: Bandoberfläche z = 53,6 mm in `world`, eben innerhalb ±1 mm.**
> Dazu der systematische Anteil ±5 mm aus M8.

**Greifhöhe, daraus abgeleitet.** Auflagenmitte = Backenspitze + 10 mm, also
235 mm unter dem Flansch. Für den stehenden Referenzklotz (50×50×100 mm),
mittig auf halber Höhe gefasst:

**`flansch_z_greifen` = 53,6 + 50 + 235 ≈ 339 mm.**

Allgemein: `flansch_z_greifen = 53,6 + klotzhoehe/2 + 235` (alles in mm),
solange die halbe Klotzhöhe ≥ `min_grip_height_m` aus B15 ist.

### M10 — Die Bandrichtung ist praktisch die y-Achse

P1 → P2 war eine Fahrt „das Band hinunter“. Sie besteht aus **−275,5 mm in y**
und +32,7 mm in x, wobei die x-Komponente aus dem seitlichen Versetzen von Hand
stammt und nicht als Bandrichtung zu lesen ist.

Das stützt die Annahme aus `datenvertraege.md` S1, dass `v_band_gemessen` als
**y-Komponente** geführt wird und nicht als Betrag. Es ersetzt **B1 nicht** —
Betrag und Vorzeichen der Bandgeschwindigkeit bleiben offen, dafür muss das
Band laufen.

### M11 — B9: Ablagepose

Stellung, in der ein geöffneter Greifer den Klotz in die Auffangkiste fallen
lässt.

| Größe | Wert |
|---|---|
| Flanschposition | x = **−316,49** · y = **+476,21** · z = **+419,71** mm |
| Orientierung (w,x,y,z) | 0,006857 · 0,680692 · 0,732524 · −0,004575 |
| Abweichung von der Lotrechten | 0,94° |
| Gelenke [°] | 105,87 · −98,24 · 124,05 · −114,87 · −89,96 · −78,33 |

Abgeleitet: Auflagenmitte bei z = 184,7 mm, Unterkante eines mittig gefassten
100-mm-Klotzes bei z = 134,7 mm — also rund 81 mm über der Bandoberfläche.

Die Pose liegt mit x = −316 · y = +476 weit außerhalb des Bandbereichs
(dort x ≈ −700…−930, y ≈ −200…+75). Der Transfer ist entsprechend eine große
Traverse; deren Zeitbedarf geht in das Budget von D12/B19 ein und ist noch
nicht gemessen.

**Zur Singularität dieser Stellung:** `wrist_2 = −89,96°` liegt maximal weit von
der Handgelenk-Singularität (0° bzw. ±180°) entfernt, der Ellbogen mit 124,05°
weit von der Streckung, und der Abstand zur Basisachse beträgt 571 mm, also
keine Nähe zur Schultersingularität. Die Ablagepose ist unkritisch. Das ersetzt
**B11 nicht**, denn dort geht es um die Bahn entlang des Bandes, nicht um diese
eine Stellung.

⚠️ **Rohdaten-Vorbehalt.** Bei dieser letzten Mitschrift war die Streuung über
alle 200 Nachrichten exakt null, während sie bei P1–P3 bei 0,01–0,03 mm lag.
Nachrichten kamen also weiter an, trugen aber identische Werte — ein Hinweis
darauf, dass der Treiber zu diesem Zeitpunkt keine frischen RTDE-Daten mehr
einspeiste (Programm auf dem Pendant gestoppt). Die Werte sind plausibel und
unterscheiden sich deutlich von den vorherigen Stellungen, sind also nach der
Bewegung übernommen worden. Vor der Übernahme in eine Konfiguration sollte die
Pose dennoch **einmal bei laufendem Programm gegengelesen** werden.

### M12 — Was damit für D12/D13 feststeht

`D13` (Ablagepose) ist mit M11 bestimmt. `D12` (Transferhöhe über dem Band)
ergibt sich als Bandhöhe + höchster erwarteter Klotz + Luft, also
53,6 + 100 + Reserve — mit dem Referenzklotz rund **200 mm** Unterkante, in
Flanschmaß **200 + 245 = 445 mm**. Der Wert ist erst verbindlich, wenn die
größte auftretende Klotzhöhe festgelegt ist.
