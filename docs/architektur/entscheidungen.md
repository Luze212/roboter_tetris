# Entscheidungsprotokoll Robotetris

Laufendes Protokoll der Architekturentscheidungen. Entsteht Thema für Thema und
ist die Grundlage für die Aktualisierung des Word-Dokuments und den
Umsetzungsplan.

**Lesereihenfolge:** Die Themen legen die Architektur fest, die Nachträge
korrigieren und ergänzen sie. **Ein neuerer Nachtrag geht vor**, wo er eine
frühere Festlegung berührt; an der alten Stelle steht dann ein Verweis.

| Abschnitt | Inhalt |
|---|---|
| Themen 1–7 | Bewegung, Zeit, Frames, Komponentenschnitt, Verträge, Zustandsautomat, Sicherheit |
| Nachtrag (1) | Restbefunde P1, P2, P4, R3, I2 |
| Nachtrag 2 | Kritische Durchsicht (13.09.): F1–F3 Freigabe, Einfrieren, Rücksprung; K1, K2, L1, L2 |
| Nachtrag 3 | Durchsicht gegen den Code (14.09.): N1–N5 |
| Nachträge 4, 5 | Messungen am Aufbau und am Roboter (14./15.09.): M1–M12 |
| **Nachtrag 6** | **Projektvorgaben (21.09.):** Z1 Ziele, Z2–Z9 Geschwindigkeitsschätzung, Z10 Greifen ohne Roboterkamera, Z11 Greifebene, Z12 kein Fallenlassen, Z13 Orientierungsgüte |
| Nachtrag 7 | Bau `priority_handler` (H1–H5) und `data_tracker` (T1–T2) |
| Nachtrag 8 | Bau `object_follower` 4a: F1 Bandlage (B23), F2–F6 |
| Nachtrag 9 | Bau `object_follower` 4b: G1–G9 |
| Nachtrag 10 | Bau `object_follower` 4c/4d: J1 Freihöhe, J2 Absenkzeit, J3–J8 |
| Nachtrag 11 | Bau `interface_streamer`: V1–V3 |
| Nachtrag 12 | Inbetriebnahme am Aufbau (22.09.): K1–K6, u. a. 180° `base`/`world`, falsche Gier der Altkalibrierung |
| **Nachtrag 13** | **Audit und Aufbau (23.09.):** L1 Audit, L2 Datenrate/Latenz, L3 Hardware-Takt, L4 Greifer im Bild, L5 Roboterkamera, L6 B23 abgeschlossen (Neigung, Parallaxe, neue Extrinsik), L7 Bandgeschwindigkeit, L8 Standardwerte, L9 Block 3 und Pool-Mindestgeschwindigkeit, L10 Weiterführung hinter dem Bild, L11 Roboterkamera und Hand-Auge, L12 Nah-Gate ohne fehlende Tiefe, L13 Erkennung der Roboterkamera, L14 Arbeitsraum und Greifzone, L15 Standardwerte für Arbeitsraum und Beobachtungspose (24.09.), L16 Kameras 15 Bilder/s, Infrarot aus (24.09.), L17 Follower ohne Signale, Fake-Zeitstempel (24.09.), L18 Block 7 gedrosselt bestanden (24.09.), **L19 erste echte Griffe im Lauf** (24.09.), L20 Schutzstopp am Bandrand: Nutzlast und Beschleunigung (24.09.), L21 Greifzone = Arbeitsraum, Bildausschnitt der Basiskamera (24.09.) |

⚠️ Namensgleichheit: **F1–F3 in Nachtrag 2** und **F1–F6 in Nachtrag 8** sind
verschiedene Punkte — im Text immer mit Nachtragsnummer zitiert.

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
| **A — Vorhalt** | `v_band / K` ⚠️ jetzt als Zeit `lead_time_s` (Nachtrag 6 / Z6) | nein | **Primärweg**, funktioniert sicher |
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
| `lead_offset_m` → **`lead_time_s`** | Nachlauf des Attractors (`v/K` → Zeit `1/K`, Nachtrag 6 / Z6) | **0** |
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

**Bestätigt (A7/A8):** Der Greifer steht **nicht** im URDF. Damit regelt der
IK-Velocity-Controller den Flansch und der `robot_state_broadcaster` meldet den
Flansch.

**Entscheidung:** URDF nicht anfassen. Das nachträgliche Einpflegen des Greifers
würde die laufende Hardware-Konfiguration und das geregelte Frame verschieben,
also genau das bereits getestete Verhalten. Stattdessen rechnet der
`object_follower`:

```
ziel_flansch = ziel_greifpunkt + [0, 0, flange_to_grip_point_m]
```

Das ist **eine einzige Zahl**, weil der Greifer immer senkrecht nach unten zeigt
und der Greifpunkt auf der Flanschachse liegt. Eine Drehung des Handgelenks um
die Hochachse ändert den Versatz nicht. Erst bei gekipptem Greifer würde daraus
eine echte Posenverkettung.

Wert: **0,235 m**, Flansch → Griffpunkt (Auflagenmitte), gemessen. ⚠️ Hier stand
„aus dem Vorgängerprojekt auslesen (C8)" und der Parameter hieß `tool_offset_z_m`.
Beides war falsch und hätte bei flachen Klötzen einen Crash ins Band erzeugt —
Herleitung und Umbenennung in Nachtrag 6 / Z7.

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

→ ~~Richtung und Geschwindigkeit fallen aus einem normalen Testlauf ab.~~
⚠️ **Überholt durch Nachtrag 6 / Z2:** Ziel 3 verlangt ein Verfahren zur
Geschwindigkeitsschätzung. Richtung und Geschwindigkeit werden deshalb **zur
Laufzeit aus den Bilddaten geschätzt** — je Objekt und gepoolt über die finalen
Klötze eines Durchlaufs. Die Genauigkeitsrechnung oben bleibt gültig und stützt
gerade den Pool: Ungenau wird es nur über ein kurzes Bahnstück, und das fällt
beim Poolen über viele Klötze weg.

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
| Bandrichtung + Bandgeschwindigkeit | **wird zur Laufzeit geschätzt**, nicht kalibriert (`vectoring`) | Nachtrag 6 / Z2; B1 nur Gegenprobe |
| Flansch → Griffpunkt, `flange_to_grip_point_m` | **`object_follower`** (neu) | **0,235 m, gemessen** (Nachtrag 6 / Z7) |
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
2. ~~**`vectoring` schrumpft drastisch**, weil Bandrichtung und -geschwindigkeit
   seit Thema 3 Konstanten sind.~~ ⚠️ **Überholt durch Nachtrag 6 / Z2:**
   `vectoring` ist die Komponente, die die Geschwindigkeit **schätzt**.

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
| ~~Plausibilität~~ | ⚠️ ersetzt durch Geschwindigkeitsschätzung und Einschwingen (Nachtrag 6 / Z2, Z3) |
| Verfall | ausbleibende IDs nach kurzer Frist vergessen |

Genauigkeitsgewinn: bei ~3 mm Rauschen je Bild und 30 Bildern bleiben quer
**0,55 mm** — Faktor 5, und zwar in der Richtung mit dem geringsten Greiferspiel.

**Entfallen ersatzlos:** Parameter `K`, `W`, `J`, `H` sowie die Schalter
"Richtung halten" / "Geschwindigkeit halten". Übrig bleibt im Wesentlichen die
Fensterlänge der Mittelung.

~~**Neu hinzugekommen:** der Plausibilitätstest — das Signal für „Block ist
heruntergefallen oder hängengeblieben".~~ ⚠️ **Entfallen (Nachtrag 6 / Z3):**
Die Annahme war falsch. Klötze bewegen sich frei mit dem Band, festhängende gibt
es nicht; sie können nur beim Aufsetzen umkippen. An die Stelle des Tests tritt
das **Einschwingen**: Status 3, bis die Geschwindigkeit konstant gemessen ist,
dann Status 0 mit eingefrorener Geschwindigkeit.

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
- "erreichbar" — aus Position, **geschätzter** Geschwindigkeit, Zeitbudget

Die Information floss von Anfang an in die andere Richtung; der Zyklus im Plan
war eine Verdrahtung entgegen der natürlichen Flussrichtung.

### Eigentümerschaft

Regel: **Jede Information hat genau einen Eigentümer.** Alle anderen bekommen sie
von ihm, niemand rechnet parallel nach.

| Komponente | besitzt | besitzt nicht |
|---|---|---|
| `base_cam` | rohe Detektionen, IDs, Zeitstempel | keine Bahnlogik |
| `vectoring` | **Geschwindigkeitsschätzung** (je Objekt + Pool), geglättete Objektzustände, Einschwingstatus | keine Auswahl, keine Flags |
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
Geschwindigkeit global allen Tracks zu (`set_global_velocity`) — als Objektfeld
wäre der Wert irreführend, weil er gar nicht je Objekt gemessen ist. Als Kopffeld
dient er der groben Laufkontrolle des Bandes. Die maßgebliche Geschwindigkeit
schätzt `vectoring` selbst (Nachtrag 6 / Z2).

**`picked_id` mit laufender Nummer** `[seq, id, outcome]` ersetzt das
"1 Sekunde lang True"-Muster. Verbraucher merken sich die letzte `seq` und
reagieren genau einmal — kein Zeitfenster, keine Flankenerkennung, kein
Verpassen bei Lastspitzen. `outcome` liefert zusätzlich, ob der Griff geklappt
hat (im Plan gar nicht vorgesehen).

### Einzige Änderung an einer laufenden Komponente

`robotiq_gripper` bekommt **zwei Bool-Ausgänge**: `motion_done` und `has_object`.
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
| `ABBRUCH` | **ohne Klotz:** TCP-Pose halten, Greifer öffnen, senkrecht hoch, zur Beobachtungspose · **mit Klotz im Greifer:** Greifer bleibt zu, senkrecht hoch, zur Ablagepose, dort öffnen (Nachtrag 6 / Z12) | → `WARTEN` | – |

**Abbruchgründe bis der Klotz gehalten wird** (`ANFAHREN` bis `GREIFEN`):
- `has_target = 0` — `priority_handler` hat das Ziel zurückgezogen
- Ziel-ID aus `tracks` verschwunden (Thema 2: bei `base_cam` das richtige Kriterium, nicht das Messungsalter)
- unplausibler Sprung der Regelabweichung (Fehldetektion, ID-Verwechslung)

> ⚠️ **Korrigiert (Nachtrag 6 / Z12).** Hier stand „aus jedem Zustand" und für
> `ABBRUCH` pauschal „Greifer öffnen". Nach einem gelungenen Griff hebt der Roboter
> den Klotz vom Band; die Basiskamera verliert ihn dort zwangsläufig, die ID
> verschwindet, das Ziel wird zurückgezogen — und der Follower hätte **jeden
> gelungenen Griff wieder fallen lassen**, über dem Band oder auf dem Weg zur
> Kiste. Ab `has_object` sind die drei Gründe oben keine Abbruchgründe mehr.

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
Die untere Grenze schützt das Band: Die Greifauflagen sind **20 mm** hoch (B15);
bei einem 20 mm flachen Block läge ihre Unterkante ohne Grenze rechnerisch genau
auf dem Band. `min_greifhoehe` = halbe Auflagenhöhe + Luft = **15 mm** (Nachtrag 6 / Z7).

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

> ⚠️ **Geändert beim Bau (Nachtrag 7 / H2):** Geprüft wird die **Diagonale**. Den
> Gierwinkel entscheidet der Follower; der `priority_handler` kennt ihn nicht.

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

`WARTEN` und `ABLEGEN` fahren stehende Ziele an. Dort muss der Vorhalt null sein
— er kompensiert den Nachlauf bei **bewegtem** Ziel. Bliebe er stehen, parkte der
Roboter dauerhaft einige Zentimeter neben Beobachtungs- und Ablageposition.

> ✅ **Seit Nachtrag 6 / Z6 automatisch erfüllt.** Der Vorhalt ist jetzt
> `v · lead_time_s`; ein stehendes Ziel hat `v = 0`, der Vorhalt wird von selbst
> null. Der Sonderfall im Code — und damit die Stelle, an der ein Fehler still
> bliebe — entfällt.

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
ziel = p₀ + v_geschätzt · (t_jetzt − t₀)     ← wächst unbegrenzt
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
> ✅ **B10 festgelegt 23.09.2026** (Nachtrag 13 / L14), `status: defined`.

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
> `robot-cam-befunde.md` §9.7/§9.8 — **seit Phase 2.2 umgesetzt**, gemeinsam für
> beide Varianten. Bei 2–3 Blöcken kann ein zweiter ins Bild geraten.

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
t_verfügbar = (zonenende − blockposition) / v_geschätzt     (Nachtrag 6 / Z2)

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
| neu einschwingend | Status fällt auf 3 zurück — anhaltende Abweichung, Schätzung neu gestartet (Nachtrag 6 / Z3) |
| ~~unplausibel~~ | *entfallen mit den Statuscodes 1/2 (Nachtrag 6 / Z3)* |

**Erreichbarkeit gehört bewusst NICHT dazu** — sie ist Auswahl-, kein
Abbruchkriterium. Sonst würde ein fast erfolgreicher Griff abgebrochen, sobald der
Block beim Greifen die Zonengrenze überschreitet. Die Greifzone ist enger als der
Sicherheitsraum; der Roboter darf dem Block ein Stück darüber hinaus folgen.

> ⚠️ **Ergänzt durch Nachtrag 6 / Z11 (Greifebene).** Das bleibt für einen
> **laufenden** Griff gültig. Hinzu kommt eine Grenze für den **Beginn**: Hat der
> Block die Greifebene überschritten, bevor abgesenkt wurde, bricht der Follower
> ab (`outcome = 3`). Der `priority_handler` zieht das Ziel dann über `picked_id`
> zurück — weiterhin ohne Rückmeldung des Follower-Zustands.

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

> ⚠️ Die Folgerung „Greifzone innerhalb der Messregion“ ist mit Nachtrag 13 / L4
> aufgegeben: Die Zone liegt außerhalb des Bildes der Basiskamera.

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
| Plausibilität in `vectoring` *(entfallen, Nachtrag 6 / Z3)* | Status 1 („steht/verklemmt") vergleicht die Längsgeschwindigkeit gegen `belt_speed_mps`. Außerhalb der Region *ist* die Längsbewegung per Konstruktion die Bandgeschwindigkeit — die Prüfung ist tautologisch und kann dort nie auslösen. |
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

> ~~Der ursprüngliche Zweck der Messregion ist für uns ohnehin entfallen.~~
> ⚠️ **Überholt durch Nachtrag 6 / Z4:** Mit Ziel 3 ist die
> Geschwindigkeitsschätzung zurück — und damit der Grund, warum das Koppeln
> schadet. Entschieden ist deshalb: **Der Tracker verwendet überall die gemessene
> Längsposition**, gekoppelt wird nur noch bei verpassten Bildern (Zweig 1 der
> Region). Die Löschregel (Zweig 2) bleibt, und damit auch die Kopplung der
> Region an die Greifzone. Die Phantom-Folge oben ist gegenstandslos: Klötze
> fallen nur am Bandende und werden nie von Hand entnommen (Z5).

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

> ⚠️ **Überholt durch Nachtrag 13 / L6:** Der Bias galt nur für diese Klotzhöhe (ein
> 25-mm-Klotz kam auf +12 mm), und die Ecken lagen auf Bandhöhe (Parallaxe).
> `HEIGHT_BIAS_MM` ist entfernt.

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

> **Nachtrag 7 / H2:** Der `priority_handler` prüft inzwischen die Diagonale und
> braucht die Schwelle nicht mehr.

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
→ tool_offset_z_m = 0,215      ⚠️ FALSCH für den Follower, siehe unten
```

> ⚠️ **Korrektur (Nachtrag 6 / Z7).** Die 215 mm sind der **konfigurierte TCP der
> UR-Steuerung** und dienen nur der Umrechnung fremder TCP-Werte (M8). Die
> Zielpose des Followers braucht den Abstand **Flansch → Griffpunkt = 0,235 m**; der
> Parameter heißt dafür jetzt `flange_to_grip_point_m`. Mit 0,215 wäre die
> Backenspitze bei einem 25-mm-Klotz 17,5 mm ins Band gefahren.

Gegenprobe über die Vorwärtskinematik (UR10e-DH aus den Gelenkwinkeln gegen
`getActualTCPPose`): −0,49 / −0,43 / **214,22** mm. Abweichung unter 0,8 mm, das
liegt im Rahmen der DH-Konstanten — der Wert ist bestätigt.

**Zwei Dinge bestätigen sich damit:**

Die Warnung aus Thema 3 und Phase 0.1 war der Größenordnung nach genau richtig —
„fährt der Greifer rund 20 cm zu tief, ins Band". Es sind **21,5 cm**.

Und die Vereinfachung hält: **x = y = 0 und keine Verdrehung.** Der Greifpunkt
liegt exakt auf der Flanschachse, der Versatz ist damit tatsächlich *eine einzige
Zahl*, wie Thema 3 angenommen hatte. `ziel_flansch = ziel_greifpunkt + [0, 0,
flange_to_grip_point_m]` ist keine Näherung, sondern exakt — mit dem richtigen
Wert 0,235 m, nicht 0,215 (Nachtrag 6 / Z7).

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
Flanschmaß **200 + 245 = 445 mm** (⚠️ vergaß den gehaltenen Klotz, korrigiert auf
0,49 m in Nachtrag 10 / J1). Der Wert ist erst verbindlich, wenn die
größte auftretende Klotzhöhe festgelegt ist.

---

## Nachtrag 6 — Projektvorgaben und Geschwindigkeitsschätzung (21.09.2026)

**Status:** entschieden
**Anlass:** Die exakten Projektvorgaben lagen vor. Ziel 3 verlangt ein
**entwickeltes Verfahren zur Geschwindigkeitsschätzung** — genau das hatten
Thema 3 und Thema 4 gestrichen. Bei der Prüfung der Doku gegen die Vorgaben
kamen außerdem ein sicherheitsrelevanter Wertefehler (Z7) und ein
Missverständnis über die Klotzphysik (Z3) zutage.

> **Vorrang.** Dieser Nachtrag geht allen früheren Festlegungen vor, die er
> berührt. An den betroffenen Stellen der Themen 3, 4, 6 und 7 sowie in N2 und M6
> steht ein Verweis hierher.

### Z1 — Die Projektziele

| Nr. | Ziel | Umsetzung |
|---|---|---|
| 1 | Ansteuerung des UR10e mit AICA | ✅ läuft am Aufbau |
| 2 | Entwicklung eines schnellen Kalibrierungsverfahrens | Kommilitone, eigenes Projekt (`Calibration/*`). **Wir sind Abnehmer**; offen ist nur die Übergabeform (C6). |
| 3 | Verfahren zur **Geschwindigkeitsschätzung** und Positionsberechnung der Gegenstände | `base_cam` (Position), `vectoring` (Geschwindigkeit) — Z2 bis Z5 |
| 4 | Algorithmus zur **Priorisierung** und zur **Bahnplanung** für das kontrollierte Greifen | `priority_handler`; Bahnplanung über die AICA-Bausteine (Z8) |

Zusätzlich, als eigenes Ziel außerhalb dieser Liste: **den Prozess nachvollziehbar
darstellen** — `data_tracker` und `interface_streamer`. Die Reihenfolge bleibt die
des Umsetzungsplans: `data_tracker` (3.2) ist ein Blatt ohne Rückwirkung auf den
Regelpfad und kann jederzeit nach 3.1 kommen; `interface_streamer` wird zuletzt
gebaut (Nachtrag P1–I2, Abschnitt I2).

### Z2 — Die Geschwindigkeit wird geschätzt, nicht kalibriert

**Hebt auf:** Thema 3 („Richtung und Geschwindigkeit fallen aus einem normalen
Testlauf ab") und Thema 4 („`vectoring` schrumpft drastisch, weil Bandrichtung und
-geschwindigkeit seit Thema 3 Konstanten sind"), dazu die YAGNI-Zeile der
`vectoring`-Spec „Keine Regression für Richtung oder Geschwindigkeit".

`vectoring` schätzt die Geschwindigkeit **zur Laufzeit aus den Bilddaten**, und
zwar in zwei Stufen:

1. **Je Objekt ein Geschwindigkeitsvektor** `(vx, vy)` aus der eigenen
   Positionshistorie — Richtung und Betrag zugleich, ohne Bandmodell.
2. **Ein Pool** über alle Objekte, die im laufenden Durchlauf final geworden sind
   (Z3): eine **gemeinsame Ausgleichsrechnung über alle ihre Messungen seit dem
   Einschwingen** — eine gemeinsame Geschwindigkeit, je Klotz ein eigener
   Achsenabschnitt (Z9). Das ist die Bandgeschwindigkeit, mit der Prädiktion und
   Priorisierung rechnen. Sie wird mit jeder Messung genauer.

**Warum der Pool richtig ist:** Das Band läuft mit konstanter, nicht einstellbarer
Geschwindigkeit, und die Klötze bewegen sich frei mit ihm. Alle finalen Klötze
messen also dieselbe Größe. Thema 3 hatte selbst das stärkste Argument dafür
geliefert — *„Ungenau wird eine Richtungsschätzung nur aus einem kurzen
Bahnstück"*. Über viele Klötze gepoolt entfällt genau das. Zugleich bekommt ein
frisch aufgelegter Klotz vom ersten Bild an eine gute Geschwindigkeit für die
Vorhersage, statt erst sein eigenes Fenster zu füllen.

**Gilt weiter aus Thema 3:** Die Richtung wird in denselben Koordinaten geschätzt,
in denen vorhergesagt wird. Ein kleiner Drehfehler der Basiskamera-Extrinsik hebt
sich für die Vorhersage deshalb weitgehend auf.

**Folge für B1:** Keine Voraussetzung mehr, sondern **Gegenprobe** (Stoppuhr,
Markierung auf dem Band). Sie prüft dabei die **ganze Kette**, nicht nur den
Schätzer: Ein Maßstabsfehler in Tiefe oder Extrinsik skaliert die geschätzte
Geschwindigkeit mit, und das sieht nur eine unabhängige Messung.

**Phase 3.1 lässt sich ohne Messung bauen** und gegen die Ground Truth aus
`fake_objects.py` prüfen. Ob die Schätzung am realen Band trägt, hängt über den
Tracker-Eingriff (Z4) an **B21**.

**Durchlauf** = eine Aktivierung der Komponente. Beim Aktivieren wird der Pool
geleert. Solange er leer ist, gibt es **keinen** Schätzwert — das wird im Vertrag
ausdrücklich gemeldet (S3-Kopf, `n_pool = 0`) und nicht als stilles `0,0`
geliefert. Genau diese Falle hat `v_band` von `base_cam`: Es startet bei 0, und
bis dahin *„koppeln alle Tracks mit Geschwindigkeit null und stehen scheinbar
still"*.

### Z3 — Einschwingen statt Plausibilitätsprüfung

**Hebt auf:** den Plausibilitätstest aus Thema 4 (*„das Signal für ‚Block ist
heruntergefallen oder hängengeblieben'"*) und die S3-Statuscodes 1 und 2.

**Die Annahme dahinter war falsch.** Die Klötze werden frei und ungehindert auf
das Band gelegt und bewegen sich mit Bandgeschwindigkeit — festhängende oder
angestoßene Klötze gibt es nicht. Was es gibt: Ein Klotz **kann beim Aufsetzen
umkippen**. Danach läuft er mit dem Band.

Jeder Klotz durchläuft deshalb genau zwei Phasen:

| Phase | S3-Status | Geschwindigkeit | Geometrie |
|---|---|---|---|
| **Einschwingen** | 3 | wird geschätzt, ist noch nicht konstant | nur Anzeige |
| **Final** | 0 | konstant gemessen; alle Messungen ab hier fließen in den Pool | wird ab hier gemittelt |

Der Übergang erfolgt, sobald die Geschwindigkeit **konstant gemessen** ist: **Zwei
aufeinanderfolgende Halbsekundenfenster messen dieselbe Geschwindigkeit** (Z9).
Ein kippender Klotz springt im Schwerpunkt; solange der Sprung in einem der beiden
Fenster liegt, weichen sie voneinander ab, und der Klotz bleibt in Phase 3.

**„Final" ist ein Status, kein eingefrorener Wert.** Der Klotz wird ab dann nicht
mehr in Frage gestellt — er ist wählbar und bleibt es. Seine Geschwindigkeit *ist*
physikalisch die Bandgeschwindigkeit; deren **Schätzung** sammelt aus allen
weiteren Messungen weiter (Z9).

**Position, Geometrie und Orientierung erst ab „Final".** Ein umgekippter Klotz hat
andere Abmessungen — ein stehender 100-mm-Klotz liegt danach mit 50 mm Höhe —, und
sein Schwerpunkt springt um rund die halbe Kantenlänge. Würde vom ersten Bild an
gemittelt, entstünde ein Mischwert aus stehend und liegend, in der Höhe wie in der
Lage. Die Höhe bestimmt direkt die Greifhöhe.

**Fehldetektionen** — ein einzelnes Bild, in dem die Position springt — sind ein
Sensoreffekt, keine Klotzphysik. Sie bekommen keinen Status: Die einzelne Messung
wird verworfen, der Track bleibt. Hält eine Abweichung dagegen über mehrere
Bilder an, ist sie eine echte Lageänderung (etwa das Umkippen), und die Schätzung
beginnt neu.

**Die Codes 1 und 2 entfallen und werden nicht wiederverwendet.** So kann eine
ältere Notiz, die von „Status 1" spricht, nie etwas anderes bedeuten als früher.

### Z4 — Der Tracker muss messen, nicht rechnen

`vision/tracker.py` behandelt die Messregion (`track_velocity_region_y_min…max`)
auf zwei Arten anders als den Rest — und beide Mechanismen hängen an denselben
zwei Parametern:

| Zweig | außerhalb der Region | Folge |
|---|---|---|
| **1 — Koppeln** (Z. 168–173) | Längsposition wird **gerechnet** (`y = prev_y + vy·dt`), **auch bei frischer Detektion** | Geschwindigkeitsschätzung zirkulär; Umkippen unsichtbar |
| **2 — Löschen** (Z. 233–238) | Löschen nur an den Bandgrenzen statt nach drei verpassten Bildern | schützt Tracks am Bildrand vor dem Abreißen |

Zweig 1 macht Z2 und Z3 unmöglich. Wer aus gerechneten Positionen eine
Geschwindigkeit schätzt, bekommt die hineingesteckte zurück. Und die Klötze werden
**vorn aufgelegt**, also im gekoppelten Bereich: Ein kippender Klotz zeigt dort in
der Längsposition eine vollkommen glatte Bewegung, die Einschwing-Erkennung sähe
ihn sofort als final.

**Entscheidung: Nur Zweig 1 wird geändert.** Bei vorhandener Detektion gilt
überall die gemessene Position; fortgeschrieben wird nur noch, wenn ein Klotz in
einem Bild fehlt. **Zweig 2 bleibt unverändert** — das Aufweiten der Region auf
das ganze Band hat N2 zu Recht verworfen, weil dann Tracks am Bildrand ständig
neue IDs bekommen und der Ziel-Lock verloren geht.

⚠️ **Am Aufbau zu prüfen (B21):** Der Kommentar im übernommenen Code begründet das
Koppeln mit *„outside the trusted region the measured y is unreliable"*. Das
stammt aus dem Aufbau der Vorgängergruppe und ist für unseren nicht gemessen — M5
hat die Streuung nur bei y = −874 mm bestimmt, also innerhalb der Region. Prüfung
wie M5: ein ruhender Klotz an mehreren y-Positionen über den ganzen Sichtbereich.

### Z5 — Abwurf nur am Bandende, und berechenbar

Klötze fallen **nur am Bandende** herunter, und nur, wenn sie nicht rechtzeitig
gegriffen wurden. Mitten auf dem Band gehen keine verloren, von Hand entnommen
wird keiner.

Damit ist der Abwurf **kein Störfall, sondern vorhersagbar**: Aus Position und
geschätzter Geschwindigkeit folgt, wann ein Klotz die Greifzone verlässt — das
ist `t_verfügbar` im `priority_handler` und `out_of_bounds` im `data_tracker`.

Die Phantom-Sorge aus N2 (ein weggenommener Klotz gleitet gerechnet weiter) ist
damit gegenstandslos: Weggenommen wird ein Klotz nur vom eigenen Greifer, und den
meldet `picked_id`.

### Z6 — Vorhalt als Zeit statt als Strecke

`lead_offset_m` war eine kalibrierte Strecke, gedacht als `v_band / K`. Mit
geschätzter Geschwindigkeit wird der Vorhalt als **Zeit** geführt:

```
vorhalt = v_geschätzt · lead_time_s
```

Er bleibt damit richtig, auch wenn die Bandgeschwindigkeit zwischen Durchläufen
leicht abweicht. B4 misst eine Zeit ein statt einer Strecke. Theoretisch ist
`lead_time_s ≈ 1/K`, bei K = 5 also 0,2 s.

Nebenbei wird die Zahl der Vorgängergruppe unmittelbar vergleichbar: Deren
*„eine empirische Konstante von 0,7 s"* (D1) war bereits eine Zeit.

`latency_compensation_s` bleibt getrennt (Thema 2), beide werden mit derselben
Geschwindigkeit multipliziert.

### Z7 — Werkzeugversatz: 0,235 m, und ein neuer Name

**Korrigiert:** M6 und alle Stellen, die `tool_offset_z_m = 0,215` angeben.

Unter dem Flansch liegen **drei** verschiedene Punkte:

| Abstand | Wert | Verwendung |
|---|---|---|
| konfigurierter TCP der UR-Steuerung | 215 mm | **nur** Umrechnung fremder TCP-Werte (M8) |
| Flansch → Griffpunkt (Auflagenmitte) | **235 mm** | **Zielpose des Followers** |
| Flansch → Backenspitze | 245 mm, ±5 mm | Höhen am Aufbau (M8/M9) |

Die Follower-Spec definiert den Parameter als *„Flansch → Greifpunkt"* — das sind
**235 mm**. Eingetragen waren aber 215 mm. Die Ursache ist eine Reihenfolge: M6
las 215 mm aus der Steuerung und erklärte Phase 0 für abgeschlossen; M8 erkannte
am selben Tag, dass die Kette den Flansch regelt und 215 mm der falsche Bezug
ist, ging aber nicht zurück zu M6 und den Parameterangaben.

**Die Folge wäre ein Crash gewesen.** Bei einem 25-mm-Klotz liegt der Griffpunkt
bei 53,6 + 12,5 = 66,1 mm. Mit 0,215 wird der Flansch auf 281,1 mm kommandiert,
die Backenspitze läge bei 281,1 − 245 = **36,1 mm — 17,5 mm unter der
Bandoberfläche.** Beim 100-mm-Referenzklotz fiele der Fehler nicht auf; der
Greifer fasst dort nur 20 mm tiefer an. Er hätte sich erst beim ersten flachen
Klotz gezeigt.

**Entscheidungen:**

- Wert **0,235 m**.
- Der Parameter heißt ab jetzt **`flange_to_grip_point_m`**. Die Namensgleichheit
  mit dem „tool offset" der UR-Steuerung hat die Verwechslung erst möglich
  gemacht.
- **Luft in `min_greifhoehe` von 2 auf 5 mm** → `min_grip_height_m = 0,015`
  (halbe Auflagenhöhe 10 mm + 5 mm), `min_graspable_height_m ≈ 0,030`. Grund: Die
  bisherigen 2 mm Luft waren kleiner als die ±5 mm Unsicherheit der 245-mm-Messung.
  ⚠️ **Folge:** Klötze unter rund 30 mm Höhe gelten als nicht greifbar, bis die
  245 mm beim ersten Testgriff bestätigt sind. Danach kann die Luft wieder sinken.

### Z8 — Bahnplanung über die AICA-Bausteine

Die Bahnplanung für das kontrollierte Greifen (Ziel 4) wird durch die
AICA-Bausteine geleistet: Der `object_follower` gibt die Zielpose vor, der
`SignalPointAttractor` und der `IKVelocityController` erzeugen daraus die Bewegung.
Die Nutzung der von AICA bereitgestellten Werkzeuge gilt als Lösung der Vorgabe.
Thema 1 bleibt damit unverändert gültig.

### Z9 — Nachgerechnet: Einschwingkriterium und Pool korrigiert (21.09.2026)

Die erste Fassung von Z2/Z3 hatte zwei Schwächen. Beide sind durch Simulation
belegt (30 Hz, 0,1 m/s, Rauschen 0,5 mm wie in M5 und pessimistisch 3 mm, je
300–400 Durchläufe).

**1. Das Kriterium „Streuung der Schätzung unter einer Schwelle" versagte genau
beim Umkippen.** Eine Ausgleichsgerade durch einen Sprung schluckt den Sprung als
zusätzliche Steigung; die Residuen bleiben mäßig, der Standardfehler klein. Es
misst die *Präzision* der Steigung, nicht ob die Bewegung *konstant* ist.

| Kriterium | Umkippen | fälschlich final | Geschwindigkeitsfehler |
|---|---|---|---|
| Standardfehler < 5 mm/s (erste Fassung) | 13 mm | **100 %** | 16 mm/s |
| | 25 mm | **100 %** | **31 mm/s** |
| **zwei Halbsekundenfenster stimmen überein** | 13 mm | **0 %** | 1,7 mm/s |
| | 25 mm | **0 %** | 0,3 mm/s |

(Rauschen 0,5 mm. Bei 3 mm muss die Toleranz wachsen; dann werden 20–30 % der
gekippten Klötze final, aber mit 3–4 mm/s statt 24–31 mm/s Fehler. Welches
Rauschen am **bewegten** Klotz auftritt, ist nicht gemessen — M5 galt einem
ruhenden.) Preis des neuen Kriteriums: rund 0,3 s späteres Einschwingen.

**2. Das Einfrieren verschenkte fast die gesamte Genauigkeit.** Der Pool mittelte
je Klotz einen Schnappschuss — den kürzesten, frühesten und am stärksten
gestörten Teil der Bahn. Über die volle Bahn (~13 s im Bild) gemessen ist die
Schätzung **160- bis 280-mal genauer** (4 Klötze: 0,84 → 0,003 mm/s bei 0,5 mm
Rauschen).

Für die *Regelung* hätten auch die Schnappschüsse gereicht — die Prädiktion
überbrückt höchstens 0,2 s. Aber Ziel 3 verlangt das Verfahren selbst, und ein
Verfahren, das 99 % der verfügbaren Messungen verwirft, ist nicht zu verteidigen.

**Neu gilt:**

- **Einschwingen:** Die Geschwindigkeit der jüngeren Halbsekunde stimmt mit der
  der älteren überein (`settle_v_tolerance`). Das ist wörtlich „konstant
  gemessen".
- **Pool:** gemeinsame Ausgleichsrechnung über alle Messungen seit dem
  Einschwingen, aller Klötze des Durchlaufs — gemeinsame Steigung, je Klotz ein
  eigener Achsenabschnitt. Das entspricht einem nach Messumfang gewichteten
  Mittel der Einzelsteigungen.
- **Geschwindigkeit je Klotz** (S3-Felder 10/11): fortlaufend aus seinen eigenen
  Messungen seit dem Einschwingen, nicht eingefroren — Anzeige und Diagnose.

Kaltstart geprüft und unkritisch: Der erste Klotz eines Durchlaufs ist nach rund
1,3 s final, bei 0,1 m/s nach 13 cm von rund 1,45 m Bahn im Bild.

### Z10 — Greifen ohne Roboterkamera vorerst erlaubt (21.09.2026)

Im Umsetzungsplan hing der erste vollständige Greifzyklus (4d) an 4c, und 4c an
**B6** — der Punkt, an dem die Roboterkamera am 15.09.2026 durchgefallen ist.
Solange sie nicht funktioniert, wäre damit **kein einziger Griff** testbar gewesen,
obwohl Ziel 3 und 4 auch mit der Basiskamera allein nachweisbar sind.

**Entscheidung:** 4d darf direkt nach 4b mit `w = 0` gebaut und getestet werden; 4c
wird zur Verbesserung, parallel dazu. `require_robot_cam_for_grasp` bleibt
**`false`** — sieht die Roboterkamera nichts, greift der Roboter auf Grundlage der
Basiskamera-Vorhersage.

**Vorerst:** Ob im Endbetrieb eine Bestätigung durch die Roboterkamera Pflicht wird
(D9/D10), wird entschieden, sobald sie am Aufbau funktioniert.

**Risiko:** Die Genauigkeit hängt dann an der Basiskamera-Extrinsik C3 — vom
Kommilitonen, derzeit ungeprüfte Altwerte. Liegt sie quer um mehr als rund 17 mm
daneben, geht ein Griff am 50-mm-Klotz vorbei. Das ergibt einen Fehlgriff
(`has_object` bleibt aus → `outcome = 1`), keinen Crash: Die Greifhöhe schützt das
Band unabhängig davon.

### Z11 — Die Greifebene (E10, 21.09.2026)

**E10 entschieden:** Ein Ziel darf **stromaufwärts** der Greifzone gewählt werden —
sonst griffe die Begrenzung auf `zone_upstream` nie.

**Dazu neu: die Greifebene.** Die letzte Position entlang des Bandes, von der aus
der gesamte Greifprozess noch vor dem Zonenende durchgeführt werden kann. Sie liegt
vor dem Zonenende:

```
s_greifebene   = s_zonenende − |v_belt| · t_greifprozess · grasp_time_margin
t_greifprozess = t_descend_s + t_grasp_s + t_lift_s
```

`s` ist die Koordinate entlang der geschätzten Bandrichtung, `v_belt` die
Pool-Geschwindigkeit (Z2). `t_lift_s` reicht bis `lift_clearance_m` — so lange
fährt der Roboter beim Heben noch mit dem Band mit.

**Warum berechnet und nicht fest eingetragen:** Wo der Greifprozess noch hinpasst,
hängt an der Bandgeschwindigkeit, und die wird geschätzt. Eine feste Position wäre
bei anderer Geschwindigkeit falsch; messbar sind dagegen die Zeiten des
Greifprozesses (Stufe 4d). Mit den Startwerten 1,0 + 1,0 + 0,5 s und Faktor 1,2
liegt die Ebene bei 0,1 m/s **0,30 m** vor dem Zonenende. (⚠️ `t_descend_s` jetzt
2,0 s, gekoppelt an Beobachtungshöhe und Sinkgeschwindigkeit — damit 0,42 m;
Nachtrag 10 / J2.)

**Die Ebene trennt die beiden Teile, die die Erreichbarkeitsrechnung bisher
vermischte:**

| Abschnitt | Was dort passiert | Wer prüft |
|---|---|---|
| bis zur Greifebene | **Anfahren**: Roboter muss synchron über dem Block stehen | `priority_handler` bei der Auswahl |
| Greifebene → Zonenende | **Greifprozess**: Absenken, Greifen, Heben | durch die Lage der Ebene gesichert |

```
t_verfügbar = (s_greifebene − s_block) / |v_belt|     ← Frist für den Beginn des Absenkens
t_benötigt  = abstand / v_max + 3/K                   ← nur noch das Anfahren
kandidat  ⟺  t_verfügbar > reach_safety_factor · t_benötigt
```

Absenk- und Greifzeit stecken jetzt in der Lage der Ebene statt in `t_benötigt`.
Der Sicherheitsfaktor wirkt damit nur noch auf das, was wirklich schwankt: den
Anfahrweg.

**Im Follower ist die Ebene ein Tor für den Beginn, kein Abbruch mitten im Griff:**

- `FOLGEN` → `ABSENKEN` nur, solange der Block **vor** der Ebene liegt.
- Überschreitet der Block die Ebene in `ANFAHREN` oder `FOLGEN` — auch nach einem
  Rücksprung aus `ABSENKEN` (F3) —, ist der Griff nicht mehr zu schaffen:
  `ABBRUCH` mit **`outcome = 3`, „verpasst"**.
- **Hat das Absenken begonnen, gilt die Ebene nicht mehr.** P4 bleibt: Ein
  laufender Griff wird zu Ende geführt, notfalls über das Zonenende hinaus, begrenzt
  nur durch den Arbeitsraum.

**Eigentum wie bei `zone_upstream` (N3):** Der `priority_handler` besitzt die
Greifzone, die Pool-Geschwindigkeit und die Prozesszeiten; er rechnet die Ebene und
gibt sie als **S4 Feld 15** mit. Der Follower wendet sie nur an.

**Zwei Folgen:**

- `not_pickable` meldet einen Klotz, sobald er die **Greifebene** ungegriffen
  überschritten hat, nicht erst am Zonenende. Das ist früher und genau: Ab dort
  ist er nicht mehr zu holen. Das aktuelle Ziel ist davon ausgenommen, solange der
  Griff läuft.
- **B19 bekommt eine Untergrenze.** Liegt die Ebene stromaufwärts von
  `zone_upstream`, ist die Zone für diese Bandgeschwindigkeit zu kurz und es wäre
  nie etwas greifbar. Der `priority_handler` warnt dann. Die Zone muss länger sein
  als `|v_belt| · t_greifprozess · grasp_time_margin` plus das Fenster, in dem der
  Roboter aufsynchronisiert.

### Z12 — Mit Klotz im Greifer wird nichts fallen gelassen (21.09.2026)

**Gefunden bei der Konsistenzprüfung, unabhängig von der Geschwindigkeitsfrage.**
Thema 6 und die Follower-Spec nannten `has_target = 0` und eine verschwundene
Ziel-ID als Abbruchgrund **aus jedem Zustand**, und `ABBRUCH` öffnete den Greifer
immer. Die Kette war damit sicher, nicht nur möglich:

1. Der Roboter greift erfolgreich und hebt den Klotz (`HEBEN`).
2. Die Basiskamera sieht ihn nicht mehr als Klotz auf dem Band, die ID verschwindet.
3. Der `priority_handler` zieht das Ziel zurück („verloren"), `has_target = 0`.
4. Der Follower bricht ab und **öffnet den Greifer** — über dem Band oder auf dem
   Weg zur Kiste.

Jeder gelungene Griff wäre so verloren gegangen.

**Entscheidungen:**

- **Ab `has_object` sind Zielrückzug, verschwundene ID und Sprünge der
  Regelabweichung keine Abbruchgründe mehr.** Der Klotz *soll* vom Band
  verschwinden; der Follower besitzt ihn jetzt und führt `HEBEN` → `ABLEGEN` →
  `LOESEN` zu Ende. Neue Ziele nimmt er erst in `WARTEN` an.
- **Abbruchgründe mit Klotz im Greifer** bleiben nur: Zeitüberschreitung,
  Sicherheitsgate, stehender Roboterzustand — und `has_object` fällt unerwartet
  weg (Klotz verloren, `outcome = 2`).
- **Bricht der Follower mit Klotz im Greifer ab, bleibt der Greifer zu.** Er fährt
  senkrecht hoch, dann zur Ablagepose und öffnet erst dort. Die Ablagepose ist eine
  feste, geprüfte Pose, das Sicherheitsgate hält auch diesen Weg im Arbeitsraum.
  Landet der Klotz so in der Kiste, meldet `picked_id` `outcome = 0` — er ist
  abgelegt; der Abbruchgrund geht ins Log.
- **Das Mitfahren in `HEBEN` braucht keine Blockvorhersage mehr.** Der Klotz ist im
  Greifer; der Roboter fährt mit der letzten Bandgeschwindigkeit weiter, bis
  `lift_clearance_m` erreicht ist. Der Deckel `max_extrapolation_s` gilt für die
  Vorhersage eines gemessenen Blocks auf dem Band, nicht für diese Bewegung.

**Nicht am Schreibtisch zu klären (B22):** Verdeckt der Greifer den Klotz schon
beim Absenken oder Greifen für die Basiskamera, verschwindet die ID **vor**
`has_object` — dann bricht der Griff ab, und die Vorhersage kann das wegen des
Deckels von 0,2 s nicht überbrücken. Ob das eintritt, hängt an der Lage der
Basiskamera zur Greifzone und zeigt sich in Stufe 4d.

### Z13 — Die Orientierungsgüte reist im Vertrag mit (21.09.2026)

**Gefunden beim Planen der Umsetzung.** Die `vectoring`-Spec ließ den Follower bei
schlechter Orientierungsgüte auf den festen Winkel zurückfallen — aber **kein
Vertragsfeld trug die Güte**, und die Follower-Spec erwähnte sie nicht. Der
Follower hätte es nie erfahren und im Modus 2 auch einen verrauschten Winkel
angefahren.

**Entscheidung:** Die Güte wird als Feld mitgegeben — S3 Feld 13 und S4 Feld 16,
`ori_quality` (0…1). Bewertet wird sie beim Verbraucher: Der Follower verwendet den
Klotzwinkel nur oberhalb von `orientation_quality_min`, sonst die Bandrichtung. Die
Schwelle (D11) wandert damit von `vectoring` zum Follower.

Begründung: Grundregel 6 des Vertrags — *„Statusfelder statt stiller Annahmen"*.
Die Alternative, dass `vectoring` den Winkel stillschweigend durch die Bandrichtung
ersetzt, hätte keine Vertragsänderung gebraucht; aber niemand stromabwärts hätte
gesehen, dass der Winkel nicht gemessen ist. Der Zeitpunkt ist günstig — noch
existiert kein Verbraucher von S3 und S4.

### Was sich **nicht** ändert

- Die **Mittelung von Querposition und Orientierung** (Thema 4) bleibt richtig —
  sie beginnt nur mit dem Übergang auf „Final".
- Die **Winkelmittelung über den doppelten Winkel** (`vectoring`-Spec) bleibt.
- Der **Ziel-Lock** und das Erreichbarkeitskriterium des `priority_handler` bleiben;
  sie rechnen nur mit der geschätzten statt mit einer kalibrierten Geschwindigkeit.
- `base_cam` liefert weiter `v_band_gemessen` im S1-Kopf — als grobe Laufkontrolle,
  nicht mehr als Kalibrierquelle.

---

## Nachtrag 7 — Beim Bau von `priority_handler` und `data_tracker` (21.09.2026)

Beim Bau der Phasen 3.3 und 3.2 ergaben sich Punkte, die die Specs offenließen oder
die Nachtrag 6 überholt hatte. H1, H2 und T1 hat der Nutzer entschieden, die übrigen
folgen aus bestehenden Entscheidungen. **H** betrifft den `priority_handler`, **T**
den `data_tracker`.

### H1 — Der Anfahrweg ist waagerecht, bis zum Anfahrpunkt

**Frage:** Welcher Abstand steht in `t_benötigt = abstand / v_max + 3/K`? Die Spec
sagte „TCP → Klotz". `robot_state` liefert aber den Flansch, und der fährt in
`ANFAHREN` und `FOLGEN` auf Beobachtungshöhe, rund 0,3 m über dem Klotz.

**Entscheidung:** **waagerechter** Abstand Flansch → **Anfahrpunkt**. Anfahrpunkt
ist der Klotz selbst, oder — solange er noch stromaufwärts von `zone_upstream`
liegt — der Punkt auf seiner Spur an `zone_upstream`, wo der Follower tatsächlich
wartet.

**Warum:** Der dreidimensionale Abstand rechnet den Höhenversatz als Fahrweg mit:
rund 0,3 m, also 1,2 s bei 0,25 m/s, mal Sicherheitsfaktor 1,5. Knappe Klötze
würden verworfen, obwohl sie erreichbar sind. Die Höhe ändert sich erst in
`ABSENKEN`, und dessen Zeit steckt schon in der Lage der Greifebene (Z11).

### H2 — Die Greiferbreite wird gegen die Diagonale geprüft

**Frage:** Welche Seite des Klotzes kommt zwischen die Backen? Das entscheidet der
Follower über seinen Gierwinkel (Modus, `orientation_quality_min`,
`gripper_yaw_offset_deg`), und die kennt der `priority_handler` nicht.

**Entscheidung:** `√(length² + width²) ≤ max_gripper_opening_m − gripper_margin_m`.

**Warum:** Die Diagonale ist die größte Ausdehnung in jeder Richtung und gilt damit
für jeden Gierwinkel. Den Gierwinkel nachzubilden hieße, drei Follower-Parameter
doppelt zu pflegen — genau das, was Befund P3 abgeschafft hat. Die Sonderregel für
fast quadratische Klötze (Verhältnis über 0,92 → `max(length, width)`, Nachtrag 4 /
M5) entfällt mit: Vertauschte Achsen ändern die Diagonale nicht. **Preis:** Bei
117 mm fallen erst Klötze wie 70 × 100 mm heraus; der Referenzklotz 50 × 100 hat
112 mm, der breite Testklotz 76 × 50 hat 91 mm.

### H3 — Geprüft wird die Spur, nicht die Lage in der Zone

Die Spec verlangte für Kandidaten „in der Greifzone" und für den Start „Ziel
außerhalb der Greifzone → nicht wählen". Beides widerspricht E10 / Z11: Ein Ziel
darf stromaufwärts gewählt werden. **Umgesetzt:** Die Stelle, an der der Klotz die
Greifebene erreicht, muss in der Zone liegen. Ein Klotz auf einer Spur neben der
Zone wird so nie gewählt, einer davor schon.

### H4 — Die Längskoordinate ist eine Funktion im Vertrag

S4 nannte `zone_upstream` und `grasp_plane` „Koordinate entlang der Bandrichtung",
ohne Ursprung und ohne zu sagen, aus welcher Richtung. Wenn Follower und
`priority_handler` das verschieden auslegen, wartet der Roboter woanders, als die
Erreichbarkeitsrechnung annimmt — der Fehler aus N3.

**Festgelegt:** `s = (x·vx + y·vy) / |v|` mit (vx, vy) aus S4 Feld 13/14 derselben
Nachricht, Ursprung `world`, wächst stromabwärts — als Funktion
`contracts.along_belt`, die beide Seiten aufrufen. Dazu zwei Folgen im Vertrag:

- **S4 Felder 12–15 gelten ohne Ziel nur mit Bandschätzung.** Bisher hieß es, 12
  und 15 blieben bei `has_target = 0` gültig, 13/14 dagegen seien dann
  bedeutungslos. Ohne 13/14 sind 12 und 15 aber nicht lesbar. Jetzt: 12–15 sind
  gültig, solange eine Bandschätzung vorliegt; ohne sie sind alle vier null.
- **Die Zone bei schräger Bandrichtung.** Die Zone ist ein achsparalleles
  Rechteck in `world`. Als Grenzen entlang des Bandes gelten die beiden mittleren
  der vier Eckprojektionen: bei achsparallelem Band genau die Kanten, bei schrägem
  die inneren Werte.

Ohne Richtung (`n_pool = 0`) gibt es keine Koordinate. Unter **0,01 m/s** gilt das
Band als stehend — die Richtung wäre dann Rauschen.

### H5 — Drei kleine Festlegungen

- **`seq = 0` in S7 heißt „kein Versuch".** Der Follower setzt `seq` beim Aktivieren
  auf 0; ab 0 zu zählen hätte den ersten Versuch ununterscheidbar vom Startwert
  gemacht. Der erste Versuch trägt 1, Verbraucher reagieren auf jede Änderung außer
  auf 0.
- **`not_pickable` ist eine aktuelle Liste**, keine Historie: alle Tracks hinter
  der Greifebene außer dem Ziel, jeder Status. Das Festhalten übernimmt der
  `data_tracker`.
- **`zone_empty` ist räumlich:** kein Klotz im Rechteck der Zone, gleich welcher
  Status. Die Lesart „keine Kandidaten" wäre nach der Auswahl dasselbe wie
  `not has_target` gewesen.

### T1 — S10 bekommt das Feld `present` (Nachlauf)

**Befund:** Bei jedem Griff verschwindet der Klotz beim Heben aus dem Bild,
`picked_id` mit `outcome = 0` kommt erst nach dem Ablegen, Sekunden später. Ließe
der `data_tracker` den Eintrag mit dem Track verschwinden, käme `picked` nie an —
das Flag wäre wertlos. Der Eintrag muss den Track also überdauern. Dann zeigt er
aber für einige Sekunden eine eingefrorene Position, als läge der Klotz noch auf
dem Band.

**Entscheidung:** S10 Feld 16 **`present`** (Stride 16 → 17): 1 = in den aktuellen
`tracks`, 0 = Nachlauf mit den letzten bekannten Werten. Grundregel 6 —
*„Statusfelder statt stiller Annahmen"*. Der Zeitpunkt ist günstig: Der einzige
Verbraucher, `interface_streamer`, existiert noch nicht.

**Verfall dazu:** Ein Eintrag ist erledigt, sobald `picked`, `out_of_bounds` oder
`present = 0` gilt, und fällt `expiry_after_done_s` nach der **letzten** dieser
Änderungen heraus. So bleibt ein gerade abgelegter Klotz die volle Frist sichtbar,
obwohl er schon Sekunden vorher aus dem Bild verschwand. Gezählt wird in
**S3-Zeit**: Steht der Eingang, friert die Anzeige ein, statt sich zu leeren. Ein
Eintrag, der noch in `tracks` steht, wenn er abläuft, kommt nicht wieder, bis seine
ID `tracks` verlassen hat.

### T2 — Die `seq`-Regel ist eine Klasse im Vertrag

`priority_handler` und `data_tracker` werten `picked_id` beide nach H5 aus. Die
Regel hat eine Falle: Der Merker muss auch die 0 übernehmen, sonst gilt nach einem
Neustart des Followers eine schon einmal gesehene Nummer als alt. Damit sie nicht
zweimal verschieden gebaut wird, steht sie als `contracts.AttemptWatcher` im
Vertragsmodul — wie `along_belt` (H4) eine gemeinsame Semantik, keine Auslegung.


---

## Nachtrag 8 — Beim Bau des `object_follower`, Stufe 4a (21.09.2026)

F1–F3 hat der Nutzer entschieden, F4–F6 folgen aus bestehenden Entscheidungen
oder aus einem Test.

### F1 — Basiskamera und Roboter sehen das Band an verschiedenen Stellen

> ✅ **Aufgeklärt:** 180° zwischen `base` und `world` (Nachtrag 12 / K5); erledigt
> mit Nachtrag 13 / L6.

**Befund.** Der Roboter hat die Bandoberfläche bei **x = −0,70 … −0,93 m**,
y = −0,20 … +0,07 m angetastet (M9, aus `robot_state_broadcaster` — dasselbe System,
in dem der IK-Controller regelt). Die Basiskamera meldete mit den alten
Kalibrierwerten (C3 offen) einen Klotz bei **x = +0,814**, y = −0,874 (M4). Das
Vorzeichen von x widerspricht sich. Die Beträge passen auffällig gut zusammen: Die
Mitte der Antastpunkte liegt bei x = −0,816. Das spricht für eine Drehung um 180° um
die Hochachse zwischen beiden Systemen, wie zwischen den UR-Rahmen `base` und
`base_link`. Dazu passt ein zweiter Befund: Das Arbeitsraum-Rechteck der
Vorgängergruppe (x −0,05…1,05, y −0,8…0,3, B10) schließt in unserem System das Band
aus, um 180° gedreht enthält es Band und Ablagepose. **Das ist eine Vermutung,
keine Messung.** Eine Erklärung läge nahe: Die Vorgängergruppe las ihre Posen über
RTDE, also im UR-Rahmen `base`; AICA arbeitet in `base_link`, und beide sind im
UR-URDF um 180° um z gedreht (`vorgaengerprojekt-abgleich.md`, Falle 5b).

**Folge.** Bisher lagen die Zonen-Platzhalter im `priority_handler` und die
Geometrie von `fake_objects` im System der Basiskamera. Am virtuellen Roboter wäre
der Follower damit auf die andere Seite der Basis gefahren, und Wege, Reichweite
und Singularitäten dort hätten nichts über den Aufbau gesagt. Die Ablagepose (B9)
stammt dagegen aus dem Robotersystem.

**Entscheidung:** Testdaten und Platzhalter liegen im **Robotersystem**:
`fake_objects` mit Bandmitte x = −0,816 und Klötzen von y = +0,60 nach −0,80, die
Zonen-Platzhalter bei x −0,95…−0,68, y −0,45…+0,05. Die Mitte ist gemessen, die
Längsausdehnung geschätzt. Neu als offener Punkt **B23**: Die Basiskamera darf erst
an den Follower, wenn ihre Positionen die Antastpunkte treffen.

### F2 — Die Ablagepose kommt erst mit Stufe 4d

Die Abnahme von 4a verlangte die Fahrt zur Ablagepose, ohne Greifzyklus gibt es
aber keinen regulären Weg dorthin. **Entschieden:** Sie wird erst in 4d angefahren.
4a prüft Start, Abbruchpfad und Beobachtungspose. Verworfen wurden ein eigener
Inbetriebnahme-Schalter und `has_object` in `WARTEN` als Auslöser (flackert beim
Öffnen, Nachtrag 6 / Z12).

### F3 — Arbeitsraum und Beobachtungspose haben keine Defaults

> ⚠️ **Überholt durch Nachtrag 13 / L15 (24.09.2026):** Seit der Festlegung von B10
> sind beide als Standardwert eingetragen. Pflichtparameter bleiben sie: leer →
> `configure` scheitert.

`ws_x_min` … `ws_z_max` (B10) und `observe_x/y/z/yaw_deg` (B8) sind
**Pflichtparameter** (`default_value: null`). Fehlt einer, schlägt `on_configure`
mit einer Meldung fehl, die alle fehlenden nennt. Grund für den Arbeitsraum ist die
Regel in `Safety/README.md`: Werte erst mit dokumentierter Festlegung übernehmen.
Die Beobachtungspose ist offen (B8). Für den virtuellen Roboter stehen Vorschläge in
der Einrichtung (§9); sie gelten nur dort.

Dazu wird jede Parameteränderung zur Laufzeit als ganzer Satz geprüft: Beobachtungs-
pose im Arbeitsraum, Freihöhe im Arbeitsraum, Grenzen nicht leer. Ist der Satz
unbrauchbar, gibt der Follower keine neue Zielpose aus und warnt gedrosselt.

### F4 — Der Abbruchpfad in 4a

- **Senkrecht hoch aus der Einstiegspose**, Orientierung beibehalten, auf
  `max(z, transfer_height_m)` — die Freihöhe aus D12, seit J1 0,49 m. Danach `WARTEN`,
  dessen Ziel die Beobachtungspose ist. Liegt der Flansch schon höher, geht es sofort
  weiter; nach unten fährt der Abbruchpfad nie.
- **Das Warten auf das Öffnen des Greifers kommt mit 4d.** Nötig ist es nur, wenn der
  Follower selbst geschlossen hat. Der Greifer meldet `motion_done` nur nach einem
  echten Kommando — ein Öffnen-Befehl an einen offenen Greifer änderte nichts, ein
  Warten darauf hinge bis zum Timeout.
- **„Angekommen" prüft der Follower selbst**, gegen `pose_tolerance_m` (0,01 m, wie
  `linear_precision` des Attractors). Das Predicate `is_in_range` des Attractors
  steht einer Komponente nicht als Signal zur Verfügung.
- **Ohne frischen `robot_state` keine neue Zielpose.** Der Follower stempelt jede
  Nachricht beim Eintreffen; ist die letzte älter als `robot_state_max_age_s`
  (0,2 s), gilt die Flanschpose als unbekannt. Der Attractor hält dann die letzte
  Zielpose — das ist nach Thema 7 der sichere Fall. Beim ersten `robot_state` wird
  sein Bezugsrahmen ins Log geschrieben, damit sich am Aufbau prüfen lässt, ob er
  `world` ist.

### F5 — Zustandscodes in S8

S8 nannte „Zustandscode (Thema 6)", ohne Zahlen. Festgelegt in der Reihenfolge des
Automaten: 0 `WARTEN` … 8 `ABBRUCH` (`datenvertraege.md` S8,
`contracts.STATE_*`). In Zuständen ohne Block sind die Regelabweichungen und
`target_id` 0.

### F6 — Das Sicherheitsgate verglich Sprünge gegen die gedeckelte Pose

**Gefunden durch einen Test.** Das Gate merkte sich die *gedeckelte* Pose und prüfte
den nächsten Takt dagegen. Stand der Arm beim Start außerhalb des Arbeitsraums,
galt schon der zweite Takt als Sprung: Abbruch, neu ansetzen, wieder Sprung. Der
Sprungtest vergleicht jetzt **Rohziel mit Rohziel**; das Deckeln kommt danach.

**Absichtlich so:** Wird die Beobachtungspose zur Laufzeit um mehr als
`max_target_jump_m` verstellt, nimmt der Follower den Abbruchpfad — erst hoch,
dann zur neuen Pose. Das ist der sichere Weg für eine große Verschiebung.

---

## Nachtrag 9 — Beim Bau des `object_follower`, Stufe 4b (21.09.2026)

Keine dieser Festlegungen brauchte eine Rückfrage; sie folgen aus bestehenden
Entscheidungen oder aus einer Lücke der Spec. Offene Werte haben dokumentierte
Startwerte und werden am Aufbau erhoben.

### G1 — Gefolgt wird auf Beobachtungshöhe; der Werkzeugversatz wirkt erst beim Absenken

In `ANFAHREN` und `FOLGEN` fährt der Flansch auf `observe_z` — dieselbe Höhe, auf
die `ABSENKEN` bei wachsender Abweichung zurücksteigt (Thema 6). Weil der Greifpunkt
auf der Flanschachse liegt (M6), wirkt `flange_to_grip_point_m` nur in z, also erst
mit `ABSENKEN` (4d). In 4b ist die Zielpose in x und y die vorhergesagte
Klotzposition plus Vorhalt.

### G2 — Jeder Versuch endet mit `picked_id`, neuer `outcome = 4`

Die Spec verlangt nach jedem Abbruch ein `picked_id`, damit der `priority_handler`
den Klotz als erledigt behandelt. Für einen Abbruch vor dem Griff aus anderem Grund
als der Greifebene passte kein Code: „verloren" (2) stimmt für einen zurückgezogenen
oder gewechselten Zielsatz nicht, für eine Zeitüberschreitung auch nicht. Neu in S7:
**`outcome = 4`, abgebrochen, bevor gegriffen wurde**; der Grund steht im Log.
Beide Verbraucher behandeln jeden Code außer 0 gleich, die Ergänzung ist folgenlos
für sie. In 4b endet jeder Versuch so, denn gegriffen wird erst in 4d.

**Dazu eine Sperre:** Nach einem Abbruch zeigt S4 noch einige Takte dasselbe Ziel,
bis der `priority_handler` die `picked_id` gelesen hat. Der Follower merkt sich die
IDs abgeschlossener Versuche und nimmt sie nicht wieder an — sonst finge er
denselben Klotz sofort ein zweites Mal an.

### G3 — `ANFAHREN` endet am Klotz, nicht am Roboter

`ANFAHREN` → `FOLGEN`, sobald der **vorhergesagte Klotz** `zone_upstream` erreicht
(„Block in der Zone", Thema 6). Weil ein Ziel stromaufwärts gewählt werden darf
(Z11), kann das dauern: Der Roboter wartet am Zonenrand. Eine feste Zeitgrenze
bräche genau dieses Warten ab. Die Grenze ist deshalb **relativ**: erwartete
Ankunft des Klotzes an der Zone plus `timeout_approach_s` (2 s).

### G4 — `FOLGEN` hat in 4b keinen regulären Ausgang

Den Übergang nach `ABSENKEN` und das Tor an der Greifebene bringt erst 4d. In 4b
endet `FOLGEN` nur durch einen Abbruch — meist, weil der `priority_handler` das Ziel
zurückzieht, wenn der Klotz aus dem Bild fährt. `timeout_track_s` steht auf 3 s
(Thema 6: 2–3 s, gemeint als Zeit bis zum Absenken). **Zum Einmessen des Vorhalts
(B4) in 4b auf 10 s oder mehr setzen**, damit der Roboter der ganzen Zone folgt.

### G5 — In `ANFAHREN` und `FOLGEN` ist ein gedeckeltes Ziel ein Abbruch

Thema 7, Prüfung 5: „deckeln **und** Abbruch melden". In den verfolgenden Zuständen
heißt das: gedeckelte Pose ausgeben und den Versuch abbrechen. Das ist die Grenze
aus P4, bis zu der der Roboter einem Klotz über die Zone hinaus folgen darf. In
`WARTEN` und `ABBRUCH` wird nur gedeckelt und einmal gemeldet (4a).

### G6 — Der Gierwinkel bleibt stetig

Von den zwei gleichwertigen Stellungen (Greifer 180°-symmetrisch) wird die gewählt,
die dem **zuletzt kommandierten** Winkel am nächsten liegt; zu Beginn eines
Versuchs dem gemessenen. Das verhindert die halbe Umdrehung des Handgelenks, und es
hält das Kommando auch dann stetig, wenn der Klotzwinkel an der Grenze von [0, π)
umspringt. Das Einfrieren beim Übergang nach `ABSENKEN` kommt mit 4d.

### G7 — Die Selbstüberwachung des Vorhalts ist das Einmesswerkzeug für B4

Im eingeschwungenen `FOLGEN` steht der Flansch mit richtigem Vorhalt genau über dem
vorhergesagten Klotz; ohne Vorhalt liegt er `v/K` dahinter (bei 0,1 m/s und K = 5:
20 mm — so auch in der Simulation). `err_laengs` in S8 ist diese Abweichung,
**positiv = Flansch voraus**. Nach 2 s in `FOLGEN` prüft der Follower ihr
gleitendes Mittel (τ = 0,5 s) und warnt einmal je Versuch, wenn es 10 mm übersteigt.
Einmessen heißt also: `lead_time_s` ändern, bis `err_laengs` im Mittel null ist.
`err_quer` ist positiv links der Laufrichtung.

### G8 — Der Vorhersagehorizont wird in beide Richtungen begrenzt

`t_jetzt − t_ziel` wird auf `[0, max_extrapolation_s]` begrenzt. Negativ hieße: der
Zielsatz stammt aus der Zukunft — die Uhren von Kamera und Rechner laufen dann nicht
in derselben Domäne (B13). Beide Fälle werden einmal je Versuch ins Log geschrieben.

### G9 — Der Montagewinkel der Backen ist nicht gemessen

`gripper_yaw_offset_deg` steht auf 0. Welcher Winkel die Backen quer zum Klotz
schließen lässt, hängt an der Montage und ist nirgends notiert — neu als **D23**.
In 4b wirkt er nur auf die Handgelenkstellung, gegriffen wird erst in 4d.

---

## Nachtrag 10 — Beim Bau des `object_follower`, Stufen 4c und 4d (21.09.2026)

Keine Rückfrage nötig: Die Punkte folgen aus Thema 6/7, den Nachträgen 2, 3 und 6
oder aus Lücken der Spec. Offene Werte haben dokumentierte Startwerte und werden
am Aufbau erhoben.

### J1 — Die Freihöhe vergaß den gehaltenen Klotz

D12 rechnete die Freihöhe als Band + höchster Klotz + Luft **unter den Backen**:
0,445 m im Flanschmaß. Ein gehaltener Klotz hängt aber unter die Backen — bei
100 mm, auf halber Höhe gegriffen, 40 mm unter die Backenspitze. Über einem
stehenden 100-mm-Klotz blieben bei 0,445 m **6 mm**. **Neu: 0,49 m** = Band 53,6
+ stehender Klotz 100 + untere Hälfte des gehaltenen 50 + Griffpunkt 235 + Luft
50 mm. Gilt für Transfer und Abbruchpfad.

### J2 — Sinkgeschwindigkeit und Greifebene hängen zusammen

Die Spec nannte 0,05 m/s zum Absenken, der `priority_handler` rechnet die
Greifebene mit `t_descend_s` = 1,0 s. Von der vorgeschlagenen Beobachtungshöhe
0,60 m auf die Greifhöhe 0,34 m (100-mm-Klotz) sind das 0,26 m — bei 0,05 m/s
**5,2 s**; jeder Griff hätte viel zu spät begonnen. **Neue Startwerte:**
`descend_speed_mps` = **0,15** (1,7 s), `t_descend_s` = **2,0**. Die Kopplung gilt
allgemein: `t_descend_s ≈ (observe_z − Greifhöhe) / descend_speed_mps` plus
Einschwingen. Wer B8 (Beobachtungshöhe) festlegt, zieht `t_descend_s` nach.

### J3 — Die Roboterkamera als Korrektur (4c)

- **Verglichen wird zur Bildzeit:** Messung der Roboterkamera (über Ringpuffer und
  Hand-Auge nach `world`) gegen die Vorhersage der Basiskamera für **dieselbe**
  Zeit. Die Differenz ist die Korrektur; gemittelt über
  `correction_filter_window` angenommene Messungen.
- **Höhenkorrektur zuerst** (N1), an der rohen Messung.
- **Hand-Auge:** Richtung Flansch → Kamera, Konvention **R = Rz·Ry·Rx** — damit
  reproduzieren die rpy aus `Calibration_results_final.yaml` deren Quaternion auf
  fünf Stellen. Ein Test prüft gegen einen von Hand gerechneten Punkt.
- **Ausgeblendet** (`w` → 0 über `w_ramp_s`) bei `valid = 0`, Messung älter als
  `robot_cam_max_age_s`, Korrektur über `max_correction_m` (R4) — und wenn die
  Bildzeit **nicht im Ringpuffer** liegt. Dann laufen die Uhren von Roboterkamera
  und Rechner nicht in derselben Domäne; das steht einmal im Log (neu: **B24**).
- `w_wirksam` in S8 ist der **Anteil der eingestellten Gewichte**, der gerade wirkt
  (die Rampe, 0…1) — ein Wert für beide Achsen.
- Die Korrektur wird je Versuch neu gelernt. Beim Übergang nach `ABSENKEN`
  eingefroren (F2), bei einem Rücksprung nach `FOLGEN` wieder freigegeben (F3).

### J4 — Greif-Freigabe in Bandkoordinaten

Die vier Toleranzen wirken auf dieselben Größen wie `err_laengs`/`err_quer` in S8,
also **entlang und quer zum Band** — nicht in Backenrichtung wie in B18. Startwerte
**5 / 5 / 10 mm und 0,05 rad**, gehalten über 10 Takte. Die Gierabweichung zählt
modulo 180° (Greifer symmetrisch). B18 liefert die geometrischen Obergrenzen, D3
die endgültigen Werte.

### J5 — `ABSENKEN` und `GREIFEN`

- `ABSENKEN`: Zielhöhe sinkt von `observe_z` mit `descend_speed_mps` bis zur
  Greifhöhe; seitlich wird mit Vorhalt weiter gefolgt. Wächst die Abweichung über
  das **Doppelte** der Toleranz, zurück nach `FOLGEN` auf Beobachtungshöhe (F3).
  Zeitgrenze: Absenkdauer + `timeout_grasp_s`, danach `outcome = 4`.
- `GREIFEN`: mitfahren auf Greifhöhe, Greifer zu. `has_object` → `HEBEN` — und
  zwar **vor** den Abbruchgründen geprüft: Meldet der Greifer den Klotz im selben
  Takt, in dem die ID verschwindet (B22), gilt der Griff.
- **Fehlgriff** (`outcome = 1`): Die Bewegung hat begonnen (`motion_done` einmal
  0) und ist fertig, ohne `has_object` — oder `timeout_grasp_s` läuft ab.

### J6 — `HEBEN`, `ABLEGEN`, `LOESEN`

- `HEBEN`: erst mit der **letzten** Bandgeschwindigkeit weiter, auf Greifhöhe +
  `lift_clearance_m`; dann senkrecht auf die Freihöhe; dann `ABLEGEN`.
- **`has_object` fällt weg** → erst nach 0,1 s ohne Klotz gilt er als verloren
  (`outcome = 2`); kürzeres Flackern wird überbrückt.
- `ABLEGEN`: „angekommen" mit `pose_tolerance_m`. **Zeitüberschreitung:** Abbruch
  mit Klotz — hoch, erneut zur Ablage. Scheitert auch das, **hält der Follower den
  Klotz** und meldet „Eingriff nötig", statt ihn irgendwo fallen zu lassen.
  `timeout_place_s` = 8 s: Die Fahrt vom Band zur Kiste ist rund 0,8 m, bei
  0,25 m/s über 3 s plus Einschwingen.
- `LOESEN`: fertig, wenn die Öffnungsbewegung gemeldet ist — oder nach
  `timeout_release_s` mit einer Warnung. **Abweichung von der Spec-Tabelle**
  (dort Zeitüberschreitung → `ABBRUCH`): Ein Abbruch mit flackerndem `has_object`
  würde den Ablagepfad von vorn beginnen. Der Öffnen-Befehl ist über der Kiste
  gegeben; `outcome = 0`.

### J7 — Abbruch ohne Klotz nach eigenem Schließen

Das in 4a zurückgestellte Warten: Hat der Follower den Greifer geschlossen
(Fehlgriff, Abbruch in `GREIFEN`), hält er zuerst die Pose, öffnet und wartet auf
die Öffnungsbewegung (oder `timeout_release_s`), **dann** steigt er. Sonst zöge er
halb geschlossene Backen am Klotz nach oben.

### J8 — Ablagepose und Roboterkamera als Pflicht

- Ablagepose als Default aus **B9** (x −0,316, y +0,476, z 0,420 m, Gier 94,2°),
  Gegenprobe bei laufendem Programm offen. `configure` prüft, dass sie im
  Arbeitsraum liegt.
- `require_robot_cam_for_grasp` (D9/D10): wenn gesetzt, beginnt `ABSENKEN` nur
  bei voll eingeblendeter, frischer Roboterkamera. Vorerst **aus** (Z10).

---

## Nachtrag 11 — Beim Bau des `interface_streamer` (21.09.2026)

Drei kleine Festlegungen, keine Rückfrage nötig.

### V1 — Der Text im Bild ist ASCII

OpenCVs eingebaute Hershey-Schriften kennen keine Umlaute, kein Gradzeichen und
keine griechischen Buchstaben — „längs" erschiene als „l??ngs". Im Bild steht
deshalb „laengs", „Grad", „dv", „Kloetze". Ein Test prüft, dass jede Zeile ASCII
ist. Eine andere Schrift hieße eine zusätzliche Abhängigkeit (FreeType) für eine
Komponente, die das System nicht braucht.

### V2 — Ob ein Debug-Bild noch kommt, zeigt die Ankunftszeit

Die Debug-Bilder von `base_cam` und `robot_cam` tragen keinen Zeitstempel im
Header. Der Streamer merkt sich deshalb, wann ein Bild **ankam**; nach 2 s ohne
neues wird der Bereich grau mit Hinweis. Die Debug-Ausgabe muss dafür in den
Kamerakomponenten eingeschaltet sein (`debug_enable`).

### V3 — Veraltete Daten werden markiert, nicht versteckt

`follower_status` und `world_state`, deren Zeitstempel mehr als 1 s hinter der
Uhr des Rechners liegt, bekommen den Zusatz „(veraltet)". Beim `world_state` ist
das auch eine Diagnose: Er trägt die Bildzeit der Basiskamera — zeigt er dauerhaft
„veraltet", obwohl Bilder kommen, laufen Kamera- und Rechneruhr auseinander (B13).

---

## Nachtrag 12 — Inbetriebnahme am Aufbau (22.09.2026)

Messungen nach Fahrplan (`uebersicht/fahrplan-aufbau.md`), Werkzeug
`test/tools/signal_reader.py`, ausschließlich lesend.

### K1 — Block 0: Paket, Kameras, Zeitdomäne

**Das neue Paket ist wirksam.** Nach Rebuild und Neuerzeugen des Systemabbilds
liegen alle Komponentenbeschreibungen unter `/ws/install/roboter_tetris/`,
`contracts` wird aus dem Image geladen, `base_cam` kennt `camera_node`. Geöffnet
war die unveränderte Anwendung vom 15.09.2026.

**Kamerazuordnung in dieser Anwendung** (über `serial_no`):

| Knoten | Kamera | Profile Farbe / Tiefe |
|---|---|---|
| `/realsense_camera` | **Basiskamera** L515, `f1370107` | 1280×720×30 / 640×480×30 |
| `/realsense_camera_2` | **Roboterkamera** D435i, `241122074842` | 848×480×30 / 848×480×30 |

`camera_node` bei `base_kamera` steht auf `/realsense_camera` — passt.

**Zeitdomäne (B12/B13/B24), je 120 s:**

| Stream | Rate | Versatz ROS − Stempel | Drift |
|---|---|---|---|
| Basis Farbe | 29,2 Hz | +49 ms | −0,033 ms/s |
| Basis Tiefe (aligned) | 29,2 Hz | +47 ms | −0,032 ms/s |
| Roboter Farbe | 28,6 Hz | +44 ms | −0,002 ms/s |
| Roboter Tiefe (aligned, 30 s) | 25,1 Hz | +43 ms | +0,027 ms/s |

Beide Kameras stempeln in der Rechneruhr; kein Stempel eingefroren. **B24 ist
damit erledigt**, B13 am neuen Stand bestätigt. Der konstante Versatz von rund
45 ms ist die Laufzeit Belichtung → Ankunft, keine Uhrenabweichung.

**Zwei Randbefunde, ohne Handlungsbedarf:**

- Die aligned Tiefe der Roboterkamera kam mit **25 Hz** statt ~29 Hz (A6 hatte am
  14.09. 29,7 Hz gemessen). Das Alignment läuft auf dem Rechner; wahrscheinlich
  Last. Beobachten, sobald alle Komponenten laufen — `robot_cam` braucht Farbe
  und Tiefe paarweise.
- Die **Belichtungsautomatik der Roboterkamera ist wieder an** (`true`,
  `exposure` 166). Wie in Einrichtung §1 beschrieben, geht die Laufzeiteinstellung
  mit jedem Start verloren. Für Block 6 vorher abschalten.

**Werkzeug:** Die RealSense-Treiber erscheinen in AICA **nicht** in der
Knotenliste, ihre Parameterdienste sind aber erreichbar. Und ein
`GetParameters`-Aufruf, der auch nur einen unbekannten Namen enthält, kommt
**vollständig leer** zurück. `signal_reader.py` leitet die Knoten deshalb aus den
Topics ab und fragt jeden Parameter einzeln.

### K2 — Alle eigenen Komponenten laufen mit 10 Hz, nicht mit 100 Hz

**Befund.** Jede eigene Komponente meldet `rate = 10.0` (am Aufbau abgefragt:
`base_kamera`, `roboter_kamera`, `roboter_kamera_2_kanten`, `robotiq_gripper`).
Ursache im Framework (`modulo_components/component_interface.py`, Z. 68–78):
`rate` ist ein Parameter der Basisklasse mit **Default 10 Hz**. Er wird **einmal
im Konstruktor** gelesen und legt den Schritt-Timer fest; eine Änderung zur
Laufzeit wirkt nicht.

**Folge.** Auch `vectoring`, `priority_handler` und `object_follower` laufen mit
10 Hz, sofern `rate` in der Anwendung nicht gesetzt wird — Einrichtung §5 und die
Specs setzen 100 Hz an. Beim Follower hieße das nur alle 100 ms eine neue
Zielpose.

**`rate` ist in der Oberfläche vorhanden — geerbt.** Alle unsere Beschreibungen
erben von `modulo_components::LifecycleComponent`, und deren Beschreibung
(`/ws/install/modulo_components/component_descriptions/modulo_lifecycle_component.json`)
deklariert `rate` mit Default 10,0. Der Parameter muss also **in der AICA-
Oberfläche am Block gesetzt** werden, nicht im Paket.

⚠️ **Nicht in die eigenen Beschreibungen eintragen.** Am 22.09.2026 versucht:
Ein zusätzlicher Eintrag `rate` in `component_descriptions/*.json` erzeugt ein
Duplikat zum geerbten — die Oberfläche fügte daraufhin **bei jedem Klick auf den
Block ein weiteres Rate-Feld** hinzu (acht nach kurzer Zeit). Zurückgenommen, die
Beschreibungen sind wieder auf dem Commit-Stand. Merksatz: **Parameter der
Basisklasse nie in der eigenen Beschreibung wiederholen.**

**Zu setzen in der Anwendung** (Rate wirkt erst nach Neuladen des Blocks):
`vectoring`, `priority_handler`, `object_follower` **100 Hz**; `base_cam`,
`robot_cam*` nach Rechenleistung (10 Hz Ist-Stand, über 30 Hz sinnlos);
`data_tracker` 10 Hz; `interface_streamer` 5 Hz.

⚠️ **Kopplung:** `settle_half_window` in `vectoring` zählt **Messungen**, nicht
Zeit; der Default 15 ist auf 30 Hz ausgelegt (0,5 s). Bei `base_cam` mit 10 Hz
dauert ein Halbfenster 1,5 s, das Einschwingen also rund dreimal so lange. Läuft
`base_cam` mit 10 Hz, gehört `settle_half_window` auf 5.

### K3 — B6 Stufe 1, zweiter Anlauf: Erkennung ja, Banddistanz nein

**Aufbau.** Stehender roter Referenzklotz (100 mm) mittig unter der
Roboterkamera, Flansch bei z = 411 mm, Werkzeug lotrecht. Gemessen: Kamera →
Band **284 mm**, Kamera → Klotzoberseite **184 mm**, der Klotz füllt rund
160 × 165 px. Beide Varianten mit dem neuen Stand aus 2.2 (Defaults: Auswahl nach
Fläche, keine ROI, `max_contour_area` 50 000). Ein Bildpaar aufgenommen und
**offline mit dem unveränderten Code aus `vision/`** nachgerechnet
(Bilder: `architektur/bilder/2026-09-22-robotcam-*.png`).

| Variante | Ergebnis am Aufbau | Ursache (offline belegt) |
|---|---|---|
| `robot_cam` (Farbe) | `valid = 0` | Das Band ist **ungleichmäßig gefärbt**: links ausgewaschen (H 80, **S 22**, V 132), rechts türkis (H 90, S 95). 92 % des Bildes gelten als „nicht Band“ und verschmelzen zu einer Kontur über das ganze Bild. `max_contour_area` verwirft sie **richtig** — ohne die Sicherung wäre x 3,0 / y −6,2 mm herausgekommen: zufällig nahe am Klotz, weil die Bildmitte, nicht weil der Klotz. |
| `robot_cam_2` (Kanten) | `valid = 0` | Der Klotz wird gefunden, aber mit dem Tiefenschatten zu einer Kontur bis zum unteren Bildrand verbunden. Die Banddistanz wird **direkt unter dem Blob** abgetastet — dort liegt Bildrand bzw. ungültige Tiefe → keine Banddistanz → `valid = 0`. |
| `robot_cam_2` ohne Tiefenkanten | Klotz sauber bei Pixel (430, 271) | Die Abtastung unter dem Blob trifft jetzt die **Klotzoberseite**: `z_band` = 184 statt 284 mm — ein Maßstabsfehler von einem Drittel, der als gültige Messung durchginge. |

**Die Belichtungsautomatik war hier nicht die Ursache.** Nach dem Abschalten
(`exposure` 166) blieb das Band links bei S 22 — der Befund aus §9.3 gilt für
Überbelichtung, hier ist die Bandoberfläche selbst an dieser Stelle hell und
entsättigt.

**Folgerungen:**

1. **Die Banddistanz gehört nicht „direkt unter den Blob“.** Aus kurzer Distanz
   ist diese Stelle Schatten, Bildrand oder Klotz. Der Median der gültigen Tiefe
   über das Bild (`belt_reference_mm`, im Code schon vorhanden und stabil bei
   284 mm) wäre robust. Das ist eine Änderung am gemeinsamen Kern in
   `vision/robot_detection.py` — **Vorschlag, nicht umgesetzt**; die Regel aus dem
   Projektkontext §4 („Rückprojektion bleibt“) muss dafür bewusst aufgehoben werden.
2. **Die Farbvariante hat an diesem Band ein grundsätzliches Problem**, keine
   Einstellungsfrage: Eine Maske „alles, was nicht Band ist“ scheitert, sobald das
   Band stellenweise entsättigt ist. Der Klotz selbst ist mit S 227 eindeutig —
   eine positive Klotzmaske wäre für Rot und Blau trivial, für Weiß aber nicht.
   Das berührt den Detektionskern und damit den A/B-Vergleich (B6).
3. **Die Kantenvariante ist der aussichtsreichere Kandidat.** Mit Folgerung 1
   hätte sie den Klotz hier richtig gemessen.
4. **B8:** 284 mm Kamera → Band ist nah; der Tiefenschatten des stehenden Klotzes
   ist dort groß. Eine Messung aus größerer Höhe steht aus.

### K4 — Block 1: Datenpfad läuft; der Rechner ist der Engpass

**Nach dem zweiten Rebuild** (Beschreibungen wieder auf Commit-Stand, Blöcke neu
angelegt) kommt die am Block gesetzte, geerbte `rate` an: `vectoring` und
`priority_handler` 100 Hz, `data_tracker` 10 Hz, `interface_streamer` 5 Hz,
`base_kamera` 10 Hz. **K2 ist damit gelöst** — über die Oberfläche, ohne
Paketänderung.

**Datenpfad bestanden.** Ein ruhender Klotz erscheint in `objects`, `tracks` und
`world_state` mit derselben ID, ohne Vertragsfehler; in `tracks` Status 0 (bei
stehendem Band ist die Geschwindigkeit sofort konstant — null), `n_pool = 1`,
v = 0. Der `priority_handler` wählt erwartungsgemäß nichts (Zone im Robotersystem,
Klotz im Kamerasystem bei x = +801, y = −886 mm; B23), `not_pickable` ist leer.
Streuung in `objects` σ ≈ 0,4–0,5 mm, nach `vectoring` 0,04–0,07 mm.

**Der eigentliche Befund: `base_cam` liefert nur ~3,3 neue Messungen/s.** Die
Komponente läuft mit 10 Hz, bekommt aber nur in jedem dritten Schritt ein
fertig verarbeitetes neues Bild. Der Rechner ist voll ausgelastet
(Load-Average ~12; AICA-Event-Engine ~180 % CPU, Python-Komponenten ~140 %, RViz
40 %); selbst die Kamera-Topics kommen im Container nur noch mit ~7 Hz an.
Mitverursacher: **zwei aktive Roboterkamera-Komponenten** mit Bildverarbeitung
und der `interface_streamer`, die in Block 1–3 nicht gebraucht werden.

**Folgen:**
- `settle_half_window` = 15 Messungen hieße bei 3,3 Hz **4,5 s je Halbfenster**,
  ein Klotz würde erst nach ~9 s final. Bei 3,3 Hz gehört der Wert auf **5**
  (1,5 s).
- Bei 0,1 m/s Band bewegt sich ein Klotz zwischen zwei Messungen um ~30 mm.
  Für die Schätzung (Ausgleichsgerade über viele Messungen) unkritisch, für die
  Vorhersage im Follower relevant (`max_extrapolation_s`).
- **Für Messungen mit der Basiskamera alles abschalten, was Bilder verarbeitet
  und nicht gebraucht wird** — beide Roboterkamera-Blöcke, `interface_streamer`,
  RViz.

### K5 — B23 aufgeklärt: 180° aus der TF, aber die Kalibrierung ist die des Vorgängers

**Die 180° sind belegt, nicht mehr vermutet.** Im laufenden System liefert
`tf2_echo ur_base_link ur_base` eine Drehung um **exakt 180° um z, Verschiebung
null**; `world` ist identisch mit `ur_base_link`. AICA regelt in `world`. Die
Kamerakalibrierung rechnet dagegen in den UR-Rahmen `base` (Band bei x ≈ +0,65 m
laut `T_robot_conveyor`) — daher das umgekehrte Vorzeichen aus F1.

**Umrechnung** der vorhandenen Werte nach `world` (reine Drehung um z um 180°,
`R' = Rz(180°)·R` ⇒ nur Yaw + 180°):

| `base_kamera` | in `base` | in `world` |
|---|---|---|
| `cal_x` | 0,6118 | **−0,6118** |
| `cal_y` | −0,7820 | **0,7820** |
| `cal_yaw` | −12,94° | **167,06°** |
| `track_min_y_mm` / `track_max_y_mm` | −1080 / 375 | **−375 / 1080** |
| `track_velocity_region_y_min` / `_max` | −1000 / −500 | **500 / 1000** |

`cal_z`, `cal_roll`, `cal_pitch` bleiben. Der Tracker nimmt keine Laufrichtung
an (nur Intervalle, vorzeichenbehaftetes `vy`) — die gespiegelten Grenzen
funktionieren ohne Codeänderung.

**Aber: Die Kalibrierwerte sind nicht aktuell.** `Calibration/calibration.json`
trägt den Status `validated` (22.08.2026), die Zahlen sind jedoch **identisch mit
denen der Vorgängergruppe** (`UR10_Pick_ws/cameras/config_cam_static.yml`,
Dateistand 29.03.2026). Der eigene Validierungsblock nennt
**`position_rmse_mm` = 49,04** bei 5 Punkten und `rotation_rmse_deg` = 0,0 — der
zweite Wert wirkt wie ein Platzhalter. 49 mm liegen weit über der Toleranz
quer von ~17 mm (Z10). Dazu passt die Beobachtung am Aufbau, dass die Kamera
„halbwegs gerade“ über dem Band hängt, während die Kalibrierung 13° Gier angibt;
die Vorgängergruppe hatte selbst eine ältere Fassung mit Gier 0,0° und Pitch
−0,5° (dort auskommentiert) und korrigierte zusätzlich `x_offset_mm = −35`.
Die Basiskamera steht auf einem beweglichen Gestell — ob sie seit März an
derselben Stelle hängt, ist unbekannt.

**Folge:** Die Doku-Aussage „Legacy-Werte, nicht validiert“ (Projektkontext §9,
C3) war in der Sache richtig, auch wenn die Datei inzwischen `validated` sagt.
Block 2 schrumpft deshalb nicht auf eine Formalie: Die Antastpunkte
quantifizieren den Fehler der **gedrehten Altkalibrierung** und zeigen, ob er
eine Verdrehung (Gier) oder eine Verschiebung ist. Das Ergebnis geht als
unabhängige Prüfung an das Kalibrierprojekt (C3); neu kalibriert wird hier nicht.

### K6 — Block 2: Die 13° Gier der Kalibrierung sind der Fehler

Zwei Antastpunkte, geschlossene Backen mittig auf der Oberseite eines roten
Klotzes (Höhe laut Roboter 25 mm), Werkzeug lotrecht. Kamera mit der nach
`world` gedrehten Altkalibrierung (K5). P2 wurde nach einem AICA-Absturz über
die **reine Leseschnittstelle der UR-Steuerung (Port 30013)** gelesen: TCP der
Steuerung + 215 mm in z, dann 180° nach `world` (P1 mit beiden Wegen: 324,0 mm
über AICA, 323,6 mm über 30013 bei P2 — passt).

| Punkt | Kamera x / y | Roboter x / y | Abweichung | h Kamera / Roboter |
|---|---|---|---|---|
| P1 | −800,7 / +885,7 | −855,4 / +804,8 | **97,6 mm** | 38,5 / 25,5 |
| P2 | −677,0 / +677,2 | −690,3 / +632,1 | **47,0 mm** | 36,6 / 25,0 |

Rohdaten: `architektur/bilder/2026-09-22-b23-punkte.json`.

**Befund.** Die starre Ausgleichsrechnung Kamera → Roboter ergibt eine Drehung
von **+13,01°** — spiegelbildlich zur Gier der Kalibrierung (−12,94°). Der
Abstand P1–P2 stimmt in beiden Systemen auf 3,5 mm überein (242,4 gegen
238,9 mm, Maßstab 0,986), es ist also im Wesentlichen eine **Verdrehung, kein
Maßstabsfehler**. Das Kamerabild bestätigt es unabhängig: Die Bandkanten laufen
dort **exakt senkrecht** (Pixel x ≈ 337 bzw. 1188 oben wie unten,
`bilder/2026-09-22-basiskamera.png`) — die Kamera hängt gerade über dem Band, wie
am Aufbau beobachtet. Die Höhe misst die Kamera um **+12 mm** zu groß.

Mit zwei Paaren ist die Transformation exakt bestimmt; der Restfehler hat keine
Aussage. **Deshalb als Hypothese, zu prüfen an einem dritten Punkt:**

| `base_kamera` | gedrehte Altkalibrierung (K5) | korrigiert (Vorschlag) |
|---|---|---|
| `cal_yaw` | 167,06° | **180,07°** |
| `cal_x` | −0,6118 | **−0,6491** |
| `cal_y` | 0,7820 | **0,7476** |

`cal_z`, Roll und Pitch unverändert — ob die +12 mm Höhenfehler aus z oder aus
dem Pitch stammen, trennen zwei Punkte nicht. **Prüfung:** Werte setzen, Klotz an
eine dritte Stelle, Kamera lesen, antasten. Die Abweichung dort ist eine echte
Vorhersage.

**Sicherheit nach Absturz:** Startet AICA neu, fährt der Attractor auf die
zuletzt gespeicherte Zielpose. Steht der Greifer dann auf einem Klotz, vorher
per Pendant freifahren.

---

## Nachtrag 13 — Audit und Aufbau (23.09.2026)

Ein Audit des Gesamtstands (L1), danach Messungen am Aufbau mit
`test/tools/signal_reader.py`. Umgesetzt wurde nur, was der Nutzer freigegeben
hat; gebaut hat der Nutzer.

### L1 — Audit: drei Größen waren angenommen statt gemessen

Code und Architektur tragen (Logik ohne ROS, Verträge, Sicherheitsgate,
Geschwindigkeitsschätzung). Die Schwachstelle: drei Größen, die das Design
bestimmen, stammten vom Schreibtisch — **Messrate und Latenz der Basiskamera**,
**Lage der Greifzone zum Kamerabild** und die **Bandgeschwindigkeit**. Alle drei
sind heute gemessen (L2, L4, L7). Offen aus dem Audit bleiben:

- **Deckel der Vorhersage.** `max_extrapolation_s` = 0,2 s im Follower liegt unter
  dem gemessenen Horizont (L2). Nachgerechnet mit dem Regelkreismodell aus
  `test_follower_logic.py`: Der Flansch läuft dann bei 0,13 m/s still 10–60 mm
  hinter dem Klotz her, `err_laengs` sieht es nicht (es misst gegen die gedeckelte
  Vorhersage), bei 0,2 m/s löst das Sprung-Gate aus. Ebenso liegen
  `target_timeout_s`, `STALE_TIMEOUT_S` (priority_handler) und `track_expiry_s`
  bei 0,5 s, knapp über dem Messabstand. Tests und `fake_objects.py` bilden
  30 Hz ohne Latenz ab.
- **Strecke ohne Basiskamera.** Außerhalb ihres Bildes führt `base_cam` Tracks
  mit ihrer eigenen, verrauschten EMA-Geschwindigkeit weiter (nur in y) und
  publiziert sie in S1 ununterscheidbar von Messungen; `vectoring` nimmt sie als
  Messungen. Genau dort wird gegriffen (L4). Vorschlag: S1 kennzeichnet
  weitergeführte Einträge, `vectoring` extrapoliert selbst mit der gepoolten
  Geschwindigkeit, Tracks verfallen am Zonenende statt nach 0,5 s.
- **Greifzonen-Platzhalter** y −0,45 … +0,05 reicht über das Bandende (−0,375) hinaus. *(Behoben: L14.)*
- **Startverhalten:** Der Attractor fährt beim Start auf die gespeicherte Zielpose.

### L2 — Datenrate und Latenz der Basiskamera

Gemessen als Alter des S1-Zeitstempels (Bildzeit) beim Empfang, je 40 s:

| Stand | neue Messungen/s | Alter bei Ankunft (Median) | Alter vor der nächsten (Median / max) | längste Lücke |
|---|---|---|---|---|
| Ausgang (Roboterkameras unkonfiguriert in der Anwendung) | 5,1 | 450 ms | 643 / 1052 ms | 0,7 s |
| mit `data_tracker`, Frame-Steuerung | 2,8 | 441 ms | 631 / 8310 ms | **7,9 s** |
| A: `vectoring`/`priority_handler` 20 Hz, `data_tracker` 2 Hz | 7,1 | 409 ms | 528 / 796 ms | 0,43 s |
| B: zusätzlich Warteschlange Tiefe 1 | 7,1 | **139 ms** | **266 / 506 ms** | 0,4 s |

**Zwei Ursachen.** (1) AICA lädt **alle Python-Komponenten in einen Prozess**
(`component_container_mt`, MultiThreadedExecutor). Wegen der GIL teilen sie sich
praktisch einen Kern — der Rechner hatte 27 % frei, aber `base_cam` bekam keine
Rechenzeit. Jede weitere Python-Komponente und jede unnötig hohe `rate` kostet
Messrate. (2) `modulo` legt jeden Eingang mit **Warteschlange Tiefe 10** an; die
Bilder stauten sich, `base_cam` rechnete auf rund 0,4 s alten Bildern. Die Kamera
selbst liefert nach 46–51 ms (gemessen an `*_camera_info`).

**Maßnahmen:** A in der Oberfläche; B im Code (`base_cam`, `robot_cam_2`:
`set_qos(QoSProfile(depth=1))` vor den Bildeingängen, Voreinstellung danach
zurück). Auch unkonfigurierte Blöcke abonnieren ihre Bilder — nicht gebrauchte
Kamerakomponenten gehören **aus** der Anwendung.

### L3 — Hardware-Takt und Abstürze

Der Regelkreis zum Roboter (500 Hz, `event_engine` mit Kameratreibern und
Tiefenausrichtung im selben Prozess) fiel zeitweise auf 72–440 Hz — ausgelöst
schon durch zusätzliche Leseprozesse. Die Container vom 22.09. lebten 17–27 min,
zweimal endeten sie ohne Stopp-Signal (16:25, 16:54); kein OOM. Mit dem reduzierten
Aufbau trat kein Absturz mehr auf. **Hypothese: Überlast.** Das Thema ruht, solange
es nicht wiederkommt. Beobachtet: Der Speicher der `event_engine` wächst mit jedem
Neustart der Anwendung (390 → 630 MB). Außerdem meldet der UR-Treiber, dass die
Werkskalibrierung des Roboters nicht zur Kinematik passt (Flanschposen aus AICA
können um wenige mm von der Steuerung abweichen; bei P1 0,4 mm).

### L4 — Der Greifer im Bild der Basiskamera

Steht der Greifer tief über dem Band, erkennt `base_cam` die offenen Backen als
zwei Klötze. Folgen bei stehendem Band: Bandgeschwindigkeit −140 … +40 mm/s,
laufend neue IDs, **Geister-Tracks**, die mit der falschen Geschwindigkeit bis
y ≈ −0,33 wanderten und zeitweise *final* waren — einer im Greifzonen-Platzhalter.

**Entscheidung (Nutzer):** Greifzone und Wartebereich des Greifers liegen
**außerhalb des Bildes der Basiskamera** (Bild endet bei y ≈ 0,46; Zone etwa
y 0,30 … −0,30). Die Strecke zwischen Bildrand und Griff (0,16–0,76 m) überbrückt
die **Roboterkamera** — der Verzicht auf sie (Z10) war nur vorläufig. Folgen: Die
Zone ist mit rund 0,6 m knapp (Bedarf bei 0,13 m/s und heutigen Zeiten 0,55 m);
die Ablagepose (y +0,476) liegt am Bildrand und ist zu prüfen; L1 „Strecke ohne
Basiskamera“ bleibt als Zubringer nötig.

### L5 — Roboterkamera: Kantenvariante, Banddistanz aus dem Median

**Entscheidung (Nutzer):** Weiter nur mit `robot_cam_2` (Kanten). Die Farbvariante
kommt nicht in die Anwendung (Rechenlast, K3). Damit entfällt der A/B-Vergleich,
und die Regel „Rückprojektion in `vision/robot_detection*` bleibt“ ist aufgehoben.

**Umgesetzt (K3, Folgerung 1):** Die Banddistanz ist der Median der gültigen
Tiefe über das Bild (`belt_reference_mm`), nicht mehr ein Messpunkt unter dem
Blob. Der Median wird einmal je Bild berechnet (vorher zweimal).
`depth_search_radius_px` bleibt als Parameter ohne Wirkung. Am Aufbau noch
nicht erprobt (Block 6).

### L6 — B23 abgeschlossen: Neigung, Parallaxe, neue Extrinsik

**P3** (flacher Klotz, x −0,93 / y 0,87, am Rand der Reichweite) war die erste
echte Vorhersage der K6-Korrektur: **3,3 mm**. **P4** (100-mm-Klotz) lag
**15,6 mm** daneben, **P5** (flacher Klotz an exakt derselben Stelle) 4,7 mm —
die Kamera sah die 75 mm höhere Oberseite 16 mm versetzt.

**Ursache 1 — Neigung.** Eine Ebene, angepasst an ein einzelnes Tiefenbild des
Bandes (457 000 Punkte, Rest-σ 1,3 mm): Die Basiskamera schaut **auf 0,7° senkrecht**
aufs Band. Die Altkalibrierung nahm 9,1° an (Roll 176,26, Pitch −8,27); mit ihr
wäre das Band um 9,3° gekippt und reichte über das Bild von −1 bis 109 mm Höhe.
Roll und Pitch waren so falsch wie der Yaw (K6).

**Ursache 2 — Parallaxe in `base_cam`.** Die Ecken des `minAreaRect` wurden je mit
der Tiefe ihres eigenen Pixels umgerechnet; diese Pixel liegen an der Kante und
lesen das Band (bei allen fünf Punkten 860 mm, auch beim 100-mm-Klotz). Gemessen
wurde der Umriss der Oberseite, aufs Band projiziert. Vorhergesagt für P4:
16,3 mm in y — gemessen 16,2 mm. Die Höhe kam aus dem *Mittelwert* der Tiefe im
Umriss, von den Kantenpixeln Richtung Band gezogen; `HEIGHT_BIAS_MM = 20` glich das
nur für eine Klotzhöhe aus (M5: 100 mm richtig, 25-mm-Klotz +12 mm).

**Umgesetzt in `vision/detection.py`:** Tiefe der Oberseite = **Median** über den
Umriss; alle Ecken mit dieser Tiefe umgerechnet; **Höhe = z der Oberseite in
`world` − `belt_surface_z_mm`** (neu, 53,6); `HEIGHT_BIAS_MM` entfällt; S1-z ist
jetzt die Klotzmitte (vorher Ecken + halbe Höhe).

**Neue Extrinsik** (Neigung aus der Bandebene, Yaw und x/y aus den fünf Punkten,
z so, dass das Band bei 53,6 mm liegt): `cal_x` −0,7787 · `cal_y` 0,7934 ·
`cal_z` 0,9163 · `cal_roll` 179,46 · `cal_pitch` 0,45 · `cal_yaw` 179,76. Rest
über alle fünf Punkte ≤ 3,7 mm. Rohdaten:
`architektur/bilder/2026-09-23-b23-punkte.json` (P1/P2-Kamerawerte darin mit der
K5-Kalibrierung, P3–P5 mit K6 — für eine neue Auswertung zuerst umrechnen).

**Gegenprobe nach dem Build** (Klötze von Hand an die P4/P5-Stelle gestellt):

| | Position Kamera | Roboter | Abweichung | Höhe Kamera / echt |
|---|---|---|---|---|
| flach | −808,8 / 674,8 | −807,5 / 674,1 | 1,5 mm | 14,3 / 24,2 |
| hoch | −812,7 / 676,1 | −807,5 / 674,1 | 5,6 mm | 87,3 / 100,4 |

Die Position erfüllt B23 (≤ 10 mm; im Rest steckt der Aufstellfehler von Hand).
Grundfläche jetzt 49 × 47 mm (echt 50 × 50). **Die Höhe ist um 11,5 ± 1,6 mm zu
niedrig** — ein fester Versatz: Die L515 liest die Klotzoberseiten zu tief
(Eindringen des LiDAR in den Kunststoff), das Band nicht. Vorschlag: Parameter
`top_depth_bias_mm` = 11,5, von der Oberseitentiefe abgezogen — korrigiert Höhe
und den kleinen Maßstabsfehler der Position. **Umgesetzt** (Standardwert 11,5);
die Bestätigung am Aufbau steht nach dem nächsten Build aus.

Die Werte sind eine **Übergangskalibrierung**; `Calibration/calibration.json`
(Vorgängerwerte im Rahmen `base`) bleibt unberührt. Für das Kalibrierprojekt (C3):
180° zwischen `base` und `world`, Kamera senkrecht, Parallaxe in der Detektion.

### L7 — Bandgeschwindigkeit, Bandlage

Stoppuhr (Nutzer, mäßig genau): 1 m in 7,7–8,0 s, ein Klotz 11,3 s über ~1,5 m
Band → **v ≈ 0,13 m/s**. Nur Gegenprobe; maßgeblich ist die Schätzung (Z2).
Das Band reicht in `world` von y ≈ +1,08 (Anfang) bis −0,375 (Ende) — genau die
Trackergrenzen der Vorgängergruppe. Das Bild der Basiskamera deckt davon
y ≈ 0,46 … 1,03 ab; die übrigen ~0,84 m (6,5 s) sieht sie nicht.

### L8 — Standardwerte auf den Stand gebracht

`base_cam`: Extrinsik (L6), Trackergrenzen −375 / 1080, Messregion 500 / 1000,
`camera_node` `/realsense_camera`, `belt_surface_z_mm` 53,6,
`top_depth_bias_mm` 11,5. `vectoring`:
Einschwing-Halbfenster 5 (zählt Messungen; bei 5–7 Messungen/s ≈ 0,7–1 s).
⚠️ **Von Hand gesetzte Blockparameter gehen vor.** Neue Standardwerte übernimmt
nach dem Build auch ein bestehender Block, wo der Parameter auf dem Standardwert
steht (korrigiert 24.09.2026, Nutzer; vorher hieß es „nur neu eingefügte Blöcke“).
`rate` bleibt in der Oberfläche (K2).

### L9 — Block 3: Schätzung je Klotz stimmt, Pool und Bildrand nicht

Nach dem Build mit `top_depth_bias_mm`: Höhe flach 23,8 mm (echt 24,2), hoch
96,2 mm (echt 100,4) — Schritt 0 bestanden. Dann Band an, drei Klötze, 120 s
`tracks`:

| Klotz | im Bild (y, mm) | final nach | geschätzte Geschwindigkeit |
|---|---|---|---|
| flach | 776 → 586 | 1,7 s | **−128,0 mm/s** |
| gekippt, liegend | 971 → 608 | 3,3 s | **−126,6 mm/s** |
| hoch, stehend | 959 → 548 | nie | — |

**Je Klotz trägt Ziel 3:** Stoppuhr 125–133 mm/s, Richtung sauber −y (quer
< 1,1 mm/s).

**Befund 1 — Pool stand auf 0.** Zwei Abschnitte stammten von den **stehenden**
Klötzen aus Schritt 0; nach `S_tt` gewichtet, begruben sie die kurzen bewegten
Abschnitte. **Entscheidung (Nutzer):** Nicht vergessen, sondern nur **bewegte**
Abschnitte poolen — neuer Parameter `pool_min_speed_mps` = 0,05 (Untergrenze der
Vorgängergruppe; Band ≈ 0,13). Stehende Objekte, auch erkannte Greiferbacken,
bleiben damit draußen, die Präzision langer Abschnitte bleibt. Umgesetzt mit Test.

**Befund 2 — Tracks enden am Bildrand.** Das sichtbare Bild endet bei
y ≈ 0,55–0,61, die Messregion erst bei 0,50: Die Klötze verlassen das Bild noch
*in* der Region und werden nach drei Fehlbildern gelöscht. Hinter dem Bild kommt
nichts an — die Greifzone (L4) sieht nie ein Ziel. Das bestätigt L1 („Strecke
ohne Basiskamera“) am echten Band; die Weiterführung gehört in `vectoring`, mit
der gepoolten Geschwindigkeit. Offen.

**Befund 3 — Einschwingen knapp.** Ein Klotz ist nur ~3,5 s im Bild; der
stehende hohe Klotz wurde darin nicht final. Ursache noch nicht untersucht.

### L10 — Hinter dem Bild führt `vectoring` weiter (umgesetzt)

Umsetzung von L1 („Strecke ohne Basiskamera“) nach Befund 2 aus L9:

- **Neuer Status 4 „vorhergesagt“** in S3 (`contracts.TRACK_PREDICTED`). Ein
  finaler Track, der im letzten Bild fehlt, wird nicht vergessen, sondern mit der
  gepoolten Geschwindigkeit fortgeschrieben — dieselbe Projektion, die ein finaler
  Track immer nutzt, nur ohne neue Messungen. Höchstens `predict_max_s` = 8 s
  (Bildrand bis Bandende bei 0,13 m/s ≈ 7 s). Einschwingende Tracks werden wie
  bisher nach `track_expiry_s` vergessen (jetzt 1,0 s statt 0,5).
- **Übergabe:** Liegt eine Messung näher als `handover_distance_m` = 0,05 m an
  einem vorhergesagten Track, endet dessen Vorhersage — der Klotz ist unter neuer
  ID wieder im Bild.
- **`base_cam` führt nicht mehr selbst weiter:** Die Messregion umfasst das ganze
  Band (−375 … 1080). Innerhalb löscht der Tracker nach drei Fehlbildern, außerhalb
  hatte er mit seiner EMA-Geschwindigkeit weitergeführt. S1 enthält damit nur
  Messungen. Reine Parameteränderung.
- **Empfänger:** `priority_handler` wählt aus Status 0 und 4 und meldet bereits
  versuchte IDs nicht mehr als „hinter der Greifebene“ (ein gegriffener Klotz läuft
  vorhergesagt weiter). Ein gegriffener Klotz hält das Ziel jetzt bis zur Meldung in
  `picked_id` — vorher verschwand seine ID schon beim Anheben. `data_tracker`
  zählt den Ablauf ab dem *ersten* Erledigt-Zeitpunkt. Anzeige: „vorhergesagt“.
- **Zeitgrenzen an die gemessene Latenz (L2):** Follower `max_extrapolation_s`
  0,2 → **0,6**, `target_timeout_s` 0,5 → **1,0**; `priority_handler`
  `STALE_TIMEOUT_S` 0,5 → **1,0**.

Tests: 248 lokal grün (neu: Vorhersage und Ablauf, Übergabe, Vergessen
einschwingender Tracks; angepasst: Deckel und Zeitgrenzen des Followers, Laufzeit
des synthetischen Buchhaltungstests). Am Aufbau zu prüfen: Tracks laufen als
Status 4 bis in die Greifzone, die Position stimmt dort (Antasten).

**Block 3 wiederholt (nach dem Build), 150 s, vier Klötze:**

| Klotz | final nach | vorhergesagt ab | Ende der Vorhersage |
|---|---|---|---|
| hoch (96 mm) | 0,9 s (y 873) | y 515 | y −493 |
| flach (24 mm) | 1,0 s (y 695) | y 515 | y −496 |
| hoch | 1,2 s (y 827) | y 508 | y −502 |
| hoch (100 mm) | 1,3 s (y 829) | y 522 | y −487 |

- **Pool −127,9 mm/s**, quer −0,1 mm/s, ab dem ersten Klotz stabil (−127,8 …
  −128,1) — Stoppuhr 125–133 mm/s. **Ziel 3 ist am laufenden Band bestätigt.**
- Alle Klötze werden nach ~1 s final, auch die hohen (vorher nie); jeder läuft ab
  dem Bildrand (y ≈ 0,51) als Status 4 durch die Greifzone bis hinter das Bandende.
  Keine doppelten IDs, keine Übergabe nötig.
- `predict_max_s` = 8 s reicht ~0,12 m über das Bandende (−0,375) hinaus;
  unkritisch, die Greifzone endet vorher.
- Offen: Genauigkeit der Vorhersage in der Greifzone. Nach der Rechnung klein
  (Pool-Streuung < 0,5 mm/s × ≤ 6 s ≈ 3 mm, dazu B23 ≤ 6 mm); direkt messen lässt
  sie sich erst mit der Roboterkamera (Block 6) oder beim ersten Griff.

### L11 — Block 6: Roboterkamera misst, die Hand-Auge-Kalibrierung war falsch

**Messung.** `robot_cam_2` über einem 100-mm-Klotz hinter dem Bild der
Basiskamera (Flansch z 624 mm): `valid = 1`, Streuung < 1 mm, Banddistanz
**499 mm** — die Median-Banddistanz (L5) trägt (am 22.09. noch 184 statt 284 mm).
Belichtungsautomatik der D435i war aus (`exposure` 166).

**Hand-Auge falsch (C1).** Die Umrechnung nach `world` legte das Band auf z = 185
statt 53,6 mm und den Klotz 34 mm neben den Antastpunkt. Die Richtung Flansch →
Kamera im Code stimmt (die umgekehrte lag 200 mm daneben); falsch ist der
**Versatz**: Die Vorgängergruppe setzt die Kamera 6 cm *über* den Flansch, sie sitzt
7 cm darunter — unabhängig bestätigt durch die Banddistanz.

**Neu eingemessen** an einem angetasteten flachen Klotz (50 × 75 × 25 mm) aus fünf
Ansichten, Flanschhöhe 0,47–0,62 m, eine mit um ~20° gekipptem Werkzeug. Die Mitte
der Oberseite wurde **offline** aus Farb- und Tiefenbild segmentiert, unabhängig von
`robot_cam_2` (s. u.). Ausgleich der sechs Werte:

| Flansch → Kamera | x mm | y mm | z mm | Roll | Pitch | Yaw |
|---|---|---|---|---|---|---|
| Vorgängergruppe (C1) | 108,7 | −34,4 | −59,9 | 1,66° | 1,56° | 91,50° |
| **neu** | **78,3** | **−32,6** | **72,0** | **4,26°** | **0,08°** | **90,95°** |

Rest ≤ 4,4 mm je Ansicht. Die fünfte Ansicht, vorher aus den übrigen vier
vorhergesagt: **1,7 mm** (C1: 14,8 mm). x und Roll sind korreliert (vier Ansichten:
x 93,7 / Roll 2,0°) — beide Sätze sagen im gemessenen Höhenbereich auf ~2 mm gleich
voraus. Neue Standardwerte `handeye_*` im Follower; Test mit der echten
Prüfansicht. Rohdaten: `architektur/bilder/2026-09-23-handauge-ansichten.json`.

**Drei Befunde an `robot_cam_2` selbst (offen, B6):**

1. **Glanzstreifen als Klotz.** Auf dem Band liegt ein heller Reflexstreifen ohne
   gültige Tiefe; das Nah-Gate zählt fehlende Tiefe als „nah“ und nahm den Streifen,
   als der Klotz am Bildrand stand (`bilder/2026-09-23-robotcam2-glanzstreifen.png`).
   Auch bei sichtbarem Klotz verschmolz der Streifen mit dessen Umriss. Vorschlag:
   Das Gate verlangt gültige Tiefe über dem Band, nicht nur fehlende.
2. **Seitenfläche.** Schräg gesehen gehört die zugewandte Seitenfläche eines hohen
   Klotzes zum Umriss; die Mitte rutscht zur Kamera (≈ 14 mm beim 100-mm-Klotz,
   `bilder/2026-09-23-robotcam2-seitenflaeche.png`). Beim Folgen steht die Kamera
   nahezu senkrecht darüber, der Effekt bleibt aber bei Versatz — Kandidat für
   eine Oberseiten-Auswahl über die Tiefe.
3. **Rate:** 2 Messungen/s bei 460 ms Alter (Debug-Bild an, gemeinsamer
   Python-Prozess, L2).

### L12 — Nah-Gate: nur gültige Tiefe über dem Band zählt (umgesetzt)

Folgerung aus L11, Befund 1. `near_mask` (gemeinsamer Kern, wirkt in beiden
Varianten auf das Gate und in `robot_cam_2` auf die Tiefenkanten) zählt fehlende
Tiefe (0) nicht mehr als „nah“, nur noch gültige Tiefe mindestens 20 mm über dem
Band. Die matte Oberseite liefert aus Beobachtungshöhe gültige Tiefe (325–385 mm
gemessen); fehlende Tiefe kommt von Glanz, glänzenden Seitenflächen oder zu
geringem Abstand. Tests: synthetische Klötze jetzt erhöht statt Tiefe 0; neu ein
Glanzstreifen ohne Tiefe, der verworfen werden muss.

**Nachgerechnet mit dem echten Code an den fünf Rohansichten aus L11** (Abstand
der gemeldeten Mitte zur offline ausgeschnittenen Oberseite, ~0,6 mm/px):

| Ansicht | vorher | nachher |
|---|---|---|
| F1 | 31 px (mit Glanzstreifen verschmolzen) | **5 px** |
| F2 | 66 px | 28 px |
| F3 | **260 px** (Glanzstreifen gewählt) | **4,5 px** |
| F4 (Werkzeug ~20° gekippt) | 22 px | 22 px |
| F5 (Klotz am oberen Bildrand) | 44 px, falsch | keine Erkennung |

Der Glanzstreifen ist damit gelöst. Offen bleiben 14–17 mm Versatz in F2/F4 —
vermutlich Seitenfläche oder Schatten im Umriss (L11, Befund 2) — und die Rate.

### L13 — Roboterkamera nach dem Build: Hand-Auge bestätigt, Erkennung nicht

Gegenprobe an einer neuen Pose über dem angetasteten flachen Klotz: offline
ausgeschnittene Oberseite mit der neuen Hand-Auge-Kalibrierung **3,3 mm** vom
Antastpunkt — die Kalibrierung trägt (sechs Ansichten, 2–4 mm). `robot_cam_2` selbst
meldete die Mitte **32 mm** daneben (62 px). Offline an allen sechs Rohbildern:

| Ansatz | Abweichung zur Oberseite je Bild (px, ~0,6 mm/px) |
|---|---|
| Kante (`robot_cam_2`) | 5 · 28 · 4 · 22 · — · 66 |
| Farbe (`robot_cam`) | keine Erkennung (Bandmaske versagt) |
| nur Tiefe über dem Band | 36 · 23 · 45 · — · — · — |
| positive Klotzfarbe (gesättigt), Tiefe nur für die Oberseite — offline | ≈ 0 |

Beim flachen Klotz trennt die Tiefe kaum (Oberseite 23 mm über dem Band), und die
Kanten schließen sich oft nicht. Die positive Klotzfarbe trägt für Rot und Blau;
Weiß ist offen (K3). **Entscheidung:** Die Roboterkamera wird vorerst nicht weiter
repariert; der erste Griff läuft mit der Basiskamera allein (Gewichte 0, Z10). Ein
neuer Erkennungskern entsteht am Schreibtisch mit den gesicherten Rohbildern.

### L14 — Arbeitsraum (B10), Greifzone (B19), Greifhöhe

Von Hand abgefahren (Nutzer): sicher, ohne Singularität, außerhalb des Bildes der
Basiskamera — **x −1,000 … −0,530, y −0,320 … +0,445**; y −0,32 ist das Bandende
(danach senkt sich das Band). **z max 0,60** (darüber Singularität im hinteren
Bereich), **z min 0,3086** (geschlossene Backenspitze 10 mm über dem Band:
53,6 + 10 + 245 mm). Die Gelenke wurden nicht mitgeschrieben.

- **Ablage neben dem Band** (x −0,316, y +0,476) liegt außerhalb → Arbeitsraum für
  den Follower erweitert auf **x_max −0,30, y_max +0,48**. Auf dem Band zielt der
  Follower nie dorthin (Zone, Klemmung beim Anfahren); nur der Transfer auf
  Freihöhe führt hinein. Festgehalten in `Safety/workspace_bounds.json`
  (`status: defined`).
- **Greifzone** (`priority_handler`, Standardwert; ⚠️ seit 24.09. = Arbeitsraum, L21): x −0,95 … −0,68,
  **y +0,40 … −0,22** — 5 cm unter dem Arbeitsraumrand fürs Anfahren, 10 cm über dem
  Bandende fürs Mitfahren nach dem Greifen.
- **`min_grip_height_m` 0,015 → 0,021:** Sonst läge die Greifhöhe flacher Klötze
  (Flansch 0,3036) unter der Arbeitsraum-Untergrenze, und das Gate bräche ab. Die
  Auflage reicht von 11 bis 31 mm; die Greifbarkeitsgrenze 30 mm bleibt.
- **Wartepose am Zonenanfang:** Ein synthetischer Lauf mit Wartepose am Zonenende
  (y −0,20) fand kein erreichbares Ziel mehr. Vorschlag `observe_*`: x −0,816
  (Bandmitte), **y +0,35**, Gier 90°.
- **Höhe beim Folgen:** Mit `observe_z` 0,60 und `t_descend_s` 2,0 liegt die
  Greifebene bei 0,128 m/s bei y ≈ +0,32 — nur 8 cm nach Zonenbeginn. Ohne
  Roboterkamera spricht nichts gegen tieferes Folgen: **`observe_z` 0,45,
  `t_descend_s` 1,2** (0,14 m / 0,15 m/s + Einschwingen) → Greifebene y ≈ +0,20,
  20 cm Fenster. Für die Roboterkamera später wieder höher (B8).

### L15 — Arbeitsraum und Beobachtungspose als Standardwert (24.09.2026)

Nutzer: Die Werte aus L14 gelten als fest und werden Standardwerte. Damit ist F3
(Nachtrag 8) in seinem Grund überholt — der Arbeitsraum ist dokumentiert
festgelegt, die Beobachtungspose ist für den Betrieb ohne Roboterkamera entschieden.

- **Standardwerte** in `FollowerParams` und in der Komponentenbeschreibung:
  `ws_*` = `Safety/workspace_bounds.json` (x −1,0 … −0,30, y −0,32 … +0,48,
  z 0,3086 … 0,60); `observe_*` = −0,816 / +0,35 / 0,45 / 90°.
- **Pflicht bleibt:** Wird ein Feld in der Oberfläche geleert, scheitert
  `configure` wie bisher mit der Liste der fehlenden. Die Prüfung als Satz
  (Beobachtungs-, Ablagepose und Freihöhe im Arbeitsraum) bleibt unverändert.
- **Zwei Stellen, eine Quelle:** Ein Test prüft, dass die Standardwerte gleich
  `workspace_bounds.json` sind. Wird der Arbeitsraum neu abgefahren, Datei,
  `FollowerParams` und Komponentenbeschreibung gemeinsam ändern.
- **Grenzen:** Von Hand gesetzte Werte eines Blocks gehen dem Standardwert vor. Wird
  Band oder Roboter verschoben, gilt der Standardwert unbemerkt weiter —
  dann B10 neu abfahren. Mit Roboterkamera muss `observe_z` wieder höher (B8).

### L16 — Kameras auf 15 Bilder/s, Infrarotbilder der Roboterkamera aus (24.09.2026)

Der Rechner hat **keine NVIDIA-Grafik** (Intel Core 3 100U mit integrierter Grafik,
2 Performance- und 4 Effizienzkerne): „Enable GPU capabilities“ im AICA-Launcher
kann nicht funktionieren, und die eigenen Komponenten rechnen ohnehin auf der CPU.
Entlastet wurde deshalb an den Kameratreibern (`event_engine`, dort läuft auch der
500-Hz-Regelkreis).

- **Profile auf 15 Bilder/s** (Nutzer, im AICA-Block): Basiskamera Farbe
  `1280x720x15`, Tiefe `640x480x30` (die L515 kann nur 30); Roboterkamera beide
  `848x480x15`. Gemessen über `camera_info`: 15,0 / 15,0 Bilder/s. `base_cam`
  liefert unverändert **8,6 Messungen/s**, Alter bei Ankunft Median 161 ms, vor der
  nächsten Median 263 ms / 95 % 357 ms — sie war nicht durch die Kamerarate
  begrenzt. ⚠️ `ros2 topic hz` auf Bild-Topics misst sich selbst (4–7/s); Raten
  über die kleinen `camera_info`-Topics messen.
- **Infrarotbilder der Roboterkamera aus:** Der D435i-Treiber publizierte
  `infra1`/`infra2` und richtete die Tiefe zusätzlich auf `infra1` aus. Der
  Parameter `enable_infra` des AICA-Blocks gilt nur für die L515 und erreicht sie
  nicht. Zur Laufzeit abgeschaltet: `event_engine` **123 → 106 % CPU**, ausgerichtete
  Tiefe 12,9 → 14,6 Bilder/s. **Dauerhaft:** `robot_cam_2` schaltet sie bei jeder
  Aktivierung über den neuen Parameter `camera_node` (Standard
  `/realsense_camera_2`) ab — wie `base_cam` `global_time_enabled` erzwingt.
  `robot_cam` hat das nicht (nicht in der Anwendung, L5).
  ⚠️ **Nach dem Build hängte sich damit die Anwendung auf:** Die Roboterkamera
  antwortete nicht mehr auf Parameteranfragen, die ausgerichtete Tiefe blieb aus,
  und nach dem Neustart luden Greifer und `data_tracker` nicht mehr, und AICA ließ
  sich nicht mehr beenden. Mit geleertem `camera_node` lief alles wieder.
  Vermutung: Das Umschalten der Ströme kurz nach dem Start des Treibers blockiert
  ihn und damit den `event_engine`; von Hand im laufenden Betrieb ging es. Bis das
  geklärt ist, **`camera_node` am Block leer lassen** — seit 24.09. auch der
  Standardwert.
- **Größter verbleibender Verbraucher außerhalb von AICA:** die AICA-Oberfläche
  (`WebKitWebProcess`) mit ~100 % eines Kerns — während eines Laufs minimieren.

### L17 — Der Follower hatte keine Signale; der Fake stempelte in Laufzeit (24.09.2026)

Beim Trockenlauf zu Block 7 blieb die Statusanzeige des Followers leer. Im Log beim
Laden: `Failed to add input 'robot_state': ParameterError … must be real number,
not str`, ebenso für alle anderen Ein- und Ausgänge.

- **Ursache:** modulo legt zu jedem Signal einen Text-Parameter `<signal>_topic`
  an und prüft ihn über dasselbe `on_validate_parameter_callback`. Die Prüfung des
  Followers behandelte **jeden** Parameter als Zahl (`math.isfinite`) — der Aufruf
  scheiterte an der Zeichenkette, das Signal wurde nicht angelegt. Seit dem ersten
  Build; aufgefallen ist es erst jetzt, weil der Follower bisher nie verdrahtet lief.
  Die anderen Komponenten prüfen nur namentlich aufgezählte Parameter.
- **Korrektur:** Die Prüfung greift nur für die eigenen Parameter
  (`FollowerParams` und `robot_state_max_age_s`). Test in der AICA-Umgebung: zu
  jedem Signal existiert der Topic-Parameter.
- **`fake_objects.py`:** stempelte S1 mit Sekunden seit dem Start. `vectoring` und
  `priority_handler` rechnen nur mit Zeitdifferenzen und liefen damit (Pool
  −70,0 mm/s bei `--velocity -0.07 --rate 8`), der Follower altert Ziele aber gegen
  seine Uhr und hätte sie als ~56 Jahre alt verworfen. Jetzt trägt S1 die ROS-Zeit;
  die Szene läuft weiter in Sekunden seit dem Start.

### L18 — Block 7: erster Lauf am echten Roboter, gedrosselt, bestanden (24.09.2026)

Zielquelle `fake_objects.py` (`--velocity -0.07 --rate 8`), Basiskamera aus, Band
steht und ist leer, IK-Controller `max_linear_velocity` 0,10 m/s, Hand am Not-Aus.
Kette: `object_follower.target_pose` → Attractor (`attractor`) → `twist` →
IK-Controller. Mitgelesen: Zustand, Zielpose, Flanschpose, `gripper_close`; ein
Wächter hätte den Fake beim Verlassen des erwarteten Quaders beendet (löste nie aus).

1. **Trockenlauf ohne Anschluss:** Zustände und Zielposen plausibel (ANFAHREN auf den
   Zonenanfang y +0,40 geklemmt, FOLGEN mit 70 mm/s, Gier 90°). Beobachtet: Nach der
   Aktivierung bleibt der Follower in ABBRUCH, bis der Flansch über der Freihöhe
   steht (≥ 0,48) — ohne angeschlossene Bewegung also für immer; gewollt.
2. **Verdrahtungsfehler vor dem ersten Lauf:** `target_pose` hing direkt am
   IK-Controller statt am Attractor — vor dem Start durch Mitlesen gefunden. Vor jedem
   Lauf prüfen: Abnehmer von `target_pose` ist `signal_point_attractor`.
3. **Folgen ohne Absenken, Attractor K = 1 (AICA-Standard):** Flansch 68–70 mm hinter
   der Zielpose = v/K. **`linear_gains` [5.0] und `rate` 50 Hz am Attractor gesetzt**
   (Einrichtung §2), `attractor_v_max_mps` im `priority_handler` = 0,10 (kleinere
   der beiden Grenzen).
4. **Mit K = 5:** Flansch 14 mm hinter der Zielpose (70/5), der Vorhalt 0,2 s gleicht
   es aus: **`err_laengs` −2,4 … −2,8 mm, `err_quer` ±0,1 mm**, Einschwingen ~1,5 s.
   Feinabstimmung `lead_time_s` ≈ 0,24 erst am echten Band (B4).
5. **Mit Absenken, `descend_speed_mps` 0,15:** kein Griff — viermal ABSENKEN → FOLGEN.
   Ursache: Die IK-Grenze gilt für den **Betrag** der Geschwindigkeit; Band 0,07 und
   Absenken 0,15 ergeben 0,17 m/s, gekappt auf 0,10 fällt der Arm längs zurück
   (−11 … −15 mm) und der Rücksprung F3 greift — Schutzfunktion korrekt.
6. **`descend_speed_mps` 0,05, `t_descend_s` 3,0, `timeout_track_s` 5:** voller
   Zyklus. ABSENKEN bei `err_laengs` −2 … −3,5 mm von 0,45 auf **0,339** (Klotz
   100 mm: 53,6 + 50 + 235 = 338,6 mm), GREIFEN, Greifer zu, **„Greifer geschlossen,
   kein Klotz (Fehlgriff)“** nach 1 s, öffnen, senkrecht auf 0,49, WARTEN. Klotz 2
   (1,06 s bis zur Greifebene gewählt): „Greifebene überschritten, bevor abgesenkt
   wurde“ → sauberer Abbruch. `picked_id` gibt jedes Mal das nächste Ziel frei.

7. **Betriebsgeschwindigkeit (IK 0,30, `descend_speed_mps` 0,15, `t_descend_s` 1,2), Fake
   0,13 m/s:** Zyklus in ~2 s, `err_laengs` beim Greifen −4,6 … −5,0 mm — an der
   Toleranzgrenze. Der Rest ist in beiden Läufen 0,037 s × v → **`lead_time_s` 0,24**
   (Standardwert seit dem Build vom 24.09.), `attractor_v_max_mps` Standard 0,30,
   `t_descend_s` Standard 1,2 (Absenken gemessen ~1,0 s; vorher 2,0 für `observe_z` 0,60).
   Der Kontrolllauf mit 0,24 brach an einem **Einbruch der 500-Hz-Schleife** ab
   (4/500 Hz für ~1 s, „RTDE Pipeline overflowed“, External Control stoppte);
   Gate brach korrekt ab. Die Schleife läuft auch in Ruhe nur mit 84–86 %; der
   Rechner ist zeitweise voll (Warteschlange 5–13 bei 8 Threads), `event_engine`
   wächst (430 → 705 MB). Mitlesen jetzt ohne die 500-Hz-Flanschpose.

**Offen:** Kontrolllauf `lead_time_s` 0,24; Ablegen (nur mit echtem Klotz im Greifer), IK auf Betriebswert mit
`descend_speed_mps` 0,15 und `t_descend_s` 1,2 zurück, dann Basiskamera und echtes
Band. Beobachtung: Der `priority_handler` wählte Klotz 2 mit 1,06 s Restzeit — die
Einschwingzeit des Followers (~1,5 s) steckt nicht in der Erreichbarkeit.

### L19 — Erste echte Griffe im Lauf: zwei von zwei (24.09.2026)

Basiskamera aktiv, Band ≈ 0,13 m/s, IK-Controller 0,30 m/s, Attractor K = 5 / 50 Hz,
Follower `lead_time_s` 0,24, `descend_speed_mps` 0,15, `stable_cycles` 10,
`timeout_track_s` 3, `priority_handler` `t_descend_s` 1,2. Je ein Klotz von Hand
am Bandanfang aufgelegt. **Genau der Pick im Lauf ist der Punkt, an dem das
Vorgängerprojekt gescheitert ist.**

| | Klotz 1 (50 × 50 × 100, hochkant) | Klotz 2 (50 × 75 × 25, auf der Schmalseite, 75 hoch) |
|---|---|---|
| Lage quer | x −0,770 | x −0,694 (Bandrand) |
| gewählt | 4,5 s vor der Greifebene | 4,9 s vor der Greifebene |
| `err_laengs` / `err_quer` beim Absenken und Greifen | +0,3 … +1,0 / ±0,2 mm | +0,6 … +1,1 / ±0,1 mm |
| Greifhöhe Flansch | 336,6 mm (→ Klotz ≈ 96 mm) | 325,7 mm (→ ≈ 74 mm) |
| Ergebnis | gegriffen, gehoben, **abgelegt** (outcome 0) | gegriffen an der breiten Fläche, **abgelegt** |
| Wahl → abgelegt | ~9 s | ~9 s |

- **Beobachtung am Aufbau (Nutzer):** Beide Griffe sahen richtig aus, beide Klötze
  liegen auf der Ablageposition. Damit bestätigt: **B9** (Ablagepose), **245 mm**
  Flansch → Backenspitze und Bandhöhe im Rahmen, **B4** `lead_time_s` 0,24 mit echter
  Kameralatenz (Rest ≈ +1 mm statt −5 mm mit 0,20 im Fake).
- **Klotz 2 flach liegend (25 mm hoch):** keine Reaktion — so gewollt, er liegt unter
  der Greifbarkeitsgrenze 30 mm (B15). Im Log fehlt allerdings die Meldung „nicht
  greifbar“; ob er als Track ankam, ist nicht belegt → beim nächsten Mal `tracks`
  mitlesen.
- Beim Anfahren je einmal „Vorhersage gedeckelt: S4 0,60 / 0,61 s alt“ — knapp über
  dem Deckel 0,6 s, ohne Folgen. Die 500-Hz-Schleife lief mit 74–78 %, ohne Einbruch.

**Nächste Schritte:** mehrere Klötze nacheinander und kurz hintereinander
(Priorisierung, Ziel 4), Farben, flache 50-mm-Klötze (Grenzfall Greifhöhe),
Dauerlauf auf Stabilität der Regelschleife.

### L20 — Schutzstopp am Bandrand: falsche Nutzlast, Anfahren ohne Beschleunigungsgrenze (24.09.2026)

Nach zwei weiteren fehlerfreien Griffen (Ziel 3 und 5) löste ein Klotz am Bandrand
gleich zu Beginn von ANFAHREN einen **Schutzstopp der UR-Steuerung** aus: **C157A2**
„Roboter konnte dem Pfad nicht folgen (Kollision oder falsche Einstellung)“ und
**C162A0** (Hinweis: falsche Nutzlastmasse/Schwerpunkt kann zu Sicherheitsstopps
führen). Keine Kollision. Mit der Frame-Steuerung waren dieselben Stellungen
problemlos angefahren worden.

- **Ursache (sehr wahrscheinlich, beides zusammen):**
  1. **Nutzlast am UR 2,8 kg eingetragen, gemessen 1,3 kg** (Assistent „Messen“,
     Greifer leer, Roboterkamera dran): Schwerpunkt **CX 12 / CY 24 / CZ 45 mm**
     (alter Schwerpunkt unbekannt). Mit falschem Modell hält die Steuerung starke
     Beschleunigungen für eine Störung.
  2. **Anfahren als Sprung:** Mit der Wahl springt das Ziel von der Beobachtungspose
     zum Anfahrpunkt; der Attractor fordert K × Abstand (K = 5), gekappt auf
     0,30 m/s — und der IK-Controller hatte **keine Beschleunigungsgrenze**
     (`command_rate_limit` = ∞). Bei der Frame-Steuerung (K = 1, schrittweise Ziele)
     gab es diesen Sprung nie. Am Bandrand ist er am größten.
- **Abhilfe (Nutzer, am Aufbau):** Nutzlast **1,3 kg** mit gemessenem Schwerpunkt in
  der Installation gespeichert; IK-Controller **`command_rate_limit` 2,0** — die
  größte Änderung des Befehls pro Sekunde (modulo `RobotControllerInterface`), beim
  IK-Controller also Gelenkbeschleunigung in rad/s².
- **Wiederholung:** drei Griffe, alle abgelegt, kein Stopp — darunter der Bandrand
  **x −0,94** (Anfahrt 13 cm quer), `err_laengs` +0,3 … +2 mm. Seit dem ersten echten
  Griff damit **7 von 7** greifbaren Klötzen abgelegt (L19, L20).
- Die Nutzlast gilt für das leere Werkzeug; gegriffene Klötze (0,1–0,2 kg) liegen
  in der Toleranz der Steuerung.

### L21 — Greifzone gleich Arbeitsraum; Bildausschnitt der Basiskamera an den Roboter angepasst (24.09.2026)

Beobachtung (Nutzer): Es lagen mehr Klötze auf dem Band, als erkannt wurden, vor allem
an den Seiten. Ausgewertet an zwei Einzelbildern (Band steht) mit der Erkennung von
`base_cam` offline:

- **Das Band ist im Bild ~0,8 m breit** (u ≈ 335 … 1185, world x ≈ −0,49 … −1,25). Der
  Ausschnitt u 360 … 1160 schnitt es am **roboternahen Rand** an; ein Klotz dort
  (u 352 … 408, x −0,536) berührte den Ausschnittrand und wurde nach der Randregel
  verworfen. Mit `roi_x` 342 wird er erkannt (75 mm hoch, 50 × 30).
- **Ein Klotz bei x −0,961 wurde erkannt, lag aber außerhalb der Greifzone**
  (x −0,95 … −0,68) — der `priority_handler` wählte ihn nie.
- **Ein flach liegender 25-mm-Klotz** liegt roh nur 11–15 mm über dem Band (Unterschätzung
  der Oberkante, L6) und fällt unter `min_obj_height` 15 mm — mal erkannt, mal nicht.
  Greifbar ist er ohnehin nicht (< 30 mm). Offen: `min_obj_height` 10, vorher am leeren
  Band auf Falschmeldungen prüfen.

**Entscheidung (Nutzer):** Die Greifzone bekommt keinen eigenen Sicherheitsabstand mehr —
der steckt schon in den abgefahrenen Werten. **Zone x −1,0 … −0,53, y −0,32 … +0,445**
(Standardwert). Der **Bildausschnitt** reicht vom Bandrand am Roboter bis zur
Arbeitsraumgrenze x −1,0: **`roi_x` 342, `roi_width` 618** (u 342 … 960; u 960 = x −1,03
auf Höhe einer 100-mm-Oberkante, damit ein Klotz mit Mitte bei −1,0 ganz im Ausschnitt
liegt). Klötze jenseits von x −1,0 sind nicht erreichbar und fallen jetzt heraus; der
kleinere Ausschnitt spart Rechenzeit.

Zu beobachten: Die Wartestellung am Zonenanfang liegt jetzt bei y +0,445, 8–10 cm vor dem
Bildbeginn der Basiskamera (y ≈ +0,52 … 0,54). Der Greifer steht dort hoch
(> `max_obj_height_mm` über dem Band) und wird nicht als Klotz gemeldet — beim ersten
Lauf gegenlesen.
