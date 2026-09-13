# Design: Object Follower Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung — **in vier Stufen bauen, nicht am Stück**

## Zweck

Das Herzstück des On-the-fly-Pickvorgangs. Führt den Roboter an den fahrenden
Block heran, hält ihn darüber, senkt ab, greift mitfahrend, hebt ab und legt ab.

Ausgabe ist eine **Zielpose**, die an den `attractor`-Eingang des
`SignalPointAttractor` geht — an die Stelle, an der heute `frame_to_signal`
hängt. Der Attractor macht daraus einen Twist, der IK-Velocity-Controller
Gelenkgeschwindigkeiten.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`, besonders: nie blockieren, kein `time.sleep()`, kein
`copy.deepcopy()` auf `state_representation`-Objekte.
Datenverträge: `docs/review/datenvertraege.md` (S2, S4, S9 ein; S6, S7, S8 aus).
Architekturentscheidungen: `docs/review/entscheidungen.md` Themen 1, 2, 6, 7.

## Regelkette

```
object_follower ──Zielpose──▶ SignalPointAttractor ──Twist──▶ IKVelocityController
       ▲                              ▲
       └──── cartesian_state ─────────┴──── robot_state_broadcaster
```

**Warum eine Pose und kein Twist:** Der Attractor rechnet
`v = K · (ziel − tcp)`. Er erzeugt Geschwindigkeit nur bei vorhandenem Abstand.
Bei einem fahrenden Ziel bleibt deshalb dauerhaft ein Abstand `v_band / K`
bestehen — bei 0,2 m/s und K = 5 sind das 4 cm, stabil und damit von einer
Stabilitätsprüfung nicht zu unterscheiden. Deshalb der **Vorhalt**:

```
zielpose = blockposition_prädiziert + bandrichtung · lead_offset_m
```

Mit `lead_offset_m = v_band / K` steht der TCP im eingeschwungenen Zustand genau
auf dem Block. Wird später der `base_frame`-Eingang des Attractors genutzt
(mitfahrender Bezugsrahmen), geht `lead_offset_m` auf 0 — **ohne Codeänderung**.

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `ObjectFollower` |
| Basisklasse | `LifecycleComponent` |
| Python-Datei | `roboter_tetris/object_follower.py` |
| Registrierung | `roboter_tetris::ObjectFollower = roboter_tetris.object_follower:ObjectFollower` |
| UI-Anzeigename | `Object Follower` |
| Rate | **100 Hz** (Hardware-Rate; glatte Zielpose) |

## Schnittstellen

### Inputs

| Signal | Typ | Quelle |
|---|---|---|
| `target` | `Float64MultiArray` | S4 aus `priority_handler` |
| `object_position` | `Float64MultiArray` | S2 aus `robot_cam` |
| `robot_state` | `cartesian_state` | `robot_state_broadcaster` |
| `gripper_is_closed` | `Bool` | `robotiq_gripper` |
| `gripper_has_object` | `Bool` | `robotiq_gripper` |

### Outputs

| Signal | Typ | Ziel |
|---|---|---|
| `target_pose` | `cartesian_state` (A2 bestätigen) | `attractor`-Eingang |
| `gripper_close` | `Bool` | `robotiq_gripper` |
| `picked_id` | `Float64MultiArray` | S7 → `priority_handler`, `data_tracker` |
| `follower_status` | `Float64MultiArray` | S8 → `interface_streamer` |

### Predicates

| Name | Bedeutung |
|---|---|
| `is_tracking` | Zustand `FOLGEN` oder `ABSENKEN` |
| `is_holding_object` | Block im Greifer |
| `has_aborted` | letzter Versuch endete im Abbruch |

### Parameter

**Band (aus B1)**

| Name | Typ | Bedeutung |
|---|---|---|
| `belt_heading_deg` | double | Laufrichtung in der xy-Ebene von `world` |
| `belt_speed_mps` | double | Bandgeschwindigkeit |
| `belt_surface_z_m` | double | Höhe der Bandoberfläche in `world` (B17) |

**Regelung**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `lead_offset_m` | double | – (B4) | Vorhalt entlang der Bandrichtung; `v_band / K`. Bei Option C auf 0. |
| `latency_compensation_s` | double | 0.0 | Kamera- und Kommandolatenz |
| `max_extrapolation_s` | double | 0.2 | Deckel der Vorhersage (Sicherheit) |

**Kameramischung**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `weight_along` | double | 0.0 | Gewicht der Roboterkamera in Bandrichtung (0…1, entspricht X/100) |
| `weight_across` | double | 0.0 | Gewicht quer (entspricht Y/100) |
| `w_ramp_s` | double | 0.2 | Übergangsdauer beim Ein-/Ausblenden |
| `correction_filter_window` | int | 10 | Glättung der Korrektur |
| `max_correction_m` | double | 0.05 | darüber gilt die Messung als anderes Objekt (R4) |
| `robot_cam_max_age_s` | double | 0.3 | darüber Rückfall auf `w = 0` |
| `require_robot_cam_for_grasp` | bool | false | Greif-Freigabe nur mit frischer Messung (D9/D10) |

**Kalibrierung (aus C1, C8)**

| Name | Typ | Bedeutung |
|---|---|---|
| `handeye_x/y/z` | double | Hand-Auge Kamera → Flansch, Translation |
| `handeye_roll/pitch/yaw` | double | dito, Grad |
| `tool_offset_z_m` | double | Flansch → Greifpunkt |
| `gripper_yaw_offset_deg` | double | Montagewinkel der Backen |

**Posen**

| Name | Typ | Bedeutung |
|---|---|---|
| `observe_x/y/z` | double | Beobachtungspose (B8) — Orientierung **fest senkrecht** |
| `place_x/y/z` | double | Ablagepose über der Auffangkiste (B9) |
| `transfer_height_m` | double | Freihöhe für die Transferfahrt (D12) |

**Greifen**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `min_grip_height_m` | double | – (B15) | untere Grenze der Greifhöhe (schützt das Band) |
| `descend_speed_mps` | double | 0.05 | Sinkgeschwindigkeit |
| `lift_clearance_m` | double | 0.10 | Höhe, ab der das Mitfahren endet |
| `tol_along_m` / `tol_across_m` / `tol_z_m` / `tol_yaw_rad` | double | – (B18) | **vier getrennte** Freigabetoleranzen. Der Gierwinkel gehört zwingend dazu: Im Modus 2 kann der Roboter positionsmäßig stehen, während das Handgelenk noch dreht. |
| `stable_cycles` | int | 10 | wie lange die Toleranz gehalten werden muss (V aus dem Plan) |
| `use_block_orientation` | bool | false | Modus 2 (beliebige Lage) statt Modus 1 (gerade aufgelegt) |

**Sicherheit**

| Name | Typ | Bedeutung |
|---|---|---|
| `ws_x_min` … `ws_z_max` | double | Arbeitsraum (B10) — **nicht** die Greifzone |
| `max_target_jump_m` | double | Sprungerkennung im Sicherheitsgate (D15) |

**Timeouts** — `timeout_approach_s`, `timeout_track_s`, `timeout_grasp_s`,
`timeout_place_s`, `timeout_release_s`

## Zustandsautomat

| Zustand | Regelverhalten | Übergang | Timeout → |
|---|---|---|---|
| `WARTEN` | Beobachtungspose, **kein Vorhalt** | `has_target = 1` → `ANFAHREN` | – |
| `ANFAHREN` | Ziel auf Greifzone begrenzt | Block in der Zone → `FOLGEN` | `ABBRUCH` |
| `FOLGEN` | volle Regelung, Orientierung festlegen | Toleranz für `stable_cycles` gehalten → `ABSENKEN` | `ABBRUCH` |
| `ABSENKEN` | wie `FOLGEN` + `vz` abwärts, **Korrektur eingefroren** | Greifhöhe erreicht → `GREIFEN` | Abweichung wächst → **auf Beobachtungshöhe steigen**, zurück zu `FOLGEN` |
| `GREIFEN` | **reines Mitfahren**, Greifer schließt | `has_object` → `HEBEN` | `ABBRUCH`, `outcome = 1` |
| `HEBEN` | Mitfahren bis `lift_clearance_m`, dann steigen bis `transfer_height_m` | erreicht → `ABLEGEN` | `ABBRUCH` |
| `ABLEGEN` | Ablagepose, **kein Vorhalt** | `is_in_range` → `LOESEN` | `ABBRUCH` |
| `LOESEN` | Greifer auf, `picked_id` mit `outcome = 0` | fertig → `WARTEN` | `ABBRUCH` |
| `ABBRUCH` | TCP halten, Greifer auf, senkrecht hoch, Beobachtungspose | → `WARTEN` | – |

**Start im Zustand `ABBRUCH`.** Beim Anwendungsstart steht der Roboter irgendwo,
und der Attractor führe geradlinig zur Beobachtungspose — je nach Ausgangslage
durch das Band. Der Abbruchpfad macht bereits das Richtige: senkrecht hoch, dann
Beobachtungspose. Ein Codepfad, zwei Zwecke.

**Abbruchgründe aus jedem Zustand:** `has_target = 0` · Ziel-ID aus `target`
verschwunden · unplausibler Sprung der Regelabweichung · Zeitstempel der
Eingangssignale steht still.

## Kernlogik

### Prädiktion

```
p(t) = p_ziel + bandrichtung · belt_speed_mps · (t − t_ziel + latency_compensation_s)
```
mit `(t − t_ziel)` gedeckelt auf `max_extrapolation_s`.

Die Prädiktion dient **nicht primär** der Latenzkompensation, sondern der
Glättung: `base_cam` liefert ~30 Bilder/s, der Follower läuft mit 100 Hz. Ohne
Extrapolation stünde die Zielpose zwischen zwei Bildern still und spränge dann —
bei 0,2 m/s in 6,7-mm-Stufen. Der Attractor sähe eine Treppe statt einer Fahrt.

### Kameramischung — Korrektur filtern, nicht Position mischen

```
p_robotcam_welt = T_world_flansch(t_bild) · T_flansch_kamera · p_kamera
korrektur       = p_robotcam_welt − p_prädiktion
zielpunkt       = p_prädiktion + w · korrektur_gefiltert
```

Mathematisch identisch zur wörtlichen X/Y-Mischung, aber **filterbar**: Die
absolute Position wandert mit Bandgeschwindigkeit und lässt sich nicht glätten,
ohne Verzögerung einzubauen. Die Korrektur ist dagegen nahezu konstant — sie
besteht aus dem Versatz der beiden Kalibrierketten plus Rauschen. Damit landet
das Rauschen der Roboterkamera nicht im Zielsignal.

`w` wird **achsweise** angewandt (`weight_along`, `weight_across`) und über
`w_ramp_s` ein- und ausgeblendet. Auf 0 zurück bei:
`valid = 0` · Messung älter als `robot_cam_max_age_s` · `|korrektur| > max_correction_m`.

**Beim Übergang `FOLGEN → ABSENKEN` wird die Korrektur eingefroren** und bis
einschließlich `GREIFEN` konstant gehalten. Grund: Die Kamera unterschreitet beim
Absenken zwangsläufig `min_belt_distance_m` und meldet `valid = 0`. Ohne
Einfrieren würde die Korrektur genau dann ausgeblendet, wenn der Greifer um den
Klotz herum nach unten fährt — die Zielpose wanderte dabei seitlich um den Betrag
der Korrektur. Das Einfrieren ist auch sachlich richtig: Die Ausrichtung wurde
vorher geprüft, danach soll sich am Ziel nichts mehr ändern.

Die letzte Bedingung ist die Identitätsprüfung (R4): Sieht die Roboterkamera
einen anderen Block, springt die Korrektur auf dessen Abstand — bei von Hand
aufgelegten Blöcken deutlich über den wenigen Zentimetern des Normalfalls.

### TCP-Historie

Ringpuffer `(t, Flanschpose)` aus `robot_state`, etwa 1 s bei 100 Hz.
`robot_cam`-Messungen werden über ihren Bildzeitstempel interpoliert
zugeordnet — nicht über die aktuelle Pose. Bei 0,2 m/s TCP-Geschwindigkeit und
40 ms Latenz wären das sonst 8 mm Fehler, und zwar genau in der
Feinpositionierung.

### Orientierung

```
gierwinkel = gripper_yaw_offset_deg + ( use_block_orientation
                                        ? block_orientierung
                                        : belt_heading_deg )
```

Modus 1 ist damit ein **Spezialfall** von Modus 2, kein eigener Zweig — wichtig,
damit Modus 2 nicht erst am Ende getestet wird.

Von den zwei gleichwertigen Handgelenkstellungen (der Greifer ist
180°-symmetrisch) die **nähere zur aktuellen** wählen, sonst dreht das Gelenk
gelegentlich eine halbe Umdrehung und läuft in seine Grenzen.

Die Orientierung wird **einmal festgelegt und beim Übergang nach `ABSENKEN`
eingefroren**. Eine Winkeländerung während des Absenkens ist unnötig und
gefährlich.

### Greifhöhe

```
greif_z = belt_surface_z_m + max( blockhoehe / 2, min_grip_height_m )
```

Mittig auf der Seitenfläche gibt nach oben und unten Reserve. Die untere Grenze
schützt das Band: Die Backen haben selbst Höhe; bei einem flachen Block läge ihre
Unterkante sonst unter der Bandoberfläche.

Bezug ist die **Bandoberfläche** — ein fester, einmal vermessener Wert — und
nicht die gemessene Blockoberkante, die an der Extrinsik der Basiskamera hängt.

### Greif-Freigabe

Abweichung zwischen TCP und **prädizierter Blockposition** (also im mitfahrenden
System, nicht absolute Positionsstabilität), getrennt geprüft gegen
`tol_along_m`, `tol_across_m`, `tol_z_m`, gehalten über `stable_cycles`.

Das Attractor-Prädikat `is_in_range` ist hier **nicht** verwendbar: Der Vorhalt
hält den Abstand absichtlich ungleich null. Für stehende Ziele (`WARTEN`,
`ABLEGEN`) ist es dagegen genau das richtige Mittel.

### Werkzeugversatz

```
zielpose_flansch = zielpose_greifpunkt + [0, 0, tool_offset_z_m]
```

Eine einzige Zahl, weil der Greifer immer senkrecht nach unten zeigt und der
Greifpunkt auf der Flanschachse liegt. Eine Drehung um die Hochachse ändert den
Versatz nicht.

> Zu klären vor Stufe 4a (A7/A8/C8/C9): Regelt der Controller den Flansch oder
> den Greifpunkt? Regelt er den Flansch und wir geben die Blockposition direkt
> aus, fährt der Greifer rund 20 cm zu tief — ins Band.

### Selbstüberwachung des Vorhalts

Der Follower kennt die ausgegebene Zielpose und die gemeldete TCP-Pose. Im
eingeschwungenen Zustand muss deren Differenz dem eingestellten `lead_offset_m`
entsprechen. Dauerhafte Abweichung → Warnung ins Log.

Grund: `lead_offset_m` hängt am Gain des Attractors. Wird der im Studio verändert
— beim Abstimmen völlig normal — stimmt der Vorhalt nicht mehr und der Griff geht
**still** daneben. Fünf Zeilen gegen einen schwer auffindbaren Fehler.

### Sicherheitsgate

**Letzte Funktion vor der Ausgabe.** Nichts umgeht sie.

| # | Prüfung | Bei Verstoß |
|---|---|---|
| 1 | alle Eingangswerte endlich | Abbruch |
| 2 | Eingangszeitstempel schreiten fort | Abbruch |
| 3 | Extrapolationshorizont ≤ `max_extrapolation_s` | deckeln |
| 4 | Sprung zur vorigen Zielpose ≤ `max_target_jump_m` | **Abbruch**, nicht deckeln |
| 5 | Zielpose im Arbeitsraum | deckeln **und** Abbruch melden |

Zu 4: Ein großer Sprung heißt, dass die Daten falsch sind. Deckeln hieße, falschen
Daten langsam zu folgen. Die Prüfung gilt **innerhalb** eines Zustands und wird
bei jedem Zustandswechsel zurückgesetzt, weil dort legitime Sprünge auftreten.

Der wichtigste Weglaufpfad ist Prüfung 2/3: Stürzt `base_cam` ab, veralten
`p_ziel`/`t_ziel`, die Extrapolation läuft aber weiter — nach 10 s läge das Ziel
bei 0,2 m/s zwei Meter daneben.

## Ausführungsmodell

Ein `on_step_callback` bei 100 Hz, darin: Eingänge gaten → Zustandsautomat →
Zielpose berechnen → Sicherheitsgate → ausgeben. Keine Threads, keine
blockierenden Aufrufe. Alle Zeitmessungen über
`(self.get_clock().now() - t0).nanoseconds / 1e9`.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen, Hand-Auge-Matrix und Bandrichtung vorberechnen, Ringpuffer anlegen |
| `on_activate` | Zustand `ABBRUCH`, Ringpuffer leeren, `seq = 0`, Greifer öffnen |
| `on_deactivate` | Greifer öffnen; letzte Zielpose bleibt beim Attractor stehen (der Roboter fährt sie zu Ende an und hält — begrenzt, weil das Gate sie bereits im Arbeitsraum gehalten hat) |

## Fehlerbehandlung

- Jeder Zustand hat einen Timeout mit definiertem Rückfall
- Fehlgriff (`is_closed` ohne `has_object`) → `ABBRUCH`, `outcome = 1`
- Verlorenes Objekt → `ABBRUCH`, `outcome = 2`
- Nach jedem Abbruch wird `picked_id` gesendet, damit der `priority_handler` das
  Objekt als erledigt behandelt und das nächste wählt

## Dateien

- **Neu:** `roboter_tetris/object_follower.py`
- **Neu:** `component_descriptions/roboter_tetris_object_follower.json`
- **Neu:** `test/python_tests/test_object_follower.py`
- **Geändert:** `setup.cfg`

## Umsetzung in vier Stufen

| Stufe | Umfang | Prüft |
|---|---|---|
| **4a** | Gerüst, Sicherheitsgate, TCP-Ringpuffer; aktiv nur `WARTEN`, `ABLEGEN`, `ABBRUCH` | Attractor-Anbindung, Werkzeugversatz, Startverhalten |
| **4b** | `ANFAHREN` + `FOLGEN` mit `w = 0` | Prädiktion, Vorhalt (hier `lead_offset_m` kalibrieren) |
| **4c** | Kameramischung, Rampe, Rückfall, Identitätsprüfung | Hand-Auge, Korrekturfilter |
| **4d** | `ABSENKEN`, `GREIFEN`, `HEBEN`, `LOESEN`, `picked_id` | vollständiger Zyklus, erst Modus 1, dann Modus 2 |

Nicht am Stück bauen. Jede Stufe ist für sich testbar, und das Risiko wächst
schrittweise statt auf einen Schlag.

## Testbarkeit

Zustandsautomat, Prädiktion, Mischung, Winkelwahl und Sicherheitsgate in
ROS-freie Klassen auslegen — dieselbe Trennung wie `GripperTargetLogic` und
`MoveTriggerLogic` in den bestehenden Komponenten.

Ohne Hardware prüfbar: alle Zustandsübergänge und Timeouts · Prädiktion inklusive
Deckelung · Rampe und alle drei Rückfallgründe von `w` · Identitätsprüfung ·
Wahl der näheren Handgelenkstellung · Greifhöhe an der unteren Grenze · alle fünf
Gate-Prüfungen · `seq`-Vergabe bei Erfolg und Abbruch.

## Bewusste YAGNI-Entscheidungen

- **Kein eigenes Sicherheitsgate als Komponente** — bräuchte dieselben Daten,
  fügte Verzögerung und eine Fehlerquelle hinzu.
- **Kein Twist-Ausgang** (Option B). Nicht wegen verlorener
  Stabilitätseigenschaften — ein Point Attractor *ist* ein P-Regler, ein selbst
  gerechnetes `v = v_ff + K·(ziel − tcp)` mit Begrenzung wäre gleichwertig.
  Sondern weil die bestehende Kette getestet ist und ein neuer Signaltyp plus
  neue Controller-Verbindung bei knapper Laborzeit unnötiges Risiko sind.
- **Kein Wiederholversuch nach Fehlgriff.** Der Block ist weitergefahren; der
  `priority_handler` wählt neu.
- **Keine Ablage nach Farbe** — feste Ablagepose. Die Farbe liegt im
  `target`-Vertrag bereit, nachrüstbar ohne Schnittstellenänderung.
- **Kein `gripper_change`** zur Vorpositionierung der Öffnungsweite in der ersten
  Fassung. Die Blockbreite steht in S4 bereit.
- **Keine Kippung des Greifers.** Senkrecht hält den Werkzeugversatz bei einer
  Zahl und vermeidet den höhenabhängigen Parallaxenfehler der Roboterkamera.
