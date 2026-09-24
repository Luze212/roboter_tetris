# Design: Object Follower Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** **Alle vier Stufen ✅ umgesetzt 21.09.2026** (lokal getestet, in AICA
noch nicht gelaufen). Logik in `follower_logic.py` (ohne ROS), Schale in
`object_follower.py`. Beim Bau entschieden: `entscheidungen.md` **Nachtrag 8** (4a),
**9** (4b), **10** (4c, 4d).

> ⚠️ **Seit 23.09.2026 geänderte Standardwerte** (`entscheidungen.md` Nachtrag 13):
> `max_extrapolation_s` 0,2 → **0,6**, `target_timeout_s` 0,5 → **1,0** (L10);
> Hand-Auge `handeye_*` neu eingemessen — **0,0783 / −0,0326 / 0,0720 m,
> 4,26 / 0,08 / 90,95°** statt C1 (L11); `min_grip_height_m` 0,015 → **0,021** (L14).
> Der Arbeitsraum `ws_*` ist festgelegt (B10, `Safety/workspace_bounds.json`),
> Vorschläge für `observe_*`: Einrichtung §9. Die Tabellen unten zeigen den Stand
> beim Bau.

> **Diese Spec wird gelöscht**, sobald der Follower am Aufbau gelaufen ist.

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


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
Datenverträge: `docs/architektur/datenvertraege.md` (S2, S4, S9 ein; S6, S7, S8 aus).
Architekturentscheidungen: `docs/architektur/entscheidungen.md` Themen 1, 2, 6, 7.

## Regelkette

```
object_follower ──Zielpose──▶ SignalPointAttractor ──Twist──▶ IKVelocityController
       ▲                              ▲
       └──── cartesian_state ─────────┴──── robot_state_broadcaster
```

**Warum eine Pose und kein Twist:** Der Attractor rechnet
`v = K · (ziel − tcp)`. Er erzeugt Geschwindigkeit nur bei vorhandenem Abstand.
Bei einem fahrenden Ziel bleibt deshalb dauerhaft ein Abstand `v / K` bestehen —
bei 0,2 m/s und K = 5 sind das 4 cm, stabil und damit von einer
Stabilitätsprüfung nicht zu unterscheiden. Deshalb der **Vorhalt**, geführt als
**Zeit** (Nachtrag 6 / Z6):

```
zielpose = blockposition_prädiziert + v · lead_time_s
```

`v` ist die geschätzte Geschwindigkeit aus S4 (Felder 13/14). Mit
`lead_time_s = 1/K` steht der TCP im eingeschwungenen Zustand genau auf dem
Block — und das bleibt so, auch wenn die Bandgeschwindigkeit zwischen Durchläufen
leicht abweicht; eine Strecke müsste man dafür neu einmessen. Wird später der
`base_frame`-Eingang des Attractors genutzt (mitfahrender Bezugsrahmen), geht
`lead_time_s` auf 0 — **ohne Codeänderung**.

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
| `gripper_motion_done` | `Bool` | `robotiq_gripper` |
| `gripper_has_object` | `Bool` | `robotiq_gripper` |

### Outputs

| Signal | Typ | Ziel |
|---|---|---|
| `target_pose` | **`cartesian_pose`** (A2 bestätigt 14.09.2026) | `attractor`-Eingang. Bleibt leer — also unveröffentlicht —, bis die erste Zielpose das Gate passiert hat |
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

**Band**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `belt_surface_z_m` | double | 0.0536 | Höhe der Bandoberfläche in `world` (B17, gemessen) |

> Richtung und Geschwindigkeit des Bandes sind **keine** Parameter mehr: Sie
> kommen geschätzt über S4 (Nachtrag 6 / Z2).

**Regelung**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `lead_time_s` | double | 0.2 (B4) | Vorhalt als **Zeit**, multipliziert mit der geschätzten Geschwindigkeit; theoretisch `1/K`. Bei Option C auf 0. Einmessen über `err_laengs` (Nachtrag 9 / G7) |
| `latency_compensation_s` | double | 0.0 | Kamera- und Kommandolatenz |
| `max_extrapolation_s` | double | 0.2 | Deckel der Vorhersage (Sicherheit), begrenzt auch negative Alter (G8) |
| `target_timeout_s` | double | 0.5 | S4-Zeitstempel steht still → Abbruch (Gate-Prüfung 2) |

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

**Kalibrierung (aus C1, Nachtrag 6 / Z7)**

> ⚠️ **Richtung verbindlich: Flansch → Kamera.** Genau so liegen die Werte vor
> (`camera_mount_to_camera` in `Robot/Calibration_results_final.yaml`, Bezug
> `tool0` — Beweiskette in `architektur/vorgaengerprojekt-abgleich.md` §2):
> x = 0,1087 · y = −0,03436 · z = −0,05987 m, rpy ≈ (1,66° · 1,56° · **91,5°**).
>
> Die Drehung um rund 90° macht eine Richtungsverwechslung **nicht sichtbar**: Sie
> vertauscht x und y, statt nur ein Vorzeichen zu drehen. Das Ergebnis sieht
> plausibel aus und ist vollständig falsch. Die Richtung deshalb im Code
> ausschreiben und im Test gegen einen von Hand gerechneten Punkt prüfen.

| Name | Typ | Bedeutung |
|---|---|---|
| `handeye_x/y/z` | double | Hand-Auge **Flansch → Kamera**, Translation (m) |
| `handeye_roll/pitch/yaw` | double | dito, Grad |
| `flange_to_grip_point_m` | double | **0,235** — Flansch → Griffpunkt (Auflagenmitte), gemessen. ⚠️ Hieß `tool_offset_z_m` und stand fälschlich auf 0,215 — das ist der TCP der UR-Steuerung. Mit 0,215 führe die Backenspitze bei einem 25-mm-Klotz 17,5 mm ins Band. |
| `gripper_yaw_offset_deg` | double | Montagewinkel der Backen, Startwert 0 — nicht gemessen (D23) |

**Posen**

| Name | Typ | Bedeutung |
|---|---|---|
| `observe_x/y/z` | double | Beobachtungspose (B8) — Orientierung **fest senkrecht**. **Pflicht**; Default seit 24.09.2026 −0,816 / +0,35 / 0,45 (Nachtrag 13 / L15, vorher kein Default nach F3) |
| `observe_yaw_deg` | double | Gierwinkel der Beobachtungspose — Richtung der Werkzeug-x-Achse. Pflicht (F3) |
| `place_x/y/z` | double | Ablagepose über der Auffangkiste (B9) |
| `transfer_height_m` | double | Freihöhe für die Transferfahrt (D12), **0,49** — mit dem gehaltenen Klotz (Nachtrag 10 / J1); auch die Höhe, auf die der Abbruchpfad steigt |
| `place_yaw_deg` | double | 94,2 — Gier der Ablagepose (B9); `place_x/y/z` haben die B9-Werte als Default (J8) |

**Greifen**

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `min_grip_height_m` | double | 0.015 | untere Grenze der Greifhöhe — halbe Auflagenhöhe 10 mm + **5 mm Luft**, solange die 245 mm Flansch → Backenspitze (±5 mm) nicht beim ersten Testgriff bestätigt sind (B15, Nachtrag 6 / Z7) |
| `descend_speed_mps` | double | 0.15 | Sinkgeschwindigkeit — gekoppelt an `t_descend_s` des `priority_handler` (Nachtrag 10 / J2; vorher 0,05) |
| `lift_clearance_m` | double | 0.10 | Höhe, ab der das Mitfahren endet |
| `tol_along_m` / `tol_across_m` / `tol_z_m` / `tol_yaw_rad` | double | 0.005 / 0.005 / 0.01 / 0.05 (D3) | **vier getrennte** Freigabetoleranzen, **in Bandkoordinaten** wie `err_laengs`/`err_quer` (J4). Der Gierwinkel gehört zwingend dazu: Im Modus 2 kann der Roboter positionsmäßig stehen, während das Handgelenk noch dreht. |
| `stable_cycles` | int | 10 | wie lange die Toleranz gehalten werden muss (V aus dem Plan) |
| `use_block_orientation` | bool | false | Modus 2 (beliebige Lage) statt Modus 1 (gerade aufgelegt) |
| `orientation_quality_min` | double | 0.7 | Mindestgüte (S4 Feld 16), ab der der Klotzwinkel im Modus 2 verwendet wird; darunter Bandrichtung (D11, Nachtrag 6 / Z13) |

**Sicherheit**

| Name | Typ | Bedeutung |
|---|---|---|
| `ws_x_min` … `ws_z_max` | double | Arbeitsraum (B10) — **nicht** die Greifzone. **Pflicht**; Default seit 24.09.2026 = `Safety/workspace_bounds.json` (Nachtrag 13 / L15, vorher kein Default nach Nachtrag 8 / F3) |
| `max_target_jump_m` | double | Sprungerkennung im Sicherheitsgate (D15), vorläufig 0,05 |
| `robot_state_max_age_s` | double | 0,2 — älter gilt die Flanschpose als unbekannt, keine neue Zielpose (F4) |
| `pose_tolerance_m` | double | 0,01 — „angekommen" für stehende Ziele; ersetzt `is_in_range` des Attractors, das kein Signal ist (F4) |

**Timeouts** — `timeout_approach_s`, `timeout_track_s`, `timeout_grasp_s`,
`timeout_place_s`, `timeout_release_s`. Seit 4b: `timeout_approach_s` = 2 s **über
die erwartete Ankunft des Klotzes an der Zone hinaus** (Nachtrag 9 / G3),
`timeout_track_s` = 3 s, zum Einmessen des Vorhalts ≥ 10 s (G4).

## Zustandsautomat

| Zustand | Regelverhalten | Übergang | Timeout → |
|---|---|---|---|
| `WARTEN` | Beobachtungspose, **kein Vorhalt** | `has_target = 1` → `ANFAHREN` | – |
| `ANFAHREN` | Ziel auf `zone_upstream` (S4 Feld 12) begrenzt — **nur stromaufwärts** | Block in der Zone → `FOLGEN` · **Block hinter der Greifebene → `ABBRUCH`, `outcome = 3`** | `ABBRUCH` |
| `FOLGEN` | volle Regelung, Orientierung festlegen | Toleranz für `stable_cycles` gehalten **und Block vor der Greifebene** → `ABSENKEN` · **Block hinter der Greifebene → `ABBRUCH`, `outcome = 3`** | `ABBRUCH` |
| `ABSENKEN` | wie `FOLGEN` + `vz` abwärts, **Korrektur eingefroren** | Greifhöhe erreicht → `GREIFEN` | Abweichung wächst → **auf Beobachtungshöhe steigen**, zurück zu `FOLGEN` |
| `GREIFEN` | **reines Mitfahren**, Greifer schließt | `has_object` → `HEBEN` | `ABBRUCH`, `outcome = 1` |
| `HEBEN` | Mitfahren **mit der letzten Bandgeschwindigkeit** bis `lift_clearance_m`, dann steigen bis `transfer_height_m` — keine Blockvorhersage mehr, der Klotz ist im Greifer | erreicht → `ABLEGEN` | `ABBRUCH` |
| `ABLEGEN` | Ablagepose, **kein Vorhalt** | angekommen (`pose_tolerance_m`, F4) → `LOESEN` | `ABBRUCH` |
| `LOESEN` | Greifer auf, `picked_id` mit `outcome = 0` | fertig → `WARTEN` | **`WARTEN` mit Warnung**, nicht `ABBRUCH` (Nachtrag 10 / J6) |
| `ABBRUCH` | **ohne Klotz:** TCP halten, Greifer auf, senkrecht hoch, Beobachtungspose · **mit Klotz (`has_object`):** Greifer bleibt zu, senkrecht hoch, Ablagepose, dort öffnen, `picked_id` mit `outcome = 0` | → `WARTEN` | – |

**Zur Begrenzung in `ANFAHREN`:** Begrenzt wird ausschließlich die
**stromaufwärtige Längskoordinate** gegen `zone_upstream` aus S4 Feld 12. Die
Querposition bleibt unbegrenzt — sie erzeugt gerade den Abfangkurs, weil der
Roboter am Zonenrand auf der Spur des Blocks wartet. Stromabwärts wird **nicht**
begrenzt, sonst bricht ein fast gelungener Griff ab, sobald der Block beim Greifen
die Zonengrenze überschreitet (P4). Nach oben und unten schützt der
Arbeitsraum-Clamp des Sicherheitsgates. Die Greifzone selbst gehört weiterhin
allein dem `priority_handler`; der Follower hält keine Zonenparameter
(`entscheidungen.md`, Nachtrag 3 / N3).

**Wie gebaut (4b, Nachtrag 9):** gefolgt wird auf `observe_z` (G1).
`ANFAHREN` → `FOLGEN`, sobald der vorhergesagte Klotz `zone_upstream` erreicht (G3).
In `ANFAHREN` und `FOLGEN` ist eine gedeckelte Zielpose ein Abbruch (G5). Jeder
Versuch endet mit `picked_id`; ein Abbruch vor dem Griff trägt `outcome = 4` (G2),
und der Follower nimmt eine abgeschlossene ID nicht wieder an.

**Die Längskoordinate** für `zone_upstream` und die Greifebene rechnet der Follower
mit `contracts.along_belt(x, y, (vx, vy))`, die Richtung aus S4 Feld 13/14 derselben
Nachricht — dieselbe Funktion wie der `priority_handler` (Nachtrag 7 / H4).

**Start im Zustand `ABBRUCH`.** Beim Anwendungsstart steht der Roboter irgendwo,
und der Attractor führe geradlinig zur Beobachtungspose — je nach Ausgangslage
durch das Band. Der Abbruchpfad macht bereits das Richtige: senkrecht hoch, dann
Beobachtungspose. Ein Codepfad, zwei Zwecke.

**Der Abbruchpfad ohne Klotz, wie gebaut (4a, Nachtrag 8 / F4):** aus der
Einstiegspose senkrecht hoch auf `max(z, transfer_height_m)`, Orientierung
beibehalten; angekommen (`pose_tolerance_m`) → `WARTEN`, dessen Ziel die
Beobachtungspose ist. Nach unten fährt der Pfad nie. Das Warten auf das Öffnen des
Greifers kommt mit 4d — nötig nur, wenn der Follower selbst geschlossen hat.
Ohne frischen `robot_state` gibt der Follower keine neue Zielpose aus.

**Abbruchgründe bis der Klotz gehalten wird** (`ANFAHREN` bis `GREIFEN`):
`has_target = 0` · Ziel-ID aus `target` verschwunden · unplausibler Sprung der
Regelabweichung · Zeitstempel der Eingangssignale steht still.

**Ab `has_object`** (`HEBEN`, `ABLEGEN`, `LOESEN`) gelten diese nicht mehr — der
Klotz *soll* vom Band verschwinden. Abgebrochen wird dann nur noch bei
Zeitüberschreitung, Sicherheitsgate, stehendem Roboterzustand oder wenn
`has_object` unerwartet wegfällt (`outcome = 2`). Neue Ziele nimmt der Follower
erst in `WARTEN` an.

> ⚠️ **Korrigiert (Nachtrag 6 / Z12).** Hier stand „aus jedem Zustand", und
> `ABBRUCH` öffnete den Greifer immer. Nach jedem gelungenen Griff verschwindet die
> ID des gehobenen Klotzes aus `tracks` — der Follower hätte ihn deshalb jedes Mal
> wieder fallen lassen.

**Die Greifebene (S4 Feld 15, Nachtrag 6 / Z11) ist ein Tor, kein Abbruch mitten
im Griff.** Sie gilt nur, **bevor** abgesenkt wird — in `ANFAHREN` und `FOLGEN`,
auch nach einem Rücksprung aus `ABSENKEN` (F3). Überschreitet der Block sie dort,
ist der Greifprozess vor dem Zonenende nicht mehr zu schaffen: `ABBRUCH` mit
`outcome = 3`, „verpasst". Hat das Absenken begonnen, spielt sie keine Rolle mehr —
der Griff wird zu Ende geführt, notfalls über das Zonenende hinaus (P4).

> Ohne diese Ebene hatte `FOLGEN` nur einen Zeitablauf, aber keine Positionsgrenze.
> Kam der Roboter spät an, begann er womöglich einen Griff, der vor dem Zonenende
> nicht mehr fertig werden konnte.

## Kernlogik

### Prädiktion

```
p(t) = p_ziel + v · (t − t_ziel + latency_compensation_s)
```
mit `(t − t_ziel)` gedeckelt auf `max_extrapolation_s`. `v = (vx, vy)` aus S4
Feld 13/14 — die in `vectoring` geschätzte und gepoolte Bandgeschwindigkeit
(Nachtrag 6 / Z2). Eine Bandrichtung als eigene Größe gibt es nicht mehr; sie
steckt im Vektor.

Die Prädiktion dient **nicht primär** der Latenzkompensation, sondern der
Glättung: `base_cam` liefert ~30 Bilder/s, der Follower läuft mit 100 Hz. Ohne
Extrapolation stünde die Zielpose zwischen zwei Bildern still und spränge dann —
bei 0,2 m/s in 6,7-mm-Stufen. Der Attractor sähe eine Treppe statt einer Fahrt.

### Kameramischung — Korrektur filtern, nicht Position mischen

```
x_korr, y_korr  = (x, y) · (z_band − blockhoehe) / z_band     ← Höhenkorrektur, ZUERST
p_robotcam_welt = T_world_flansch(t_bild) · T_flansch_kamera · (x_korr, y_korr, …)
korrektur       = p_robotcam_welt − p_prädiktion
zielpunkt       = p_prädiktion + w · korrektur_gefiltert
```

**Die Höhenkorrektur ist Pflicht, kein Feinschliff.** `robot_cam` projiziert den
Blobmittelpunkt mit der **Banddistanz** zurück, der Punkt liegt aber auf der
**Oberseite** des Klotzes — beide Werte sind dadurch um `z_band/(z_band − h)` zu
groß. Weil die Kamera rund 114 mm seitlich neben der Flanschachse sitzt, steht der
Klotz im eingeschwungenen Zustand gerade **nicht** im Bildzentrum, wo der Fehler
null wäre, sondern weit außen, wo er maximal ist: 6…39 mm je nach Klotzhöhe und
Arbeitsdistanz, **je Klotz verschieden**, und unterhalb von `max_correction_m`,
also von der Identitätsprüfung nicht abgefangen. `blockhoehe` kommt aus S4
Feld 10, `z_band` aus S2 Feld 4. Vollständige Herleitung: `entscheidungen.md`,
Nachtrag 3 / N1.

> **Reihenfolge:** Die Skalierung wirkt auf die **rohe Messung**, also vor dem
> Korrekturfilter und damit vor dem Einfrieren beim Übergang
> `FOLGEN → ABSENKEN` (F2). Sonst friert der unkorrigierte Wert ein und der
> Fehler wird über den gesamten Absenkvorgang konserviert.

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
                                          and ori_quality ≥ orientation_quality_min
                                        ? block_orientierung
                                        : atan2(vy, vx) )
```

`ori_quality` ist S4 Feld 16. Ist die Orientierung eines Klotzes unbrauchbar —
verrauscht, teilverdeckt, halb außerhalb der ROI —, greift der Roboter auch im
Modus 2 entlang der Bandrichtung (Nachtrag 6 / Z13).

`atan2(vy, vx)` ist die Richtung der geschätzten Bandgeschwindigkeit aus S4 — im
Modus 1 richtet sich der Greifer also an der **gemessenen** Laufrichtung aus, nicht
an einem eingetragenen Winkel (Nachtrag 6 / Z2).

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
schützt das Band: Die Greifauflagen sind 20 mm hoch (B15); bei einem flachen Block
läge ihre Unterkante sonst unter der Bandoberfläche. Mit `min_grip_height_m =
0,015` bleiben 5 mm Luft — genug für die ±5 mm Unsicherheit der Flanschmessung.

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
zielpose_flansch = zielpose_greifpunkt + [0, 0, flange_to_grip_point_m]
```

Eine einzige Zahl, weil der Greifer immer senkrecht nach unten zeigt und der
Greifpunkt auf der Flanschachse liegt (bestätigt, M6: x = y = 0, keine
Verdrehung). Eine Drehung um die Hochachse ändert den Versatz nicht.

> **Geklärt (A7/A8):** Der Controller regelt den **Flansch**, der Greifer steht
> nicht im URDF. Der Versatz ist deshalb zwingend.
>
> ⚠️ **Drei Abstände nicht verwechseln** (Nachtrag 6 / Z7): 215 mm ist der TCP der
> UR-Steuerung und dient nur der Umrechnung fremder Werte. Hier gehört der
> **Griffpunkt mit 235 mm** hin. Die 245 mm zur Backenspitze gelten für
> Höhenmessungen am Aufbau.

### Selbstüberwachung des Vorhalts

Der Follower kennt die ausgegebene Zielpose und die gemeldete TCP-Pose. Im
eingeschwungenen Zustand muss deren Differenz dem Vorhalt `v · lead_time_s`
entsprechen. Dauerhafte Abweichung → Warnung ins Log.

Grund: `lead_time_s` hängt am Gain des Attractors. Wird der im Studio verändert
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
| `on_activate` | Zustand `ABBRUCH`, Ringpuffer leeren, `seq = 0` („kein Versuch" — der erste abgeschlossene Versuch trägt 1, Nachtrag 7 / H5), Greifer öffnen |
| `on_deactivate` | Greifer öffnen; letzte Zielpose bleibt beim Attractor stehen (der Roboter fährt sie zu Ende an und hält — begrenzt, weil das Gate sie bereits im Arbeitsraum gehalten hat) |

## Fehlerbehandlung

- Jeder Zustand hat einen Timeout mit definiertem Rückfall
- Fehlgriff (`motion_done` ohne `has_object`) → `ABBRUCH`, `outcome = 1`
- `LOESEN` **nicht** über `has_object = 0` verlassen: Der Robotiq-Status `gOBJ`
  meldet auch ein Objekt, auf das der Greifer **beim Öffnen** trifft, `has_object`
  kann dort also flackern. Übergang über den Abschluss der Öffnungsbewegung
  (`motion_done = 1` nach dem Öffnen-Kommando) bzw. den Timeout. ⚠️ Hier stand
  vorher `is_closed = 0` — unter der Vertragssemantik „Bewegung abgeschlossen"
  hieße das *Bewegung läuft noch*, also genau das Gegenteil.
- Verlorenes Objekt → `ABBRUCH`, `outcome = 2`
- Greifebene überschritten, bevor abgesenkt wurde → `ABBRUCH`, `outcome = 3`
- Nach jedem Abbruch wird `picked_id` gesendet, damit der `priority_handler` das
  Objekt als erledigt behandelt und das nächste wählt

## Dateien

- **Neu:** `roboter_tetris/follower_logic.py` — Zustandsautomat, Gate, Ringpuffer, ohne ROS
- **Neu:** `roboter_tetris/object_follower.py` — Komponentenschale
- **Neu:** `component_descriptions/roboter_tetris_object_follower.json`
- **Neu:** `test/python_tests/test_follower_logic.py`, `test_object_follower.py`
- **Geändert:** `setup.cfg`, `contracts.py` (Zustandscodes S8)

## Umsetzung in vier Stufen

| Stufe | Umfang | Prüft |
|---|---|---|
| **4a** ✅ | Gerüst, Sicherheitsgate, TCP-Ringpuffer; aktiv nur `WARTEN` und `ABBRUCH` ohne Klotz. `ABLEGEN` wandert nach 4d (Nachtrag 8 / F2), der Werkzeugversatz nach 4b — erst dort gibt es Klotzpositionen | Attractor-Anbindung, Startverhalten, Gate |
| **4b** ✅ | `ANFAHREN` + `FOLGEN` mit `w = 0`, Gierwinkel (Modus 1 und 2), Abbruchgründe, `picked_id` mit `outcome = 4` | Prädiktion, Vorhalt (hier `lead_time_s` kalibrieren) |
| **4c** ✅ | Kameramischung, Rampe, Rückfall, Identitätsprüfung | Hand-Auge, Korrekturfilter — **Verbesserung, parallel zu 4d** |
| **4d** ✅ | `ABSENKEN`, `GREIFEN`, `HEBEN`, `ABLEGEN`, `LOESEN`, `picked_id`, `ABBRUCH` mit Klotz, Warten auf das Öffnen des Greifers | vollständiger Zyklus, erst Modus 1, dann Modus 2 — **baut auf 4b auf, zunächst mit `w = 0`** |

> **Reihenfolge seit Nachtrag 6 / Z10:** 4d setzt 4b voraus, nicht 4c. Der
> Greifzyklus läuft zuerst mit der Basiskamera allein; die Roboterkamera wird als
> Verbesserung aufgeschaltet. Ein erneutes Scheitern von B6 blockiert damit nicht
> mehr den ganzen Greifvorgang.

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
