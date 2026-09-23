# Design: Priority Handler Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** ✅ **umgesetzt 21.09.2026**, lokal getestet (29 Tests, darunter ein
Ende-zu-Ende-Lauf mit `fake_objects` → `vectoring` → Auswahl), in AICA noch nicht
gelaufen. Die Rechenlogik liegt in `target_selection.py` (ohne ROS).

> **Diese Spec wird gelöscht**, sobald die Komponente am Aufbau gelaufen ist —
> gemeinsam mit den Specs von `vectoring` und den Bestandskomponenten. Die beim Bau
> getroffenen Entscheidungen stehen in `entscheidungen.md`, **Nachtrag 7**.

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


## Zweck

Wählt aus den geglätteten Objektzuständen das Objekt aus, das als nächstes
gegriffen wird, und hält diese Wahl fest, bis sie erledigt ist.

Die Auswahlregel des ursprünglichen Plans — "das nächste Objekt zum nicht
pickbaren Bereich" — bleibt erhalten, bekommt aber eine **Erreichbarkeitsprüfung
davor**. Ohne sie wählt das System systematisch das Objekt mit der geringsten
verbleibenden Zeit und jagt damit Blöcken hinterher, die nicht mehr zu schaffen
sind.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`. Datenverträge: `docs/architektur/datenvertraege.md`
(S3 und S7 ein, S4 und S5 aus).

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `PriorityHandler` |
| Basisklasse | `LifecycleComponent` |
| Python-Datei | `roboter_tetris/priority_handler.py` |
| Registrierung | `roboter_tetris::PriorityHandler = roboter_tetris.priority_handler:PriorityHandler` |
| UI-Anzeigename | `Priority Handler` |
| Rate | 100 Hz |

## Schnittstellen

### Inputs

| Signal | Typ | Inhalt |
|---|---|---|
| `tracks` | `Float64MultiArray` | S3 aus `vectoring` |
| `picked_id` | `Float64MultiArray` | S7 aus `object_follower` |
| `robot_state` | `cartesian_state` | `robot_state_broadcaster` — liefert den Anfahrweg für die Erreichbarkeitsprüfung |

> **Kein Zyklus.** `cartesian_state` kommt aus der Hardware, nicht aus dem
> Regelpfad, und der `object_follower` hört es ohnehin ab — es ist eine
> zusätzliche Abzweigung eines vorhandenen Signals. Die Entkopplung aus Thema 4
> betrifft die Rückmeldung des **Follower-Zustands**, und die bleibt unberührt
> (`entscheidungen.md`, Nachtrag 3 / N4).

### Outputs

| Signal | Typ | Inhalt |
|---|---|---|
| `target` | `Float64MultiArray` | S4 — feste Länge **17** |
| `not_pickable` | `Float64MultiArray` | S5 — nur für `data_tracker` |

### Predicates

| Name | Bedeutung |
|---|---|
| `has_target` | ein Ziel ist gewählt |
| `zone_empty` | **kein Klotz innerhalb der Greifzone**, gleich welcher Status — eine räumliche Aussage. „Keine Kandidaten" wäre nach der Auswahl dasselbe wie `not has_target` (Nachtrag 7 / H5) |
| `is_zone_feasible` | Greifebene liegt stromabwärts von `zone_upstream` — sonst ist die Zone für die geschätzte Bandgeschwindigkeit zu kurz (Nachtrag 6 / Z11). Falsch, solange keine Bandschätzung vorliegt |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `zone_x_min` / `zone_x_max` | double | −0.95 / −0.68 | Greifzone in `world` — ~~Platzhalter bis B19~~ bestätigt 23.09.2026 (Nachtrag 13 / L14), im Robotersystem: das angetastete Band x = −0,70 … −0,93 (Nachtrag 8 / F1) |
| `zone_y_min` / `zone_y_max` | double | −0.22 / 0.40 (bis 23.09.: −0.45 / 0.05) | **Vier Grenzen** — im Plan fehlte `y_min`. **Platzhalter bis B19**, geschätzt um den angetasteten Abschnitt. Vorher lagen die Platzhalter im System der Basiskamera (x ≈ +0,8) — auf der anderen Seite der Basis (B23) | ⚠️ Seit Nachtrag 13 / L4 liegt die Zone außerhalb des Bildes der Basiskamera, etwa y 0,30 … −0,30; das Band endet bei −0,375.
| `attractor_v_max_mps` | double | 0.25 | max. Fahrgeschwindigkeit, für die Anfahrtsschätzung — der **kleinere** Wert aus Attractor und IK-Controller; bindend ist meist der IK-Controller (A1) |
| `attractor_gain` | double | 5.0 | `K`, für die Einschwingzeit `3/K` — muss zu `linear_gains` des Attractors passen (Einrichtung §2) |
| `t_descend_s` | double | 2.0 | Absenkzeit — geht in die Greifebene ein (D22). Gekoppelt: `(observe_z − Greifhöhe) / descend_speed_mps` des Followers + Einschwingen (Nachtrag 10 / J2; vorher 1,0) |
| `t_grasp_s` | double | 1.0 | Greifzeit bis `motion_done` — geht in die Greifebene ein (D22) |
| `t_lift_s` | double | 0.5 | Hebezeit bis `lift_clearance_m`, so lange fährt der Roboter noch mit (D22) |
| `grasp_time_margin` | double | 1.2 | Aufschlag auf die Zeit des Greifprozesses bei der Lage der Greifebene |
| `reach_safety_factor` | double | 1.5 | Sicherheitsfaktor im Erreichbarkeitskriterium |
| `min_graspable_height_m` | double | 0.030 | flachere Blöcke gar nicht erst anfahren — `2 · min_grip_height_m` des Followers (B15, Nachtrag 6 / Z7) |
| `max_gripper_opening_m` | double | – (B16) | Öffnungsweite des Greifers |
| `gripper_margin_m` | double | 0.01 | Reserve zur Öffnungsweite |

> Die Greifzone ist **nicht** der Sicherheitsraum. Sie ist eine
> Aufgabenentscheidung und liegt innerhalb des Arbeitsraums, den der
> `object_follower` überwacht. Im ursprünglichen Plan waren beide vermischt.

## Kernlogik

### Erreichbarkeit

```
t_verfügbar = (greifebene_längs − position_längs) / |v_belt|    ← Frist für den Beginn des Absenkens

t_benötigt  = abstand / attractor_v_max_mps        ← Fahrt, abstand = TCP (robot_state) → Klotz
            + 3 / attractor_gain                   ← Einschwingen des Attractors

kandidat ⟺ t_verfügbar > reach_safety_factor · t_benötigt
```

> ⚠️ **Geändert (Nachtrag 6 / Z11):** Gemessen wird bis zur **Greifebene**, nicht
> bis zum Zonenende, und `t_descend_s + t_grasp_s` stehen nicht mehr in
> `t_benötigt` — sie stecken jetzt in der Lage der Ebene. Die Rechnung trennt damit
> das **Anfahren** (bis zur Ebene, hier geprüft) vom **Greifprozess** (Ebene bis
> Zonenende, durch die Lage der Ebene gesichert). Der Sicherheitsfaktor wirkt nur
> noch auf das, was wirklich schwankt: den Anfahrweg.

`abstand` ist der **waagerechte** Weg vom Flansch zum **Anfahrpunkt** (Nachtrag 7 /
H1): Der Flansch fährt in `ANFAHREN` und `FOLGEN` auf Beobachtungshöhe, die Höhe
ändert sich vor `ABSENKEN` nicht. Anfahrpunkt ist der Klotz selbst — liegt er noch
stromaufwärts von `zone_upstream`, der Punkt auf seiner Spur an `zone_upstream`,
denn dort wartet der Follower.

**Woher `v_belt` kommt** (Nachtrag 6 / Z2): aus dem **S3-Kopf** — die in
`vectoring` gepoolte, geschätzte Bandgeschwindigkeit. Es gibt keinen
Geschwindigkeitsparameter mehr. Ist `n_pool = 0` (noch kein Klotz final), gibt es
keinen Schätzwert — dann ist **nichts** erreichbar und es wird kein Ziel gewählt.
Da nur finale Tracks Kandidaten sind und jeder finale Track zum Pool beiträgt,
tritt der Fall ohnehin nur vor dem ersten eingeschwungenen Klotz auf.

`zonenende_längs` ist zugleich die Stelle, ab der ein nicht gegriffener Klotz
verloren ist — er fällt danach am Bandende herunter (Nachtrag 6 / Z5). `t_verfügbar`
ist damit genau die berechenbare Restzeit bis zum Abwurf.

Der Anfahrweg ist nicht entbehrlich: Über Bandbreite und Zonenlänge schwankt der
Anfahrweg realistisch zwischen 0,3 und 0,8 m — rund eine Sekunde bei einem Budget
von wenigen Sekunden. Nebeneffekt in die richtige Richtung: Steht der Roboter
gerade weit weg, etwa mitten im Abbruchpfad, gilt korrekt nichts als erreichbar,
bis er zurück ist.

Der Term `3/K` ist wesentlich: Der Point Attractor nähert sich exponentiell, nach
etwa drei Zeitkonstanten ist er praktisch da. Bei K = 5 sind das 0,6 s. Ohne
diesen Term plant man mit einer Ankunft, die so nie eintritt.

### Greifbarkeit

| Prüfung | Bedingung |
|---|---|
| nicht zu flach | `height ≥ min_graspable_height_m` |
| passt in den Greifer | **Diagonale** `√(length² + width²) ≤ max_gripper_opening_m − gripper_margin_m` |

> ⚠️ **Geändert beim Bau (Nachtrag 7 / H2).** Hier stand „Abmessung quer zur
> Backenrichtung, aus Blockorientierung und kommandiertem Gierwinkel". Den
> Gierwinkel entscheidet aber der Follower (Modus, `orientation_quality_min`,
> `gripper_yaw_offset_deg`) — ihn hier nachzubilden wäre die Doppelpflege aus
> Befund P3. Die Diagonale ist die größte Ausdehnung in jeder Richtung und gilt
> damit für jeden Gierwinkel. Sie macht auch die frühere Sonderregel für fast
> quadratische Klötze (Verhältnis über 0,92 → `max(length, width)`) entbehrlich:
> Vertauschte Achsen ändern die Diagonale nicht. Der Preis: Bei 117 mm fallen erst
> Klötze wie 70 × 100 mm heraus, der Referenzklotz 50 × 100 hat 112 mm.

> ⚠️ **Die Prüfung gilt für die geglätteten Werte aus S3, nicht für eine
> Einzelmessung.** Am ruhenden 50 × 50-Klotz streut das Seitenverhältnis bildweise
> von 0,727 bis 0,999 (299 Messungen, 15.09.2026) — bildweise wäre die Schwelle
> wertlos, egal wo man sie ansetzt. Über das Mittelungsfenster von `vectoring`
> liegt sie stabil bei 0,95. Einzelheiten: `entscheidungen.md`, Nachtrag 4 / M5.

Das verhindert die frustrierendste Fehlerart: sauber anfahren, greifen, passt
nicht.

### Kandidatenfilter

Ein Track ist Kandidat, wenn **alle** zutreffen: `status = 0` · seine **Spur
kreuzt die Greifzone** · nicht in der Gepickt-Liste · greifbar · erreichbar (damit
auch: noch vor der Greifebene).

> ⚠️ **Geändert (Nachtrag 6 / Z11, E10).** Hier stand „in der Greifzone". Ein Ziel
> darf aber **stromaufwärts** der Zone gewählt werden, sonst griffe die Begrenzung
> auf `zone_upstream` nie. Geprüft wird deshalb die Spur: Die Position, an der der
> Klotz die Greifebene erreicht, muss in der Zone liegen (Nachtrag 7 / H3). Ein
> Klotz neben der Zone wird so nie gewählt, einer davor schon.

### Auswahl

Unter den Kandidaten das **dringendste** — kleinstes `t_verfügbar`. Das ist die
Regel aus dem ursprünglichen Plan, nur eben angewandt auf die tatsächlich
erreichbaren Objekte.

### Ziel-Lock

Einmal gewählt, bleibt gewählt. Ein auftauchendes "besseres" Objekt löst **keinen**
Wechsel aus — das verwirft investierte Zeit und bringt bei zwei bis drei Blöcken
nichts.

Zurückgezogen (`has_target = 0`) wird nur bei:

| Grund | Auslöser |
|---|---|
| gepickt **oder verpasst** | `picked_id` mit dieser ID — jedes `outcome`, auch 3 („Greifebene überschritten, bevor abgesenkt wurde", Nachtrag 6 / Z11) |
| verloren | ID verschwindet aus `tracks` |
| neu einschwingend | Status fällt auf 3 zurück — anhaltende Abweichung, Schätzung neu gestartet (Nachtrag 6 / Z3) |

**Erreichbarkeit gehört bewusst nicht dazu.** Sie ist Auswahl-, kein
Abbruchkriterium. Sonst würde ein fast erfolgreicher Griff abgebrochen, sobald
der Block beim Greifen die Zonengrenze überschreitet. Die Greifzone ist enger als
der Sicherheitsraum; der Roboter darf dem Block ein Stück darüber hinaus folgen.

Nebeneffekt: Es braucht keine Rückmeldung des Follower-Zustands — die Entkopplung
aus dem Komponentenschnitt bleibt erhalten.

### Füllen von S4

Aus dem gewählten Track (S3) werden Lage, Abmessungen, Orientierung und
**Orientierungsgüte** (S3 Feld 13 → S4 Feld 16) übernommen. Die Geschwindigkeit
(Felder 13/14) kommt dagegen aus dem **S3-Kopf**, also aus dem Pool — nicht aus
der eigenen Schätzung des Tracks. `contracts.pack_target` nimmt sie deshalb
getrennt entgegen.

### Greifebene (S4 Feld 15)

Die letzte Längsposition, von der aus der gesamte Greifprozess noch vor dem
Zonenende durchgeführt werden kann (Nachtrag 6 / Z11):

```
greifebene_längs = zonenende_längs − |v_belt| · (t_descend_s + t_grasp_s + t_lift_s) · grasp_time_margin
```

Längs heißt entlang der geschätzten Bandrichtung, wie bei `zone_upstream`. Die Ebene
wandert mit der geschätzten Geschwindigkeit — bei schnellerem Band rückt sie weiter
stromaufwärts. Bei `n_pool = 0` gibt es keine Geschwindigkeit und damit keine
Ebene; dann wird ohnehin kein Ziel gewählt.

Wird im `target`-Satz auch bei `has_target = 0` mitgegeben. Der Follower beginnt
`ABSENKEN` nur, solange der Block davor liegt, und bricht mit `outcome = 3` ab,
wenn er sie vorher überschreitet. **Ist das Absenken begonnen, gilt sie nicht
mehr** — ein laufender Griff wird zu Ende geführt.

**Plausibilitätsprüfung der Konfiguration:** Liegt `greifebene_längs` stromaufwärts
von `zone_upstream`, ist die Greifzone für diese Bandgeschwindigkeit zu kurz — es
wäre nie ein Klotz greifbar. Dann gedrosselt warnen und `is_zone_feasible = false`
(neues Predicate). Das gibt B19 eine Untergrenze für die Zonenlänge.

### `zone_upstream` (S4 Feld 12)

Der `priority_handler` gibt die **stromaufwärtige Längsgrenze** der Greifzone im
`target`-Satz mit — auch bei `has_target = 0`, solange eine Bandschätzung vorliegt.

**Längskoordinate** (Nachtrag 7 / H4): `s = contracts.along_belt(x, y, v_belt)` —
Projektion auf die Pool-Richtung, Ursprung `world`, wächst stromabwärts. Die Zone
ist ein achsparalleles Rechteck in `world`; als Grenzen gelten die beiden
**mittleren** der vier Eckprojektionen. Bei einer Bandrichtung entlang einer Achse
sind das genau die Zonenkanten, bei schräger Richtung die inneren, also
konservativen Werte. Der `object_follower` begrenzt in
`ANFAHREN` seine Zielpose darauf und erzeugt damit den Abfangkurs.

Nur diese eine Grenze wird übertragen: Quer darf nicht begrenzt werden (der
Roboter wartet am Zonenrand auf der Spur des Blocks), stromabwärts ebenfalls nicht
(P4 — der Roboter muss dem Block über die Zonengrenze hinaus folgen dürfen), und
die Höhe deckt der Arbeitsraum-Clamp des Follower ab. Die Greifzone bleibt damit
vollständig in **einem** Besitz (`entscheidungen.md`, Nachtrag 3 / N3).

### Greifzone und Messregion des Trackers

**Die Greifzone muss innerhalb der Messregion von `base_cam` liegen**
(`track_velocity_region_y_min` / `_max`). Außerhalb der Region löscht der Tracker Tracks nicht bei ausbleibender Detektion,
sondern erst an den Bandgrenzen. In der Greifzone soll ein verschwundenes Ziel aber
innerhalb von drei Bildern auffallen. Beide
Grenzen werden in **B19 gemeinsam** festgelegt.

> ⚠️ **Begründung geändert (Nachtrag 6 / Z4, Z5).** Hier stand, dass der Tracker
> außerhalb der Region die Längsposition *koppelt* und ein Phantom durch die Zone
> liefe. Das Koppeln entfällt mit Umsetzungsplan 2.4, und Phantome gibt es nicht:
> Klötze fallen nur am Bandende herunter und werden nie von Hand entnommen. Die
> Schlussfolgerung bleibt, nur aus dem Grund oben.

### `not_pickable`

IDs, die die **Greifebene** ungegriffen überschritten haben — ab dort sind sie nicht
mehr zu holen und fallen später am Bandende herunter (Nachtrag 6 / Z5, Z11). Das
aktuelle Ziel ist davon ausgenommen, solange es gewählt ist. Reine Buchhaltung
für die Anzeige, nicht im Regelpfad.

Gebaut als **aktuelle** Liste aus den Tracks des Zyklus, jeder Status — keine
Historie. Das Festhalten übernimmt der `data_tracker` (`out_of_bounds`).

> Hier stand zusätzlich „oder unplausibel sind". Die Plausibilitätsstatus sind
> entfallen (Nachtrag 6 / Z3), damit auch dieser Fall. Erst so ist die Zuordnung
> im `data_tracker` (`not_pickable` → `out_of_bounds`) sauber.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen; prüfen, dass die Greifzone nicht leer ist |
| `on_activate` | Lock lösen, Gepickt-Liste leeren, `seq`-Merker zurücksetzen, `has_target = False` |
| `on_deactivate` | Lock lösen. **Ausgegeben wird das nicht mehr** — außerhalb von `ACTIVE` publiziert AICA nichts; der Follower merkt es am stehenden S4-Zeitstempel |

## Fehlerbehandlung

- Inkonsistente Eingangslänge → Zyklus überspringen, gedrosselt loggen
- Bleibt der S3-Zeitstempel **0,5 s** stehen → Ziel zurückziehen. Ein Ziel auf
  Basis toter Daten ist gefährlicher als kein Ziel. Gewählt wird erst wieder mit
  frischen Daten.
- `robot_state` fehlt → nichts Neues wählen (Erreichbarkeit unbekannt), ein
  bestehendes Ziel bleibt. Gedrosselt warnen.
- Ohne Bandschätzung (`n_pool = 0` oder Band unter 0,01 m/s) → kein Ziel,
  S4-Felder 12–15 null.
- `picked_id` zwischen zwei S3-Bildern → sofort freigeben, gewählt wird mit dem
  nächsten Bild. `seq = 0` heißt „Follower ohne Versuch" und löst nichts aus.
- Gewählt und zurückgezogen wird **mit Grund im Log** — dafür ist die Abnahme am
  Aufbau ausgelegt. Nicht greifbare Klötze werden je ID einmal gemeldet.

## Dateien

- **Neu:** `roboter_tetris/target_selection.py` — die gesamte Auswahllogik, ohne ROS
- **Neu:** `roboter_tetris/priority_handler.py` — dünne Komponentenschale
- **Neu:** `component_descriptions/roboter_tetris_priority_handler.json`
- **Neu:** `test/python_tests/test_target_selection.py`, `test_priority_handler.py`
- **Geändert:** `setup.cfg`, `contracts.py` (`along_belt`)

## Testbarkeit

Auswahllogik ROS-frei auslegen. Ohne Hardware prüfbar: Erreichbarkeit an der
Grenze, Greifbarkeit (zu flach, zu breit), Auswahl des dringendsten unter
mehreren, Lock hält bei neuem "besserem" Objekt, alle drei Rückzugsgründe,
Verhalten bei stehendem Eingangszeitstempel.

**Am Aufbau vollständig bei stehendem Roboter prüfbar:** Blöcke auflegen und im
Log verfolgen, welches Ziel wann gewählt und aus welchem Grund zurückgezogen
wird.

## Bewusste YAGNI-Entscheidungen

- Keine gemerkte Listenposition zur Suchbeschleunigung (stand so im Plan). Bei
  2–3 Objekten ist eine Suche über die ID kostenlos; die Zustandshaltung wäre
  reine Fehlerquelle.
- Kein Zielwechsel zugunsten eines besseren Objekts.
- Keine Warteschlange mehrerer vorgeplanter Ziele.
- Keine Rückmeldung des Follower-Zustands.
