# Design: Priority Handler Component

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
| `target` | `Float64MultiArray` | S4 — feste Länge **13** (Feld 12 = `zone_upstream`) |
| `not_pickable` | `Float64MultiArray` | S5 — nur für `data_tracker` |

### Predicates

| Name | Bedeutung |
|---|---|
| `has_target` | ein Ziel ist gewählt |
| `zone_empty` | keine Kandidaten in der Greifzone |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `zone_x_min` / `zone_x_max` | double | – (B19) | Greifzone in `world` |
| `zone_y_min` / `zone_y_max` | double | – (B19) | **Vier Grenzen** — im Plan fehlte `y_min` |
| `belt_heading_deg` | double | – (B1) | Laufrichtung |
| `belt_speed_mps` | double | – (B1) | Bandgeschwindigkeit |
| `attractor_v_max_mps` | double | – (A1) | max. Fahrgeschwindigkeit, für die Anfahrtsschätzung |
| `attractor_gain` | double | – (A1) | `K`, für die Einschwingzeit `3/K` |
| `t_descend_s` | double | 1.0 | pauschale Absenkzeit |
| `t_grasp_s` | double | 1.0 | pauschale Greifzeit |
| `reach_safety_factor` | double | 1.5 | Sicherheitsfaktor im Erreichbarkeitskriterium |
| `min_graspable_height_m` | double | – (B15) | flachere Blöcke gar nicht erst anfahren |
| `max_gripper_opening_m` | double | – (B16) | Öffnungsweite des Greifers |
| `gripper_margin_m` | double | 0.01 | Reserve zur Öffnungsweite |

> Die Greifzone ist **nicht** der Sicherheitsraum. Sie ist eine
> Aufgabenentscheidung und liegt innerhalb des Arbeitsraums, den der
> `object_follower` überwacht. Im ursprünglichen Plan waren beide vermischt.

## Kernlogik

### Erreichbarkeit

```
t_verfügbar = (zonenende_längs − position_längs) / belt_speed_mps

t_benötigt  = abstand / attractor_v_max_mps        ← Fahrt, abstand = TCP (robot_state) → Klotz
            + 3 / attractor_gain                   ← Einschwingen des Attractors
            + t_descend_s + t_grasp_s

kandidat ⟺ t_verfügbar > reach_safety_factor · t_benötigt
```

`abstand` ist der Weg von der **aktuellen TCP-Position** (aus `robot_state`) zum
Klotz. Der Term ist nicht entbehrlich: Über Bandbreite und Zonenlänge schwankt der
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
| passt in den Greifer | Abmessung quer zur Backenrichtung `≤ max_gripper_opening_m − gripper_margin_m` |

Welche Abmessung quer zur Backenrichtung liegt, folgt aus der Blockorientierung
und dem kommandierten Gierwinkel — gilt damit in beiden Orientierungsmodi.

**Ausnahme für fast quadratische Klötze:** Liegt `min(length, width) / max(…)`
über **0,85**, ist die Achszuordnung unsicher — der Tracker friert bei solchen
Objekten den zuerst gemessenen Winkel ein, und der kann die beiden Achsen
vertauscht haben. Dann **`max(length, width)` prüfen**, nicht die zugeordnete
Abmessung. Eine Zeile, konservativ, und sie macht ein Feld `square` im Datenvertrag
entbehrlich (Begründung in `datenvertraege.md` unter S1). Der Preis ist gering:
Bei einem Seitenverhältnis über 0,85 unterscheiden sich die beiden Abmessungen um
weniger als 18 %.

> **Warum 0,85 und nicht die 0,92 aus `detection.py`:** Die Grundfläche wird
> richtungsabhängig zu groß gemessen — ein exakt quadratischer 50 × 50-Klotz kam am
> 14.09.2026 als 58,8 × 52,3 heraus (Verhältnis 0,889). Mit 0,92 hätte die Ausnahme
> bei genau dem Klotz nicht gegriffen, für den sie gedacht ist
> (`entscheidungen.md`, Nachtrag 4 / M5). Ein einzelner Messpunkt — an weiteren
> Klötzen zu bestätigen.

Das verhindert die frustrierendste Fehlerart: sauber anfahren, greifen, passt
nicht.

### Kandidatenfilter

Ein Track ist Kandidat, wenn **alle** zutreffen: `status = 0` · in der Greifzone ·
nicht in der Gepickt-Liste · erreichbar · greifbar.

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
| gepickt | `picked_id` mit dieser ID |
| verloren | ID verschwindet aus `tracks` |
| unplausibel | `status ≠ 0` |

**Erreichbarkeit gehört bewusst nicht dazu.** Sie ist Auswahl-, kein
Abbruchkriterium. Sonst würde ein fast erfolgreicher Griff abgebrochen, sobald
der Block beim Greifen die Zonengrenze überschreitet. Die Greifzone ist enger als
der Sicherheitsraum; der Roboter darf dem Block ein Stück darüber hinaus folgen.

Nebeneffekt: Es braucht keine Rückmeldung des Follower-Zustands — die Entkopplung
aus dem Komponentenschnitt bleibt erhalten.

### `zone_upstream` (S4 Feld 12)

Der `priority_handler` gibt die **stromaufwärtige Längsgrenze** der Greifzone im
`target`-Satz mit — auch bei `has_target = 0`. Der `object_follower` begrenzt in
`ANFAHREN` seine Zielpose darauf und erzeugt damit den Abfangkurs.

Nur diese eine Grenze wird übertragen: Quer darf nicht begrenzt werden (der
Roboter wartet am Zonenrand auf der Spur des Blocks), stromabwärts ebenfalls nicht
(P4 — der Roboter muss dem Block über die Zonengrenze hinaus folgen dürfen), und
die Höhe deckt der Arbeitsraum-Clamp des Follower ab. Die Greifzone bleibt damit
vollständig in **einem** Besitz (`entscheidungen.md`, Nachtrag 3 / N3).

### Greifzone und Messregion des Trackers

**Die Greifzone muss innerhalb der Messregion von `base_cam` liegen**
(`track_velocity_region_y_min` / `_max`). Außerhalb dieser Region koppelt der
Tracker die Längsposition statt sie zu messen und löscht Tracks nicht mehr bei
ausbleibender Detektion — ein Phantom liefe dort mit Status 0 durch die Zone und
würde hier ausgewählt. Beide Grenzen werden in **B19 gemeinsam** festgelegt.
Begründung: `entscheidungen.md`, Nachtrag 3 / N2.

### `not_pickable`

IDs, die die Greifzone ohne Griff verlassen haben oder unplausibel sind. Reine
Buchhaltung für die Anzeige, nicht im Regelpfad.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen; prüfen, dass die Greifzone nicht leer ist |
| `on_activate` | Lock lösen, Gepickt-Liste leeren, `has_target = False` |
| `on_deactivate` | Lock lösen, `has_target = 0` ausgeben |

## Fehlerbehandlung

- Inkonsistente Eingangslänge → Zyklus überspringen, gedrosselt loggen
- Bleibt der Eingangszeitstempel stehen → Ziel zurückziehen. Ein Ziel auf Basis
  toter Daten ist gefährlicher als kein Ziel.
- Ziel außerhalb der Greifzone beim Start → nicht wählen

## Dateien

- **Neu:** `roboter_tetris/priority_handler.py`, JSON, Test
- **Geändert:** `setup.cfg`

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
