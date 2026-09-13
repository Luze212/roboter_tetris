# Design: Priority Handler Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung

## Zweck

Wählt aus den geglätteten Objektzuständen das Objekt aus, das als nächstes
gegriffen wird, und hält diese Wahl fest, bis sie erledigt ist.

Die Auswahlregel des ursprünglichen Plans — "das nächste Objekt zum nicht
pickbaren Bereich" — bleibt erhalten, bekommt aber eine **Erreichbarkeitsprüfung
davor**. Ohne sie wählt das System systematisch das Objekt mit der geringsten
verbleibenden Zeit und jagt damit Blöcken hinterher, die nicht mehr zu schaffen
sind.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`. Datenverträge: `docs/review/datenvertraege.md`
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

### Outputs

| Signal | Typ | Inhalt |
|---|---|---|
| `target` | `Float64MultiArray` | S4 — feste Länge 12 |
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

t_benötigt  = abstand / attractor_v_max_mps        ← Fahrt
            + 3 / attractor_gain                   ← Einschwingen des Attractors
            + t_descend_s + t_grasp_s

kandidat ⟺ t_verfügbar > reach_safety_factor · t_benötigt
```

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
