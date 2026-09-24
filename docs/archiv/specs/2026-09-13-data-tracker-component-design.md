# Design: Data Tracker Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** ✅ **umgesetzt 21.09.2026**, lokal getestet (12 Tests, darunter ein
Ende-zu-Ende-Lauf mit `fake_objects` → `vectoring` → Auswahl → Liste), in AICA noch
nicht gelaufen. Die Logik liegt in `world_bookkeeping.py` (ohne ROS).

> **Diese Spec wird gelöscht**, sobald die Komponente am Aufbau gelaufen ist. Die
> beim Bau getroffenen Entscheidungen stehen in `entscheidungen.md`,
> **Nachtrag 7 / T1–T2**.

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


## Zweck

Führt die Gesamtliste aller Objekte mit den Merkmalen `gepickt` und
`out_of_bounds` — **ausschließlich für Diagnose und Anzeige**.

Die Komponente ist ein **Blatt im Graphen**: Von ihr geht keine Leitung zurück in
den Regelpfad. Damit kann sie nichts blockieren, nichts verzögern und nichts
inkonsistent machen, und darf gemütlich mit 10 Hz laufen.

> Im ursprünglichen Plan war sie tragend: `vectoring` holte sich von hier die
> Information, was schon gepickt wurde. Das schloss einen Zyklus über drei
> Komponenten. Tatsächlich fließt diese Information in die andere Richtung —
> `object_follower` weiß es direkt und sagt es per `picked_id` allen Beteiligten.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`. Datenverträge: `docs/architektur/datenvertraege.md`
(S3 und S5 und S7 ein, S10 aus).

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `DataTracker` |
| Basisklasse | `LifecycleComponent` |
| Python-Datei | `roboter_tetris/data_tracker.py` |
| Registrierung | `roboter_tetris::DataTracker = roboter_tetris.data_tracker:DataTracker` |
| UI-Anzeigename | `Data Tracker` |
| Rate | 10 Hz |

## Schnittstellen

### Inputs

| Signal | Typ | Inhalt |
|---|---|---|
| `tracks` | `Float64MultiArray` | S3 aus `vectoring` |
| `not_pickable` | `Float64MultiArray` | S5 aus `priority_handler` |
| `picked_id` | `Float64MultiArray` | S7 aus `object_follower` |

### Outputs

| Signal | Typ | Inhalt |
|---|---|---|
| `world_state` | `Float64MultiArray` | S10 |

### Predicates

| Name | Bedeutung |
|---|---|
| `has_objects` | Liste nicht leer |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `expiry_after_done_s` | double | 10.0 | wie lange erledigte Einträge noch angezeigt werden — gepickt, `out_of_bounds` oder aus `tracks` verschwunden, gezählt ab der **letzten** dieser Änderungen, in S3-Zeit. Muss länger sein als Heben, Transfer und Ablegen, sonst kommt `picked` nicht mehr an |

## Kernlogik

Liste je ID, gespeist aus `tracks`. Zusätzlich:

- **`picked`** — gesetzt, wenn `picked_id` mit `outcome = 0` für diese ID eintrifft
- **`out_of_bounds`** — gesetzt, wenn die ID in `not_pickable` erscheint: Der
  Klotz hat die **Greifebene** ungegriffen überschritten, ist ab dort nicht mehr zu
  holen und fällt später am Bandende herunter (Nachtrag 6 / Z5, Z11). Ein Griff,
  der mit `outcome = 3` („verpasst") endete, landet auf demselben Weg hier.
- **`present`** — 1, solange die ID in den aktuellen `tracks` steht; 0 danach
  (**Nachlauf**, Nachtrag 7 / T1). Die Felder 0–13 bleiben dann auf den letzten
  bekannten Werten.

> **Warum der Nachlauf nötig ist.** Bei jedem Griff verschwindet der Klotz beim
> Heben aus dem Bild; `picked_id` mit `outcome = 0` kommt erst nach dem Ablegen,
> Sekunden später. Würde der Eintrag mit dem Track verschwinden, käme `picked` nie
> an. Nachlauf-Einträge ohne späteres Flag — ein verlorener Klotz, `outcome = 2` —
> fallen nach derselben Frist heraus.

Der S3-Kopf mit der gepoolten Bandgeschwindigkeit wird **unverändert** in den
S10-Kopf übernommen, damit die Anzeige sie zeigen kann.

`picked_id` wird über die **laufende Nummer `seq`** ausgewertet, mit
`contracts.AttemptWatcher` — derselben Regel wie im `priority_handler`: jede
Änderung außer 0 genau einmal (Nachtrag 7 / H5, T2). Das ersetzt das
"1 Sekunde lang True"-Muster des ursprünglichen Plans — kein Zeitfenster, keine
Flankenerkennung, kein Verpassen bei Lastspitzen.

**Verfallsregel:** Ein Eintrag ist erledigt, sobald `picked`, `out_of_bounds` oder
`present = 0` gilt, und verschwindet `expiry_after_done_s` nach der **letzten**
dieser Änderungen. Gezählt wird in **S3-Zeit**, nicht in Wanduhrzeit — steht der
Eingang, friert die Liste ein, statt sich zu leeren. Ein Eintrag, der dabei noch in
`tracks` steht (ein Klotz hinter der Greifebene auf dem Weg zum Bandende), kommt
nicht wieder, bis seine ID `tracks` verlassen hat. Ohne die Regel wächst das Signal
über den Programmlauf monoton; bei periodischem Publizieren wird das zu echter
Bandbreite. Historie gehört ins Log, nicht ins Signal.

**Reihenfolge je Schritt:** erst `tracks` (gegatet auf `t`), dann `not_pickable`,
dann `picked_id` — die Flags nehmen ihre Zeit vom neuesten Bild.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen |
| `on_activate` | Liste leeren, `seq`-Merker zurücksetzen, Frist übernehmen |
| `on_deactivate` | Liste leeren |

## Fehlerbehandlung

Inkonsistente Eingangslängen überspringen und gedrosselt loggen. Da die
Komponente nicht im Regelpfad liegt, darf sie im Zweifel schweigen — es entsteht
kein Sicherheitsrisiko.

## Dateien

- **Neu:** `roboter_tetris/world_bookkeeping.py` — die Buchhaltung, ohne ROS
- **Neu:** `roboter_tetris/data_tracker.py` — dünne Komponentenschale
- **Neu:** `component_descriptions/roboter_tetris_data_tracker.json`
- **Neu:** `test/python_tests/test_world_bookkeeping.py`, `test_data_tracker.py`
- **Geändert:** `setup.cfg`, `contracts.py` (S10 Feld 16, `AttemptWatcher`),
  `target_selection.py` (nutzt `AttemptWatcher`)

## Testbarkeit

Reine Logik, vollständig ohne ROS prüfbar: Aufnahme neuer IDs, Setzen der Flags,
`seq`-Auswertung (dieselbe `seq` zweimal → nur eine Reaktion), Verfall,
beschränkte Listenlänge über viele simulierte Pickvorgänge.

## Bewusste YAGNI-Entscheidungen

- Keine Historie über den Programmlauf hinaus, keine Persistenz.
- Keine Statistik (Pickrate, Erfolgsquote). Ließe sich aus `outcome` ergänzen,
  wird aber für den Betrieb nicht gebraucht.
- Keine eigene Plausibilitätsprüfung — der Status kommt aus `vectoring`.
