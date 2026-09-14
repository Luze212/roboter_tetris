# Design: Data Tracker Component

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
| `expiry_after_done_s` | double | 10.0 | wie lange erledigte Einträge (gepickt oder out_of_bounds) noch angezeigt werden |

## Kernlogik

Liste je ID, gespeist aus `tracks`. Zusätzlich:

- **`picked`** — gesetzt, wenn `picked_id` mit `outcome = 0` für diese ID eintrifft
- **`out_of_bounds`** — gesetzt, wenn die ID in `not_pickable` erscheint

`picked_id` wird über die **laufende Nummer `seq`** ausgewertet: Die zuletzt
gesehene `seq` merken, nur bei Änderung reagieren. Das ersetzt das
"1 Sekunde lang True"-Muster des ursprünglichen Plans — kein Zeitfenster, keine
Flankenerkennung, kein Verpassen bei Lastspitzen.

**Verfallsregel:** Einträge mit `picked` oder `out_of_bounds` verschwinden nach
`expiry_after_done_s`. Ohne sie wächst das Signal über den Programmlauf monoton;
bei periodischem Publizieren wird das zu echter Bandbreite. Historie gehört ins
Log, nicht ins Signal.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen |
| `on_activate` | Liste leeren, `seq`-Merker zurücksetzen |
| `on_deactivate` | Liste leeren |

## Fehlerbehandlung

Inkonsistente Eingangslängen überspringen und gedrosselt loggen. Da die
Komponente nicht im Regelpfad liegt, darf sie im Zweifel schweigen — es entsteht
kein Sicherheitsrisiko.

## Dateien

- **Neu:** `roboter_tetris/data_tracker.py`, JSON, Test
- **Geändert:** `setup.cfg`

## Testbarkeit

Reine Logik, vollständig ohne ROS prüfbar: Aufnahme neuer IDs, Setzen der Flags,
`seq`-Auswertung (dieselbe `seq` zweimal → nur eine Reaktion), Verfall,
beschränkte Listenlänge über viele simulierte Pickvorgänge.

## Bewusste YAGNI-Entscheidungen

- Keine Historie über den Programmlauf hinaus, keine Persistenz.
- Keine Statistik (Pickrate, Erfolgsquote). Ließe sich aus `outcome` ergänzen,
  wird aber für den Betrieb nicht gebraucht.
- Keine eigene Plausibilitätsprüfung — der Status kommt aus `vectoring`.
