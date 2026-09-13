# Design: Vectoring Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung

## Zweck

Glättet die Objektzustände aus `base_cam` und bewertet ihre Plausibilität.

Weil Bandrichtung und Bandgeschwindigkeit kalibrierte Konstanten sind (Thema 3),
ist die Bahn jedes Objekts bis auf zwei Zahlen bekannt: die Querablage (ändert
sich nie) und die Phase entlang des Bandes. Beide werden **gemittelt, nicht
gefittet** — deutlich billiger und genauer als eine Regression.

Genauigkeitsgewinn: bei ~3 mm Rauschen je Bild und 30 Messungen bleiben quer
**0,55 mm** — Faktor 5, in der Richtung mit dem geringsten Greiferspiel.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`, besonders: `LifecycleComponent`, kein Pub/Sub, nie blockieren,
Registrierung mit `::`, JSON in `component_descriptions/`.
Datenverträge: `docs/review/datenvertraege.md` (S1 ein, S3 aus).

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `Vectoring` |
| Basisklasse | `modulo_components.lifecycle_component.LifecycleComponent` |
| Python-Datei | `source/roboter_tetris/roboter_tetris/vectoring.py` |
| Beschreibung | `component_descriptions/roboter_tetris_vectoring.json` |
| Registrierung | `roboter_tetris::Vectoring = roboter_tetris.vectoring:Vectoring` |
| UI-Anzeigename | `Vectoring` |
| Rate | 100 Hz (rechnet nur bei neuem Zeitstempel) |

## Schnittstellen

### Inputs

| Signal | Attribut | Typ | Inhalt |
|---|---|---|---|
| `objects` | `_objects` | `Float64MultiArray` | S1 aus `base_cam` |

### Outputs

| Signal | Attribut | Typ | Inhalt |
|---|---|---|---|
| `tracks` | `_tracks` | `Float64MultiArray` | S3 |

### Predicates

| Name | Bedeutung |
|---|---|
| `has_tracks` | mindestens ein Track mit Status 0 |
| `is_receiving` | Eingangszeitstempel schreitet fort |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `belt_heading_deg` | double | – (B1) | Laufrichtung des Bandes in der xy-Ebene von `world` |
| `belt_speed_mps` | double | – (B1) | Bandgeschwindigkeit |
| `smoothing_window` | int | 30 | Anzahl Messungen für die Mittelung |
| `min_samples` | int | 5 | darunter Status 3 (nicht eingeschwungen) |
| `speed_tolerance` | double | 0.35 | relative Abweichung von `belt_speed_mps`, ab der ein Objekt als unplausibel gilt |
| `orientation_quality_min` | double | 0.7 | Mindest-Resultantenlänge der Winkelmittelung |
| `track_expiry_s` | double | 0.5 | ausbleibende IDs danach vergessen |

## Ausführungsmodell

`on_step_callback`, **gegatet auf den Zeitstempel** im Kopf von S1. Ist `t`
unverändert, sofortige Rückkehr. AICA publiziert Ausgänge in jedem Schritt, der
Eingang wird also mehrfach gesehen.

## Kernlogik

Je ID ein Ringpuffer mit `(t, x, y, z, orientation, length, width, height)`.

**Querposition und Höhe:** laufender Mittelwert. Beides ist je Objekt konstant.

**Längsposition:** Messungen auf die Bandgerade projizieren, Phase `s₀` zum
Bezugszeitpunkt mitteln, zur Ausgabe auf den aktuellen Kopfzeitstempel
extrapolieren. Die Ausgabeposition ist damit **gültig für `t` im Kopf**.

**Orientierung — über den verdoppelten Winkel:**

```
mittel  = ½ · atan2( ⟨sin 2θ⟩ , ⟨cos 2θ⟩ )
guete   = | ⟨e^{i2θ}⟩ |        (0…1)
```

Grund: Die Orientierung ist π-periodisch. Ein arithmetischer Mittelwert aus 0°
und 90° wäre 45° — genau die Lage, in der der Greifer die Ecken erwischt. Bei
fast quadratischen Blöcken springt der Winkel aus `minAreaRect` genau so.
`guete < orientation_quality_min` → Orientierung unbrauchbar, Status bleibt 0,
aber der Follower fällt auf den festen Winkel zurück (Modus-1-Verhalten).

**Plausibilität:** gemessene Längsgeschwindigkeit aus dem Puffer gegen
`belt_speed_mps`.

| Bedingung | Status |
|---|---|
| Messungen < `min_samples` | 3 |
| Geschwindigkeit < (1 − tol) · v_band | 1 (steht / verklemmt) |
| Geschwindigkeit > (1 + tol) · v_band oder Positionssprung | 2 (angestoßen / Fehldetektion) |
| sonst | 0 |

> **Abstimmhinweis:** Ein gleichgewichtetes Fenster ist optimal, solange das
> Messrauschen zufällig ist. Ist der Fehler dagegen positionsabhängig — etwa
> durch einen kleinen Kalibrierfehler, der über das Sichtfeld wandert — mittelt
> man über eine Drift statt über Rauschen. Das lässt sich vorher nicht
> entscheiden. Falls die Quergenauigkeit im Betrieb enttäuscht, ist eine
> **exponentielle Gewichtung** (neuere Messungen zählen mehr) der erste Versuch,
> bevor irgendetwas anderes angefasst wird.

**Verfall:** IDs ohne Update für `track_expiry_s` werden entfernt. Ohne diese
Regel wächst der Zustand monoton — auch durch kurzlebige Fehl-IDs, etwa wenn beim
Auflegen eine Hand ins Bild ragt.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen, Bandrichtung als Einheitsvektor vorberechnen |
| `on_activate` | Puffer leeren, Predicates zurücksetzen |
| `on_deactivate` | Puffer leeren |

## Fehlerbehandlung

- Inkonsistente Eingangslänge → Zyklus überspringen, gedrosselt loggen
- Nicht-endliche Werte einer Messung → diese Messung verwerfen, Track behalten
- Bleibt der Eingangszeitstempel stehen → `is_receiving = False`, Ausgabe `[t, 0]`

## Dateien

- **Neu:** `roboter_tetris/vectoring.py`
- **Neu:** `component_descriptions/roboter_tetris_vectoring.json`
- **Neu:** `test/python_tests/test_vectoring.py`
- **Geändert:** `setup.cfg`

## Testbarkeit

Die Rechenlogik in eine ROS-freie Klasse (etwa `TrackSmoother`) auslegen, analog
zu `GripperTargetLogic` im Greifer. Ohne Hardware prüfbar:
Mittelungskonvergenz, Statusübergänge 3→0, Erkennung eines stehenden Objekts,
**Winkelmittelung bei 0°/90°-Sprüngen** (muss nahe 0° oder 90° liefern, nie 45°),
Gütemaß bei konsistenten und bei springenden Winkeln, Verfall.

## Bewusste YAGNI-Entscheidungen

- Keine Regression für Richtung oder Geschwindigkeit — beides ist kalibriert.
  Parameter `K`, `W`, `J`, `H` und die "halten"-Schalter des ursprünglichen Plans
  entfallen ersatzlos.
- Keine Re-Identifikation verlorener IDs. Ein Track-Abriss führt zu einer neuen
  ID; der `priority_handler` wählt dann neu.
- Keine z-Glättung über die Bandgeometrie — der Mittelwert genügt.
