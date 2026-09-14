# Design: Interface Streamer Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung — **als letzte Komponente zu bauen**

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.


## Zweck

Setzt ein Übersichtsbild für RViz zusammen, damit der Prozess live verfolgt
werden kann.

Es ist die **einzige Komponente, ohne die das System vollständig funktioniert**.
Deshalb steht sie am Ende der Umsetzung — und deshalb ist sie bewusst schlank
gehalten.

## Rechenzeit-Vorgabe

Zwei Kamerabilder zu kopieren, zu skalieren, zusammenzusetzen und zu beschriften
kostet dieselbe CPU, die die Echtzeit-Bildverarbeitung braucht. Das System ist
dort bereits an der Grenze.

→ **Rate 5 Hz**, feste Zielauflösung, keine aufwendige Grafik. Wenn es im Betrieb
stört, wird die Rate weiter gesenkt, bevor irgendetwas anderes angefasst wird.

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `InterfaceStreamer` |
| Basisklasse | `LifecycleComponent` |
| Python-Datei | `roboter_tetris/interface_streamer.py` |
| Registrierung | `roboter_tetris::InterfaceStreamer = roboter_tetris.interface_streamer:InterfaceStreamer` |
| UI-Anzeigename | `Interface Streamer` |
| Rate | 5 Hz |

## Schnittstellen

### Inputs

| Signal | Typ | Quelle |
|---|---|---|
| `base_debug_image` | `sensor_msgs/Image` | `base_cam` |
| `robot_debug_image` | `sensor_msgs/Image` | `robot_cam` |
| `world_state` | `Float64MultiArray` | S10 aus `data_tracker` |
| `follower_status` | `Float64MultiArray` | S8 aus `object_follower` |

### Outputs

| Signal | Typ |
|---|---|
| `interface_image` | `sensor_msgs/Image` |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `panel_width` | int | 1280 | Breite des Gesamtbildes |
| `image_height` | int | 360 | Höhe des Bildbereichs je Kamera |
| `show_object_list` | bool | true | Objektliste einblenden |

## Layout

```
┌───────────────────────┬───────────────────────┐
│  Debug-Bild Basiscam  │  Debug-Bild Robotcam  │
├───────────────────────┴───────────────────────┤
│  Zustand: FOLGEN   Ziel: 003   w: 0.65        │
│  Abweichung  längs 2.1mm  quer 0.8mm  z 1.4mm │
├───────────────────────────────────────────────┤
│  ID 003  blau   status 0                      │
│  ID 004  rot    status 1 (steht)              │
│  ID 002  gelb   gepickt                       │
└───────────────────────────────────────────────┘
```

Alle Werte liegen in `world_state` und `follower_status` bereits an — es braucht
keine zusätzlichen Leitungen und keine eigene Logik.

Der Wert `w_wirksam` ist dabei der wichtigste in der Anzeige: Er macht sichtbar,
aus welcher Quelle die Zielposition gerade stammt, und ist damit die Grundlage
für den geplanten Vergleich Basiskamera ↔ Roboterkamera.

Statuscodes werden als Klartext ausgegeben (0 ok · 1 steht · 2 Sprung ·
3 einschwingend), damit im Betrieb **sichtbar ist, warum** ein Block nicht
angefahren wurde.

## Ausführungsmodell

`on_step_callback` bei 5 Hz. Fehlt ein Eingangsbild, wird sein Bereich grau
gefüllt und beschriftet — die Anzeige darf nie ausfallen, nur weil eine Quelle
schweigt.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | `CvBridge` anlegen, Layout vorberechnen |
| `on_activate` | Puffer leeren |
| `on_deactivate` | – |

## Fehlerbehandlung

Alle Konvertierungen in `try/except`; bei Fehlern gedrosselt loggen und den
letzten gültigen Bereich behalten. Die Komponente darf unter keinen Umständen
andere Komponenten beeinträchtigen.

## Dateien

- **Neu:** `roboter_tetris/interface_streamer.py`, JSON, Test
- **Geändert:** `setup.cfg`

## Testbarkeit

Layout-Berechnung und Textaufbereitung ROS-frei prüfbar. Bildzusammensetzung mit
synthetischen Bildern testen (richtige Größe, keine Ausnahme bei fehlendem
Eingang).

## Bewusste YAGNI-Entscheidungen

- Kein Video-Mitschnitt, keine Aufzeichnung.
- Keine interaktive Bedienung.
- Keine Trendkurven oder Diagramme.
- Alternative, falls die Rechenzeit doch nicht reicht: Komponente weglassen und
  in RViz zwei Image-Displays plus ein Text-Overlay verwenden. Der Funktionsumfang
  des Systems ändert sich dadurch nicht.
