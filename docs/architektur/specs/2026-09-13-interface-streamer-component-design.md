# Design: Interface Streamer Component

**Datum:** 2026-09-13
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** ✅ **umgesetzt 21.09.2026**, lokal getestet (10 Tests, dazu ein
gerendertes Beispielbild), in AICA noch nicht gelaufen. Text und Zusammensetzung in
`interface_layout.py`, Schale in `interface_streamer.py`. Beim Bau entschieden:
`entscheidungen.md` **Nachtrag 11** (ASCII-Text, Ankunftszeit der Debug-Bilder,
Markierung veralteter Daten).

> **Diese Spec wird gelöscht**, sobald die Komponente am Aufbau gelaufen ist.

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
│  Band  v = 101 mm/s  Richtung 90.3°  (4 Kl.)  │
├───────────────────────────────────────────────┤
│  ID 003  blau  final         v = 100 mm/s     │
│  ID 004  rot   einschwingend Δv = 31 mm/s     │
│  ID 002  gelb  gepickt                        │
└───────────────────────────────────────────────┘
```

Alle Werte liegen in `world_state` und `follower_status` bereits an — es braucht
keine zusätzlichen Leitungen und keine eigene Logik.

Der Wert `w_wirksam` ist dabei der wichtigste in der Anzeige: Er macht sichtbar,
aus welcher Quelle die Zielposition gerade stammt, und ist damit die Grundlage
für den geplanten Vergleich Basiskamera ↔ Roboterkamera.

Statuscodes werden als Klartext ausgegeben (0 final · 3 einschwingend), damit im
Betrieb **sichtbar ist, warum** ein Block nicht angefahren wurde.

Einträge mit `present = 0` (S10 Feld 16) sind **Nachlauf**: Der Klotz ist aus dem
Bild verschwunden, die Werte sind eingefroren — er liegt im Greifer oder ist
verloren. Solche Zeilen werden abgesetzt gezeigt (etwa grau, „nicht mehr im Bild"),
nie mit ihrer letzten Position, als läge der Klotz noch dort (Nachtrag 7 / T1).

**Die Geschwindigkeitsschätzung gehört sichtbar in die Anzeige** — sie ist Ziel 3,
und diese Komponente dient dem eigenen Ziel, den Prozess nachvollziehbar
darzustellen (Nachtrag 6 / Z1). Gezeigt werden die gepoolte Bandgeschwindigkeit mit
Richtung und Anzahl beitragender Klötze aus dem S10-Kopf sowie je Klotz die eigene
Schätzung; bei einschwingenden Klötzen statt der Geschwindigkeit `v_change` — wie
weit die beiden Halbfenster noch auseinanderliegen. Daran sieht man, *warum* ein
Klotz noch nicht final ist. Beides liegt
in `world_state` bereits an.

## Ausführungsmodell

`on_step_callback` bei 5 Hz. Fehlt ein Eingangsbild, wird sein Bereich grau
gefüllt und beschriftet — die Anzeige darf nie ausfallen, nur weil eine Quelle
schweigt. „Fehlt" heißt: seit 2 s kein neues Bild angekommen — die Debug-Bilder
tragen keinen Zeitstempel (Nachtrag 11 / V2). Die Bildgröße hängt nur an den
Parametern (1280 × 636 bei den Defaults), Bilder werden seitenrichtig eingepasst.
Die Liste zeigt höchstens 8 Zeilen, bei mehr die neuesten.

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

- **Neu:** `roboter_tetris/interface_layout.py` — Text (ohne ROS) und Zusammensetzung (numpy, OpenCV)
- **Neu:** `roboter_tetris/interface_streamer.py` — Komponentenschale
- **Neu:** `component_descriptions/roboter_tetris_interface_streamer.json`
- **Neu:** `test/python_tests/test_interface_layout.py`, `test_interface_streamer.py`
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
