# Design: Vectoring Component

**Datum:** 2026-09-13, **überarbeitet 2026-09-21** (Nachtrag 6)
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** ✅ **umgesetzt 21.09.2026**, lokal getestet (17 Tests gegen die
Ground Truth aus `fake_objects.py`), in AICA geladen, Datenpfad am 22.09. geprüft
(Nachtrag 12 / K4); mit laufendem Band noch offen (Block 3).

> **Diese Spec wird gelöscht**, sobald die Komponente am Aufbau gelaufen ist — dann
> gemeinsam mit der Spec der Bestandskomponenten (Regel „Specs haben ein Ende").
> Bis dahin trägt sie die Begründungen, die im Code nur knapp stehen.

> **Vorrang.** Normativ sind `docs/architektur/entscheidungen.md` und
> `docs/architektur/datenvertraege.md`. Diese Spec ist daraus **abgeleitet** und
> erzählt sie bewusst nach, damit sie ohne Vorkontext lesbar ist. Bei Widerspruch
> gelten die beiden normativen Dokumente. **Sobald die Komponente gebaut und ihre
> JSON-Beschreibung geschrieben ist, wird diese Datei gelöscht** — Code und JSON
> tragen den Vertrag dann selbst, und eine dritte Stelle wäre nur Pflegeaufwand.

> ⚠️ **Überarbeitet nach Nachtrag 6.** Die erste Fassung ging davon aus, dass
> Bandrichtung und -geschwindigkeit kalibrierte Konstanten sind, und strich die
> Geschwindigkeitsschätzung ausdrücklich. Ziel 3 verlangt genau diese Schätzung.
> Außerdem prüfte sie auf „festhängende" und „angestoßene" Klötze, die es nicht
> gibt. Beides ist ersetzt.


## Zweck

**Die Komponente, die Ziel 3 umsetzt:** Sie schätzt die Geschwindigkeit jedes
Klotzes aus den Bilddaten, erkennt, wann ein frisch aufgelegter Klotz
eingeschwungen ist, und glättet die Objektzustände.

Drei Aufgaben:

1. **Geschwindigkeit je Objekt** als Vektor `(vx, vy)` aus der eigenen
   Positionshistorie — Richtung und Betrag, ohne Bandmodell.
2. **Einschwingen erkennen.** Ein Klotz kann beim Aufsetzen umkippen. Er gilt als
   **final**, sobald seine Geschwindigkeit konstant gemessen ist — ab dann wird er
   nicht mehr in Frage gestellt.
3. **Pool.** Aus allen Messungen seit dem Einschwingen, aller finalen Klötze des
   laufenden Durchlaufs, eine gemeinsame Ausgleichsrechnung. Das ist die
   Bandgeschwindigkeit, mit der `priority_handler` und `object_follower` rechnen —
   sie wird mit jeder Messung genauer.

Warum der Pool: Das Band läuft mit konstanter Geschwindigkeit, die Klötze bewegen
sich frei mit ihm. Alle finalen Klötze messen also dieselbe Größe. Über viele
Klötze gemittelt wird die Schätzung genauer als jede einzelne, und ein frisch
aufgelegter Klotz hat vom ersten Bild an eine brauchbare Geschwindigkeit.

## Verbindliche Rahmenregeln

`ARCHITECTURE.md`, besonders: `LifecycleComponent`, kein Pub/Sub, nie blockieren,
Registrierung mit `::`, JSON in `component_descriptions/`.
Datenverträge: `docs/architektur/datenvertraege.md` (S1 ein, S3 aus).
Feldindizes und Packen **ausschließlich** über `roboter_tetris/contracts.py`.

## Voraussetzung

**Der Tracker in `base_cam` muss die gemessene Längsposition liefern**
(Nachtrag 6 / Z4, Umsetzungsplan 2.4). Solange er außerhalb der Messregion
Positionen aus einer Geschwindigkeit *rechnet*, schätzt diese Komponente dort nur
zurück, was hineingesteckt wurde — und ein kippender Klotz erscheint glatt.

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
| `tracks` | `_tracks` | `Float64MultiArray` | S3 — Kopf mit Pool-Geschwindigkeit, Tracks mit eigener Schätzung |

### Predicates

| Name | Bedeutung |
|---|---|
| `has_tracks` | mindestens ein Track mit Status 0 |
| `has_belt_estimate` | Pool nicht leer (`n_pool ≥ 1`) |
| `is_receiving` | Eingangszeitstempel schreitet fort |

### Parameter

| Name | Typ | Default | Bedeutung |
|---|---|---|---|
| `settle_half_window` | int | 5 (bis 23.09.: 15) | Messungen je Halbfenster des Einschwingkriteriums — bei 30 Hz wären 15 eine halbe Sekunde; am Aufbau kommen 5–7 Messungen/s (Nachtrag 13 / L2, L8) |
| `settle_v_tolerance` | double | 0.010 | m/s — so weit dürfen die Geschwindigkeiten der beiden Halbfenster auseinanderliegen, damit sie als „konstant" gelten (D20) |
| `outlier_distance_m` | double | 0.02 | Abstand zur Vorhersage, ab dem eine einzelne Messung verworfen wird (D21) |
| `outlier_persist_frames` | int | 3 | so viele Ausreißer in Folge gelten als echte Lageänderung → Neustart |
| `smoothing_window` | int | 30 | Anzahl Messungen für die Mittelung von Position, Orientierung, Geometrie |
| `track_expiry_s` | double | 0.5 | ausbleibende IDs danach vergessen |

> **Entfallen gegenüber der ersten Fassung:** `belt_heading_deg`,
> `belt_speed_mps`, `speed_tolerance`, `min_samples`. Die ersten beiden waren die
> kalibrierten Konstanten, die jetzt geschätzt werden; die Toleranz prüfte auf
> nicht existierende festhängende Klötze; `min_samples` wird durch das
> Einschwingkriterium ersetzt.
>
> **Ebenfalls entfallen (Z9):** `velocity_window`, `settle_min_samples` und
> `settle_v_sigma_max` aus der Überarbeitung vom Vormittag. Ihr Kriterium — der
> Standardfehler der Steigung — erklärte einen um 25 mm gekippten Klotz in jedem
> Versuch für final, mit 31 % falscher Geschwindigkeit.

## Ausführungsmodell

`on_step_callback`, **gegatet auf den Zeitstempel** im Kopf von S1. Ist `t`
unverändert, sofortige Rückkehr. AICA publiziert Ausgänge in jedem Schritt, der
Eingang wird also mehrfach gesehen.

## Kernlogik

Je ID ein Ringpuffer mit `(t, x, y, z, orientation, length, width, height)`.

### 1. Einschwingen: „konstant gemessen" — Status 3 → 0

Über die letzten `2 · settle_half_window` Messungen je Track zwei Ausgleichsgeraden
für `x(t)` und `y(t)` — eine über die ältere, eine über die jüngere Hälfte. Ihre
Steigungen sind die Geschwindigkeiten `v_alt` und `v_neu`.

```
v_change = | v_neu − v_alt |          (Vektorbetrag)
```

| Bedingung | Status |
|---|---|
| weniger als `2 · settle_half_window` Messungen **oder** `v_change > settle_v_tolerance` | 3 — einschwingend |
| sonst | **0 — final** |

Das ist wörtlich „konstant gemessen": Die gemessene Geschwindigkeit ändert sich
nicht mehr. Ein kippender Klotz springt im Schwerpunkt; solange der Sprung in einem
der beiden Halbfenster liegt, weichen ihre Steigungen deutlich voneinander ab.
Final wird er erst, wenn der Sprung aus beiden heraus ist — dann sind beide Hälften
sauber.

> **Warum nicht der Standardfehler der Steigung** (erste Fassung): Eine
> Ausgleichsgerade durch einen Sprung schluckt ihn als zusätzliche Steigung. Die
> Residuen bleiben mäßig, der Standardfehler klein — ein um 25 mm gekippter Klotz
> galt in jedem Versuch als final, mit 31 % falscher Geschwindigkeit. Das neue
> Kriterium: 0 % (Nachtrag 6 / Z9).

**„Final" ist ein Status, kein eingefrorener Wert.** Der Übergang ist einseitig —
mit einer Ausnahme, siehe Punkt 4. Der Klotz ist ab dann wählbar und wird nicht
mehr in Frage gestellt.

**Der saubere Abschnitt** eines Tracks beginnt mit dem älteren der beiden
übereinstimmenden Halbfenster. Ab dort gehen seine Messungen in den Pool und in
die Glättung.

### 2. Geschwindigkeit je Objekt

S3-Felder 10/11: Ausgleichsgerade über **alle** Messungen des sauberen Abschnitts —
fortlaufend genauer, nicht eingefroren. Vor dem Einschwingen die Steigung über das
aktuelle Doppelfenster. Das ist Anzeige und Diagnose; gerechnet wird mit dem Pool.

### 3. Pool — die Bandgeschwindigkeit

Eine **gemeinsame Ausgleichsrechnung** über die sauberen Abschnitte aller Tracks
des Durchlaufs: eine gemeinsame Steigung, je Abschnitt ein eigener
Achsenabschnitt. Je Koordinate:

```
v_belt = Σ_Abschnitte S_tp  /  Σ_Abschnitte S_tt

S_tt = Σ (t − t̄)²              je Abschnitt
S_tp = Σ (t − t̄)(p − p̄)        je Abschnitt
n_pool = Anzahl der Abschnitte
```

Das entspricht dem gewichteten Mittel der Einzelsteigungen, gewichtet mit `S_tt` —
also mit der Genauigkeit jeder einzelnen. Eine lange, saubere Bahn zählt mehr als
eine kurze. Über die volle Bahn ist das 160- bis 280-mal genauer als das Mitteln
eingefrorener Schnappschüsse (Z9).

**Laufend und mit festem Speicher:** Je Abschnitt genügen fünf Summen (`n`, `Σt`,
`Σt²`, `Σp`, `Σtp` je Koordinate). ⚠️ **Zeiten relativ zum Abschnittsbeginn
führen**, nicht als absolute Zeitstempel: Bei Werten um 1,7·10⁹ s frisst die
Auslöschung in `Σt² − (Σt)²/n` die Genauigkeit vollständig.

Der Pool behält die Beiträge von Klötzen, die das Band inzwischen verlassen haben
oder gegriffen wurden — gerechnet wird über den **Durchlauf**, nicht nur über die
gerade sichtbaren Klötze. Ein Durchlauf ist eine Aktivierung; beim Aktivieren wird
der Pool geleert.

**Kaltstart unkritisch:** Der erste Klotz ist nach rund 1,3 s final — bei 0,1 m/s
nach 13 cm von rund 1,45 m Bahn im Bild.

**`n_pool = 0` ist ein Zustand, kein Wert.** Vor dem ersten finalen Klotz gibt es
keine Bandgeschwindigkeit, und das steht ausdrücklich im Vertrag. Verbraucher
dürfen `v_belt` dann nicht verwenden.

### 4. Fehldetektionen und Lageänderungen

Eine Messung, die weiter als `outlier_distance_m` von der Vorhersage der
aktuellen Ausgleichsgeraden entfernt liegt, wird **verworfen** — der Track bleibt,
die Messung geht nicht in den Puffer. Das fängt einzelne Fehldetektionen ab.

Folgen `outlier_persist_frames` solcher Messungen **hintereinander**, ist es keine
Fehldetektion, sondern eine echte Lageänderung — typisch das Umkippen beim
Aufsetzen. Dann beginnt der Puffer neu, der Track geht zurück auf Status 3.

> Die Unterscheidung ist wichtig: Ohne sie würden nach einem Umkippen **alle**
> folgenden Messungen als Ausreißer verworfen, weil sie auf einer neuen Linie
> liegen. Der Track verhungerte.

Physikalisch kippt ein Klotz nur beim Aufsetzen. Tritt eine anhaltende Abweichung
bei einem bereits finalen Track auf, deutet das auf etwas Ungewöhnliches — etwa
eine vertauschte ID im Tracker. Auch dann zurück auf Status 3: Der Klotz ist
damit kein Kandidat, bis er wieder eingeschwungen ist. Sein bisheriger sauberer
Abschnitt wird **abgeschlossen und bleibt im Pool**, denn er wurde gemessen, als
der Klotz konstant lief; nach dem erneuten Einschwingen beginnt ein neuer.

### 5. Glättung — erst ab „final"

**Alles wird erst ab dem sauberen Abschnitt gemittelt: Position, Geometrie
(`length`, `width`, `height`) und Orientierung.** Ein umgekippter Klotz hat andere
Abmessungen — ein stehender 100-mm-Klotz liegt danach mit 50 mm Höhe —, und sein
Schwerpunkt springt um rund die halbe Kantenlänge. Beides darf nicht in einen
Mittelwert mit dem Zustand vor dem Kippen. Vor dem Übergang wird die letzte
Einzelmessung ausgegeben, nur zur Anzeige.

**Querposition:** laufender Mittelwert quer zur Bewegungsrichtung. Die Richtung
ist die des Pools, solange `n_pool ≥ 1`; für den allerersten Klotz eines
Durchlaufs seine eigene.

**Längsposition:** Messungen auf die Bewegungsgerade projizieren, Phase zum
Bezugszeitpunkt mitteln, zur Ausgabe mit `v_belt` auf den aktuellen
Kopfzeitstempel extrapolieren. Die Ausgabeposition ist damit **gültig für `t` im
Kopf**.

> Position und Geschwindigkeit fallen damit aus **einem** Modell: gemeinsame
> Bandgeschwindigkeit, je Klotz eine Lage. Das ist Ziel 3 — Geschwindigkeitsschätzung
> *und* Positionsberechnung — als ein zusammenhängendes Verfahren. Für die Position
> bleibt es beim gleitenden Fenster (`smoothing_window`), nicht beim ganzen
> Abschnitt: Wandert ein kleiner Kalibrierfehler über das Sichtfeld, zählt die
> Lage nahe der Greifzone, nicht der Mittelwert über das ganze Bild.

**Orientierung — über den verdoppelten Winkel:**

```
mittel  = ½ · atan2( ⟨sin 2θ⟩ , ⟨cos 2θ⟩ )
guete   = | ⟨e^{i2θ}⟩ |        (0…1)
```

Grund: Die Orientierung ist π-periodisch. Ein arithmetischer Mittelwert aus 0°
und 90° wäre 45° — genau die Lage, in der der Greifer die Ecken erwischt.
`guete` wird als **S3 Feld 13 `ori_quality`** ausgegeben. Bewertet wird sie
**nicht hier**, sondern beim Verbraucher: Der Follower fällt unter seiner Schwelle
`orientation_quality_min` auf die Bandrichtung zurück (Modus-1-Verhalten). Der
Status bleibt von der Güte unberührt.

> ⚠️ **Geändert (Nachtrag 6 / Z13).** Hier stand die Schwelle als Parameter dieser
> Komponente, mit dem Satz „der Follower fällt auf den festen Winkel zurück". Kein
> Vertragsfeld trug aber die Güte — der Follower hätte es nie erfahren.

> **Zum Gütemaß bei quadratischen Klötzen:** `vision/tracker.py` friert die
> Orientierung fast quadratischer Objekte (`square`, Seitenverhältnis ≥ 0,92) auf
> den zuerst gemessenen Wert ein. Für sie kommt ein konstanter Winkel an, `guete`
> bleibt nahe 1 und schlägt nie an. Das Gütemaß wirkt gegen verrauschte
> Detektion, Teilverdeckung und Objekte halb außerhalb der ROI. Wichtig bei der
> Festlegung von D11: An einem quadratischen Klotz lässt sich die Schwelle nicht
> einstellen. Hintergrund: `architektur/vorgaengerprojekt-abgleich.md` §7.

> **Abstimmhinweis:** Ein gleichgewichtetes Fenster ist optimal, solange das
> Messrauschen zufällig ist. Ist der Fehler dagegen positionsabhängig — etwa
> durch einen kleinen Kalibrierfehler, der über das Sichtfeld wandert — mittelt
> man über eine Drift statt über Rauschen. Falls die Quergenauigkeit im Betrieb
> enttäuscht, ist eine **exponentielle Gewichtung** (neuere Messungen zählen mehr)
> der erste Versuch, bevor irgendetwas anderes angefasst wird.

### 6. Verfall

IDs ohne Update für `track_expiry_s` werden aus der Trackliste entfernt — ihr
Pool-Beitrag bleibt. Ohne diese Regel wächst der Zustand monoton, auch durch
kurzlebige Fehl-IDs, etwa wenn beim Auflegen eine Hand ins Bild ragt.

## Lifecycle

| Transition | Verhalten |
|---|---|
| `on_configure` | Parameter lesen |
| `on_activate` | Puffer und **Pool** leeren, Predicates zurücksetzen — ein neuer Durchlauf beginnt |
| `on_deactivate` | Puffer leeren |

## Fehlerbehandlung

- Inkonsistente Eingangslänge → Zyklus überspringen, gedrosselt loggen
- Nicht-endliche Werte einer Messung → diese Messung verwerfen, Track behalten
- Bleibt der Eingangszeitstempel stehen → `is_receiving = False`, Ausgabe nur noch
  Kopf mit `n = 0`; der Pool bleibt erhalten

## Dateien

- **Neu:** `roboter_tetris/track_estimation.py` — die gesamte Rechenlogik, ohne
  ROS und ohne numpy, damit sie ohne AICA prüfbar ist
- **Neu:** `roboter_tetris/vectoring.py` — dünne Komponentenschale
- **Neu:** `component_descriptions/roboter_tetris_vectoring.json`
- **Neu:** `test/python_tests/test_vectoring.py`
- **Geändert:** `setup.cfg`

## Testbarkeit

Die Rechenlogik in eine ROS-freie Klasse (etwa `TrackEstimator`) auslegen, analog
zu `GripperTargetLogic` im Greifer.

**`test/tools/fake_objects.py` liefert die Ground Truth** — am realen Band kennt
niemand die wahre Geschwindigkeit, am synthetischen schon. Damit prüfbar:

- geschätzte Geschwindigkeit trifft die eingestellte, in Betrag **und** Richtung
- Übergang 3 → 0 nach dem Einschwingen; der Status bleibt danach 0
- **Umkippen:** ein Sprung von 13 und 25 mm beim Aufsetzen hält den Track in
  Status 3, bis er aus beiden Halbfenstern heraus ist — und er wird dann mit
  **korrekter** Geschwindigkeit final (Regressionstest für den Fehler aus Z9)
- Pool wird mit der Bahnlänge genauer; ein langer Abschnitt zählt mehr als ein
  kurzer
- einzelner Ausreißer wird verworfen, der Track bleibt final
- Pool über mehrere Klötze, `n_pool = 0` vor dem ersten finalen; Beiträge
  verschwundener Klötze bleiben
- Zeitstempel in der Größenordnung 1,7·10⁹ s ändern das Ergebnis nicht
  (Auslöschung)
- Position und Geometrie werden erst ab „final" gemittelt — ein umgekippter
  Klotz meldet danach seine neue Höhe und seine neue Lage
- Winkelmittelung bei 0°/90°-Sprüngen: nahe 0° oder 90°, nie 45°
- Verfall entfernt den Track, nicht den Pool-Beitrag

## Bewusste YAGNI-Entscheidungen

- **Kein Kalman-Filter.** Bei konstanter Bandgeschwindigkeit konvergiert er zur
  selben Lösung wie die Ausgleichsrechnung, braucht aber zwei Rauschparameter zum
  Abstimmen — und das Umkippen verletzt sein Modell genauso.
- **Kein Schätzwert über Durchläufe hinweg.** Der Kaltstart ist nach 1,3 s
  überstanden (Punkt 3); ein gemerkter Wert brächte Persistenz ohne Nutzen.
- **Keine Re-Identifikation verlorener IDs.** Ein Track-Abriss führt zu einer
  neuen ID; der `priority_handler` wählt dann neu.
- **Keine z-Glättung über die Bandgeometrie** — der Mittelwert genügt.
- **Keine eigene Plausibilitätsprüfung auf Festhängen oder Anstoßen.** Beides
  gibt es nicht (Nachtrag 6 / Z3).
