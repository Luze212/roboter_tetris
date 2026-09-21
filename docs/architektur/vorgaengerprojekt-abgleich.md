# Abgleich mit dem Vorgängerprojekt (`UR10_Pick_ws`)

**Stand 14.09.2026.**
**Quelle:** `/home/tetripick/UR10_Pick_ws` — das Archiv der Vorgängergruppe, in der
übrigen Doku als `FuE_Greifen-main` bezeichnet. **Read-only, es wird dort nichts verändert.**
**Zweck:** Systematischer Durchgang durch `offene-punkte.md` — was lässt sich aus dem
Archiv beantworten, was nicht, und welches Know-how ist übertragbar.

> **Abgrenzung.** AICA ist für uns Vorgabe, der Aufbau weicht dadurch zwangsläufig ab.
> Die Vorgängerlösung ist C++/OpenCV + ZeroMQ/Protobuf + Python-Zustandsmaschine über
> `ur_rtde` — **kein** Architekturvorbild und kein Vergleichsmaßstab. Es geht hier
> ausschließlich um **Messwerte, Umsetzungsdetails und Erfahrungswissen**.
>
> Ergänzt `robot-cam-befunde.md` §6, das dieselbe Quelle für die Roboterkamera auswertet.

---

## 1. Warum der Architekturteil nichts hergibt

Die Vorgängerlösung ist eine blockierende Zustandsmaschine
(`Robot/base_strat2.py`): `IDLE → MOVE → OBJECT_WAITING → GRIP → PLACE`, jeder
Fehlerfall führt über „Greifer auf, Home anfahren" zurück nach `IDLE`. Bewegungen
sind `moveL`/`moveJ_IK` über `ur_rtde`.

Der eigentliche „Griff im Lauf" ist **Koppelnavigation, keine Regelung**
(`Robot/object_waiting_handler_strat2.py:136`):

```
total_travel_time = ((distance_y + DISTANCE_TO_CAMERA) / v_band) + 0.7
stop_time         = total_travel_time - (jetzt − aufnahmezeitpunkt)
time.sleep(stop_time)        # danach wird blind zugegriffen
```

Der Roboter wechselt die Spur in x, wartet auf fester y-Position und **schläft**,
bis der Klotz rechnerisch da ist. Es gibt keine Rückkopplung während des Griffs
und keine Zielauswahl — verarbeitet wird immer `objs[0]`, das erste Objekt der
Liste (`Robot/idle_handler_strat2.py`).

**Folgen für uns:**
- Für `priority_handler`, `vectoring` und den Regelteil des `object_follower`
  existiert **kein Vorbild** im Archiv. Diese Komponenten sind ohne Vorlage.
- `time.sleep()` in der Ablauflogik ist unter AICA ohnehin verboten
  (`ARCHITECTURE.md` §3).
- Die Zustandsfolge selbst deckt sich inhaltlich mit unserer
  (`WARTEN/ANFAHREN/FOLGEN/…`), inklusive des Musters „jeder Fehler → Greifer auf,
  definierte Pose anfahren, von vorn" — das ist unser `ABBRUCH`. Insofern eine
  **Bestätigung des Schnitts**, mehr nicht.

---

## 2. Beantwortete offene Punkte

| Punkt | Befund | Fundstelle | Sicherheit |
|---|---|---|---|
| **C1 / C4** | **Die Hand-Auge-Kalibrierung ist Flansch → Kamera.** Damit ist die offene Hälfte („meint `mount` den Flansch oder den Greifpunkt?") geklärt. Werte direkt als `handeye_*` verwendbar: x = 0,1087 · y = −0,03436 · z = −0,05987 m; rpy = (0,02898 · 0,02722 · 1,597) rad ≈ (1,66° · 1,56° · 91,5°). | Beweiskette unten | **belegt** |
| **A6** | Beide Kameras liefen mit **640×480 @ 30 fps** (Color BGR8, Depth Z16). Der in Thema 2 angenommene Kameratakt von ~30 Bildern/s ist damit plausibilisiert. | `cameras/camera_reader_base.cpp:72–73` | belegt für den *alten* Aufbau |
| **B16** | Greiferhub wurde auf **0…130 mm** kalibriert (`GRIPPER.calibrate(0, 130)`), Bezug 2F-140. Brauchbarer Startwert für `max_gripper_opening_m`. | `Robot/base_strat2.py:35` | belegt |
| **B10** | Der Arbeitsraum war nicht nur notiert, sondern **wurde vor jeder Bewegung geprüft** (`save_pos.is_save_position`, Rechteck-Test — dieselbe Form wie unsere Gate-Prüfung 5). Werte: X −0,05…1,05 · Y −0,8…0,3 · Z 0,095…0,37 m. | `Robot/pose.yaml`, `Robot/save_pos.py` | belegt, **aber TCP-Bezug**, siehe Falle 5 |
| **B9** | Ablagepositionen sind geteacht vorhanden, auch farbgetrennt: `Move_to_Place` [0,063 · −0,78 · 0,22], `Place_blue` [0,086 · −0,602 · 0,200]. Beide liegen im obigen Arbeitsraum. | `Robot/pose.yaml` | belegt, als Anhaltspunkt |
| **C9 / A7** | Bestätigt sich: kommandiert wurden **TCP-Posen** (`getActualTCPPose` / `moveL`). Das beantwortet A7 aber **weiterhin nicht** — siehe Abschnitt 4. | `Robot/Robot.py:109`, `Robot/move_handler_strat2.py` | erschlossen |

### Beweiskette zu C1/C4

Drei unabhängige Stellen, alle in dieselbe Richtung:

1. `ros2_ws/src/get_pose.py:54` — die Roboterposen für die Kalibrierung werden mit
   **`getActualToolFlangePose()`** aufgenommen; `getActualTCPPose()` steht direkt
   darüber **auskommentiert**. Es wurde also bewusst der Flansch gewählt.
2. `ros2_ws/src/convert_poses.py` liest genau diese Dateien aus `poses/` und
   erzeugt daraus die Eingabe für `industrial_calibration`, das
   `Calibration_results_final.yaml` schreibt (Format mit `converged` /
   `initial_cost_per_obs` stammt aus dieser Bibliothek, nicht aus einem eigenen Skript).
3. `ros2_ws/src/industrial_calibration_ros2/launch/data_collection.launch.xml:5` —
   `camera_mount_frame` steht per Default auf **`tool0`**, dem ROS-Standardnamen des
   UR-Flansches.

**Rest-Unsicherheit:** Punkt 3 ist ein Launch-Default und ließe sich beim Aufruf
überschreiben. Punkt 1 ist dagegen hart im Skript und trägt den Befund allein.

---

## 3. Anhaltspunkte ohne Beweiskraft — Größenordnungen

Nützlich, um Parameter **vorzubelegen** statt bei null zu starten. Keiner dieser
Werte ist auf unseren Aufbau übertragbar, weil die Kette (ZeroMQ + Python +
`moveL` gegen AICA + Attractor + IK-Controller) eine völlig andere ist.

| Unser Parameter | Anhaltspunkt | Herkunft |
|---|---|---|
| `latency_compensation_s` (D7) / `lead_time_s` (D1, vorher `lead_offset_m`) | **≈ 0,7 s** Gesamtvorhalt — seit Nachtrag 6 / Z6 ist auch unser Vorhalt eine **Zeit**, die Zahl ist also unmittelbar vergleichbar — die `+ 0.7` in der Timing-Formel, im Code kommentiert als „dein Stellrad für früher/später greifen". Fasst Kamera-, Verarbeitungs- und Bewegungslatenz in einer empirisch eingedrehten Zahl zusammen. | `object_waiting_handler_strat2.py:136` |
| `settle_half_window` in `vectoring` (Default 15, also frühestens nach 30 Messungen final) | Die Vorgängergruppe verwarf die **ersten 7 Messungen** einer Spur und mittelte erst über die Messungen 7…12. Das ist im Kern dieselbe **Einschwingphase**, die Nachtrag 6 / Z3 einführt — direkt nach dem Auflegen ist die Geschwindigkeit noch nicht verlässlich. Unser Kriterium wartet länger (zwei übereinstimmende Halbsekunden statt einer festen Zahl), weil es das Umkippen sicher abfangen muss (Z9). | `idle_handler_strat2.py:64–68` |
| ~~Plausibilitätsschwelle in `vectoring`~~ | Objekte unter **0,05 m/s** wurden als „läuft nicht" verworfen. Die Plausibilitätsprüfung ist entfallen (Nachtrag 6 / Z3), der Befund bleibt als **Untergrenze**: Die Bandgeschwindigkeit liegt deutlich über 0,05 m/s — und damit auch über der 30-mm/s-Totzone des Trackers. | `idle_handler_strat2.py:64` |
| `gripper_margin_m` (Default 0,01) | Vorpositionierung war **Blockbreite + 30 mm**. Andere Bedeutung als unsere Marge (Vorpositionierung vs. Greifbarkeitsprüfung), aber die Größenordnung der nötigen Luft. | `idle_handler_strat2.py:76` |
| `timeout_grasp_s` | Nach `close()` wurde **0,3 s** gewartet, bevor angehoben wurde — mit dem Kommentar „evt. anpassen!". | `grip_handler_strat2.py` |
| `has_object` (S9) | Erfolgskontrolle war **`gripper.getPositionmm() < 10` ⇒ Fehlgriff** (Greifer ganz zu = nichts drin). Ein brauchbarer Gegencheck zum Robotiq-Predicate `is_object_grasped`. | `grip_handler_strat2.py:62` |
| Greifzone (B19) | Der Tracker löschte Objekte außerhalb **y = +375 … −1080 mm** (Bandlänge im Roboter-Frame) und maß Geschwindigkeit nur im Fenster **y = −1000 … −500 mm**. Gleiches Frame wie bei uns, also direkt als Orientierung brauchbar. | `cameras/tracker.hpp` |

**`v_band` selbst (B1/C5) steht nirgends als Zahl im Archiv.** Sie wurde zur
Laufzeit aus dem Tracker bezogen (`vy`, mm/s) und nie festgeschrieben. Genau das
entspricht dem Ansatz von Nachtrag 6: Die Geschwindigkeit wird zur Laufzeit
geschätzt, nicht festgeschrieben. B1 ist damit keine Lücke mehr, sondern nur noch
die Gegenprobe.

---

## 4. Was das Archiv **nicht** beantworten kann

| Punkt | Warum nicht |
|---|---|
| **C8** — Werkzeugversatz Flansch → Greifpunkt | Die Gruppe fuhr über `getActualTCPPose()` / `moveL()`. Der Werkzeugversatz saß damit in der **UR-Installation am Teach-Pendant**, nicht im Code. Im gesamten Archiv gibt es kein `set_tcp` / `setTcp` / `setPayload`. **Der Wert muss aus der Robotersteuerung selbst kommen.** |
| **A7** — steht der Greifer im URDF? | Im Archiv existiert überhaupt kein URDF (`ros2_ws/src` enthält nur die Kalibrierpakete). Die Vorgängergruppe hatte keine URDF-basierte Kette. Bleibt in AICA Studio zu prüfen. |
| **A1–A5, A8–A10** | AICA-spezifisch (Attractor, `robot_state_broadcaster`, Hardware-Interface). Im Archiv gibt es nichts Vergleichbares. |
| **B13** — Uhrendrift Kamera ↔ ROS | Die Vorgängergruppe stempelte mit `time.time()` beim Empfang der ZMQ-Nachricht, nutzte also nie den Kamerastempel. Das Problem konnte dort gar nicht auftreten — und ist deshalb auch nicht vorgelöst. |
| **B1** — Bandgeschwindigkeit und -richtung | siehe oben, nie festgeschrieben. |
| **C2** — Intrinsik | siehe Falle 4. |

---

## 5. Übertragbares Know-how

Dinge, die die Vorgängergruppe auf die harte Tour gelernt hat und die bei uns
dieselbe Fehlerklasse betreffen.

**Veraltete Daten waren ein reales Problem.** Vor jedem Greifvorgang wurde der
Empfangspuffer aktiv leergeräumt („Leere alten Daten-Puffer rigoros…",
`object_waiting_handler_strat2.py`), weil sonst auf Bildern von vor mehreren
Sekunden gegriffen wurde. Bei uns ist das die Begründung für Regel 7 der
Datenverträge (Gaten auf `t` bzw. `seq`) und für die Gate-Prüfungen 2/3 des
`object_follower` — es ist kein theoretisches Risiko.

**Die geneigte Beobachtungspose hat sie Genauigkeit gekostet.** Bei 25° Neigung
mussten sie den Versatz von Hand herausrechnen:
`y_versatz = BAUTEIL_HOEHE · tan(25°)` — mit `BAUTEIL_HOEHE = 0.05` als
**fester Konstante**, obwohl die Klötze unterschiedlich hoch sind
(`object_waiting_handler_strat2.py:76–108`). Genau dieser Fehler ist die
Begründung unserer Festlegung „Beobachtungspose senkrecht" (Thema 6) — hier steht
er als gelebter Code.

**Der Rest-Versatz wurde nie sauber weg.** `MANUELLER_X_OFFSET = -0.03`,
`BILD_MITTE_X = 0.839255`, dazu zwei Drift-Faktoren für „links/rechts", beide auf
`0.0` stehen geblieben — Spuren einer Handabstimmung, die nicht konvergiert ist.
Unser Gegenstück ist die **gefilterte Korrektur** aus der Roboterkamera
(Thema 6): derselbe Versatz, aber gemessen statt geraten.

**Das Quadrat-Problem war bekannt und blieb ungelöst.** Die Protobuf-Nachricht
führt ein Feld `bool square`, und in `cameras/camera_static.cpp:148–150` steht die
beabsichtigte Behandlung (`orientation mod π/2`) **auskommentiert**. Das ist
dieselbe Falle, die unsere Winkelmittelung über den verdoppelten Winkel adressiert
— siehe dazu aber Abschnitt 7, wir sind dort weiter als gedacht.

**Zwei Konstanten gleichen Namens mit verschiedenen Werten.**
`DISTANCE_TO_CAMERA` ist in `move_handler_strat2.py` aus der YAML geladen
(`camera_mount_to_camera.y` = −0,03436) und in
`object_waiting_handler_strat2.py:9` hart auf `0.1087` gesetzt — also den
**x**-Wert derselben Kalibrierung. Mindestens eine der beiden Stellen ist falsch.
→ **Keine abgeleiteten Konstanten aus dem Archiv übernehmen, nur die YAML-Werte**,
und die Transformation selbst aufbauen.

---

## 6. Fallen

Die Fallen **1 bis 3** stehen bereits in `robot-cam-befunde.md` §6:
`conveyor_z_dist: 1000` als unbenutzter Altwert (1), die fixe TCP-Höhe in
`move_handler_strat2.py` (2) und die widersprüchliche Wartepose 0,1 m / 0,15 m (3).
Die Nummerierung wird hier fortgesetzt.

**Falle 4 — die Intrinsik in `ros2_ws/src/config.yaml` ist vertauscht.**

```yaml
fx: 324.703   fy: 241.537     # das sind Hauptpunkte für 640×480
cx: 598.554   cy: 598.359     # das sind Brennweiten
```

Bei 640×480 sind ≈(324,7 · 241,5) ein plausibler Hauptpunkt und ≈598 eine
plausible Brennweite — die Felder sind also gegeneinander vertauscht beschriftet.
Folgenlos geblieben, weil `perform_intrinsic_calibration: true` gesetzt war und
die Werte nur als Startschätzung dienten. **Als Intrinsik für C2 unbrauchbar.**

**Falle 5 — der Arbeitsraum aus `pose.yaml` gilt für den TCP, nicht für den Flansch.**
`Safety/README.md` definiert `workspace_bounds.json` ausdrücklich für den
**Roboterflansch**. Die Vorgängerwerte wurden gegen `getActualTCPPose()` geprüft,
liegen also um den Werkzeugversatz (C8) versetzt — vor allem in z
(`Z_min = 0,095` ist eine Greifpunkt-, keine Flanschhöhe). Übernahme nur mit
Umrechnung und dokumentierter Begründung (Änderungsregel in `Safety/README.md`).

**Falle 5b — und womöglich in einem gedrehten Rahmen (Nachtrag 8 / F1, B23).** Die
Vorgängergruppe las ihre Posen über RTDE (`getActualTCPPose`,
`getActualToolFlangePose`). RTDE meldet im UR-Rahmen **`base`**; ROS und damit AICA
arbeiten in **`base_link`**, und im UR-URDF sind beide um **180° um z** gegeneinander
gedreht. Das würde erklären, warum ihr Arbeitsraum-Rechteck und die alten
Kamerawerte (x ≈ +0,8) nur gedreht zu unseren Antastpunkten (x ≈ −0,8) passen. Die
alte Extrinsik der Basiskamera ist aus solchen Posen entstanden. **Am Aufbau
prüfen, bevor irgendein Vorgängerwert übernommen wird.**

---

## 7. Ein Befund am eigenen Bestand: das Quadrat-Problem

Beim Abgleich des `square`-Felds aus ihrer Protobuf-Nachricht gegen unseren
Python-Port ist aufgefallen: **wir sind dort bereits weiter als das C++-Original.**

### Worum es geht

Die Bildverarbeitung legt um jeden Klotz das kleinstmögliche gedrehte Rechteck
(`minAreaRect`) und gibt dessen Winkel aus. Daraus baut der Greifer später seine
Handgelenkstellung.

Bei einem länglichen Klotz ist das eindeutig. Bei einem **fast quadratischen** —
etwa 50 × 54 mm — nicht: Mal ist die 54er-Seite ein Pixel länger, mal durch
Rauschen die 50er. Der ausgegebene Winkel springt deshalb von Bild zu Bild
zwischen ~0° und ~90°, obwohl der Klotz still liegt. Wer das arithmetisch
mittelt, landet bei 45° — genau der Stellung, in der die Backen die Ecken
erwischen.

### Was die Vorgängergruppe daraus gemacht hat

Ihre Nachricht führt ein Feld `bool square`, und in `cameras/camera_static.cpp`
steht die beabsichtigte Behandlung (`orientation mod π/2`) **auskommentiert**.
Erkannt, nicht zu Ende gebracht.

### Was unser Port daraus gemacht hat

Fertig gebaut, an zwei Stellen:

- `vision/detection.py` setzt `square = True`, sobald das Seitenverhältnis über
  **0,92** liegt.
- `vision/tracker.py` (`_resolve_orientation`) **behält für solche Objekte den
  zuerst gemessenen Winkel**, statt bei jedem Bild den neuen zu übernehmen.

Das Springen ist damit unterdrückt, bevor es `base_cam` verlässt.

### Folge 1 — ein Lehrbeispiel in den Specs stimmt nicht mehr

`vectoring` sollte über den verdoppelten Winkel mitteln, und die Resultantenlänge
sollte als Gütemaß dienen: nahe 1 = Messungen einig, nahe 0 = Winkel flattert.
Als Beispiel nennen die Specs durchgehend den quadratischen Klotz.

**Genau bei dem schlägt das Gütemaß nie an**, weil der Winkel konstant
hereinkommt. Mittelung und Gütemaß bleiben richtig und nötig — die
π-Periodizität betrifft jede Winkelmittelung, und das Gütemaß wirkt weiter gegen
Rauschen, Teilverdeckung und Objekte am ROI-Rand. Falsch war nur das Beispiel.

Praktisch zählt das bei **D11**: Wer die Schwelle an einem quadratischen Klotz
einzustellen versucht, wartet vergeblich auf einen Ausschlag.

→ Korrigiert in `entscheidungen.md` (Thema 6), der `vectoring`-Spec und dem
Umsetzungsplan (3.1).

### Folge 2 — `square` selbst: geprüft, bewusst nicht in den Vertrag

Das Merkmal wird in `base_cam` berechnet und dann verworfen; S1 kennt es nicht.
Die Aufnahme als zehntes Feld lag nahe, wurde geprüft und **verworfen**:

1. **Die Information steckt schon im Vertrag.** `length` und `width` liefern
   dasselbe über `min/max ≥ 0,92`. Nicht bitgleich — `detection.py` misst am
   unerodierten Pixelrechteck, `length`/`width` stammen aus der erodierten
   3D-Kontur — für eine Schwellenentscheidung aber gleichwertig.
2. **Kein Verbraucher braucht es.** „Fast quadratisch" heißt per Definition, dass
   sich die beiden Abmessungen um **weniger als 8 %** unterscheiden. Mehr kann
   eine um 90° vertauschte Achszuordnung nicht anrichten. Bei Klötzen von
   25…76 mm gegen rund 130 mm Öffnung deckt das die Greifermarge ab.
3. **Der Preis wäre unverhältnismäßig.** Stride 9 → 10 zwingt `datenvertraege.md`,
   `contracts.py`, `base_cam` und **jeden** Verbraucher zur gleichzeitigen
   Umstellung. Wer zurückbleibt, liest ab dem zweiten Objekt Unsinn.

Statt des Vertragsumbaus eine Zeile im `priority_handler`: Liegt das
Seitenverhältnis über 0,92, wird `max(length, width)` gegen die Greiferöffnung
geprüft statt der zugeordneten Abmessung. Konservativ, lokal, kostenlos.

→ Festgehalten in `datenvertraege.md` unter S1 und S3.

> **Überholt beim Bau (Nachtrag 7 / H2):** Der `priority_handler` prüft die
> **Diagonale** gegen die Greiferöffnung. Die gilt für jeden Gierwinkel und ändert
> sich durch vertauschte Achsen nicht — die Zeile mit der 0,92-Schwelle ist damit
> entbehrlich. Die Begründung gegen ein Feld `square` bleibt.
