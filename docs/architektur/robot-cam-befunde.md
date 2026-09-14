# Befunde zur Roboterkamera — Szene, Physik, verworfene Wege

**Stand 14.09.2026.**
**Quelle:** die Laptop-Sitzung, in der `robot_cam` und `robot_cam_2` entstanden sind.
**Zweck:** Wissen festhalten, das **in keinem Repo und in keinem Archiv steht** —
es stammt aus Versuchen an den realen Kameras und aus der Rücksprache mit der
Vorgängergruppe.

> Dieses Dokument erklärt das **Warum** hinter `robot_cam` / `robot_cam_2`. Der
> Ist-Stand der Parameter steht in den beiden JSONs unter
> `component_descriptions/` — die sind die verbindliche Quelle, nicht dieses
> Dokument.

---

## 1. Szene und Material

Ohne diese Fakten sind die Default-Parameter der Farberkennung nicht
nachvollziehbar und nicht sinnvoll nachzuziehen.

| Eigenschaft | Wert |
|---|---|
| Förderband | **grün**, durchgehend |
| Klötze | **rot, blau, weiß** |
| Oberseite der Klötze | **matt/rau** (3D-Druck-Schichten nach oben) |
| Seitenflächen der Klötze | **glatt und spiegelnd** (3D-Druck) |
| Spiegelungen im Bild | treten **nur auf dem Band** auf, nicht auf den Klötzen |
| Beispielklotz | 2,5 × 5,1 × 7,6 cm |

**Daraus folgt der gesamte Ansatz von `robot_cam`:** Das grüne Band ist ein
sauberer Chroma-Key. Man muss nicht die Klötze suchen, sondern nur **eine** Farbe
sicher klassifizieren (Grün) und den Rest behalten. Deshalb heißen die Parameter
`belt_h_min` / `belt_h_max` (Default 35–85, Grün liegt bei OpenCV-H um 60) und
nicht etwa `block_color_*`.

**Und deshalb funktioniert Farbe hier überhaupt:** Die matte Oberseite liefert ein
glanzfreies Farbfeld. Bei einer glänzenden Oberseite wären Glanzlichter
entsättigt und die HSV-Maske bräche zusammen.

> **Kalibrier-Gegenprobe:** Mit `conveyor_z_dist = 865` mm misst `base_cam` den
> hochkant stehenden Beispielklotz zu **76,6 mm** (Soll 76 mm). Der Wert 865 ist
> damit am realen Aufbau bestätigt — nicht geraten.

---

## 2. Warum der Loch-Trick verworfen wurde

Die Vorgängergruppe hat die Klötze über den **Tiefenwert 0** segmentiert: Die
Kamera wurde so positioniert, dass das Band gerade noch im Messbereich lag,
alles darüber aber herausfiel. Das ist in `projektkontext.md` §2 beschrieben.

**Das Verfahren ist bei uns nicht reproduzierbar — aus einem physikalischen
Grund, nicht aus einem Einstellungsfehler:**

1. **Die Tiefenkamera sitzt seitlich versetzt zur RGB-Kamera.** In Greifnähe
   blickt sie dadurch schräg auf den Klotz und sieht dessen **Seitenfläche**.
2. **Die Seitenflächen sind spiegelnd.** Der LiDAR-Sensor bekommt dort kein
   Rücksignal und liefert ebenfalls **0** — also dasselbe Signal wie „zu nah".
3. Das Loch besteht damit aus **Deckfläche + parasitärer Seitenfläche**, und die
   Seitenfläche ist durch die Parallaxe zusätzlich **seitlich verschoben**.

Das Ergebnis ist ein horizontal verzogener Umriss: In Tests erschien ein
quadratischer Körper als etwa **1,5-fach zu breites Rechteck**, nach einer Seite
überhängend, während auf der anderen Seite rund 20 % des Körpers unbedeckt
blieben.

**Wichtig für die Fehlersuche:** Der Verzug liegt **sowohl im rohen Tiefenbild
als auch im aligned-Tiefenbild**. Er ist also *kein* Artefakt der Registrierung
und nicht durch Umschalten der Tiefenquelle zu beheben.

**Die Vorgängergruppe ist mit ihrer eigenen Lösung selbst unzufrieden** (per
Rücksprache bestätigt). Der Loch-Trick ist also kein bewährtes Verfahren, das wir
nur falsch anwenden.

### Die Höhenabhängigkeit — der praktisch wichtigste Punkt

**Der Verzug verschwindet, sobald die Kamera höher steht.** Das ist am Aufbau
empirisch bestätigt. Geometrisch plausibel: Der Verdeckungsschatten der
Seitenfläche schrumpft mit wachsendem Abstand, weil der Blickwinkel auf die
Seite flacher wird.

> **Das ist die fehlende Begründung zu B8.** `offene-punkte.md` hält korrekt
> fest, die Beobachtungshöhe sei „nicht frei wählbar" — der Grund steht hier:
> Zu tief ⇒ Seitenflächen-Artefakte und verzogene Geometrie. Zu hoch ⇒ zu wenig
> Auflösung auf dem Klotz. Das Fenster dazwischen ist das, was **D18**
> (`min_belt_distance_m` / `max_belt_distance_m`) abstecken soll.

---

## 3. Ist-Stand der beiden Varianten

Beide Komponenten haben **identische Ein-/Ausgänge** und sind im Graphen
gegeneinander austauschbar. Nur der Detektionskern unterscheidet sich.

**Gemeinsame Inputs:** `color_image`, `color_camera_info`, `aligned_depth_image`.
Aligned Depth ist Pflicht — das Nah-Gate muss pixelgenau über dem Farbbild
liegen. Die Komponente skaliert die Intrinsik und resized die Tiefe per Nearest
auf die Farbauflösung, falls sie abweicht.

| | `robot_cam` | `robot_cam_2` |
|---|---|---|
| Was definiert den Klotz | **Farbe** (grünes Band ausmaskieren) | **Gradient** (Canny-Kanten) |
| Zusatzquelle | — | Tiefenkante, fusioniert (`use_depth_edges`) |
| Stärke | sattes Farbfeld, tolerant gegen Lücken | unabhängig von Farbe |
| Schwäche | grünstichige Klötze | schwacher Helligkeitskontrast (weiß auf grün) |

**Der gemeinsame Kern** (`localize_largest_blob` in
`vision/robot_detection.py`) ist bei beiden **derselbe Code**: Blob-Auswahl,
Nah-Gate, `minAreaRect` für Mittelpunkt und Orientierung, Band-Distanz,
Rückprojektion. Nur die Kandidatenmaske wird unterschiedlich gebaut. Das ist
Absicht — nur so vergleicht der A/B-Test wirklich *Farbe gegen Kante* und nicht
zwei verschiedene Geometriepfade.

### Das Nah-Gate

Trennt einen **weißen Klotz** von einer **Spiegelung auf dem Band** — der einzige
Fall, den Farbe allein nicht lösen kann:

- Eine Bandspiegelung liefert weiterhin **gültige Band-Tiefe** → kein Nah-Signal
- Ein echter Klotz liefert **0** (zu nah/spiegelnd) **oder** eine Tiefe deutlich
  über dem Band → Nah-Signal

Konkret: `near = (depth == 0) | (depth < median(gültige Tiefe) − 20 mm)`. Ein
Blob zählt nur, wenn er diese Region zu **mindestens 10 %** überlappt. Beides ist
im Code als Konstante hinterlegt, nicht als Parameter.

---

## 4. Verworfene Optionen — mit Begründung

Damit die Umsetzungssitzung diese Wege nicht erneut vorschlägt.

| Verworfen | Grund |
|---|---|
| **Loch-Trick als Primärverfahren** | Abschnitt 2 — physikalische Ursache, nicht behebbar durch Parameter |
| **Kanten ∩ Nah-Region** (Schnittmenge als finale Maske) | Die spiegelnde Seitenfläche gehört zur Nah-Region. Der Schnitt würde die Maske genau dort **aufblähen**, wo sie falsch ist. |
| **Canny auf dem rohen Tiefenbild** | Die Tiefe schwankt am Aufbau massiv → lauter Störkanten. Stattdessen: Kante = **Rand der bereits geschwellten Nah-Region**, das ist rauschrobust. |
| **`border_filter_mode` / `reference_point_mode`** | Parameter des alten Loch-Trick-Ports, im Farb-/Kantenansatz bedeutungslos → entfernt |
| **Farbklassifikation in `robot_cam`** | Die Farbe bestimmt bereits `base_cam`. Doppelt wäre Rechenzeit ohne Gewinn. |

### Eine Reserve, falls die Höhe nicht anhebbar ist

Die Vorgängergruppe hat den Verdeckungsschatten **umgangen**, nicht gelöst: Sie
filterte auf Konturen am **oberen Bildrand** (der Klotz fährt von oben ins Bild)
und nahm die **Unterkante** des Lochs als Referenz — die vordere Kante beim
Reinfahren ist schattenfrei. Falls sich die Beobachtungshöhe am Aufbau nicht weit
genug anheben lässt, ist das der erprobte Ausweichweg.

---

## 5. RealSense-Konfigurationsfallen

Kosten am Aufbau real Zeit, wenn man sie nicht kennt.

| Falle | Symptom | Abhilfe |
|---|---|---|
| **Depth/Color-Profile leer** im AICA-RealSense-Block | Kamera läuft mit Default **424×240** statt 848×480 | Beide Profilfelder explizit setzen |
| **„Enable alignment" aus** (Default) | `aligned_depth_image` existiert nicht, Topic bleibt leer | Schalter aktivieren |
| Aligned Depth in **anderer Auflösung** als Color | Overlay versetzt/abgeschnitten | Von den Komponenten bereits abgefangen (Intrinsik-Skalierung + Nearest-Resize) |

Relevant für **A6** (tatsächliche Kamerarate).

---

## 6. Werte aus dem Vorgängerprojekt — und zwei Fallen

Das Archiv `FuE_Greifen-main` liegt **auf dem Projektrechner** unter
`/home/tetripick/UR10_Pick_ws` und ist dort auslesbar (**read-only**). Es ist die
verbindliche Quelle; die folgenden Werte sind nur eine Abkürzung, damit die Suche
nicht wiederholt werden muss.

> **Der vollständige Durchgang durch alle offenen Punkte steht in
> `vorgaengerprojekt-abgleich.md`.** Dieser Abschnitt hält nur fest, was die
> Roboterkamera betrifft.

| Gesucht | Wert | Fundstelle |
|---|---|---|
| Hand-Auge Roboterkamera (**C1/C4**) | `camera_mount_to_camera`: x = 0,1087 · y = −0,03436 · **z = −0,05987** (m), rpy = (0,02898 · 0,02722 · 1,597) rad ≈ (1,66° · 1,56° · **91,5°**). **Bezug geklärt: Flansch → Kamera** (Beweiskette in `vorgaengerprojekt-abgleich.md` §2) — damit direkt als `handeye_*` des `object_follower` verwendbar. | `Robot/Calibration_results_final.yaml` |
| Wartepose Roboterkamera (**B8**) | `Kamera_2_Kalib`: die **aktive** Zeile ist TCP **Z = 0,15 m** mit *senkrechter* Orientierung; die Zeile mit **Z = 0,1 m** und geneigter Haltung ist **auskommentiert** (siehe Falle 3). | `Robot/pose.yaml` |
| Arbeitsraum-Indiz (**B10**) | Workspace **vollständig**: X −0,05…1,05 · Y −0,8…0,3 · Z 0,095…0,37 (m). Wurde vor jeder Bewegung geprüft (`Robot/save_pos.py`). ⚠️ **TCP-Bezug, nicht Flansch** — siehe `vorgaengerprojekt-abgleich.md` Falle 5. | `Robot/pose.yaml` |
| Bandmitte | `KAMERA_MITTE_X` = 0,418329 | `Robot/move_handler_strat2.py` |
| Kommandiertes Frame (**C9/A7**) | Es wurden **TCP-Posen** kommandiert (`target_tcp`, `getActualTCPPose`) | `Robot/move_handler_strat2.py`, `Robot/Robot.py:109` |
| Werkzeugversatz (**C8**) | **Nicht im Archiv.** Der Versatz saß in der UR-Installation am Teach-Pendant; es gibt kein `set_tcp`/`setPayload`. | — (`vorgaengerprojekt-abgleich.md` §4) |
| Kamerakonfiguration (**A6**) | Beide Kameras: **640×480 @ 30 fps**, Tiefe per `rs2::align` auf Farbe registriert | `cameras/camera_reader_base.cpp:72–73` |

### ⚠️ Falle 1 — `conveyor_z_dist: 1000`

`cameras/config_cam_robot.yml` nennt einen `conveyor_z_dist` von 1000 mm. Das
liest sich wie die Arbeitsdistanz der Roboterkamera. **`camera_robot.cpp`
verwendet den Wert nirgends** — die Erkennung arbeitet mit `depth == 0` und der
vor Ort gemessenen Banddistanz. Der Eintrag ist ein Altwert.
**Nicht als Arbeitsdistanz übernehmen.**

### ⚠️ Falle 2 — die TCP-Höhe ist fix, nicht berechnet

In `move_handler_strat2.py` steht `target_tcp[2] = KAMERA_2_KALIB_TCP_POS[2]`.
Die Zeilen, die die Höhe pro Objekt anpassen würden, sind **auskommentiert**. Die
Höhe ist also konstant — wer nur die aktive Zeile sieht, hält sie für einen
gerechneten Wert.

### ⚠️ Falle 3 — die 0,1 m sind die auskommentierte Pose

In `pose.yaml` steht unter `Kamera_2_Kalib` **zweimal** eine TCP-Pose: aktiv mit
**Z = 0,15 m** und senkrechter Orientierung, darunter auskommentiert unter
„Greifstrategie 2" mit **Z = 0,1 m** und geneigter Orientierung. Geladen wird von
`move_handler_strat2.py` die **aktive** Zeile, also 0,15 m.

Widersprüchlich dazu rechnet `object_waiting_handler_strat2.py:76–108` mit einem
festen Neigungswinkel von **25°**. Die beiden Dateien passen also nicht zusammen —
vermutlich wurde die YAML nachträglich geändert.

**Konsequenz für B8:** Der Anhaltspunkt lautet eher 0,15 m als 0,1 m — und weil
die Quellen sich widersprechen, bleibt er genau das: ein Anhaltspunkt. Die
Schlussfolgerung aus §8 ändert sich nicht, die Beobachtungshöhe **muss am Aufbau
eingemessen werden**.

---

## 7. Inbetriebnahme am Aufbau — Reihenfolge (B6)

Die Stufen bauen aufeinander auf. **Nicht überspringen** — jede Stufe schließt
eine Fehlerquelle aus, bevor die nächste beurteilbar wird. Alles ist am
Debug-Bild ablesbar; es wurde genau dafür gebaut.

> Zum Nachschlagen der Farben: Debug-Legende steht in der Kopfzeile des Bildes.
> `robot_cam`: grau = verworfene Kandidaten, blaue Fläche = Nah-Gate, grün =
> Detektion, gelb = Box, rot = Mitte/Winkel.
> `robot_cam_2` zusätzlich: cyan = Farbkante, orange = Tiefenkante.

### Stufe 0 — Signalweg (vor allem anderen)

1. Im AICA-RealSense-Block **beide Profilfelder** explizit setzen (sonst 424×240,
   §5) und **„Enable alignment" aktivieren**.
2. `debug_enable = true` setzen, Komponente starten.

**Abnahme:** Debug-Bild erscheint, `is_receiving_frames` = true, die Kopfzeile
zeigt plausible Zahlen. Erscheint kein Bild, fehlt meist `aligned_depth_image`.

### Stufe 1 — Farbmaske (nur `robot_cam`)

3. Einen Klotz ins Bild legen. Im Debug-Bild die **grauen** Kandidaten-Konturen
   beurteilen:

| Beobachtung | Bedeutung | Stellschraube |
|---|---|---|
| Bandflächen erscheinen als Kandidat | Grün-Maske **leckt** | `belt_h_min`/`belt_h_max` weiten, ggf. `belt_s_min` senken |
| Klotz erscheint gar nicht | Grünbereich zu weit, Klotz mit-entfernt | `belt_h`-Bereich einengen |
| Viele kleine Flecken | Rauschen | `morph_kernel_size` erhöhen, `min_contour_area` anheben |

**Abnahme:** Genau ein Blob auf dem Klotz, das Band ist sauber.

### Stufe 2 — Nah-Gate

4. Die **blau eingefärbte Fläche** prüfen: Deckt sie den Klotz?
5. Gegenprobe mit einem **weißen Blatt Papier auf dem Band**: Es muss verworfen
   werden (grau bleiben), ein **weißer Klotz** dagegen nicht.

**Abnahme:** Blatt wird verworfen, weißer Klotz erkannt. Greift das Gate gar
nicht, zum Isolieren kurz `use_depth_gate = false` setzen — erkennt die
Komponente dann, liegt der Fehler im Gate (Höhe/Tiefenbild), nicht in der Farbe.

### Stufe 3 — Höhe bestimmen ⚠️ der eigentliche Kernpunkt

6. Die Kamerahöhe **in Stufen variieren** und je Höhe festhalten:
   - Ist der Umriss horizontal verzogen? (§2 — Seitenflächen-Artefakt)
   - Stimmt die gelbe Box mit den echten Klotzkanten überein?
   - Ist der Klotz noch vollständig und groß genug im Bild?

**Ergebnis:** die untere und obere brauchbare Distanz → **D18**
(`min_belt_distance_m` / `max_belt_distance_m`) und die Beobachtungshöhe → **B8**.

> Das ist die Stufe, an der das Vorgängerprojekt gescheitert ist. Sie zuerst und
> gründlich zu machen ist die beste Zeitinvestition am Aufbau. Lässt sich keine
> verzugsfreie Höhe finden, siehe die Unterkanten-Reserve in §4.

### Stufe 4 — Geometrie-Gegenprobe

7. Klotz mit bekannten Maßen in bekannter Lage: Winkel (`ang=…deg` in der
   Kopfzeile) gegen die tatsächliche Lage prüfen, x/y gegen eine bekannte
   Bandposition.

**Abnahme:** Winkel stimmt auf wenige Grad, x/y plausibel. Systematischer
Winkelversatz von 90° ist harmlos — die Ausgabe ist auf [0, π) normiert und der
Verbraucher behandelt das.

### Stufe 5 — A/B gegen `robot_cam_2`

8. Gleiche Szene, **gleiche Höhe**, `robot_cam_2` dazuhängen. Beurteilen:
   - Trägt die **cyane Farbkante** den Umriss allein?
   - Wo springt die **orange Tiefenkante** ein? (erwartbar bei weiß auf grün)
   - Zum Vergleich einmal `use_depth_edges = false` — bricht der Umriss auf?

**Abnahme:** Eine begründete Entscheidung, welche Variante produktiv läuft. Beide
liefern denselben Vertrag; die Wahl ist eine Leitung im Graphen.

---

## 8. Was hier *nicht* bewiesen ist

Ehrlichkeitshalber, damit niemand auf zu dünnem Eis baut:

- **C9 ist aus zwei Stellen gelesen, nicht mit der Gruppe verifiziert.** Dass
  TCP-Posen kommandiert wurden, steht so im Code (`move_handler_strat2.py`,
  `Robot.py:109`). **A7 beantwortet das nicht** — und zwar aus einem klareren
  Grund als bisher angenommen: Im Archiv existiert **überhaupt kein URDF**, die
  Vorgängergruppe hatte keine URDF-basierte Kette. A7 ist in AICA Studio zu
  prüfen, eine Antwort aus dem Archiv ist nicht zu erwarten.
- **Die Kamera↔Band-Distanz der Vorgängergruppe ist unbekannt.** Die
  Bandoberflächen-Z im Roboter-Basis-Frame ließ sich in deren Projekt nicht
  finden. Aus TCP-Z = 0,1 m allein folgt sie nicht — erst recht nicht bei
  geneigter Kamera mit Montageversatz. **Die Beobachtungshöhe (B8) muss am
  Aufbau eingemessen werden**, die 0,1 m sind nur ein Anhaltspunkt.
- **Die Höhenabhängigkeit ist qualitativ bestätigt, nicht vermessen.** „Höher ⇒
  Verzug weg" ist beobachtet; ab welcher Distanz genau, ist offen — das ist
  genau der Inhalt von D18.
- **`robot_cam_2` ist am realen Aufbau noch ungetestet.** Die Kantenvariante
  inklusive Tiefenkanten-Fusion ist implementiert und mit synthetischen Szenen
  unit-getestet, aber noch nie gegen echte Klötze gelaufen (**B6**).
