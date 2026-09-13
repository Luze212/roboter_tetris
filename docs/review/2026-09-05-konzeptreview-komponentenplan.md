# Konzeptreview: Komponentenplan Robotetris

**Datum:** 2026-09-05
**Grundlage:** `docs/Komponentenplan Robotetris.docx` (Commit `0e74878`), `ARCHITECTURE.md`, Ist-Stand `source/roboter_tetris/`
**Scope:** Konzeptprüfung. Die Bildverarbeitung selbst (Algorithmik in `vision/`) ist ausdrücklich **nicht** Gegenstand — nur, wie ihre Ergebnisse ausgegeben und weiterverarbeitet werden.
**Status:** Review, keine Codeänderung. Rein lokal.

---

## 0. Gesamturteil

Der Plan ist inhaltlich weit gedacht und deckt die Aufgabe (farbige Klötze on-the-fly vom laufenden Band picken) in der Grundstruktur ab. Er hat aber drei Klassen von Problemen:

1. **Überarbeitungsartefakte** — dieselbe Sache heißt an zwei Stellen unterschiedlich, Parameter sind gelistet aber nie benutzt, Grenzwerte sind doppelt definiert. Mechanisch behebbar, aber zahlreich.
2. **AICA-Modellbrüche** — der Plan behandelt Predicates wie Signale und Objektlisten wie übertragbare Datentypen. Beides geht in AICA so nicht. Das betrifft fast jede Schnittstelle im Dokument.
3. **Konzeptionelle Lücken im zeitkritischen Kern** — Latenzkompensation, Zielauswahl nach Erreichbarkeit, der z-Abstieg beim Greifen, Abbruch-/Fehlerpfade. Das sind genau die Punkte, an denen ein On-the-fly-Pick in der Praxis scheitert, und sie sind die wahrscheinlichste Ursache für späteres Nachbessern.

Die Reihenfolge der Behebung sollte 2 → 3 → 1 sein: Modellbrüche legen die Schnittstellen fest, die Lücken bestimmen die Komponentengrenzen, die Artefakte sind Redaktionsarbeit am Ende.

---

## 1. Systematische Befunde (betreffen den gesamten Plan)

### S1 — Predicates werden durchgängig als Datensignale verdrahtet
**Betrifft:** `new object base`, `object changed`, `Objekt auf Ablage`, `Greifen`, `Greifer zu`, `Objekt im Bild 1`, `Objekt im Bild 2`, `Objekt Changed`

In AICA sind **Predicates Ereignisquellen**, keine Signale. Sie werden global gebroadcastet und lösen Events aus — und zwar ausschließlich auf der **steigenden Flanke** (`ARCHITECTURE.md` §12). Ein Predicate lässt sich nicht in einen Daten-Input einer anderen Komponente verdrahten. Wer einen bool als *Datenfluss* braucht, braucht auf der Senderseite einen `add_output(..., Bool)`.

Der Plan mischt beides durchgehend („Parameter Greifer zu" ist mal Input, mal Predicate). Konkret bereits im Code sichtbar: `robotiq_gripper` hat **gar keine Signal-Outputs**, nur die Predicates `is_connected`/`is_object_grasped` — der im Plan vorgesehene Input „Parameter Greifer zu" des `object_follower` ist damit heute nicht verdrahtbar.

**Festzulegende Regel:** Daten immer als Signal (`bool`/`double_array`/…). Predicates nur für (a) Lifecycle-/Studio-Events und (b) Diagnose in der UI. Wo beides gebraucht wird: beides ausgeben.

### S2 — Objektlisten sind kein übertragbarer Typ
`ARCHITECTURE.md` §9/§12: AICA überträgt **keine Listen komplexer Objekte**. Erlaubt sind Skalare, `state_representation`-Einzelobjekte und flache `double_array`s.

Der Plan spricht an neun Stellen von „Array mit Objekten" / „Objektliste" / „Geradenvektoren mit Zeitpunkt und ID", ohne ein einziges Mal das Packformat zu nennen. Ohne festen **Stride** laufen Sender und Empfänger garantiert auseinander, sobald jemand ein Feld ergänzt — und genau das ist in den letzten Wochen mit `t` in `base_cam` schon passiert (siehe B1).

**Empfehlung:** Ein gemeinsames Modul `roboter_tetris/contracts.py` mit Stride-Konstanten und `pack_*`/`unpack_*`-Funktionen. Jede Komponente importiert daraus; kein Empfänger indiziert von Hand. Das ist wenig Aufwand und eliminiert eine ganze Fehlerklasse.

### S3 — Einheiten sind nicht festgelegt
Vision rechnet in **mm** (`base_cam` Output, `robot_cam` Output), AICA/TF/JTC arbeiten in **Metern**. Der Plan sagt nirgends, wo umgerechnet wird. Ein einzelner vergessener Faktor 1000 ist hier ein Roboter, der ins Band fährt.

**Empfehlung:** Umrechnungsgrenze genau einmal definieren und im Vertrag dokumentieren. Vorschlag: alles ab `vectoring` in **Metern**, die Vision-Komponenten behalten mm (Ist-Stand) und die Umrechnung passiert an genau einer Stelle beim Entpacken.

### S4 — Zeitbasis und Latenz sind nicht adressiert
Der Plan nennt `self.get_clock()` und Frame-Zeitstempel, klärt aber nicht:
- Liefern die RealSense-Stempel **ROS-Zeit** oder eine eigene Kamerauhr? Bei Letzterem entsteht ein konstanter Offset, der die gesamte Prädiktion systematisch verschiebt.
- Der `object_follower` extrapoliert „mittels der aktuellen Systemzeit". **Das ist der falsche Zeitpunkt.** Die Zielkoordinate muss auf den Zeitpunkt prädiziert werden, zu dem der Roboter dort *ankommt* — also `t_jetzt + t_bewegung`, nicht `t_jetzt`. Bei 100 mm/s Band und 300 ms Anfahrtszeit sind das 30 mm systematischer Nachlauf, immer in dieselbe Richtung.
- Die gesamte Kette Belichtung → Transport → `base_cam` → `vectoring` → `priority` → `follower` → JTC → Bewegung ist nirgends budgetiert.

**Empfehlung:** (a) Zeitquelle verifizieren und dokumentieren; (b) Prädiktion konsequent auf einen *Zielzeitpunkt* statt auf „jetzt"; (c) einmal die reale Kettenlatenz messen und als Parameter `latency_compensation_s` führen.

### S5 — Zyklische Abhängigkeit über drei Komponenten
```
vectoring → prioritie_handler → data_tracker → vectoring
                             ↘ object_follower ↗
```
`vectoring` braucht die Objektliste von `data_tracker`; `data_tracker` braucht Geradenvektoren und die oob-Liste von `prioritie_handler`; `prioritie_handler` braucht die Vektordaten von `vectoring`.

Das ist kein Deadlock (AICA-Topics sind asynchron), aber ein Latenzring mit undefinierter Konvergenz: eine Änderung braucht drei Zyklen, bis sie zurückkommt, und in der Zwischenzeit arbeiten die drei Komponenten mit inkonsistenten Ständen. Debugging wird sehr unangenehm. Der Plan spürt das selbst — beide „Offene Fragen" in `vectoring` und `prioritie_handler` kreisen genau um diesen Ring.

### S6 — Drei Komponenten halten überlappenden Zustand
`vectoring` hält alle Objekte „unabhängig der Eingabe aus base_cam", `data_tracker` hält die Objektliste mit Flags, `prioritie_handler` hält das gelockte Zielobjekt samt Listenposition. Synchronisiert wird über Listenvergleiche.

Das ist die Hauptquelle für Inkonsistenzen und gleichzeitig unnötige CPU-Last auf einem System, das laut Vorgabe schon an der Grenze läuft. Zwei Listen zu vergleichen, um herauszufinden, was `object_follower` ohnehin genau weiß, ist verschenkte Rechenzeit.

### S7 — Keine Verfallsregel: unbegrenztes Listenwachstum
Weder `vectoring` („alle ermittelten Objekte werden gehalten") noch `data_tracker` („gepickt"/„out_of_bounds" werden nur *markiert*) entfernen jemals einen Eintrag. Über einen Testlauf mit hunderten Blöcken wächst das `double_array` monoton — und AICA publiziert Outputs **periodisch mit der Step-Rate**. Bei 50 Hz wird das zu echter Bandbreite und Serialisierungslast.

**Empfehlung:** Harte Verfallsregel — Objekte verschwinden aus dem Live-Array, wenn (gepickt ODER oob ODER seit X s kein Update). Historie, falls gebraucht, ins Log, nicht ins Signal.

### S8 — Kein einziger Fehler-, Timeout- oder Abbruchpfad
Der gesamte Plan beschreibt ausschließlich den Gutfall. Nicht behandelt:
- Greifer fasst daneben (`is_object_grasped` bleibt false)
- Zielobjekt geht während der Anfahrt verloren (Track-Abriss, Objekt fällt vom Band)
- „Position über V Frames konstant" tritt nie ein
- Objekt wird während des Greifens verschoben
- Not-Halt / Deaktivieren mit Objekt im Greifer
- Band schneller als der Roboter picken kann

Für ein System, das laut Zielsetzung *nicht nachgebessert* werden soll, ist das die größte praktische Lücke. Jede Phase des `object_follower` braucht einen Timeout und einen definierten Rückfallpfad.

### S9 — Safety kommt im Plan nicht vor
`source/roboter_tetris/roboter_tetris/Safety/workspace_bounds.json` existiert, ist aber Platzhalter (`status: placeholder_not_yet_defined`, alle Grenzen `null`) und wird im Komponentenplan mit keinem Wort erwähnt. `object_follower` gibt Zielkoordinaten aus, die ungeprüft an die Robotersteuerung gehen.

**Empfehlung:** Ein Clamp/Reject-Gate direkt vor der Ausgabe der Zielkoordinaten, gespeist aus den Workspace-Parametern. Zielposen außerhalb → verwerfen + Warnung + Rückfall in Home. Das ist billig und verhindert die teuerste Fehlerart.

### S10 — Namenskonsistenz
Im Dokument koexistieren: `roboter_cam` / `robot_cam` (Code), `objekt_follower` / `object_follower`, `prioritie_handler` (korrekt wäre `priority_handler`), „Objekt auf Ablage" / „Objekt abgelegt", `z_wait` / `z_find`, „Objekt Changed" / „object changed". AICA verlangt `lower_snake_case` für Signalnamen und exakte Übereinstimmung zwischen `setup.cfg`, JSON und Klassennamen — Tippfehler kosten hier Buildzeit.

---

## 2. Befunde je Komponente

### base_cam

| # | Befund | Schwere |
|---|---|---|
| B1 | Plan fordert `t` je Objekt. Ist-Stand: Stride 10 **ohne** `t` (`[id, color, x, y, z, orientation, vy, length, width, height]`). `vectoring` (Regression über Zeit) und `object_follower` (Extrapolation) brauchen ihn zwingend. → Vertragsänderung auf Stride 11. | hoch |
| B2 | „x/y/z in mm im **Roboter-Frame**" und „Koordinaten in **Weltkoordinaten**" stehen im selben Absatz. In AICA ist `world` der TF-Default, der Roboter-Base-Frame ist ein anderer Frame. Eindeutig festlegen. | hoch |
| B3 | `vy` wird von `base_cam` geliefert **und** von `vectoring` neu berechnet. Doppelarbeit und zwei Wahrheiten. Widerspricht zudem der Vorgabe, die Vision-Komponenten rechenarm zu halten. | mittel |
| B4 | „Abruf Kalibrierungsdaten aus Kalibrierungsdatei" als Input widerspricht Implementierung und `Calibration/README.md`: Werte werden als AICA-Parameter (`cal_x`…`cal_yaw`) gespiegelt, nicht zur Laufzeit aus einer Datei gelesen. Dateizugriff im Container ist zudem fragil. | mittel |
| B5 | Predicate „new object base": Semantik unklar — Puls bei *neuem* Objekt oder Pegel „Objekte vorhanden"? Ist-Stand ist `has_objects` (Pegel). Events feuern nur auf steigender Flanke; ein dauerhaft wahrer Pegel triggert genau einmal. | hoch |
| B6 | Parameter-Feld im Plan ist **leer**, die Komponente hat 31 Parameter. | niedrig |
| B7 | Plan beschreibt ID-Zuordnung „anhand überlappender Blockpositionen"; implementiert ist Nearest-Neighbour mit Prädiktion und max. Matchdistanz. | niedrig |
| B8 | Kein Konzept für ID-Verlust. Kurzer Track-Abriss → neue ID → `vectoring` startet neu, `prioritie_handler` verliert das Ziel, `data_tracker` behält eine Leiche. Realer Betriebsfall, im Plan nicht vorgesehen. | hoch |

### vectoring

| # | Befund | Schwere |
|---|---|---|
| V1 | Input-Liste nennt Predicate „object changed", der Aufgabentext reagiert auf „True-Flanke auf **Objekt abgelegt**". Zwei verschiedene Signale aus zwei verschiedenen Komponenten. | hoch |
| V2 | „Es werden dabei nur Vektoren berücksichtigt die um W voneinander abweicht" — gemeint ist offensichtlich „um **höchstens** W". So gelesen ist die Bedingung invertiert. | mittel |
| V3 | Winkelmittelung/-vergleich muss **zirkular** erfolgen. Arithmetisches Mitteln springt bei ±180°. Nicht erwähnt. | mittel |
| V4 | Richtungsschätzung per Regression **pro Objekt** ist schlecht konditioniert: kurze Bahn im Sichtfeld, Messrauschen im mm-Bereich. Auf einem Band sind alle Bahnen ohnehin parallel. Der Plan nennt die Alternative selbst unter „Offene Fragen". | hoch |
| V5 | Keine Verfallsregel (siehe S7). | hoch |
| V6 | Der Umweg „gepickte Objekte über Listenvergleich mit `data_tracker` entfernen" erzeugt den Zyklus S5 und kostet Rechenzeit für Information, die `object_follower` direkt hat. Der Plan nennt das selbst als offene Frage. | mittel |
| V7 | Ausgabeformat „ID, t Ortsvektor, Ortsvektor, Richtungsvektor, Geschwindigkeit" ohne Dimensionsangabe (2D oder 3D?) und ohne Stride. | hoch |

### prioritie_handler

| # | Befund | Schwere |
|---|---|---|
| P1 | **Auswahlkriterium ist riskant:** „nächstes Objekt zum nicht pickbaren Bereich" = das am weitesten fortgeschrittene = das mit der **geringsten** verbleibenden Zeit. Ohne Erreichbarkeitsprüfung jagt der Roboter systematisch Objekte, die er nicht mehr erreicht, und pickt am Ende gar nichts. Aus meiner Sicht der gravierendste konzeptionelle Fehler im Plan. | **kritisch** |
| P2 | Parameter `y-max greifbar`, `x-max`, `x-min` — **`y-min` fehlt**. Der Greifbereich ist ein Rechteck. | mittel |
| P3 | **Derselbe Arbeitsbereich ist zweimal definiert:** hier als `x-min`/`x-max`/`y-max`, im `object_follower` als `ROI_x_min`/`ROI_x_max`/`ROI_y_min`/`ROI_y_max`. Zwei Parametersätze für eine physikalische Grenze driften garantiert auseinander → widersprüchliche Entscheidungen zwischen den Komponenten. Klassisches Mehrfachüberarbeitungs-Artefakt. | hoch |
| P4 | Kein Lock-/Hysterese-Konzept: Was passiert, wenn während der Anfahrt ein „dringenderes" Objekt auftaucht? Ohne explizites Commit wechselt der Roboter das Ziel mitten in der Bewegung. | hoch |
| P5 | Die „Listenposition merken und nur bei Abweichung suchen"-Optimierung ist Mikro-Optimierung an der falschen Stelle. Bei <20 Objekten ist eine Dict-Suche über die ID kostenlos; die Zustandshaltung erhöht nur die Fehlerwahrscheinlichkeit. | niedrig |
| P6 | Output „Nicht pickbare Objekte (nur IDs)" ist redundant — aus `y` und der Grenze trivial ableitbar; erzeugt zusätzlich eine Kante im Zyklus S5. | mittel |

### roboter_cam

| # | Befund | Schwere |
|---|---|---|
| R1 | **TCP-Pose muss zum Bildzeitstempel gelten, nicht „aktuell".** Der Arm bewegt sich hier per Definition. Bei 0,3 m/s TCP und 30 ms Versatz sind das 9 mm Fehler — und zwar genau in der Feinpositionierung, wo es am meisten weh tut. Lösung: TF-Lookup zum Frame-Stempel statt „Koordinaten TCP von AICA" als Momentanwert. | **kritisch** |
| R2 | Die TCP-Transformation in der Kamera-Komponente widerspricht der Vorgabe, die Vision-Komponenten rechenarm und entkoppelt zu halten. Die Rechnung selbst ist billig, die Kopplung an Roboterzustand + Hand-Auge-Kalibrierung ist es nicht. Alternative: Kamera-Frame ausgeben, `object_follower` transformiert. | mittel |
| R3 | „Min-Distanz/Max-Distanz" — Distanz **wozu**? Zum Band, zum Objekt, TCP-Höhe? Nicht definiert. Ist-Stand: `robot_cam` misst die Banddistanz selbst aus der Tiefe. | mittel |
| R4 | „Objekt am nächsten zum Bildzentrum" — bei mehreren Blöcken im Bild kann das das **falsche** sein. Es gibt keine Identitätsprüfung gegen das Zielobjekt, obwohl Farbe und Abmessungen aus `base_cam` bekannt sind. Verschenkte Information. | hoch |
| R5 | **Greiferorientierung fehlt im gesamten Plan.** `roboter_cam` gibt laut Plan nur Koordinaten aus, keine Orientierung; `object_follower` setzt nirgends einen Yaw. Ein 2F-Greifer muss aber zur Objektorientierung ausgerichtet werden. Ist-Stand liefert die Orientierung bereits — der Plan nutzt sie nicht. | **kritisch** |
| R6 | Ist-Stand: `z` im Output ist die **Banddistanz**, nicht die Greifhöhe. Der Plan unterscheidet das nicht. | mittel |

### object_follower

| # | Befund | Schwere |
|---|---|---|
| O1 | **`z_wait` vs. `z_find`:** Die Parameterliste nennt `z_wait`, der Fließtext nennt zweimal `z_find`. `z_wait` wird im Text nie verwendet. Eindeutiges Überarbeitungsartefakt. | mittel |
| O2 | **Parameter `W` ist gelistet, kommt im Text nicht vor.** Stattdessen steht dort eine hart codierte „Abweichung von über 1°". Vermutlich sollte `W` genau das sein. | mittel |
| O3 | **Der z-Abstieg auf Greifhöhe fehlt vollständig.** Der Text beschreibt x/y-Nachführung auf `z_find`, dann direkt „Greifen = True", dann nach dem Greifen „+10 cm". Das Absenken auf Bandhöhe + Objekthöhe — **während das Objekt weiterfährt** — ist die eigentlich schwierige Bewegung und kommt im Konzept nicht vor. | **kritisch** |
| O4 | **Mitfahren während der Greifzeit fehlt.** Ein Robotiq 2F braucht real ~0,3–1 s zum Schließen. In dieser Zeit fährt das Band weiter. Der Plan behandelt „Greifen = True" als instantan. | **kritisch** |
| O5 | **Wie werden „Zielkoordinaten für UR10e-Steuerung" tatsächlich in Bewegung umgesetzt?** Der Plan sagt es nicht. Laut `ARCHITECTURE.md` bricht **jede neue Trajektorie die laufende sofort ab** (kein Buffering). Zyklisches `set_trajectory` im Step-Takt ergibt daher eine permanent abgebrochene, ruckelnde Bewegung. Das ist die zentrale offene Architekturentscheidung. | **kritisch** |
| O6 | „Falls eine Abweichung vorliegt, wird der Bewegungsvektor entsprechend angepasst" — das ist eine Regelung ohne Reglertyp, Verstärkung, Abtastrate oder Stabilitätsbetrachtung. | hoch |
| O7 | „Wenn die Position über V Frames konstant ist" — **„konstant" ist ohne Toleranz nicht definiert.** Es fehlt ein Parameter in mm. Außerdem: konstant relativ wozu (absolut oder relativ zum mitfahrenden Objekt)? Gemeint ist offensichtlich Letzteres. | hoch |
| O8 | Gewichtung X/Y (0–100) zwischen `prioritie_handler`- und `robot_cam`-Koordinate: unklar, **was** gewichtet wird — Position, Richtung, Geschwindigkeit oder alle drei getrennt. Zwei Gewichte für zwei Achsen deuten auf Position hin. Muss präzisiert werden, sonst baut jeder etwas anderes. (Das Prinzip als solches — Umschaltbarkeit zwischen beiden Quellen für den Praxistest — ist gut und sollte erhalten bleiben.) | hoch |
| O9 | „+10 cm" nach dem Greifen ist hart codiert, obwohl sonst alles parametriert ist. | niedrig |
| O10 | Das Sammeln von N `robot_cam`-Koordinaten kostet Zeit (N=10 bei 30 fps = 0,33 s ≈ 33 mm Bandweg). Ob währenddessen mitgefahren oder gehalten wird, sagt der Plan nicht. | mittel |
| O11 | Kein Timeout, kein Abbruch, kein Retry (siehe S8). | **kritisch** |
| O12 | Die Komponente ist faktisch ein Zustandsautomat (IDLE → INTERCEPT → TRACK → DESCEND → GRASP → LIFT → PLACE → RELEASE), wird aber als Fließtext beschrieben. Zustände sollten benannt und beobachtbar sein — auch für den `interface_streamer`. | hoch |
| O13 | Feste Ablageposition. Beim Projektnamen „Robot Tetris" wäre eine sortierte/positionierte Ablage zu erwarten. Falls später vorgesehen: Schnittstelle jetzt offenhalten. | offen |

### robotiq-gripper

| # | Befund | Schwere |
|---|---|---|
| G1 | **Widerspruch zur Implementierung:** Ist-Stand hat Inputs `gripper_close` (Bool) und `gripper_change` (Int32) und **keine Signal-Outputs**, nur Predicates. Der Plan-Input „Parameter Greifer zu" des `object_follower` ist so nicht verdrahtbar. → bool-Output ergänzen (kleine, klar umrissene Änderung). | hoch |
| G2 | **„Greifer zu" ist als Rückmeldung wertlos:** „Ist geschlossen, solange Greifen = true und gibt dabei selbst auf Greifer zu = true aus" — das ist ein reines Echo des Eingangs. Die informative Größe ist `is_object_grasped` (Objekt tatsächlich gefasst). Empfehlung: **zwei** Bools ausgeben (`is_closed`, `has_object`), `object_follower` wertet `has_object` aus. | hoch |
| G3 | `gripper_change` (Ziel-Öffnungsweite) bleibt ungenutzt. Aus der `base_cam`-Breite ließe sich der Greifer vorpositionieren → schneller und sicherer. Optionale Verbesserung. | niedrig |

### data_tracker

| # | Befund | Schwere |
|---|---|---|
| D1 | Leitet „welches Objekt wird gerade gepickt" **indirekt** aus den Geradenvektoren des `prioritie_handler` ab. `object_follower` weiß es direkt. Indirektion ohne Nutzen, erzeugt eine Zykluskante. | hoch |
| D2 | „Parameter Objekt Changed für 1 sec auf True" — zeitgesteuerter Pegel statt Flanke. Bei 50 Hz sind das 50 Steps; wer auf Pegel reagiert, verarbeitet 50-fach, wer auf Flanke reagiert, braucht die Sekunde nicht. Fragil und unnötig. Besser: 1-Step-Puls oder eine Sequenznummer. | mittel |
| D3 | `out_of_bounds` über eine separate Liste vom `prioritie_handler` statt lokal aus `y` + Grenze — Redundanz und Zykluskante. | mittel |
| D4 | Kein Löschen, nur Markieren → monoton wachsendes Array (siehe S7). | hoch |
| D5 | Zusammen mit `vectoring` doppelte Zustandshaltung (siehe S6). | hoch |

### interface_streamer

| # | Befund | Schwere |
|---|---|---|
| I1 | **Teuerste Komponente auf einem System, das laut Vorgabe schon an der Grenze läuft.** Zwei Kamerabilder kopieren, skalieren, komponieren, beschriften und als drittes Bild publizieren kostet spürbar CPU — und zwar dieselbe CPU, die die Echtzeit-Bildverarbeitung braucht. | hoch |
| I2 | „Details folgen." — Die Komponente ist nicht spezifiziert. | offen |
| I3 | Bezieht sich auf „Parameter Objekt im Bild 1/2" usw., also auf Predicates (siehe S1). | mittel |

---

## 3. Die zehn wichtigsten Punkte, verdichtet

| Rang | Befund | Warum kritisch |
|---|---|---|
| 1 | O5 — Bewegungsschnittstelle undefiniert; JTC bricht laufende Trajektorien ab | Ohne diese Entscheidung ist kein On-the-fly-Tracking implementierbar |
| 2 | P1 — Zielauswahl ohne Erreichbarkeitsprüfung | Führt systematisch dazu, dass gar nichts gepickt wird |
| 3 | O3/O4 — z-Abstieg und Mitfahren während des Greifens fehlen | Der eigentliche Greifvorgang ist nicht spezifiziert |
| 4 | R5 — Greiferorientierung kommt im Plan nicht vor | 2F-Greifer muss ausgerichtet werden; Daten sind da, werden nicht genutzt |
| 5 | S4/R1 — Latenz- und Zeitsynchronisation nicht adressiert | Systematischer, immer gleichgerichteter Positionsfehler |
| 6 | S8/O11 — keine Fehler-/Timeout-/Retry-Pfade | Erster Fehlgriff blockiert das System dauerhaft |
| 7 | S1 — Predicates als Signale verdrahtet | Große Teile des Plans sind so nicht baubar |
| 8 | S2/V7/B1 — Datenverträge ohne Stride, `t` fehlt in `base_cam` | Schnittstellen laufen garantiert auseinander |
| 9 | S5/S6 — Zyklus und dreifache Zustandshaltung | Inkonsistenzen + verschenkte CPU auf einem knappen System |
| 10 | P3 — Arbeitsbereich zweimal parametriert | Zwei Komponenten entscheiden widersprüchlich |

---

## 4. Anpassungsvorschläge

### A — Datenverträge festschreiben (billig, hoher Nutzen)
Ein Modul `roboter_tetris/contracts.py` mit Stride-Konstanten und `pack`/`unpack`-Funktionen für jedes Listen-Signal. Sender und Empfänger importieren dieselbe Definition. Konkret zu definieren:
- `objects` (base_cam): Stride **11** inkl. `t`
- `object_vectors` (vectoring)
- `target_line` (priority → follower)
- `world_state` (data_tracker)

Zusätzlich: Einheit (m) und Frame (`world` oder Base — einmal entscheiden) im Docstring jeder Funktion.

### B — Zyklus auflösen
Die Rückkante `data_tracker → vectoring` streichen. Stattdessen gibt `object_follower` beim Ablegen die **gepickte ID** als Signal aus (`picked_id` + 1-Step-Puls). `vectoring`/`data_tracker` hören direkt darauf. Der Datenfluss wird azyklisch; der Rückkanal ist ein reines Ereignis, keine Listenrunde.

### C — Zustand konsolidieren
`vectoring` und `data_tracker` führen dieselbe Objektwelt. Vorschlag: zu **einer** Komponente zusammenfassen (Arbeitstitel `object_model`), die
- die Objekte aus `base_cam` hält,
- Vektoren/Geschwindigkeit schätzt,
- `picked`/`out_of_bounds` selbst setzt (aus `picked_id` bzw. `y` + Grenze),
- eine harte Verfallsregel anwendet.

`priority_handler` bleibt zustandsarm (nur das Ziel-Lock). Das eliminiert die Listenvergleiche, den Zyklus und zwei Signalpfade — spürbar weniger CPU.

> **Rückfrage:** Falls die Komponentengrenzen aus organisatorischen Gründen (Arbeitsteilung, Abgabe) so bleiben müssen, ist die Zusammenlegung nicht die richtige Antwort. Dann stattdessen: `vectoring` hält keinen eigenen Zustand mehr, sondern rechnet zustandsfrei auf dem `data_tracker`-Array.

### D — Bandrichtung als Kalibrierwert statt Online-Schätzung
Einmalig bestimmen (ein Objekt durchlaufen lassen, Richtung fitten), in einer JSON ablegen — analog zu `Calibration/` und `Safety/` — und als AICA-Parameter spiegeln. Dann reduziert sich die Regression pro Objekt von 2D-Richtung auf eine **1D-Schätzung von Position und Geschwindigkeit entlang einer bekannten Achse**. Das ist deutlich robuster und billiger. Der `Richtung halten`-Mechanismus (K/W) bleibt als optionaler Online-Schätzer erhalten, Default aber = Kalibrierwert. Beantwortet auch die „Offene Frage" im Plan.

### E — Erreichbarkeit statt Dringlichkeit in der Zielauswahl
```
t_rest(obj)   = (y_grenze − y_obj) / v_band
t_benoetigt   = t_anfahrt(dist) + t_settle + t_greifen      (Parameter/Schätzung)
kandidat      ⟺ t_rest > t_benoetigt · sicherheitsfaktor
ziel          = unter den Kandidaten das mit dem kleinsten t_rest
```
Damit bleibt die Grundidee des Plans („das dringendste zuerst"), aber nur unter den *tatsächlich erreichbaren* Objekten. Dazu ein **Lock**: einmal gewählt, bleibt das Ziel bis abgelegt / verloren / unerreichbar.

### F — Prädiktion auf Ankunftszeit, TCP per TF-Lookup zum Bildstempel
- `object_follower` extrapoliert auf `t_jetzt + t_bewegung`, nicht auf `t_jetzt`.
- `robot_cam` (oder der Follower) holt die TCP-Pose per `tf2`-Lookup **zum Frame-Zeitstempel**, nicht als Momentanwert.
- Ein Parameter `latency_compensation_s`, einmal am realen Aufbau gemessen.

### G — Bewegungsschnittstelle explizit entscheiden
Empfohlenes Muster, sofern kein kartesischer Velocity-/Servo-Controller verfügbar ist:
1. **Abfangen:** eine einzige `set_trajectory`-Bewegung auf einen *prädizierten Treffpunkt* (nicht auf die aktuelle Objektposition).
2. **Feinführen:** ab da nur noch kleine Korrekturen, mit begrenzter Rate (z. B. 5–10 Hz statt 50 Hz), damit nicht jede neue Trajektorie die vorige zerhackt.
3. **Greifen:** Abstieg und Schließen als *mitfahrende* Bewegung, nicht als Halt.

Falls ein kartesischer Twist-Controller verfügbar ist, ist Schritt 2/3 als Geschwindigkeitsregelung deutlich sauberer. → **Rückfrage unten.**

### H — object_follower als expliziter Zustandsautomat
Zustände benennen, je Zustand ein Timeout und ein Rückfallpfad, aktueller Zustand als Signal/Predicate nach außen (auch für den `interface_streamer`). Ergänzen: z-Abstieg, Greiferorientierung (Yaw aus der Objektorientierung), Mitfahren während des Schließens, Abbruch bei `has_object == false`.

### I — Greifer um bool-Outputs erweitern
`is_closed` und `has_object` als `Bool`-Signale ergänzen (Predicates bleiben für die UI). Kleine, klar abgegrenzte Änderung an einer funktionierenden Komponente.

### J — Predicate/Signal-Regel dokumentieren
Eine Zeile in der Architektur: *Daten fließen als Signal, Ereignisse als Predicate.* Danach den Plan einmal komplett durchgehen und jedes „Parameter X" eindeutig zuordnen.

### K — interface_streamer entschärfen
Eigene, niedrige Rate (Parameter, z. B. 5 Hz) — oder ganz weglassen und stattdessen in RViz zwei Image-Displays plus ein Text-Overlay nutzen. Auf einem System an der Rechengrenze ist das die Komponente mit dem schlechtesten Nutzen/Kosten-Verhältnis.

### L — Safety-Gate vor der Zielkoordinatenausgabe
Workspace-Grenzen aus `Safety/workspace_bounds.json` als Parameter spiegeln, Zielposen dagegen prüfen, außerhalb → verwerfen + Warnung + Home. Voraussetzung: die Grenzen müssen erst einmal *festgelegt* werden (aktuell alle `null`).

### M — Redaktion
`z_wait`/`z_find` vereinheitlichen, `W` im Text verwenden, `y_min` ergänzen, Arbeitsbereichsgrenzen an genau einer Stelle definieren, Namensschema durchziehen, base_cam-Parameterliste nachtragen.

---

## 5. Offene Fragen

1. **Welche Controller stehen in eurer AICA-Installation zur Verfügung?** Nur der JTC, oder auch ein kartesischer Twist-/Velocity-/Servo-Controller? Das entscheidet, ob echtes kontinuierliches Tracking möglich ist oder ob es auf „prädizierte Abfangbewegung + wenige Korrekturen" hinausläuft (Vorschlag G).
2. **Bandgeschwindigkeit:** typischer und maximaler Wert, konstant oder regelbar? Davon hängen Erreichbarkeitsrechnung, Latenzbudget und Prädiktionsgüte direkt ab.
3. **Hand-Auge-Kalibrierung `robot_cam` → TCP:** liegt die vor bzw. macht sie der Kommilitone mit? Der gesamte `object_follower` hängt daran.
4. **Zeitquelle:** Liefern die RealSense-Stempel ROS-Zeit, oder muss ein Offset kompensiert werden?
5. **Ablage:** feste Position dauerhaft, oder ist später sortierte/positionierte Ablage vorgesehen (Projektname „Robot Tetris")? Falls ja, Schnittstelle jetzt offenhalten.
6. **Komponentengrenzen:** Sind `vectoring` und `data_tracker` aus organisatorischen Gründen (Arbeitsteilung/Abgabe) zwingend getrennt, oder darf zusammengelegt werden (Vorschlag C)?
7. **Wie viele Objekte gleichzeitig** sollen realistisch auf dem Band sein? Bestimmt, ob die Listen-Performance überhaupt ein Thema ist.

---
---

# Teil B — Überarbeitete Empfehlung nach Klärung (2026-09-05)

**Geklärt:**
- Es steht ein **Velocity-Controller** zur Verfügung, mit dem bereits getestet wurde.
- Die **Bandgeschwindigkeit ist konstant**, der Wert aber unbekannt.
- Der Kommilitone kalibriert **beide** Kameras intrinsisch und extrinsisch.
- Ablage: feste Position, aber so, dass sich mehrere abgelegte Objekte nicht im Weg stehen.
- `vectoring` und `data_tracker` bleiben aus Übersichtlichkeitsgründen **getrennt**.
- Realistisch **2–3 Objekte**, **von Hand aufgelegt**.

Damit ändern sich mehrere Bewertungen aus Teil A erheblich — überwiegend zum Besseren.

---

## B1 — Velocity-Controller löst die vier kritischsten Befunde auf einmal

Befund O5 (Bewegungsschnittstelle), O3 (z-Abstieg), O4 (Mitfahren beim Greifen) und O6 (Regelung ohne Regler) waren alle Symptome derselben Ursache: der Plan versuchte, eine kontinuierliche Verfolgung über ein positionsbasiertes Trajektorien-Interface abzubilden. Mit einem Velocity-Controller entfällt das.

### Empfohlenes Regelgesetz

```
v_cmd = v_ff + Kp · e                       (kartesisch)

v_ff  = band_richtung · band_geschwindigkeit    (Vorsteuerung, konstant)
e     = p_ziel − p_tcp                          (Lageabweichung)
p_ziel = (1−w) · p_basecam_prädiziert  +  w · p_robotcam_gemessen
```

Der entscheidende Punkt ist die **Vorsteuerung `v_ff`**. Ein reiner P-Regler auf ein bewegtes Ziel hat systematisch bleibende Regelabweichung (er hinkt immer hinterher, proportional zur Bandgeschwindigkeit). Mit der Bandgeschwindigkeit als Vorsteuerung geht die bleibende Abweichung gegen null, und der P-Anteil muss nur noch den Restfehler ausregeln. Da die Bandgeschwindigkeit **konstant** ist, ist `v_ff` eine Kalibrierkonstante — nicht zu schätzen, nicht zu filtern.

Das entschärft auch Befund S4/R1 deutlich: Messlatenz verschiebt in einem geschlossenen Regelkreis mit korrekter Vorsteuerung nicht mehr den Arbeitspunkt, sondern beeinflusst nur noch das Einschwingverhalten. Die Latenzkompensation bleibt sinnvoll, ist aber kein systematischer Offset mehr.

### `w` ist exakt die gewünschte Testbarkeit

Deine X/Y-Gewichte bekommen hier eine saubere Definition: **gewichtet wird die Zielposition**, je Achse getrennt (X entlang Band, Y quer). `w=0` → reine base_cam-Prädiktion, `w=1` → reine robot_cam-Messung, alles dazwischen mischbar. Richtung und Geschwindigkeit werden **nicht** gewichtet — die kommen aus der Kalibrierung. Damit ist O8 präzisiert, ohne die Testmöglichkeit zu verlieren.

Sinnvolle Ergänzung: Wenn robot_cam kein Objekt sieht, muss automatisch auf `w=0` zurückgefallen werden (sonst friert das Ziel ein). Das gehört als explizite Regel in die Spezifikation, nicht in die Implementierung.

### Phasen als Zustandsautomat mit Velocity-Kommandos

| Zustand | `v_cmd` | Übergang |
|---|---|---|
| `IDLE` | 0, TCP auf Warteposition | Ziel von priority_handler → `APPROACH` |
| `APPROACH` | Regelung auf Abfangpunkt, z auf Beobachtungshöhe | lateraler Fehler < Toleranz → `TRACK` |
| `TRACK` | `v_ff + Kp·e`, z konstant | Fehler < Toleranz über V Frames → `DESCEND` |
| `DESCEND` | wie TRACK, zusätzlich `vz = −v_descend` | Greifhöhe erreicht → `GRASP` |
| `GRASP` | **`v_ff` pur** (reines Mitfahren), Greifer schließt | `has_object` → `LIFT`, Timeout → `ABORT` |
| `LIFT` | `v_ff + vz` aufwärts | Abhebehöhe erreicht → `PLACE` |
| `PLACE` | Regelung auf Ablageposition[i] | erreicht → `RELEASE` |
| `RELEASE` | 0, Greifer öffnet | fertig → `IDLE`, `picked_id` publizieren |
| `ABORT` | 0 bzw. Rückzug | → `IDLE` |

Der z-Abstieg (O3) ist damit einfach eine `vz`-Komponente im selben Twist — keine Sonderbehandlung nötig. Das Mitfahren beim Greifen (O4) ist der Zustand `GRASP` mit reiner Vorsteuerung. Beides fällt aus dem Regelungsansatz heraus, statt nachträglich eingebaut werden zu müssen.

### Neue Pflicht: Sicherheitsmechanismen für Geschwindigkeitsregelung

Ein Velocity-Controller ist gefährlicher als ein Trajektorien-Interface: **er fährt weiter, wenn die Kommandos ausbleiben.** Zwingend zu spezifizieren:

1. **Watchdog / Totmannschaltung** — kommt für `t_watchdog` (z. B. 100 ms) kein frisches Kommando, wird `v = 0` gesetzt. Das gehört in die Komponente, nicht in die Hoffnung auf den Controller.
2. **`v = 0` bei jedem Zustandswechsel nach unten** — `on_deactivate`, `on_error`, Lifecycle-Stop. Niemals ein „letztes Kommando" stehen lassen.
3. **Geschwindigkeits- und Beschleunigungsgrenze** — `|v_cmd| ≤ v_max`, Rampenbegrenzung `|Δv| ≤ a_max·dt`. Ohne Rampe springt der Regler beim Zustandswechsel.
4. **Workspace-Gate mit Geschwindigkeits-Null** — nicht nur Zielposen verwerfen (Befund S9), sondern die Geschwindigkeitskomponente in Richtung Grenze auf null setzen, wenn der TCP die Grenze erreicht. Ein Positions-Clamp allein wirkt bei Geschwindigkeitsregelung nicht.
5. **Plausibilitätsgrenze für `e`** — springt die Lageabweichung unrealistisch (Fehldetektion, ID-Verwechslung), nicht regeln, sondern in `ABORT`.

Diese fünf Punkte sind aus meiner Sicht nicht verhandelbar und stehen bisher an keiner Stelle im Plan.

### Offen: welcher Controller genau

Die Twist-Schnittstelle muss eindeutig sein: **kartesischer Twist in welchem Referenzframe** (`world`, Roboterbasis oder TCP-Frame)? Und: ist es ein kartesischer Twist-Controller oder ein Joint-Velocity-Controller mit vorgeschalteter IK? Davon hängt ab, ob `object_follower` eine `sr.CartesianTwist` oder Gelenkgeschwindigkeiten ausgibt. → Frage 1 unten.

### Offen: Umschaltung zwischen Controllern

`move_to_pose_test` nutzt den JTC. Wenn der `object_follower` über den Velocity-Controller fährt, laufen zwei Controller im selben System. In `ros2_control` darf für dieselben Gelenke immer nur **einer** aktiv sein. Empfehlung: **alles** im Betrieb über den Velocity-Controller, den JTC höchstens für eine einmalige Home-Fahrt beim Start, mit sauberer Aktivierung/Deaktivierung über AICA-Events. Zwei parallel aktive Controller sind eine der unangenehmsten Fehlerarten, weil sie sich stumm gegenseitig überschreiben.

---

## B2 — Konstante Bandgeschwindigkeit vereinfacht `vectoring` drastisch

Weil das Band konstant läuft, sind **Richtung und Geschwindigkeit beides Kalibrierkonstanten**. Damit fällt der aufwendigste Teil des Plans weg:

| Plan (bisher) | Empfehlung |
|---|---|
| Richtungsvektor je Objekt per 2D-Regression über N Punkte | Richtung = kalibrierte Konstante |
| „Richtung halten" über K Objekte mit Abweichung W | entfällt (bzw. nur noch als Plausibilitätsprüfung) |
| Geschwindigkeit je Objekt aus Zeitstempeln | Geschwindigkeit = kalibrierte Konstante |
| „Geschwindigkeit halten" über J Messungen mit Abweichung H | entfällt (bzw. Watchdog: weicht die Messung dauerhaft ab → Warnung, Band steht/stockt) |

Was von `vectoring` übrig bleibt, ist deutlich robuster: **je Objekt nur noch Querablage (senkrecht zum Band) und Phase (Position entlang des Bands zu einem Zeitpunkt t₀)** — beides 1D-Schätzungen entlang bekannter Achsen, aus einem kurzen Ringpuffer geglättet. Statt einer schlecht konditionierten 2D-Richtungsregression über eine kurze Bahn (Befund V4) bleibt eine numerisch harmlose Geradenparametrierung.

Die Parameter `K`, `W`, `J`, `H`, „Richtung halten", „Geschwindigkeit halten" können damit entfallen oder auf eine reine Überwachungsfunktion reduziert werden. Das ist weniger Code, weniger Parameter zum Falscheinstellen und weniger Rechenzeit auf einem knappen System.

### Bandgeschwindigkeit bestimmen

Der Wert ist unbekannt, aber leicht zu bekommen — `base_cam` schätzt ihn im Ist-Stand bereits (`vy`, EMA-gefiltert, Messregion `track_velocity_region_y_*`):

1. Mehrere Objekte durchlaufen lassen, `objects`-Signal mitschneiden, `vy` über die Durchläufe mitteln.
2. Gegenprobe von Hand: Markierung aufs Band, Strecke und Zeit stoppen. Zwei unabhängige Verfahren, weil ein Vision-Wert allein sich selbst bestätigt.
3. Wert zusammen mit Bandrichtung in eine versionierte JSON — analog zu `Calibration/calibration.json` und `Safety/workspace_bounds.json`, mit demselben Muster (`status`, `measured_at`, `operator`, `method`). Zur Laufzeit als AICA-Parameter gespiegelt.

Die Bandrichtung fällt beim selben Durchlauf mit ab: die Bahn eines einzelnen durchlaufenden Objekts über das gesamte Sichtfeld ist eine weit bessere Richtungsschätzung als jede Online-Regression über einen Teilausschnitt.

---

## B3 — 2–3 Objekte, von Hand aufgelegt

### Was einfacher wird

- **Listen-Performance ist kein Thema.** Befund S7 (unbegrenztes Wachstum) bleibt trotzdem gültig, aber als Sauberkeits-, nicht als Performancefrage. Die Mikro-Optimierung „Listenposition merken" im `prioritie_handler` (Befund P5) ist damit endgültig unbegründet — eine Suche über 3 Einträge ist kostenlos.
- **Der `prioritie_handler` wird nahezu trivial.** Bei 2–3 Objekten und manuellem Auflegen liegt meist ohnehin nur ein greifbares Objekt vor. Das Erreichbarkeitskriterium (Befund P1) bleibt aber unverändert wichtig — es verhindert, dass der Roboter einem bereits zu weit gelaufenen Objekt hinterherjagt, statt auf das nächste zu warten.
- **Durchsatz ist kein Problem** (Befund S6 entschärft).

### Was schwieriger wird

- **Beliebige Orientierung.** Von Hand aufgelegt heißt: jeder Winkel ist möglich. Befund R5 (Greiferorientierung fehlt im Plan) wird damit von „wichtig" zu **zwingend**. Der Greifer-Yaw muss aus der Objektorientierung gesetzt werden, und bei Geschwindigkeitsregelung heißt das: entweder vorab ausrichten (im `APPROACH`) und dann nur noch translatorisch regeln, oder eine Winkelgeschwindigkeit `ωz` mitführen. **Vorab ausrichten ist deutlich einfacher und ausreichend** — das Objekt rotiert auf dem Band nicht.
- **Beliebige Querablage.** Objekte können nah am Bandrand liegen. Das Erreichbarkeitskriterium braucht deshalb nicht nur eine y-Grenze, sondern die volle Rechteckprüfung — womit Befund P2 (`y_min` fehlt) und P3 (Grenzen doppelt definiert) direkt relevant werden.
- **Die Hand im Bild.** Beim Auflegen greift eine Hand in den Sichtbereich der base_cam. Das erzeugt kurzzeitige Fehldetektionen mit eigenen IDs. Der vorhandene Tracker federt das teilweise ab (neue Detektionen werden erst nach Wiedererkennung zu Tracks befördert), aber ein Plausibilitätsfilter auf Größe/Höhe wäre eine billige Absicherung. Das ist Bildverarbeitung und damit außerhalb dieses Reviews — es beeinflusst aber den Vertrag: **`vectoring` und `data_tracker` müssen damit umgehen, dass IDs auftauchen und wieder verschwinden, ohne je gepickt zu werden.** Die Verfallsregel aus Befund S7 ist dafür die Antwort.
- **Unregelmäßige Ankunft.** Ein Objekt kann aufgelegt werden, während der Roboter bereits ein anderes verfolgt. Das macht das Ziel-Lock (Befund P4) zur Pflicht, nicht zur Kür.

---

## B4 — Getrennte Komponenten, aber eindeutige Zuständigkeit

`vectoring` und `data_tracker` bleiben getrennt. Der Zyklus (Befund S5) und die Doppelhaltung (S6) lassen sich trotzdem auflösen, indem die **Eigentümerschaft** eindeutig verteilt wird statt die Komponenten zusammenzulegen:

| Komponente | besitzt | besitzt **nicht** |
|---|---|---|
| `data_tracker` | die Objektwelt: Identität, Stammdaten, `picked`/`out_of_bounds`, Verfall | keine Geometrie-/Vektorrechnung |
| `vectoring` | nur einen kurzen Messpuffer je ID zur Glättung | keine eigene Objektliste, kein Zustand über den Puffer hinaus |
| `prioritie_handler` | nur das Ziel-Lock (eine ID) | keine Liste |
| `object_follower` | den Zustandsautomaten und die Regelung | keine Objektverwaltung |

### Vorgeschlagener Datenfluss (ohne kontinuierlichen Zyklus)

```
base_cam ──objects──> vectoring ──vectors──> prioritie_handler ──target_line──> object_follower ──twist──> Velocity-Controller
    │                                             ▲                                    │
    └────objects────> data_tracker ──flags───────┘                                     │
                            ▲                                                          │
                            └──────────────── picked_id (Ereignis) ────────────────────┤
                                              picked_id (Ereignis) ────────────────────┘
                                                                  ▲
robot_cam ──measurement──────────────────────────────────────────┘
```

Zwei Änderungen gegenüber dem Plan:
1. **`vectoring` bezieht nichts mehr von `data_tracker`.** Es rechnet vorwärts auf dem base_cam-Strom und lässt IDs verfallen, die ausbleiben. Damit fällt der teure Listenvergleich weg — genau die offene Frage, die der Plan selbst stellt.
2. **`picked_id` geht als Ereignis an `data_tracker` *und* `prioritie_handler`.** Der Regelpfad läuft nicht mehr über `data_tracker`; die einzige verbleibende Rückkante ist ein diskretes Ereignis pro Pick.

`data_tracker` wird damit zum reinen Weltmodell und Diagnoselieferanten (auch für den `interface_streamer`) und liegt nicht mehr im zeitkritischen Pfad. Genau die Übersichtlichkeit, wegen der ihr die Komponenten getrennt haben — nur ohne den Ring.

---

## B5 — Ablage ohne gegenseitige Behinderung

Statt einer einzelnen festen Ablageposition: **ein parametriertes Ablageraster**.

```
ablage_basis_pose            (x, y, z, yaw)
ablage_pitch_x, pitch_y      Abstand zwischen Ablageplätzen
ablage_spalten               Plätze pro Reihe
ablage_index                 läuft hoch, Reset beim Start
```
→ Ablageposition = Basis + (index mod spalten)·pitch_x + (index div spalten)·pitch_y

Deterministisch, kein Stapeln, kein Kollisionsproblem, und der Roboter fährt jedes Mal eine bekannte Pose an. Der Index gehört in den `object_follower` (er legt ab) und sollte als Signal/Predicate sichtbar sein, damit man beim Test sieht, wo der nächste Block landet. Ein `ablage_voll`-Zustand bei erschöpftem Raster verhindert, dass nach dem letzten Platz wieder auf Platz 1 abgelegt wird.

Falls später doch sortiert abgelegt werden soll: Farbe ist in den Objektdaten bereits enthalten, die Ablageposition würde dann aus der Farbe statt aus dem Index bestimmt. Die Schnittstelle bleibt dieselbe — deshalb ist der Index-Ansatz nicht später wegzuwerfen.

---

## B6 — Kalibrierung: Abgrenzung prüfen

Der Kommilitone kalibriert beide Kameras intrinsisch und extrinsisch. Zu klären ist, **was „extrinsisch" bei der Roboterkamera bedeutet**: bei der base_cam ist es die Transformation Kamera → Roboterbasis (existiert bereits als `cal_x`…`cal_yaw`), bei der Roboterkamera muss es die **Hand-Auge-Transformation Kamera → Flansch/TCP** sein. Das ist eine andere Messaufgabe mit anderem Verfahren.

Falls diese Hand-Auge-Transformation nicht mit abgedeckt ist, fehlt sie — und ohne sie kann der `object_follower` die robot_cam-Messung nicht in den Weltframe bringen, was den kompletten `w`-Anteil der Regelung unbrauchbar macht. Ebenfalls zu klären: **in welcher Form** die Werte ankommen (JSON zum manuellen Übertragen wie bisher, oder als TF), damit der Übergabepunkt eindeutig ist.

---

## B7 — Was von den Befunden aus Teil A unverändert stehen bleibt

Durch die Klärung **nicht** entschärft und weiterhin zu beheben:

- **P1** Erreichbarkeitskriterium in der Zielauswahl — unverändert kritisch
- **P3** Arbeitsbereich doppelt parametriert; **P2** `y_min` fehlt
- **P4** Ziel-Lock fehlt — durch manuelles Auflegen sogar wichtiger geworden
- **R4** keine Identitätsprüfung zwischen base_cam-Ziel und dem, was robot_cam im Zentrum sieht
- **R5** Greiferorientierung — durch beliebige Auflegewinkel zwingend
- **S1** Predicates als Signale verdrahtet; **G1/G2** Greifer braucht bool-Outputs, `has_object` statt Echo
- **S2/V7/B1** Stride-Verträge, `t` fehlt in `base_cam`
- **S3** mm/m-Grenze undefiniert
- **S8/O11** Fehler-, Timeout-, Abbruchpfade
- **S9** Safety-Gate — bei Geschwindigkeitsregelung sogar verschärft (siehe B1)
- **S10/M** Namens- und Redaktionsartefakte
- **D2** „1 sec True" statt Flanke
- **I1** `interface_streamer` als CPU-Fresser
- **O7** „konstant" ohne Toleranzangabe
- **O10** Verhalten während des Sammelns von N Messungen

---

## B8 — Neue offene Fragen

1. **Welcher Velocity-Controller genau** — kartesischer Twist-Controller oder Joint-Velocity mit vorgeschalteter IK? Und in welchem **Referenzframe** wird der Twist erwartet (`world`, Roboterbasis, TCP)? Bestimmt den Ausgabetyp des `object_follower`.
2. **`z_wait` vs. `z_find`:** Ich habe das in Teil A als Redaktionsfehler bewertet. Bei genauerem Hinsehen könnten es **zwei verschiedene Höhen** sein, die beim Überarbeiten verschmolzen sind — eine Warte-/Beobachtungshöhe im Leerlauf und eine (niedrigere) Höhe, auf der robot_cam misst. Waren zwei Höhen gemeint?
3. **Deckt die Kalibrierung des Kommilitonen die Hand-Auge-Transformation robot_cam → TCP ab** (siehe B6), und in welcher Form kommt sie an?
4. **Wird der JTC im Betrieb überhaupt noch gebraucht** (Home-Fahrt beim Start), oder soll alles über den Velocity-Controller laufen? Zwei gleichzeitig aktive Controller auf denselben Gelenken müssen ausgeschlossen sein.
5. **Objekthöhe für die Greifhöhe:** kommt sie aus `base_cam` (`height` ist im Vertrag enthalten) oder wird sie live aus der robot_cam-Tiefe bestimmt? Der Plan sagt es nicht, der `DESCEND`-Zustand braucht es.
