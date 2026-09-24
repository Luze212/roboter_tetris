# Fahrplan — Arbeit am Roboter mit Claude

**Stand 24.09.2026.** Wie die Zeit am Aufbau aufgeteilt wird, wer was tut und
woran jeder Block als erledigt gilt. Die fachliche Reihenfolge und ihre Gründe
stehen in `uebergabe.md` §6; dieses Dokument ist die **Arbeitsfassung** davon, mit
Rollen, Dauer, Mitlese-Verfahren und Abbruchregeln.

Zeit am Aufbau ist die knappe Ressource (Projektkontext §6). Deshalb gilt:
**Was ohne Roboter vorbereitet werden kann, wird vorher vorbereitet, und jeder
Block wird sofort nach seinem Ende dokumentiert** — nicht gesammelt am Schluss.

---

## Stand nach Termin B (23.09.2026)

| Block | Ergebnis |
|---|---|
| 0, 1 | ✅ (22.09.) — Nachtrag 12 / K1, K2, K4 |
| Rechenlast | ✅ Ursache gefunden: **alle Python-Komponenten teilen sich einen Prozess** (GIL), Bilder stauten sich in Warteschlangen der Tiefe 10. Rates gesenkt, Warteschlange Tiefe 1 → 7 Messungen/s, Alter 139 ms (Nachtrag 13 / L2) |
| 2 | ✅ **B23 erledigt.** Kamera schaut senkrecht, Parallaxe in der Detektion korrigiert, neue Extrinsik als Standardwert; Position ≤ 6 mm auch für 100-mm-Klötze. Höhe 11,5 mm zu niedrig (L6) |
| 3 | ✅ **Pool −127,9 mm/s** (Stoppuhr 125–133), alle Klötze nach ~1 s final, hinter dem Bild als Status 4 bis ans Bandende (L9, L10). **Ziel 3 bestätigt** |
| 6 | 🟡 **Hand-Auge neu eingemessen und bestätigt** (2–4 mm, L11, L13). Erkennung von `robot_cam_2` an flachen Klötzen unzuverlässig (bis 32 mm) → neuer Erkennungskern am Schreibtisch; erster Griff ohne Roboterkamera |
| 5 | ✅ **Arbeitsraum und Greifzone festgelegt** (L14): ws x −1,0 … −0,30, y −0,32 … +0,48, z 0,3086 … 0,60; Zone y +0,40 … −0,22 |

**Entscheidungen des Tages:** Greifzone und Wartebereich **außerhalb des Bildes
der Basiskamera** (L4); die Strecke dahinter überbrückt die Roboterkamera.

## Stand nach Termin C (24.09.2026)

| Block | Ergebnis |
|---|---|
| Rechenlast | Kameras auf 15 Bilder/s; `base_cam` unverändert 8,6 Messungen/s. **Keine NVIDIA-GPU** im Rechner (L16). Infrarot der Roboterkamera abschalten hängte AICA auf — `camera_node` leer (L16) |
| Follower | Legte **keine Signale** an (Parameterprüfung), korrigiert (L17). Arbeitsraum und Beobachtungspose sind Standardwert (L15) |
| 7 | ✅ **Gedrosselt bestanden (L18)** mit `fake_objects.py`: Folgen, Absenken, Greifen (Fehlgriff ins Leere), Abbruch, Heben — bei 0,07 m/s (IK 0,10) und 0,13 m/s (IK 0,30). Attractor auf K = 5 / 50 Hz; `err_laengs` −2,5 bzw. −5 mm → `lead_time_s` 0,24 |
| ⚠️ | **Einbruch der 500-Hz-Schleife** (4/500 Hz, ~1 s) beim Kontrolllauf → External Control stoppte. Schleife auch in Ruhe nur 84–86 % (L18) |

### Weiter am nächsten Termin

0. **Build** mit L15–L18 (Standardwerte `lead_time_s` 0,24, `attractor_v_max_mps` 0,30,
   `t_descend_s` 1,2; Parametergruppen). Vorher ChatGPT, Discord u. Ä. beenden,
   AICA-Oberfläche während der Läufe minimieren.
1. **AICA-Blöcke von Hand** (nicht im Paket): Attractor `linear_gains` [5.0], `rate`
   50; IK-Controller `max_linear_velocity` 0,30; `rate` Follower 50,
   `priority_handler`/`vectoring` 20, `data_tracker` 2. **Verdrahtung prüfen:**
   `target_pose` → Attractor (nicht IK-Controller), `picked_id` → `priority_handler`,
   `gripper_close` → Greifer. `camera_node` von `robot_cam_2` leer.
2. **Kontrolllauf `lead_time_s` 0,24** mit `fake_objects.py --velocity -0.13`, Band
   leer: erwartet `err_laengs` ≈ 0 statt −5 mm.
3. **Erster echter Griff:** Basiskamera konfigurieren und aktivieren, Referenzklotz
   50 × 50 × 100 aufs laufende Band. Erstmals mit Klotz im Greifer: Heben, **Ablage**
   (−0,316 / +0,476 / 0,42, B9 gegenlesen), öffnen, zurück. Greifhöhe prüft die
   245 mm und die Bandhöhe ±5 mm (B17).
4. **Rechenlast** weiter beobachten; bleiben Einbrüche, Rate des Hardware-Interface
   (250 statt 500 Hz) prüfen — vorher recherchieren. `event_engine` wächst, AICA
   zwischendurch neu starten.
5. Offen am Schreibtisch: neuer Erkennungskern für die Roboterkamera (L13),
   `fake_objects.py` mit realer Latenz; Erreichbarkeit im `priority_handler` ohne
   Einschwingzeit des Followers (~1,5 s, L18).

Rohdaten: `architektur/bilder/2026-09-23-b23-punkte.json`, `…-handauge-ansichten.json`. Werkzeuge: §3; neu in
`signal_reader.py` ist die Auswertung des Signalalters bei jedem Vertragssignal.
Mitlesen belastet den Rechner spürbar — **immer nur ein Leseprozess gleichzeitig**, die
Flanschpose (500 Hz) nicht in Python mitdekodieren, während eines Laufs keine
`ros2`-Befehle absetzen (L18).

---

## 1. Rollen

| | Nutzer | Claude |
|---|---|---|
| Bauen, Systemabbild neu erzeugen, Anwendung laden | ✅ ausschließlich | ✗ |
| Verdrahten und Parameter in der AICA-Oberfläche | ✅ | sagt, was wohin gehört |
| Roboter bewegen (Pendant, Freedrive), Not-Aus | ✅ ausschließlich | ✗ |
| Klötze auflegen, Band ein/aus, Stoppuhr | ✅ | ✗ |
| Topics mitlesen, auswerten, rechnen | — | ✅ frei, read-only |
| `ros2 param set`, `fake_objects.py` starten, Lifecycle-Übergänge | gibt **Go** | ✅ **nur nach Go für genau diesen Schritt** |
| Git | ✅ ausschließlich | ✗ (nur lesend) |
| Doku nachziehen | prüft | ✅ nach jedem Block |

**Bevor sich der Roboter durch etwas bewegt, das Claude auslöst**, sagt Claude in
einem Satz, **was** sich bewegen wird und **wohin**. Erst dann das Go.

---

## 2. Sicherheitsregeln für jeden Block mit Bewegung

1. **Eine Hand am Not-Aus**, solange eine eigene Komponente Zielposen ausgibt.
2. **Erster Lauf jeder Stufe am echten Roboter mit gedrosselter Geschwindigkeit:**
   `max_linear_velocity` des IK-Controllers auf **0,10 m/s**. Erst wenn der Ablauf
   stimmt, auf den Betriebswert 0,25 (Einrichtung §2). ⚠️ Das Band läuft mit ≈ 0,13 m/s
   (Nachtrag 13 / L7) — gedrosselt lässt es sich **nicht** einholen. Gedrosselte
   Läufe deshalb mit `fake_objects.py` und kleiner Geschwindigkeit, mit laufendem
   Band erst mit ≥ 0,30 m/s.
3. **Am echten Roboter nur mit dokumentiertem Arbeitsraum** (B10, nach
   `Safety/README.md`). Die Werte aus Einrichtung §9 gelten **nur** am virtuellen
   Roboter. → Block 5 kommt vor Block 7.
4. **Sofortabbruch durch Claude:** `fake_objects.py` läuft als Hintergrundprozess
   und wird bei jeder Unklarheit sofort beendet. Das stoppt die Zielquelle; der
   Attractor hält dann die letzte Zielpose (Thema 7).
5. Ruft der Nutzer **„Stopp“**, führt Claude keinen weiteren Schritt aus, bis der
   Nutzer ausdrücklich weitermacht.

---

## 3. Vorbereitung — ohne Roboter

| Wer | Was | Stand |
|---|---|---|
| Claude | **`test/tools/signal_reader.py`** — liest im Container mit, ausschließlich lesend: Topics, Kameras (Serial, `global_time`, Belichtung), Zeitstempel-Drift, Flanschpose, Gelenke, alle Vertragssignale S1–S10 über `contracts.unpack_*`. Live-Zeilen, CSV, JSON-Zusammenfassung | ✅ 22.09.2026, offline gegen `contracts.pack_*` getestet |
| Claude | **`test/tools/b23_compare.py`** — Punktpaare Basiskamera ↔ Flansch: Identität, 180°-Hypothese, starre 2D-Ausgleichsrechnung mit Restfehlern, B21-Streuung, Höhenvergleich. Läuft auf dem Rechner, reines Python | ✅ 22.09.2026, an synthetischen Daten geprüft (183° / 12 mm vorgegeben → zurückgewonnen) |
| Nutzer | Rebuild und Systemabbild neu erzeugen, sobald das Kalibrierprojekt es zulässt | — |
| Nutzer | Referenzklotz 50×50×100 und 2–3 weitere Klötze bereitlegen; Band frei von Zetteln? | — |

Beide Skripte sind **untracked**, bis der Nutzer sie übernimmt.

### Aufrufe

```bash
C=$(docker ps --format '{{.Names}}' | grep aica-launcher | head -1)
T=source/roboter_tetris
docker cp $T/test/tools/signal_reader.py $C:/tmp/ && docker cp $T/roboter_tetris/contracts.py $C:/tmp/
R() { docker exec -u ros2 -e ROS_DOMAIN_ID=0 "$C" bash -lc "python3 /tmp/signal_reader.py $*"; }   # bash -lc: sonst fehlt die ROS-Umgebung
```

| Block | Aufruf |
|---|---|
| 0 | `R cameras` · `R stamps --topic <Kamera>/color_camera_info --duration 120` (beide Kameras) · `R topics` |
| 1 | `R objects --live` · `R tracks --live` · `R target --live` · `R world_state --duration 20` · `R picked_id --duration 60` |
| 2 | siehe unten — Pipes gehören nicht in eine Tabelle |
| 3 | `R tracks --duration 120 --csv /tmp/block3.csv` (Status 3 → 0, Pool) |
| 4, 7 | `R follower_status --live` · `R target --live` · `R picked_id --duration 300` |
| 5 | `R joints --duration 180 --live --csv /tmp/block5_joints.csv` und parallel `R cartesian --duration 180 --csv /tmp/block5_cart.csv` |
| 6 | `R object_position --live` |

Block 2, je Punkt `Pn` (P1, P2, …):

```bash
B="python3 $T/test/tools/b23_compare.py"
R objects   --duration 10 --summary | tail -1 | $B add b23.json Pn cam   # Klotz ruht
R cartesian --duration 3  --summary | tail -1 | $B add b23.json Pn rob   # Backen auf der Oberseite
$B eval b23.json                                                         # nach dem letzten Punkt
```

Das Topic wird automatisch gefunden, wenn genau eines auf `/<signal>` endet; sonst
`--topic` angeben. `cartesian`/`joints` lesen den `robot_state_broadcaster`.

---

## 4. Die Blöcke

Jeder Block setzt die vorigen voraus, sofern nicht anders vermerkt.

### Block 0 — Inbetriebnahme-Check · 15 min · Roboter steht

| | |
|---|---|
| **Nutzer** | AICA mit dem neuen Systemabbild starten, Kameras und Hardware-Interface wie bisher |
| **Claude** | Container finden; Kameraknoten über `serial_no` zuordnen; `ros_zeit − header_stempel` beider Kameras 2 min mitlesen (B12/B13, dazu **B24** für die D435i); Bildraten |
| **Abnahme** | `camera_node` erscheint bei `base_cam` → das neue Paket ist wirklich geladen. Beide Zeitstempel konstant versetzt. ~30 Hz. |
| **Abbruch** | `camera_node` fehlt → altes Paket; Systemabbild neu erzeugen (Einrichtung §1). Alles Weitere wäre vergeudet. |

### Block 1 — Registrierung und Datenpfad · 30 min · Roboter steht

| | |
|---|---|
| **Nutzer** | Prüfen, ob `vectoring`, `priority_handler`, `data_tracker`, `object_follower`, `interface_streamer` in der Bibliothek erscheinen. Verdrahten: `base_cam` → `vectoring` → `priority_handler` → `data_tracker` → `interface_streamer`; `cartesian_state` → `priority_handler`. Greifer mit den neuen Ausgängen `motion_done`/`has_object`. |
| **Claude** | `objects`, `tracks`, `target`, `not_pickable`, `world_state` mitlesen; Log des `priority_handler`; Übersichtsbild in RViz gemeinsam ansehen. Greifer: einmal schließen/öffnen, `motion_done` und `has_object` mitlesen. |
| **Abnahme** | Jede Komponente läuft durch `configure`/`activate`. Ein ruhender Klotz erscheint in allen Signalen mit derselben ID. Greiferausgänge folgen der Bewegung. |
| **Erwartet, kein Fehler** | Der `priority_handler` **wählt keinen Klotz**: Die Greifzone liegt als Platzhalter im Robotersystem (x −0,95…−0,68), die Basiskamera meldet mit alten Kalibrierwerten x ≈ +0,81 (B23). Die Auswahl wird in Block 4 mit `fake_objects.py` geprüft. | *(Stand 22.09.; seit Nachtrag 13 liegt die Basiskamera im Robotersystem.)*

### Block 2 — B23 und B21 in einem Durchgang · 45 min · Band steht

**Der wichtigste Block.** Er entscheidet, ob die Basiskamera den Follower speisen
darf, und liefert dem Kalibrierprojekt unabhängige Prüfpunkte.

Ablauf, an **4–5 Stellen längs des Bandes**, über den ganzen Sichtbereich der
Basiskamera verteilt (auch außerhalb der alten Messregion −1000…−500 mm):

1. **Nutzer** stellt den Referenzklotz auf das Band, **Claude** liest `objects`
   10 s mit → Position x, y, Höhe, Streuung.
2. **Nutzer** setzt die **geschlossenen Backen mittig auf die Klotzoberseite**,
   Werkzeug lotrecht, ohne den Klotz zu verschieben. **Claude** liest den Flansch.
3. Nächste Stelle.

| | |
|---|---|
| **Auswertung (Claude, sofort)** | **B23:** Gilt `(x, y)_Kamera ≈ (−x, −y)_Roboter`? Starre 2D-Ausgleichsrechnung über alle Paare: Drehwinkel, Verschiebung, Restfehler je Punkt. **B21:** Streuung der Längsposition an jeder Stelle — innerhalb wie außerhalb der alten Region gleich klein? |
| **Abnahme B23** | Restfehler aller Paare ≤ ~10 mm → Basiskamera darf an den Follower. Sonst bleibt sie abgeklemmt, bis C3 geliefert ist. |
| **Abnahme B21** | Streuung außerhalb der Region nicht wesentlich größer als M5 (σ 0,2…0,6 mm) → der Tracker-Eingriff 2.4 trägt. |
| **Nicht verwechseln** | Das ist **keine Kalibrierung** — die liefert der Kommilitone (C3). Die Punktpaare sind eine Prüfung. Ob sich ein bestätigter 180°-Versatz übergangsweise ausgleichen lässt, wird nach der Messung entschieden. |
| **Nebenbei** | Höhe des Klotzes in `objects` gegen 100 mm (bestätigt M5 an neuen Stellen). |

### Block 3 — Band läuft: Ziel 3 · 30 min · nur wenn das Band laufen darf

| | |
|---|---|
| **Nutzer** | Band an. Klötze einzeln vorn auflegen, einen davon so, dass er kippt. **B1:** Markierung auf dem Band über eine abgemessene Strecke stoppen, 3×. |
| **Claude** | `tracks` mitschreiben: Status 3 → 0 je Klotz, Zeit bis final, Pool-Geschwindigkeit und `n_pool` über die Zeit. |
| **Abnahme** | Jeder Klotz startet mit Status 3 und wird final; der gekippte erst nach dem Kippen. Pool-Geschwindigkeit stimmt mit der Stoppuhr überein — das prüft die **ganze Kette** einschließlich Tiefe und Extrinsik (Z2). |
| **Wenn das Band nicht darf** | Block überspringen; alles Weitere hängt nicht daran — Block 4 arbeitet mit `fake_objects.py`. |

### Block 4 — Follower am virtuellen Roboter · 45 min · kein echter Roboter

| | |
|---|---|
| **Nutzer** | Hardware-Interface auf den virtuellen Roboter; `base_cam` abklemmen; `object_follower` verdrahten (Einrichtung §6), Pflichtparameter aus Einrichtung §9; Greifer-Rückmeldung über `toggle_signal`. |
| **Claude** | Nach Go: `fake_objects.py` auf das `objects`-Topic. Mitlesen: `target`, `follower_status` (Zustand, Abweichungen, `w_wirksam`), `target_pose`, `picked_id`. |
| **Abnahme 4a** | Aus mehreren Startlagen erst senkrecht hoch, dann Beobachtungspose. Log nennt den Bezugsrahmen von `robot_state` — **muss `world` sein**. |
| **Abnahme 4b** | `timeout_track_s` = 10 s. Roboter folgt dem Klotz mit konstantem Abstand; `err_laengs` im Mittel ≈ 0 bei `lead_time_s` = 1/K. |
| **Abnahme 4d** | Voller Zyklus mit `toggle_signal`. Die ID verschwindet nach dem Greifen aus `tracks` — der Zyklus läuft trotzdem bis zur Kiste (Z12). Künstlicher Abbruch mit Klotz im Greifer endet in der Kiste. |
| **Zugleich Abnahme 3.3** | Auswahl, Ziel-Lock, Rückzug und `not_pickable` im Log — mit synthetischen Klötzen im Robotersystem. |

### Block 5 — Greifzone, Singularitäten, Arbeitsraum · 40 min · echter Roboter, von Hand

| | |
|---|---|
| **Nutzer** | Arm per Freedrive/Pendant entlang des Bandes führen: einmal auf Beobachtungshöhe, einmal auf Greifhöhe (≈ 339 mm Flansch), vom Bandanfang bis zum Ende, soweit der Roboter kommt. Werkzeug lotrecht, Gierwinkel wie `observe_yaw_deg`. |
| **Claude** | `joint_state` und `cartesian_state` durchgehend mitschreiben. Auswertung: Abstand zu den drei UR-Singularitäten (Handgelenk: `wrist_2` gegen 0°/±180°; Ellbogen gestreckt; Handgelenkzentrum nahe der Schulterachse), Gelenkanschläge, Reichweite. |
| **Ergebnis** | **B19:** `zone_*` im Robotersystem für den `priority_handler`; Messregion von `base_cam` muss die Zone umschließen (nach B23 im selben System). **B11:** kritische Abschnitte. **B10:** Arbeitsraumvorschlag, dokumentiert nach `Safety/README.md`. |
| **Abnahme** | `is_zone_feasible` meldet die Zone als lang genug für die geschätzte Bandgeschwindigkeit. |

### Block 6 — Roboterkamera · 40 min · Roboter auf Beobachtungspose

| | |
|---|---|
| **Nutzer** | Roboter auf eine Beobachtungspose über einem ruhenden Klotz; Belichtungsautomatik aus (Einrichtung §1). |
| **Claude** | **B6 Stufe 1:** `object_position` und Debug-Bild der neuen Auswahl — trifft sie den Klotz statt Glanz oder Maschinenrand? Erwartungspunkt und ROI am Bild ablesen. Beide Varianten. **B8:** Höhe variieren, ab wann die Messung trägt → `observe_z`, D18. **B24** ist aus Block 0 bekannt. |
| **Abnahme** | Eine Variante liefert an mehreren Klotzlagen die richtige Position. Sonst bleibt 4c aus (Z10) — **das blockiert Block 7 nicht.** |

### Block 7 — Follower am echten Roboter · 60+ min · Voraussetzung Block 5

> **24.09.2026:** 4a/4b und 4d bis zum Fehlgriff mit `fake_objects.py` bestanden
> (L18). Offen: Kontrolllauf `lead_time_s`, Basiskamera und echter Klotz mit Ablage.

| | |
|---|---|
| **Stufen** | **4a** (gedrosselt) → **4b**: zuerst mit `fake_objects.py` im Robotersystem (hängt **nicht** an B23), dann mit der Basiskamera, falls B23 bestanden → **4d** mit dem Referenzklotz |
| **Nutzer** | Not-Aus; Klötze auflegen; Geschwindigkeit nach Regel 2 stufenweise freigeben. |
| **Claude** | Mitlesen wie Block 4. **B4:** `lead_time_s` aus `err_laengs` einmessen. **D23:** Montagewinkel der Backen am Foto/Augenmaß gegen `gripper_yaw_offset_deg`. **Erster Testgriff** bestätigt die 245 mm: greift der Greifer zu hoch/tief, ist die Differenz der systematische Versatz. **D22:** Dauer von Absenken, Greifen, Heben aus `follower_status` → `t_descend_s` usw. **B22:** bleibt die Ziel-ID während `ABSENKEN`/`GREIFEN`? **B18:** Toleranzen. **B9:** Ablagepose bei laufendem Programm gegenlesen. |
| **Abnahme** | Umsetzungsplan 4a, 4b, 4d am echten Roboter; ein Referenzklotz liegt in der Kiste. |

### Block 8 — Roboterkamera aufschalten (4c) · 30 min · nach Block 6 und 7

`weight_across`, dann `weight_along` schrittweise auf 1; `w_wirksam` im
Übersichtsbild. Abnahme: Umsetzungsplan 4c.

---

## 5. Empfohlene Aufteilung auf Termine

| Termin | Blöcke | Braucht |
|---|---|---|
| **A** (~2 h) | 0 · 1 · 2 · (3) | Rebuild; Band frei; Band an für 3 |
| **B** (~1,5 h) | 4 · 5 | virtueller Roboter; dann echter Roboter von Hand |
| **C** (~2 h) | 6 · 7 | B10 aus Termin B dokumentiert |
| **D** | 8 · Nacharbeit | B6 bestanden |

Fällt ein Termin kürzer aus: **Block 2 hat Vorrang vor allem anderen** — er ist
der einzige, der eine Blockade (B23) auflöst und dem Kalibrierprojekt zuarbeitet.
Block 4 braucht die Hardware kaum und lässt sich notfalls vorziehen.

---

## 6. Nach jedem Block

Claude trägt noch im Termin nach: Messwerte in `offene-punkte.md`, Herleitung als
neuer Eintrag in `entscheidungen.md` (fortlaufender Nachtrag), geänderte
Parameterwerte in `einrichtung-projektanwendung.md`. Specs einer Komponente
werden gelöscht, sobald ihre Abnahme am Aufbau bestanden ist (`docs/README.md`).
Der Nutzer committet und pusht.
