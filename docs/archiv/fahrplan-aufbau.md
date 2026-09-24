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
| 6 | 🟡 **Hand-Auge neu eingemessen und bestätigt** (2–4 mm, L11, L13). Erkennung von `robot_cam_2` an flachen Klötzen unzuverlässig (bis 32 mm) → ~~neuer Erkennungskern~~ nicht weiter verfolgt (L22); erster Griff ohne Roboterkamera |
| 5 | ✅ **Arbeitsraum und Greifzone festgelegt** (L14): ws x −1,0 … −0,30, y −0,32 … +0,48, z 0,3086 … 0,60 (seit L26 z min 0,3036); Zone y +0,40 … −0,22 (seit L21 = Arbeitsraum) |

**Entscheidungen des Tages:** Greifzone und Wartebereich **außerhalb des Bildes
der Basiskamera** (L4); die Strecke dahinter überbrückt die Roboterkamera.

## Stand nach Termin C (24.09.2026)

| Block | Ergebnis |
|---|---|
| Rechenlast | Kameras auf 15 Bilder/s; `base_cam` unverändert 8,6 Messungen/s. **Keine NVIDIA-GPU** im Rechner (L16). Infrarot der Roboterkamera abschalten hängte AICA auf — `camera_node` leer (L16) |
| Follower | Legte **keine Signale** an (Parameterprüfung), korrigiert (L17). Arbeitsraum und Beobachtungspose sind Standardwert (L15) |
| 7 | ✅ **Gedrosselt bestanden (L18)** mit `fake_objects.py`: Folgen, Absenken, Greifen (Fehlgriff ins Leere), Abbruch, Heben — bei 0,07 m/s (IK 0,10) und 0,13 m/s (IK 0,30). Attractor auf K = 5 / 50 Hz; `err_laengs` −2,5 bzw. −5 mm → `lead_time_s` 0,24 |
| **4d** | ✅ **Erste echte Griffe im Lauf (L19):** 100-mm- und 75-mm-Klotz mit Basiskamera bei 0,13 m/s gegriffen und abgelegt, `err_laengs` ≈ +1 mm; Ablagepose (B9) und 245 mm bestätigt |
| ⚠️→✅ | **Schutzstopp am Bandrand** (C157A2/C162A0): Nutzlast 2,8 → 1,3 kg gemessen, IK `command_rate_limit` 2,0 → Bandrand x −0,94 gegriffen; 7 von 7 abgelegt (L20) |
| Erkennung | Band im Bild ~0,8 m breit; Ausschnitt schnitt den roboternahen Rand an, Zone kleiner als der Arbeitsraum → **Zone = Arbeitsraum, `roi_x` 342 / `roi_width` 618** (L21, Standardwert) |
| Roboterkamera | **Nicht mehr eingebunden (L22):** Follower ohne Stufe 4c, Streamer ohne Roboterkamerabild; die Blöcke bleiben im Paket. Verhalten des Followers vorher/nachher Takt für Takt identisch; nach dem Build am Aufbau bestätigt, 10 von 10 abgelegt. Block 6 abgeschlossen, Block 8 entfällt |
| Abbrüche beim kleinen Klotz | **Vorhersagedeckel 0,6 → 1,0 s, Ziel-Timeout 1,5 s**; `priority_handler` mit `t_settle_s` 1,2 s (pessimistische Erreichbarkeit). Danach 9 von 9 abgelegt (L23) |
| ⚠️ | **Einbruch der 500-Hz-Schleife** (4/500 Hz, ~1 s) beim Kontrolllauf → External Control stoppte. Schleife auch in Ruhe nur 84–86 % (L18) |
| Tempo | **Ausgereizt (L24):** L23 zu pessimistisch — aus der Ablagepose wurde nichts in der Zone gewählt. Gemessen Einschwingen 0,3 s → `t_settle_s` 0,4, Faktor 1,0 (0,6 wählte Unerreichbares, 3 zu spät); 0,5 m/s, Absenken 0,25 m/s → **15 abgelegt, 0 zu spät, ~1 Klotz je 7 s** |
| Flache Klötze | ✅ Ab 2 cm greifbar (L24): `min_graspable_height_m` 0,02, `min_contour_area` 1000 (`min_obj_height` bleibt 15); geschlossene Backen 11 mm über dem Band |
| Modus 2 | ✅ **Gedrehte Klötze im Winkel gegriffen** (L25, L26): höchstens ±45° aus der Grundstellung; Winkel der Basiskamera im Lauf stabil (Güte 1,00); Attractor `angular_gains` [5.0], Drehung 1,0 rad/s |
| **Finaler Build** | ✅ **L26:** alle am Aufbau gefahrenen Werte als Standard — Modus 2 an, Greifhöhe min 0,016 / `ws_z_min` 0,3036 (Backen 6 mm über dem Band), Debug-Bild der Basiskamera an |
| Dauerlauf | ✅ Gemischte und gedrehte Klötze, **läuft zuverlässig** (Nutzer, L26) |

### Weiter am nächsten Termin

**Stand 24.09. spätabends — finaler Build (L26):** Alle am Aufbau gefahrenen Werte
sind Standardwert im Paket. Nach dem Build ist nichts von Hand einzutragen außer den
Werten der AICA-Bausteine (Schritt 1).

0. ✅ **`--target test` grün** (24.09.): die Tests von `test_base_cam_contract` und
   `test_interface_streamer` übersprungen (`cv_bridge` fehlt im Testabbild), alle
   übrigen bestanden, die Konstruktionstests erstmals seit Juni. `conftest.py` ist seit L26
   wieder da; die Konstruktionstests laufen damit zum ersten Mal seit Juni.
1. **Einstellungen außerhalb des Pakets** (von Hand, Einrichtung §2/§3): Attractor
   `linear_gains` [5.0], `angular_gains` [5.0], `rate` 50, `max_linear_velocity` 0,5,
   `max_angular_velocity` 1,0; IK-Controller `max_linear_velocity` 0,5,
   `max_angular_velocity` 1,0, `command_rate_limit` 2,0; `rate` Follower 50,
   `priority_handler`/`vectoring` 20, `robotiq_gripper` 20, `interface_streamer` 10,
   `data_tracker` 2; UR-Nutzlast 1,3 kg. **Verdrahtung:** `target_pose` → Attractor,
   `picked_id` → `priority_handler` und `data_tracker`, `gripper_close` → Greifer.
   Vor den Läufen Nebenprogramme beenden, AICA-Oberfläche minimieren.
2. ✅ **Dauerlauf** mit gemischten, auch gedrehten Klötzen (Ziel 4): läuft zuverlässig
   (Nutzer, 24.09., L26).
3. **Rechenlast** (optional): Aufzeichnung mit und ohne Debug-Bild/Streamer; bleiben
   Einbrüche der 500-Hz-Schleife, Rate des Hardware-Interface prüfen.
4. Offen am Schreibtisch: mehr Takt nur über eine Ablagepose näher am Band (L24);
   Verhalten in GREIFEN bei kurz stehendem Ziel (L23); `fake_objects.py` mit realer
   Latenz.

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
   stimmt, auf den Betriebswert **0,30** (Einrichtung §2, L18). Gedrosselt mit Absenken nur mit
   `descend_speed_mps` 0,05 und `t_descend_s` 3,0 — die IK-Grenze gilt für den Betrag (L18). ⚠️ Das Band läuft mit ≈ 0,13 m/s
   (Nachtrag 13 / L7) — gedrosselt lässt es sich **nicht** einholen. Gedrosselte
   Läufe deshalb mit `fake_objects.py` und kleiner Geschwindigkeit, mit laufendem
   Band erst mit ≥ 0,30 m/s.
3. **Am echten Roboter nur mit dokumentiertem Arbeitsraum** (B10, nach
   `Safety/README.md`) — seit 23.09.2026 festgelegt (L14) und Standardwert im
   Follower (L15); Einrichtung §9.
4. **Sofortabbruch durch Claude:** `fake_objects.py` läuft als Hintergrundprozess
   und wird bei jeder Unklarheit sofort beendet. Das stoppt die Zielquelle; der
   Attractor hält dann die letzte Zielpose (Thema 7). ⚠️ Mit der Basiskamera als
   Zielquelle gibt es diesen Hebel nicht — dann ist der Not-Aus die einzige
   Sicherung (L19).
5. Ruft der Nutzer **„Stopp“**, führt Claude keinen weiteren Schritt aus, bis der
   Nutzer ausdrücklich weitermacht.
6. **Vor jedem Lauf prüfen:** Abnehmer von `target_pose` ist der Attractor, nicht
   der IK-Controller (L18); UR-Nutzlast 1,3 kg, IK `command_rate_limit` gesetzt (L20).

---

## 3. Vorbereitung — ohne Roboter

| Wer | Was | Stand |
|---|---|---|
| Claude | **`test/tools/signal_reader.py`** — liest im Container mit, ausschließlich lesend: Topics, Kameras (Serial, `global_time`, Belichtung), Zeitstempel-Drift, Flanschpose, Gelenke, alle Vertragssignale S1–S10 über `contracts.unpack_*`. Live-Zeilen, CSV, JSON-Zusammenfassung | ✅ 22.09.2026, offline gegen `contracts.pack_*` getestet |
| Claude | **`test/tools/b23_compare.py`** — Punktpaare Basiskamera ↔ Flansch: Identität, 180°-Hypothese, starre 2D-Ausgleichsrechnung mit Restfehlern, B21-Streuung, Höhenvergleich. Läuft auf dem Rechner, reines Python | ✅ 22.09.2026, an synthetischen Daten geprüft (183° / 12 mm vorgegeben → zurückgewonnen) |
| Nutzer | Rebuild und Systemabbild neu erzeugen, sobald das Kalibrierprojekt es zulässt | — |
| Nutzer | Referenzklotz 50×50×100 und 2–3 weitere Klötze bereitlegen; Band frei von Zetteln? | — |

Beide Skripte liegen im Repo (`test/tools/`).

**Weitere Messverfahren vom 24.09.2026** — als Wegwerfskripte im Scratchpad, nicht im
Paket (CLAUDE.md §5); bei Bedarf nach diesem Muster neu schreiben:

| Zweck | Verfahren |
|---|---|
| Lauf mitlesen (Block 7, echte Griffe) | ein Prozess abonniert `follower_status`, `object_follower/target_pose`, `priority_handler/target`, `gripper_close` und gibt 2 Zeilen/s aus: Zustand, Ziel-ID, `err_laengs`/`err_quer`, Zielpose mit Gier. **Nicht** die Flanschpose mit 500 Hz dekodieren — das hat einen Einbruch der Regelschleife mit ausgelöst (L18). Beim Fake-Lauf zusätzlich ein Wächter, der den Fake beendet, wenn die Zielpose den erwarteten Quader verlässt |
| Abbrüche beim Absenken aufklären (L23) | CSV mit jedem `follower_status` (50 Hz) und jedem neuen S4 (x, y, t, v, Greifebene). Auswertung: Alter von S4 je Takt, Fehlerverlauf um jeden Rücksprung ABSENKEN → FOLGEN, Sprünge der S4-Position gegen die lineare Fortschreibung |
| Umbau des Followers absichern (L22) | Vor dem Umbau `FollowerCore` in 12 Szenarien im geschlossenen Regelkreis (Attractor K = 5, 0,30 m/s, Basiskamera 8 Hz mit 0,15 s Latenz, Greifer-Nachbildung) Takt für Takt als JSON aufzeichnen, danach erneut — Zielpose, Zustand, Greiferbefehl, Fehler, `picked_id` und Meldungen müssen identisch sein. `lead_time_s` im Modell 1/K = 0,2 (keine Transportverzögerung) |
| Bildausschnitt prüfen (L21) | ein Farb- und ein Tiefenbild der Basiskamera holen und `vision.detection.detect_objects` offline mit verschiedenen `roi` laufen lassen; ROI-Ecken über Intrinsik und Extrinsik auf die Bandebene projizieren |
| Tempo und Erreichbarkeit einstellen (L24) | CSV wie L23, dazu jede neue ID in `priority_handler/not_pickable` (= Klotz ohne Versuch durch die Greifebene) und jedes `picked_id` mit Ergebnis. Auswertung je Versuch: Dauer jedes Zustands, Klotzlage (S4 auf die Wandzeit fortgeschrieben) bei der Wahl und bei Absenkbeginn, Reserve bis zur Greifebene, Wahl direkt aus der Ablage ja/nein. Parameterstand einmalig per `list_parameters`/`get_parameters` je Knoten (nur lesen, nicht während eines Laufs) und gegen die Standardwerte der Komponentenbeschreibungen vergleichen. Leeres Band: 60 s `objects` mitlesen, Arm aus dem Bild |

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
| 6 | `R object_position --live` (Block 6 abgeschlossen, L22) |

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
| **Claude** | Nach Go: `fake_objects.py` auf das `objects`-Topic. Mitlesen: `target`, `follower_status` (Zustand, Abweichungen), `target_pose`, `picked_id`. |
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

> **Abgeschlossen, nicht weiter verfolgt** (Nachtrag 13 / L22): Hand-Auge neu
> eingemessen (L11), die Erkennung blieb unzuverlässig (L13), gegriffen wird ohne
> Roboterkamera.

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

> **Entfällt** (Nachtrag 13 / L22): Stufe 4c ist aus dem Follower entfernt.

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
