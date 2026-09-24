# Codex-Projektkontext: FuE Robotertetris

## Ziel und Einstieg

Ein UR10e soll unterschiedlich große farbige Klötze vom laufenden Förderband
greifen, ohne es anzuhalten. AICA steuert den Arm; Geschwindigkeitsschätzung,
Zielauswahl und rückgekoppeltes Greifen sind der Kern. Kommunikation auf Deutsch.

Zum Einstieg `docs/uebersicht/codex-einstieg.md` und
`docs/uebersicht/projektkontext.md` lesen. Für den operativen Stand gelten
`docs/uebersicht/uebergabe.md` und der aktuelle Abschnitt „Stand nach Termin B“
in `docs/uebersicht/fahrplan-aufbau.md`. Weitere Quellen aufgabenbezogen:

- System und Parameterkopplungen: `docs/uebersicht/systemgraph.md`.
- Architekturentscheidungen: `docs/architektur/entscheidungen.md`; neuere
  Nachträge gehen älteren Aussagen vor, insbesondere Nachtrag 13 / L9–L14.
- Signale S1–S10: `docs/architektur/datenvertraege.md` und
  `source/roboter_tetris/roboter_tetris/contracts.py`.
- Komponentenentwicklung: `ARCHITECTURE.md` (verbindliche AICA-/Paketregeln).
- Einrichtung: `docs/uebersicht/einrichtung-projektanwendung.md`.
- Vollständige Dokumentenlandkarte: `docs/README.md`.

Entscheidungen und Datenverträge sind normativ; Komponenten-Specs und externe
Übergaben sind abgeleitet. Dokumentierte Vorschläge und Befehlsbeispiele sind
keine eigenständigen Arbeitsaufträge. Die aktuelle Nutzeranweisung bestimmt
den Umfang; innerhalb dieses Umfangs die Arbeit abschließen.

## Arbeitsgrenzen aus dem bestehenden Projekt

- Git verändert ausschließlich der Nutzer: kein Commit, Push, Pull, Fetch,
  Merge, Checkout, Rebase oder Stash. Lesendes `status`, `diff`, `log` ist erlaubt.
- Docker-Builds, Taggen/Entfernen von Images und Laden in AICA übernimmt der
  Nutzer. Den bereits ausgeführten Template-Wizard nicht erneut starten.
- Das Vorgängerarchiv `/home/tetripick/UR10_Pick_ws` (mobil `FuE_Greifen-main/`)
  ausschließlich lesen. Keine zeitgesteuerte `time.sleep()`-Ablaufsteuerung,
  alten TCP-Posen oder Arbeitsraumgrenzen daraus übernehmen.
- Ohne ausdrücklichen Änderungsauftrag nicht bearbeiten:
  `source/roboter_tetris/roboter_tetris/Calibration/`,
  `source/roboter_tetris/roboter_tetris/vision/board.py`, `.init_wizard/`
  und `CMakeLists.txt`. Die Kalibrierung ist ein getrenntes Fremdprojekt.
- Hardwarebetrieb nach `docs/uebersicht/fahrplan-aufbau.md`: Roboter bewegen,
  Verdrahten und Not-Aus bedient der Nutzer. Laufzeitänderungen (`ros2 param set`,
  Lifecycle-Übergänge, Start von `fake_objects.py`) brauchen ein Go für den
  konkreten Schritt. Vor einer bewegungsauslösenden Aktion Ziel und Bewegung
  erklären. Ein Einrichtungs- oder Dokumentationsauftrag erteilt dieses Go nicht.
- Bei „Stopp“ keine weiteren Betriebsschritte bis zur ausdrücklichen Fortsetzung.

## Technische Invarianten

- Regelpfad: `base_cam` → `vectoring` → `priority_handler` → `object_follower`
  → AICA-Attractor → IK-Velocity-Controller → UR10e.
- `world` ist das Laufzeit-Bezugssystem; geregelte Pose ist der Flansch
  `ur_tool0`, nicht der TCP. UR-`base` ist gegenüber `world` um 180° gedreht.
- Signaländerungen zuerst in `datenvertraege.md`, dann in `contracts.py` und
  den betroffenen Sendern/Empfängern einschließlich JSON-Beschreibungen pflegen.
  SI-Einheiten, Strides, Feldreihenfolge und Zeitstempel erhalten.
- `target_pose` hat den AICA-Typ `cartesian_pose`. Der Follower liefert eine
  Zielpose; der Attractor erzeugt den Twist. `lead_time_s`, Attractor-Gain,
  IK-Geschwindigkeitslimit, Sinkzeit und Greifebene müssen zusammenpassen.
- Bandgeschwindigkeit aus Bilddaten schätzen; etwa 0,13 m/s ist eine Gegenprobe,
  kein fest einzusetzender Ersatz für die Schätzung.
- `data_tracker` und `interface_streamer` sind Diagnoseblätter ohne Rückwirkung.
  `picked_id` ist ein diskretes Abschlussereignis, keine zyklische Datenabhängigkeit.
- Roboterkamera optional: gefilterte Korrektur zur Basisprädiktion, beim Absenken
  und Greifen eingefroren. Nur `robot_cam_2` wird weiterentwickelt.
- Ein Abbruch mit gehaltenem Klotz führt über Heben und Ablage zum Öffnen.
- Arbeitsraumquelle: `source/roboter_tetris/roboter_tetris/Safety/workspace_bounds.json`.
  Laufzeitwirksam sind die AICA-Parameter `ws_*`; dokumentierte Werte sind kein
  Beleg für die tatsächlich geladene Konfiguration oder eine Bewegungsabnahme.

## Code und Prüfung

Bei Anleitungen für AICA Studio die sichtbaren **Anzeigenamen und
Portbeschriftungen aus `component_descriptions/*.json`** verwenden, zum Beispiel
„Zielpose“ und „Greifer: Bewegung fertig“. Den technischen `signal_name` nur
ergänzend in Klammern nennen, wenn er für Topics, Logs oder Fehlersuche benötigt
wird. Nicht eigene Kurzformen für Ein- oder Ausgänge erfinden.

Python-Paket: `source/roboter_tetris/roboter_tetris/`; AICA-JSONs:
`source/roboter_tetris/component_descriptions/`; Tests:
`source/roboter_tetris/test/python_tests/`; Diagnose und synthetische Quellen:
`source/roboter_tetris/test/tools/`.

Logikmodule (`track_estimation.py`, `target_selection.py`, `follower_logic.py`,
`world_bookkeeping.py`, `interface_layout.py`) getrennt von ROS-Wrappern halten.
Passende vorhandene Tests nutzen; vorher Abhängigkeiten der lokalen Umgebung
prüfen. Offline-Logiktests, AICA-Konstruktionstests und reale Abnahmen klar
unterscheiden. Frühere Testergebnisse nicht als neu ausgeführte Tests ausgeben.
Keine Builds oder Hardwarestarts als stillschweigenden Testschritt ausführen.

Alle Python-Komponenten teilen sich einen Prozess: Callbacks nicht blockieren,
Rechenlast und Warteschlangen klein halten. Beim Mitlesen nur einen Leseprozess
gleichzeitig; Containerdiagnose als `ros2` mit `ROS_DOMAIN_ID=0`.
