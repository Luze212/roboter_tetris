# Design: Robotiq Gripper Component

**Datum:** 2026-06-01
**Paket:** `roboter_tetris` (AICA Package, UR10e)
**Status:** Entwurf zur Umsetzung

## Zweck

Erste Komponente des Packages. Steuert einen an den Roboter-/AICA-Rechner per USB
angeschlossenen **Robotiq 2-Finger-Greifer 2F-140** über die Python-Library
`pyrobotiqgripper` (Klasse `RobotiqGripper`, Modbus RTU über USB).

Die Komponente reagiert auf zwei Eingangssignale (vollständig schließen/öffnen bzw.
auf eine Ziel-Öffnungsweite fahren) und meldet über Predicates ihren Status.

## Verbindliche Rahmenregeln

Diese Komponente unterliegt den Regeln aus `ARCHITECTURE.md` (verbindlich). Besonders relevant:

- **Niemals blockieren** in AICA-Callbacks, **kein `time.sleep()`**.
- **Kein `copy.deepcopy()`** auf `state_representation`-Objekte (hier nicht relevant, da keine sr-Signale).
- Strikte Ament-Struktur; Python-Modul im gleichnamigen Unterordner; Registrierung in `setup.cfg` mit `::`.
- Beschreibung als **YAML** in `extension_descriptions/` (Schema `1-0-2`).

## Komponente

| Eigenschaft | Wert |
|---|---|
| Klassenname | `RobotiqGripperComponent` (bewusst ≠ Library-Klasse `RobotiqGripper`) |
| Basisklasse | `modulo_components.lifecycle_component.LifecycleComponent` |
| Python-Datei | `source/roboter_tetris/roboter_tetris/robotiq_gripper.py` |
| Beschreibung | `source/roboter_tetris/extension_descriptions/roboter_tetris_robotiq_gripper.yaml` |
| Registrierung (`setup.cfg`) | `roboter_tetris::RobotiqGripperComponent = roboter_tetris.robotiq_gripper:RobotiqGripperComponent` |
| Hardware-Treiber | vendored `roboter_tetris.robotiq_driver` (Modbus RTU) |
| Dependencies | `pymodbus==3.6.9`, `pyserial==3.5` in `requirements.txt` |
| UI-Anzeigename | `Robotiq Gripper` |

## Schnittstellen

### Inputs (event-getrieben, je `user_callback`)

| UI-Signalname | Attribut | Typ | Verhalten |
|---|---|---|---|
| `gripper_close` | `_gripper_close` | `std_msgs.msg.Bool` | `True` → vollständig schließen (Stopp bei Objekt/Blockade). `False` → vollständig öffnen. |
| `gripper_change` | `_gripper_change` | `std_msgs.msg.Int32` | Nur wirksam wenn `gripper_close == False`: auf Ziel-Öffnungsweite in mm fahren (Stopp bei Erreichen **oder** Blockade). |

### Outputs

Keine Signal-Outputs (wie gefordert).

### Predicates

| Predicate-Name | Bedeutung | Quelle |
|---|---|---|
| `is_connected` | Serielle Verbindung steht **und** Greifer ist aktiviert. | Lifecycle (configure/activate → `True`, cleanup/Fehler → `False`). |
| `is_object_grasped` | Letzte Bewegung wurde durch ein Objekt gestoppt (Greifer hält etwas). | Worker-Thread aus Greifer-Status (`gOBJ`-Bits / `status()`); `False` sobald vollständig geöffnet. |

### Parameter (dynamisch, UI-anpassbar)

| UI-Name | Typ | Default | Beschreibung (inkl. Wertebereich für UI) |
|---|---|---|---|
| `force` | `double` | `50` | Greifkraft in Prozent (0–100 %). Intern linear auf 0–255 gemappt. |
| `grasping_speed` | `double` | `100` | Schließgeschwindigkeit in Prozent (0–100 %). Intern linear auf 0–255 gemappt. |

Der Prozent-Hinweis steht explizit in der `description` der YAML-Parameterdefinition,
damit der Bediener den erwarteten Wert einordnen kann. Parameter sind `dynamic` und
wirken ab der nächsten Bewegung.

## Hardware-Treiber (vendored)

Statt der Library `pyrobotiqgripper` wird ein schlanker, paketinterner Treiber
`roboter_tetris.robotiq_driver` verwendet. **Grund:** `pyrobotiqgripper` (move-API,
3.x) deklariert `numpy>=1.28` (faktisch numpy ≥ 2.0) und kollidiert damit mit dem
apt-installierten `numpy 1.26.4` des ROS-Jazzy-Images; AICAs Build kann diese
pip-Auflösung nicht überschreiben (kein `--no-deps`). Zusätzlich schleppt die
Library Desktop-GUI-Abhängigkeiten (pygame/pyautogui/pynput) mit, die auf einem
headless Steuerrechner unerwünscht sind.

Der Treiber implementiert nur die genutzte Methoden-Teilmenge direkt über
`pymodbus` + `pyserial` (beide ohne numpy-Konflikt) mit **identischen
Methodennamen/Signaturen** (`connect`, `activate`, `calibrate_bit`,
`calibrate_mm`, `move`, `move_mm`, `stop`, `disconnect`, `status`), sodass der
Komponenten-Code unverändert bleibt. Grundlage ist das dokumentierte
Robotiq-2F-Modbus-Registerlayout (Command-Register ab 0x03E8, Status-Register ab
0x07D0; `gOBJ` aus dem Statusbyte für Objekterkennung).

> `time.sleep` wird ausschließlich beim einmaligen Bring-up (Aktivierung/
> Kalibrierung in `on_configure`) sowie im blockierenden `wait=True`-Pfad genutzt —
> **nicht** im laufenden Zyklus. Der Worker fährt Bewegungen mit `wait=False` und
> pollt über sein eigenes `threading.Event`, nie über `time.sleep`.

## Ausführungsmodell (gewählter Ansatz)

**Worker-Thread + nicht-blockierendes `move(..., wait=False)` mit eigener Poll-Schleife.**

Begründung: Alle Bewegungsmethoden von `pyrobotiqgripper` blockieren standardmäßig
(`wait=True`) bzw. pollen den Greifer per Modbus bis zum Bewegungsende. AICA-Callbacks
dürfen nicht blockieren. Daher:

- Die AICA-Input-Callbacks **setzen nur** ein thread-sicheres Ziel und wecken den Worker
  (kein Hardware-I/O im Callback).
- Ein **dedizierter Worker-Thread** besitzt exklusiv die serielle Verbindung, schickt
  Bewegungsbefehle mit `wait=False` und pollt selbst `status()`/Position, bis Ziel
  erreicht oder Blockade erkannt — oder ein neues Ziel eintrifft.
- **Preemption:** Trifft während einer laufenden Bewegung ein neues Ziel ein (z. B.
  `gripper_close=True` überstimmt eine Change-Bewegung), ruft der Worker `stop()` und
  startet sofort den neuen Befehl.

Verworfen: blockierendes `wait=True` (ruckelige Preemption, „erreicht vs. abgebrochen"
schwer unterscheidbar) und I/O direkt im Callback (verstößt gegen „niemals blockieren").

## Vorrang- und Zustandslogik

`gripper_close` hat **Vorrang** vor `gripper_change`. Der Worker hält ein `current_target`:

| Ereignis | Neues Ziel |
|---|---|
| `gripper_close` False→True | Voll schließen |
| `gripper_close` True→False | Voll öffnen |
| `gripper_change` (neuer Wert) **und** `close == False` | Auf Öffnungsweite X mm fahren |
| `gripper_change` während `close == True` | **Ignoriert** |

Flankenerkennung: Die Callbacks vergleichen mit dem zuletzt angewandten Wert, damit
wiederholt publizierte gleiche Werte keine erneute Bewegung auslösen.

## Einheiten-Mapping

- **Force/Speed:** `raw = clamp(round(pct / 100 * 255), 0, 255)`. Bei jedem Bewegungsbefehl
  aus den aktuellen Parameterwerten gelesen.
- **Öffnungsweite (`gripper_change`):** mm, Wertebereich 0–130 (kalibrierter Bereich,
  siehe unten). `0 mm` = vollständig geschlossen, `130 mm` = vollständig geöffnet.
  Umsetzung via `move_mm(positionmm)` (erfordert mm-Kalibrierung beim Start).
- **Voll schließen / voll öffnen:** `close()` / `open()` (bzw. `move(255)` / `move(0)`).

## Lifecycle

| Callback | Aktionen |
|---|---|
| `on_configure` | `RobotiqGripper(com_port='auto')` → `connect()` → `activate()` → `calibrate_bit()` (auto) → `calibrate_mm(0, 130)`. Bei Fehler: Log + `return False`. Setzt `is_connected = True`. |
| `on_activate` | Worker-Thread starten; Befehlsverarbeitung aktiv. `return True`. |
| `on_deactivate` | Laufende Bewegung `stop()`; Verarbeitung pausieren. |
| `on_cleanup` / `on_shutdown` | Worker beenden + joinen; `stop()`; `disconnect()`; `is_connected = False`. |
| `on_error` | Best-effort `stop()`/`disconnect()`; `is_connected = False`. |

> ⚠️ **Inbetriebnahme-Hinweis:** `activate()`/`calibrate_bit()` führen beim Start eine
> vollständige Öffnungs-/Schließfahrt aus. Die Greiferfinger müssen dabei frei beweglich sein.

Bewegungen werden nur im Zustand **ACTIVE** ausgeführt.

## Fehlerbehandlung

- **Bring-up-Fehler** (kein USB-Gerät, Modbus-Timeout, Aktivierung scheitert): betroffene
  Lifecycle-Transition gibt `False` zurück, Fehler wird geloggt, `is_connected = False`.
- **Laufzeitfehler im Worker** (z. B. I/O-Fehler während einer Bewegung): gekapselt,
  geloggt; die Node bleibt am Leben. Bei wiederholtem Verbindungsverlust `is_connected = False`.

## Deployment-Voraussetzung (außerhalb des Komponenten-Codes)

Das USB-Gerät des Greifers muss in den AICA-Container durchgereicht werden
(`--device`/AICA-Hardware-Konfiguration). Gehört in die Inbetriebnahme, nicht in den Code.

## Dateien (neu/geändert)

- **Neu:** `source/roboter_tetris/roboter_tetris/robotiq_gripper.py`
- **Neu:** `source/roboter_tetris/roboter_tetris/robotiq_driver.py` (vendored Modbus-Treiber)
- **Neu:** `source/roboter_tetris/extension_descriptions/roboter_tetris_robotiq_gripper.yaml`
- **Geändert:** `source/roboter_tetris/setup.cfg` (Registrierung der neuen Komponente)
- **Geändert:** `source/roboter_tetris/requirements.txt` (`pymodbus==3.6.9`, `pyserial==3.5`)
- **Neu (Tests):** `source/roboter_tetris/test/python_tests/test_robotiq_gripper.py`
  (Konstruktion/Lifecycle mit gemocktem `RobotiqGripper`, ohne echte Hardware)

## Testbarkeit

Die Hardware-Library wird in Tests gemockt (kein realer Greifer im CI). Geprüft werden:
Konstruktion, Parameter-Mapping (% → 0–255, Clamping), Vorrang-/Flankenlogik des Ziels,
Predicate-Übergänge. Der Worker-Thread wird mit einem Fake-Gripper deterministisch getestet.

## Bewusste YAGNI-Entscheidungen

- Kein optionaler Port-Parameter (Library nutzt `com_port='auto'`). Nachrüstbar, falls Auto-Detect fehlschlägt.
- Keine physikalischen Einheiten (N, mm/s) für Force/Speed — Prozent ist gewählt.
- Keine Signal-Outputs.
