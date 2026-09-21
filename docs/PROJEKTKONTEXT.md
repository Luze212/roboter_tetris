# Wissensarchiv Robotertetris

Stand der Zusammenführung: 19.09.2026

## Zweck und Verwendung

Diese Datei ist ein **Nachschlagewerk für früheres Projektwissen**. Sie sammelt technische Fakten, alte Versuche, damalige Konfigurationen, Fehlersymptome und diskutierte Lösungsansätze. Vieles kann inzwischen veraltet oder verworfen sein.

Die Inhalte sind deshalb keine automatische Vorgabe für neue Implementierungen. Vor Änderungen ist der aktuelle Repository- und Hardwarestand maßgeblich.

Die Einträge werden nach ihrer Herkunft eingeordnet:

- **Aktueller Repository-Fakt:** am 19.09.2026 im lokalen Repository gefunden.
- **Historischer Projektstand:** vom Nutzer in früheren ChatGPT-Chats berichtet oder damals verwendet.
- **Alter Vorschlag:** diskutierte Idee oder von ChatGPT vorgeschlagene Lösung; keine Bestätigung einer Implementierung.
- **Offen:** gegen den aktuellen Stand zu prüfen.

### Aktuelle Vorgabe für Roboterbewegungen

Der frühere Ansatz **`MoveToPose` wird nicht fortgeführt**. Die Aufgabe „UR10e auf vorgegebene Koordinaten fahren“ soll später **von Grund auf neu konzipiert und umgesetzt** werden. Alte `MoveToPose`-Entwürfe, Interfaces, Beispielklassen, Waypoints und Payloads dienen nur als Historie und Fehlerreferenz. Sie sind keine Architekturvorgabe für die neue Lösung.

## Quellen

Zusammengeführt wurden:

- der vollständige ChatGPT-Export `PROJECT_CONTEXT_FuE_Robotertetris.md`,
- die über die ChatGPT-Projektansicht zugänglichen Unterhaltungen,
- das lokale Repository, insbesondere `ARCHITECTURE.md`, `aica-package.toml` und die vorhandenen Komponenten,
- der bisherige Codex-Chat zu Organisation, Git und Projektkontext.

Der ChatGPT-Export enthält neben Nutzeraussagen auch frühere Modellantworten und Empfehlungen. Diese werden hier nicht als Anweisungen behandelt.

## Historisches Projektziel

Das Projekt verbindet einen Universal Robots UR10e, einen Robotiq-Greifer und RealSense-Kameras zu einem Robotertetris-/Pick-and-Place-System für Tetrominos oder ähnliche Objekte.

Als früher angestrebter Gesamtablauf wurde diskutiert:

1. Objekt mit einer Kamera erfassen.
2. Form beziehungsweise Tetromino-Klasse erkennen.
3. Position, Orientierung und gegebenenfalls Bewegung des Objekts bestimmen.
4. Tiefeninformationen zur räumlichen Lokalisierung verwenden.
5. Kamera- in Roboter- oder Weltkoordinaten transformieren.
6. Eine Greifpose und Roboterbewegung erzeugen.
7. Den separat angebundenen Robotiq-Greifer schließen.
8. Objekt transportieren und an einer Zielposition ablegen.

Eine endgültige Strategie zur Auswahl der Tetris-Ablageposition ist in den alten Chats nicht dokumentiert.

## Hardwareinventar

### Roboter und Greifer

- **Historischer Projektstand:** Universal Robots **UR10e**.
- **Historischer Projektstand:** Robotiq 2-Finger Adaptive Gripper.
- **Aktueller Repository-Fakt:** Die Komponente bezeichnet das konkrete Modell als **Robotiq 2F-140**.
- **Historischer Projektstand:** Der Greifer hängt über USB-RS485 direkt am Ubuntu-PC und nicht am UR-Controller.
- **Historischer Projektstand:** Die Kommunikation erfolgt über Modbus RTU beziehungsweise einen Modbus Serial Client.

### Kameras

- **Historischer Projektstand:** Intel RealSense **D435i**, USB-ID `8086:0b3a`.
- **Historischer Projektstand:** Intel RealSense **L515**, USB-ID `8086:0b64`.
- Beide Kameras wurden laut alten Chats vom Rechner erkannt und bereits in Programmen verwendet.

### Rechner und Verbindungen

- Ubuntu-PC als zentrale Recheneinheit für AICA, Vision, Roboter- und Greiferkommunikation.
- D435i und L515 über USB.
- Robotiq über USB-RS485.
- UR10e über Ethernet.
- WLAN wurde parallel für Internetzugang verwendet; die lokale Roboterkommunikation benötigt selbst kein Internet.

Historisches Verbindungsschema:

```text
D435i -------- USB --------> Ubuntu-PC
L515 --------- USB --------> Ubuntu-PC
Robotiq -- RS485/USB ------> Ubuntu-PC
UR10e <------ Ethernet ----> Ubuntu-PC
```

## Software- und Versionswissen

### Aktuell im Repository gefunden

- AICA Package Builder: `v1.4.0`.
- AICA-Systemimage: `v2.0.5-jazzy`.
- `control-libraries`: `v9.2.0`.
- `modulo`: `v5.2.2`.
- Pakettyp: ROS/Ament-basiertes AICA Custom Package.
- Python-Komponenten auf Basis von Modulo `LifecycleComponent` beziehungsweise `Component`.
- OpenCV-basierte Bildverarbeitung.
- Docker-Build über `aica-package.toml`.

### Historisch aus ChatGPT-Chats

Folgende Werte wurden in einer früheren AICA-UR-Konfiguration verwendet:

```text
AICA Core: v5.0.0
AICA Schema: 2-0-6
UR10e update rate: 500
robot_ip: 192.168.96.221
```

Diese Werte beschreiben möglicherweise eine frühere Laufzeitkonfiguration und widersprechen nicht zwingend den Build-Abhängigkeiten des heutigen Pakets. Vor Wiederverwendung müssen laufendes AICA-System, Konfigurationsdateien und Repository gemeinsam geprüft werden.

Weitere früher verwendete oder diskutierte Technik:

- ROS 2 und `ros2_control`,
- AICA Universal-Robots-Hardware-Interface,
- URSim,
- Intel RealSense SDK beziehungsweise RealSense-AICA-/ROS-Komponenten,
- `pyrobotiqgripper`, `pymodbus` und `pyserial`,
- ZeroMQ als mögliche Verbindung zwischen Teilsystemen; keine fertige Umsetzung bestätigt,
- YOLO als mögliche Ergänzung der klassischen Bildverarbeitung; keine fertige Umsetzung bestätigt.

## Aktueller Repository-Überblick

Am 19.09.2026 waren unter anderem folgende Komponenten vorhanden:

- `base_cam.py`: Verarbeitung der stationären/Basiskamera, Depth- und Kamerainformationen sowie Kamera-zu-Roboter-Transformation.
- `robot_cam.py`: Feinlokalisierung mit einer Kamera am Roboter beziehungsweise Greifer.
- `robot_cam_2.py`: alternative kantenbasierte Feinlokalisierung.
- `board_detection.py`: Erkennung beziehungsweise Referenzierung des Spielfelds.
- `robotiq_gripper.py`: AICA-Komponente für den Robotiq-Greifer.
- `vision/detection.py`, `color_estimation.py`, `tracker.py`, `robot_detection.py` und `robot_detection_edge.py`: wiederverwendbare Bildverarbeitungslogik.
- Tests für Basis- und Roboterkameras, Board-Erkennung und Greifer.

Die Existenz einer Datei bedeutet nicht automatisch, dass die gesamte End-to-End-Kette am realen System funktioniert.

## UR10e, AICA und Controller – historisches Wissen

### Genannte Controller

In den alten Chats kamen folgende Controller vor:

- `robot_state_broadcaster`,
- `joint_trajectory_controller`,
- `scaled_joint_trajectory_controller`.

In einer damaligen Projektkonfiguration wurden `robot_state_broadcaster` und `joint_trajectory_controller` genannt. Andere Beispiele verwendeten den skalierten Controller. Welcher Controller heute geladen, aktiv und für neue Bewegungen geeignet ist, bleibt zu prüfen.

Historisch genannte Plugin-Bezeichnungen:

```text
aica_core_controllers/RobotStateBroadcaster
aica_core_controllers/.../trajectory/JointTrajectoryController
```

Die exakte aktuelle Registrierung darf daraus nicht abgeleitet werden.

### Frühere Waypoint-/Trajektorienversuche

Ein alter AICA-Aufbau enthielt drei kartesische Frames:

```text
wp_1
wp_2
wp_3
```

Alle wurden relativ zu `world` beschrieben. Als `set_trajectory`-Payload wurde damals diskutiert beziehungsweise verwendet:

```yaml
frames: [wp_1, wp_2, wp_3]
durations: [2.0, 2.0, 2.0]
blending_factors: [1.0, 1.0]
```

Die konkreten Posen sind nicht erhalten. Der heutige Repository-Leitfaden beschreibt sowohl ein `JointTrajectory`-Signal als auch einen `set_trajectory`-Service mit `StringTrigger`. Dieses Wissen ist für eine spätere Bestandsaufnahme nützlich, legt aber die neue Bewegungsarchitektur nicht fest.

### Frühere Voraussetzungen für reale Bewegung

Bei ausbleibender Bewegung wurden wiederholt geprüft:

- UR10e eingeschaltet und Bremsen gelöst,
- Remote Control aktiv,
- External-Control-Programm beziehungsweise passender externer Steuerungsmodus aktiv,
- Roboterprogramm auf Play,
- kein Protective Stop,
- kein blockierender Safety-/Reduced-Mode,
- benötigter Controller geladen und aktiv,
- Zielpose erreichbar und IK lösbar,
- Roboter-IP und Route korrekt.

## Neustart der Aufgabe „auf Koordinaten fahren“

Historisch war eine eigene Komponente mit dem Arbeitstitel `MoveToPose` geplant. Genannt wurden Eingänge für `x`, `y`, `z`, optional `rx`, `ry`, `rz`, Geschwindigkeit und Beschleunigung sowie Zustände wie `busy`, `finished` und `error`.

Der alte Beispielcode war generisch und nicht zuverlässig an die tatsächlich eingesetzte AICA-/Modulo-Version angepasst. Auch die alte Interface-Idee gilt nicht als übernommen.

Für die neue Umsetzung ist später neu zu entscheiden:

- welche Eingabe wirklich benötigt wird: Zielpose, Frame-Name, Gelenkziel oder anderer Vertrag,
- welches Koordinatensystem gilt,
- ob Orientierung fest, relativ oder frei vorgegeben wird,
- welcher AICA-Controller und welche Schnittstelle tatsächlich aktiv sind,
- wie Erreichbarkeit, IK, Geschwindigkeit, Safety und Abschlussstatus geprüft werden,
- wie Simulation und realer Roboter getrennt konfiguriert werden.

## URSim – historischer Stand

URSim wurde zum Testen ohne reale Hardware verwendet:

```text
URSim-IP: 192.168.56.101
Web/VNC: http://192.168.56.101:6080/vnc.html
RTDE-Port: 30004
```

Erreichbarkeit wurde beispielsweise geprüft mit:

```bash
nc -zv 192.168.56.101 30004
```

Generische Beispiele mit `127.0.0.1` oder anderen Adressen waren keine bestätigte Projektkonfiguration. URSim eignet sich für Kinematik, Controller und Abläufe, bildet reale Dynamik, Lasten, Greiferinteraktion, Kamerakalibrierung und reales Timing nur begrenzt ab.

## Netzwerk PC ↔ UR10e

### Zuletzt rekonstruierter historischer Stand

Nach Problemen über Router beziehungsweise Switch wurde eine direkte Ethernet-Verbindung eingerichtet:

| Gerät | Schnittstelle | Historische Adresse |
|---|---|---|
| UR10e | Ethernet | `192.168.96.221/24` |
| Ubuntu-PC | `enp86s0` | `192.168.96.10/24` |

Subnetzmaske: `255.255.255.0`. Bei direkter Verbindung war kein Gateway notwendig.

Die PC-Adresse wurde zunächst temporär gesetzt. Ob später ein dauerhaftes NetworkManager-Profil angelegt wurde, ist nicht dokumentiert.

Weitere damalige Netzwerte:

- WLAN: `10.172.87.62/19` (dynamischer Momentwert),
- Docker-Bridge `br-798edde2b16f`: `192.168.56.1/24`,
- `docker0`: `172.17.0.1/16`.

### Früheres Fehlerbild

- Teach Pendant zeigte zeitweise `0.0.0.0` beziehungsweise keine Netzwerkverbindung.
- PC-Ethernet hatte zunächst keine IPv4-Adresse.
- `Destination Net Unreachable` trat auf.
- AICA startete teilweise, blieb beim Laden der Anwendung aber stehen.

Die direkte Verbindung mit statischen IPs stabilisierte den damaligen Aufbau. Das beweist nicht, dass heutige Verbindungsprobleme dieselbe Ursache haben.

## AICA und Docker – alte Probleme und Erkenntnisse

Historisch wurde das eigene Paket so referenziert:

```text
docker-image://roboter-tetris:latest
```

Für die Verwendung müssen der beim Build gesetzte Tag, das lokal vorhandene Image und die AICA-Referenz exakt übereinstimmen. Ein Image muss nicht manuell als Container gestartet werden; AICA lädt die Komponenten aus dem Image.

Ein damaliges Fehlerbild:

- AICA-Launcher lief.
- Beim Start der Anwendung luden Komponenten nicht vollständig.
- In `docker ps` war nur ein `aica-launcher-...`-Container sichtbar.
- Das UR10e-Hardware-Interface meldete erfolgreiche Initialisierung.
- Eine Lizenzstatusabfrage lieferte HTTP `404`.
- Eine Warnung meldete eine bereits geladene URDF.

Die Netzwerkstörung war damals ein plausibler Hauptfaktor. Die `404`-Meldung und die URDF-Warnung wurden nicht als abschließend bestätigte Hauptursache identifiziert.

## Robotiq-Greifer

### Historisches Architekturwissen

Der Greifer sollte als vom UR10e unabhängiger Pfad betrieben werden:

```text
AICA-Komponente
  -> Robotiq-Python-Bibliothek
  -> Modbus RTU
  -> USB-RS485-Adapter
  -> Robotiq-Greifer
```

Früher wurden als Alternativen eine URCap-/UR-Controller-Anbindung, ein eigenes AICA-Hardware-Interface und ein Python-Wrapper diskutiert. Die Python-Integration über den PC wurde damals bevorzugt.

Ein einfaches altes Interface sah `open()`, `close()`, `move(position, speed, force)` und `status()` vor. Event-zu-Latch-/Bool-Logik für dauerhaftes Öffnen oder Schließen wurde ebenfalls nur konzeptionell diskutiert.

### Aktuell im Repository gefunden

- Modellbezug: Robotiq **2F-140**.
- Bibliothek: `pyrobotiqgripper` 2.x.
- Standard-Portparameter: `/dev/ttyUSB0`; `auto` ist als Alternative vorgesehen.
- Standard-Modbus-Geräte-ID: `9`.
- Blockierende serielle I/O läuft in einem dedizierten Worker-Thread.
- Der Worker ist alleiniger Besitzer der nicht thread-sicheren Modbus-Verbindung.
- Die aktuelle Komponente öffnet den seriellen Port pro Operation, verwendet kurze Wiederholungsversuche und trennt die Verbindung anschließend wieder.

Damit ist der ältere Vorschlag „Port einmal öffnen und dauerhaft halten“ im heutigen Repository überholt. Hintergrund war, dass AICA langlebige Komponentencontainer verwendet und ein dauerhaft offener Port Neustarts beziehungsweise erneutes Laden mit `Failed to connect` blockieren konnte.

Nicht aus dem Export zuverlässig bekannt war die Baudrate; sie sollte bei Bedarf aus Bibliothek, Gerät und laufendem System ermittelt werden.

## RealSense-Kameras

### D435i

- Lieferte laut altem Projektstand bereits ein Bild in AICA.
- Verwendete Tiefenauflösung: `848 × 480`.
- Beobachtung: Objekte wirkten horizontal breiter, während die vertikale Darstellung plausibel war.

Historisch diskutierte Ursachen:

- Anzeige mit falschem Seitenverhältnis,
- Intrinsics einer anderen Stream-Auflösung,
- falscher Wert für `fx`,
- nachträgliche Skalierung,
- fehlerhaftes Depth-Color-Alignment,
- unterschiedlicher Pixel-zu-Millimeter-Faktor in X und Y,
- fehlerhafte Kamera- oder Extrinsic-Kalibrierung.

Zur Diagnose muss unterschieden werden, ob bereits das Rohbild verzerrt erscheint oder erst die berechneten metrischen Koordinaten. Relevante Werte sind `width`, `height`, `fx`, `fy`, `cx` und `cy` aus der tatsächlich laufenden `CameraInfo`.

### L515

Die L515 sollte als zweite RealSense-Instanz eingebunden und über ihre Seriennummer statt nur über einen Device-Index ausgewählt werden. Historisch genannte Datenports waren:

```text
color_image
depth_image
camera_info
```

Optional wurde eine Point Cloud genannt. Diese Ports sind AICA-Datenflussports, keine physischen Anschlüsse.

Altes Fehlerbild:

```text
HW not ready
```

Untersucht wurden udev-/Geräteberechtigungen und ein Zusammenhang mit `Auto Gain Limit Toggle`. Ein endgültiger Fix ist nicht dokumentiert.

### Kamerarollen

In den alten Chats wurden widersprüchliche Rollen beschrieben:

- L515 stationär für Groberkennung/Depth/Tracking und D435i am Roboter für Feinlokalisierung, oder
- D435i am Förderband und L515 für 3D-Position beziehungsweise Greifen.

Das heutige Repository unterscheidet `base_cam` und `robot_cam`, legt durch die Dateinamen allein aber nicht sicher die physischen Modelle fest. Die aktuelle Montage ist vor weiterer Planung zu prüfen.

### Aktueller Repository-Hinweis

Die Kamerakomponenten erwarten echte `CameraInfo`-Daten. `base_cam.py` besitzt alte Extrinsic-Standardwerte, die nach einer Neumontage neu kalibriert werden müssen. Für Roh-Depth im Format `16UC1` ist ein konfigurierbarer Tiefenskalierungsfaktor vorgesehen; D400- und L515-Daten können unterschiedliche Einheiten haben.

## Vision und Lokalisierung

### Historisch diskutierte klassische Pipeline

```text
Bildaufnahme
-> ROI, Entzerrung und Beleuchtungsnormalisierung
-> Farbsegmentierung / Threshold
-> Blur, Kanten und Morphologie
-> Connected Components oder Konturen
-> Merkmale und Tetromino-Klassifikation
-> Schwerpunkt und Orientierung
-> Tracking
```

Genannte Merkmale:

- Fläche und Umfang,
- Seitenverhältnis,
- Ecken,
- Hu-Momente,
- Schwerpunkt,
- Orientierung.

Als vereinfachtes Beispiel wurde ein I-Tetromino über ein Seitenverhältnis nahe `1:4` diskutiert. Das war kein vollständiger Klassifikator.

### YOLO und Hybridansatz

YOLO wurde als Alternative für Klassen und Bounding Boxes genannt. Eine Kombination aus YOLO und geometrischer Nachbearbeitung wurde erwogen. Eine produktive Implementierung ist nicht bestätigt.

### 3D-Lokalisierung und Kalibrierung

Der alte Zielpfad war:

```text
2D-Erkennung
-> Depth / Punktwolke
-> 3D-Position im Kameraframe
-> Kamera-zu-Roboter-/World-Transformation
-> Greifpose
```

Hand-Eye-/Extrinsic-Kalibrierung wurde als notwendig erkannt. Der ChatGPT-Export enthält keine finalen Transformationsmatrizen oder TF-Werte.

Im aktuellen Repository existieren bereits OpenCV-basierte Erkennungs-, Tracking- und Transformationsbausteine. Einige Parameter sind ausdrücklich als alte Rig-Werte dokumentiert und müssen bei verändertem Aufbau neu bestimmt werden.

### Bewegte Objekte

Für einen möglichen Förderbandbetrieb wurden Position, Orientierung, Geschwindigkeit, zeitliches Tracking und Greifzeitpunkt diskutiert. Eine vollständige Conveyor-Tracking-Lösung ist nicht als fertig bestätigt.

## Frühere konzeptionelle End-to-End-Zustandsfolge

Diese Folge war eine Strukturierung der Aufgaben, keine bestätigte State-Machine-Implementierung:

```text
WAIT_FOR_OBJECT
DETECT_OBJECT
CLASSIFY_TETROMINO
ESTIMATE_2D_POSE
GET_DEPTH / ESTIMATE_3D_POSE
TRANSFORM_TO_ROBOT_FRAME
PLAN_GRASP
MOVE_TO_PREGRASP
MOVE_TO_GRASP
CLOSE_GRIPPER
VERIFY_GRIP
MOVE_TO_PLACE
OPEN_GRIPPER
RETURN / NEXT_OBJECT
```

## Bekannte alte Fehlerbilder als Schnellreferenz

| Symptom | Damals diskutierte oder gefundene Ursachen | Status des alten Falls |
|---|---|---|
| UR10e bewegt sich nicht | keine Motion-Aktion, Controller inaktiv, External/Remote Control aus, Protective Stop, unerreichbare Pose, falsche IP/Route | keine allgemeingültige Einzelursache dokumentiert |
| `Destination Net Unreachable` | PC und Roboter nicht im selben Subnetz beziehungsweise keine IPv4 am Ethernet-Port | direkte Verbindung mit statischen IPs funktionierte später |
| Teach Pendant zeigt `0.0.0.0` | DHCP ohne DHCP-Server bei Direktverbindung | statische IP wurde gesetzt |
| AICA lädt Anwendung nicht vollständig | Roboternetz nicht erreichbar, Image/Tag, Komponentenstart oder Laufzeitkonfiguration | Netzwerk war ein plausibler Hauptfaktor |
| Lizenzstatus HTTP `404` | AICA-Lizenz-/Statusabfrage | nicht als Hauptursache bestätigt |
| URDF bereits geladen | erneute Robot-Description wurde ignoriert | Warnung, keine bestätigte Hauptursache |
| Docker meldet `pull access denied` oder Image fehlt | AICA-Referenz stimmt nicht mit lokalem Build-Tag überein | Image mit identischem `-t`-Tag bauen und referenzieren |
| Greifer verbindet nach AICA-Neustart nicht | serieller Port blieb im langlebigen Containerprozess offen | aktueller Code verbindet pro Operation und trennt im `finally` |
| L515: `HW not ready` | Gerätezustand, Berechtigung/udev, Auto-Gain-Funktion | endgültige damalige Lösung unbekannt |
| D435i-Bild horizontal verbreitert | Intrinsics, Auflösung, Alignment, Anzeige oder Skalierung | exakte Ursache unbekannt |
| Git `Everything up-to-date` trotz Änderungen | Dateien waren nicht staged/committed | erst Commit erzeugen, dann pushen |
| VS-Code-Commit nicht möglich | `user.name` und `user.email` fehlten | Git-Identität konfigurieren |

## Historische Diagnosebefehle

Diese Befehle sind eine Referenz; Namen und Umgebung vor Ausführung prüfen.

```bash
# AICA
aica up
aica status
aica controller list

# ROS 2 / ros2_control
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list
ros2 action list
ros2 action list | grep follow_joint_trajectory

# Realer UR10e / URSim RTDE-Port testen
nc -zv 192.168.96.221 30004
nc -zv 192.168.56.101 30004

# RealSense
rs-enumerate-devices -s
ros2 topic list | grep realsense

# Docker
docker ps
docker ps -a
docker images
docker logs -f <container>
```

## Git und Zusammenarbeit

- Historisches Remote: `git@github.com:Luze212/roboter_tetris.git`; der lokale Checkout enthielt später auch eine HTTPS-Remote auf dasselbe Repository.
- SSH auf dem Ubuntu-Rechner authentifizierte sich damals als `Luze212`.
- Die Repository-Zuordnung beweist nicht, wer das Projekt ursprünglich erstellt hat.
- Ein früheres VS-Code-Problem war eine fehlende lokale Git-Autoridentität.
- Teammitglieder arbeiten auf getrennten Branches.
- Für Hardwaretests wird am Roboter-Rechner der gewünschte Remote-Branch abgerufen und ausgecheckt.
- Vor dem Branchwechsel müssen lokale Änderungen committed oder anderweitig gesichert sein.
- `main` und andere Branches werden durch das Veröffentlichen eines neuen Branches nicht gelöscht.

## Zeitlicher Rückblick

### Mai 2026

- AICA-UR10e-Hardware-Interface untersucht.
- Historische Konfiguration mit UR10e `192.168.96.221`, Rate `500`, Core `v5.0.0` und Schema `2-0-6`.
- Drei kartesische Waypoints und ein `set_trajectory`-Payload erstellt beziehungsweise diskutiert.
- URSim unter `192.168.56.101` mit VNC/Weboberfläche genutzt.

### Juni 2026

- Greiferanbindung über Modbus Serial und USB-RS485 direkt am PC geklärt.
- D435i und L515 erkannt; D435i lieferte ein Bild in AICA.
- Zweite RealSense-Instanz und Vision-zu-3D-zu-Greifen-Pipeline diskutiert.
- Netzwerkprobleme untersucht und später direkte PC-UR-Verbindung mit statischen IPs verwendet.

### Juli 2026

- Alte `MoveToPose`-Komponente und kartesische Zielbewegungen diskutiert.

### September 2026

- Projektwissen aus ChatGPT und Repository zusammengeführt.
- Festgelegt, dass die Roboterbewegung auf Zielkoordinaten neu entwickelt wird und die alte `MoveToPose`-Idee nur Archivwissen ist.

Ein separates Planungsdokument enthielt Werktage von Ende April beziehungsweise Mai bis Anfang Oktober 2026. Die zugrunde liegende Tabelle und belastbare Meilensteine waren beim Import nicht verfügbar.

## Noch zu verifizierende Punkte

- Aktueller physischer Aufbau und Rollen von D435i und L515.
- Aktuelles AICA-Laufzeitsystem und Versionsstand.
- Aktive UR10e-Controller und verfügbare Bewegungsinterfaces.
- Dauerhafte Netzwerkprofile auf dem Roboter-PC.
- Status der L515 und Ursache beziehungsweise Lösung von `HW not ready`.
- Aktuelle Intrinsics, Extrinsics und TF-Beziehungen.
- TCP-/Tool-Frame, Payload und Greifergewicht.
- Aktuelle serielle Einstellungen des Greifers über die im Code sichtbaren Standardwerte hinaus.
- Welche Vision-Komponenten am realen System bereits validiert wurden.
- Ob das Förderband Teil des aktuellen Aufbaus ist und welche Trackinganforderungen gelten.
- Gewünschte Tetromino-Klassifikation und Ablagestrategie.
- Anforderungen und Abnahmekriterien für das Gesamtsystem.
- Für die neue Koordinatenfahrt: Eingabeformat, Koordinatenframe, Orientierung, Controller, Safety und Erfolgsrückmeldung.

## Früher diskutierte, nicht automatisch bindende Designideen

Folgende Punkte waren frühere Präferenzen oder Empfehlungen und müssen bei neuer Arbeit erneut bewertet werden:

- AICA als primäre Robotersteuerung statt einer direkten RTDE-Anwendungsarchitektur.
- Greifer weiterhin unabhängig vom UR-Controller über PC/USB-RS485.
- getrennte Konfigurationen für realen UR10e und URSim.
- modulare Trennung von Vision, Transformation, Robot Motion, Greifer und Orchestrierung.
- klassische Vision, YOLO oder eine hybride Erkennung.
- ZeroMQ zwischen Teilprozessen.
- separate Module für Kalibrierung, Tracking und Pick-and-Place-State-Machine.

Diese Liste dokumentiert frühere Überlegungen. Nur die ausdrückliche aktuelle Vorgabe zum vollständigen Neustart der Koordinatenfahrt ist für die weitere Arbeit gesetzt.
