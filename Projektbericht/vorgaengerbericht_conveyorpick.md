# Auswertung des Vorgängerberichts ConveyorPick

## Zweck und Umgang mit der Quelle

Diese Notiz hält Erkenntnisse aus dem Bericht der Vorgängergruppe fest. Die
PDF verbleibt außerhalb des Projektordners und wird nicht kopiert. Der Bericht
beschreibt den Stand vom 31.03.2026. Seine Architektur, Kalibrierwerte,
Parameter und Versuchsergebnisse sind historische Angaben. Sie gelten nicht
automatisch für das aktuelle Projekt.

Im Berichtstext sind Aussagen über den aktuellen Aufbau ausschließlich mit
aktuellen Projektunterlagen, Messdaten oder eigenen Versuchen zu belegen. Der
Vorgängerbericht dient für die Ausgangslage, die klare Abgrenzung und die
Diskussion der Weiterentwicklung.

Quelle: `2526ws_BH_Roboter_ConveyorPick_Doku-1.pdf`, 59 PDF-Seiten,
vollständig geprüft am 24.09.2026.

## Historischer Systemstand

- Der Aufbau verwendete bereits einen UR10e, einen Robotiq-Zweifingergreifer,
  ein Förderband, eine stationäre Tiefenkamera und eine am Endeffektor
  montierte Tiefenkamera.
- Die stationäre Kamera erkannte Objekte im frühen Bandbereich. Sie schätzte
  deren Lage, Geometrie, Orientierung und Bandgeschwindigkeit. Ein Tracker
  führte die Objekte mit ID weiter, auch nachdem sie das Kamerabild verlassen
  hatten.
- Die lokale Roboterkamera war für die Feinpositionierung und die Bestimmung
  des Auslösezeitpunkts vorgesehen.
- Die Bildverarbeitung lief in C++. Die Ablaufsteuerung lief in Python. Die
  Kommunikation erfolgte über ZeroMQ und RTDE. Diese Softwarearchitektur ist
  nicht die Architektur des aktuellen AICA- und ROS-2-Systems.
- Für die Kameratransformation wurde eine Hand-Eye-Kalibrierung mit
  ArUco-Board dokumentiert. Frühere Transformationswerte dürfen nicht in den
  aktuellen Bericht übernommen werden.

## Wesentliche Abgrenzung zum aktuellen Projekt

Der Vorgängerbericht beschreibt keinen kontinuierlich geregelten
Pick-on-the-Fly-Prozess. Der zuerst verfolgte Ansatz sollte das Objekt aktiv
einholen. Er wurde wegen Latenz, begrenzter Sensordistanz und fehlender
visueller Rückkopplung als nicht robust bewertet.

Als Ergebnis positionierte sich der Roboter anhand einer Vorhersage in einer
festen Abfangposition und wartete auf das ankommende Objekt. Die finale
Auslösung erfolgte zeitbasiert. Die Vorhersage nutzte eine konstante
Bandgeschwindigkeit und eine empirische Basiszeit von zwei Sekunden. Die
Roboterkamera sollte den Zeitpunkt nur kurz vor dem Greifen bestimmen.

Das aktuelle Projekt greift dagegen während der Bewegung. Der Regelpfad führt
eine laufend aktualisierte Zielpose aus. Diese Unterscheidung muss bereits in
der Einleitung und später in der Diskussion eindeutig bleiben.

## Dokumentierte Probleme und Lehren

| Beobachtung im Vorgängerbericht | Bedeutung für den aktuellen Bericht |
|---|---|
| Der verfolgungsbasierte Erstansatz war wegen Systemlatenz und begrenzter Sicht der Roboterkamera nicht robust. | Die aktuelle Regelung muss mit ihren eigenen Signalen, Aktualisierungsraten und Grenzen beschrieben werden. Der Fortschritt darf nicht nur als neue Vorhersage formuliert werden. |
| Bei hoher Bandgeschwindigkeit verschlechterte sich die Roboterkamera stark. Reflexionen lösten zu frühe Greifbefehle aus. Flache und schmale Teile waren besonders kritisch. | Die Optimierung der `base_cam` und die eingeschränkte Rolle von `robot_cam_2` sind fachlich begründet. Messungen zur heutigen Erkennung bleiben davon getrennt. |
| Die stationäre Kamera blieb in den dokumentierten Tests robust. | Die Basiskamera ist ein sinnvoller Schwerpunkt für den aktuellen Regelpfad. Die damaligen Quoten sind keine Vergleichswerte für die aktuelle Anlage. |
| Erfolgreiche Erkennung führte nicht zwingend zu einem erfolgreichen Griff. Besonders Timing und Greifgeometrie beeinflussten das Ergebnis. | In Kapitel 6 werden Erkennung, Positionsabweichung und Greiferfolg getrennt ausgewertet. |
| Variable Orientierung und höhere Geschwindigkeit erhöhten die Streuung. | Versuchsbedingungen müssen Klotzgeometrie, Orientierung, Bandgeschwindigkeit und Beleuchtung direkt neben den Ergebnissen nennen. |
| Die alte Zustandsmaschine arbeitete mit IDLE, MOVE, OBJECT WAITING, GRIP und PLACE. | Das Diagramm kann nicht übernommen werden. Für das aktuelle System wird ein eigenes Zustandsdiagramm des `object_follower` erstellt. |

## Wiederverwendbare inhaltliche Bausteine

### Für die Einleitung

Als Ausgangslage lässt sich knapp festhalten, dass ein visionbasiertes
Förderbandgreifen bereits vorhanden war. Es arbeitete mit einer
vorausberechneten Abfangposition und einer Wartephase. Die offene Aufgabe war
die zuverlässige Synchronisation der Roboterbewegung mit dem weiterhin
laufenden Objekt.

Die Begriffe "On the Fly" und "Greifen aus der Bewegung" werden nur für den
aktuellen Regelansatz verwendet. Die Vorgängergruppe kann nicht rückwirkend
als Pick-on-the-Fly-System bezeichnet werden.

### Für Systemaufbau und Konzept

Die historische Doppelkamera-Anordnung erklärt den Ursprung des heutigen
Aufbaus. Die aktuellen Komponenten, Koordinatensysteme und Datenverträge werden
jedoch nur anhand des gegenwärtigen Systems beschrieben. Insbesondere ersetzt
der frühere direkte RTDE- und ZeroMQ-Ablauf nicht den heutigen AICA-Regelpfad.

Die damalige Track-Idee bleibt als technische Vorarbeit relevant: stabile IDs,
Geschwindigkeit aus zeitlich getrennten Detektionen, Prädiktion bei
Sichtverlust und Bestätigung neuer Kandidaten. Im Bericht darf dies nur dort
erwähnt werden, wo die aktuelle Implementierung diese Idee tatsächlich
übernimmt.

### Für Versuche und Diskussion

Der Vorgängerbericht liefert eine Begründung für getrennte Qualitätsmaße:
Erkennungsquote, Positionsqualität und Greiferfolg. Für den aktuellen Bericht
werden passende Metriken mit eigenen Versuchsreihen definiert. Die historischen
Prozentwerte, Geschwindigkeitsstufen und Objektgruppen werden nicht mit
heutigen Ergebnissen vermischt.

Eine mögliche Diskussionsaussage ist: Die frühere Lösung reduzierte die
Dynamik durch Warten am Abgriffspunkt. Das aktuelle Konzept verschiebt die
Anforderung auf einen geschlossenen, laufend aktualisierten Regelpfad. Dadurch
werden Verzögerungen und Fehler der Basiserkennung direkt greifrelevant.

## Angaben, die nicht übernommen werden dürfen

- Kalibrierwerte, Arbeitsraumgrenzen, TCP- oder Kameraposen.
- C++-, ZeroMQ-, RTDE- und Python-Dateinamen der Vorgängergruppe.
- Die pauschale Bewertung von ROS 2 als zu aufwendig. Der aktuelle Aufbau
  verwendet ROS 2.
- Die damaligen Versuchsquoten und die nicht metrisch definierte
  Geschwindigkeitsstufe 1 oder 3.
- Aussagen zur Funktion, Genauigkeit oder Reichweite von Kamera und Roboter,
  sofern sie nicht mit primären Quellen oder aktuellen Messungen bestätigt
  sind.
- Alte Greifstrategien, sofern sie nicht bewusst als verworfene Alternative
  eingeordnet werden.

## Geeignete Referenzstellen

| Thema | PDF-Seiten | Geplanter Einsatz |
|---|---:|---|
| Historischer Systemaufbau | 9 bis 10 | Einleitung, knappe Ausgangslage |
| Aufgegebener Folgeansatz und Warteposition | 15 bis 19 | Einleitung und Diskussion |
| Vorhersagemodell und Zustandsdiagramm | 22 bis 24 | Nur zur Abgrenzung, nicht übernehmen |
| Stationäre Kamera, Tracking, Roboterkamera | 25 bis 35 | Hintergrund für Entwicklungslinien |
| Reflexionen, flache Geometrien und hohe Geschwindigkeit | 45 bis 55 | Diskussion der heutigen Grenzen |
| Ausblick auf dynamisches Greifen | 56 bis 57 | Motivation und Einordnung des aktuellen Projekts |

## Prüfregel vor jeder Verwendung

Vor einer Aussage aus dieser Notiz wird geprüft:

1. Beschreibt sie den historischen oder den aktuellen Stand?
2. Existiert für den aktuellen Stand ein aktueller Beleg?
3. Ist die Abgrenzung zum wartenden Abfangprozess für Lesende eindeutig?
4. Werden historische Werte nicht als eigene Messergebnisse ausgegeben?
