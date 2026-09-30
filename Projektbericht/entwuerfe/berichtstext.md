# 1 Einleitung

## 1.1 Ausgangslage und Motivation

Das Projekt erweitert ein bestehendes System zum Greifen und Sortieren von
Klötzen auf einem bewegten Förderband. Ein duales Kamerasystem ermittelt die
Positionen und Geschwindigkeiten der ankommenden Objekte. Das ursprüngliche
System berechnete daraus einen festen Abgriffpunkt. Der Roboter fuhr diese
Position an und wartete dort auf das Objekt.

Diese Prozessführung begrenzt die Zahl der Klötze, die der Roboter verarbeiten
kann. Besonders bei kurzen Abständen zwischen den Klötzen bleibt wenig Zeit
für die nächste Positionierung. Der Roboter soll deshalb nicht an einem festen
Punkt warten. Er soll sich mit dem ausgewählten Klotz mitbewegen und ihn aus
der Bewegung heraus greifen.

Damit wird die Handhabung flexibler. Das System soll mehrere gleiche oder
unterschiedliche Klötze gleichzeitig unterscheiden und nacheinander
bearbeiten. Es muss selbst entscheiden, welcher Klotz innerhalb des
verbleibenden Arbeitsbereichs noch sicher erreichbar ist. Nicht mehr
erreichbare Klötze bleiben auf dem Förderband. Die Positions- und
Geschwindigkeitsschätzung erfolgt dabei ausschließlich auf Basis der Bilddaten
([src-projekt-inversetetris-aufgabenstellung](../referenzen/quellen/src-projekt-inversetetris-aufgabenstellung.md)).

Die Aufgabenstellung sieht außerdem den grundsätzlichen Funktionsnachweis bei
unterschiedlichen Bandgeschwindigkeiten vor. Daraus ergeben sich hohe
Anforderungen an die zeitliche Abstimmung von Erkennung, Zielauswahl und
Roboterbewegung. Der Bericht untersucht, wie diese Anforderungen im aktuellen
Versuchsaufbau umgesetzt und bewertet werden.

## 1.2 Zielsetzung des Projekts

Ziel des Projekts ist ein Konzeptnachweis für das kamerabasierte Greifen von
quaderförmigen Klötzen auf einem bewegten Förderband. Die Positions- und
Geschwindigkeitsschätzung soll ausschließlich aus Bilddaten entstehen. Ein
Encoder oder ein vergleichbares Signal des Förderbands ist dafür nicht
vorausgesetzt. Der Roboter soll die Klötze während ihrer Bewegung greifen
([src-projekt-inversetetris-aufgabenstellung](../referenzen/quellen/src-projekt-inversetetris-aufgabenstellung.md)).

Neben dem Greifvorgang soll das System die Klötze nach Form und Farbe
kategorisieren. Die Kategorisierung dient der flexiblen Objektbeschreibung,
nicht einer Ablageaufgabe. Sie soll die Übertragbarkeit des Konzepts auf
weitere vergleichbare Anwendungsfälle ermöglichen. Befinden sich mehrere
Klötze gleichzeitig auf dem Förderband, soll das System selbstständig einen
noch erreichbaren Klotz auswählen. Klötze, die innerhalb des verbleibenden
Arbeitsbereichs nicht sicher erreichbar sind, werden nicht mehr als Ziel
verfolgt.

Der Funktionsnachweis steht im Mittelpunkt. Ein fertiges Industrieprodukt und
ein unter allen Bedingungen zuverlässig arbeitendes System sind nicht Ziel des
Projekts. Die Anlage wird dennoch bei unterschiedlichen Bandgeschwindigkeiten
getestet. Damit wird untersucht, wie sich die Bandgeschwindigkeit auf das
Greifen auswirkt.

Ein Förderband mit variabler Geschwindigkeit war für den Aufbau vorgesehen,
wurde jedoch innerhalb des Projektrahmens nicht in die Anlage integriert. Die
Bewertung erfolgt deshalb mit dem vorhandenen Förderband. Dieses ermöglicht
auch höhere Bandgeschwindigkeiten. Reflexionen und die daraus entstehenden
Störungen der Bildverarbeitung werden dabei als relevante Randbedingung der
Versuche betrachtet.

<!-- Merker für Kapitel 4 und 6: Die automatische Auswahl erreichbarer Klötze
ist im aktuellen Stand umgesetzt. Umsetzung und Funktionsnachweis mit eigenen
Messdaten beschreiben. -->

<!-- Merker für Kapitel 6 und 7: Mit dem schnelleren vorhandenen Förderband
sind die Projektziele trotz höherer Geschwindigkeit und Reflexionen erreichbar.
Die Aussage nur mit dokumentierten Versuchsbedingungen und Ergebnissen belegen. -->

## 1.3 Abgrenzung und Rahmenbedingungen

Die Projektarbeit ist auf einen Konzeptnachweis im vorhandenen Versuchsaufbau
begrenzt. Sie umfasst das Erkennen, Auswählen und Greifen quaderförmiger
Klötze auf dem bewegten Förderband. Zum Funktionsnachweis gehören das Anheben
eines erfolgreich gegriffenen Klotzes und seine sichere Ablage an einem
vorgegebenen Abgabeort.

Die Form- und Farbkategorisierung unterstützt die Beschreibung und Auswahl der
Klötze. Eine vollständige Sortier- oder Ablageanlage gehört nicht zum
Projektumfang. Ebenso wird kein fehlerfreies Greifen aller Klötze gefordert.
Das System darf Klötze verwerfen, wenn diese im verbleibenden Arbeitsbereich
nicht sicher erreichbar sind ([src-projekt-inversetetris-aufgabenstellung](../referenzen/quellen/src-projekt-inversetetris-aufgabenstellung.md)).

Das neue Förderband mit variabler Geschwindigkeit wurde innerhalb des
Projektrahmens nicht in die Anlage integriert. Die Versuche erfolgen deshalb
mit dem vorhandenen Förderband bei ausgewählten Geschwindigkeiten. Die
Ergebnisse gelten für den bestehenden Arbeitsraum sowie die vorhandenen
Lichtverhältnisse und Reflexionen. Das Projekt zielt nicht auf den Nachweis
einer vollständigen industriellen Anwendung.

<!-- Merker für 2.1.3: Die Roboterkamera gehört zum Systemaufbau. Sie darf als
optionales Korrektursignal verwendet werden, ist aber nicht Voraussetzung für
den erfolgreichen Greifablauf. Details zu ihrem derzeitigen Einsatz und ihren
Grenzen später beschreiben. -->

## 1.4 Aufbau des Berichts

Zunächst werden die technischen Voraussetzungen des Versuchsaufbaus erläutert.
Dazu gehören die eingesetzte Hardware, die Koordinatensysteme und der
Softwareaufbau. Darauf aufbauend beschreibt der Bericht das Konzept, mit dem
die Bilddaten in eine dynamische Greifbewegung überführt werden.

Anschließend folgt die Umsetzung der einzelnen Komponenten. Die Inbetriebnahme
zeigt, welche Parameter und Anpassungen für das Zusammenspiel von Kamera,
Regelung und Roboter erforderlich waren. Die Versuchsergebnisse bewerten den
Funktionsnachweis unter den definierten Bedingungen. Abschließend ordnet der
Bericht die erreichten Ergebnisse ein und leitet weitere technische Schritte
ab.

## 2 System und Versuchsaufbau

### 2.1 Hardwareaufbau

#### 2.1.1 Allgemeine Geometrie und Aufbau

Der Versuchsaufbau besteht aus einem Förderband, dem seitlich angeordneten
UR10e mit Greifer, einer Base-Kamera und einer Ablagekiste. In Bandlaufrichtung
steht der Roboter auf der linken Seite nahe dem Bandende. Die Ablagekiste liegt
links neben dem Förderband und in Bandlaufrichtung vor dem Roboter. Ihre
Ablageposition beträgt im globalen Bezugssystem `world`
`x = -0,316 m`, `y = 0,476 m` und `z = 0,420 m`.

Als Base-Kamera wird eine Intel RealSense L515 eingesetzt. Sie kombiniert eine
RGB-Kamera mit einem LiDAR-Tiefensensor. Die Tiefenmessung basiert auf einem
abtastenden Infrarotlaser. Damit stehen Farbinformationen zur Kategorisierung
und Tiefeninformationen zur räumlichen Einordnung der Klötze zur Verfügung
[src-intel-realsense-l515-spezifikation](../referenzen/quellen/src-intel-realsense-l515-spezifikation.md),
[src-realsense-l515-datenblatt](../referenzen/quellen/src-realsense-l515-datenblatt.md).
Die Kamera ist mittig über dem Bandanfang montiert. Ihr Abstand zur
Bandebene beträgt etwa `0,85 m`. Sie erfasst die Klötze vor dem Greifbereich.

Zusätzlich ist eine Intel RealSense D435i am Roboterflansch neben der Aufnahme
für den Greifer montiert. Ihre optische Achse ist gegenüber der Flanschmitte
parallel versetzt und erfasst den Bereich unmittelbar vor dem Greifer. Die
Kamera besitzt neben einem RGB-Sensor ein aktives Infrarot-Stereosystem zur
Tiefenmessung sowie eine integrierte inertiale Messeinheit. Sie kann die
Basiskamera bei der Feinortung ergänzen, ist für den nachgewiesenen Greifablauf
jedoch nicht erforderlich
[src-intel-realsense-d435i-spezifikation](../referenzen/quellen/src-intel-realsense-d435i-spezifikation.md),
[src-realsense-d400-datenblatt](../referenzen/quellen/src-realsense-d400-datenblatt.md).

Für die anfängliche Kalibrierung wurde das Bezugssystem `conveyor_frame`
definiert. Sein Ursprung liegt mittig am Bandanfang. Die X-Achse zeigt, in
Bandlaufrichtung betrachtet, nach rechts. Die Y-Achse zeigt in
Bandlaufrichtung, die Z-Achse von der Bandebene nach oben. Im Rahmen der
Anwendung werden ausschließlich die globalen Koordinaten des Bezugssystems
`world` verwendet. Dieses ist das von der UR10e-Steuerung verwendete System,
dessen Ursprung in der Roboterbasis liegt. Alle folgenden Koordinaten beziehen
sich auf dieses System. Die unterschiedlichen Koordinatenkonventionen von
`conveyor_frame` und `world` ergeben sich aus der zeitlichen Zusammenführung
der Teilprojekte.

Die nutzbare gerade Förderstrecke beträgt ungefähr `1,50 m` bei einer Breite
von etwa `0,80 m`. Das Förderband ist physisch etwas länger. Die gekrümmten
Bereiche über den Bandrollen stehen jedoch nicht als Greifbereich zur
Verfügung. Die Bandkanten liegen im globalen System ungefähr bei
`x = -1,275 m` und `x = -0,480 m`. Der für das Greifen nutzbare rechteckige
Bereich reicht von `x = -1,200 m` bis `-0,530 m` sowie von
`y = -0,320 m` bis `0,430 m`. Er wird in der Draufsicht als Arbeitsbereich
dargestellt. Der festgelegte Arbeitsraum des Flansches erweitert diesen
Bereich für die Ablage der Objekte auf `x = -1,000 m` bis `-0,300 m` und
`y = -0,320 m` bis `0,480 m`.

Die räumliche Anordnung des realen Aufbaus zeigen die Abbildungen
`fig-aufbau-gesamtansicht-1` und `fig-aufbau-gesamtansicht-2`.
Die Lage von Förderband, Kamera, Roboter, Ablagekiste und Arbeitsbereich wird
in Abbildung `fig-systemaufbau-draufsicht` schematisch verdeutlicht.

Die festgelegten zulässigen Höhen im Arbeitsbereich des Flansches betragen `z = 0,309 m` , womit sich der geschlossene Greifer knapp über der Oberfläche des Förderbandes befindet, und
`z = 0,600 m`. Die Bewegung, mit der den fahrenden Objekten auf dem Fließband gefolgt wird, erfolgt auf `z = 0,450 m`, für den Transfer
zur Ablage wird eine Freihöhe von `z = 0,490 m` verwendet. Die untere
Z-Grenze hält die geschlossene Backenspitze mit einem Abstand von `10 mm` über
der Bandebene. Oberhalb von `z = 0,600 m` wurde im hinteren Bereich des
Arbeitsraums eine mögliche Singularität beobachtet zudem wird die Höhe durch die ber dem Förderband befindliche Base-Kamera limitiert.

Die zugehörigen Höhen und die Position der Basiskamera über dem Förderband sind
in Abbildung `fig-systemaufbau-seitenansicht` dargestellt.

<!-- Word-Übernahme Skizzen: `fig-systemaufbau-draufsicht` und
`fig-systemaufbau-seitenansicht` nach den jeweiligen Textverweisen einfügen.
Die Beschriftungen beruhen auf der Konfiguration vom 24.09.2026. -->

<!-- Word-Übernahme Gesamtaufnahmen: `fig-aufbau-gesamtansicht-1` und
`fig-aufbau-gesamtansicht-2` unmittelbar nach dem Textverweis einfügen. -->

#### 2.1.2 UR10e

Die zentrale Handhabungseinheit des Aufbaus ist ein UR10e von Universal Robots.
Der Roboter besitzt sechs rotierende Gelenke. Seine Reichweite beträgt
`1.300 mm`, die maximale Nutzlast `12,5 kg`
([src-universalrobots-ur10e-technische-daten](../referenzen/quellen/src-universalrobots-ur10e-technische-daten.md)).
Die Wiederholgenauigkeit ist mit `±0,05 mm` angegeben. Sie beschreibt die
Wiederholbarkeit des Roboterarms und nicht die absolute Genauigkeit des
gesamten kamerabasierten Greifprozesses. Der UR10e trägt den Greifer und
positioniert ihn über dem Förderband sowie am vorgegebenen Abgabeort.

Der UR10e ist für kollaborative Anwendungen ausgelegt. Seine
Sicherheitsfunktionen können unter anderem Grenzen für Geschwindigkeit, Kraft,
Impuls und Leistung überwachen. Wird eine konfigurierte Grenze überschritten
oder ein Sicherheitsfehler erkannt, wird die Roboterbewegung sicher
unterbrochen
([src-universalrobots-ur10e-sicherheitsfunktionen](../referenzen/quellen/src-universalrobots-ur10e-sicherheitsfunktionen.md)).
Ob der Aufbau ohne trennende
Schutzeinrichtung betrieben werden darf, ergibt sich jedoch erst aus der
Risikobeurteilung des gesamten Systems. Dabei müssen insbesondere Greifer,
Klötze, Bewegungen und Quetschstellen berücksichtigt werden. Der Endeffektor
ist nicht automatisch durch die Sicherheitsfunktionen des Roboterarms
abgedeckt
([src-universalrobots-ur10e-risikobeurteilung](../referenzen/quellen/src-universalrobots-ur10e-risikobeurteilung.md)).

#### 2.1.3 Robotiq-2F-140-Greifer

Am Flansch des UR10e ist ein adaptiver Zweifingergreifer vom Typ
Robotiq 2F-140 montiert. Die Herstellerabmessungen des geöffneten Greifers
zeigt Abbildung `fig-greifer-robotiq-2f140-abmessungen`
([src-robotiq-2f140-spezifikation](../referenzen/quellen/src-robotiq-2f140-spezifikation.md)).

<!-- Word-Übernahme: `fig-greifer-robotiq-2f140-abmessungen` an dieser Stelle
einfügen und die nachfolgende Beschriftung übernehmen. -->
![Abmessungen des geöffneten Robotiq-2F-140-Greifers](../abbildungen/robotiq_2f140_abmessungen_geoeffnet.png)

*Abbildung `fig-greifer-robotiq-2f140-abmessungen`: Herstellerabmessungen des
geöffneten Robotiq-2F-140-Greifers (Quelle:
`src-robotiq-2f140-spezifikation`).*

Jeder Finger besteht aus zwei starren Abschnitten. Diese Abschnitte werden
Phalangen genannt und sind über ein Gelenk verbunden. Ein einziger Antrieb
bewegt beide Finger. Der Greifer besitzt damit weniger Antriebe als Gelenke.
Beim Schließen drehen sich die Phalangen um ihre Gelenke. Die Greifflächen
folgen deshalb einer gekrümmten Bahn und nicht einer geraden parallelen
Bewegung. Abhängig von Geometrie und Lage des Klotzes entsteht ein paralleler
oder umschließender Griff
([src-robotiq-2f140-handbuch](../referenzen/quellen/src-robotiq-2f140-handbuch.md)).

Ein Klotz kann deshalb an seinen Seiten nicht beliebig tief gegriffen werden.
Bei einem zu tiefen seitlichen Eingriff würden die Phalangen beim Schließen mit
der Oberseite des Klotzes kollidieren. Die Greiftiefe ist somit durch die
Fingergeometrie begrenzt. Außerdem verändert sich die räumliche Höhe des
Greifers zwischen offenem und geschlossenem Zustand. Der zulässige Arbeitsraum
und die Greifhöhe müssen diese Zustandsänderung berücksichtigen.

Schließgeschwindigkeit und Schließkraft sind einstellbar. Der Kraftwert
begrenzt den maximalen Motorstrom und damit die beim Schließen verfügbare
Greifkraft. Treffen die Finger auf einen Widerstand und wird diese Grenze
erreicht, hält der Greifer an. Sein Gerätestatus meldet den Kontakt als
erkannte Objektaufnahme. Bei geeigneter Kraftkonfiguration kann die integrierte
Nachgreiffunktion einen späteren Objektverlust erkennen und die Finger weiter
schließen
([src-robotiq-2f140-handbuch](../referenzen/quellen/src-robotiq-2f140-handbuch.md)).

Die tatsächlich nutzbare Öffnungsweite wurde am aufgebauten System mit
`127 mm` gemessen. An den Fingerendgliedern sind 3D-gedruckte Aufsätze mit
Gummi-Grippmatten montiert. Die Greiffläche jeder Matte beträgt
`20 mm × 15 mm`.

Vor dem Betrieb wird der Öffnungsbereich des Greifers kalibriert. Dadurch ist
die Öffnungsweite im aufgebauten System als Millimeterwert verfügbar. Der
Greifer ist über USB mit dem Rechner verbunden. Er wird damit extern
angesteuert und nicht über die direkte Roboteransteuerung bedient.
