# Übergabevorgabe: Systemskizzen für den Projektbericht

## Zweck dieser Datei

Diese Datei ist die verbindliche Arbeitsvorgabe für eine spätere KI oder eine Person, die zwei sachliche Systemskizzen für den Projektbericht erstellt. Die Skizzen werden in Abschnitt **2.1.1 Allgemeine Geometrie und Aufbau** verwendet. Sie sollen den Versuchsaufbau verständlich einordnen. Sie sind keine Konstruktionszeichnung und keine Visualisierung der Bewegungsplanung.

Es werden genau zwei getrennte Abbildungen benötigt:

1. Draufsicht auf Förderband, Roboter, Ablagekiste, Basiskamera und greifbaren Arbeitsbereich.
2. Seitenansicht mit Förderband, Basiskamera und den relevanten Höhen im `world`-System.

Alle Zahlen in dieser Vorgabe stammen aus dem aktuellen Projektstand oder aus den abgestimmten Angaben der Projektgruppe. Zahlen, Richtungen und die relativen Positionen dürfen nicht aus Gründen der Optik gespiegelt, vertauscht oder vereinfacht werden.

## Abgrenzung und vorhandene Arbeitsstände

Im Ordner liegen bereits mehrere Entwürfe. Sie sind ausschließlich Anschauungsmaterial und **nicht** als fachlich oder gestalterisch freigegeben zu behandeln:

- `systemaufbau_draufsicht_vorlaeufig.svg` und `systemaufbau_seitenansicht_vorlaeufig.svg`
- `systemaufbau_draufsicht.png` und `systemaufbau_seitenansicht.png`
- `systemaufbau_draufsicht_ppt.png` und `systemaufbau_seitenansicht_ppt.png`
- `systemskizzen_layout_v02.pptx`

Die bisherigen Entwürfe zeigen insbesondere, welche Textüberlappungen zu vermeiden sind. Sie dürfen nicht überschrieben oder gelöscht werden. Eine neue Fassung erhält zunächst neue Dateinamen.

Zur Einordnung des realen Aufbaus liegen außerdem vier Fotos in diesem Ordner. Sie dienen nur als Referenz für die räumliche Anordnung und werden nicht als Bildbestandteile nachgezeichnet:

- `WhatsApp Image 2026-09-24 at 19.31.05.jpeg`
- `WhatsApp Image 2026-09-24 at 19.31.05 (1).jpeg`
- `WhatsApp Image 2026-09-24 at 19.31.05 (2).jpeg`
- `WhatsApp Image 2026-09-24 at 19.31.05 (3).jpeg`

## Gewünschte Gestaltung

Die Abbildungen sollen wie reduzierte technische Lehrbuchgrafiken wirken. Sie müssen nüchtern, gut lesbar und auch in einem gedruckten Word-Bericht verständlich sein.

- Weißer Hintergrund.
- Wenige Farben: dunkles Grau oder Schwarz für Konturen und Beschriftungen, ein zurückhaltendes Blau für Koordinatenachsen, Bewegungsrichtungen und den Arbeitsbereich. Sehr helles Grau darf für Förderband, Roboter- und Kistensymbol verwendet werden.
- Keine fotorealistische Darstellung, keine gerenderte 3D-Szene, keine Schlagschatten, keine Verläufe, keine dekorativen Rahmen, keine Logos, keine Wasserzeichen und keine Personen.
- Keine detaillierte Roboterkinematik. In der Draufsicht genügt ein sehr einfaches, eindeutiges Symbol für die UR10e-Basis beziehungsweise den Roboterstandort.
- Keine groß gesetzte Abbildungsüberschrift innerhalb der Grafik. Die endgültige Abbildungsbeschriftung wird später in Word gesetzt.
- Deutsche Beschriftungen, SI-Einheiten und deutsches Dezimaltrennzeichen. Beispiel: `z = 0,490 m`.
- Eine sachliche Standardschrift wie Arial oder Calibri verwenden. Sie muss nach dem späteren Einfügen in Word noch gut lesbar sein.
- Vektorgrafik oder bearbeitbare Präsentationsformen sind bevorzugt. Zusätzlich eine PNG-Vorschau in hoher Auflösung ausgeben.

### Strenge Regeln für Beschriftungen

Die Lesbarkeit hat Vorrang vor einer möglichst kompakten Darstellung.

- Jeder Text steht in reserviertem Weißraum oder in einer klaren Beschriftungsspalte.
- Kein Text darf eine Kontur, Maßlinie, Achse, Pfeilspitze, gestrichelte Begrenzung oder andere Beschriftung überdecken.
- Führungslinien dürfen keinen Text schneiden und sich nicht gegenseitig kreuzen.
- Wenn eine Führungslinie nötig ist, nur kurze horizontale oder vertikale Abschnitte verwenden. Schräg verlaufende Führungslinien möglichst vermeiden.
- Bei dicht beieinanderliegenden Höhenwerten dürfen die Beschriftungen in einer gestaffelten Legende stehen. Ihre Führungslinien müssen dann eindeutig und kreuzungsfrei bleiben.
- Vor der Übergabe beide Abbildungen bei 100 % Zoom prüfen. Eine Beschriftung, die nur knapp nicht kollidiert, ist zu weit nach außen zu ziehen.

## Bezugssysteme und Richtungen

### Kalibrierungsbezugssystem `conveyor_frame`

Für die ursprüngliche Kalibrierung wurde ein eigenes Förderbandbezugssystem festgelegt:

- Ursprung: mittig am Bandanfang.
- `X+`: nach rechts, wenn in Bandlaufrichtung geschaut wird.
- `Y+`: in Bandlaufrichtung.
- `Z+`: von der Förderbandebene nach oben.

Dieses System soll in der Draufsicht direkt am Bandanfang mit kompakten `X+`- und `Y+`-Achsen eingezeichnet werden. Die Beschriftung lautet `conveyor_frame`.

### Laufzeitbezugssystem `world`

In der späteren Anwendung wird ausschließlich das globale `world`-System verwendet. Im Bericht ist es ausdrücklich als globales, roboterbasisbezogenes räumliches Bezugssystem definiert. Objektpositionen, Zielpositionen und Arbeitsraumgrenzen werden in diesem System angegeben.

Für die Skizzen gilt:

- Das `world`-System wird in der Draufsicht an der Roboterbasis dargestellt.
- Die sichtbare Bezeichnung lautet ausschließlich `world`. Die Bezeichnung `robot_base` darf in den Abbildungen nicht erscheinen.
- `world-Y+` läuft gegen die Bandlaufrichtung.
- `world-X+` ist gegenüber der `conveyor_frame`-X-Richtung umgekehrt.
- Das Förderband bewegt sich folglich in Richtung `world-Y−`.
- In der Seitenansicht zeigt `world-Z+` nach oben.

Die geregelte Pose bezieht sich im Projekt auf den Roboterflansch `ur_tool0`, nicht auf einen frei gewählten TCP. Dieser Hinweis ist in der Seitenansicht als Fußnote relevant.

## Verbindliche Datenbasis

Alle folgenden Werte sind in Meter angegeben und auf drei Nachkommastellen gerundet.

| Größe | Wert | Verwendung in der Skizze |
|---|---:|---|
| Nutzbare gerade Bandlänge | 1,507 m | Draufsicht, Maßangabe entlang des geraden Bandabschnitts |
| Bandbreite | 0,795 m | Draufsicht, Maßangabe quer zum Band |
| Bandanfang, mittig | `y = 1,140 m` | Orientierung im `world`-System |
| Bandende, Mitte der Umlenkrolle | `y = -0,367 m` | Orientierung im `world`-System |
| Rechte Bandseite im `world`-System | `x = -1,275 m` | Draufsicht |
| Linke Bandseite im `world`-System | `x = -0,480 m` | Draufsicht |
| Greifbarer Bereich, untere X-Grenze | `x = -1,000 m` | Draufsicht, gestricheltes Rechteck |
| Greifbarer Bereich, obere X-Grenze | `x = -0,530 m` | Draufsicht, gestricheltes Rechteck |
| Greifbarer Bereich, untere Y-Grenze | `y = -0,320 m` | Draufsicht, gestricheltes Rechteck |
| Greifbarer Bereich, obere Y-Grenze | `y = 0,445 m` | Draufsicht, gestricheltes Rechteck |
| Ablageposition, X | `x = -0,316 m` | Draufsicht, Kistenmittelpunkt |
| Ablageposition, Y | `y = 0,476 m` | Draufsicht, Kistenmittelpunkt |
| Ablageposition, Z | `z = 0,420 m` | Draufsicht als Teil der Ablagepositionsangabe |
| Untere Arbeitsraumgrenze | `z = 0,299 m` | Seitenansicht |
| Folge- und Beobachtungshöhe | `z = 0,450 m` | Seitenansicht |
| Transferhöhe | `z = 0,490 m` | Seitenansicht |
| Obere Arbeitsraumgrenze | `z = 0,600 m` | Seitenansicht |
| Höhe der Basiskamera über dem Band | ca. `0,850 m` | Seitenansicht |

Die reale Förderbandlänge ist größer als die nutzbare gerade Bandlänge. Die gekrümmten Bereiche über den Bandrollen gelten jedoch nicht als nutzbarer Greifbereich. Diese Information soll in einer kleinen Anmerkung oder durch die reduzierte Darstellung der Bandrollen sichtbar werden, ohne die Zeichnung zu überladen.

## Skizze 1: Draufsicht des Systemaufbaus

### Inhalt und räumliche Anordnung

Die Draufsicht zeigt den Aufbau mit dem Band ungefähr vertikal auf der Seite. Der Bandanfang liegt oben, das Bandende unten. Die Bandlaufrichtung verläuft damit von oben nach unten.

Einzuzeichnen sind:

1. Das Förderband als schlichtes, langes Rechteck. Die beiden Endrollen können sehr vereinfacht angedeutet werden.
2. Ein klarer Pfeil für die Bandlaufrichtung von Bandanfang zu Bandende.
3. Die Basiskamera mittig oberhalb des Bandanfangs. In der Draufsicht genügt ein kleines Kamerasymbol oder ein beschrifteter Kreis. Sie ist vertikal nach unten gerichtet. Kein Sichtkegel erforderlich.
4. Der Roboter auf der **linken Seite des Bandes**, wenn in Bandlaufrichtung geschaut wird. In der gewählten Seitenanordnung liegt er daher links vom Band und nahe dessen Ende.
5. Die Ablagekiste ebenfalls links neben dem Band, in Bandlaufrichtung vor dem Roboter. Ihre Mitte entspricht der Ablageposition `(-0,316 m; 0,476 m; 0,420 m)` im `world`-System.
6. Der greifbare Arbeitsbereich als gestricheltes Rechteck innerhalb des Bandes. Seine Lage muss über die oben angegebenen `world`-Grenzen korrekt zwischen Bandanfang und Bandende positioniert werden.
7. Das `conveyor_frame` am Bandanfang.
8. Das `world`-System an der Roboterbasis.

Der Roboter und die Kiste müssen auf derselben realen Bandseite liegen. Frühere automatisch erzeugte Entwürfe spiegelten diese Lage irrtümlich. Dieser Fehler darf nicht wiederholt werden.

### Erforderliche Beschriftungen

Folgende Texte müssen lesbar vorhanden sein. Die genaue Platzierung darf angepasst werden, wenn die Kollisionsfreiheit erhalten bleibt.

- `Förderband`
- `Bandlaufrichtung`
- `Basiskamera`
- `UR10e`
- `Ablagekiste`
- `Ablageposition: x = -0,316 m; y = 0,476 m; z = 0,420 m`
- `Greifbarer Arbeitsbereich`
- `conveyor_frame`
- `world`
- `Nutzbare gerade Bandlänge: 1,507 m`
- `Bandbreite: 0,795 m`

Es ist zulässig, die X- und Y-Grenzen des Arbeitsbereichs in einer kleinen Legende zu bündeln, zum Beispiel:

`x = -1,000 bis -0,530 m`  
`y = -0,320 bis 0,445 m`

Die gestrichelte Fläche darf sehr hell blau hinterlegt sein. Sie darf jedoch nicht als `ROI` bezeichnet werden.

### Achsen und Koordinatenlogik

- Am Bandanfang: kurze blaue Achsen für `conveyor_frame`, mit `X+` nach rechts und `Y+` in Bandlaufrichtung.
- An der Roboterbasis: kurze blaue Achsen für `world`, mit `Y+` gegen die Bandlaufrichtung und der gegenüberliegenden X-Orientierung.
- Eine zusätzliche Z-Achse ist in der Draufsicht nicht nötig. Falls erforderlich, genügt die Fußnote `Z+ zeigt von der Förderbandebene nach oben.`
- Eine mathematisch exakte Millimeter-Skalierung ist nicht erforderlich. Die räumlichen Beziehungen, insbesondere die Seitenlage von Roboter und Kiste sowie die Arbeitsbereichslage, müssen aber den Koordinaten entsprechen.

## Skizze 2: Seitenansicht mit Höhenbezug

### Inhalt und räumliche Anordnung

Die Seitenansicht soll bewusst noch einfacher als die Draufsicht ausfallen. Sie beschreibt nur die vertikale Geometrie.

Einzuzeichnen sind:

1. Das Förderband als waagerechter, einfacher Querschnitt mit klarer Förderbandebene.
2. Die Basiskamera direkt über dem Bandanfang auf einer stark vereinfachten Halterung.
3. Ein vertikaler Maßpfeil oder eine Maßlinie zwischen Förderbandebene und Kamera mit `ca. 0,850 m`.
4. Eine blaue `world-Z+`-Achse nach oben.
5. Vier horizontale Höhenniveaus oder kurze Bezugslinien für Arbeitsraumgrenzen und Prozesshöhen.

Der Roboter, der Greifer, die Kiste und einzelne Klötze werden in dieser Ansicht **nicht** dargestellt. So bleibt die Abbildung frei von unnötiger Komplexität.

### Verbindliche Höhenbeschriftungen

Alle folgenden Angaben müssen deutlich lesbar erscheinen:

- `obere Arbeitsraumgrenze: z = 0,600 m`
- `Transferhöhe: z = 0,490 m`
- `Folge- und Beobachtungshöhe: z = 0,450 m`
- `untere Arbeitsraumgrenze: z = 0,299 m`

Die Höhen `z = 0,490 m` und `z = 0,450 m` liegen nah beieinander. Sie dürfen daher in einer rechtsseitigen, vertikal gestaffelten Legende stehen. Jede Bezugslinie muss dennoch eindeutig dem jeweiligen Niveau zuzuordnen sein. Vorzugsweise enden die Bezugslinien vor der Beschriftungsspalte.

Unterhalb oder seitlich der Zeichnung muss als kleine Fußnote stehen:

`Die Höhen beziehen sich auf den Flansch ur_tool0. Der Greifer ist nicht dargestellt.`

An der unteren Arbeitsraumgrenze erreicht die geschlossene Backenspitze die Förderbandebene. Die obere Grenze `z = 0,600 m` ist wegen einer möglichen kinematischen Singularität des Roboters festgelegt. Sie darf nicht mit einer Kamerakollision begründet oder beschriftet werden.

## Nicht verwenden und nicht behaupten

Folgende Angaben stammen aus früheren, überholten Notizen und dürfen weder in den Abbildungen noch in einer Bildlegende auftauchen:

- Ablageposition `(-0,310 m; 0,477 m; 0,483 m)`
- Höhenwerte `z = 0,305 m`, `z = 0,309 m`, `z = 0,480 m` oder `z = 0,665 m`
- Die Aussage, `z = 0,665 m` sei eine obere Grenze wegen einer Basiskamerakollision
- Die Bezeichnung `ROI` für den eingezeichneten Arbeitsbereich
- Die sichtbare Bezeichnung `robot_base` anstelle von `world`
- Eine gespiegelte Lage von Roboter oder Ablagekiste auf der anderen Bandseite
- Eine aufwendig gerenderte oder dekorative Gesamtanlage

## Empfohlenes Ausgabeformat und Dateinamen

Zunächst keine bestehenden Entwurfsdateien überschreiben. Folgende Ausgaben werden empfohlen:

- `systemskizzen_final_v01.pptx` oder ein vergleichbares bearbeitbares Quellformat mit nativen Formen
- `systemaufbau_draufsicht_final_v01.png`
- `systemaufbau_seitenansicht_final_v01.png`

Die PNG-Dateien sollen nur als Vorschau oder für den späteren Word-Einbau dienen. Das bearbeitbare Quellformat muss erhalten bleiben, damit Beschriftungen nach einer Rückmeldung noch verschoben werden können.

Die Einbindung in `Bericht_FuE_RoboterTetris_V02.docx` erfolgt erst auf ausdrückliche Anweisung. Danach sind der Abbildungsplan und die Word-Einfügemarke im Berichtstext zu aktualisieren.

## Abnahmecheckliste

Vor der Übergabe muss jede Aussage dieser Liste mit `ja` beantwortet werden:

- Sind Roboter und Ablagekiste in der Draufsicht auf der korrekten Bandseite?
- Liegt die Kiste in Bandlaufrichtung vor dem Roboter?
- Liegt die Basiskamera mittig am Bandanfang?
- Zeigt die Bandlaufrichtung im Plan von Bandanfang zu Bandende?
- Zeigt `world-Y+` gegen die Bandlaufrichtung?
- Sind `conveyor_frame` und `world` klar voneinander unterscheidbar?
- Ist der Arbeitsbereich ausschließlich als `Greifbarer Arbeitsbereich` bezeichnet?
- Stammen alle dargestellten Koordinaten aus der Tabelle dieser Datei?
- Enthält die Seitenansicht nur die vier gültigen Höhen `0,299 m`, `0,450 m`, `0,490 m` und `0,600 m`?
- Steht bei der Seitenansicht der Hinweis auf `ur_tool0` und den nicht dargestellten Greifer?
- Schneidet keine Linie, Achse, Pfeilspitze oder Kontur eine Beschriftung?
- Liegen keine Beschriftungen übereinander?
- Wurde bei 100 % Zoom kontrolliert, dass alle Texte gut lesbar sind?

