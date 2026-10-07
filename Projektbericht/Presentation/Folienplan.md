# Abschlusspräsentation Robotertetris

Stand: 02.10.2026. Die freigegebene Präsentation wurde als `Abschlussprasentation_RoboterTetris_V01.pptx` erstellt. Sie enthält 13 Hauptfolien mit einer Sprechzeitplanung von insgesamt 15 Minuten. Mündliche Erläuterungen und Quellen stehen in den tatsächlichen PowerPoint-Foliennotizen. Neue Diagramme und die Messaltergrafik sind native, bearbeitbare PowerPoint-Objekte. Vorhandene Berichtsabbildungen bleiben unverändert, beim Rate-Parameter wird lediglich ein Ausschnitt gezeigt.

## Verbindliche Arbeitsregeln

- Nur in der aktuellen Branch `Codex_systemtest_Tobi` arbeiten. Git verändert ausschließlich der Nutzer.
- Finale PowerPoint erst am Schluss auf ausdrücklichen Auftrag erstellen. Bis dahin Inhalte und Abbildungen schrittweise abstimmen.
- Alle Präsentationsdateien in `Projektbericht/Presentation/` speichern. Sinnvolle Unterordner nach Bedarf.
- Vorlage: `HKA_template_presentation.pptx`. HKA-Grafiken, Originalgestaltung und 16:9-Format erhalten.
- Ziel: etwa 15 Minuten Vortrag. Ob Fragen darin enthalten sein sollen, ist noch nicht abgestimmt.
- Wenig sichtbarer Text, einzelne Stichworte statt Sätze. Viele fachlich relevante Bilder, Systemskizzen und Ablaufdiagramme.
- Mündliche Erläuterungen vollständig in der PowerPoint-Foliennotizen-Funktion hinterlegen, nicht als sichtbaren Fließtext oder zusätzliche Textfolie.
- In den Notizen auch Quellen, Versuchsbedingungen und notwendige Einschränkungen nennen. Schätzungen, Beobachtungen und gemessene Werte trennen.
- Sprache Deutsch, klar und prägnant. Maximal drei Nachkommastellen bei Zahlen.
- Hauptquelle: `../entwuerfe/berichtstext.md`. Abbildungszuordnung: `../abbildungen/abbildungsplan.md` und `../referenzen/abbildungen/`.
- Bestehende Abbildungen nicht stillschweigend bearbeiten. Neue Diagramme zunächst als Vorschläge abstimmen.

## Bereits konkret angeforderte Änderungen

- Folie 2: ausschließlich die am 02.10.2026 freigegebenen Stichpunkte unter „Aufgabenstellung“ und „Zusätzliche Funktionen“. Der detaillierte Aufgabenabgleich steht in den Notizen. Keine Grafik und keine zusätzliche Abschlusszeile auf dieser Folie.
- Folie 3: `conveyor_frame` bleibt neben `world` sichtbar. Historischer Kalibrierrahmen und Laufzeitrahmen klar unterscheiden.
- Folie 3: maßstabsgetreue Überarbeitung am 04.10.2026 beauftragt. Quelle ist die erste Folie von `../abbildungen/systemskizzen_layout_v02.pptx`. Neue separate Einzelfolie: `../abbildungen/systemaufbau_draufsicht_massstab_v01.pptx`. Band und Greifzone verwenden denselben Maßstab. Warte- und Ablagepositionen sind mit W und A markiert. Seitliche Abstände und eine zusätzliche Umrandung des gesamten Flansch-Arbeitsraums entfallen. Angaben entsprechen dem aktuellen Bericht. Quelldatei, Gesamtpräsentation und Word-Bericht bleiben unverändert.
- CHECKPOINT ERLEDIGT: Die kurze Erinnerungsfrage nach drei Arbeitsschritten wurde gestellt. Der Nutzer hat die Bearbeitung bestätigt, die Quelle bereitgestellt und am 04.10.2026 die Erstellung der neuen Einzelfolie freigegeben. Nicht erneut nach derselben Erinnerung fragen.
- Erinnerungszähler: 3 von 3 weiteren Arbeitsschritten seit der Nutzerantwort vom 02.10.2026. Schritt 1 ist die Erstellung der finalen Präsentation V01. Schritt 2 ist die separate Überarbeitung von Folie 6 als `Folie_06_Systemkonzept_AICA_V02.pptx`. Schritt 3 ist die separate Überarbeitung von Folie 8 als `Folie_08_Bewegungsschaetzung_V02.pptx`. Die vorherige Ausgabe des überarbeiteten Folienplans zählt nicht als einer der nächsten drei Schritte. Ein Arbeitsschritt ist eine weitere inhaltliche Abstimmung oder Ausarbeitung auf Nutzerauftrag, kein einzelner Tool-Aufruf. Keine zeitgesteuerte Benachrichtigung gewünscht.
- Bisherige Folie 4 wird in zwei Folien aufgeteilt: Folie 4 Roboterkamera/ChArUco, Folie 5 Basiskamera/AprilGrid im Greifer. Bisherige Folien 5 bis 12 werden dadurch Folien 6 bis 13.
- Der Nutzer hat alle übrigen Folieninhalte am 02.10.2026 mit „Rest passt“ freigegeben und die Präsentationserstellung beauftragt. Spätere Änderungen an Grafiken erfolgen getrennt.

## Abgleich mit der Aufgabenstellung

Die direkte Nutzernachricht vom 02.10.2026 liefert die Aufgabenstellung. Die Beschreibung des bestehenden Systems ist vom eigentlichen Änderungsauftrag zu unterscheiden.

| Vorgabe | Umsetzung und Einordnung |
|---|---|
| Mitfahren und Greifen aus der Bewegung | UR10e folgt der vorhergesagten Position, senkt ab, schließt während der Bewegung, hebt und legt ab. Am Aufbau nachgewiesen. |
| Mehrere gleiche oder unterschiedliche Objekte gleichzeitig unterscheiden und nacheinander greifen | Dauerhafte Objekt-IDs, Abmessungen und Farbe. Auswahl eines rechtzeitig erreichbaren Ziels, anschließend nächster Zyklus. Nicht alle ankommenden Klötze müssen erreichbar sein. |
| Ausschließlich bildbasierte Position und Geschwindigkeit | RGB-/Tiefendaten der Basiskamera, Positionshistorie, Schätzung von Betrag und Richtung der Bandbewegung, Vorhersage hinter dem Bildbereich. Kein Encoder. |
| UR10e mit AICA ansteuern | Eigene Komponenten und vorhandene AICA-Bausteine für Ablauf, Bewegungsregelung und Hardwareanbindung. |
| Schnelles Kalibrierverfahren | Zwei getrennte Anwendungen: Roboterkamera mit ortsfestem ChArUco-Board und automatisierter Orbitfahrt; Basiskamera mit AprilGrid im Greifer und automatisierter Posenfolge. Im finalen Greifbetrieb gilt das Basiskamera-Verfahren. |
| Priorisierung und Bahnplanung für kontrolliertes Greifen | Geometrische Greifbarkeit, Erreichbarkeitsprüfung, bewegungsabhängige Greifebene, gebundenes Ziel, Zustandsautomat und AICA-Regelung. Dies ist gefordert, kein Zusatz. |
| Grundsätzliche Funktion bei verschiedenen Bandgeschwindigkeiten | Stufe 1 mit Gegenprobe 125 bis 133 mm/s. Höhere Stufen bis Stufe 3 erfolgreich beobachtet, aber ohne gemessene Geschwindigkeiten oder systematische Versuchsreihe. |
| Neues Förderband mit variabler Geschwindigkeit | Vorgesehen, wegen Liefer-/Projektbedingungen nicht zwingend. Nicht in den Aufbau integriert; Erprobung mit verschiedenen Stufen des vorhandenen Bandes. |
| Bildverarbeitung bei Bedarf externalisieren | Bedingte Maßnahme, keine Pflicht. Keine separate Rechenplattform umgesetzt; Last durch reduzierte Diagnose, kleine Queue und ROI begrenzt. |
| Keine fehlerfreie Verarbeitung aller Objekte oder feste Mindestquote | Konzeptnachweis. Entwicklungsläufe mit unterschiedlichen Einstellungen nicht zu einer allgemeinen Erfolgsquote zusammenfassen. |

Zusätzlich oder über die explizit genannte Mindestfunktion hinaus: Laufzeitdiagnose mit Objektzuständen und Regelabweichungen, kontrollierte Abbruch-/Ablagebehandlung, eigenständig verwendbare Kalibrieranwendungen. Der erfolgreiche Greifbetrieb ohne aktive Roboterkamera ist eine Vereinfachung des ursprünglich beschriebenen dualen Systems. Farb-/Geometriebeschreibung, Zielauswahl und verschiedene Geschwindigkeiten nicht pauschal als zusätzliche Ziele vermarkten.

Keine vollständige Sortierung in verschiedene Zielkisten oder beliebige Objektgeometrien behaupten. Nachgewiesen sind quaderförmige Klötze und die Ablage in einer Box.

## Folienplan mit Notizinhalten

### 1. Inverses „Tetris“ für Roboter (0:30)

- Sichtbar: Titel, „On the Fly“-Picking, Gruppenmitglieder, Sommersemester 2026.
- Bild: `fig-aufbau-gesamtansicht-1`, Datei `WhatsApp Image 2026-09-24 at 19.31.05 (3).jpeg`.
- Layout: HKA-Titelfolie, großes Foto oder entsprechender Bildausschnitt, keine Hardware-Datentabelle.
- Notizen: realer Versuchsaufbau, Konzeptnachweis, kurzer Übergang zur Aufgabe.

### 2. Aufgabenstellung und Projektziel (1:30)

- Keine Grafik. Zwei klar getrennte Textspalten.
- Aufgabenstellung: Greifen im Lauf; Mehrere Objekte, Priorisierung; Bildbasierte Bewegungsschätzung; UR10e mit AICA; Schnelle Kamerakalibrierung; Verschiedene Bandgeschwindigkeiten.
- Zusätzliche Funktionen: Basiskamera allein; Laufzeitdiagnose; Kontrollierte Abbrüche und Ablage.
- Keine zusätzliche sichtbare Abschlusszeile.
- Notizen: vollständiger Aufgabenabgleich oben, bestehender fester Abgriffpunkt, Sortierung nicht als Projektleistung ausgeben, neues Förderband nicht integriert und nicht zwingend, Externalisierung nur bei Bedarf, höhere Bandstufen nur qualitativ bewertet.

### 3. Versuchsaufbau (1:00)

- Sichtbar: UR10e, Robotiq 2F-140, L515, Ablagebox, `world`, `conveyor_frame`.
- Quelle: `systemskizzen_layout_v03.pdf`, Seite 1.
- Neue bearbeitbare Einzelfolie: `../abbildungen/systemaufbau_draufsicht_massstab_v01.pptx`. Maßstäbliche Überarbeitung aus der bereitgestellten ersten Folie von `systemskizzen_layout_v02.pptx`, ohne Änderung der Quelldatei. Noch nicht automatisch in die Gesamtpräsentation übernommen.
- Geometriegrundlage gemäß Bericht: gerade Länge ungefähr 1,50 m und Breite etwa 0,80 m. Die Koordinaten der Bandkanten x -1,275 bis -0,480 m ergeben rechnerisch 0,795 m und bestimmen die maßstäbliche Breite. Die gezeichnete gerade Länge verwendet die berichtete Näherung von 1,500 m. Greifbereich x -1,000 bis -0,530 m und y -0,320 bis +0,445 m. Daraus 0,470 m quer und 0,765 m längs. Näherungen nicht als neue genaue Vermessung ausgeben.
- Warteposition des Flansches W: x -0,816 m, y +0,350 m, z +0,450 m. Ablageposition A: x -0,316 m, y +0,476 m, z +0,420 m. Beide als horizontale Projektionen einzeichnen. Positionen nicht aus Gründen der Gestaltung verschieben. W liegt nach den hinterlegten Bandkanten rund 62 mm seitlich der geometrischen Bandmitte.
- Abstand Bandanfang bis Beginn der Greifzone ungefähr 0,695 m. Dieser Wert folgt aus dem zuvor grob vermessenen Bandanfang y +1,140 m und der Zonenbegrenzung y +0,445 m. Keine neue Messung. Seitliche Abstände wurden ausdrücklich weggelassen. Robotersymbol und Kistensymbol bleiben schematisch.
- Notizen: Kamera misst vor der Greifzone, spätere Position ist vorhergesagt; `conveyor_frame` historische Kalibrierkonvention, `world` Laufzeitrahmen. Keine Gleichsetzung von `world` und dem technisch gedrehten UR-`base`. D435i vorhanden, aber nicht Teil des finalen Regelpfads.

### 4. Kalibrierung mit der Roboterkamera (1:15)

- Anordnung: bewegte D435i am Flansch, ortsfestes ChArUco-Board auf dem Band.
- Hauptbild: `fig-orbit-trajektorie.jpg`. Kennzeichnung als schematische, KI-generierte Darstellung erhalten.
- Sichtbar: Eye-in-Hand; ChArUco; 9 Wegpunkte; Radius 50 mm; Kamera relativ zum Flansch; eigenständige Kalibrieranwendung.
- Layout: Orbitgrafik nimmt etwa zwei Drittel der Fläche ein, daneben kurze Posen-/Berechnungsschritte. Keine lange Formelherleitung.
- Ablauf: Startpose, Orbitfahrt, Posenmessung, Transformation, Testfahrt.
- Notizen: Kamera blickt je Wegpunkt auf Boardmitte, Detektionen nach Ausschwingen mitteln; Roboterpose und Boardpose gemeinsam auswerten. Ergebnis `T_ee_cam`; weitere Transformationen als Nebenprodukt. Prüffahrt Boardzentrierung und 200 mm entlang Bandachse. Bericht nennt etwa 1,5 bis 2 mm RMSE über Wegpunkte, kein direkt vergleichbares Maß zu den Prüfposen des AprilGrid-Verfahrens. Kalibrieranwendung ist vorhanden, Roboterkamera im finalen Greifbetrieb nicht aktiv.

### 5. Kalibrierung der Basiskamera mit AprilGrid (1:30)

- Anordnung: ortsfeste L515, bewegtes AprilGrid im Greifer.
- Hauptbild: `fig-kalibrierung-board-greifer.png`.
- Sichtbar: AprilGrid im Greifer; 40 Messposen und 4 Prüfposen; rund 4 Minuten; Prüfposenabweichung 0,54 mm; im Greifbetrieb verwendet.
- Layout: großes Foto links, rechts kurzer Ablauf und wenige klar benannte Ergebniswerte.
- Ablauf: Board einlegen, Posenfolge, Berechnung, unabhängige Prüfung, Speichern.
- Notizen: Board manuell einklemmen, danach automatische Messung. Kameralage und Boardlage am Flansch gemeinsam bestimmen. 4 Prüfposen nicht im Fit, Startpose wiederholen zur Verrutschkontrolle. Lauf rund 4 Minuten, mit Vorbereitung etwa 5. Bildfehler 0,34 px, Prüfposenabweichung 0,54 mm, Boardverrutschen 0,04 mm, Wiederholbarkeit Kameralage über 3 Tage unter 1 mm. Nicht als absolute Greifgenauigkeit darstellen. Farb-/Tiefengeometrie der L515 unterscheiden; Tiefenebene 1,0 bis 1,8 Grad verkippt und 5,5 bis 13,5 mm zu tief, daher Anpassung für Tiefenauswertung. Betrieblich erprobt, protokollierter Kalibrier-Greiflauf 7 von 7. Kalibrierung bestimmt keine Bandgeschwindigkeit oder Bewegungsrichtung.

### 6. Systemkonzept und AICA (0:45, bisher Folie 5)

- Separate Revision vom 02.10.2026: `Folie_06_Systemkonzept_AICA_V02.pptx` enthält genau eine Folie. Oben abgerundete Kästen für `base_cam` (15 Hz), `vectoring` (15 Hz), `priority_handler` (20 Hz), `object_follower` (50 Hz), Signal Point Attractor (50 Hz) und IK Velocity Controller im Hardware-Regelpfad (500 Hz). Darunter die hellblaue Funktionskette mit Zuordnungspfeilen. Die Rückmeldekästen entfallen in dieser Übersicht. Die Gesamtpräsentation wurde dabei nicht geändert; die Einzelfolie muss später übernommen werden.
- Neue editierbare Funktionskette: Bildauswertung, Bewegungsschätzung, Zielauswahl, Greifablauf, Bewegungsregelung. Roboter-/Greiferrückmeldungen getrennt einzeichnen.
- Sichtbar: AICA; 15 Hz für Kameraauswertung/Schätzung; 20 Hz Zielauswahl; Zielpose 50 Hz; Roboterregelung 500 Hz.
- Notizen: Komponenten als Funktionsgruppen erklären. Follower liefert Zielpose, Attractor Twist, IK Gelenkgeschwindigkeiten. Keine Verwechslung von Zielposenaktualisierung und Roboterregelung.
- Vollständiger AICA-Screenshot nur Reserve.

### 7. Erkennung und Objektbeschreibung (1:00, bisher Folie 6)

- Hauptbild: `fig-basecam-erkennung-roi.png`, groß, ID/Farbe/ROI gezielt markieren.
- Sichtbar: RGB und Tiefe; ROI; ID; Farbe; Position; Abmessungen; Orientierung.
- Notizen: Tiefenmaske oberhalb Band, umschließendes Rechteck und mediane Tiefe; Farbe aus Farbbild; feste Kennung über aufeinanderfolgende Bilder. Keine Sortierung nach Farbe behaupten. Reflexionen und schwankende Abmessungen motivieren zeitliche Glättung.

### 8. Bewegungsschätzung und Vorhersage (1:15, bisher Folie 7)

- Separate Revision vom 02.10.2026: `Folie_08_Bewegungsschaetzung_V02.pptx` enthält genau eine Folie. Die Gesamtpräsentation bleibt unverändert, die Einzelfolie muss später übernommen werden.
- Grafik: schematische Banddraufsicht mit Rollen am Anfang und Ende. x-Koordinate nach oben, y-Koordinate und Bandrichtung nach rechts. Zwei nicht verbundene, gestreute Punktreihen im Kamerabereich. Die untere Reihe hat größere Punktabstände und stärkere Streuung. Zwei unterschiedlich steile, ansteigende Ausgleichsgeraden laufen gestrichelt bis zum Ende der y-Achse. Eine rote, durchgezogene Gerade zeigt schematisch die gemeinsame Schätzung mit einer Steigung zwischen den Einzelschätzungen. Alle Punkte, Linien, Rollen und Beschriftungen sind native bearbeitbare PowerPoint-Objekte.
- Sichtbar: Positionshistorie, Kamerabereich, Vorhersage, Ende Kamerabild, Objekt 1, Objekt 2, gemeinsame Schätzung. Kennzeichnung als schematische Draufsicht ohne reale Messreihe.
- Notizen: Die räumliche x/y-Steigung veranschaulicht die Bewegungsrichtung. Die Geschwindigkeit braucht zusätzlich Zeitstempel. Die frei orientierte Skizze ist keine Darstellung der tatsächlichen world-Achsen. Einschwingen über zwei Fenster von je 5 Messungen, Toleranz 0,01 m/s, Ausreißer. Gemeinsame Ausgleichsrechnung stabiler Tracks mit höherem Gewicht langer ungestörter Messreihen. Glättung von Position, Abmessungen und Orientierung über 20 Messungen. Gegenprobe rund 128 mm/s gegenüber 125 bis 133 mm/s nur in den Notizen, nicht aus der schematischen Grafik ableiten. Kalibrierung liefert Bezugssystem, keine Bandgeschwindigkeit.

### 9. Zielauswahl und Erreichbarkeit (1:00, bisher Folie 8)

- Neue Grafik: Band mit mehreren Kandidaten, Greifzone, Greifebene. Status ausgewählt/einschwingend/zu spät. Zweite Lage der Greifebene bei höherer Geschwindigkeit.
- Sichtbar: eingeschwungen; geometrisch greifbar; rechtzeitig erreichbar; Zielbindung.
- Notizen: dringendstes noch erreichbares Ziel; Greifebene muss verbleibenden Greifprozess mit Reserve ermöglichen, Zeitvergleich mit Anfahren/Einschwingen. Kein Zielwechsel während eines Versuchs. Durchlauf kann korrektes Auswahlresultat sein.

### 10. Greifen während der Bewegung (1:30, bisher Folie 9)

- Grundlage: `fig-follower-zustandsdiagramm.png`, für Folie vereinfachen und schrittweise hervorheben.
- Zustände: warten, anfahren, folgen, absenken, greifen, heben, ablegen, lösen.
- Neue ergänzende Skizze: bewegter Klotz, Griffpunkt, vorausgesetzte Zielpose, Nachlauf und Vorhalt 0,24 s.
- Notizen: weiter folgen beim Absenken/Schließen, Greiferrückmeldung bestätigen Griff, erst sicher heben dann transferieren; bei gehaltenem Klotz auch nach Abbruch kontrolliert ablegen. Proportionaler Attractor, Nachlauf etwa v/K, Vorhalt empirisch abgestimmt. Keine neue Behauptung zur Greifmindesthöhe 11 mm aufnehmen.

### 11. Inbetriebnahme und Rechenlast (1:00, bisher Folie 10)

- Neue Grafik: Messalter 450 ms gegenüber 139 ms als getrennte Entwicklungsstände, jeweilige Maßnahmen benennen. Keine kontrollierte Einzelmaßnahmenstudie behaupten.
- Bestehendes Zusatzbild: Ausschnitt aus `fig-aica-vectoring-parameter.png`, Parameterbereich um Rate.
- Sichtbar: gemeinsamer Python-Prozess; Diagnoseraten; Queue 1; ROI.
- Notizen: konfigurierte Raten sind nicht Rate neuer Messungen. Diagnose reduzieren, alte Bilder nicht aufstauen, nicht benötigte Ansichten schließen. Letzter dokumentierter Betrieb etwa 8,6 neue Messungen/s trotz Rate 15 Hz. Roboterkamera nicht laden. 50 Hz Zielpose versus 500 Hz Roboterregelung.

### 12. Entwicklungsabnahme und Ergebnisse (1:45, bisher Folie 11)

- Video 20 bis 30 Sekunden eines echten vollständigen Zyklus vorgeschlagen; bisher nicht im Berichtsordner vorhanden. Alternativ reale Bildfolge, keine erfundene Fotoserie.
- Bestehendes Bild: `fig-interface-streamer-betrieb.png`, relevante Anzeigeausschnitte nutzen.
- Ergebnisgrafik: 3 von 7 vor und 9 von 9 nach Zeitgrenzen-Anpassung; separater Dauerlauf 15 Ablagen in rund 2 min, 1 Fehlgriff und 9 durchgelaufene Klötze. Höhere Bandstufen bis 3 nur qualitativ.
- Notizen: Entwicklungsläufe mit unterschiedlichen Konfigurationen, keine gemeinsame Erfolgsquote. Durchläufer waren nicht mehr rechtzeitig erreichbar. Kalibrier-Greiflauf nicht als unabhängigen Zusatzdatensatz doppelt zählen. Erfolg umfasst Anheben und Ablage.

### 13. Bewertung und Weiterentwicklung (1:00, bisher Folie 12)

- Sichtbar: Konzeptnachweis; ohne Encoder; automatische Zielauswahl; Ablagezyklus; Rechenlast/Tiefenfehler.
- Neue reduzierte Skizzen: kürzerer Ablageweg; getrennte Verarbeitung/Diagnose; Tiefenkorrektur und vereinfachte Nachkalibrierung.
- Notizen: quaderförmige Klötze, geprüfter Aufbau; keine Industrieproduktreife behaupten. Roboterkamera erst bei nachgewiesenem Zusatznutzen wieder einbinden. Verbesserungen sind Vorschläge, nicht bereits implementierte Fähigkeiten.

## Zeit und Reserve

Die in den Foliennotizen hinterlegten Zeiten summieren sich auf 15 Minuten. Mehr Kalibrierungszeit wird durch kürzere Erklärungen der übrigen Folien ausgeglichen. Die tatsächliche Dauer wird bei der Vortragsprobe geprüft.

Reserve: vollständiger AICA-Regelpfad, Koordinaten-/Greifgeometrie, ausführliche Kalibrierprüfung, Versuchstabelle und optional Projektzeitplan aus Vorlage. Keine Reservefolien ohne Freigabe final erstellen.

## Nächste Abstimmung

V01 gemeinsam durchsehen und gegebenenfalls einzelne Folien oder Grafiken überarbeiten. Separate Revisionen für Folie 6 und 8 sowie die neue maßstäbliche Draufsicht liegen vor, wurden jedoch nicht automatisch in V01 übernommen. Nächster Schritt ist die Sichtprüfung der Draufsicht durch den Nutzer. Der Erinnerungscheckpoint ist erledigt.
