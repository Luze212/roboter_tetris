# Inhaltsstruktur und Arbeitsregeln Projektbericht Robotertetris

Diese Datei ist die verbindliche Arbeitsgrundlage für den Projektbericht.
Unterkapitel, Abbildungen, Messergebnisse und Quellen werden direkt an den
passenden Stellen ergänzt.

## Arbeitsregeln

### Ablage

- Alle neu erzeugten berichtsrelevanten Dateien werden in `Projektbericht/`
  abgelegt.
- Abbildungen kommen in `Projektbericht/abbildungen/`.
- Messdaten, aus denen Tabellen oder Diagramme entstehen, kommen in
  `Projektbericht/daten/`.
- Arbeitsfassungen, falls sie benötigt werden, kommen in
  `Projektbericht/entwuerfe/`.
- Der abgestimmte Berichtstext wird zunächst als Markdown-Datei
  `Projektbericht/entwuerfe/berichtstext.md` geführt. Darin werden Kapitel
  und Unterkapitel direkt gemäß dieser Inhaltsstruktur ergänzt.
- Die aktuelle Word-Fassung wird erst auf ausdrückliche Anweisung aus dem
  abgestimmten Markdown-Text ergänzt oder erzeugt. Änderungen an Word sind
  kein automatischer Folgeschritt nach einem fertig abgestimmten Abschnitt.
- Dateien erhalten aussagekräftige, kurze Namen. Abbildungsdateien beginnen mit
  ihrer Nummer, zum Beispiel `A3_aica_regelpfad.svg`.

### Sprache und Stil

- Der Bericht ist auf Deutsch und im wissenschaftlichen Kontext geschrieben.
- Die Sprache bleibt klar, sachlich und gut verständlich. Sie wirkt nicht
  unnötig akademisch oder künstlich kompliziert.
- Sätze sind kurz und prägnant. Jeder Satz trägt eine fachliche Information.
- Der Satzbau wird variiert. Kurze Feststellungen, begründende Sätze und
  Folgerungen wechseln sich ab. Mehrere Sätze mit gleichem Anfang oder gleicher
  Struktur werden vermieden.
- Gedankenstriche werden nicht verwendet. Stattdessen werden Sätze getrennt
  oder klar umformuliert.
- Selbstverständliche Aussagen, allgemeine Lehrbucherklärungen und Füllsätze
  entfallen.
- Fachbegriffe werden nur verwendet, wenn sie für das Verständnis nötig sind.
  Sie werden bei der ersten Verwendung knapp eingeordnet.
- Messwerte, Schlussfolgerungen und offene Punkte werden klar voneinander
  getrennt.
- Ein später bereitgestelltes Sprachbeispiel ist für Wortwahl, Satzlänge und
  Ton verbindlich.

### Wissenschaftliche Quellen

- Für jeden wissenschaftlichen Abschnitt wird vor dem Schreiben geprüft,
  welche Grundlagen- und Fremdaussagen einen Beleg benötigen.
- Geeignete Quellen werden im Chat mit ihrem Verwendungszweck und einem direkt
  prüfbaren Link vorgeschlagen. Bevorzugt werden Primärquellen, offizielle
  Dokumentationen, Normen und begutachtete Fachveröffentlichungen.
- Erst nach der Rückmeldung des Nutzers werden vorgeschlagene Quellen im
  Bericht verwendet.
- Projektinterne Aufgabenstellung, aktuelle Dokumentation und eigene
  Versuche werden als solche gekennzeichnet. Sie ersetzen keine externe Quelle
  für allgemeine technische Grundlagen.
- Jede verwendete Quelle erhält eine Kennung im
  [`quellenregister.md`](quellenregister.md). Der Eintrag enthält Abschnitt,
  Aussage, vollständige Quelle und Link.
- Die Kennung steht im Markdown-Entwurf direkt an der belegten Aussage. Beim
  späteren Word-Export wird sie in den endgültigen Zitierstil überführt.

### Stilreferenz Bachelorarbeit Wertstromoptimierung

Als sprachliche Referenz dient die geprüfte Bachelorarbeit
`Unger_SS24_BA_Wertstromoptimierung_CVT_Laschen.pdf`. Die PDF wird nicht in
den Projektordner übernommen.

Das vollständige, aus repräsentativen Fachabschnitten abgeleitete Profil steht
in [`stilprofil_bachelorarbeit.md`](stilprofil_bachelorarbeit.md). Es ist bei
jeder neuen Berichtsfassung zu beachten.

- Der Stil ist sachlich, technisch und überwiegend aktiv formuliert.
- Absätze folgen einer klaren Reihenfolge: Sachverhalt, Begründung, Ergebnis
  oder Folgerung.
- Zahlen, Einheiten und Randbedingungen stehen direkt bei der Aussage, die sie
  belegen.
- Fachbegriffe werden konsequent verwendet. Neue Begriffe werden knapp im
  Zusammenhang erklärt.
- Abbildungen, Tabellen und Gleichungen werden im Text angekündigt und danach
  inhaltlich ausgewertet. Sie stehen nicht ohne Einordnung im Bericht.
- Ursache und Wirkung werden direkt benannt, zum Beispiel mit Formulierungen
  wie „Daraus ergibt sich“, „Dies führt zu“ oder „Auf dieser Grundlage“.
- Belegte Aussagen erhalten die Quelle unmittelbar im Satz oder Absatz.
- Der Bericht vermeidet persönliche Wertungen. Ausnahmen sind begründete
  technische Entscheidungen und klar gekennzeichnete Beobachtungen.
- Die Referenz verwendet teils lange Sätze und Gedankenstriche. Für diesen
  Projektbericht gilt weiterhin die strengere Regel: kurze Sätze und keine
  Gedankenstriche.

### Historische Quelle: Vorgängerbericht ConveyorPick

Der vollständig geprüfte Vorgängerbericht bleibt außerhalb des Projektordners.
Seine Erkenntnisse und die Regeln für ihre Verwendung stehen in
[`vorgaengerbericht_conveyorpick.md`](vorgaengerbericht_conveyorpick.md).

- Der historische Abfangprozess mit Wartephase wird klar vom aktuellen
  Pick-on-the-Fly-Regelpfad unterschieden.
- Historische Architekturangaben, Messwerte, Kalibrierwerte und Grenzwerte
  gelten nicht als Beleg für den heutigen Systemstand.
- Wiederverwendbar sind die Ausgangslage, die Abgrenzung, dokumentierte
  Herausforderungen und die Trennung von Erkennungs-, Positions- und
  Greiferfolg.

### Inhaltliche Regeln

- Jedes Kapitel beantwortet eine eigene Frage. Inhalte werden nicht in mehreren
  Kapiteln wiederholt.
- Der Systemaufbau gehört in Kapitel 2, das Konzept in Kapitel 3, die konkrete
  Implementierung in Kapitel 4 und die Messergebnisse in Kapitel 6.
- Ergebnisse werden mit ihren Versuchsbedingungen angegeben. Bewertungen oder
  Schlussfolgerungen folgen erst danach.
- Abbildungen und Tabellen werden nur aufgenommen, wenn sie eine Aussage
  schneller oder klarer vermitteln als Text.
- Jede Abbildung und Tabelle wird im Text eingeführt und fachlich ausgewertet.
- Quellen, Annahmen und Unsicherheiten stehen dort, wo sie für die jeweilige
  Aussage relevant sind.

### Zusammenarbeit und Ablauf

- Berichtstexte werden ausschließlich in der Branch `Codex_systemtest_Tobi`
  erstellt und geändert. Vor einer Berichtänderung wird der Branchname
  rein lesend geprüft. Weicht er ab, wird nicht am Bericht gearbeitet.
- Vor jedem neuen Kapitel oder Unterkapitel steht im Chat eine Gliederung in
  Stichpunkten. Sie nennt die beabsichtigten Aussagen, benötigte Belege,
  Messwerte sowie mögliche Abbildungen oder Tabellen.
- Die Übersicht enthält außerdem den Quellenbedarf. Falls Grundlagenquellen
  nötig sind, folgen vor dem Textentwurf konkrete Quellenvorschläge mit Link.
- Danach folgen konkrete Fragen an den Nutzer. Sie werden auch gestellt, wenn
  bereits Projektinformationen vorliegen. Vorhandene Dokumentation ergänzt die
  Antworten, ersetzt sie aber nicht.
- Die inhaltliche Klärung geht vor dem Schreiben in die Tiefe. Unklare
  Voraussetzungen, Entscheidungen, Versuchsergebnisse und Grenzen werden
  zuerst gemeinsam geklärt.
- Erst nach der Übersicht und den notwendigen Antworten wird der Abschnitt in
  den abgestimmten Markdown-Entwurf geschrieben.
- Der Markdown-Entwurf ist die führende Textfassung. Das Word-Dokument dient
  während der Erarbeitung nur als bewusst erzeugte Zwischenfassung oder als
  abschließende formatierte Abgabeversion.
- Vor jeder Übernahme in die Word-Fassung wird ausdrücklich festgelegt, welche
  abgestimmten Kapitel übertragen werden sollen.
- Nach jedem Abschnitt folgt im Chat eine kurze Einordnung: Was ist fertig,
  was bleibt offen und welcher konkrete Schritt als Nächstes sinnvoll ist.
- Der nächste Schritt wird verständlich erläutert. Die Erklärung nennt seinen
  Zweck, die vom Nutzer benötigten Informationen und das konkrete Ergebnis,
  das anschließend entsteht. Kurze, nicht selbsterklärende
  Handlungsvorschläge genügen nicht.
- Der Prozess wird Kapitel für Kapitel geführt. Es werden keine späteren
  Kapitel vorgezogen, sofern ihre Grundlagen noch nicht geklärt sind.

## 1. Einleitung

### 1.1 Ausgangslage und Motivation

### 1.2 Zielsetzung des Projekts

### 1.3 Abgrenzung und Rahmenbedingungen

### 1.4 Aufbau des Berichts

## 2. System und Versuchsaufbau

### 2.1 Hardwareaufbau

#### 2.1.1 Allgemeine Geometrie und Aufbau

#### 2.1.2 UR10e

#### 2.1.3 Robotiq-2F-140-Greifer

#### 2.1.4 Förderband und Klötze

#### 2.1.5 Basiskamera

#### 2.1.6 Roboterkamera

### 2.2 Koordinatensysteme und Greifgeometrie

#### 2.2.1 Bezugssysteme `world`, UR-Basis und Flansch `ur_tool0`

#### 2.2.2 TCP, Flansch und Griffpunkt

#### 2.2.3 Bandhöhe, Arbeitsraum und Greifzone

### 2.3 Softwareumgebung

#### 2.3.1 AICA und ROS 2

#### 2.3.2 Komponentenstruktur

#### 2.3.3 Signal- und Datenfluss

## 3. Konzept für das Greifen während der Bandbewegung

### 3.1 Anforderungen an den Regelpfad

### 3.2 Objekterkennung mit der Basiskamera

### 3.3 Geschwindigkeits- und Positionsschätzung

### 3.4 Zielauswahl und Erreichbarkeitsprüfung

### 3.5 Bahnführung mit Attractor und IK-Velocity-Controller

### 3.6 Greifablauf und Fehlerbehandlung

#### 3.6.1 Anfahren

#### 3.6.2 Folgen

#### 3.6.3 Absenken und Greifen

#### 3.6.4 Heben, Ablage und Abbruch

## 4. Umsetzung

### 4.1 Komponentenübersicht

### 4.2 `base_cam` und Objektdaten

### 4.3 `vectoring` und Track-Vorhersage

### 4.4 `priority_handler` und Greifebene

### 4.5 `object_follower` und Zustandsautomat

### 4.6 Greiferansteuerung und Rückmeldungen

### 4.7 Sicherheitsgrenzen und Arbeitsraum

## 5. Inbetriebnahme und Optimierung

### 5.1 Kalibrierung und Positionsgenauigkeit

### 5.2 Latenz, Rechenlast und Bildrate

### 5.3 Wahl von Vorhalt, Attractor-Gain und Geschwindigkeitslimit

### 5.4 Optimierung der Basiskamera

#### 5.4.1 Bildausschnitt (ROI)

#### 5.4.2 Randnahe Klötze

#### 5.4.3 Flache Klötze und Mindesthöhe

## 6. Versuchsergebnisse

### 6.1 Versuchsbedingungen

### 6.2 Validierung der Geschwindigkeitsschätzung

### 6.3 Genauigkeit der Basiskamera

### 6.4 Erstes Pick-on-the-Fly

### 6.5 Wiederholversuche und Randbereich

### 6.6 Ergebnisbewertung

#### 6.6.1 Erfolgreiche Griffe

#### 6.6.2 Grenzen des aktuellen Systems

## 7. Diskussion und Ausblick

### 7.1 Erreichte Projektziele

### 7.2 Grenzen des aktuellen Systems

### 7.3 Nächste technische Schritte

#### 7.3.1 Priorisierung mehrerer Klötze

#### 7.3.2 Robustere Basiskamera-Erkennung

#### 7.3.3 Roboterkamera als Korrektursignal

#### 7.3.4 Stabilisierung der 500-Hz-Regelschleife

## 8. Fazit

## Vorgesehene Abbildungen und Diagramme

| Nr. | Inhalt | Vorgesehene Stelle |
|---|---|---|
| A1 | Beschriftete Gesamtansicht des realen Aufbaus | 2.1 |
| A2 | Systembild mit Roboter, Band, Kameras, Arbeitsraum, Greifzone und Ablage | 2.2 |
| A3 | AICA-Regelpfad von `base_cam` bis IK-Controller | 2.3 oder 3.1 |
| A4 | Zustandsdiagramm des `object_follower` | 3.6 oder 4.5 |
| A5 | Koordinatensysteme sowie TCP, Flansch und Griffpunkt | 2.2 |
| A6 | Bildfolge eines erfolgreichen Pick-on-the-Fly | 6.4 |
| A7 | Bandposition über Zeit: Messung und Track-Vorhersage | 6.2 |
| A8 | Einzelgeschwindigkeiten, Pool-Schätzung und Stoppuhr-Gegenprobe | 6.2 |
| A9 | Alter und neuer ROI der Basiskamera | 5.4.1 |
| A10 | Rechenlast, Bildrate und 500-Hz-Regelschleife | 5.2 oder 6.6 |
| A11 | Abmessungen des geöffneten Robotiq-2F-140-Greifers | 2.1.3 |

## Vorgesehene Tabellen

| Nr. | Inhalt | Vorgesehene Stelle |
|---|---|---|
| T1 | Hardwarekomponenten und Aufgaben | 2.1 |
| T2 | Signale des Regelpfads | 2.3 |
| T3 | Relevante Regel- und Sicherheitsparameter | 5.3 |
| T4 | Versuchsergebnisse: Klotz, Geschwindigkeit, Regelabweichung und Ergebnis | 6.5 |
