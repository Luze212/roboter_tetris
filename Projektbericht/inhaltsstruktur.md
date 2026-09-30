# Inhaltsstruktur und Arbeitsregeln Projektbericht Robotertetris

Diese Datei ist die verbindliche Arbeitsgrundlage für den Projektbericht.
Unterkapitel, Abbildungen, Messergebnisse und Quellen werden direkt an den
passenden Stellen ergänzt. Für den Einstieg bei paralleler Arbeit zuerst
[`README.md`](README.md) lesen.

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
- Dateien erhalten aussagekräftige, kurze Namen. Abbildungsdateien verwenden
  ihre stabile Kennung, zum Beispiel `fig-regelpfad-aica.svg`.

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
- Während der parallelen Bearbeitung erhält jede neue Quelle zunächst eine
  eindeutige fachliche Kennung und eine Einzeldatei unter
  [`referenzen/quellen/`](referenzen/quellen/). So können keine doppelten
  fortlaufenden Quellennummern entstehen.
- Bei der zentralen Integration wird die Quelle in
  [`quellenregister.md`](quellenregister.md) aufgenommen. Der Eintrag enthält
  Abschnitt, Aussage, vollständige Quelle und Link. Erst dann wird die
  endgültige Kennung für den Word-Export vergeben.
- Die fachliche Kennung steht im Markdown-Entwurf direkt an der belegten
  Aussage. Beim späteren Word-Export wird sie in den endgültigen Zitierstil
  überführt.

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
- Vor jeder Abbildung, Tabelle oder Gleichung steht im Fließtext mindestens ein
  kurzer Verweis, zum Beispiel „Abbildung `fig-systemaufbau-draufsicht` zeigt
  …“ oder „Die Ergebnisse sind in Tabelle `tab-versuche-greifergebnisse`
  zusammengefasst“. Der Verweis nennt, welche Aussage das Element unterstützt.
  Ein Element darf nicht ohne vorherigen Textverweis erscheinen.
- Besteht eine Abbildung aus Teilbildern, werden die Teilbilder eindeutig über
  eigene stabile Kennungen referenziert. Die endgültige Nummerierung wird erst
  beim Word-Export vergeben.
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
- Jede Abbildung und Tabelle wird vor ihrem Erscheinen im Text eingeführt und
  anschließend fachlich ausgewertet.
- Quellen, Annahmen und Unsicherheiten stehen dort, wo sie für die jeweilige
  Aussage relevant sind.

### Zusammenarbeit und Ablauf

- Die Branch `Codex_systemtest_Tobi` ist die Integrationsbranch. Bei paralleler
  Arbeit erhält jede Person eine eigene Themen-Branch. Erst nach Prüfung werden
  deren Änderungen in die Integrationsbranch übernommen.
- Fortlaufende Kennungen für Quellen, Abbildungen und Tabellen werden nur bei
  der Integration vergeben. In Themen-Branches gelten die fachlichen
  Kennungen und Einzeldateien unter `referenzen/`.
- Die verbindliche Arbeitsorganisation, Branch-Namen und Prüfschritte stehen
  in [`arbeitsorganisation.md`](arbeitsorganisation.md).
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

## 5. Kalibrierung

### 5.1 Kalibrierungsstrategie und Bezugssysteme

### 5.2 Extrinsische Kalibrierung der Basiskamera

### 5.3 Hand-Auge-Kalibrierung der Roboterkamera

### 5.4 Validierung der Koordinatentransformation und Positionsgenauigkeit

## 6. Inbetriebnahme und Optimierung

### 6.1 Inbetriebnahme des Regelpfads

### 6.2 Latenz, Rechenlast und Bildrate

### 6.3 Wahl von Vorhalt, Attractor-Gain und Geschwindigkeitslimit

### 6.4 Optimierung der Basiskamera

#### 6.4.1 Bildausschnitt (ROI)

#### 6.4.2 Randnahe Klötze

#### 6.4.3 Flache Klötze und Mindesthöhe

## 7. Versuchsergebnisse

### 7.1 Versuchsbedingungen

### 7.2 Validierung der Geschwindigkeitsschätzung

### 7.3 Genauigkeit der Basiskamera

### 7.4 Erstes Pick-on-the-Fly

### 7.5 Wiederholversuche und Randbereich

### 7.6 Ergebnisbewertung

#### 7.6.1 Erfolgreiche Griffe

#### 7.6.2 Grenzen des aktuellen Systems

## 8. Diskussion und Ausblick

### 8.1 Erreichte Projektziele

### 8.2 Grenzen des aktuellen Systems

### 8.3 Nächste technische Schritte

#### 8.3.1 Priorisierung mehrerer Klötze

#### 8.3.2 Robustere Basiskamera-Erkennung

#### 8.3.3 Roboterkamera als Korrektursignal

#### 8.3.4 Stabilisierung der 500-Hz-Regelschleife

## 9. Fazit

## Vorgesehene Abbildungen und Diagramme

| Stabile Kennung | Inhalt | Vorgesehene Stelle |
|---|---|---|
| `fig-aufbau-gesamtansicht-1` und `fig-aufbau-gesamtansicht-2` | Beschriftete Gesamtansichten des realen Aufbaus | 2.1 |
| `fig-systemaufbau-uebersicht` | Systembild mit Roboter, Band, Kameras, Arbeitsraum, Greifzone und Ablage | 2.2 |
| `fig-regelpfad-aica` | AICA-Regelpfad von `base_cam` bis IK-Controller | 2.3 oder 3.1 |
| `fig-follower-zustandsdiagramm` | Zustandsdiagramm des `object_follower` | 3.6 oder 4.5 |
| `fig-koord-systeme` | Koordinatensysteme sowie TCP, Flansch und Griffpunkt | 2.2 |
| `fig-pick-bildfolge` | Bildfolge eines erfolgreichen Pick-on-the-Fly | 7.4 |
| `fig-track-positionsverlauf` | Bandposition über Zeit: Messung und Track-Vorhersage | 7.2 |
| `fig-geschwindigkeitsschaetzung` | Einzelgeschwindigkeiten, Pool-Schätzung und Stoppuhr-Gegenprobe | 7.2 |
| `fig-basiskamera-roi-vergleich` | Alter und neuer ROI der Basiskamera | 6.4.1 |
| `fig-systemleistung` | Rechenlast, Bildrate und 500-Hz-Regelschleife | 6.2 oder 7.6 |
| `fig-greifer-robotiq-2f140-abmessungen` | Abmessungen des geöffneten Robotiq-2F-140-Greifers | 2.1.3 |

## Vorgesehene Tabellen

| Stabile Kennung | Inhalt | Vorgesehene Stelle |
|---|---|---|
| `tab-hardware-komponenten` | Hardwarekomponenten und Aufgaben | 2.1 |
| `tab-regelpfad-signale` | Signale des Regelpfads | 2.3 |
| `tab-regel-sicherheitsparameter` | Relevante Regel- und Sicherheitsparameter | 6.3 |
| `tab-versuche-greifergebnisse` | Versuchsergebnisse: Klotz, Geschwindigkeit, Regelabweichung und Ergebnis | 7.5 |
