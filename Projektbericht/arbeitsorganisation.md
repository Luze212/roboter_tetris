# Arbeitsorganisation für den Projektbericht

Diese Regelung gilt, solange mehrere Personen gleichzeitig am Bericht arbeiten.
Sie soll fachliche Überschneidungen und Git-Konflikte vermeiden.

## Git-Workflow

Für parallele Arbeit wird **nicht** auf dieselbe Branch gepusht. Jede Person
arbeitet in einer eigenen Themen-Branch. Die aktuelle Branch
`Codex_systemtest_Tobi` ist die Integrationsbranch für den Bericht.

Empfohlene Benennung:

```
bericht/<kapitel>-<thema>-<vorname>
```

Beispiele:

```
bericht/kapitel-3-konzept-lukas
bericht/kapitel-5-kalibrierung-tobias
bericht/kapitel-7-versuche-maria
```

Die Arbeitsschritte sind:

1. Die eigene Themen-Branch aus dem aktuellen Stand der Integrationsbranch
   erzeugen.
2. Nur die vereinbarten Kapitel und zugehörigen Dateien bearbeiten.
3. Kleine, klar abgegrenzte Commits erstellen.
4. Änderungen über einen Pull Request oder durch eine Person mit
   Integrationsverantwortung in `Codex_systemtest_Tobi` übernehmen.
5. Die Integrationsperson prüft dabei Quellen, Abbildungs- und Tabellenverweise
   sowie die endgültige Reihenfolge.

Gleichzeitiges Pushen auf dieselbe Branch ist technisch möglich, erzeugt aber
bei nicht zusammenführbaren Änderungen abgewiesene Pushes und manuelle
Merge-Konflikte. Besonders konfliktanfällig sind `berichtstext.md`,
`quellenregister.md`, `abbildungsplan.md`, `inhaltsstruktur.md` und das
Word-Dokument. Diese Dateien werden deshalb nach den folgenden Regeln
behandelt.

## Zuständigkeiten und Schreibbereiche

| Bereich | Bearbeitung in Themen-Branches | Integration |
|---|---|---|
| Text eines klar abgegrenzten Unterkapitels in `entwuerfe/berichtstext.md` | erlaubt | zusammenführen und Stilübergänge prüfen |
| Neue Quelle | Einzeldatei unter `referenzen/quellen/` anlegen | in `quellenregister.md` übernehmen |
| Neue Abbildung | Datei und Einzeldatei unter `referenzen/abbildungen/` anlegen | endgültige Nummer und Eintrag im Abbildungsplan vergeben |
| Neue Tabelle | Einzeldatei unter `referenzen/tabellen/` anlegen | endgültige Nummer vergeben |
| `inhaltsstruktur.md` | nur nach Absprache | eine Person führt Änderungen zusammen |
| `quellenregister.md` und `abbildungsplan.md` | keine neuen fortlaufenden Nummern vergeben | eine Person führt die Register |
| Word-Dokument | nicht parallel bearbeiten | nur auf ausdrückliche Anweisung und durch eine Person |

Mehrere Personen dürfen gleichzeitig an `berichtstext.md` arbeiten, wenn sie
voneinander getrennte Überschriften bearbeiten. Niemand formatiert dabei den
gesamten Text um, verschiebt fremde Abschnitte oder ändert Quell- und
Abbildungskennungen anderer Personen.

## Kennungen statt fortlaufender Nummern

Während der parallelen Markdown-Phase werden keine fortlaufenden Kennungen wie
`Q11`, `A12` oder `T5` verwendet. Sie können sonst in zwei Branches doppelt
entstehen.

Stattdessen verwendet jede neue Referenz eine eindeutige fachliche Kennung:

- Quelle: `src-<herausgeber>-<kurzthema>`
- Abbildung: `fig-<bereich>-<inhalt>`
- Tabelle: `tab-<bereich>-<inhalt>`

Beispiele:

```
src-realsense-d435i-datenblatt
fig-kalibrierung-antastpunkte
tab-versuche-greifergebnisse
```

Vor dem Anlegen wird die gewünschte Kennung mit `rg` im Ordner
`Projektbericht/` gesucht. Existiert sie bereits, wird eine genauere,
abweichende Kennung gewählt. Jede Kennung erhält eine eigene Datei unter
`Projektbericht/referenzen/`. Damit ändern zwei Personen nicht dieselbe
Registerzeile.

Im Fließtext werden neue Elemente vorläufig mit ihrer fachlichen Kennung
referenziert, zum Beispiel:

```
Die Messanordnung ist in Abbildung `fig-kalibrierung-antastpunkte` dargestellt.
Die Parameter sind in Tabelle `tab-follower-parameter` zusammengefasst.
Die Angaben zur Kamera beruhen auf [src-realsense-d435i-datenblatt].
```

Bei der Integration ersetzt die zuständige Person diese Kennungen durch die
endgültigen Quellenangaben sowie Word-Abbildungs- und Tabellennummern. So sind
die Nummern unabhängig von der Reihenfolge, in der Beiträge entstehen.

Die früheren Kennungen `Q01` bis `Q10` sowie `A1` bis `A11` bleiben nur als
historische Zuordnung in den zentralen Registern erhalten. Der Markdown-Entwurf
und neue Beiträge verwenden ausschließlich die stabilen Kennungen.

## Verbindliche Prüfschritte vor einem Pull Request

- Nur die vereinbarten Kapitel wurden geändert.
- Jede neue Quelle, Abbildung oder Tabelle besitzt eine eigene Referenzdatei.
- Jede Abbildung und Tabelle ist im Text vor ihrem Auftreten referenziert.
- Webquellen wurden vor der Textübernahme von der Projektgruppe geprüft.
- Bei neuen Abbildungen liegt die Quelldatei im vorgesehenen Unterordner von
  `Projektbericht/abbildungen/`.
- Das Word-Dokument wurde nicht nebenläufig geändert.
