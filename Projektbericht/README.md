# Einstieg für die parallele Berichtserstellung

Diese Datei ist der Einstiegspunkt für Personen und KIs, die am
Projektbericht arbeiten. Vor jeder Änderung sind die folgenden Dateien in
dieser Reihenfolge zu lesen:

1. [`inhaltsstruktur.md`](inhaltsstruktur.md): verbindliche Kapitelstruktur,
   Schreibregeln und Arbeitsablauf.
2. [`stilprofil_bachelorarbeit.md`](stilprofil_bachelorarbeit.md): gewünschter
   Sprachstil.
3. [`arbeitsorganisation.md`](arbeitsorganisation.md): Branch-Workflow,
   Zuständigkeiten und Umgang mit Konflikten.
4. [`referenzen/README.md`](referenzen/README.md): Quellen-, Abbildungs- und
   Tabellenkennungen.
5. [`entwuerfe/berichtstext.md`](entwuerfe/berichtstext.md): aktueller
   abgestimmter Berichtstext.

## Kurzregeln

- Der Bericht wird zuerst in `entwuerfe/berichtstext.md` geschrieben. Das
  Word-Dokument wird erst auf ausdrückliche Anweisung und nicht parallel
  bearbeitet.
- Vor einem neuen Kapitel oder Unterkapitel werden im Chat zuerst die
  geplanten Inhalte, benötigten Informationen, mögliche Abbildungen und
  Quellen vorgestellt. Danach folgen konkrete inhaltliche Fragen.
- Quellen für wissenschaftliche Grundlagen werden zuerst mit Link vorgeschlagen
  und erst nach Prüfung durch die Projektgruppe im Text verwendet.
- Der Stil ist deutsch, wissenschaftlich und klar. Sätze bleiben prägnant.
  Keine unnötigen Erklärungen, Wiederholungen oder Gedankenstriche.
- Zahlenwerte im Berichtstext, in Tabellen und in Bildunterschriften werden
  auf höchstens drei Nachkommastellen gerundet. Rohdaten und Quellendateien
  behalten ihre ursprüngliche Genauigkeit.
- Abbildungen, Tabellen und Gleichungen werden vor ihrem Auftreten im Text
  referenziert und danach fachlich eingeordnet.
- Zu jeder Abbildung wird im Berichtstext am vorgesehenen Einfügepunkt bereits
  eine Bildunterschrift gespeichert. Sie wird beim späteren Word-Export
  unverändert übernommen und muss mit dem Eintrag im Abbildungsregister
  übereinstimmen.

## Parallele Arbeit

Jede Person arbeitet in einer eigenen Themen-Branch. Nicht gleichzeitig auf
`Codex_systemtest_Tobi` pushen. Diese Branch ist die Integrationsbranch.
Details stehen in [`arbeitsorganisation.md`](arbeitsorganisation.md).

## Referenzen

Im Markdown-Entwurf werden ausschließlich stabile Kennungen verwendet:

- Quellen: `src-<herausgeber>-<kurzthema>`
- Abbildungen: `fig-<bereich>-<inhalt>`
- Tabellen: `tab-<bereich>-<inhalt>`

Für jedes neue Element wird eine Einzeldatei unter `referenzen/` angelegt. Die
Vorlagen liegen in den jeweiligen Unterordnern. Es werden keine Q-, A- oder
T-Nummern vergeben. Die endgültigen Word-Nummern entstehen erst bei der
zentralen Integration.

## Ablage

- Berichtstext: `entwuerfe/berichtstext.md`
- Bilddateien: `abbildungen/`
- Messdaten: `daten/`
- Quellen-, Abbildungs- und Tabellenmetadaten: `referenzen/`
- Zentrale historische Zuordnung: `quellenregister.md` und
  `abbildungen/abbildungsplan.md`
