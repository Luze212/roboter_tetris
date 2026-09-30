# Abbildungen im Projektbericht

Vor dem Anlegen oder Verwenden einer Abbildung zuerst
[`../README.md`](../README.md) und
[`../referenzen/README.md`](../referenzen/README.md) lesen.

## Verbindlicher Ablauf

1. Eine eindeutige Kennung nach dem Muster `fig-<bereich>-<inhalt>` wählen und
   mit `rg` unter `Projektbericht/` auf Einzigartigkeit prüfen.
2. Die Bilddatei mit dieser Kennung beginnen lassen und hier ablegen, zum
   Beispiel `fig-kalibrierung-antastpunkte.png`.
3. Die Vorlage `../referenzen/abbildungen/_vorlage.md` als
   `../referenzen/abbildungen/<kennung>.md` kopieren und vollständig ausfüllen.
4. Die Abbildung vor ihrem Auftreten im Fließtext mit ihrer fortlaufenden
   Nummer referenzieren, zum Beispiel: „Die Messanordnung ist in Abbildung 18
   dargestellt.“ Die Zuordnung zur stabilen Kennung wird zugleich in
   [`../nummerierungszuordnung.md`](../nummerierungszuordnung.md) ergänzt.
5. Erst danach kann die Abbildung beim späteren Word-Export eingefügt werden.

Keine A-Nummern vergeben. Die fortlaufende Abbildungsnummer gilt bereits im
Markdown-Bericht und wird später unverändert nach Word übernommen.

## Bestehende Dateien

Der aktuelle Status jeder vorgesehenen Berichtabbildung steht in
[`abbildungsplan.md`](abbildungsplan.md). Dateien, die dort unter
„Arbeitsstände, nicht zur Übernahme in Word vorgesehen“ stehen, sind keine
freigegebenen Berichtabbildungen.
