# Referenzverwaltung während der parallelen Berichtserstellung

Diese Struktur verhindert doppelte Quellen-, Abbildungs- und
Tabellenkennungen. Sie ergänzt die bestehenden zentralen Register. Während der
parallelen Arbeit werden neue Einträge ausschließlich als einzelne Dateien in
den passenden Unterordnern angelegt. Zuerst
[`../README.md`](../README.md) und
[`../arbeitsorganisation.md`](../arbeitsorganisation.md) lesen.

## Unterordner

- `quellen/`: eine Datei je externer oder projektinterner Quelle
- `abbildungen/`: eine Datei je geplanter Abbildung
- `tabellen/`: eine Datei je geplanter Tabelle

Die Datei heißt immer wie ihre stabile Kennung. Beispiel:

```
quellen/src-realsense-d435i-datenblatt.md
abbildungen/fig-kalibrierung-antastpunkte.md
tabellen/tab-versuche-greifergebnisse.md
```

## Ablauf

1. Kennung nach der Regel in `../arbeitsorganisation.md` wählen.
2. Mit `rg` sicherstellen, dass sie noch nicht verwendet wird.
3. Die passende Vorlagendatei kopieren und vollständig ausfüllen.
4. Im eigenen Text die Kennung verwenden. Quellen stehen direkt an der
   belegten Aussage. Abbildungen und Tabellen werden vor ihrem Auftreten im
   Fließtext referenziert.
5. Bei der Integration werden die Informationen zentral in
   `../quellenregister.md`, `../abbildungen/abbildungsplan.md` und später in
   das Word-Dokument übernommen.

Die fortlaufenden Nummern für Literaturverzeichnis, Abbildungen und Tabellen
werden ausschließlich bei der zentralen Integration vergeben.

Die früheren Q-, A- und T-Nummern dienen nur noch als historische Zuordnung in
den zentralen Registern. Im aktuellen Markdown-Entwurf werden sie nicht
verwendet.
