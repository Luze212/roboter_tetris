# Dokumentenlandkarte

Wo was steht und was verbindlich ist. **Stand:** Die Kette greift am Aufbau Klötze vom
laufenden Band und legt sie ab, allein mit der Basiskamera (finaler Build 24.09.2026).
Die Basiskamera kalibriert ein eigenes automatisches Verfahren mit dem Roboter; in Kraft
ist die Handkalibrierung L6 (28.09.2026).

---

## `uebersicht/` — verstehen, einrichten, bedienen

| Dokument | Inhalt |
|---|---|
| `projektkontext.md` | Aufgabe, Ziele und Ergebnis, Aufbau, Paket, Tests. **Einstieg.** |
| `systemgraph.md` | Der AICA-Graph: Komponenten mit Ein- und Ausgängen und Raten, Ablauf eines Griffs, Bezugssysteme, Kopplungen zwischen Parametern |
| `einrichtung-projektanwendung.md` | Was beim Anlegen der AICA-Anwendung gesetzt werden muss, alle Messwerte des Aufbaus (§8), Bedienung der Kalibrierung (§10) |
| `kalibrierung-abschluss-heute.md` | Konkreter Ablauf für den Abschluss: BaseCamCalibration, Drei-Lagen-Robot-Cam-Kalibrierung, anschließender Vergleich und späterer Folienauftrag |
| `kalibrierdateien-ablage.md` | Aktive Ergebnisdateien und unveränderliche Archive aller Kalibrierverfahren |
| `kalibrierungen-kombinieren.md` | Fusionskomponente und ihre bewusste, manuelle Gewichtung |
| `anwendung-kalibrierung-basiskamera.yaml` | AICA-Anwendung für die Kalibrierung der Basiskamera |
| `Komponentenplan Robotetris - Stand 2026-09-28.docx` | Alle Komponenten, Signale, Abläufe und Einstellungen zum Einlesen, mit Farbcode |

## `architektur/` — die technische Grundlage

| Dokument | Inhalt | |
|---|---|---|
| `entscheidungen.md` | Alle geltenden Entwurfsentscheidungen nach Themen, mit Begründung und Messwerten; am Ende das Register der Kennungen (L6, Z7, B17 …), unter denen Code und Komponentenbeschreibungen verweisen | **normativ** |
| `datenvertraege.md` | Signalspezifikation S1–S10: Felder, Strides, Einheiten | **normativ** |
| `bilder/` | Messdaten, siehe unten | |

### `architektur/bilder/` — Messdaten

| Datei / Ordner | Inhalt |
|---|---|
| `2026-09-22-basiskamera.png` | Bild der Basiskamera: Bandkanten senkrecht im Bild — die Kamera hängt gerade über dem Band (`entscheidungen.md` §5.3) |
| `2026-09-22-b23-punkte.json`, `2026-09-23-b23-punkte.json` | Punkte, an denen Basiskamera und Roboter denselben Klotz maßen — Grundlage der Handkalibrierung L6. Kamerawerte von P1/P2 mit der gedrehten Altkalibrierung, P3–P5 mit deren Korrektur; für eine neue Auswertung zuerst umrechnen |
| `2026-09-25-basiskamera-kalibrierung/` | Kalibrierläufe 2 und 3, Board-Aufnahmen auf Band und Klotz, Messung Tiefe gegen Farbe der L515, Modell des Tiefenfehlers mit Auswerteskripten; `LIESMICH.md` |
| `2026-09-28-basiskamera-kalibrierung/` | Abnahmelauf: Rohdaten, Kalibrierung, Log des Greiflaufs (7 von 7); `LIESMICH.md` |

> `ARCHITECTURE.md` im Projektroot ist die allgemeine Regelsammlung für
> AICA-Komponentenpakete (Aufbau, Signale, JSON-Beschreibungen, Build).

---

## Vorrang

Bei Widerspruch gelten `entscheidungen.md` und `datenvertraege.md`. Änderungen an einem
Signal werden **zuerst** in `datenvertraege.md` gemacht, dann in `contracts.py`
nachgezogen — nie umgekehrt.

Die Entstehung — Konzeptreview, Bauvorlagen, Arbeitslisten und Protokolle der Termine
am Aufbau — ist in der Git-Historie erhalten (Tag `vor-bereinigung`).

---

## Kurzregister

| Frage | steht in |
|---|---|
| Was wurde erreicht? | `uebersicht/projektkontext.md` §1; `entscheidungen.md` §1 |
| Warum eine Zielpose und ein Vorhalt als Zeit? | `entscheidungen.md` §2 |
| Woher der Zeitstempel kommt, warum `base_cam` `global_time_enabled` setzt | §3.1, §3.2 |
| Warum alle Python-Komponenten einen Prozess teilen und was das für die Raten heißt | §3.3, §3.4 |
| `world`, 180° zum UR-Rahmen `base`, Flansch statt TCP | §4.1 |
| Warum der Werkzeugversatz 0,235 m ist, nicht 0,215 | §4.2 |
| Wie `base_cam` Höhe und Lage misst (Median, Parallaxe, `top_depth_bias_mm`) | §5.1 |
| Warum die Greifzone hinter dem Bild der Basiskamera liegt | §5.2 |
| Welche Kalibrierung gilt und woher sie stammt | §5.3 |
| Warum die Farb-Lage nicht die Kalibrierung für `base_cam` ist | §5.4 |
| Wie die automatische Kalibrierung arbeitet und wie sie abgenommen wurde | §5.5; Bedienung `einrichtung-projektanwendung.md` §10 |
| Wie die Bandgeschwindigkeit geschätzt wird (Ziel 3) | §6.1 |
| Warum das Einschwingen zwei Halbfenster vergleicht | §6.2 |
| Wie Klötze hinter dem Bild weitergeführt werden (Status 4) | §6.4 |
| Greifzone, Greifebene, Erreichbarkeit, Ziel-Lock (Ziel 4) | §7 |
| Zustandsautomat, Greif-Freigabe, Greifhöhe | §8 |
| Warum nichts fallen gelassen wird, auch bei einem Abbruch | §8.5 |
| Wie weit sich der Greifer im Winkel des Klotzes dreht (±45°) | §8.6 |
| Sicherheitsgate, Arbeitsraum, Schutzstopp | §9 |
| Wer welche Information besitzt, warum `data_tracker` ein Blatt ist | §10 |
| Warum die Roboterkamera nicht eingebunden ist | §12 |
| Was nicht erreicht wurde, was sich verbessern ließe | §13 |
| Welche Werte von Hand in AICA gesetzt werden | `uebersicht/einrichtung-projektanwendung.md` §2, §5 |
| Welche Parameter zusammenpassen müssen | `uebersicht/systemgraph.md` |
| Worauf eine Kennung wie „L6“ oder „B17“ im Code verweist | `entscheidungen.md`, Register der Kennungen |
