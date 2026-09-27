# Dokumentenlandkarte

Wo was liegt und was verbindlich ist. **Stand 24.09.2026, finaler Build:** Die
Kette greift am Aufbau Klötze vom laufenden Band und legt sie ab; gegriffen wird
allein mit der Basiskamera.

---

## `uebersicht/` — Überblick und Betrieb

Was man liest, um das System zu verstehen, einzurichten und zu bedienen.

| Dokument | Inhalt |
|---|---|
| `projektkontext.md` | Ziele, Aufbau, Abgrenzungen, Arbeitsweise. **Einstieg.** |
| `Komponentenplan Robotetris - Stand 2026-09-24.docx` | Beschreibung aller Komponenten, Signale, Abläufe und Einstellungen zum Einlesen, mit Farbcode |
| `systemgraph.md` | Der AICA-Graph: Komponenten mit Ein- und Ausgängen und Raten, Ablauf eines Griffs, Bezugssysteme, Kopplungen zwischen Parametern |
| `einrichtung-projektanwendung.md` | Was beim Anlegen der AICA-Anwendung gesetzt werden muss, mit allen Messwerten des Aufbaus |
| `ablauf-kalibrierung-aufbau.md` | Ablauf des Termins am 28.09.2026: automatische Kalibrierung der Basiskamera laufen lassen, einstellen, gegen L6 antasten, Greiflauf; nach dem Termin ins Archiv |

## `architektur/` — die technische Grundlage

Das Warum hinter den Entscheidungen und die verbindlichen Schnittstellen.

| Dokument | Inhalt | |
|---|---|---|
| `entscheidungen.md` | Alle Architekturentscheidungen mit Begründung: Themen 1–7, Nachträge 1–13. Neuere Nachträge gehen vor, wo sie frühere Festlegungen berühren. Nachtrag 6: Projektvorgaben; 7–11: Bau der Komponenten; 12–13: Inbetriebnahme am Aufbau bis zum finalen Build, danach die Kalibrierung der Basiskamera (L27) | **normativ** |
| `datenvertraege.md` | Signalspezifikation S1–S10: Felder, Strides, Einheiten | **normativ** |
| `bilder/` | Messbilder und Rohdaten: Kalibrierung L6 (22./23.09.2026), Kalibrierung mit dem Roboter und Tiefe gegen Farbe der L515 (25.09.2026) | |

## `archiv/` — abgeschlossen, nicht mehr gepflegt

Arbeitsdokumente aus Bau und Inbetriebnahme. Sie geben den Stand ihrer Zeit wieder;
gültig ist, was in `uebersicht/` und `architektur/` steht.

| Dokument | Inhalt |
|---|---|
| `fahrplan-aufbau.md` | Ablauf der Termine am Roboter, Mitlese- und Messverfahren |
| `offene-punkte.md` | Arbeitsliste der Mess- und Festlegungspunkte (A–E) mit ihren Ergebnissen |
| `uebergabe.md` | Übergabe zwischen den Rechnern während des Aufbaus |
| `specs/` | Bauvorlagen der Komponenten und der Umsetzungsplan (13.09.2026) |
| `robot-cam-befunde.md` | Szene, Materialphysik und verworfene Wege der Roboterkamera |
| `vorgaengerprojekt-abgleich.md` | Abgleich gegen das Vorgängerarchiv `UR10_Pick_ws` |
| `2026-09-05-konzeptreview-komponentenplan.md` | Die ursprüngliche Analyse mit 61 Befunden, abgearbeitet in den Themen 1–7 |
| `Komponentenplan Robotetris.docx` | Der Originalstand des Komponentenplans |
| `2026-06-01-robotiq-gripper-component-design.md` | Bauvorlage des Greifers; liegt in `.gitignore` |
| `creation-1.png`, `creation-2.png` | Bilder aus dem README der AICA-Paketvorlage |

> `ARCHITECTURE.md` liegt im Projektroot. Es ist die **verbindliche
> AICA-Regelsammlung** und gilt über allem in diesem Ordner.

---

## Vorrang

Bei Widerspruch gelten `entscheidungen.md` und `datenvertraege.md`. Alles andere
ist daraus abgeleitet. Änderungen an einem Signal werden **zuerst** in
`datenvertraege.md` gemacht, dann in `contracts.py` nachgezogen — nie umgekehrt.

---

## Was wo entschieden wurde — Kurzregister

„ebd.“ meint `architektur/entscheidungen.md`.

| Frage | steht in |
|---|---|
| Warum eine Zielpose und kein Twist? | `architektur/entscheidungen.md`, Thema 1 |
| Woher kommt der Zeitstempel, wie wird Latenz behandelt? | ebd., Thema 2 |
| Welche Kalibrierung tritt wo ein? | ebd., Thema 3 |
| Wer besitzt welche Information? | ebd., Thema 4 |
| Wie sehen die Signale aus? | `architektur/datenvertraege.md` |
| Zustandsautomat, Greifhöhe, Orientierung | ebd., Thema 6 |
| Sicherheitsgate, Arbeitsraum, Singularitäten | ebd., Thema 7 |
| Die fünf Befunde aus der Code-Durchsicht | ebd., Nachtrag 3 |
| Welche AICA-Parameter beim Anlegen gesetzt werden müssen | `uebersicht/einrichtung-projektanwendung.md` |
| Die gemessenen Werte des Aufbaus | `uebersicht/einrichtung-projektanwendung.md` §8; Herleitung in ebd. Nachträge 5 und 13 |
| Warum Höhen Flansch- und nicht TCP-Maße sind | ebd. Nachtrag 5 / M8 |
| Wie das System aufgebaut ist und welche Parameter zusammenpassen müssen | `uebersicht/systemgraph.md` |
| Die offiziellen Projektziele | ebd. Nachtrag 6 / Z1; `uebersicht/projektkontext.md` §1 |
| Wie die Geschwindigkeit geschätzt wird (Ziel 3) | ebd. Nachtrag 6 / Z2–Z4 |
| Warum der Werkzeugversatz 0,235 m ist, nicht 0,215 | ebd. Nachtrag 6 / Z7 |
| Warum der Vorhalt eine Zeit ist | ebd. Nachtrag 6 / Z6 |
| Warum das Einschwingkriterium zwei Halbfenster vergleicht | ebd. Nachtrag 6 / Z9 |
| Ab wann kein Griff mehr beginnen darf (Greifebene) | ebd. Nachtrag 6 / Z11 |
| Wie der `priority_handler` Anfahrweg und Greiferbreite rechnet | ebd. Nachtrag 7 / H1, H2 |
| Was „Koordinate entlang der Bandrichtung“ in S4 heißt | ebd. Nachtrag 7 / H4; `contracts.along_belt` |
| Warum S10 ein Feld `present` hat und wann Einträge verfallen | ebd. Nachtrag 7 / T1 |
| Wie der Follower startet und warum Arbeitsraum und Beobachtungspose Pflichtparameter sind | ebd. Nachtrag 8 / F3, F4; Nachtrag 13 / L15 |
| Wie `lead_time_s` eingemessen wird und was `outcome = 4` heißt | ebd. Nachtrag 9 / G7, G2 |
| Wie der Greifzyklus abläuft, warum die Freihöhe 0,49 m ist | ebd. Nachtrag 10 / J1–J8 |
| Warum die Anzeige ASCII schreibt und wann sie „veraltet“ zeigt | ebd. Nachtrag 11 / V1–V3 |
| Warum Basiskamera und Roboter das Band an verschiedenen Stellen sahen (180°) | ebd. Nachtrag 8 / F1, Nachtrag 12 / K5 |
| Woher die Kalibrierung der Basiskamera stammt, warum sie senkrecht schaut, was die Parallaxe war | ebd. Nachtrag 13 / L6 |
| Wie die Basiskamera mit dem Roboter kalibriert wird, warum `base_cam` dafür unverändert bleibt und wie die Tiefe der L515 umgerechnet wird | ebd. Nachtrag 13 / L27; `uebersicht/einrichtung-projektanwendung.md` §10; `uebersicht/ablauf-kalibrierung-aufbau.md` |
| Warum alle Python-Komponenten einen Prozess teilen und was das für die Messrate heißt | ebd. Nachtrag 13 / L2 |
| Warum Greifzone und Wartebereich außerhalb des Bildes der Basiskamera liegen | ebd. Nachtrag 13 / L4 |
| Wie Klötze hinter dem Bild weitergeführt werden (Status 4) | ebd. Nachtrag 13 / L10; `architektur/datenvertraege.md` S3 |
| Arbeitsraum, Greifzone, Wartepose und Folgehöhe | ebd. Nachtrag 13 / L14, L15; `Safety/workspace_bounds.json`; `einrichtung-projektanwendung.md` §9 |
| Kamerabildrate, keine GPU | ebd. Nachtrag 13 / L16; `einrichtung-projektanwendung.md` §3 |
| Erster Lauf am echten Roboter, Attractor-Verstärkung, Vorhalt | ebd. Nachtrag 13 / L18 |
| Erste echte Griffe im Lauf | ebd. Nachtrag 13 / L19 |
| Schutzstopp der UR-Steuerung, Nutzlast, Beschleunigungsgrenze | ebd. Nachtrag 13 / L20; `einrichtung-projektanwendung.md` §2 |
| Greifzone gleich Arbeitsraum, Bildausschnitt der Basiskamera | ebd. Nachtrag 13 / L21 |
| Warum die Roboterkamera nicht eingebunden ist und wie das geprüft wurde | ebd. Nachtrag 13 / L22 |
| Vorhersagedeckel und Einschwingzeit in der Erreichbarkeit | ebd. Nachtrag 13 / L23 |
| Tempo, Sicherheitsfaktor und Prozesszeiten; flache Klötze | ebd. Nachtrag 13 / L24; `einrichtung-projektanwendung.md` §2, §8 |
| Wie weit sich der Greifer im Winkel des Klotzes dreht (höchstens ±45°) | ebd. Nachtrag 13 / L25 |
| Finaler Build: welche Werte Standard sind, welche von Hand in AICA gesetzt werden | ebd. Nachtrag 13 / L26; `einrichtung-projektanwendung.md` §2, §3 |
