# Dokumentenlandkarte

Wo was liegt und was verbindlich ist. Die Aufteilung folgt der Frage, **für wen**
ein Dokument geschrieben ist.

**Stand 24.09.2026:** Alle Komponenten sind gebaut und laufen in AICA; die
Inbetriebnahme am Aufbau ist im Gang, B23 ist erledigt. Stand und Reihenfolge:
`uebersicht/fahrplan-aufbau.md`.

---

## `uebersicht/` — für den Betrieb und den Überblick

Was man liest, um das System zu verstehen, zu bedienen und einzurichten.

| Dokument | Inhalt |
|---|---|
| `uebergabe.md` | **Einstieg beim Wechsel des Rechners:** Arbeitsregeln, Stand, gemessene Werte, nächste Schritte |
| `fahrplan-aufbau.md` | **Arbeit am Roboter mit Claude:** Blöcke, Rollen, Dauer, Abnahme, Sicherheitsregeln, Aufteilung auf Termine |
| `projektkontext.md` | Rahmen, Aufbau, Abgrenzungen, Arbeitsweise. **Einstieg für jede Sitzung.** |
| `systemgraph.md` | **Der Systemaufbau:** AICA-Graph, Komponenten mit Ein- und Ausgängen, Ablauf eines Griffs, Bezugssysteme, Kopplungen zwischen Parametern |
| `offene-punkte.md` | Arbeitsliste A/B/C/D — was noch zu messen, abzulesen oder festzulegen ist |
| `einrichtung-projektanwendung.md` | Was beim Anlegen der AICA-Anwendung gesetzt werden muss und warum die Defaults nicht taugen |
| `Komponentenplan Robotetris - Stand 2026-09-13.docx` | Systembeschreibung für Menschen, mit Farbcode. ⚠️ **Stand 13.09.** — vor Nachtrag 6; wird nachgezogen, wenn das System steht. Bis dahin gilt `systemgraph.md` |

## `architektur/` — die technische Grundlage

Das Warum hinter den Entscheidungen und die verbindlichen Schnittstellen.

| Dokument | Inhalt | |
|---|---|---|
| `entscheidungen.md` | Alle Architekturentscheidungen mit Begründung: Themen 1–7, Nachträge 1–11. **Neuere Nachträge gehen vor**, wo sie frühere Festlegungen berühren. Nachtrag 6: Projektvorgaben; 7–11: beim Bau der Komponenten | **normativ** |
| `datenvertraege.md` | Signalspezifikation S1–S10: Felder, Strides, Einheiten | **normativ** |
| `robot-cam-befunde.md` | Szene, Materialphysik und verworfene Wege der Roboterkamera | |
| `vorgaengerprojekt-abgleich.md` | Abgleich gegen das Vorgängerarchiv `UR10_Pick_ws`: was von dort beantwortet ist, was nicht, fünf Fallen | |
| `specs/` | Umsetzungsvorlage je Komponente, jetzt mit Stand „umgesetzt"; werden nach dem ersten Lauf am Aufbau gelöscht | abgeleitet |

## `archiv/` — historisch, nicht mehr pflegen

| Dokument | Warum abgelegt |
|---|---|
| `2026-09-05-konzeptreview-komponentenplan.md` | Die ursprüngliche Analyse mit 61 Befunden. Alle sind in den Themen 1–7 abgearbeitet; das Dokument wird nur noch über Befundnummern zitiert. |
| `Komponentenplan Robotetris.docx` | Der unveränderte Originalstand des Komponentenplans |
| `2026-06-01-robotiq-gripper-component-design.md` | Beschreibt eine fertig gebaute, laufende Komponente. Liegt in `.gitignore`, fehlt also auf einem frisch gepullten Rechner |

> `ARCHITECTURE.md` liegt im Projektroot, nicht hier. Es ist die **verbindliche
> AICA-Regelsammlung** und gilt über allem in diesem Ordner.

---

## Zwei Regeln, damit es übersichtlich bleibt

**Vorrang.** Bei Widerspruch gelten `entscheidungen.md` und `datenvertraege.md`.
Alles andere ist daraus abgeleitet. Änderungen an einem Signal werden **zuerst**
in `datenvertraege.md` gemacht, dann in `contracts.py` nachgezogen — nie umgekehrt.

**Specs haben ein Ende.** Eine Komponenten-Spec ist eine Bauanleitung, keine
Dokumentation. Sobald die Komponente gebaut und ihre JSON-Beschreibung geschrieben
ist, **wird die Spec gelöscht**: Code und JSON tragen den Vertrag dann selbst, und
eine dritte Stelle wäre nur Pflegeaufwand. Genau daran hat sich gezeigt, wie teuer
Mehrfachpflege ist — Nachtrag 3 in `entscheidungen.md` musste eine einzige
Erkenntnis in fünf Dateien schreiben.

---

## Was wo entschieden wurde — Kurzregister

| Frage | steht in |
|---|---|
| Warum eine Zielpose und kein Twist? | `architektur/entscheidungen.md`, Thema 1 |
| Woher kommt der Zeitstempel, wie wird Latenz behandelt? | ebd., Thema 2 |
| Welche Kalibrierung tritt wo ein? | ebd., Thema 3 |
| Wer besitzt welche Information? | ebd., Thema 4 |
| Wie sehen die Signale aus? | `architektur/datenvertraege.md` |
| Zustandsautomat, Greifhöhe, Orientierung | `architektur/entscheidungen.md`, Thema 6 |
| Sicherheitsgate, Arbeitsraum, Singularitäten | ebd., Thema 7 |
| Die fünf Befunde aus der Code-Durchsicht | ebd., Nachtrag 3 |
| Warum die Roboterkamera nicht den Loch-Trick nutzt | `architektur/robot-cam-befunde.md` §2 |
| Was das Vorgängerprojekt beantwortet — und was nicht | `architektur/vorgaengerprojekt-abgleich.md` |
| Welche AICA-Parameter beim Anlegen gesetzt werden müssen | `uebersicht/einrichtung-projektanwendung.md` |
| Die gemessenen Werte des Aufbaus (Bandhöhe, Greifhöhe, Ablagepose) | ebd. Abschnitt 8; Herleitung in `architektur/entscheidungen.md` Nachtrag 5 |
| Warum Höhen Flansch- und nicht TCP-Maße sind | `architektur/entscheidungen.md` Nachtrag 5 / M8 |
| **Wie das System aufgebaut ist und welche Parameter zusammenpassen müssen** | `uebersicht/systemgraph.md` |
| **Was am Aufbau als Nächstes zu tun ist, in welcher Reihenfolge** | `uebersicht/fahrplan-aufbau.md` („Stand nach Termin B“) |
| **Die offiziellen Projektziele** | `architektur/entscheidungen.md` Nachtrag 6 / Z1; `uebersicht/projektkontext.md` §1 |
| **Wie die Geschwindigkeit geschätzt wird** (Ziel 3) | ebd. Nachtrag 6 / Z2–Z4 |
| Warum der Werkzeugversatz 0,235 m ist, nicht 0,215 | ebd. Nachtrag 6 / Z7 |
| Warum der Vorhalt eine Zeit ist | ebd. Nachtrag 6 / Z6 |
| Warum das Einschwingkriterium zwei Halbfenster vergleicht | ebd. Nachtrag 6 / Z9 |
| Ab wann kein Griff mehr beginnen darf (Greifebene) | ebd. Nachtrag 6 / Z11 |
| Wie der `priority_handler` Anfahrweg und Greiferbreite rechnet | ebd. Nachtrag 7 / H1, H2 |
| Was „Koordinate entlang der Bandrichtung“ in S4 genau heißt | ebd. Nachtrag 7 / H4; `contracts.along_belt` |
| Warum S10 ein Feld `present` hat und wann Einträge verfallen | ebd. Nachtrag 7 / T1 |
| Warum Basiskamera und Roboter das Band an verschiedenen Stellen sahen (180°) | ebd. Nachtrag 8 / F1, Nachtrag 12 / K5; `uebersicht/offene-punkte.md` B23 |
| Woher die Kalibrierung der Basiskamera stammt, warum sie senkrecht schaut und was die Parallaxe war | ebd. Nachtrag 13 / L6 |
| Warum `base_cam` nur ~7 Messungen/s schafft (ein Python-Prozess, Warteschlangen) | ebd. Nachtrag 13 / L2 |
| Warum Greifzone und Wartebereich außerhalb des Bildes der Basiskamera liegen | ebd. Nachtrag 13 / L4 |
| Wie Klötze hinter dem Bild der Basiskamera weitergeführt werden (Status 4) | ebd. Nachtrag 13 / L10; `architektur/datenvertraege.md` S3 |
| Woher die Hand-Auge-Kalibrierung der Roboterkamera stammt | ebd. Nachtrag 13 / L11 |
| Arbeitsraum, Greifzone, Wartepose und Folgehöhe | ebd. Nachtrag 13 / L14, L15; `Safety/workspace_bounds.json`; `uebersicht/einrichtung-projektanwendung.md` §9 |
| Erster Lauf am echten Roboter, Attractor-Verstärkung, Vorhalt, Einbrüche der 500-Hz-Schleife | ebd. Nachtrag 13 / L18; `uebersicht/fahrplan-aufbau.md` „Stand nach Termin C“ |
| Kamerabildrate, Infrarot der Roboterkamera, keine GPU | ebd. Nachtrag 13 / L16; `uebersicht/einrichtung-projektanwendung.md` §3 |
| Warum der Follower anfangs keine Signale hatte | ebd. Nachtrag 13 / L17 |
| Erste echte Griffe im Lauf (Messwerte, was bestätigt ist) | ebd. Nachtrag 13 / L19 |
| Schutzstopp der UR-Steuerung, Nutzlast, Beschleunigungsgrenze | ebd. Nachtrag 13 / L20; `uebersicht/einrichtung-projektanwendung.md` §2 |
| Warum Klötze am Bandrand fehlten; Greifzone und Bildausschnitt | ebd. Nachtrag 13 / L21 |
| Wie der Follower startet (Abbruchpfad) und warum Arbeitsraum und Beobachtungspose keine Defaults haben | ebd. Nachtrag 8 / F3, F4 |
| Wie `lead_time_s` eingemessen wird (B4) und was `outcome = 4` heißt | ebd. Nachtrag 9 / G7, G2 |
| Wie die Roboterkamera wirkt und wie der Greifzyklus abläuft, warum die Freihöhe 0,49 m ist | ebd. Nachtrag 10 / J1–J8 |
| Warum die Anzeige ASCII schreibt und wann sie „veraltet" zeigt | ebd. Nachtrag 11 / V1–V3 |
