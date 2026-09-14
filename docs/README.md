# Dokumentenlandkarte

Wo was liegt und was verbindlich ist. Die Aufteilung folgt der Frage, **für wen**
ein Dokument geschrieben ist.

---

## `uebersicht/` — für den Betrieb und den Überblick

Was man liest, um das System zu verstehen, zu bedienen und einzurichten.

| Dokument | Inhalt |
|---|---|
| `projektkontext.md` | Rahmen, Aufbau, Abgrenzungen, Arbeitsweise. **Einstieg für jede Sitzung.** |
| `systemgraph.md` | Der AICA-Graph: welche Komponente hängt an welcher |
| `offene-punkte.md` | Arbeitsliste A/B/C/D — was noch zu messen, abzulesen oder festzulegen ist |
| `einrichtung-projektanwendung.md` | Was beim Anlegen der AICA-Anwendung gesetzt werden muss und warum die Defaults nicht taugen |
| `Komponentenplan Robotetris - Stand 2026-09-13.docx` | Systembeschreibung für Menschen, mit Farbcode |

## `architektur/` — die technische Grundlage

Das Warum hinter den Entscheidungen und die verbindlichen Schnittstellen.

| Dokument | Inhalt | |
|---|---|---|
| `entscheidungen.md` | Alle Architekturentscheidungen mit Begründung (Themen 1–7, Nachträge 1–3) | **normativ** |
| `datenvertraege.md` | Signalspezifikation S1–S10: Felder, Strides, Einheiten | **normativ** |
| `robot-cam-befunde.md` | Szene, Materialphysik und verworfene Wege der Roboterkamera | |
| `vorgaengerprojekt-abgleich.md` | Abgleich gegen das Vorgängerarchiv `UR10_Pick_ws`: was von dort beantwortet ist, was nicht, fünf Fallen | |
| `specs/` | Umsetzungsvorlage je Komponente | abgeleitet |

## `archiv/` — historisch, nicht mehr pflegen

| Dokument | Warum abgelegt |
|---|---|
| `2026-09-05-konzeptreview-komponentenplan.md` | Die ursprüngliche Analyse mit 61 Befunden. Alle sind in den Themen 1–7 abgearbeitet; das Dokument wird nur noch über Befundnummern zitiert. |
| `Komponentenplan Robotetris.docx` | Der unveränderte Originalstand des Komponentenplans |
| `2026-06-01-robotiq-gripper-component-design.md` | Beschreibt eine fertig gebaute, laufende Komponente |

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
