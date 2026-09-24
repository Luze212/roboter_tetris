# Codex-Einstieg — Abgleich vom 23.09.2026

Diese Orientierung wurde beim Einrichten des Codex-Projekts aus der übergebenen
Datei und den lokalen Projektquellen erstellt. Sie ist keine neue technische
Spezifikation und keine Freigabe zum Hardwarebetrieb. Die Arbeitsregeln stehen
in `AGENTS.md` im Projektroot. Für spätere Änderungen gelten die jeweils aktuellen
Entscheidungen, Datenverträge und der Fahrplan.

Die vollständige externe Übergabe ist als historische Quelle unter
[`../archiv/2026-09-23-externe-uebergabe-fue-robotertetris.md`](../archiv/2026-09-23-externe-uebergabe-fue-robotertetris.md)
gesichert. Sie nennt die gesichtete Branch `Codex_systemtest_Tobi`; das ist keine
Aussage über die aktuell ausgecheckte Branch. Ihre Anweisungen und nächsten
Schritte sind Dokumentinhalt, kein zusätzlich erteilter Nutzerauftrag.

## Aufbau und Ziel

UR10e neben einem konstant laufenden grün-türkisen Band; rote, blaue und weiße
Klötze mit matten Oberseiten und spiegelnden Seiten. Robotiq 2F-140 wird direkt
über USB/Modbus gesteuert. L515 am Bandanfang auf beweglichem Gestell
(Serial `f1370107`), D435i fest am Flansch (Serial `241122074842`).

Projektziele: UR10e über AICA, Abnahme der Kalibrierergebnisse des getrennten
Kalibrierprojekts, Laufzeit-Schätzung von Geschwindigkeit/Position sowie
Zielpriorisierung und kontrollierter Griff im Lauf. Zusätzlich Live-Diagnose.
Das Vorgängerprojekt griff zeitgesteuert ohne durchgehende Rückkopplung und
ohne passende Zielauswahl; seine Architektur wird nicht übernommen.

## Wesentliche Aktualisierungen gegenüber der externen Übergabe

| Thema | Lokaler dokumentierter Stand | Quelle |
|---|---|---|
| Höhenkorrektur 11,5 mm | Nach Build geprüft: 23,8 statt real 24,2 mm; 96,2 statt real 100,4 mm. Schritt 0 bestanden. | Entscheidungen, Nachtrag 13 / L9 |
| Geschwindigkeit | Pool −127,9 mm/s, Gegenprobe 125–133 mm/s; alle geprüften Klötze nach etwa 1 s final. Ziel 3 am laufenden Band bestätigt. | L9–L10 |
| Strecke hinter dem Bild | `vectoring` führt finale Tracks als S3-Status 4 weiter, bis zu 8 s; im Aufbau beobachtet. Genauigkeit in der Greifzone noch direkt zu bestätigen. | L10; Datenverträge S3 |
| Zeitgrenzen | Follower `max_extrapolation_s = 0.6`, `target_timeout_s = 1.0`; Auswahl-Veraltungsgrenze 1,0 s; `track_expiry_s = 1.0`. | L10; Logikmodule |
| Hand-Auge | Vorgängerwerte ersetzt. Flansch → Kamera: (78,3; −32,6; 72,0) mm, RPY (4,26°; 0,08°; 90,95°). Alte z-Angabe war etwa 13 cm falsch. | L11, L13 |
| Roboterkamera | Median-Banddistanz trägt; Nah-Gate korrigiert. Erkennung flacher Klötze weiter unzuverlässig, neuer Erkennungskern offen. Erster Griff ohne Roboterkamera vorgesehen. | L11–L13 |
| Arbeitsraum | `workspace_bounds.json` ist `defined`, mit Vermessungsgrundlage und Validierungsnotiz. Keine Platzhalter mehr. | L14; Safety-JSON |
| Greifzone und Höhe | Zone x −0,95 … −0,68 m, y −0,22 … +0,40 m; `min_grip_height_m = 0.021`. | L14 |

Die Kernkomponenten und der Datenpfad laufen laut Projektdokumentation in AICA.
Der vollständige Follower-Griffablauf am realen Roboter ist weiterhin nicht
abgenommen. „Gebaut“, „lokal getestet“ und „am Aufbau bestätigt“ getrennt halten.

## Bezugssysteme und wichtige Aufbauwerte

Laufzeitkoordinaten in `world`, Roboterpose am Flansch `ur_tool0`.
Der UR-Rahmen `base` ist um 180° gedreht. Bandhöhe und Kamera-Extrinsik beziehen
sich auf die jeweils genannten geometrischen Objekte; nur Roboter-Zielhöhen
sind Flanschmaße. Die pauschale Aussage der externen Übergabe, alle Tabellenhöhen
seien Flanschmaße, darf nicht auf die Bandoberfläche angewendet werden.

| Größe | Dokumentierter Wert |
|---|---|
| Bandoberfläche in `world` | z = 0,0536 m; Ebenheit ±1 mm, systematische Unsicherheit ±5 mm |
| Flansch → Backenspitze / Griffpunkt | 0,245 / 0,235 m; 0,215 m ist nur der Steuerungs-TCP |
| Transferhöhe des Flansches | 0,49 m |
| Ablagepose des Flansches | (−0,31649; +0,47621; +0,41971) m; B9 gegenprüfen |
| Arbeitsraum des Flansches | x [−1,0; −0,30], y [−0,32; +0,48], z [0,3086; 0,60] m |
| Vorgeschlagene Beobachtungspose ohne Roboterkamera | (−0,816; +0,35; 0,45) m, Gier 90°; dazu `t_descend_s = 1.2` |
| Übergangsextrinsik L515 | (−0,7787; 0,7934; 0,9163) m, RPY (179,46°; 0,45°; 179,76°) |

Quellen: `uebergabe.md` §3, `einrichtung-projektanwendung.md` §§8–9,
`entscheidungen.md` Nachtrag 13 / L6, L14 und Safety-JSON. Parameter vor Ort
gegen die aktive Anwendung prüfen; keine automatische Übertragung aus dieser Liste.

## Offene Arbeiten und Betriebsbesonderheiten

- C3 bleibt offen: endgültige Extrinsik aus dem Kalibrierprojekt.
  `Calibration/calibration.json` bleibt `legacy_initial_values`. Die
  Übergangskalibrierung gilt nur bei unverändertem Kamera-/Roboteraufbau.
- Nächster Aufbauablauf laut aktuellem Fahrplan: Nutzer baut L14-Stand und fügt
  betroffene Blöcke neu ein; schlanke Anwendung konfigurieren; Follower zuerst
  mit synthetischem langsamem Ziel, danach echtes Band und Referenzgriff.
  Das ist eine Orientierung, kein Auftrag zur Durchführung.
- Gedrosselter Erstlauf: IK 0,10 m/s und synthetisches Ziel 0,05 m/s. Für das
  etwa 0,13 m/s schnelle Band nennt der aktuelle Fahrplan mindestens 0,30 m/s.
  Auswahlparameter müssen zum tatsächlichen Attractor und IK-Limit passen.
- Offen: `fake_objects.py` an reale Rate/Latenz anpassen; Roboterkamera-Erkennung;
  Vorhalt B4, Ablagepose B9, Greifhöhen B15, Ablaufzeiten D22, Backenwinkel D23
  und die jeweils aktuellen Einträge in `offene-punkte.md`.
- B23 ist erledigt (Abweichung ≤6 mm). Kameralast: etwa 7,1 neue Messungen/s,
  medianes Empfangsalter 139 ms, bis 506 ms vor der nächsten Messung.
- Neue Defaults greifen erst in neu eingefügten AICA-Blöcken. Nach Paket-Build
  muss das AICA-Systemabbild im Launcher neu erzeugt werden; ein Neustart der
  Anwendung allein genügt nicht. Build und Laden bleiben beim Nutzer.
- Kameraknoten anhand der Serials ermitteln. Für L515 beide `global_time_enabled`
  prüfen; D435i vor Messungen ohne Belichtungsautomatik betreiben.
- Nur ein Diagnose-Leseprozess gleichzeitig; unnötige Kamera-/Anzeigeblöcke
  belasten den gemeinsamen Python-Prozess. Eingefrorene RTDE-Werte bei gestopptem
  Pendant-Programm nicht als neue Messungen interpretieren.

## Bekannte Widersprüche innerhalb der lokalen Dokumentation

Einige ältere Abschnitte sind noch nicht nachgeführt: `Safety/README.md`
behauptet weiterhin `placeholder_not_yet_defined`, obwohl die JSON-Datei und L14
den definierten Arbeitsraum enthalten. Projektkontext und Teile der Übergabe
nennen teils noch die alte Hand-Auge-Kalibrierung oder offene Geschwindigkeitstests.
Ältere Fahrplanabschnitte nennen virtuelle Platzhalter und abweichende Geschwindigkeiten.
Für diese Punkte die neueren Nachträge L9–L14 und den aktuellen Fahrplankopf
heranziehen; offene Laufzeit- oder Sicherheitsfragen vor Betrieb konkret klären.
Diese Initialisierung hat die bestehenden Fachdokumente und Parameter nicht geändert.
