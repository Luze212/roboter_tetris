# Quellenregister Projektbericht Robotertetris

Diese Datei ordnet jede im Markdown-Entwurf verwendete Quelle einem Abschnitt
und einer konkreten Aussage zu. Die bisherigen Kennungen `Q01` bis `Q10`
bleiben als Bestandskennungen erhalten. Die Spalte „Stabile Kennung“ verweist
auf die kollisionsfreie Einzeldatei unter `referenzen/quellen/`. Neue Quellen
werden dort zuerst angelegt und erst bei der zentralen Integration hier ergänzt.

| Kennung | Stabile Kennung | Status | Abschnitt | Verwendungszweck | Quelle und Link | Geprüft |
|---|---|---|---|---|---|---|
| Q01 | `src-projekt-inversetetris-aufgabenstellung` | verwendet | 1.1, 1.2, 1.3 | Ausgangslage, bildbasierte Schätzung, mehrere Objekte, verschiedene Bandgeschwindigkeiten, Greifen aus der Bewegung und Projektgrenzen | Hochschule Karlsruhe: *Aufgabenstellung Projektarbeit, Inverses „Tetris“ für Roboter. „On the Fly“-Picking bei verschiedenen Geschwindigkeiten des Förderbands*, SS 2026. Lokale Quelle: `C:\Users\tobiu\OneDrive\Dokumente\04M_RKIM_Semester4\FuE_Robotertetris_SoSe26\26ss_BH_Roboter_InverseTetris_Projektarbeit.pdf` | vollständig gelesen und visuell geprüft am 24.09.2026 |
| Q02 | `src-universalrobots-ur10e-technische-daten` | verwendet | 2.1.2 | Technische Grunddaten des UR10e, insbesondere Gelenkzahl, Reichweite, Nutzlast und Wiederholgenauigkeit | Universal Robots: [UR10e Technical Specification](https://www.universal-robots.com/manuals/EN/TechSheets/UR10e_techsheet_pdf_online/UR10e_techsheet_en.pdf) | Herstellerquelle, Link am 24.09.2026 geprüft |
| Q03 | `src-robotiq-2f140-spezifikation` | verwendet | 2.1.3 | Bauart und Herstellerabmessungen des Robotiq 2F-140 sowie Abbildung A11 | Robotiq: [Specifications, 2F-85 and 2F-140 Instruction Manual](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190206.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm) | Herstellerquelle, Link am 24.09.2026 geprüft |
| Q04 | `src-robotiq-2f140-handbuch` | verwendet | 2.1.3 | Unteraktuiertes Fingerdesign, gekrümmte Schließbewegung, Kraftregelung, Kontaktmeldung und Nachgreiffunktion des Robotiq 2F-140 | Robotiq: [2F-85 & 2F-140 Instruction Manual](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_CB-Series_PDF_20190206.pdf) | Herstellerquelle, Link am 24.09.2026 geprüft |
| Q05 | `src-universalrobots-ur10e-sicherheitsfunktionen` | verwendet | 2.1.2 | Konfigurierbare Sicherheitsgrenzen des UR10e für Geschwindigkeit, Kraft, Impuls und Leistung sowie Reaktion bei Sicherheitsverletzungen | Universal Robots: [Safety Functions Table, UR10e](https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-usr-man/complianceUR10e/safetyFunctionsAndinterfaces/safety_functions_table1.htm) | Herstellerquelle, Link am 24.09.2026 geprüft |
| Q06 | `src-universalrobots-ur10e-risikobeurteilung` | verwendet | 2.1.2 | Anforderungen an die anwendungsbezogene Risikobeurteilung und Abgrenzung der Endeffektor-Sicherheit | Universal Robots: [Safety-related Functions and Interfaces, UR10e](https://www.universal-robots.com/manuals/EN/HTML/SW10_6/Content/prod-usr-man/hardware/arm_e-Series/UR10e/H_g5_sections/safetyFunctionsAndinterfaces/safety_related_functions_en_g5.htm) | Herstellerquelle, Link am 24.09.2026 geprüft |
| Q07 | `src-intel-realsense-l515-spezifikation` | verwendet | 2.1.1 | Produktbezeichnung der Basiskamera, LiDAR-Tiefentechnik und RGB-Sensor der Intel RealSense L515 | Intel: [Intel RealSense LiDAR Camera L515, Specifications](https://www.intel.com/content/www/us/en/products/sku/201775/intel-realsense-lidar-camera-l515/specifications.html) | Herstellerquelle, Link vom Nutzer am 26.09.2026 bestätigt |
| Q08 | `src-realsense-l515-datenblatt` | verwendet | 2.1.1 | Abtastender Infrarotlaser und Zusammenspiel von RGB- und LiDAR-Tiefenmessung der Intel RealSense L515 | RealSense: [Intel RealSense LiDAR Camera L515 Datasheet, Rev. 003](https://realsenseai.com/wp-content/uploads/2025/06/Intel_RealSense_LiDAR_L515_Datasheet_Rev003.pdf) | Herstellerquelle, Link vom Nutzer am 26.09.2026 bestätigt |
| Q09 | `src-intel-realsense-d435i-spezifikation` | verwendet | 2.1.1 | Produktbezeichnung der Roboterkamera, aktive Stereotiefentechnik und RGB-Sensor der Intel RealSense D435i | Intel: [Intel RealSense Depth Camera D435i, Specifications](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html) | Herstellerquelle, Link vom Nutzer am 26.09.2026 bestätigt |
| Q10 | `src-realsense-d400-datenblatt` | verwendet | 2.1.1 | Infrarot-Stereosystem, Infrarotprojektor und integrierte IMU der Intel RealSense D435i | RealSense: [Intel RealSense D400 Series Datasheet, September 2023](https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf) | Herstellerquelle, Link vom Nutzer am 26.09.2026 bestätigt |

| — | `src-projekt-foerderband-kloetze` | verwendet | 2.1.4 | Vorhandenes Förderband, gemessene Geschwindigkeit sowie Abmessungen, Farben und Oberflächen der verwendeten Klötze | Projektgruppe Robotertetris: Angaben zum Versuchsaufbau, bestätigt im Arbeitsgespräch am 30.09.2026; ergänzt durch den lokalen Komponentenplan vom 28.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-basiskamera-konfiguration` | verwendet | 2.1.5 | Betriebsprofile, Sichtbereich und Montagesituation der Intel RealSense L515 im finalen Aufbau | Projektgruppe Robotertetris: Dokumentation der Projektanwendung und Komponentenplan des finalen Builds, Stand 28.09.2026; ergänzt durch Angaben der Projektgruppe im Arbeitsgespräch am 30.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-roboterkamera-einbindung` | verwendet | 2.1.6 | Mechanische Anordnung der Intel RealSense D435i und ihr Status im finalen Greifablauf | Projektgruppe Robotertetris: Komponentenplan des finalen Builds, Stand 28.09.2026; ergänzt durch Angaben der Projektgruppe im Arbeitsgespräch am 30.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-bezugssysteme` | verwendet | 2.2.1 | Bezugssystem `world`, historisches `conveyor_frame` und Flanschpose `ur_tool0` | Projektgruppe Robotertetris: Systemgraph, Architekturentscheidungen und Dokumentation der Projektanwendung, Stand 28.09.2026; ergänzt durch Angaben der Projektgruppe im Arbeitsgespräch am 30.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-greifgeometrie` | verwendet | 2.2.2 | Gemessene Abstände vom Flansch zur Backenspitze und zum Griffpunkt des geschlossenen Greifers | Projektgruppe Robotertetris: Messwerte zur Greifgeometrie vom 15.09.2026, dokumentiert in Architekturentscheidungen und Datenverträgen; ergänzt durch Angaben der Projektgruppe im Arbeitsgespräch am 30.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-arbeitsraum-greifzone` | verwendet | 2.2.3 | Im Projekt hinterlegte Arbeitsraumgrenzen, abgegrenzte Greifzone und Transferhöhe des Flansches | Projektgruppe Robotertetris: Arbeitsraummessung am Aufbau vom 23.09.2026 und aktuelle Parameterkonfiguration, Stand 28.09.2026; ergänzt durch Angaben der Projektgruppe im Arbeitsgespräch am 30.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-aica-system-uebersicht` | verwendet | 2.3.1 | Einordnung von AICA Core und AICA Studio sowie Grundlage in ROS 2 | AICA: [Getting Started](https://docs.aica.tech/) und [Built on ROS 2](https://docs.aica.tech/docs/concepts/ros-concepts/built-on-ros/) | Links durch Projektgruppe am 30.09.2026 bestätigt |

| — | `src-aica-komponenten` | verwendet | 2.3.1, 2.3.2 | Komponentenmodell, periodische Ausführung, Parameter und Predicates | AICA: [Components](https://docs.aica.tech/docs/concepts/building-blocks/components/) | Link durch Projektgruppe am 30.09.2026 bestätigt |

| — | `src-aica-signale` | verwendet | 2.3.3 | Periodische Datenübertragung über AICA-Signale und Anbindung an ROS-2-Topics | AICA: [Signals](https://docs.aica.tech/docs/concepts/building-blocks/signals/) | Link durch Projektgruppe am 30.09.2026 bestätigt |

| — | `src-projekt-softwareumgebung` | verwendet | 2.3.1, 2.3.2, 2.3.3 | Verwendete Versionen, Komponentenstruktur und projektspezifische Datenverträge | Projektgruppe Robotertetris: Dokumentation der Projektanwendung und Systemgraph, Stand 28.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

## Regeln für neue Einträge

- Jede neue Quelle erhält zuerst eine eindeutige stabile Kennung und eine
  Einzeldatei unter `referenzen/quellen/`. Fortlaufende Q-Kennungen vergibt
  nur die Integrationsperson.
- Der Eintrag nennt immer Abschnitt, konkrete Verwendung, vollständige
  Quellenangabe und einen direkt prüfbaren Link.
- Nach der Bestätigung im Chat wird eine Quelle als `freigegeben` markiert.
  Als `verwendet` gilt sie erst, wenn sie im abgestimmten Text steht.
- Eigene Messwerte und Beobachtungen erhalten keinen Literaturverweis. Ihre
  Versuchsbedingungen und die zugehörigen Rohdaten werden stattdessen im
  Bericht oder im Datenordner nachvollziehbar festgehalten.
