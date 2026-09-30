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

| — | `src-projekt-greifablauf` | verwendet | 3.1 bis 3.6 | Implementierte Funktionskette, Zustandsautomat, Bewegungsumsetzung und Greiferrückmeldungen | Projektgruppe Robotertetris: Systemgraph, Architekturentscheidungen, Datenverträge und Implementierung des finalen Builds, Stand 28.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-kalibrierung` | verwendet | 4.2, 4.4 | Aufbau, Ablauf, Gütegrenzen und Messwerte der automatischen Basiskamera-Kalibrierung sowie ihre Validierung | Projektgruppe Robotertetris: Architekturentscheidungen und Messdaten der Basiskamera-Kalibrierung vom 25. und 28.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-projekt-inbetriebnahme-optimierung` | verwendet | 5.1 bis 5.3 | Inbetriebnahme, Raten, Latenzen, Rechenlast sowie eingestellte Parameter des finalen Regelpfads und der Basiskamera | Projektgruppe Robotertetris: Architekturentscheidungen, Systemgraph und Konfiguration des finalen Builds, Stand 28.09.2026 | durch Projektgruppe bestätigt am 30.09.2026 |

| — | `src-kalibr-aprilgrid` | verwendet | 4.2 | Aufbau eines AprilGrid-Kalibrierboards aus AprilTags | Autonomous Systems Lab, ETH Zürich: [Kalibr Calibration Targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-wang-apriltag2-2016` | verwendet | 4.2 | Eindeutige Kennung und Detektion von AprilTags | J. Wang, E. Olson: [AprilTag 2: Efficient and robust fiducial detection](https://doi.org/10.1109/IROS.2016.7759617), 2016 | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-tsai-handauge-1989` | verwendet | 4.2 | Verfahren nach Tsai und Lenz als Startwert der Hand-Auge-Kalibrierung | R. Y. Tsai, R. K. Lenz: [A new technique for fully autonomous and efficient 3D robotics hand/eye calibration](https://doi.org/10.1109/70.34770), 1989 | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-park-handauge-1994` | verwendet | 4.2 | Verfahren nach Park und Martin als Startwert der Hand-Auge-Kalibrierung | F. C. Park, B. J. Martin: [Robot sensor calibration: solving AX = XB on the Euclidean group](https://doi.org/10.1109/70.326576), 1994 | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-opencv-handeye` | verwendet | 4.2 | OpenCV-Implementierung der Hand-Auge-Kalibrierung und Lageschätzung | OpenCV: [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-mathworks-handeye-kalibrierung` | verwendet | 4.1, 4.3 | Unterschied zwischen Eye-in-Hand- und Eye-to-Hand-Kalibrierung | MathWorks: [What Is Robot Hand-Eye Calibration?](https://de.mathworks.com/help/vision/ug/what-is-robot-hand-eye-calibration.html) | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-opencv-charuco-aufbau` | verwendet | 4.3 | Aufbau eines ChArUco-Boards aus Schachbrettmuster und ArUco-Markern sowie eindeutige Zuordnung der inneren Schachbrettecken | OpenCV: [Create Calibration Pattern](https://docs.opencv.org/5.0/tutorials/calib3d/camera_calibration_pattern/camera_calibration_pattern.html) | durch Projektgruppe geprüft am 30.09.2026 |

| — | `src-opencv-charuco-erkennung` | verwendet | 4.3 | Verdeckungsrobuste Markererkennung und präzise Schachbrettecken von ChArUco-Boards für Kalibrieranwendungen | OpenCV: [Detection of ChArUco Boards](https://docs.opencv.org/4.12.0/df/d4a/tutorial_charuco_detection.html) | durch Projektgruppe geprüft am 30.09.2026 |

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
