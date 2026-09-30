# Abbildungsplan

Die Dateien in diesem Ordner werden erst beim späteren Word-Export in das
Dokument übernommen. Die Zuordnung wird während der Textabstimmung ergänzt.

| Stabile Kennung | Datei | Vorgesehene Verwendung | Vorläufige Beschriftung | Verweis im Text |
|---|---|---|---|---|
| `fig-aufbau-gesamtansicht-1` | `WhatsApp Image 2026-09-24 at 19.31.05 (3).jpeg` | Gesamtansicht in 2.1.1 | Versuchsaufbau mit Förderband, UR10e und Ablagekiste. | „Die räumliche Anordnung des realen Aufbaus zeigen die Abbildungen `fig-aufbau-gesamtansicht-1` und `fig-aufbau-gesamtansicht-2`.“ |
| `fig-aufbau-basiskamera` | `WhatsApp Image 2026-09-24 at 19.31.05 (1).jpeg` | Detailansicht in 2.1.5 | Über dem Bandanfang montierte Basiskamera. | Beim Schreiben von 2.1.5 vor dem Bild ergänzen. |
| `fig-greifer-roboterkamera` | `WhatsApp Image 2026-09-24 at 19.31.05 (2).jpeg` | Detailansicht in 2.1.3 oder 2.1.6 | Greifer mit montierter Roboterkamera und Fingeraufsätzen. | Beim Schreiben des gewählten Abschnitts vor dem Bild ergänzen. |
| `fig-aufbau-gesamtansicht-2` | `WhatsApp Image 2026-09-24 at 19.31.05.jpeg` | Ergänzende Gesamtansicht in 2.1.1 | Arbeitsbereich des Roboters über dem Förderband. | „Die räumliche Anordnung des realen Aufbaus zeigen die Abbildungen `fig-aufbau-gesamtansicht-1` und `fig-aufbau-gesamtansicht-2`.“ |
| `fig-systemaufbau-draufsicht` | `systemskizzen_layout_v03.pdf`, Seite 1 | Schematische Abbildung in 2.1.1 | Draufsicht mit Förderband, Basiskamera, Ablagekiste, Arbeitsbereich und den Bezugssystemen `conveyor_frame` sowie `world`. | „Die Lage von Förderband, Kamera, Roboter, Ablagekiste und Arbeitsbereich wird in Abbildung `fig-systemaufbau-draufsicht` schematisch verdeutlicht.“ |
| `fig-systemaufbau-seitenansicht` | `systemskizzen_layout_v03.pdf`, Seite 2 | Schematische Abbildung in 2.1.1 | Seitenansicht mit Basiskamera sowie Arbeitsraum-, Folge- und Transferhöhen im Bezugssystem `world`. | „Die zugehörigen Höhen und die Position der Basiskamera über dem Förderband sind in Abbildung `fig-systemaufbau-seitenansicht` dargestellt.“ |
| `fig-greifer-robotiq-2f140-abmessungen` | `robotiq_2f140_abmessungen_geoeffnet.png` | Abbildung in 2.1.3 | Herstellerabmessungen des geöffneten Robotiq-2F-140-Greifers. | „Die Herstellerabmessungen des geöffneten Greifers zeigt Abbildung `fig-greifer-robotiq-2f140-abmessungen`." |
| `fig-koord-systeme` | `fig-koord-systeme.png` | Schematische Abbildung in 2.2.1 | Seitenansicht des Roboters mit Bezugssystem `world`, Flansch `ur_tool0` und Griffpunkt. | „Die räumliche Zuordnung von Roboterbasis, Flansch `ur_tool0` und Griffpunkt zeigt Abbildung `fig-koord-systeme`." |
| `fig-follower-zustandsdiagramm` | `fig-follower-zustandsdiagramm.png` | Zustandsdiagramm in 3.4.1 | Zustandsautomat des `object_follower`. Die gestrichelte Umrandung fasst die Zustände zusammen, aus denen ein Versuch abgebrochen werden kann, bevor der Greifer den Klotz hält. | „Abbildung `fig-follower-zustandsdiagramm` zeigt die Zustände und ihre Übergänge.“ |
| `fig-kalibrierung-board-greifer` | `fig-kalibrierung-board-greifer.png` | Aufnahme in 4.2 | AprilGrid-Kalibrierboard im Greifer in der Startpose, aufgenommen von der Basiskamera (Graubild der Farbkamera, `0,53 m` Abstand). | „Abbildung `fig-kalibrierung-board-greifer` zeigt diese Anordnung aus Sicht der Kamera.“ |
| `fig-orbit-trajektorie` | `fig-orbit-trajektorie.jpg` | Schematische Abbildung in 4.3 | Orbit-Trajektorie der Eye-in-Hand-Kalibrierung. Draufsicht: 9 Wegpunkte (Startpose 0 im Zentrum, Wegpunkte 1–8 auf dem Kreisring mit r = 50 mm, 45°-Abstände). Seitenansicht: Die Kamera zeigt an jedem Wegpunkt auf das Board-Zentrum. (KI generiert.) | „Abbildung `fig-orbit-trajektorie` zeigt die Trajektorie mit den Standardwerten von 9 Wegpunkten und einem Kreisradius von 50 mm in Drauf- und Seitenansicht.“ |
| `fig-regelpfad-aica` | `fig-regelpfad-aica.png` | Anhang, Verweis in 5.1 | Gesamtansicht der AICA-Anwendung mit Komponenten und Signalverbindungen des finalen Systems. | „Eine Gesamtansicht der implementierten Komponenten und ihrer in AICA verdrahteten Signale enthält Anhangabbildung `fig-regelpfad-aica`." |
| `fig-aica-bildverarbeitung` | `fig-aica-bildverarbeitung.png` | Anhang, Verweis in 5.1 | AICA-Ausschnitt der Bildverarbeitung mit den Verbindungen von Kameraeingängen über Base Kamera zu Vectoring. | Siehe Sammelverweis in 5.1. |
| `fig-aica-zielauswahl-diagnose` | `fig-aica-zielauswahl-diagnose.png` | Anhang, Verweis in 5.1 | AICA-Ausschnitt der Zielauswahl und Diagnose mit Priority Handler, Data Tracker, Interface Streamer und Hardware Interface. | Siehe Sammelverweis in 5.1. |
| `fig-aica-greifablauf` | `fig-aica-greifablauf.png` | Anhang, Verweis in 5.1 | AICA-Ausschnitt des Greifablaufs mit Robotiq Gripper, Object Follower und Signal Point Attractor. | Siehe Sammelverweis in 5.1. |
| `fig-aica-bewegungsregelung` | `fig-aica-bewegungsregelung.png` | Anhang, Verweis in 5.1 | AICA-Ausschnitt der Bewegungsregelung mit Priority Handler, Object Follower, Signal Point Attractor und Hardware Interface mit IK Velocity Controller. | Siehe Sammelverweis in 5.1. |
| `fig-aica-vectoring-parameter` | `fig-aica-vectoring-parameter.png` | Direkt in 5.1 | Einstellbare Taktrate und weitere Parameter der Komponente Vectoring im AICA-Interface. | „Abbildung `fig-aica-vectoring-parameter` zeigt dies beispielhaft für den Parameter `Rate` von Vectoring." |
| `fig-interface-streamer-betrieb` | `fig-interface-streamer-betrieb.png` | Direkt in 3.7 | Interface Streamer im Betrieb mit Debug-Bild der Basiskamera, Follower-Zustand, Ziel, Regelabweichungen und Trackübersicht. | „Abbildung `fig-interface-streamer-betrieb` zeigt die Anzeige während eines Greifvorgangs.“ |
| `fig-basecam-erkennung-roi` | `fig-basecam-erkennung-roi.png` | Direkt in 3.7 | Debug-Bild der Basiskamera mit den erkannten Objekten ID 10 und ID 11, deren Farben und dem für die Detektion verwendeten Bildausschnitt (ROI). | „Das Debug-Bild der Basiskamera verdeutlicht Abbildung `fig-basecam-erkennung-roi`.“ |

## Arbeitsstände, nicht zur Übernahme in Word vorgesehen

- `systemaufbau_draufsicht_vorlaeufig.svg` und
  `systemaufbau_seitenansicht_vorlaeufig.svg`: frühere SVG-Entwürfe.
- `systemaufbau_draufsicht_ppt.png`, `systemaufbau_seitenansicht_ppt.png` und
  `systemskizzen_layout.pptx`: frühere Layout-Arbeitsstände.
- `erzeuge_systemskizzen.py`: Erzeugungsskript für einen früheren Entwurf.
- `briefing_systemskizzen.md`: Übergabevorgabe für eine spätere Neuanfertigung.
