> **Hinweis:** Dieses Dokument beschreibt ein geplantes zukünftiges Feature, das in der aktuellen Softwareversion noch nicht implementiert ist. Es dient als Konzept für die Weiterentwicklung.

# Konzept für einen "Rangiermodus"

Ein "Rangiermodus" (engl. "shunting mode" oder "switching mode") ist eine spezielle Betriebsart für Modellbahn-Lokomotiven, die extrem langsame und präzise Fahrten ermöglicht, wie sie beim Rangieren von Waggons im Vorbild üblich sind.

## 1. Ziel und Zweck

Das Hauptziel ist es, die feinfühlige Steuerung im untersten Geschwindigkeitsbereich drastisch zu verbessern. Anstatt dass die Lok bei der kleinsten Reglerbewegung bereits losspringt, soll sie sanft anfahren und sich im Kriechgang millimetergenau bewegen lassen.

Typische Anwendungsfälle:
- Ankuppeln an Waggons ohne Ruck.
- Exaktes Positionieren von Zügen.
- Realistisches Befahren von Lade- oder Werkstattgleisen.

## 2. Kernfunktionen des Rangiermodus

Ein effektiver Rangiermodus wird durch eine Kombination der folgenden Anpassungen erreicht:

### a) Reduzierung der Höchstgeschwindigkeit

Die grundlegendste Maßnahme ist die Reduzierung der maximal erreichbaren Geschwindigkeit. Wenn der Rangiermodus aktiv ist, wird der gesamte Regelbereich des Fahrreglers (z.B. von Fahrstufe 1 bis 128) auf einen kleinen Teil der normalen Höchstgeschwindigkeit abgebildet (z.B. 0-25%).

**Beispiel:**
- **Normalmodus:** Fahrregler auf 50% -> Lok fährt mit 50% V_max.
- **Rangiermodus:** Fahrregler auf 50% -> Lok fährt mit 12.5% V_max (50% von 25% V_max).

Dadurch wird die Auflösung der Geschwindigkeitssteuerung vervielfacht.

### b) Deaktivierung der Anfahr-/Bremsverzögerung

Für präzise Manöver ist eine sofortige Reaktion der Lok auf Befehle unerlässlich. Jegliche simulierte Massenträgheit (eingestellte Anfahr- und Bremsverzögerung) wird im Rangiermodus typischerweise abgeschaltet. Die Lok beschleunigt und bremst also unmittelbar mit den Eingaben am Fahrregler.

### c) Anpassung des Regelalgorithmus

Der bestehende P-Regler ist ein guter Anfang, aber für den Kriechgang unter Last kann eine Anpassung oder Erweiterung notwendig sein.

- **Erhöhung des P-Anteils (Kp):** Ein höherer `Kp`-Wert kann helfen, auf Laständerungen (z.B. das Ankoppeln) schneller zu reagieren und die Zieldrehzahl stabiler zu halten. Dies muss jedoch vorsichtig geschehen, um ein Schwingen des Systems zu vermeiden.
- **Einführung eines I-Anteils (Integral):** Ein PI-Regler wäre ideal für den Rangiermodus. Der I-Anteil eliminiert die bleibende Regelabweichung. Das bedeutet, wenn die Lok unter Last langsamer wird, erhöht der I-Anteil die Motorleistung so lange, bis die Zieldrehzahl exakt erreicht ist. Dies sorgt für eine extrem konstante Kriechfahrt, unabhängig von der Last.

## 3. Technische Umsetzung (Konzept)

1.  **Aktivierung:** Eine globale `volatile bool isShuntingMode` Variable wird eingeführt. Diese kann über eine externe Schnittstelle (z.B. DCC-Funktionstaste F3, Webinterface-Button) gesetzt werden.
2.  **Geschwindigkeits-Mapping:** In der `loop()`-Funktion, wo die `targetSpeed` aus der Benutzereingabe berechnet wird, wird eine Abfrage eingefügt:
    ```cpp
    float maxSpeedFactor = 1.0;
    if (isShuntingMode) {
      maxSpeedFactor = 0.25; // Max 25% Geschwindigkeit
    }
    // Berechne die Ziel-Geschwindigkeit basierend auf dem Faktor
    targetSpeed = map(userInput, 0, 128, 0, MAX_MOTOR_SPEED * maxSpeedFactor);
    ```
3.  **Regler-Anpassung:**
    - Die `Kp`-Konstante könnte durch eine Variable ersetzt werden, die im Rangiermodus einen anderen Wert annimmt.
    - Alternativ wird der Regelalgorithmus um einen I-Anteil erweitert, der nur aktiv ist, wenn `isShuntingMode == true`.
4.  **Trägheit:** Die Logik zur Simulation der Anfahr- und Bremsverzögerung wird übersprungen, wenn der Rangiermodus aktiv ist.

## 4. Zusammenfassung

| Eigenschaft | Normalmodus | Rangiermodus (Vorschlag) |
| :--- | :--- | :--- |
| **Max. Geschwindigkeit** | 100% | Konfigurierbar (z.B. 25%) |
| **Anfahr-/Bremsverzögerung**| Aktiv | Deaktiviert |
| **Regelstrategie** | P-Regler | PI-Regler oder P-Regler mit höherem Kp |
| **Zweck** | Normaler Fahrbetrieb | Präzises, langsames Manövrieren |

Die Implementierung eines Rangiermodus würde den Nutzwert des Decoders erheblich steigern und ein vorbildgerechteres Fahren ermöglichen.
