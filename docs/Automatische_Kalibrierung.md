# Konzept für eine automatische Kalibrierungsroutine

Eine automatische Kalibrierungsroutine ist ein Prozess, bei dem der Decoder selbstständig die spezifischen Eigenschaften des angeschlossenen Motors ermittelt. Dies ermöglicht eine optimale Anpassung des Regelalgorithmus an den jeweiligen Motor, ohne dass der Benutzer manuell komplexe Parameter einstellen muss.

## 1. Ziel und Zweck

Jeder Motor ist unterschiedlich, selbst innerhalb derselben Baureihe. Parameter wie der interne Widerstand, die Induktivität und vor allem die BEMF-Konstante variieren. Eine automatische Kalibrierung soll:

- **Benutzerfreundlichkeit erhöhen:** Der Benutzer muss kein Experte für Regelungstechnik sein.
- **Optimale Leistung sicherstellen:** Der Regler (P- oder PID-Regler) wird perfekt an den Motor angepasst, was zu einem sehr guten Fahrverhalten führt.
- **Austauschbarkeit von Motoren ermöglichen:** Wird ein anderer Motor an den Decoder angeschlossen, kann durch eine Neukalibrierung schnell wieder ein optimales Ergebnis erzielt werden.

## 2. Zu bestimmende Parameter

Eine Kalibrierungsroutine könnte folgende Motoreigenschaften bestimmen:

### a) BEMF-Konstante (Ke)

Dies ist der wichtigste Parameter. Er beschreibt das Verhältnis zwischen der Motordrehzahl und der erzeugten BEMF-Spannung (Volt pro U/min). Um diesen Wert zu ermitteln, muss der Motor mit einer bekannten Spannung betrieben und die resultierende Drehzahl (gemessen über die BEMF) erfasst werden.

### b) Minimale Anfahrspannung

Dies ist die kleinste PWM-Tastrate, bei der sich der Motor gerade so in Bewegung setzt. Dieser Wert ist wichtig, um die untere Grenze des Regelbereichs festzulegen und ein "Steckenbleiben" bei sehr niedrigen Geschwindigkeiten zu verhindern.

### c) PID-Gewichte (Kp, Ki, Kd)

Die Bestimmung der optimalen Gewichte für einen PID-Regler ist die anspruchsvollste Aufgabe. Hierfür gibt es etablierte Methoden aus der Regelungstechnik:

- **Ziegler-Nichols-Methode:** Ein klassisches Verfahren, bei dem das System gezielt zum Schwingen gebracht wird, um aus der Schwingungsfrequenz und -amplitude die Regelparameter abzuleiten.
- **Relais-Methode (Åström-Hägglund):** Eine sicherere und modernere Methode, die ebenfalls Schwingungen provoziert, aber auf eine kontrolliertere Weise.

Für den Anfang würde sich die Kalibrierung auf die BEMF-Konstante und die Anfahrspannung konzentrieren. Die Bestimmung der PID-Gewichte ist ein fortgeschrittenes Feature.

## 3. Ablauf einer möglichen Kalibrierungsroutine

Die Routine könnte wie folgt ablaufen, nachdem sie vom Benutzer (z.B. über eine Taste am Decoder, DCC-Befehl oder Webinterface) gestartet wurde. Die Lokomotive sollte dabei frei auf einem geraden Gleisstück stehen.

1.  **Phase 1: Bestimmung der minimalen Anfahrspannung**
    - Der PWM-Wert wird langsam von 0 erhöht.
    - Gleichzeitig wird die BEMF überwacht.
    - Sobald eine stabile BEMF > 0 gemessen wird, hat sich der Motor in Bewegung gesetzt. Der zugehörige PWM-Wert wird als `minPwm` gespeichert.

2.  **Phase 2: Messung der Kennlinie (BEMF vs. PWM)**
    - Der Motor wird nun mit verschiedenen, festen PWM-Werten betrieben (z.B. in 10%-Schritten von `minPwm` bis 100%).
    - Bei jedem Schritt wird gewartet, bis sich die Drehzahl stabilisiert hat, und dann der Durchschnitt der BEMF-Messwerte über einen kurzen Zeitraum erfasst.
    - Das Ergebnis ist eine Tabelle von Wertepaaren: `(PWM_Wert, gemessene_BEMF)`.

3.  **Phase 3: Berechnung der BEMF-Konstante**
    - Aus der gemessenen Kennlinie kann die BEMF-Konstante angenähert werden. Vereinfacht gesagt, beschreibt sie die Steigung dieser Kennlinie. Der Decoder kann intern eine lineare Regression durchführen, um eine möglichst genaue Konstante zu berechnen.
    - Dieser Wert wird permanent im Flash-Speicher des Mikrocontrollers gespeichert.

4.  **Phase 4 (Optional): Bestimmung der PID-Parameter**
    - Der Decoder führt einen Test nach der Relais-Methode durch. Dabei wird die Motorleistung schnell zwischen zwei Werten umgeschaltet, was den Motor in eine stabile Schwingung versetzt.
    - Aus der Amplitude und Frequenz dieser Schwingung können nach festen Formeln die Parameter Kp, Ki und Kd berechnet und gespeichert werden.

## 4. Sicherheitsaspekte

- Die Lok wird sich während der Kalibrierung bewegen. Der Benutzer muss sicherstellen, dass genügend freies Gleis vorhanden ist.
- Die Routine sollte einen Timeout haben, um nicht endlos zu laufen, falls etwas nicht wie erwartet funktioniert.
- Der Stromverbrauch sollte überwacht werden, um ein Blockieren des Motors zu erkennen und die Routine abzubrechen.

## Fazit

Eine automatische Kalibrierungsroutine ist ein hochprofessionelles Feature, das den Decoder von einfachen Steuerungen abhebt. Die Implementierung ist anspruchsvoll, aber der Nutzen für den Endanwender ist enorm. Eine schrittweise Realisierung, beginnend mit der Anfahrspannung und der BEMF-Konstante, ist ein realistischer Ansatz.
