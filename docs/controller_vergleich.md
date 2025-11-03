# Vergleich der Motorregelungskonzepte

Dieses Dokument vergleicht die Regelungsalgorithmen der `xDuinoRails_MotorDriver`-Bibliothek mit dem des Decoders von Gabriel Koppenstein (`core1.c`).

## 1. BEMF-Messung (Gegen-EMK)

Die Messung der Back-EMF ist die Grundlage der lastgeregelten Motorsteuerung. Beide Regler verfolgen hier unterschiedliche Ansätze zur Filterung und Aufbereitung des Messwertes.

### xDuinoRails_MotorDriver

*   **Ansatz:** Kontinuierliche Echtzeit-Filterung. Jeder einzelne ADC-Messwert wird sofort verarbeitet, um die geschätzte Motordrehzahl zu aktualisieren.
*   **Filtertechnik:** Eine zweistufige Filterkette:
    1.  **EMA-Filter (Exponential Moving Average):** Eine erste schnelle Glättung, um hochfrequentes Rauschen zu reduzieren.
    2.  **Kalman-Filter:** Ein anspruchsvollerer Filter, der den "wahren" Wert schätzt, indem er das Rauschen des Messprozesses und die erwartete Systemdynamik berücksichtigt.
*   **Vorteile:**
    *   **Geringe Latenz:** Der Regler kann sehr schnell auf Änderungen der Motordrehzahl reagieren, da jeder Messwert sofort in die Regelung einfließt.
    *   **Effizient:** Kontinuierliche kleine Berechnungen statt geblockter Verarbeitung.
*   **Nachteile:**
    *   **Anfälligkeit für Ausreißer:** Trotz der Filterung kann ein einzelner, stark abweichender Messwert (Spike) das System kurzzeitig beeinflussen.

### Gabriel Koppenstein Decoder

*   **Ansatz:** Statistische Filterung im Batch-Verfahren. Es wird ein Paket von Messwerten gesammelt, bevor eine Entscheidung getroffen wird.
*   **Filtertechnik:**
    1.  **Sammeln:** Eine definierte Anzahl von ADC-Messungen wird in einem Array gespeichert.
    2.  **Sortieren:** Das Array wird sortiert.
    3.  **Ausreißer-Entfernung:** Eine bestimmte Anzahl der kleinsten und größten Werte wird verworfen.
    4.  **Mittelwertbildung:** Der Durchschnitt der verbleibenden "guten" Werte wird gebildet.
*   **Vorteile:**
    *   **Hohe Robustheit:** Extrem unempfindlich gegenüber zufälligem Rauschen und einzelnen Messfehlern, da diese statistisch eliminiert werden.
*   **Nachteile:**
    *   **Höhere Latenz:** Der Regler reagiert per Design träger, da er warten muss, bis ein vollständiges Paket an Messwerten gesammelt und verarbeitet wurde.

### Abwägung

Die Wahl hängt vom Ziel ab: `xDuinoRails` priorisiert eine **schnelle Reaktionszeit**, während Gabriels Decoder auf maximale **Störsicherheit und Stabilität** des Messwerts ausgelegt ist.

---

## 2. Regelkreis (Control Loop)

Der Regelkreis ist das Herzstück, das den Motor auf der Zieldrehzahl hält.

### xDuinoRails_MotorDriver

*   **Typ:** PI-Regler (Proportional, Integral).
*   **Parameter-Management (Gains):**
    *   Es gibt zwei feste Sätze von Parametern (Kp, Ki): einen für den Normalbetrieb und einen für den Langsamfahrbereich ("Rangiermodus").
    *   Die Umschaltung erfolgt dynamisch, und der Integral-Anteil wird zurückgesetzt, um Sprünge zu vermeiden.
*   **Besonderheiten:**
    *   **Anti-Windup:** Der Integral-Fehler wird nur dann aufsummiert, wenn der Regler nicht am maximalen PWM-Anschlag ist.
    *   **Beschleunigung:** Die Zieldrehzahl wird über eine sanfte, lineare Rampe angefahren, deren Dauer frei wählbar ist.
*   **Vorteile:**
    *   **Einfache Abstimmung:** Mit nur zwei Parametern (Kp, Ki) ist der Regler relativ einfach zu tunen.
    *   **Weiches Fahrverhalten:** Die lineare Rampe sorgt für eine sehr gleichmäßige Beschleunigung.
*   **Nachteile:**
    *   **Weniger aggressiv:** Ohne D-Anteil reagiert der Regler weniger stark auf plötzliche Laständerungen (z.B. bei einer Steigung).

### Gabriel Koppenstein Decoder

*   **Typ:** PID-Regler (Proportional, Integral, Derivativ) mit Feed-Forward-Term.
*   **Parameter-Management (Gains):**
    *   **Gain Scheduling:** Der P-Anteil (Kp) ist nicht konstant, sondern wird dynamisch anhand einer zweigeteilten linearen Funktion aus der aktuellen Zieldrehzahl berechnet. Dies ermöglicht eine feinfühlige Regelung bei niedrigen und eine stärkere bei hohen Drehzahlen.
*   **Besonderheiten:**
    *   **Beschleunigung:** Ein separater `speed_helper` setzt die Zieldrehzahl des Reglers schrittweise hoch. Die Geschwindigkeit der Schritte wird über Beschleunigungs-/Bremskennlinien (CVs) gesteuert, was dem NMRA/DCC-Standard entspricht.
*   **Vorteile:**
    *   **Hohe Präzision und Reaktivität:** Der D-Anteil ermöglicht eine sehr schnelle Reaktion auf Laständerungen. Das Gain Scheduling sorgt für eine optimale Regelung über den gesamten Geschwindigkeitsbereich.
*   **Nachteile:**
    *   **Hohe Komplexität:** Die Abstimmung der vielen Parameter (Kp-Kurve, Ki, Kd, Tau) ist deutlich anspruchsvoller.
    *   Das Fahren über eine Speed-Tabelle fühlt sich anders an als eine stufenlose lineare Rampe.

### Abwägung

`xDuinoRails` bietet einen **einfachen, robusten PI-Regler mit sanftem Fahrgefühl**. Gabriels Decoder implementiert einen **hochpräzisen, komplexen PID-Regler**, der für die Einhaltung von DCC-Normen und maximale Kontrolle optimiert ist.

---

## 3. Anfahrverhalten (Startup)

Das Überwinden des Losbrechmoments des Motors ist eine kritische Phase.

### xDuinoRails_MotorDriver

*   **Ansatz:** Direkter Start über die Geschwindigkeitsrampe. Der PI-Regler muss durch den sich aufbauenden Integral-Fehler das Losbrechmoment überwinden.
*   **Vorteile:**
    *   **Einfach und direkt:** Keine spezielle Logik für den Start.
*   **Nachteile:**
    *   **Potenzielles Ruckeln:** Bei Motoren mit hohem Haftmoment kann es einen Moment dauern, bis der I-Anteil groß genug ist, was zu einem verzögerten oder ruckartigen Anfahren führen kann.

### Gabriel Koppenstein Decoder

*   **Ansatz:** Dedizierter, adaptiver Startup-Modus.
*   **Logik:**
    1.  Der Regler gibt einen initialen PWM-Impuls, dessen Stärke auf vorherigen erfolgreichen Starts basiert.
    2.  Die PWM wird schrittweise erhöht, bis eine Bewegung über die BEMF-Messung detektiert wird.
    3.  Der erfolgreiche PWM-Wert wird für zukünftige Starts "gelernt".
    4.  Erst danach wird in den PID-Regelbetrieb gewechselt.
*   **Vorteile:**
    *   **Sehr zuverlässig:** Das Anfahren ist auch bei schwergängigen Motoren sehr robust und wiederholbar.
    *   **Adaptiv:** Lernt den optimalen Startimpuls für den jeweiligen Motor.
*   **Nachteile:**
    *   **Leicht verzögert:** Der Start dauert einen kurzen Augenblick länger, da die Logik auf eine Reaktion des Motors wartet.

### Abwägung

`xDuinoRails` nutzt einen **universellen Ansatz**, der für die meisten Motoren gut funktioniert. Gabriels Decoder hat eine **spezialisierte Routine**, die ein optimales und zuverlässiges Anfahren unter allen Bedingungen sicherstellt.
