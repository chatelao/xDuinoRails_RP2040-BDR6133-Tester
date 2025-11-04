# PWM-Ansteuerung und Frequenzwahl

Die Pulsweitenmodulation (PWM) ist die Kerntechnologie zur Regelung der Motorleistung. Dieses Dokument beschreibt die aktuelle Implementierung und die Auswirkungen der gewählten PWM-Frequenz.

## Aktuelle Implementierung: Hardware-beschleunigtes PWM

Das Projekt nutzt einen Hardware-Timer des RP2040, um eine nicht-blockierende PWM-Ansteuerung zu realisieren. Dies ist eine wesentliche Abkehr von einer einfachen, softwarebasierten `analogWrite()`-Implementierung.

- **PWM-Frequenz:** Die Basisfrequenz ist auf **1 kHz** festgelegt.
- **Timer-Logik:** Ein wiederholender Timer (`repeating_timer`) löst den Beginn jedes PWM-Zyklus aus (die ON-Phase). Innerhalb dieses Timers wird ein zweiter, einmaliger Alarm (`one-shot alarm`) für das Ende der ON-Phase geplant. Die Callback-Funktion dieses Alarms führt dann die BEMF-Messung durch.

**Vorteile dieser Implementierung:**
- **Nicht-blockierend:** Die CPU wird nicht durch `delay()`-Aufrufe blockiert und steht für andere Aufgaben (Zustandslogik, Kommunikation) zur Verfügung.
- **Präzises Timing:** Das Timing der PWM-Flanken und der BEMF-Messung ist hardware-genau und unabhängig von der Auslastung des Hauptprozessors.
- **Effizient:** Reduziert die CPU-Last erheblich im Vergleich zu einer Software-PWM.

## Analyse der aktuellen PWM-Frequenz (1 kHz)

Die Wahl von 1 kHz ist ein bewusster Kompromiss für die erste Implementierung.

### Vorteile bei 1 kHz

- **Geringe Schaltverluste:** Der BDR6133-Motortreiber schaltet relativ selten, was die thermische Belastung minimiert und die Effizienz erhöht.
- **Großes Zeitfenster für BEMF-Messung:** Ein 1-kHz-Zyklus dauert 1000 Mikrosekunden (`µs`). Selbst bei 90% PWM-Tastverhältnis bleiben 100 µs für das Abschalten der Brücke, das Einschwingen der Spannung und die ADC-Messung. Dies ist ausreichend Zeit für die aktuell implementierte `delayMicroseconds(100)`-Pause.

### Nachteile bei 1 kHz

- **Hörbares Betriebsgeräusch:** Motoren neigen dazu, bei Frequenzen im menschlichen Hörbereich (bis ca. 18 kHz) ein leises Pfeifen oder Summen zu erzeugen. Dies ist bei 1 kHz der Fall und für hochwertige Modellbahnanwendungen oft unerwünscht.
- **Drehmoment-Ripple:** Der Strom durch die Motorwicklungen ist nicht vollständig geglättet, was besonders bei sehr niedrigen Drehzahlen zu einem leicht unruhigen Lauf führen kann.

## Ausblick: Umstellung auf eine höhere PWM-Frequenz

Für zukünftige Versionen ist eine Erhöhung der PWM-Frequenz in den Ultraschallbereich (z.B. > 18 kHz) wünschenswert, um den Motorbetrieb lautlos zu machen und den Motorlauf weiter zu glätten. Eine solche Umstellung ist jedoch nicht trivial und hat mehrere Konsequenzen:

1.  **Stark reduziertes Zeitfenster:** Bei 20 kHz dauert ein Zyklus nur noch 50 µs. Das Zeitfenster für die BEMF-Messung wird extrem kurz. Die aktuelle `delayMicroseconds(100)`-Pause wäre länger als der gesamte PWM-Zyklus und ist somit **nicht mehr möglich**. Das Timing müsste komplett überarbeitet werden, um mit wenigen Mikrosekunden auszukommen.
2.  **Erhöhte Schaltverluste:** Der Motortreiber würde 20-mal häufiger schalten, was zu einer deutlich höheren Temperaturentwicklung führt. Es müsste evaluiert werden, ob der Treiber dies ohne zusätzliche Kühlung bewältigen kann.
3.  **Schnellere ADC-Messung:** Die ADC-Wandlung müsste schnell und präzise innerhalb des kurzen Zeitfensters erfolgen.

## Fazit

Die aktuelle 1-kHz-PWM, die über Hardware-Timer realisiert wird, ist eine robuste und effiziente Grundlage. Sie bietet ein stabiles System mit viel Zeit für eine zuverlässige BEMF-Messung. Eine zukünftige Erhöhung der Frequenz zur Geräuschreduzierung ist ein logischer nächster Schritt, erfordert aber eine signifikante Überarbeitung des BEMF-Mess-Timings und eine sorgfältige Analyse der thermischen Auswirkungen auf den Motortreiber.
