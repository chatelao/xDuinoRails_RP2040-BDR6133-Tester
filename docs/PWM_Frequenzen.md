# Unterstützung für verschiedene PWM-Frequenzen und deren Auswirkungen

Die Pulsweitenmodulation (PWM) ist die Kerntechnologie zur Ansteuerung des Motors. Die Frequenz, mit der die PWM-Signale gesendet werden, hat erhebliche Auswirkungen auf die Motorleistung, die Effizienz des Motortreibers und die Geräuschentwicklung.

## Grundlagen der PWM-Frequenz

Die PWM-Frequenz bestimmt, wie oft der Motor pro Sekunde ein- und ausgeschaltet wird.
- **Niedrige Frequenzen** bedeuten längere, aber weniger Zyklen pro Sekunde.
- **Hohe Frequenzen** bedeuten kürzere, aber sehr viele Zyklen pro Sekunde.

Die Wahl der richtigen Frequenz ist immer ein Kompromiss zwischen verschiedenen Faktoren.

## Auswirkungen verschiedener Frequenzbereiche

### 1. Niedrige Frequenzen (< 1 kHz)

- **Vorteile:**
    - **Geringe Schaltverluste:** Der Motortreiber (H-Brücke) schaltet seltener, was zu weniger Abwärme und höherer Effizienz im Treiber führt.
- **Nachteile:**
    - **Hörbares Pfeifen:** Der Motor und die Wicklungen können hörbar im Takt der PWM-Frequenz schwingen, was oft als störendes Pfeifen oder Summen wahrgenommen wird. Für Modellbahnen ist dies meist unerwünscht.
    - **Höherer Drehmoment-Ripple:** Der Motorlauf wird "ruckeliger", da der Strom in den Wicklungen zwischen den Impulsen stärker schwankt. Dies führt zu ungleichmäßiger Kraftentfaltung, besonders bei niedrigen Drehzahlen.

### 2. Mittlere Frequenzen (ca. 1 kHz - 20 kHz)

Dieser Bereich stellt oft einen guten Kompromiss dar. Die aktuell im Projekt verwendeten 1 kHz fallen in diese Kategorie.

- **Vorteile:**
    - **Geringere Geräuschentwicklung:** Ab ca. 16-18 kHz liegt die Frequenz außerhalb des menschlichen Hörbereichs, was den Motorbetrieb nahezu geräuschlos macht.
    - **Sanfterer Motorlauf:** Der Drehmoment-Ripple wird reduziert, was zu einem gleichmäßigeren und präziseren Lauf führt.
- **Nachteile:**
    - **Erhöhte Schaltverluste:** Der Motortreiber wird wärmer, da er häufiger schaltet. Die Effizienz sinkt leicht.
    - **Auswirkungen auf den Eisenkern:** Höhere Frequenzen können zu erhöhten Wirbelstromverlusten im Eisenkern des Motors führen (sogenannte "Eisenverluste").

### 3. Hohe Frequenzen (> 20 kHz, Ultraschallbereich)

- **Vorteile:**
    - **Völlig geräuschlos:** Jede Geräuschentwicklung durch die PWM-Ansteuerung wird eliminiert.
    - **Sehr sanfter Lauf:** Der Motorstrom ist nahezu konstant (DC), was zu einem extrem gleichmäßigen Drehmoment und exzellenter Regelbarkeit führt.
- **Nachteile:**
    - **Hohe Schaltverluste:** Dies ist der größte Nachteil. Der Motortreiber kann sehr heiß werden und benötigt unter Umständen eine Kühlung. Die maximale Schaltfrequenz des Treibers (z.B. BDR6133) muss beachtet werden.
    - **Elektromagnetische Störungen (EMI):** Schnelles Schalten kann zu erhöhter Störabstrahlung führen, was andere elektronische Komponenten beeinflussen könnte.
    - **Herausforderungen bei der BEMF-Messung:** Die Zeitfenster für die BEMF-Messung werden extrem kurz. Dies stellt hohe Anforderungen an die Geschwindigkeit des ADCs und die Präzision der Synchronisation. Der Einschwingvorgang nach dem Abschalten der PWM-Brücke benötigt Zeit, die bei sehr hohen Frequenzen möglicherweise nicht mehr ausreicht.

## Vorhersage der Auswirkungen im Projekt

Eine Erhöhung der PWM-Frequenz von den aktuellen 1 kHz hätte folgende Konsequenzen:

- **1 kHz -> 18 kHz (empfohlener Schritt):**
    - **Positiv:** Das leise Motorpfeifen würde komplett verschwinden. Der Motorlauf, insbesondere im unteren Drehzahlbereich, würde spürbar sanfter werden.
    - **Negativ:** Die Schaltverluste im BDR6133 würden zunehmen. Es müsste beobachtet werden, ob die Erwärmung im zulässigen Rahmen bleibt. Das Zeitfenster für die BEMF-Messung schrumpft von 1 ms auf ca. 55 µs. Die aktuelle Implementierung mit `delayMicroseconds(100)` wäre nicht mehr möglich und müsste durch eine präzisere, schnellere Timing-Methode (z.B. über Hardware-Timer) ersetzt werden.

## Fazit

Die Implementierung einer konfigurierbaren PWM-Frequenz wäre ein wertvolles Feature. Eine Frequenz im Bereich von **16 kHz bis 25 kHz** ist für hochwertige Modellbahn-Decoder üblich und erstrebenswert, da sie einen geräuschlosen und sehr sanften Betrieb ermöglicht.

Allerdings muss bei einer Umstellung die gesamte Kette von der PWM-Erzeugung über das Timing der BEMF-Messung bis hin zur ADC-Geschwindigkeit angepasst werden. Die thermische Belastung des Motortreibers ist dabei ein kritischer Punkt, der überwacht werden muss.
