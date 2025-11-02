# Hardware-beschleunigte Motor-Steuerung (CPU-assistiert)

Dieses Dokument beschreibt die hardwarebeschleunigte Motoransteuerung und BEMF-Messung, die implementiert wird, wenn das Compiler-Flag `USE_RP2040_LOWLEVEL` gesetzt ist. Diese Implementierung nutzt die Standard-Peripherie (PWM, ADC, DMA) des RP2040 und eine minimale CPU-Unterstützung, um eine präzise und CPU-schonende Steuerung zu erreichen.

## Architektur-Überblick

Das System basiert auf einer Kette von Hardware-Blöcken, die durch kurze CPU-Interrupts synchronisiert werden:

1.  **PWM-Slice (Motorsteuerung):** Erzeugt das 25-kHz-PWM-Signal, das den Motor über den BDR6133-Treiber ansteuert. Am Ende jedes PWM-Zyklus löst dieser Block einen Hardware-Interrupt (`PWM_IRQ_WRAP`) aus.
2.  **CPU (PWM-ISR):** Die extrem kurze `on_pwm_wrap()` ISR wird aufgerufen. Ihre einzige Aufgabe ist es, einen hochpräzisen Hardware-Timer für die BEMF-Verzögerung zu starten.
3.  **Hardware-Timer:** Läuft für die exakte Verzögerungszeit von 10µs und löst nach Ablauf eine Callback-Funktion aus.
4.  **CPU (Timer-Callback):** Die `delayed_adc_trigger_callback`-Funktion wird aufgerufen. Ihre einzige Aufgabe ist es, den ADC-Messzyklus zu starten.
5.  **ADC (Analog-Digital-Wandler):** Führt eine "Round-Robin"-Messung auf den beiden BEMF-Pins durch.
6.  **DMA (Direct Memory Access):** Kopiert die ADC-Ergebnisse automatisch in einen Ringpuffer im RAM, ohne die CPU weiter zu belasten.

## Der Prozess im Detail

### 1. PWM-Phase (Motor angetrieben)

- Die `loop()`-Funktion ruft `update_pwm_duty_cycle()` auf, um die PWM-Level für die H-Brücken-Pins zu setzen.
- **Forward:** Pin INA erhält das PWM-Signal, Pin INB wird auf LOW gehalten.
- **Reverse:** Pin INB erhält das PWM-Signal, Pin INA wird auf LOW gehalten.
- Die PWM-Hardware sorgt dafür, dass während der OFF-Phase beide Pins auf LOW sind, was den BDR6133 in den hochohmigen "Stand-by"-Modus versetzt.

### 2. BEMF-Messphase (CPU-assistierter Trigger)

- **PWM-Wrap-Interrupt:** Das Ende des PWM-Zyklus löst den `PWM_IRQ_WRAP`-Interrupt aus. Die ISR `on_pwm_wrap()` wird sofort ausgeführt.
- **Stabilisierungs-Verzögerung:** Die ISR startet einen einmaligen Hardware-Timer (`add_alarm_in_us`) mit einer präzisen Verzögerung von 10µs.
- **ADC-Trigger:** Nach Ablauf der 10µs wird die Timer-Callback-Funktion `delayed_adc_trigger_callback` aufgerufen, deren einzige Aufgabe es ist, den ADC mit `adc_run(true)` zu starten.

### 3. Datenverarbeitungs-Phase (DMA-gesteuert)

- **Automatische Messung & DMA-Transfer:** Der ADC führt eine Messung auf beiden BEMF-Pins durch. Nach jeder Messung sendet der ADC ein DREQ-Signal an den DMA-Controller, der den Wert sofort in den `bemf_ring_buffer` kopiert.
- **DMA-Interrupt & Datenverarbeitung:** Sobald der DMA-Controller den Puffer gefüllt hat, löst er einen Interrupt aus. Die ISR `dma_irq_handler()` wird aufgerufen und führt die Berechnung des Durchschnitts, die Filterung, die Puls-Erkennung und die PI-Regelung aus.

Dieser Zyklus wiederholt sich kontinuierlich mit 25 kHz. Die CPU wird nur für zwei extrem kurze, zeitlich unkritische ISR-Aufrufe pro Zyklus benötigt, während das präzise Timing vollständig von der Hardware übernommen wird.
