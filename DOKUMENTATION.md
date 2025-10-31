# Erweiterte Dokumentation

Dieses Dokument enthält detaillierte Informationen über das xDuinoRails_RP2040-BDR6133-Tester Projekt.

## Inhaltsverzeichnis

1.  [Erste Schritte](#erste-schritte)
2.  [Funktionsweise (Theory of Operation)](#funktionsweise-theory-of-operation)
3.  [Fehlerbehebung (Troubleshooting)](#fehlerbehebung-troubleshooting)

## Erste Schritte

Diese Anleitung hilft Ihnen, das Projekt von Grund auf einzurichten.

### Benötigte Hardware

*   **XIAO SEED RP2040:** Ein kompakter Mikrocontroller, der als Gehirn des Projekts dient.
*   **BDR-6133:** Ein H-Brücken-Motortreiber, der den Motor steuert.
*   **Märklin Motor:** Der Motor, der gesteuert werden soll (z.B. aus einer alten Lokomotive).
*   **Widerstände:**
    *   2x 6.8kΩ
    *   2x 1kΩ
*   **Verkabelung und ein Breadboard** für den Aufbau.

### Hardware-Aufbau

Die Komponenten werden wie folgt verbunden. Eine detaillierte Beschreibung der Pin-Belegung finden Sie im Haupt-README.

1.  **Motor an Treiber:** Verbinden Sie die beiden Anschlüsse des Märklin-Motors mit den `OutA`- und `OutB`-Anschlüssen des BDR-6133.
2.  **Treiber an XIAO:**
    *   Verbinden Sie `D7` des XIAO mit `InA` des BDR-6133.
    *   Verbinden Sie `D8` des XIAO mit `InB` des BDR-6133.
3.  **Spannungsteiler für BEMF:**
    *   Erstellen Sie zwei Spannungsteiler. Jeder Teiler besteht aus einem 6.8kΩ-Widerstand in Reihe mit einem 1kΩ-Widerstand. Der 1kΩ-Widerstand wird mit GND verbunden.
    *   Verbinden Sie den Mittelpunkt des ersten Spannungsteilers (zwischen den beiden Widerständen) mit `A3` des XIAO und dem `OutA`-Anschluss des Motortreibers.
    *   Verbinden Sie den Mittelpunkt des zweiten Spannungsteilers mit `A2` des XIAO und dem `OutB`-Anschluss des Motortreibers.
4.  **Stromversorgung:** Verbinden Sie die Stromversorgung für den Motor mit dem BDR-6133 und stellen Sie sicher, dass der XIAO RP2040 ebenfalls mit Strom versorgt wird (z.B. über USB).

### Software-Einrichtung

Dieses Projekt verwendet **PlatformIO**, eine professionelle Entwicklungsumgebung für eingebettete Systeme. Wir empfehlen die Verwendung von **Visual Studio Code** mit der PlatformIO IDE-Erweiterung.

1.  **Visual Studio Code installieren:** Laden Sie VS Code von der [offiziellen Website](https://code.visualstudio.com/) herunter und installieren Sie es.
2.  **PlatformIO IDE installieren:**
    *   Öffnen Sie Visual Studio Code.
    *   Gehen Sie zum "Extensions"-Tab (Erweiterungen) in der linken Seitenleiste.
    *   Suchen Sie nach "PlatformIO IDE" und installieren Sie die Erweiterung.
3.  **Python installieren:** PlatformIO benötigt Python. Falls Sie es noch nicht installiert haben, folgen Sie den Anweisungen auf der [Python-Website](https://www.python.org/downloads/). Stellen Sie sicher, dass Python zu Ihrem System-PATH hinzugefügt wird.
4.  **PlatformIO Core installieren (alternativ):** Falls Sie nicht VS Code verwenden möchten, können Sie PlatformIO Core über die Befehlszeile installieren:
    ```bash
    pip install platformio
    ```

### Code kompilieren und hochladen

1.  **Projekt öffnen:**
    *   Klonen Sie dieses Repository auf Ihren Computer.
    *   Öffnen Sie in Visual Studio Code den Ordner des geklonten Repositorys (`File > Open Folder...`). PlatformIO sollte das Projekt automatisch erkennen.
2.  **Abhängigkeiten installieren:** PlatformIO installiert automatisch alle notwendigen Bibliotheken und Werkzeuge, wenn Sie das Projekt zum ersten Mal bauen.
3.  **Kompilieren und Hochladen:**
    *   Verbinden Sie Ihren XIAO RP2040 mit Ihrem Computer.
    *   Klicken Sie in der PlatformIO-Symbolleiste (am unteren Rand von VS Code) auf den **Pfeil nach rechts (->)**. Dieser Knopf kompiliert den Code und lädt ihn auf das Gerät hoch.
    *   Alternativ können Sie dies über das PlatformIO-Terminal tun:
      ```bash
      platformio run --target upload
      ```
4.  **Seriellen Monitor öffnen:**
    *   Um Debug-Ausgaben zu sehen, klicken Sie auf das **Stecker-Symbol** in der PlatformIO-Symbolleiste, um den seriellen Monitor zu öffnen.

## Funktionsweise (Theory of Operation)

Dieses Projekt nutzt eine **geschlossene Regelschleife (Closed-Loop Control)**, um die Geschwindigkeit eines Märklin-Motors präzise zu steuern. Herzstück des Systems ist die Messung der **Gegen-EMK (Elektromotorische Kraft)**, englisch **Back-EMF (BEMF)**, um die tatsächliche Motorgeschwindigkeit zu ermitteln.

### Software-PWM und BEMF-Messung im Detail

Die Motorsteuerung basiert auf einem **nicht-blockierenden, softwarebasierten PWM-Signal** mit einer Frequenz von 1 kHz. Das bedeutet, der Mikrocontroller erzeugt die PWM-Impulse manuell, anstatt eine dedizierte Hardware-PWM-Einheit zu verwenden. Dies bietet den entscheidenden Vorteil, dass die BEMF-Messung exakt mit dem PWM-Zyklus synchronisiert werden kann.

Der kombinierte PWM- und Messzyklus läuft wie folgt ab:

1.  **PWM "ON"-Phase:**
    *   Zu Beginn jedes 1ms-Zyklus (1000 Hz) werden die Treiberpins (`D7`, `D8`) als `OUTPUT` konfiguriert.
    *   Abhängig von der `forward`-Variable wird entweder `D7` auf `HIGH` und `D8` auf `LOW` (vorwärts) oder umgekehrt (rückwärts) gesetzt.
    *   Dieser Zustand bleibt für die Dauer der berechneten `on_time_us` bestehen, die vom `current_pwm`-Wert abhängt. Der Motor wird aktiv angetrieben.

2.  **PWM "OFF"-Phase und BEMF-Messung:**
    *   Sobald die `on_time_us` abgelaufen ist, beginnt die "OFF"-Phase für den Rest des 1ms-Zyklus.
    *   **Coasting (Schweben):** Die Treiberpins (`D7`, `D8`) werden sofort als `INPUT` konfiguriert. Dies versetzt die H-Brücke des BDR-6133 in einen hochohmigen Zustand. Der Motor ist nun von der Versorgungsspannung getrennt und dreht sich aufgrund seiner Trägheit weiter.
    *   **Stabilisierungs-Delay:** Ein `delayMicroseconds(100)` wird eingefügt. Diese kurze Pause ist entscheidend, damit die durch die PWM-Spule erzeugten Spannungsspitzen abklingen können.
    *   **Differenzielle BEMF-Messung:** Nach der Pause wird die Spannung an den ADC-Pins `A3` und `A2` gemessen. Die tatsächliche BEMF wird als Absolutwert der Differenz zwischen diesen beiden Messungen berechnet: `abs(analogRead(bemfAPin) - analogRead(bemfBPin))`. Dieser differenzielle Ansatz macht die Messung unempfindlicher gegenüber Gleichtaktstörungen.

Das folgende ASCII-Art-Diagramm veranschaulicht das Timing:

```
|<- - - - - - - - - - 1000µs (1ms) PWM-Periode - - - - - - - - ->|

|<- - - - on_time_us - - ->|<- - - - - off_time_us - - - - - ->|
+---------------------------+-----------------------------------+
|                           |                                   |
|  Motor wird angetrieben   |  Motor im Leerlauf (Coasting)     |
|  (Pins = OUTPUT)          |  (Pins = INPUT)                   |
|                           |                                   |
+---------------------------+-----------+-----------------------+
                            |           |
                            |<- 100µs ->|
                            |  Delay    | BEMF-Messung hier
                            |           |
```

### Geschwindigkeitsmessung durch Kommutierungspulse

Ein Bürsten-Gleichstrommotor erzeugt beim Drehen eine wellenförmige BEMF-Spannung. Die Spitzen dieser Wellen, **Kommutierungspulse** genannt, entstehen, wenn die Bürsten von einem Kollektorsegment zum nächsten wechseln. Die Frequenz dieser Pulse ist direkt proportional zur Motorgeschwindigkeit.

*   Das System zählt diese Pulse, indem es prüft, ob der gemessene BEMF-Wert einen Schwellenwert (`bemf_threshold`) überschreitet.
*   Alle 100 Millisekunden wird die Anzahl der gezählten Pulse (`commutation_pulse_count`) ausgelesen, um die Geschwindigkeit in **Pulsen pro Sekunde (PPS)** zu berechnen.
*   Die Zählvariable `commutation_pulse_count` ist als `volatile` deklariert, da sie sowohl in der Hauptschleife als auch in der quasi-interruptartigen `update_motor_pwm()`-Funktion verwendet wird. Der Zugriff in der `loop()` erfolgt atomar durch Deaktivieren der Interrupts (`noInterrupts()` / `interrupts()`), um Race Conditions zu vermeiden.

### Proportional-Regler (P-Regler)

Das System verwendet einen einfachen **Proportional-Regler**, um die Motorgeschwindigkeit anzupassen.

1.  **Fehlerberechnung:** Der Regler berechnet die Regelabweichung (`error`), indem er die gemessene Geschwindigkeit (`measured_speed`) von der Soll-Geschwindigkeit (`target_speed`) subtrahiert.
    `error = target_speed - measured_speed`
2.  **Stellgröße berechnen:** Die neue PWM-Stellgröße (`current_pwm`) wird berechnet, indem zum `target_speed` ein Korrekturwert addiert wird. Dieser Korrekturwert ist der `error` multipliziert mit der Proportionalverstärkung `Kp`.
    `current_pwm = target_speed + (Kp * error)`
3.  **Begrenzung (Clamping):** Der berechnete PWM-Wert wird mit `constrain()` auf den gültigen Bereich von 0 bis 255 begrenzt.

#### Abstimmung des `Kp`-Wertes

Der `Kp`-Wert (`const float Kp = 0.1;` in `main.cpp`) bestimmt, wie stark der Regler auf Abweichungen reagiert.

*   **Zu kleiner `Kp`-Wert:** Der Motor reagiert träge auf Laständerungen und erreicht möglicherweise nie die Soll-Geschwindigkeit.
*   **Zu großer `Kp`-Wert:** Das System neigt zum Überschwingen und zu Instabilität. Der Motor kann anfangen zu "ruckeln" oder zu oszillieren, da der Regler überreagiert.
*   **Optimaler `Kp`-Wert:** Der Motor reagiert schnell auf Laständerungen, ohne instabil zu werden.

Eine gute Methode zur Abstimmung ist, mit einem kleinen Wert (z.B. 0.05) zu beginnen und ihn schrittweise zu erhöhen, bis das System beginnt, instabil zu werden. Wählen Sie dann einen Wert, der etwas unterhalb dieser Schwelle liegt.

### Layered Controller Diagram

+--------------------------------------------------------------------------------------------------+
| Layer 1: High-Level State Machine (Logic in loop())                                              |
|--------------------------------------------------------------------------------------------------|
|   Inputs:                                                                                        |
|     - `millis()` for timing state durations (e.g., COAST_HIGH for 3s)                            |
|   Logic:                                                                                         |
|     - `switch (current_state)` manages transitions:                                              |
|       RAMP_UP -> COAST_HIGH -> RAMP_DOWN -> COAST_LOW -> STOP -> (reverse) -> RAMP_UP ...         |
|     - Determines the desired speed for the motor.                                                |
|   Outputs:                                                                                       |
|     - `int target_speed` (0-255)  -> To Layer 2                                                  |
+--------------------------------------------------------------------------------------------------+
             |
             | `target_speed`
             v
+--------------------------------------------------------------------------------------------------+
| Layer 2: Proportional Controller (Logic in pwm_off_callback())                                   |
|--------------------------------------------------------------------------------------------------|
|   Inputs:                                                                                        |
|     - `int target_speed` (from Layer 1)                                                          |
|     - `float measured_speed_pps` (from Feedback Loop)                                            |
|   Constants:                                                                                     |
|     - `const float Kp = 0.1` (Proportional Gain)                                                 |
|   Logic:                                                                                         |
|     1. `error = target_speed - measured_speed` (approx. mapping)                                 |
|     2. `new_pwm = constrain(target_speed + (Kp * error), 0, max_speed)`                          |
|   Outputs:                                                                                       |
|     - `volatile int current_pwm` (0-255) -> To Layer 3                                           |
+--------------------------------------------------------------------------------------------------+
             |
             | `current_pwm`
             v
+--------------------------------------------------------------------------------------------------+
| Layer 3: PWM & BEMF Hardware Abstraction (RP2040 Hardware Timers)                                |
|--------------------------------------------------------------------------------------------------|
|   Setup:                                                                                         |
|     - `add_repeating_timer_us(1000, pwm_on_callback, ...)` -> Establishes 1kHz base frequency     |
|   Logic (executed every 1ms):                                                                    |
|     1. `pwm_on_callback`:                                                                        |
|        - Reads `current_pwm`.                                                                    |
|        - Calculates `on_time_us`.                                                                |
|        - Drives H-Bridge (sets pins to OUTPUT, HIGH/LOW).                                        |
|        - Schedules `pwm_off_callback` via `add_alarm_in_us(on_time_us, ...)`.                    |
|     2. `pwm_off_callback`:                                                                       |
|        - Coasts H-Bridge (sets pins to INPUT).                                                   |
|        - `delayMicroseconds(100)`.                                                               |
|        - Triggers BEMF ADC reading.                                                              |
|   Outputs:                                                                                       |
|     - Digital HIGH/LOW signals on `pwmAPin` (D7) / `pwmBPin` (D8) -> To Layer 4                  |
|     - ADC read trigger on `bemfAPin` (A3) / `bemfBPin` (A2)       -> To Layer 4                  |
+--------------------------------------------------------------------------------------------------+
             |                                       ^
             | PWM Signals                           | BEMF Voltage
             v                                       |
+--------------------------------------------------------------------------------------------------+
| Layer 4: Physical Hardware                                                                       |
|--------------------------------------------------------------------------------------------------|
|   Components:                                                                                    |
|     - XIAO RP2040: Microcontroller                                                               |
|     - BDR-6133 H-Bridge: Motor Driver                                                            |
|       - Takes `InA`/`InB` as input, drives motor at `OutA`/`OutB`.                                |
|     - Märklin Motor: The actuator, produces rotation and BEMF.                                   |
|     - Voltage Dividers (6.8k/1k): Scale BEMF voltage for ADC.                                    |
|   Outputs:                                                                                       |
|     - Motor Rotation                                                                             |
|     - BEMF Voltage from motor terminals -> To Feedback Loop                                      |
+--------------------------------------------------------------------------------------------------+
             ^
             |
             +-------------------------------------------------------------------------------------+
                                                   |
+--------------------------------------------------------------------------------------------------+
| Feedback Loop                                                                                    |
|--------------------------------------------------------------------------------------------------|
|   Inputs:                                                                                        |
|     - BEMF voltage from Layer 4 motor terminals.                                                 |
|   Logic:                                                                                         |
|     1. ADC reads scaled voltage: `bemfA = analogRead(A3)`, `bemfB = analogRead(A2)`.              |
|     2. Differential measurement: `measured_bemf = abs(bemfA - bemfB)`.                           |
|     3. Pulse detection: `if (measured_bemf > bemf_threshold) ... commutation_pulse_count++`.       |
|     4. Speed calculation (in `loop()`, every 100ms):                                             |
|        - Atomically read and reset `commutation_pulse_count`.                                    |
|        - `measured_speed_pps = pulses / 0.1`.                                                    |
|   Outputs:                                                                                       |
|     - `float measured_speed_pps` -> To Layer 2                                                   |
+--------------------------------------------------------------------------------------------------+

### Software-Architektur

Der Code ist in zwei Hauptteile gegliedert: eine übergeordnete Zustandsmaschine für den Testablauf und eine untergeordnete Schleife für die PWM- und BEMF-Logik.

1.  **Zustandsmaschine (`MotorState` enum):**
    *   Diese `switch`-Anweisung in der `loop()`-Funktion steuert den automatischen Testzyklus: `RAMP_UP`, `COAST_HIGH`, `RAMP_DOWN`, `COAST_LOW`, `STOP`.
    *   Sie verwendet `millis()` für das Timing der Zustandsdauern (z.B. 3 Sekunden `COAST_HIGH`).
    *   Diese Maschine bestimmt den `target_speed` für den P-Regler.

2.  **PWM- und Regelschleife:**
    *   Dieser Teil des Codes in `loop()` und `update_motor_pwm()` wird bei jedem Durchlauf der `loop()` ausgeführt und ist für die Echtzeitsteuerung des Motors verantwortlich.
    *   Er verwendet `micros()` für das präzise Timing der PWM-Impulse.
    *   Er implementiert die oben beschriebene Logik zum Fahren des Motors und Messen der BEMF.
    *   Der P-Regler ist ebenfalls hier implementiert und berechnet den `current_pwm`-Wert basierend auf dem `target_speed` der Zustandsmaschine.

## Fehlerbehebung (Troubleshooting)

Hier sind einige häufig auftretende Probleme und deren Lösungen.

| Problem | Mögliche Ursache(n) | Lösung(en) |
|---|---|---|
| **Motor bewegt sich überhaupt nicht.** | 1. Falsche Verkabelung.<br>2. Keine oder unzureichende Stromversorgung für den Motor.<br>3. `platformio.ini` ist nicht korrekt für das Board konfiguriert. | 1. Überprüfen Sie alle Verbindungen gemäß dem Schaltplan.<br>2. Stellen Sie sicher, dass der BDR-6133 eine separate, geeignete Stromversorgung für den Motor erhält.<br>3. Vergewissern Sie sich, dass `board = seeed_xiao_rp2040` in der `platformio.ini` gesetzt ist. |
| **Motor dreht sich nur in eine Richtung.** | 1. Einer der PWM-Pins (`D7`, `D8`) ist nicht korrekt mit dem Motortreiber (`InA`, `InB`) verbunden.<br>2. Der Motortreiber ist defekt. | 1. Überprüfen Sie die Verkabelung zwischen den XIAO-Pins und den Treiber-Eingängen.<br>2. Testen Sie den Treiber mit einem einfacheren Sketch (z.B. dem "Blink"-Beispiel, das die Pins direkt ansteuert). |
| **Motor läuft unruhig oder stottert stark.** | 1. Der `Kp`-Wert des P-Reglers ist zu hoch, was zu Oszillationen führt.<br>2. Der `bemf_threshold` ist falsch eingestellt (zu hoch oder zu niedrig).<br>3. Schlechte elektrische Verbindung zum Motor (Wackelkontakt). | 1. Reduzieren Sie den `Kp`-Wert in `src/main.cpp` schrittweise (z.B. auf 0.08, 0.05 etc.).<br>2. Beobachten Sie die `measured_bemf`-Werte im seriellen Monitor und passen Sie den `bemf_threshold` an einen Wert an, der zuverlässig über dem Rauschen, aber unter den Spitzen liegt.<br>3. Überprüfen Sie die Lötstellen und Steckverbindungen. |
| **Geschwindigkeitsregelung funktioniert nicht; Motor läuft immer mit voller Geschwindigkeit.** | 1. Die BEMF-Messung funktioniert nicht korrekt (liefert immer 0).<br>2. Die Spannungsteiler sind falsch verdrahtet oder haben falsche Werte. | 1. Fügen Sie `Serial.println(measured_bemf);` in der `update_motor_pwm()`-Funktion hinzu, um die Rohwerte im seriellen Monitor zu überprüfen. Der Wert sollte schwanken, wenn sich der Motor dreht.<br>2. Stellen Sie sicher, dass die Widerstände korrekt (6.8kΩ zu `OutA/B`, 1kΩ zu GND) und die Mittelpunkte mit den ADC-Pins (`A2`, `A3`) verbunden sind. |
| **Code lässt sich nicht kompilieren oder hochladen.** | 1. PlatformIO ist nicht korrekt installiert.<br>2. Der XIAO RP2040 befindet sich nicht im Bootloader-Modus.<br>3. USB-Kabel ist defekt oder nur ein Ladekabel. | 1. Folgen Sie der Installationsanleitung im Abschnitt "Erste Schritte" erneut.<br>2. Halten Sie die "BOOT"-Taste am XIAO gedrückt, während Sie ihn an den USB-Port anschließen, um den Bootloader-Modus manuell zu erzwingen. Lassen Sie die Taste los und versuchen Sie den Upload erneut.<br>3. Verwenden Sie ein anderes USB-Kabel, das nachweislich Daten überträgt. |
| **Der serielle Monitor zeigt keine Ausgabe.** | 1. Falsche Baudrate eingestellt.<br>2. Falscher COM-Port ausgewählt. | 1. Stellen Sie sicher, dass der serielle Monitor in PlatformIO auf `9600` Baud eingestellt ist (fügen Sie `monitor_speed = 9600` zur `platformio.ini` hinzu, falls nötig).<br>2. Überprüfen Sie im Gerätemanager (Windows) oder mit `ls /dev/tty.*` (Mac/Linux), welchen Port der XIAO verwendet, und stellen Sie sicher, dass PlatformIO diesen Port nutzt. |

## Projekt-Roadmap

Dies ist eine Roadmap geplanter Features und Verbesserungen. Die vollständige Liste mit über 200 Ideen finden Sie in der [`PROJEKT_IDEEN.md`](PROJEKT_IDEEN.md). Beiträge aus der Community sind willkommen!

### Kurzfristige Ziele (Nächste Schritte)

*   **PID-Regler:** Implementierung eines vollständigen PID-Reglers (Proportional-Integral-Differential) für eine noch präzisere Geschwindigkeitsregelung, insbesondere unter Last.
*   **CLI-Schnittstelle:** Entwicklung einer seriellen Kommandozeilenschnittstelle (CLI) zur Laufzeitkonfiguration von Parametern wie `Kp`, `Ki`, `Kd` und `target_speed`.
*   **Refactoring:** Umstrukturierung des Codes in C++-Klassen (z.B. `MotorController`, `StateMachine`), um die Lesbarkeit und Wartbarkeit zu verbessern.

### Mittelfristige Ziele

*   **DCC-Decoder:** Implementierung eines einfachen DCC-Decoders zum Empfangen und Verarbeiten von Befehlen von einer digitalen Modellbahnzentrale.
*   **RailCom-Sender:** Hinzufügen eines RailCom-Senders, um Daten (z.B. die Lokadresse) an die Zentrale zurückzumelden.
*   **OLED-Display:** Unterstützung für ein kleines I2C-OLED-Display zur Anzeige von Statusinformationen in Echtzeit.

### Langfristige Ziele

*   **Hardwarebeschleunigung:** Erneute Untersuchung der Nutzung der PIO-Einheiten des RP2040, um die PWM- und BEMF-Messschleife in Hardware zu implementieren und die CPU zu entlasten.
*   **Umfassende DCC-Implementierung:** Erweiterung des DCC-Decoders um Funktionen wie POM (Programming on the Main), Servicemodus-Programmierung und Funktionsmapping.
*   **Drahtlose Steuerung:** Hinzufügen von Bluetooth Low Energy (BLE) oder WLAN (über einen Co-Prozessor) für die drahtlose Steuerung per Smartphone oder PC.
