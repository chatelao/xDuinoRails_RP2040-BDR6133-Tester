# Extended Documentation

This document contains detailed information about the xDuinoRails_RP2040-BDR6133-Tester project.

## Table of Contents

1.  [Getting Started](#getting-started)
2.  [Theory of Operation](#theory-of-operation)
3.  [Troubleshooting](#troubleshooting)

## Getting Started

This guide will help you set up the project from scratch.

### Required Hardware

*   **XIAO SEED RP2040:** A compact microcontroller that serves as the brain of the project.
*   **BDR-6133:** An H-bridge motor driver that controls the motor.
*   **Märklin Motor:** The motor to be controlled (e.g., from an old locomotive).
*   **Resistors:**
    *   2x 6.8kΩ
    *   2x 1kΩ
*   **Wiring and a breadboard** for the setup.

### Hardware Setup

The components are connected as follows. A detailed description of the pin assignments can be found in the main README.

1.  **Motor to Driver:** Connect the two terminals of the Märklin motor to the `OutA` and `OutB` terminals of the BDR-6133.
2.  **Driver to XIAO:**
    *   Connect `D7` of the XIAO to `InA` of the BDR-6133.
    *   Connect `D8` of the XIAO to `InB` of the BDR-6133.
3.  **Voltage Divider for BEMF:**
    *   Create two voltage dividers. Each divider consists of a 6.8kΩ resistor in series with a 1kΩ resistor. The 1kΩ resistor is connected to GND.
    *   Connect the midpoint of the first voltage divider (between the two resistors) to `A3` of the XIAO and the `OutA` terminal of the motor driver.
    *   Connect the midpoint of the second voltage divider to `A2` of the XIAO and the `OutB` terminal of the motor driver.
4.  **Power Supply:** Connect the power supply for the motor to the BDR-6133 and ensure that the XIAO RP2040 is also powered (e.g., via USB).

### Software Setup

This project uses **PlatformIO**, a professional development environment for embedded systems. We recommend using **Visual Studio Code** with the PlatformIO IDE extension.

1.  **Install Visual Studio Code:** Download and install VS Code from the [official website](https://code.visualstudio.com/).
2.  **Install PlatformIO IDE:**
    *   Open Visual Studio Code.
    *   Go to the "Extensions" tab in the left sidebar.
    *   Search for "PlatformIO IDE" and install the extension.
3.  **Install Python:** PlatformIO requires Python. If you haven't installed it yet, follow the instructions on the [Python website](https://www.python.org/downloads/). Make sure to add Python to your system's PATH.
4.  **Install PlatformIO Core (alternative):** If you don't want to use VS Code, you can install PlatformIO Core via the command line:
    ```bash
    pip install platformio
    ```

### Compile and Upload Code

1.  **Open Project:**
    *   Clone this repository to your computer.
    *   In Visual Studio Code, open the folder of the cloned repository (`File > Open Folder...`). PlatformIO should automatically detect the project.
2.  **Install Dependencies:** PlatformIO automatically installs all necessary libraries and tools when you build the project for the first time.
3.  **Compile and Upload:**
    *   Connect your XIAO RP2040 to your computer.
    *   In the PlatformIO toolbar (at the bottom of VS Code), click the **right arrow (->)**. This button compiles the code and uploads it to the device.
    *   Alternatively, you can do this via the PlatformIO terminal:
      ```bash
      platformio run --target upload
      ```
4.  **Open Serial Monitor:**
    *   To see debug output, click the **plug icon** in the PlatformIO toolbar to open the serial monitor.

## Theory of Operation

This project uses a **closed-loop control** system to precisely control the speed of a Märklin motor. The core of the system is the measurement of the **Back-EMF (BEMF)** to determine the actual motor speed.

### Software PWM and BEMF Measurement in Detail

The motor control is based on a **non-blocking, software-based PWM signal** with a frequency of 1 kHz. This means the microcontroller generates the PWM pulses manually instead of using a dedicated hardware PWM unit. This offers the crucial advantage that the BEMF measurement can be precisely synchronized with the PWM cycle.

The combined PWM and measurement cycle runs as follows:

1.  **PWM "ON" Phase:**
    *   At the beginning of each 1ms cycle (1000 Hz), the driver pins (`D7`, `D8`) are configured as `OUTPUT`.
    *   Depending on the `forward` variable, either `D7` is set to `HIGH` and `D8` to `LOW` (forward) or vice versa (reverse).
    *   This state is maintained for the duration of the calculated `on_time_us`, which depends on the `current_pwm` value. The motor is actively driven.

2.  **PWM "OFF" Phase and BEMF Measurement:**
    *   Once the `on_time_us` has elapsed, the "OFF" phase begins for the remainder of the 1ms cycle.
    *   **Coasting:** The driver pins (`D7`, `D8`) are immediately configured as `INPUT`. This puts the H-bridge of the BDR-6133 into a high-impedance state. The motor is now disconnected from the power supply and continues to rotate due to inertia.
    *   **Stabilization Delay:** A `delayMicroseconds(100)` is inserted. This short pause is crucial to allow the voltage spikes generated by the PWM coil to decay.
    *   **Differential BEMF Measurement:** After the pause, the voltage at the ADC pins `A3` and `A2` is measured. The actual BEMF is calculated as the absolute value of the difference between these two measurements: `abs(analogRead(bemfAPin) - analogRead(bemfBPin))`. This differential approach makes the measurement less sensitive to common-mode noise.

The following ASCII art diagram illustrates the timing:

```
|<- - - - - - - - - - 1000µs (1ms) PWM Period - - - - - - - - ->|

|<- - - - on_time_us - - ->|<- - - - - off_time_us - - - - - ->|
+---------------------------+-----------------------------------+
|                           |                                   |
|       Motor is driven     |      Motor is coasting            |
|       (Pins = OUTPUT)     |      (Pins = INPUT)               |
|                           |                                   |
+---------------------------+-----------+-----------------------+
                            |           |
                            |<- 100µs ->|
                            |   Delay   | BEMF measurement here
                            |           |
```

### Speed Measurement via Commutation Pulses

A brushed DC motor generates a rippling BEMF voltage as it rotates. The peaks of these ripples, called **commutation pulses**, occur when the brushes switch from one collector segment to the next. The frequency of these pulses is directly proportional to the motor speed.

*   The system counts these pulses by checking if the measured BEMF value exceeds a threshold (`bemf_threshold`).
*   Every 100 milliseconds, the number of counted pulses (`commutation_pulse_count`) is read to calculate the speed in **pulses per second (PPS)**.
*   The counter variable `commutation_pulse_count` is declared as `volatile` because it is used in both the main loop and the quasi-interrupt-like `update_motor_pwm()` function. Access in the `loop()` is made atomic by disabling interrupts (`noInterrupts()` / `interrupts()`) to avoid race conditions.

### Proportional Controller (P-Controller)

The system uses a simple **proportional controller** to adjust the motor speed.

1.  **Error Calculation:** The controller calculates the control deviation (`error`) by subtracting the measured speed (`measured_speed`) from the target speed (`target_speed`).
    `error = target_speed - measured_speed`
2.  **Calculate Control Variable:** The new PWM control variable (`current_pwm`) is calculated by adding a correction value to the `target_speed`. This correction value is the `error` multiplied by the proportional gain `Kp`.
    `current_pwm = target_speed + (Kp * error)`
3.  **Clamping:** The calculated PWM value is limited to the valid range of 0 to 255 using `constrain()`.

#### Tuning the `Kp` Value

The `Kp` value (`const float Kp = 0.1;` in `main.cpp`) determines how strongly the controller reacts to deviations.

*   **`Kp` value too small:** The motor reacts sluggishly to load changes and may never reach the target speed.
*   **`Kp` value too large:** The system tends to overshoot and become unstable. The motor may start to "jerk" or oscillate as the controller overreacts.
*   **Optimal `Kp` value:** The motor responds quickly to load changes without becoming unstable.

A good tuning method is to start with a small value (e.g., 0.05) and gradually increase it until the system starts to become unstable. Then choose a value slightly below this threshold.

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

### Software Architecture

The code is divided into two main parts: a high-level state machine for the test sequence and a low-level loop for the PWM and BEMF logic.

1.  **State Machine (`MotorState` enum):**
    *   This `switch` statement in the `loop()` function controls the automatic test cycle: `RAMP_UP`, `COAST_HIGH`, `RAMP_DOWN`, `COAST_LOW`, `STOP`.
    *   It uses `millis()` for timing the state durations (e.g., 3 seconds of `COAST_HIGH`).
    *   This machine determines the `target_speed` for the P-controller.

2.  **PWM and Control Loop:**
    *   This part of the code in `loop()` and `update_motor_pwm()` is executed on every iteration of the `loop()` and is responsible for the real-time control of the motor.
    *   It uses `micros()` for precise timing of the PWM pulses.
    *   It implements the logic described above for driving the motor and measuring BEMF.
    *   The P-controller is also implemented here and calculates the `current_pwm` value based on the `target_speed` from the state machine.

## Troubleshooting

Here are some common problems and their solutions.

| Problem | Possible Cause(s) | Solution(s) |
|---|---|---|
| **Motor does not move at all.** | 1. Incorrect wiring.<br>2. No or insufficient power supply for the motor.<br>3. `platformio.ini` is not configured correctly for the board. | 1. Check all connections according to the circuit diagram.<br>2. Ensure that the BDR-6133 receives a separate, suitable power supply for the motor.<br>3. Make sure that `board = seeed_xiao_rp2040` is set in `platformio.ini`. |
| **Motor only turns in one direction.** | 1. One of the PWM pins (`D7`, `D8`) is not correctly connected to the motor driver (`InA`, `InB`).<br>2. The motor driver is defective. | 1. Check the wiring between the XIAO pins and the driver inputs.<br>2. Test the driver with a simpler sketch (e.g., the "Blink" example that directly controls the pins). |
| **Motor runs erratically or stutters.** | 1. The `Kp` value of the P-controller is too high, causing oscillations.<br>2. The `bemf_threshold` is set incorrectly (too high or too low).<br>3. Poor electrical connection to the motor (loose connection). | 1. Gradually reduce the `Kp` value in `src/main.cpp` (e.g., to 0.08, 0.05, etc.).<br>2. Observe the `measured_bemf` values in the serial monitor and adjust the `bemf_threshold` to a value that is reliably above the noise but below the peaks.<br>3. Check the solder joints and plug connections. |
| **Speed control does not work; motor always runs at full speed.** | 1. The BEMF measurement is not working correctly (always returns 0).<br>2. The voltage dividers are wired incorrectly or have the wrong values. | 1. Add `Serial.println(measured_bemf);` in the `update_motor_pwm()` function to check the raw values in the serial monitor. The value should fluctuate when the motor is running.<br>2. Ensure that the resistors are correct (6.8kΩ to `OutA/B`, 1kΩ to GND) and the midpoints are connected to the ADC pins (`A2`, `A3`). |
| **Code does not compile or upload.** | 1. PlatformIO is not installed correctly.<br>2. The XIAO RP2040 is not in bootloader mode.<br>3. The USB cable is defective or is a charge-only cable. | 1. Follow the installation instructions in the "Getting Started" section again.<br>2. Press and hold the "BOOT" button on the XIAO while connecting it to the USB port to manually force bootloader mode. Release the button and try uploading again.<br>3. Use a different USB cable that is known to transfer data. |
| **The serial monitor shows no output.** | 1. Incorrect baud rate set.<br>2. Incorrect COM port selected. | 1. Make sure the serial monitor in PlatformIO is set to `9600` baud (add `monitor_speed = 9600` to `platformio.ini` if necessary).<br>2. Check in the Device Manager (Windows) or with `ls /dev/tty.*` (Mac/Linux) which port the XIAO is using, and make sure PlatformIO is using that port. |

## Project Roadmap

This is a roadmap of planned features and improvements. The full list of over 200 ideas can be found in [`PROJECT_IDEAS.md`](PROJEKT_IDEAS.md). Contributions from the community are welcome!

### Short-Term Goals (Next Steps)

*   **PID Controller:** Implement a full PID (Proportional-Integral-Differential) controller for even more precise speed control, especially under load.
*   **CLI Interface:** Develop a serial command-line interface (CLI) for runtime configuration of parameters such as `Kp`, `Ki`, `Kd`, and `target_speed`.
*   **Refactoring:** Restructure the code into C++ classes (e.g., `MotorController`, `StateMachine`) to improve readability and maintainability.

### Mid-Term Goals

*   **DCC Decoder:** Implement a simple DCC decoder to receive and process commands from a digital model train command station.
*   **RailCom Sender:** Add a RailCom transmitter to send data (e.g., the locomotive address) back to the command station.
*   **OLED Display:** Support for a small I2C OLED display to show real-time status information.

### Long-Term Goals

*   **Hardware Acceleration:** Re-investigate the use of the RP2040's PIO units to implement the PWM and BEMF measurement loop in hardware, offloading the CPU.
*   **Comprehensive DCC Implementation:** Extend the DCC decoder with features like POM (Programming on the Main), service mode programming, and function mapping.
*   **Wireless Control:** Add Bluetooth Low Energy (BLE) or Wi-Fi (via a co-processor) for wireless control via a smartphone or PC.
