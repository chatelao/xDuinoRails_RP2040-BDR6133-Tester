# BDR-6133 Märklin Motor Driver

This project describes the wiring and control of a digital Märklin motor using a BDR6-133 motor driver and a XIAO SEED RP2040 microcontroller.

## Control System

This project uses a **closed-loop control system** to regulate the motor's speed. The system is based on measuring the motor's back-EMF (BEMF) to determine its actual speed and then adjusting the power delivered to the motor via Pulse Width Modulation (PWM) to match a target speed.

The control loop operates as follows:

1.  **Speed Measurement (BEMF Sensing):**
    *   To measure the BEMF, the motor must be temporarily disconnected from the power source. This is achieved by putting the motor driver (H-Bridge) into a high-impedance state, effectively letting the motor "coast" for a very short period.
    *   During this coasting phase, the motor acts as a generator, producing a voltage proportional to its speed. This voltage is the BEMF.
    *   The BEMF is read as a differential voltage between the two motor terminals (`OutA` and `OutB`) using the RP2040's ADC pins.
    *   As the motor turns, the BEMF signal forms a wave. The system counts the peaks (commutation pulses) of this wave over a fixed time interval (100ms) to calculate the motor's speed in pulses per second (PPS).

2.  **Proportional Controller:**
    *   A proportional (P) controller is used to adjust the motor's speed.
    *   It first calculates the `error` by subtracting the measured speed from the `target_speed`.
    *   It then calculates a new PWM duty cycle by adding a correction factor to the target speed. This correction is the `error` multiplied by a proportional gain constant (`Kp`).
    *   The formula is: `PWM_Duty_Cycle = Target_Speed + (Kp * Error)`
    *   The result is constrained to valid PWM values (0-255).

3.  **Actuation (Software PWM):**
    *   A non-blocking, software-based PWM signal with a frequency of 1kHz is used to drive the motor.
    *   The BEMF measurement is strategically timed to occur during the "off" phase of each PWM cycle, ensuring that the measurement is not affected by the driving voltage.

### Control Loop Diagram

```
            +------------------+     +------------------+     +------------------+
Set-point   |                  |     |                  |     |                  |
(Target ---->|    Controller    |----->|      Motor     |----->|      Motor     |----> Actual
 Speed)     | (P-Regler)       | PWM |      Driver      | PWM |                  |      Speed
            |                  |     |    (BDR-6133)    |     |                  |
            +------------------+     +------------------+     +------------------+
                  ^                                                   |
                  |                                                   |
                  | Error                                             | BEMF
                  |                                                   |
                  +---------------------------------------------------+
                                        Feedback
```

## Wiring

The components are connected as follows:

*   **BDR6-133 Motor Driver:**
    *   `OutA` and `OutB` are connected to the two terminals of the Märklin motor.
    *   `InA` and `InB` are the control inputs. `InA` controls the forward direction, and `InB` controls the reverse direction.

*   **Voltage Dividers (for Back EMF):**
    *   `OutA` is connected to ground via a voltage divider (6.8kΩ / 1kΩ).
    *   `OutB` is connected to ground via a voltage divider (6.8kΩ / 1kΩ).

*   **XIAO SEED RP2040 Connections:**
    *   `D2` (GPIO28) -> `BDR6-133 InA` - Controls motor speed in the forward direction.
    *   `D3` (GPIO29) -> `BDR6-133 InB` - Controls motor speed in the reverse direction.
    *   `A0` (GPIO26) -> Middle of the `OutA` voltage divider - Measures back EMF from motor terminal A.
    *   `A1` (GPIO27) -> Middle of the `OutB` voltage divider - Measures back EMF from motor terminal B.

### Connection Diagram (Physical Layout)

```
                     +----------------------+
                     |  XIAO SEED RP2040    |
                     |      (Top View)      |
                     +----------------------+
        (BEMF A) ----| A0/D0             5V |
        (BEMF B) ----| A1/D1            GND |
(PWM Fwd -> InA) ----| A2/D2            3V3 |
(PWM Rev -> InB) ----| A3/D3            D10 |
                     | D4                D9 |
                     | D5                D8 |
                     | D6                D7 |
                     +----------------------+
```
