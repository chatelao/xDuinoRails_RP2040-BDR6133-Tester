# BDR-6133 Märklin Motor Driver

This project describes the wiring and control of a digital Märklin motor using a BDR6-133 motor driver and a XIAO SEED RP2040 microcontroller.

## Control System

This project uses a **closed-loop control system** to regulate the motor's speed. A non-blocking, software-based PWM system is used to control the motor. This allows for precise timing of back-EMF (BEMF) measurements during the "off" phase of the PWM cycle, providing an accurate measurement of the motor's speed. A proportional controller then adjusts the PWM duty cycle to match the target speed. The entire control loop is managed with non-blocking timers for smooth operation.

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
