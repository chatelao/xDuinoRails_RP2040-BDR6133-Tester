# BDR-6133 Märklin Motor Driver

This project describes the wiring and control of a digital Märklin motor using a BDR6-133 motor driver and a XIAO SEED RP2040 microcontroller.

**For a complete and detailed technical description, a "Getting Started" guide, and a troubleshooting guide, please read the [EXTENDED DOCUMENTATION](DOCUMENTATION.md).**

## Control System

This project uses a **closed-loop control system** to regulate the motor's speed. The system is based on measuring the motor's back-EMF (BEMF) to determine its actual speed and then adjusting the power delivered to the motor via Pulse Width Modulation (PWM) to match a target speed.

A non-blocking, software-based PWM signal with a frequency of 1kHz is used to drive the motor. The control loop is integrated directly into this software PWM loop.

The control loop operates as follows:

1.  **Speed Measurement (BEMF Sensing):**
    *   The BEMF measurement is strategically timed to occur during the "off" phase of each PWM cycle, ensuring that the measurement is not affected by the driving voltage.
    *   To measure the BEMF, the motor is briefly disconnected from the driving voltage. This is achieved by setting the motor driver's input pins (`D7`, `D8`) to `INPUT` mode, which puts the H-Bridge into a high-impedance (coasting) state.
    *   After a 100-microsecond delay to allow the motor's electrical state to stabilize, the BEMF is read as a differential voltage between the two motor terminals (`OutA` and `OutB`).
    *   As the motor turns, the BEMF signal forms a wave. The system counts the peaks (commutation pulses) of this wave. The total count is used to calculate the motor's speed in pulses per second (PPS) over a 100ms interval.

2.  **Proportional Controller:**
    *   A proportional (P) controller is used to adjust the motor's speed.
    *   It first calculates the `error` by subtracting the measured speed from the `target_speed`.
    *   It then calculates a new PWM duty cycle by adding a correction factor to the target speed. This correction is the `error` multiplied by a proportional gain constant (`Kp`).
    *   The formula is: `PWM_Duty_Cycle = Target_Speed + (Kp * Error)`
    *   The result is constrained to valid PWM values (0-255).

### Control Loop Diagram

```
            +------------------+     +------------------+     +------------------+
Set-point   |                  |     |                  |     |                  |
(Target --->|    Controller    |---->|      Motor       |---->|      Motor       |----> Actual
 Speed)     | (P-Controller)   | PWM |      Driver      | PWM |                  |      Speed
            |                  |     |    (BDR-6133)    |     |                  |
            +------------------+     +------------------+     +------------------+
                  ^                                                   |
                  |                                                   |
                  | Error                                             | BEMF
                  |                                                   |
                  +---------------------------------------------------+
                                        Feedback
```

### Concept Idea: Soft-Start Routine

To avoid high inrush currents when starting the motor, a soft-start routine can be implemented. This routine would gradually increase the PWM duty cycle instead of immediately jumping to the value calculated by the P-controller.

**How it works:**

1.  **Initialization:** When the motor is started, the PWM duty cycle begins at a very low value (e.g., 10 out of 255).
2.  **Ramp:** The duty cycle is increased by a small amount in each control cycle until it reaches the value required by the P-controller.
3.  **Handover to Controller:** Once the ramp is complete, the P-controller takes full control of the PWM duty cycle.

This method ensures that the motor starts smoothly, which reduces mechanical stress and avoids electrical peaks.

## Wiring

The components are connected as follows:

*   **BDR6-133 Motor Driver:**
    *   `OutA` and `OutB` are connected to the two terminals of the Märklin motor.
    *   `InA` and `InB` are the control inputs. `InA` controls the forward direction, and `InB` controls the reverse direction.

*   **Voltage Dividers (for Back EMF):**
    *   `OutA` is connected to ground via a voltage divider (6.8kΩ / 1kΩ).
    *   `OutB` is connected to ground via a voltage divider (6.8kΩ / 1kΩ).

*   **XIAO SEED RP2040 Connections:**
    *   `D7` -> `BDR6-133 InA` - Controls motor speed in the forward direction.
    *   `D8` -> `BDR6-133 InB` - Controls motor speed in the reverse direction.
    *   `A3` -> Middle of the `OutA` voltage divider - Measures back EMF from motor terminal A.
    *   `A2` -> Middle of the `OutB` voltage divider - Measures back EMF from motor terminal B.

### Connection Diagram (Physical Layout)

```
                     +----------------------+      +----------------------+      +----------------+
                     |  XIAO SEED RP2040    |      |     BDR-6133         |      |      Motor     |
                     |      (Top View)      |      |    Motor Driver      |      |                |
                     +----------------------+      +----------------------+      +----------------+
                     | D0/A0            5v  |      |                      |      |                |
                     | D1/A1            GND |      |                      |      |                |
        (BEMF B) <---| D2/A2            3v3 | <----+                 OutB |----->| B              |
        (BEMF A) <---| D3/A3            D10 | <----+                 OutA |----->| A              |
        (DCC-RX) ----| D4               D9  |      |                      |      |                |
       (ACC-ACK) ----| D5               D8  |----->| InB                  |      |                |
    (Railcom-TX) ----| D6               D7  |----->| InA                  |      |                |
                     +----------------------+      +----------------------+      +----------------+
```

## Status Indicator

The built-in LED on the XIAO SEED RP2040 provides visual feedback on the motor's operational state:

*   **Solid ON:** The motor is ramping up to its target speed.
*   **Slow Blink (500ms interval):** The motor is ramping down.
*   **Fast Blink (100ms interval):** The motor is in a brief delay period while changing direction (from forward to reverse or vice-versa).
*   **Solid OFF:** The motor is stopped.
