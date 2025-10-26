# BDR6133 Märklin Motor Driver

This project describes the wiring and control of a digital Märklin motor using a BDR6133 motor driver and a XIAO SEED RP2040 microcontroller.

## Wiring

* **BDR6133 Motor Driver:**
    * `OutA` and `OutB` are connected to the motor.
    * `OutA` is connected to ground with a voltage divider (6.8kΩ / 1kΩ).
    * `OutB` is connected to ground with a voltage divider (6.8kΩ / 1kΩ).
* **XIAO SEED RP2040:**
    * The middle of the `OutA` voltage divider is connected to an ADC pin on the RP2040 for back EMF measurement.
    * The middle of the `OutB` voltage divider is connected to another ADC pin on the RP2040 for back EMF measurement.
    * A PWM pin on the RP2040 is used to control the speed of the motor via the BDR6133.
    * Another digital pin on the RP2040 is used to control the direction of the motor.
