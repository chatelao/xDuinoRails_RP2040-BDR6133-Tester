# Getting Started Example

This example demonstrates basic motor control with the XDuinoRails library.

## Hardware Setup (XIAO RP2040)

- **Motor Driver**: BDR-6133 (or compatible)
- **Connections**:
    - **PWM A**: Pin 7 -> Driver IN A
    - **PWM B**: Pin 8 -> Driver IN B
    - **BEMF A**: Pin A3 -> Voltage Divider -> Motor Out A
    - **BEMF B**: Pin A2 -> Voltage Divider -> Motor Out B

## Code Structure
- `GettingStarted.ino`: Main sketch.
- `motor.update()`: Must be called in the loop.

See `examples/GettingStarted/GettingStarted.ino` for details.
