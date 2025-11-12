# SAMD21 Getting Started Example

This example demonstrates how to use the `XDuinoRails_MotorDriver` library with a SAMD21-based microcontroller, specifically the Adafruit QT Py M0.

## Wiring Diagram

Connect the Adafruit QT Py M0 to the BDR-6133 motor driver as follows:

| Adafruit QT Py M0 Pin | BDR-6133 Pin |
| --------------------- | -------------- |
| A0                    | InA            |
| A1                    | InB            |
| A2                    | BEMF B         |
| A3                    | BEMF A         |
| 5V                    | VCC            |
| GND                   | GND            |

**Note:** The BEMF pins must be connected to the output of a voltage divider from the motor's terminals. Refer to the main project README for the voltage divider circuit diagram.

## How to Use

1.  Select the `adafruit_qtpy_m0` board in your `platformio.ini` file or in the PlatformIO IDE.
2.  Build and upload the `SAMD21_GettingStarted` example.
3.  Open the serial monitor at 115200 baud to see the motor's target and measured speed. The motor will run for 10 seconds and then stop.
