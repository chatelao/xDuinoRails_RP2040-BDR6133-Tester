# Watchdog Example

This example demonstrates the use of the optional watchdog timer in the `XDuinoRails_MotorDriver` library.

## What it Does

The watchdog is a safety feature. If you enable it, the motor will automatically stop if it does not receive a new speed command within a specified timeout period. This can be useful to prevent the motor from running indefinitely if the controlling program crashes or loses connection.

This sketch initializes the motor driver, sets a watchdog timeout of 500 milliseconds, and then sets an initial motor speed. Because the `loop()` function does not continuously call `setTargetSpeed()`, the watchdog timer is allowed to expire. After 500ms, the library's internal watchdog will trigger, automatically setting the motor's target speed to 0.

The sketch prints a message to the serial monitor when this occurs.

## Wiring

For detailed wiring instructions, refer to the main `README.md` file in the root of the library. A typical setup for a XIAO RP2040 is as follows:

*   **Motor A Output** -> BDR6133 **OUTA**
*   **Motor B Output** -> BDR6133 **OUTB**
*   **XIAO RP2040 Pin D0** -> BDR6133 **INA**
*   **XIAO RP2040 Pin D1** -> BDR6133 **INB**
*   **XIAO RP2040 Pin A2** -> Voltage Divider -> **Motor A Output**
*   **XIAO RP2040 Pin A3** -> Voltage Divider -> **Motor B Output**
*   **XIAO RP2040 3V3** -> BDR6133 **VCC**
*   **XIAO RP2040 GND** -> BDR6133 **GND**
*   **External Power Supply (e.g., 12V)** -> BDR6133 **VM**

Remember that the BEMF (Back-EMF) feedback pins must be connected through a voltage divider to protect the microcontroller's ADC pins from the higher motor voltage.
