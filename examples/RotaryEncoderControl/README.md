# Rotary Encoder Control Example

This example demonstrates how to control the motor's speed and direction using a standard KY-040 rotary encoder.

## How it Works

- **Speed Control:** Turning the encoder knob increases or decreases the motor's target speed. One full rotation of a 24-detent encoder will ramp the speed from 0 to 100%.
- **Stop & Direction Control:** Pressing the encoder's push-button has two functions:
  1. If the motor is currently moving, it acts as an emergency stop, setting the target speed to 0.
  2. If the motor is stopped, it toggles the direction of travel for the next time the motor starts (Forward -> Reverse -> Forward).

## Hardware Setup

Connect the rotary encoder to the XIAO RP2040 as follows:

| Encoder Pin | XIAO RP2040 Pin |
| :---------- | :-------------- |
| CLK         | D0              |
| DT          | D1              |
| SW (Switch) | D9              |
| + (VCC)     | 3.3V            |
| GND         | GND             |

**Note:** The example uses the microcontroller's internal pull-up resistors for the CLK, DT, and SW pins. Therefore, you do not need to add external pull-up resistors to your circuit.
