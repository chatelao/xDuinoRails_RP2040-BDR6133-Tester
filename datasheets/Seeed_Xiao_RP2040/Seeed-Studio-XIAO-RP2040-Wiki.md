# Seeed Studio XIAO RP2040 Wiki (Abridged)

This document contains key sections from the Seeed Studio XIAO RP2040 wiki page for quick reference.
Source: https://wiki.seeedstudio.com/XIAO-RP2040/

## Features

*   **Powerful MCU:** Dual-core ARM Cortex M0+ processor, flexible clock running up to 133 MHz
*   **Rich on-chip resources:** 264KB of SRAM, and 2MB of on-board Flash memory
*   **Flexible compatibility:** Support Micropython/Arduino/CircuitPython
*   **Easy project operation:** Breadboard-friendly & SMD design, no components on the back
*   **Small size:** As small as a thumb (21x17.8mm) for wearable devices and small projects.
*   **Multiple interfaces:** 11 digital pins, 4 analog pins, 11 PWM Pins, 1 I2C interface, 1 UART interface, 1 SPI interface, 1 SWD Bonding pad interface.

## Specification

| Item                                | Value                                          |
| ----------------------------------- | ---------------------------------------------- |
| CPU                                 | Dual-core ARM Cortex M0+ processor up to 133MHz |
| Flash Memory                        | 2MB                                            |
| SRAM                                | 264KB                                          |
| Digital I/O Pins                    | 11                                             |
| Analog I/O Pins                     | 4                                              |
| PWM Pins                            | 11                                             |
| I2C interface                       | 1                                              |
| SPI interface                       | 1                                              |
| UART interface                      | 1                                              |
| Power supply and downloading interface | Type-C                                         |
| Power                               | 3.3V/5V DC                                     |
| Dimensions                          | 21×17.8×3.5mm                                  |

## Hardware Overview

### Cautions

*   **General I/O pins:** Working voltage of MCU is 3.3V. Voltage input connected to general I/O pins may cause chip damage if it's higher than 3.3V.
*   **Power supply pins:** The built-in DC-DC converter circuit is able to change 5V voltage into 3.3V, allowing the device to be powered with a 5V supply via the VIN-PIN and 5V-PIN.
*   **Battery:** XIAO RP2040 currently only supports battery power supply and cannot connect to Type-C while a battery is connected, as it may pose a safety risk.
*   Do not lift the shield cover.

### Enter Bootloader Mode

If the port disappears when the programming process fails, you can enter Bootloader mode:
1.  Long press the "B" button.
2.  Connect the Seeed Studio XIAO RP2040 to your computer.
3.  A disk drive will appear on the computer.

The chip has two partitions: Bootloader and user program. This procedure allows you to switch modes.

### Reset

1.  Connect the Seeed Studio XIAO RP2040 to your computer.
2.  Press the "R" pins once.
Note: The built-in programmable Single-colour LEDs are reversed compared to an Arduino; the pin has to be pulled low to enable them.
