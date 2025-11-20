# Technical Debts

## RP2040 Arduino Implementation
The default Arduino implementation for RP2040 (when `USE_RP2040_LOWLEVEL` is not defined) performs the control loop in the main `update()` method rather than in a hardware-timed interrupt. This ensures safety and simplicity but may result in less precise control timing compared to the Low-Level HAL implementation if the main loop is blocked.

## Dependency Management
The HAL library `xDuinoRails_MotorControl_bEMF` is pulled in via Git URL in `platformio.ini` and `library.json`. A more stable release versioning strategy would be preferable.
