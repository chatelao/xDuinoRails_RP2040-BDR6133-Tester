# Integration with xDuinoRails_MotorControl_bEMF

This document describes the integration between the main `XDuinoRails_MotorControl` library and the `xDuinoRails_MotorControl_bEMF` library.

## Overview

The `XDuinoRails_MotorControl` library is responsible for the high-level motor control logic, including:

-   Proportional-Integral (PI) control
-   Speed ramping (acceleration/deceleration)
-   Stall detection
-   BEMF signal filtering pipeline

The `xDuinoRails_MotorControl_bEMF` library provides a Hardware Abstraction Layer (HAL) for low-level motor control and BEMF sensing. This library is responsible for the hardware-specific implementation of:

-   PWM generation
-   BEMF ADC reading
-   Hardware timers

This separation of concerns allows the high-level control logic to remain platform-independent, while the low-level HAL can be optimized for different microcontrollers.

## Dependency Management

The `xDuinoRails_MotorControl_bEMF` library is included as a dependency in the `platformio.ini` file for specific build environments that support hardware-accelerated features. It is pulled directly from its GitHub repository:

```ini
lib_deps =
    ...
    https://github.com/chatelao/xDuinoRails_MotorControl_bEMF.git#main
    ...
```

## Conditional Compilation

The integration is managed using C++ preprocessor directives. The HAL is only compiled and used when one of the following flags is defined in the build environment:

-   `USE_RP2040_LOWLEVEL`
-   `ARDUINO_ARCH_STM32`
-   `ARDUINO_ARCH_ESP32`
-   `ARDUINO_ARCH_SAMD`

The inclusion of the HAL header file is guarded as follows in `XDuinoRails_MotorDriver.cpp`:

```cpp
#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAMD)
#include <motor_control_hal.h>
#endif
```

## HAL Functions

The `XDuinoRails_MotorDriver` class calls the following functions provided by the HAL:

-   `hal_motor_init(pwmAPin, pwmBPin, bemfAPin, bemfBPin, callback)`: This function is called in the `begin()` method to initialize the hardware peripherals (timers, ADC, etc.) and register a callback function.
-   `hal_motor_set_pwm(pwm, forward)`: This function is called in the `update()` method to set the motor's PWM duty cycle and direction.
-   `hal_read_and_process_bemf()`: For certain platforms like the ESP32, this function is called in the main `update()` loop to trigger BEMF readings.

## Callback Mechanism

The HAL communicates BEMF data back to the `XDuinoRails_MotorControl` library via a callback mechanism.

1.  During initialization, `XDuinoRails_MotorDriver` passes a static wrapper function, `on_bemf_update_wrapper`, to `hal_motor_init`.
2.  The HAL's hardware interrupt or timer calls this wrapper when a new BEMF measurement is available.
3.  The static wrapper function calls the `on_bemf_update` method on the active `XDuinoRails_MotorDriver_Impl` instance.
4.  The `on_bemf_update` method processes the raw BEMF value through the configured filter pipeline and updates the commutation pulse count, which is then used to calculate the motor's speed.

This design decouples the high-level control logic from the low-level, hardware-specific timing of the BEMF measurement, enabling precise, hardware-timed readings without blocking the main control loop.
