# User Manual

## Overview
The XDuinoRails Motor Driver library provides closed-loop DC motor control using Back-EMF (BEMF) feedback. It is designed for model railroad applications.

## Features
- **Closed-Loop Control**: Maintains constant speed under varying loads.
- **Rangiermodus**: Precise low-speed control with automatic gain scheduling.
- **Inertia Simulation**: Configurable acceleration and deceleration.
- **Startup Kick**: Overcomes motor stiction.
- **Stall Detection**: Protects motor and mechanics.

## Configuration
- **PI Gains**: `setPIgains(kp, ki)`
- **Acceleration**: `setAcceleration(rate)`
- **Startup Kick**: `setStartupKick(pwm, duration)`

## Supported Hardware
- RP2040 (Seeed XIAO)
- SAMD21 (Qt Py M0)
- STM32 (Nucleo F446RE)
- ESP32
