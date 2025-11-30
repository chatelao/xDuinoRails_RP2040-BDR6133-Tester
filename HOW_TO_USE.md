# How To Use

## Installation
1. Install PlatformIO.
2. Add `XDuinoRails_MotorControl` to your `platformio.ini` dependencies.

## Basic Usage
1. Include the header: `#include <XDuinoRails_MotorControl.h>`
2. Instantiate the driver: `XDuinoRails_MotorDriver driver(pwmA, pwmB, bemfA, bemfB);`
3. Call `driver.begin()` in `setup()`.
4. Call `driver.update()` in `loop()`.
5. Set speed: `driver.setTargetSpeed(speed);`

See `examples/` for complete sketches.
