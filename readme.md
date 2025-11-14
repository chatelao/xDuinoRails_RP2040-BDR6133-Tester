# XDuinoRails Motor Driver

A powerful, closed-loop motor driver library for Arduino and PlatformIO, designed for controlling DC motors with high precision. This library is ideal for projects like model trains, robotics, or any application that requires smooth and accurate speed control.

At its core, this library uses back-EMF (BEMF) sensing to measure the motor's actual speed, and then uses a Proportional-Integral (PI) controller to finely adjust the motor's power via PWM. This ensures the motor maintains a constant speed under varying loads.

## Features

*   **Closed-Loop Speed Control:** Uses a PI controller for precise and stable motor speed.
*   **Back-EMF Sensing:** Accurately measures motor speed without the need for an external encoder.
*   **Advanced Signal Filtering:** A two-stage filter (EMA and Kalman) provides a clean and stable speed reading, even with noisy motors.
*   **Easy to Use:** A simple and intuitive API makes it easy to get your motor running in minutes.
*   **Arduino and PlatformIO Compatible:** Works seamlessly with both development environments.
*   **Hardware Accelerated:** Includes an optional, high-performance mode for the RP2040 that uses hardware timers for even more precise control.

## Installation

### Arduino IDE

1.  Download this repository as a `.zip` file by clicking the "Code" button and selecting "Download ZIP".
2.  In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`
3.  Select the downloaded `.zip` file.
4.  The library will now be available in your `Sketch` -> `Include Library` menu.

### PlatformIO

1.  Add this repository to the `lib_deps` section of your `platformio.ini` file:
    ```ini
    lib_deps =
        https://github.com/chatelao/xDuinoRails_RP2040-BDR6133-Tester.git
        denyssene/SimpleKalmanFilter
    ```
2.  PlatformIO will automatically download and install the library the next time you build your project.

## Wiring Diagram

This diagram shows a typical wiring setup using a XIAO SEED RP2040 and a BDR-6133 motor driver.

```
                     +----------------------+      +----------------------+      +---------------+
                     |  XIAO SEED RP2040    |      |     BDR-6133         |      |     Motor     |
                     |      (Top View)      |      |    Motor Driver      |      |               |
                     +----------------------+      +----------------------+      +---------------+
                     | D0/A0            5v  |      |                      |      |               |
                     | D1/A1            GND |      |                      |      |               |
        (BEMF B) <---| D2/A2            3v3 | <----+                 OutB |=====>| B (-> bEMF B) |
        (BEMF A) <---| D3/A3            D10 | <----+                 OutA |=====>| A (-> bEMF A) |
        (DCC-RX) ----| D4               D9  |      |                      |      |               |
       (ACC-ACK) ----| D5               D8  |----->| InB                  |      |               |
    (Railcom-TX) ----| D6               D7  |----->| InA                  |      |               |
                     +----------------------+      +----------------------+      +---------------+
```

## Getting Started

This simple example demonstrates how to get your motor up and running.

```cpp
/**
 * @file getting_started.ino
 * @brief A simple example to demonstrate the basic functionality of the XDuinoRails_MotorDriver library.
 */

#include <Arduino.h>
#include <XDuinoRails_MotorDriver.h>

// 1. Define Pin Connections
// These pins are for the XIAO SEED RP2040, but you can change them for your board.
const int MOTOR_PWM_A_PIN = 7;  // Connect to BDR-6133 InA
const int MOTOR_PWM_B_PIN = 8;  // Connect to BDR-6133 InB
const int MOTOR_BEMF_A_PIN = A3; // Connect to the middle of the OutA voltage divider
const int MOTOR_BEMF_B_PIN = A2; // Connect to the middle of the OutB voltage divider

// 2. Create an instance of the motor driver
XDuinoRails_MotorDriver motor(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

void setup() {
  Serial.begin(115200);

  // 3. Initialize the motor driver
  motor.begin();

  // 4. Set a target speed
  // The speed is in "pulses per second" (PPS). The valid range depends on your motor.
  // A value between 50 and 200 is a good starting point.
  motor.setTargetSpeed(100);
}

void loop() {
  // It is crucial to call motor.update() in every loop iteration.
  // This function handles the BEMF measurement and PI controller logic.
  motor.update();

  // You can also print the measured speed for debugging
  Serial.print("Target Speed: ");
  Serial.print(motor.getTargetSpeed());
  Serial.print(" PPS, Measured Speed: ");
  Serial.print(motor.getMeasuredSpeedPPS());
  Serial.println(" PPS");

  delay(100); // Delay for readability
}
```

You can find this and other examples in the `examples` folder of this repository.

## API Reference

### `XDuinoRails_MotorDriver(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin)`

The constructor creates a new motor driver instance. You must provide the four pins used to control the motor.

### `void begin()`

Initializes the motor driver and the associated hardware. Call this once in your `setup()` function.

### `void update()`

This is the heart of the library. It performs the BEMF measurement, runs the PI controller, and updates the motor's PWM signal. **You must call this function in every iteration of your `loop()` for the library to work correctly.**

### `void setTargetSpeed(int speed)`

Sets the desired motor speed in "pulses per second" (PPS). The library will automatically adjust the motor's power to reach and maintain this speed. Set the speed to `0` to stop the motor.

### `int getTargetSpeed() const`

Returns the current target speed.

### `float getMeasuredSpeedPPS() const`

Returns the most recent speed measurement from the BEMF sensor, after filtering.

### `void setDirection(bool forward)`

Sets the motor's direction. `true` for forward, `false` for reverse.

### `bool getDirection() const`

Returns the current direction of the motor.

### `void setAcceleration(float rate)`

Sets the acceleration rate in speed units (PPS) per second. For example, a rate of 50 means the motor's speed will increase by 50 PPS every second until it reaches the target speed. Set to `0` to disable acceleration.

### `void setDeceleration(float rate)`

Sets the deceleration rate in speed units (PPS) per second. For example, a rate of 100 means the motor's speed will decrease by 100 PPS every second until it reaches the target speed. Set to `0` to disable deceleration.

### `void setStartupKick(int pwm, int duration_ms)`

Configures a "startup kick" to help overcome the motor's initial friction. It applies a direct PWM value for a short duration when the motor starts from a standstill.

- `pwm`: The PWM value to apply (0-255).
- `duration_ms`: The duration of the kick in milliseconds.

## How It Works (Advanced)

For those interested in the technical details, the library follows this control loop:

1.  **BEMF Measurement:** During the "off" phase of each PWM cycle, the motor is briefly disconnected from power, and the BEMF voltage is read from the motor terminals.
2.  **Signal Filtering:** The raw BEMF signal is passed through a flexible processing pipeline. By default, this pipeline contains an Exponential Moving Average (EMA) filter followed by a Kalman filter, but it can be customized with any number of user-defined filter functions.
3.  **PI Control:** A Proportional-Integral (PI) controller calculates the error between the target speed and the measured speed, and then computes a new PWM value to correct for this error. This controller can also be replaced with a custom user function.
4.  **PWM Update:** The PWM signal sent to the motor driver is updated with the new value from the PI controller.

This entire process happens automatically within the `update()` function.

## Hardware Abstraction Layer (HAL)

To support multiple microcontrollers and enable high-performance, hardware-accelerated features, the library uses a dedicated Hardware Abstraction Layer (HAL) provided by the `xDuinoRails_MotorControl_bEMF` library. This library handles the low-level, platform-specific code for PWM generation and BEMF sensing.

For a detailed explanation of how these two libraries work together, please see the [Integration Documentation](docs/INTEGRATION.md).

## Advanced Extensibility

For advanced users, the library now offers a high degree of flexibility by allowing you to replace core components with your own custom logic:

*   **Customizable Filter Pipeline:** The default EMA and Kalman filters can be replaced or augmented with any number of custom filter functions. You can create complex, multi-stage signal processing chains tailored to your specific motor and application.
*   **Custom Speed Controller:** The internal PI controller, along with its associated logic for acceleration and stall detection, can be completely replaced by your own controller function. This allows you to implement any control strategy you can imagine, from a simple proportional controller to a more complex PID or fuzzy logic controller.

Check out the `CustomControlLoop` example to see a practical demonstration of how to use these powerful features.

![Control Loop Diagram](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/chatelao/xDuinoRails_RP2040-BDR6133-Tester/main/docs/control_loop.puml)
