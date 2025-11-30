/**
 * @file getting_started.ino
 * @brief A simple example to demonstrate the basic functionality of the XDuinoRails_MotorDriver library.
 *
 * This sketch shows how to:
 * 1. Include the library.
 * 2. Define the pin connections for your microcontroller.
 * 3. Create an instance of the motor driver.
 * 4. Initialize the driver in the `setup()` function.
 * 5. Set a target speed and let the motor run for a few seconds.
 * 6. Stop the motor.
 *
 * For this example to work, you must have the XDuinoRails library installed
 * and your hardware wired correctly. Please see the README.md file for the
 * wiring diagram.
 */

#include <Arduino.h>

// 1. Include the library
#include <XDuinoRails_MotorControl.h>

// 2. Define Pin Connections
// These pins are for the XIAO SEED RP2040, but you can change them for your board.
const int MOTOR_PWM_A_PIN = 7;  // Connect to BDR-6133 InA
const int MOTOR_PWM_B_PIN = 8;  // Connect to BDR-6133 InB
const int MOTOR_BEMF_A_PIN = A3; // Connect to the middle of the OutA voltage divider
const int MOTOR_BEMF_B_PIN = A2; // Connect to the middle of the OutB voltage divider

// 3. Create an instance of the motor driver
XDuinoRails_MotorDriver motor(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

void setup() {
  // Start the serial communication for debugging (optional)
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for the serial port to connect.
  }

  Serial.println("Motor Driver Getting Started Example");

  // 4. Initialize the motor driver
  motor.begin();

  // 5. Set an initial target speed
  // The speed is in "pulses per second" (PPS). The valid range depends on your motor.
  // A value between 50 and 200 is often a good starting point.
  int targetSpeed = 100; // Let's set a moderate speed
  Serial.print("Setting target speed to ");
  Serial.print(targetSpeed);
  Serial.println(" PPS");
  motor.setTargetSpeed(targetSpeed);
}

void loop() {
  // The motor driver's internal logic (like the PI controller and BEMF measurement)
  // is handled in the update() function. It is crucial to call this in every
  // loop iteration for the controller to work correctly.
  motor.update();

  // You can also print the measured speed for debugging or visualization
  // Note: Printing too frequently can slow down the loop.
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { // Print every 100ms
    Serial.print("Target Speed: ");
    Serial.print(motor.getTargetSpeed());
    Serial.print(" PPS, Measured Speed: ");
    Serial.print(motor.getMeasuredSpeedPPS());
    Serial.println(" PPS");
    lastPrintTime = millis();
  }

  // This example will run the motor for 10 seconds, then stop it.
  static bool motorStopped = false;
  if (!motorStopped && millis() > 10000) {
    Serial.println("10 seconds have passed. Stopping the motor.");
    motor.setTargetSpeed(0); // Set speed to 0 to stop
    motorStopped = true;
  }
}
