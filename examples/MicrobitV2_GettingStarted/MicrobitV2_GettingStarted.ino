#include "XDuinoRails_MotorControl.h"

// This is a basic example of how to use the XDuinoRails_MotorControl library
// with the Microbit v2.21.

// Define the pins for the motor driver.
// These pins will need to be connected to your motor driver.
const int PWM_A_PIN = 0;
const int PWM_B_PIN = 1;
const int BEMF_A_PIN = 2;
const int BEMF_B_PIN = 3;

// Create a new motor driver instance.
XDuinoRails_MotorDriver motor(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

void setup() {
  // Initialize the motor driver.
  motor.begin();
}

void loop() {
  // Set the motor to full speed forward.
  motor.setTargetSpeed(255);
  motor.update();
  delay(1000);

  // Set the motor to half speed reverse.
  motor.setTargetSpeed(-128);
  motor.update();
  delay(1000);
}
