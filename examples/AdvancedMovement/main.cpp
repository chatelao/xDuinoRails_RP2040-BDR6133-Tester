/**
 * @file AdvancedMovement.ino
 * @brief Demonstrates the use of acceleration, deceleration, and startup kick.
 */

#include <Arduino.h>
#include <XDuinoRails_MotorDriver.h>

// 1. Define Pin Connections
const int MOTOR_PWM_A_PIN = 7;
const int MOTOR_PWM_B_PIN = 8;
const int MOTOR_BEMF_A_PIN = A3;
const int MOTOR_BEMF_B_PIN = A2;

// 2. Create an instance of the motor driver
XDuinoRails_MotorDriver motor(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial connection
  }

  // 3. Initialize the motor driver
  motor.begin();

  // 4. Configure advanced movement parameters
  // Set acceleration to 50 speed units per second.
  motor.setAcceleration(50);
  // Set deceleration to 100 speed units per second (faster braking).
  motor.setDeceleration(100);
  // Give the motor a "kick" of 100 PWM for 50ms to overcome initial friction.
  motor.setStartupKick(100, 50);

  Serial.println("Starting advanced movement demo...");

  // 5. Set an initial target speed
  motor.setTargetSpeed(200);
}

void loop() {
  // It is crucial to call motor.update() in every loop iteration.
  motor.update();

  // Print status every 250ms
  static unsigned long last_print_ms = 0;
  if (millis() - last_print_ms > 250) {
    last_print_ms = millis();
    Serial.print("Target Speed: ");
    Serial.print(motor.getTargetSpeed());
    Serial.print(" PPS, Measured Speed: ");
    Serial.print(motor.getMeasuredSpeedPPS());
    Serial.println(" PPS");
  }

  // Reverse the direction every 10 seconds
  static unsigned long last_reverse_ms = 0;
  if(millis() - last_reverse_ms > 10000) {
    last_reverse_ms = millis();
    int current_target = motor.getTargetSpeed();
    motor.setTargetSpeed(0); // Stop first

    // Simple state machine to alternate between stopped and moving
    if (current_target > 0) {
      Serial.println("Stopping motor...");
      motor.setTargetSpeed(0);
    } else {
      Serial.println("Starting motor...");
      motor.setTargetSpeed(200);
    }
  }
}
