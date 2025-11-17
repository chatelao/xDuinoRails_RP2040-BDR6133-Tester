/**
 * @file SineSpeed5f5b2s.ino
 * @brief Drives a motor with a sine wave speed input, alternating between forward and backward motion.
 *
 * This example demonstrates how to:
 * 1. Create a timed sequence for motor control.
 * 2. Use a sine wave to generate a smooth, non-linear speed profile.
 * 3. Alternate the motor's direction.
 *
 * The motor runs:
 * - 5 seconds forward with a sine wave speed profile (2 full waves).
 * - 5 seconds backward with the same sine wave profile.
 * - 1 second stopped.
 * This cycle repeats indefinitely.
 */

#include <Arduino.h>
#include <XDuinoRails_MotorDriver.h>
#include <math.h> // For sin()

// Define Pin Connections (using XIAO SEED RP2040 defaults)
const int MOTOR_PWM_A_PIN = 7;
const int MOTOR_PWM_B_PIN = 8;
const int MOTOR_BEMF_A_PIN = A3;
const int MOTOR_BEMF_B_PIN = A2;

// Create an instance of the motor driver
XDuinoRails_MotorDriver motor(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

// --- Speed Profile Configuration ---
const float MIN_POWER_PERCENT = 5.0;
const float MAX_POWER_PERCENT = 80.0;
// Assuming a nominal max speed of 200 PPS for calculation.
// You may need to adjust this based on your specific motor's characteristics.
const float NOMINAL_MAX_SPEED_PPS = 200.0;

// Convert percentages to PPS
const float MIN_SPEED_PPS = NOMINAL_MAX_SPEED_PPS * (MIN_POWER_PERCENT / 100.0);
const float MAX_SPEED_PPS = NOMINAL_MAX_SPEED_PPS * (MAX_POWER_PERCENT / 100.0);

// Sine wave parameters
const float AMPLITUDE = (MAX_SPEED_PPS - MIN_SPEED_PPS) / 2.0;
const float OFFSET = MIN_SPEED_PPS + AMPLITUDE;
const float WAVE_DURATION_MS = 2500.0; // 2.5 seconds per wave
const float FREQUENCY = 1.0 / (WAVE_DURATION_MS / 1000.0); // Frequency in Hz

// --- Timing Configuration ---
const unsigned long FORWARD_DURATION_MS = 5000;
const unsigned long BACKWARD_DURATION_MS = 5000;
const unsigned long STOP_DURATION_MS = 1000;

// --- State Machine ---
enum MotorState {
  FORWARD,
  BACKWARD,
  STOP
};
MotorState currentState = STOP;
unsigned long stateStartTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("Sine Wave Speed Control Example");

  motor.begin();
  stateStartTime = millis(); // Initialize the state timer
}

void loop() {
  // Always call the motor update function in every loop
  motor.update();

  unsigned long currentTime = millis();
  unsigned long elapsedTimeInState = currentTime - stateStartTime;

  // --- State Machine Logic ---
  switch (currentState) {
    case FORWARD:
      // Check if it's time to switch to the backward state
      if (elapsedTimeInState >= FORWARD_DURATION_MS) {
        currentState = BACKWARD;
        stateStartTime = currentTime;
        Serial.println("Switching to BACKWARD");
      } else {
        // Calculate the current position in the sine wave (0.0 to 1.0)
        float time_sec = (elapsedTimeInState % (unsigned long)WAVE_DURATION_MS) / 1000.0;
        // Calculate the speed using the sine function
        float speed = OFFSET + AMPLITUDE * sin(2.0 * PI * FREQUENCY * time_sec);
        motor.setTargetSpeed(speed);
      }
      break;

    case BACKWARD:
      // Check if it's time to switch to the stop state
      if (elapsedTimeInState >= BACKWARD_DURATION_MS) {
        currentState = STOP;
        stateStartTime = currentTime;
        motor.setTargetSpeed(0); // Stop the motor
        Serial.println("Switching to STOP");
      } else {
        // Calculate the current position in the sine wave
        float time_sec = (elapsedTimeInState % (unsigned long)WAVE_DURATION_MS) / 1000.0;
        // Calculate speed and set it as negative for backward motion
        float speed = OFFSET + AMPLITUDE * sin(2.0 * PI * FREQUENCY * time_sec);
        motor.setTargetSpeed(-speed);
      }
      break;

    case STOP:
      // Check if it's time to switch to the forward state and restart the cycle
      if (elapsedTimeInState >= STOP_DURATION_MS) {
        currentState = FORWARD;
        stateStartTime = currentTime;
        Serial.println("Switching to FORWARD");
      }
      // Motor speed is already set to 0 in the previous state transition
      break;
  }

  // Optional: Print status periodically for debugging
  static unsigned long lastPrintTime = 0;
  if (currentTime - lastPrintTime > 100) { // Print every 100ms
    Serial.print("State: ");
    Serial.print(currentState == FORWARD ? "FWD" : (currentState == BACKWARD ? "BCK" : "STP"));
    Serial.print(", Target Speed: ");
    Serial.print(motor.getTargetSpeed());
    Serial.print(", Measured Speed: ");
    Serial.println(motor.getMeasuredSpeedPPS());
    lastPrintTime = currentTime;
  }
}
