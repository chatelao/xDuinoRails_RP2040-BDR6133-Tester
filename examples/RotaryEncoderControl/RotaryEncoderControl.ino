/**
 * @file RotaryEncoderControl.cpp
 * @brief An example of controlling motor speed and direction with a rotary encoder.
 *
 * This sketch demonstrates how to use a standard KY-040 rotary encoder to
 * control the motor's speed and its integrated push-button to stop the motor or
 * change its direction.
 *
 * ## How it Works
 * - Turning the encoder knob increases or decreases the motor's target speed.
 *   One full rotation of a 24-detent encoder will ramp the speed from 0 to 100%.
 * - Pressing the encoder's push-button has two functions:
 *   1. If the motor is currently moving, it acts as an emergency stop, setting
 *      the target speed to 0.
 *   2. If the motor is stopped, it toggles the direction of travel for the next
 *      time the motor starts (Forward -> Reverse -> Forward).
 *
 * ## Hardware Setup
 * Connect the rotary encoder to the XIAO RP2040 as follows:
 * - Encoder CLK pin  -> D0
 * - Encoder DT pin   -> D1
 * - Encoder SW pin   -> D9
 * - Encoder + pin    -> 3.3V
 * - Encoder GND pin  -> GND
 *
 * No external pull-up resistors are needed as the microcontroller's internal
 * pull-ups are used.
 */

#include <Arduino.h>
#include <XDuinoRails_MotorDriver.h>
#include <RotaryEncoder.h>

// --- Pin Definitions ---
// Define the pins for the motor driver.
const int MOTOR_PWM_A_PIN = 7;
const int MOTOR_PWM_B_PIN = 8;
const int MOTOR_BEMF_A_PIN = A3;
const int MOTOR_BEMF_B_PIN = A2;

// Define the pins for the rotary encoder.
const int ENCODER_PIN_A = 0;      // CLK pin
const int ENCODER_PIN_B = 1;      // DT pin
const int ENCODER_SWITCH_PIN = 9; // SW pin

// --- Motor and Encoder Instances ---
// Create an instance of the motor driver.
XDuinoRails_MotorDriver motor(MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN, MOTOR_BEMF_A_PIN, MOTOR_BEMF_B_PIN);

// Create an instance of the rotary encoder using polling.
// For higher responsiveness, you could use interrupts. See the library's documentation.
RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// --- Control Logic Parameters ---
const long ENCODER_MIN_POSITION = 0;
const long ENCODER_MAX_POSITION = 24; // Assumes a standard 24-detent encoder for one full turn.
const int MAX_SPEED_PPS = 200;        // The motor speed (in Pulses Per Second) at 100% encoder turn.
bool motorDirection = true;           // Current motor direction: true for forward, false for reverse.

// --- Button Debouncing ---
// Variables to handle button debouncing to prevent multiple triggers from a single press.
unsigned long lastButtonPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50 milliseconds

void setup() {
  // Start serial communication for debugging output.
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for the serial port to connect. Needed for native USB port only.
  }
  Serial.println("Rotary Encoder Motor Control Example");
  Serial.println("Turn the knob to change speed, press it to stop or change direction.");

  // Initialize the motor driver.
  motor.begin();
  motor.setDirection(motorDirection);

  // Set up the encoder's switch pin with an internal pull-up resistor.
  // This means the pin will be HIGH by default and LOW when the button is pressed.
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);

  // Start the encoder at the minimum position.
  encoder.setPosition(ENCODER_MIN_POSITION);
}

void loop() {
  // These two functions must be called in every loop iteration for the components to work.
  motor.update();   // Handles the motor's PI controller and BEMF measurement.
  encoder.tick();   // Polls the encoder for any new movement.

  // --- Encoder Logic for Speed Control ---
  long newPosition = encoder.getPosition();

  // Constrain the encoder's value to stay within our defined min/max range.
  if (newPosition < ENCODER_MIN_POSITION) {
    newPosition = ENCODER_MIN_POSITION;
    encoder.setPosition(newPosition);
  } else if (newPosition > ENCODER_MAX_POSITION) {
    newPosition = ENCODER_MAX_POSITION;
    encoder.setPosition(newPosition);
  }

  // Map the constrained encoder position (e.g., 0-24) to the desired motor speed range (e.g., 0-200 PPS).
  int newSpeed = map(newPosition, ENCODER_MIN_POSITION, ENCODER_MAX_POSITION, 0, MAX_SPEED_PPS);
  if (newSpeed != motor.getTargetSpeed()) {
    motor.setTargetSpeed(newSpeed);
    Serial.print("New Speed: ");
    Serial.println(newSpeed);
  }

  // --- Button Logic for Stop/Direction Control ---
  // Check if the button is pressed (pin is LOW) and if enough time has passed since the last press.
  if (digitalRead(ENCODER_SWITCH_PIN) == LOW && (millis() - lastButtonPressTime) > DEBOUNCE_DELAY) {
    if (motor.getTargetSpeed() > 0) {
      // If the motor is currently moving, stop it and reset the encoder position.
      motor.setTargetSpeed(0);
      encoder.setPosition(ENCODER_MIN_POSITION);
      Serial.println("Motor stopped.");
    } else {
      // If the motor is stopped, toggle the direction for the next run.
      motorDirection = !motorDirection;
      motor.setDirection(motorDirection);
      Serial.print("Direction changed to: ");
      Serial.println(motorDirection ? "Forward" : "Reverse");
    }
    // Record the time of this press to handle debouncing.
    lastButtonPressTime = millis();
  }
}
