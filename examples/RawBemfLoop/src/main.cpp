#include <xDuinoRails_MotorControl.h>

// Pin Definitions
const int PWM_A_PIN = D7;
const int PWM_B_PIN = D8;
const int BEMF_A_PIN = A3;
const int BEMF_B_PIN = A2;

xDuinoRails_MotorControl motor;

void setup() {
  Serial.begin(9600);
  motor.begin(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

  // Set the motor direction and a raw PWM duty cycle.
  // This bypasses the PI controller and the high-level state machine.
  motor.setDirection(true);
  motor.setRawPwm(150); // Set a moderate PWM value (0-255)
}

void loop() {
  // In this example, we are not using the high-level motor.loop() function.
  // Instead, we are directly accessing the low-level BEMF data.

  int head;
  int size;
  const volatile int* bemf_buffer = motor.getBemfBuffer(&head, &size);

  // Print the entire BEMF buffer to the serial plotter.
  // This is useful for visualizing the commutation ripple and debugging the sensor.
  Serial.print("BEMF Buffer: ");
  for (int i = 0; i < size; i++) {
    Serial.print(bemf_buffer[i]);
    if (i == head) {
      Serial.print("*"); // Mark the head of the circular buffer
    }
    Serial.print(" ");
  }
  Serial.println();

  delay(100);
}
