#include <Arduino.h>
#include <XDuinoRails_MotorDriver.h>

// Pin Definitions for Nucleo F446RE
#define PWM_A_PIN PA0
#define PWM_B_PIN PA1
#define BEMF_A_PIN PC0
#define BEMF_B_PIN PC1

XDuinoRails_MotorDriver motor(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

void setup() {
  Serial.begin(115200);
  motor.begin();
  motor.setTargetSpeed(100); // Set a moderate speed for testing
}

void loop() {
  motor.update();

  // Print motor speed every 250ms
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 250) {
    Serial.print("Target Speed: ");
    Serial.print(motor.getTargetSpeed());
    Serial.print("\tMeasured Speed: ");
    Serial.println(motor.getMeasuredSpeedPPS());
    last_print_time = millis();
  }
}
