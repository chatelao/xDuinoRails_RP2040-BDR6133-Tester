#include <XDuinoRails_MotorControl.h>

// This example demonstrates the use of the optional watchdog timer.
//
// The watchdog is a safety feature. If you enable it, the motor will automatically
// stop if it does not receive a new speed command within a specified timeout period.
// This can be useful to prevent the motor from running indefinitely if the
// controlling program crashes or loses connection.
//
// For detailed wiring instructions, see the main README.md in the root of the library.
// - RP2040 PWM Pins: Any GPIO pins can be used.
// - RP2040 BEMF Pins: Any ADC-capable pins (A0-A3) can be used.
//
// This example assumes a XIAO RP2040, but is easily adaptable.
#define PWM_A_PIN 0
#define PWM_B_PIN 1
#define BEMF_A_PIN 28 // A2 on XIAO RP2040
#define BEMF_B_PIN 29 // A3 on XIAO RP2040

XDuinoRails_MotorDriver driver(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Watchdog Example");

  driver.begin();

  // Enable the watchdog with a 500ms timeout.
  // If setTargetSpeed() is not called at least once every 500ms, the motor
  // will automatically stop.
  // Setting the timeout to 0 disables the watchdog.
  driver.setWatchdogTimeout(500);

  // Set the motor to a moderate speed
  driver.setTargetSpeed(150);
}

void loop() {
  driver.update();

  // In a real application, you would periodically call setTargetSpeed()
  // to keep the watchdog from triggering.
  // For this example, we will just let it time out.

  // After 500ms, the watchdog will trigger and the target speed will be set to 0.
  if (millis() > 1000 && driver.getTargetSpeed() == 0) {
    Serial.println("Watchdog has triggered and stopped the motor.");
    Serial.println("To restart, send a new speed command.");
    // To prove the point, let's set a new speed. It will run for 500ms and stop again.
    driver.setTargetSpeed(100);
    delay(1000); // Wait long enough for it to time out again
  }
}
