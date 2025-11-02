#include <xDuinoRails_MotorControl.h>
#include <Adafruit_NeoPixel.h>

// Pin Definitions
const int PWM_A_PIN = D7;
const int PWM_B_PIN = D8;
const int BEMF_A_PIN = A3;
const int BEMF_B_PIN = A2;

// RGB LED Definitions
const int NEOPIXEL_POWER_PIN = 11;
const int NEOPIXEL_PIN = 12;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED Color Definitions
const uint32_t COLOR_BLUE = pixel.Color(0, 0, 255);
const uint32_t COLOR_GREEN = pixel.Color(0, 255, 0);
const uint32_t COLOR_RED = pixel.Color(255, 0, 0);
const uint32_t COLOR_YELLOW = pixel.Color(255, 255, 0);
const uint32_t COLOR_PINK = pixel.Color(255, 105, 180);
const uint32_t COLOR_OFF = pixel.Color(0, 0, 0);

xDuinoRails_MotorControl motor;

void setup() {
  Serial.begin(9600);

  // Initialize the NeoPixel
  pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(50);
  pixel.show();

  motor.begin(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

  // To start the automatic test pattern, we can set an initial state.
  // Let's assume the library's loop() function will handle the pattern.
  // We'll need to add a public method to the library to start the pattern.
  // For now, let's just set a speed and direction.
  motor.setDirection(true);
  motor.setSpeed(100);
}

void loop() {
  // The motor.loop() function will handle the automatic test pattern.
  motor.loop();

  // You can also add your own logic here, for example, to print the speed.
  Serial.print("Speed (PPS): ");
  Serial.println(motor.getSpeedPPS());

  // Update the status light based on motor state (example)
  // This logic could be more sophisticated, checking the motor's internal state
  // if the library exposes it.
  if (motor.getSpeedPPS() > 0) {
    pixel.setPixelColor(0, COLOR_GREEN);
  } else {
    pixel.setPixelColor(0, COLOR_BLUE);
  }
  pixel.show();

  delay(100);
}
