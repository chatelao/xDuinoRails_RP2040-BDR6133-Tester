#include <xDuinoRails_MotorDriver.h>
#include <Adafruit_NeoPixel.h>

//== Pin Definitions ==
const int INA_PIN = D7;
const int INB_PIN = D8;
const int BEMFA_PIN = A3;
const int BEMFB_PIN = A2;

//== RGB LED Definitions ==
const int NEOPIXEL_POWER_PIN = 11;
const int NEOPIXEL_PIN = 12;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//== LED Color Definitions ==
const uint32_t COLOR_BLUE = pixel.Color(0, 0, 255);
const uint32_t COLOR_GREEN = pixel.Color(0, 255, 0);
const uint32_t COLOR_RED = pixel.Color(255, 0, 0);
const uint32_t COLOR_YELLOW = pixel.Color(255, 255, 0);
const uint32_t COLOR_PINK = pixel.Color(255, 105, 180);
const uint32_t COLOR_OFF = pixel.Color(0, 0, 0);

// Instantiate the motor driver library
XDuinoRails_MotorDriver motor(INA_PIN, INB_PIN, BEMFA_PIN, BEMFB_PIN);

// State machine for the example sketch
enum ExampleState {
    ACCELERATE,
    COAST_HIGH,
    DECELERATE,
    COAST_LOW,
    STOP,
    CHANGE_DIRECTION
};
ExampleState currentState = ACCELERATE;
unsigned long stateStartTime = 0;

void setup() {
    Serial.begin(9600);

    // Initialize the NeoPixel
    pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_POWER_PIN, HIGH);
    pixel.begin();
    pixel.setBrightness(50);
    pixel.show();

    // Initialize the motor driver
    motor.begin();

    // Start the initial acceleration
    motor.setTargetSpeed(255, 5000); // Ramp to full speed over 5 seconds
    stateStartTime = millis();
}

void loop() {
    // The library's update() method must be called on every loop iteration
    // to handle the internal motor control logic (PI controller, BEMF, etc.)
    motor.update();

    // Example state machine to demonstrate the library's API
    unsigned long timeInState = millis() - stateStartTime;

    switch (currentState) {
        case ACCELERATE:
            // Ramping is handled by the library, we just wait for it to finish
            if (!motor.isMoving() && timeInState > 500) { // Give it a moment to start
                 // Stall or error condition, for now just stop
                 currentState = STOP;
                 motor.coast();
                 stateStartTime = millis();
            } else if (motor.getCurrentSpeed() >= 250) {
                currentState = COAST_HIGH;
                stateStartTime = millis();
            }
            break;

        case COAST_HIGH:
            if (timeInState >= 3000) {
                currentState = DECELERATE;
                stateStartTime = millis();
                motor.setTargetSpeed(25, 3000); // Ramp down to 10% speed over 3 seconds
            }
            break;

        case DECELERATE:
             if (motor.getCurrentSpeed() <= 30) {
                currentState = COAST_LOW;
                stateStartTime = millis();
            }
            break;

        case COAST_LOW:
            if (timeInState >= 3000) {
                currentState = STOP;
                stateStartTime = millis();
                motor.setTargetSpeed(0, 1000); // Ramp to stop over 1 second
            }
            break;

        case STOP:
            if (timeInState >= 2000) {
                currentState = CHANGE_DIRECTION;
                stateStartTime = millis();
                motor.changeDirection();
            }
            break;

        case CHANGE_DIRECTION:
            if (timeInState >= 500) {
                currentState = ACCELERATE;
                stateStartTime = millis();
                motor.setTargetSpeed(255, 5000); // Ramp back up
            }
            break;
    }

    // Update status light (simplified for example)
    if (motor.isMoving()) {
        pixel.setPixelColor(0, COLOR_GREEN);
    } else {
        pixel.setPixelColor(0, COLOR_RED);
    }
    pixel.show();
}
