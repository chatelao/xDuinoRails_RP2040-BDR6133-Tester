#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <XDuinoRails_MotorControl.h>

// Pin Definitions
const int pwmAPin = D7;
const int pwmBPin = D8;
const int bemfAPin = A3;
const int bemfBPin = A2;

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

// Motor Control Parameters
const int max_speed = 255;
const int rangiermodus_speed_threshold = max_speed * 0.1;

XDuinoRails_MotorDriver motor(pwmAPin, pwmBPin, bemfAPin, bemfBPin);

// State Machine for Test Pattern
enum MotorState {
    MOTOR_DITHER,
    RAMP_UP,
    COAST_HIGH,
    RAMP_DOWN,
    COAST_LOW,
    STOP,
    CHANGE_DIRECTION,
    MOTOR_STALLED
};
MotorState current_state = MOTOR_DITHER;

unsigned long state_start_ms = 0;
unsigned long last_ramp_update_ms = 0;
const int ramp_step_delay_ms = 20;

void setup() {
    Serial.begin(9600);

    // Initialize the NeoPixel
    pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(NEOPIXEL_POWER_PIN, HIGH);
    pixel.begin();
    pixel.setBrightness(50);
    pixel.show();

    motor.begin();
}

void update_status_light();

void loop() {
    motor.update();
    update_status_light();

    unsigned long current_millis = millis();
    unsigned long time_in_state = current_millis - state_start_ms;

    switch (current_state) {
        case MOTOR_DITHER:
            if (time_in_state == 0) {
                motor.enablePIController(false);
            }

            if (time_in_state < 80) {
                bool dither_direction = (time_in_state / 5) % 2 == 0;
                motor.setDirection(dither_direction);
                motor.setCurrentPWM(max_speed * 0.15);
            } else {
                motor.enablePIController(true);
                motor.setCurrentPWM(0);
                motor.setTargetSpeed(0);
                current_state = RAMP_UP;
                state_start_ms = current_millis;
            }
            break;

        case RAMP_UP:
            if (current_millis - last_ramp_update_ms >= ramp_step_delay_ms) {
                last_ramp_update_ms = current_millis;
                int currentSpeed = motor.getTargetSpeed();
                if (currentSpeed < max_speed) {
                    motor.setTargetSpeed(currentSpeed + 1);
                } else {
                    current_state = COAST_HIGH;
                    state_start_ms = current_millis;
                }
            }
            break;

        case COAST_HIGH:
            if (time_in_state >= 3000) {
                current_state = RAMP_DOWN;
                state_start_ms = current_millis;
            }
            break;

        case RAMP_DOWN:
            if (current_millis - last_ramp_update_ms >= ramp_step_delay_ms) {
                last_ramp_update_ms = current_millis;
                int currentSpeed = motor.getTargetSpeed();
                if (currentSpeed > max_speed * 0.1) {
                    motor.setTargetSpeed(currentSpeed - 1);
                } else {
                    current_state = COAST_LOW;
                    state_start_ms = current_millis;
                }
            }
            break;

        case COAST_LOW:
            if (time_in_state >= 3000) {
                current_state = STOP;
                state_start_ms = current_millis;
                motor.setTargetSpeed(0);
                motor.resetPIController();
            }
            break;

        case STOP:
            if (time_in_state >= 2000) {
                current_state = CHANGE_DIRECTION;
                state_start_ms = current_millis;
            }
            break;

        case CHANGE_DIRECTION:
            if (time_in_state >= 500) {
                motor.setDirection(!motor.getDirection());
                motor.resetPIController();
                current_state = MOTOR_DITHER;
                state_start_ms = current_millis;
            }
            break;
        case MOTOR_STALLED:
            // The motor has been stopped by the library, just stay here.
            break;
    }
}

void update_status_light() {
    unsigned long current_millis = millis();
    uint32_t color = COLOR_OFF;
    bool blink_state = true;

    if (current_state != MOTOR_STALLED && motor.getTargetSpeed() > 0 && motor.getTargetSpeed() <= rangiermodus_speed_threshold) {
        color = COLOR_BLUE;
    } else {
        switch (current_state) {
            case STOP:
                color = COLOR_BLUE;
                break;
            case CHANGE_DIRECTION:
                color = COLOR_PINK;
                break;
            case MOTOR_STALLED:
                color = COLOR_RED;
                break;
            case RAMP_UP:
            case RAMP_DOWN:
            case COAST_HIGH:
            case COAST_LOW:
                // For all moving states, the color depends on the P-controller's action.
                ControllerAction action = motor.getControllerAction();

                switch (action) {
                    case ACCELERATING:
                        color = COLOR_GREEN;
                        break;
                    case DECELERATING:
                        color = COLOR_RED;
                        break;
                    case STEADY:
                        color = COLOR_YELLOW;
                        break;
                }
                break;
        }
    }

    switch (current_state) {
        case RAMP_DOWN:
            blink_state = (current_millis / 500) % 2;
            break;
        case CHANGE_DIRECTION:
            blink_state = (current_millis / 100) % 2;
            break;
        case MOTOR_STALLED:
            blink_state = (current_millis / 50) % 2;
            break;
        default:
            blink_state = true;
            break;
    }

    if (blink_state) {
        pixel.setPixelColor(0, color);
    } else {
        pixel.setPixelColor(0, COLOR_OFF);
    }
    pixel.show();
}
