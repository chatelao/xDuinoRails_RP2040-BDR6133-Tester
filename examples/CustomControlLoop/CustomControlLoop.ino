#include <XDuinoRails_MotorDriver.h>

// Pin Definitions for XIAO RP2040
const int PWM_A_PIN = 28;
const int PWM_B_PIN = 29;
const int BEMF_A_PIN = 26;
const int BEMF_B_PIN = 27;

XDuinoRails_MotorDriver motor(PWM_A_PIN, PWM_B_PIN, BEMF_A_PIN, BEMF_B_PIN);

// --- Custom Moving Average Filter ---
const int num_readings = 5;
float readings[num_readings];
int read_index = 0;
float total = 0;

float moving_average_filter(float new_reading) {
    total = total - readings[read_index];
    readings[read_index] = new_reading;
    total = total + readings[read_index];
    read_index = read_index + 1;

    if (read_index >= num_readings) {
        read_index = 0;
    }

    return total / num_readings;
}

// --- Custom Proportional Controller ---
int proportional_controller(float target_speed, float measured_speed) {
    const float Kp = 0.8f;
    float error = target_speed - measured_speed;
    int output = target_speed + (int)(error * Kp);
    return constrain(output, 0, 255);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    motor.begin();

    // 1. Clear the default EMA + Kalman filter pipeline
    motor.clearFilters();

    // 2. Add our custom moving average filter
    motor.appendCustomFilter(moving_average_filter);

    // 3. Set our custom controller
    motor.setCustomController(proportional_controller);

    motor.setDirection(true);
    motor.setTargetSpeed(100);
}

void loop() {
    motor.update();

    static unsigned long last_print_ms = 0;
    if (millis() - last_print_ms > 100) {
        last_print_ms = millis();
        Serial.print("Target: ");
        Serial.print(motor.getTargetSpeed());
        Serial.print(" PPS, Measured: ");
        Serial.print(motor.getMeasuredSpeedPPS(), 2);
        Serial.print(" PPS, PWM: ");
        Serial.println(motor.getCurrentPWM());
    }
}
