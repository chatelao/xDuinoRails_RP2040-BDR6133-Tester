#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>

// Store pin numbers for later use.
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static hal_bemf_update_callback_t g_bemf_callback;

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    g_bemf_callback = callback;

    // Configure PWM pins as outputs.
    pinMode(g_pwm_a_pin, OUTPUT);
    pinMode(g_pwm_b_pin, OUTPUT);

    // Set a default PWM frequency. This may need adjustment.
    // The nRF52 Arduino core's analogWrite frequency defaults to 500Hz.
    // For motor control, a higher frequency is often better to reduce audible noise.
    // Let's aim for 20kHz. The formula is Base_Freq / Resolution.
    // Base_Freq can be 16MHz, 8MHz, ..., 125kHz.
    // Let's use 16MHz and a resolution of 800. 16,000,000 / 800 = 20,000 Hz.
    analogWriteResolution(10); // 10-bit resolution (0-1023)
    // The actual frequency setting is more complex and may require direct register manipulation
    // or using a library like Adafruit_nRF52_Arduino's HardwarePWM.
    // For now, we'll rely on the default and refine later if needed.

    // Configure BEMF pins as analog inputs.
    pinMode(g_bemf_a_pin, INPUT);
    pinMode(g_bemf_b_pin, INPUT);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    if (forward) {
        analogWrite(g_pwm_a_pin, duty_cycle);
        digitalWrite(g_pwm_b_pin, LOW);
    } else {
        digitalWrite(g_pwm_a_pin, LOW);
        analogWrite(g_pwm_b_pin, duty_cycle);
    }
}

// TODO: Implement a timer-based interrupt to read BEMF and call the callback.
// This will be a more complex task involving hardware timers.

#endif // ARDUINO_ARCH_NRF52
