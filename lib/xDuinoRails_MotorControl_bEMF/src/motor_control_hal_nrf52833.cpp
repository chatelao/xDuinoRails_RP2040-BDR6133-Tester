#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>
#include <nRF52_PWM.h>
#include <nrfx_timer.h>

// PWM instances for motor control.
static nRF52_PWM* pwm_a;
static nRF52_PWM* pwm_b;

// Timer instance for BEMF measurement.
static const nrfx_timer_t bemf_timer = NRFX_TIMER_INSTANCE(0);

// Store pin numbers for later use.
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static hal_bemf_update_callback_t g_bemf_callback;

// Desired PWM frequency in Hz.
const float PWM_FREQUENCY = 20000.0f;

// BEMF measurement interval in milliseconds.
const uint32_t BEMF_MEASUREMENT_INTERVAL_MS = 10;

// Timer event handler for BEMF measurement.
void bemf_timer_event_handler(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // Read the ADC values from the BEMF pins.
        int bemf_a_value = analogRead(g_bemf_a_pin);
        int bemf_b_value = analogRead(g_bemf_b_pin);

        // Calculate the differential BEMF.
        int differential_bemf = abs(bemf_a_value - bemf_b_value);

        // Call the callback function with the new BEMF value.
        if (g_bemf_callback) {
            g_bemf_callback(differential_bemf);
        }
    }
}

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    g_bemf_callback = callback;

    // Create new PWM instances for each motor direction.
    pwm_a = new nRF52_PWM(g_pwm_a_pin, PWM_FREQUENCY, 0.0f);
    pwm_b = new nRF52_PWM(g_pwm_b_pin, PWM_FREQUENCY, 0.0f);

    // Initialize the PWM instances.
    if (pwm_a) {
        pwm_a->setPWM();
    }
    if (pwm_b) {
        pwm_b->setPWM();
    }

    // Configure BEMF pins as analog inputs.
    pinMode(g_bemf_a_pin, INPUT);
    pinMode(g_bemf_b_pin, INPUT);

    // Configure the BEMF measurement timer.
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    nrfx_timer_init(&bemf_timer, &timer_cfg, bemf_timer_event_handler);

    // Set the timer to trigger an interrupt every BEMF_MEASUREMENT_INTERVAL_MS.
    uint32_t ticks = nrfx_timer_ms_to_ticks(&bemf_timer, BEMF_MEASUREMENT_INTERVAL_MS);
    nrfx_timer_extended_compare(&bemf_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    // Start the timer.
    nrfx_timer_enable(&bemf_timer);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // The duty cycle is a float from 0.0 to 100.0 for the nRF52_PWM library.
    float duty_cycle_float = map(duty_cycle, 0, 255, 0, 100);

    if (forward) {
        pwm_a->setPWM(g_pwm_a_pin, PWM_FREQUENCY, duty_cycle_float);
        pwm_b->setPWM(g_pwm_b_pin, PWM_FREQUENCY, 0.0f);
    } else {
        pwm_a->setPWM(g_pwm_a_pin, PWM_FREQUENCY, 0.0f);
        pwm_b->setPWM(g_pwm_b_pin, PWM_FREQUENCY, duty_cycle_float);
    }
}

#endif // ARDUINO_ARCH_NRF52
