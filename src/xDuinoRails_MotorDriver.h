#ifndef XDUINORAILS_MOTORDRIVER_H
#define XDUINORAILS_MOTORDRIVER_H

#if ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include <SimpleKalmanFilter.h>
#ifdef __cplusplus
}
#endif

#include "PIController.h"

#ifdef USE_RP2040_LOWLEVEL
#include "motor_control_hal.h"
#endif

class XDuinoRails_MotorDriver {
public:
    // Constructor: Accepts the four motor pin numbers
    XDuinoRails_MotorDriver(uint8_t inaPin, uint8_t inbPin, uint8_t bemfaPin, uint8_t bemfbPin);

    // Initializes the motor driver and hardware
    void begin();

    // Main update loop to be called repeatedly
    void update();

    // Sets the target speed with a specified ramp duration
    void setTargetSpeed(int speed, unsigned long duration);

    // Puts the motor into a coasting (high-impedance) state
    void coast();

    // Changes the motor's direction of rotation
    void changeDirection();

    // Returns true if the motor is currently moving
    bool isMoving();

    // Gets the current speed of the motor
    int getCurrentSpeed();

private:
    // Pin numbers
    uint8_t _inaPin;
    uint8_t _inbPin;
    uint8_t _bemfaPin;
    uint8_t _bemfbPin;

    // Motor Control Parameters
    const int max_speed = 255;
    const int rangiermodus_speed_threshold = max_speed * 0.1;

    // Stall Detection Parameters
    const int stall_speed_threshold_pps = 10;
    const unsigned long stall_timeout_ms = 1000;

    // Proportional-Integral Controller
    volatile bool pi_controller_enabled = true;
    const float Kp_normal = 0.1;
    const float Ki_normal = 0.1;
    const float Kp_rangier = 0.15;
    const float Ki_rangier = 0.05;
    PIController pi_controller;
    bool was_in_rangiermodus = false;

    // BEMF Pulse Counting
    const float EMA_ALPHA = 0.21;
    const float BEMF_MEA_E = 2.0;
    const float BEMF_EST_E = 2.0;
    const float BEMF_Q = 0.01;
    const int bemf_threshold = 500;
    volatile int commutation_pulse_count = 0;
    float measured_speed_pps = 0.0;
    bool last_bemf_state = false;
    SimpleKalmanFilter bemfKalmanFilter;
    float _smoothed_bemf = 0.0;
    bool _filter_initialized = false;

    // PWM Parameters
#ifndef USE_RP2040_LOWLEVEL
    const int pwm_frequency = 1000;
    const long pwm_period_us = 1000000 / pwm_frequency;
    struct repeating_timer pwm_timer;
#endif

    // Global Motor & PWM State Variables
    volatile int target_speed = 0;
    volatile int current_pwm = 0;
    volatile bool motor_forward = true;

    // Ramping variables
    int _ramp_start_speed = 0;
    int _ramp_target_speed = 0;
    unsigned long _ramp_start_time_ms = 0;
    unsigned long _ramp_duration_ms = 0;
    bool _is_ramping = false;

    // Private methods
#ifndef USE_RP2040_LOWLEVEL
    static int64_t pwm_off_callback(alarm_id_t alarm_id, void *user_data);
    static bool pwm_on_callback(struct repeating_timer *t);
#else
    static void on_bemf_update(int measured_bemf);
#endif
};

#endif // XDUINORAILS_MOTORDRIVER_H
