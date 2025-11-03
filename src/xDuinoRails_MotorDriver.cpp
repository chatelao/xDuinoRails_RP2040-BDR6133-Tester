#if ARDUINO
#include "xDuinoRails_MotorDriver.h"

// Define the static instance pointer
XDuinoRails_MotorDriver* XDuinoRails_MotorDriver::instance = nullptr;

XDuinoRails_MotorDriver::XDuinoRails_MotorDriver(uint8_t inaPin, uint8_t inbPin, uint8_t bemfaPin, uint8_t bemfbPin)
    : _inaPin(inaPin), _inbPin(inbPin), _bemfaPin(bemfaPin), _bemfbPin(bemfbPin),
      pi_controller(Kp_normal, Ki_normal, max_speed),
      bemfKalmanFilter(BEMF_MEA_E, BEMF_EST_E, BEMF_Q) {
    instance = this; // Set the static instance pointer
}

void XDuinoRails_MotorDriver::begin() {
    pinMode(_bemfaPin, INPUT);
    pinMode(_bemfbPin, INPUT);
    // Note: NeoPixel setup will be moved to the example sketch

#ifndef USE_RP2040_LOWLEVEL
    add_repeating_timer_us(pwm_period_us, pwm_on_callback, NULL, &pwm_timer);
#else
    hal_motor_init(_inaPin, _inbPin, _bemfaPin, _bemfbPin, on_bemf_update);
#endif
}

void XDuinoRails_MotorDriver::update() {
    unsigned long current_millis = millis();

    // --- Ramping Logic ---
    if (_is_ramping) {
        unsigned long elapsed_time = current_millis - _ramp_start_time_ms;
        if (elapsed_time >= _ramp_duration_ms) {
            target_speed = _ramp_target_speed;
            _is_ramping = false;
        } else {
            // Linear interpolation
            float progress = (float)elapsed_time / (float)_ramp_duration_ms;
            target_speed = _ramp_start_speed + (int)(progress * (_ramp_target_speed - _ramp_start_speed));
        }
    }

    // --- Speed calculation based on commutation pulses ---
    static unsigned long last_speed_calc_ms = 0;
    if (current_millis - last_speed_calc_ms >= 100) {
        noInterrupts();
        int pulses = commutation_pulse_count;
        commutation_pulse_count = 0;
        interrupts();

        float elapsed_time_s = (current_millis - last_speed_calc_ms) / 1000.0;
        measured_speed_pps = pulses / elapsed_time_s;
        last_speed_calc_ms = current_millis;
    }

    // --- Stall Detection Logic ---
    static unsigned long stall_check_start_ms = 0;
    if (target_speed > 0 && measured_speed_pps < stall_speed_threshold_pps) {
        if (stall_check_start_ms == 0) {
            stall_check_start_ms = current_millis;
        } else if (current_millis - stall_check_start_ms >= stall_timeout_ms) {
            target_speed = 0;
            _is_ramping = false; // Stop ramping on stall
        }
    } else {
        stall_check_start_ms = 0;
    }

#ifdef USE_RP2040_LOWLEVEL
    hal_motor_set_pwm(current_pwm, motor_forward);
#endif
}

void XDuinoRails_MotorDriver::setTargetSpeed(int speed, unsigned long duration) {
    if (speed == target_speed) {
        return; // No change needed
    }

    _ramp_start_speed = target_speed;
    _ramp_target_speed = constrain(speed, 0, max_speed);
    _ramp_start_time_ms = millis();
    _ramp_duration_ms = duration;

    if (duration == 0) {
        target_speed = _ramp_target_speed;
        _is_ramping = false;
    } else {
        _is_ramping = true;
    }
}

void XDuinoRails_MotorDriver::coast() {
    target_speed = 0;
    current_pwm = 0;
#ifdef USE_RP2040_LOWLEVEL
    hal_motor_set_pwm(0, motor_forward);
#else
    // For the timer-based implementation, we can force coast by setting pins to INPUT
    pinMode(_inaPin, INPUT);
    pinMode(_inbPin, INPUT);
#endif
}

void XDuinoRails_MotorDriver::changeDirection() {
    motor_forward = !motor_forward;
    pi_controller.reset();
}

bool XDuinoRails_MotorDriver::isMoving() {
    return measured_speed_pps > 0;
}

int XDuinoRails_MotorDriver::getCurrentSpeed() {
    // Map PPS to a 0-255 range for the user
    return map(measured_speed_pps, 0, 500, 0, 255);
}

//== Static Callback Implementations ==

#ifdef USE_RP2040_LOWLEVEL
void XDuinoRails_MotorDriver::on_bemf_update(int measured_bemf) {
    if (!instance) return;

    if (!instance->_filter_initialized) {
        instance->_smoothed_bemf = measured_bemf;
        instance->_filter_initialized = true;
    } else {
        instance->_smoothed_bemf = (instance->EMA_ALPHA * measured_bemf) + ((1.0 - instance->EMA_ALPHA) * instance->_smoothed_bemf);
    }

    float kalman_filtered_bemf = instance->bemfKalmanFilter.updateEstimate(instance->_smoothed_bemf);

    bool current_bemf_state = (kalman_filtered_bemf > instance->bemf_threshold);
    if (current_bemf_state && !instance->last_bemf_state) {
        instance->commutation_pulse_count++;
    }
    instance->last_bemf_state = current_bemf_state;

    if (instance->pi_controller_enabled) {
        bool is_in_rangiermodus = (instance->target_speed > 0 && instance->target_speed <= instance->rangiermodus_speed_threshold);
        if (is_in_rangiermodus != instance->was_in_rangiermodus) {
            instance->pi_controller.reset();
        }
        instance->was_in_rangiermodus = is_in_rangiermodus;

        instance->pi_controller.setGains(is_in_rangiermodus ? instance->Kp_rangier : instance->Kp_normal, is_in_rangiermodus ? instance->Ki_rangier : instance->Ki_normal);

        int measured_speed = map(instance->measured_speed_pps, 0, 500, 0, 255);
        instance->current_pwm = instance->pi_controller.calculate(instance->target_speed, measured_speed, instance->current_pwm);
    }
}
#else
int64_t XDuinoRails_MotorDriver::pwm_off_callback(alarm_id_t alarm_id, void *user_data) {
    if (!instance) return 0;

    pinMode(instance->_inaPin, INPUT);
    pinMode(instance->_inbPin, INPUT);
    delayMicroseconds(100);

    int bemfA = analogRead(instance->_bemfaPin);
    int bemfB = analogRead(instance->_bemfbPin);
    int measured_bemf = abs(bemfA - bemfB);

    if (!instance->_filter_initialized) {
        instance->_smoothed_bemf = measured_bemf;
        instance->_filter_initialized = true;
    } else {
        instance->_smoothed_bemf = (instance->EMA_ALPHA * measured_bemf) + ((1.0 - instance->EMA_ALPHA) * instance->_smoothed_bemf);
    }

    float kalman_filtered_bemf = instance->bemfKalmanFilter.updateEstimate(instance->_smoothed_bemf);

    bool current_bemf_state = (kalman_filtered_bemf > instance->bemf_threshold);
    if (current_bemf_state && !instance->last_bemf_state) {
        instance->commutation_pulse_count++;
    }
    instance->last_bemf_state = current_bemf_state;

    if (instance->pi_controller_enabled) {
        bool is_in_rangiermodus = (instance->target_speed > 0 && instance->target_speed <= instance->rangiermodus_speed_threshold);
        if (is_in_rangiermodus != instance->was_in_rangiermodus) {
            instance->pi_controller.reset();
        }
        instance->was_in_rangiermodus = is_in_rangiermodus;

        instance->pi_controller.setGains(is_in_rangiermodus ? instance->Kp_rangier : instance->Kp_normal, is_in_rangiermodus ? instance->Ki_rangier : instance->Ki_normal);

        int measured_speed = map(instance->measured_speed_pps, 0, 500, 0, 255);
        instance->current_pwm = instance->pi_controller.calculate(instance->target_speed, measured_speed, instance->current_pwm);
    }

    return 0;
}

bool XDuinoRails_MotorDriver::pwm_on_callback(struct repeating_timer *t) {
    if (!instance) return false;

    int pwm_val = instance->current_pwm;
    long on_time_us = map(pwm_val, 0, 255, 0, instance->pwm_period_us);

    if (on_time_us > 0) {
        pinMode(instance->_inaPin, OUTPUT);
        pinMode(instance->_inbPin, OUTPUT);
        if (instance->motor_forward) {
            digitalWrite(instance->_inaPin, HIGH);
            digitalWrite(instance->_inbPin, LOW);
        } else {
            digitalWrite(instance->_inaPin, LOW);
            digitalWrite(instance->_inbPin, HIGH);
        }
    }

    add_alarm_in_us(on_time_us, pwm_off_callback, NULL, true);

    return true;
}
#endif
#endif // ARDUINO
