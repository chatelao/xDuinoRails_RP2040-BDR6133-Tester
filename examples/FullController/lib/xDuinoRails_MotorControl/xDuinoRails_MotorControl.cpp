#include "xDuinoRails_MotorControl.h"

//== Motor Control Parameters ==
const int max_speed = 255;  ///< Maximum target speed value, corresponds to max PWM duty cycle.
const int pwm_frequency = 1000; ///< PWM frequency in Hz.
const long pwm_period_us = 1000000 / pwm_frequency; ///< PWM period in microseconds.


// Global instance pointer to access the class instance in the C-style timer callbacks
xDuinoRails_MotorControl* motor_instance = nullptr;

// Constructor
xDuinoRails_MotorControl::xDuinoRails_MotorControl() :
    pi_controller(0.1, 0.1, max_speed), // Default Kp, Ki, max_speed
    bemfKalmanFilter(2.0, 2.0, 0.01) // Default BEMF_MEA_E, BEMF_EST_E, BEMF_Q
{
    if (motor_instance == nullptr) {
        motor_instance = this;
    }
    _forward = true;
    _bemf_buffer_head = 0;
    _last_bemf_state = false;
    _commutation_pulse_count = 0;
    measured_speed_pps = 0.0;
    target_speed = 0;
    current_pwm = 0;
    _current_state = MOTOR_IDLE;
    _filterCallback = defaultFilter;
    _controllerCallback = defaultController;
}

// High-Level API
void xDuinoRails_MotorControl::begin(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin) {
    _pwmAPin = pwmAPin;
    _pwmBPin = pwmBPin;
    _bemfAPin = bemfAPin;
    _bemfBPin = bemfBPin;

    pinMode(_bemfAPin, INPUT);
    pinMode(_bemfBPin, INPUT);

    // Initialize the hardware timer for the PWM base frequency
    add_repeating_timer_us(pwm_period_us, pwm_on_callback, NULL, &_pwm_timer);
}

void xDuinoRails_MotorControl::setSpeed(int speed) {
    target_speed = constrain(speed, 0, max_speed);
}

void xDuinoRails_MotorControl::setDirection(bool forward) {
    _forward = forward;
}

void xDuinoRails_MotorControl::stop() {
    target_speed = 0;
    current_pwm = 0;
    pi_controller.reset();
    _current_state = MOTOR_IDLE;
}

// Low-Level/Advanced API
void xDuinoRails_MotorControl::setRawPwm(int dutyCycle) {
    current_pwm = constrain(dutyCycle, 0, 255);
}

int xDuinoRails_MotorControl::getBemf() {
    int head = _bemf_buffer_head;
    if (head == 0) {
        return _bemf_buffer[BEMF_BUFFER_SIZE - 1];
    }
    return _bemf_buffer[head - 1];
}

float xDuinoRails_MotorControl::getSpeedPPS() {
    return measured_speed_pps;
}

const volatile int* xDuinoRails_MotorControl::getBemfBuffer(int* head, int* size) {
    *head = _bemf_buffer_head;
    *size = BEMF_BUFFER_SIZE;
    return _bemf_buffer;
}

// Configuration API
void xDuinoRails_MotorControl::setPIgains(float kp, float ki) {
    pi_controller.setGains(kp, ki);
}

void xDuinoRails_MotorControl::setFilterCallback(BemfFilterCallback callback) {
    _filterCallback = callback;
}

void xDuinoRails_MotorControl::setControllerCallback(SpeedControllerCallback callback) {
    _controllerCallback = callback;
}


// Default callback implementations
float xDuinoRails_MotorControl::defaultFilter(int rawBemf, xDuinoRails_MotorControl* controller) {
    const float EMA_ALPHA = 0.21;
    static float smoothed_bemf = 0.0;
    static bool filter_initialized = false;

    if (!filter_initialized) {
        smoothed_bemf = rawBemf;
        filter_initialized = true;
    } else {
        smoothed_bemf = (EMA_ALPHA * rawBemf) + ((1.0 - EMA_ALPHA) * smoothed_bemf);
    }

    return controller->bemfKalmanFilter.updateEstimate(smoothed_bemf);
}

int xDuinoRails_MotorControl::defaultController(int targetSpeed, int measuredSpeed, xDuinoRails_MotorControl* controller) {
    return controller->pi_controller.calculate(targetSpeed, measuredSpeed, controller->current_pwm);
}


// Timer interrupt callbacks
bool xDuinoRails_MotorControl::pwm_on_callback(struct repeating_timer *t) {
    if (motor_instance == nullptr) return false;

    int pwm_val = motor_instance->current_pwm;
    long on_time_us = map(pwm_val, 0, 255, 0, pwm_period_us);

    if (on_time_us > 0) {
        pinMode(motor_instance->_pwmAPin, OUTPUT);
        pinMode(motor_instance->_pwmBPin, OUTPUT);
        if (motor_instance->_forward) {
            digitalWrite(motor_instance->_pwmAPin, HIGH);
            digitalWrite(motor_instance->_pwmBPin, LOW);
        } else {
            digitalWrite(motor_instance->_pwmAPin, LOW);
            digitalWrite(motor_instance->_pwmBPin, HIGH);
        }
    }
    add_alarm_in_us(on_time_us, pwm_off_callback, motor_instance, true);
    return true;
}

int64_t xDuinoRails_MotorControl::pwm_off_callback(alarm_id_t alarm_id, void *user_data) {
    xDuinoRails_MotorControl* instance = (xDuinoRails_MotorControl*)user_data;

    pinMode(instance->_pwmAPin, INPUT);
    pinMode(instance->_pwmBPin, INPUT);
    delayMicroseconds(100);

    int bemfA = analogRead(instance->_bemfAPin);
    int bemfB = analogRead(instance->_bemfBPin);
    int measured_bemf = abs(bemfA - bemfB);

    // Store in circular buffer
    instance->_bemf_buffer[instance->_bemf_buffer_head] = measured_bemf;
    instance->_bemf_buffer_head = (instance->_bemf_buffer_head + 1) % BEMF_BUFFER_SIZE;

    // Filter BEMF
    float filtered_bemf = instance->_filterCallback(measured_bemf, instance);

    // Detect commutation pulse
    const int bemf_threshold = 500;
    bool current_bemf_state = (filtered_bemf > bemf_threshold);
    if (current_bemf_state && !instance->_last_bemf_state) {
        instance->_commutation_pulse_count++;
    }
    instance->_last_bemf_state = current_bemf_state;

    // Run speed controller
    int measured_speed = map(instance->measured_speed_pps, 0, 500, 0, 255);
    instance->current_pwm = instance->_controllerCallback(instance->target_speed, measured_speed, instance);

    return 0;
}


void xDuinoRails_MotorControl::calculateSpeed() {
    static unsigned long last_speed_calc_ms = 0;
    unsigned long current_millis = millis();

    if (current_millis - last_speed_calc_ms >= 100) {
        noInterrupts();
        int pulses = _commutation_pulse_count;
        _commutation_pulse_count = 0;
        interrupts();

        float elapsed_time_s = (current_millis - last_speed_calc_ms) / 1000.0;
        measured_speed_pps = pulses / elapsed_time_s;
        last_speed_calc_ms = current_millis;
    }
}

void xDuinoRails_MotorControl::loop() {
    calculateSpeed();

    // High-level state machine for the test pattern
    // Note: This is an example implementation. A real application would likely
    // have a more sophisticated state machine or respond to external commands.
    // This logic has been adapted from the original main.cpp.

    unsigned long current_millis = millis();
    unsigned long time_in_state = current_millis - _state_start_ms;
    const int ramp_step_delay_ms = 20;
    const int stall_speed_threshold_pps = 10;
    const unsigned long stall_timeout_ms = 1000;

    // Stall Detection Logic
    static unsigned long stall_check_start_ms = 0;
    if (target_speed > 0 && measured_speed_pps < stall_speed_threshold_pps) {
        if (stall_check_start_ms == 0) {
            stall_check_start_ms = current_millis;
        } else if (current_millis - stall_check_start_ms >= stall_timeout_ms) {
            _current_state = MOTOR_STALLED;
            _state_start_ms = current_millis;
            stop();
        }
    } else {
        stall_check_start_ms = 0;
    }


    switch (_current_state) {
        case MOTOR_IDLE:
            // Do nothing
            break;

        case RAMP_UP:
            if (current_millis - _last_ramp_update_ms >= ramp_step_delay_ms) {
                _last_ramp_update_ms = current_millis;
                if (target_speed < max_speed) {
                    target_speed++;
                } else {
                    _current_state = COAST_HIGH;
                    _state_start_ms = current_millis;
                }
            }
            break;

        case COAST_HIGH:
            if (time_in_state >= 3000) {
                _current_state = RAMP_DOWN;
                _state_start_ms = current_millis;
            }
            break;

        case RAMP_DOWN:
            if (current_millis - _last_ramp_update_ms >= ramp_step_delay_ms) {
                _last_ramp_update_ms = current_millis;
                if (target_speed > max_speed * 0.1) {
                    target_speed--;
                } else {
                    _current_state = COAST_LOW;
                    _state_start_ms = current_millis;
                }
            }
            break;

        case COAST_LOW:
            if (time_in_state >= 3000) {
                _current_state = STOP;
                _state_start_ms = current_millis;
                stop();
            }
            break;

        case STOP:
            if (time_in_state >= 2000) {
                _current_state = CHANGE_DIRECTION;
                _state_start_ms = current_millis;
            }
            break;

        case CHANGE_DIRECTION:
            if (time_in_state >= 500) {
                _forward = !_forward;
                pi_controller.reset();
                _current_state = RAMP_UP; // Or MOTOR_DITHER if implemented
                _state_start_ms = current_millis;
            }
            break;

        case MOTOR_STALLED:
            // Motor is stopped and remains in this state.
            // A reset is required to clear this state.
            break;
    }
}
