#include "XDuinoRails_MotorDriver.h"
#if ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
#include <SimpleKalmanFilter.h>
#include "pi_controller.h"

#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32)
#include "motor_control_hal.h"
#endif

class XDuinoRails_MotorDriver_Impl {
public:
    XDuinoRails_MotorDriver_Impl(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin);
    void begin();
    void update();

    void setTargetSpeed(int speed);
    int getTargetSpeed() const;
    float getMeasuredSpeedPPS() const;

    void setDirection(bool forward);
    bool getDirection() const;

    void enablePIController(bool enable);
    void resetPIController();

    void setPIgains(float kp, float ki);
    void setFilterParameters(float ema_alpha, float mea_e, float est_e, float q);

    void enableEmaFilter(bool enable);
    void enableKalmanFilter(bool enable);

    int getCurrentPWM() const;
    void setCurrentPWM(int pwm);

    ControllerAction getControllerAction() const;

    void setAcceleration(float rate);
    void setDeceleration(float rate);
    void setStartupKick(int pwm, int duration_ms);

private:
    // Pin Definitions
    int _pwmAPin, _pwmBPin, _bemfAPin, _bemfBPin;

    // Motor Control Parameters
    const int max_speed = 255;
    const int rangiermodus_speed_threshold = max_speed * 0.1;

    // Stall Detection Parameters
    const int stall_speed_threshold_pps = 10;
    const unsigned long stall_timeout_ms = 1000;

    // Proportional-Integral Controller
    volatile bool _pi_controller_enabled = true;
    float Kp_normal = 0.1;
    float Ki_normal = 0.1;
    float Kp_rangier = 0.15;
    float Ki_rangier = 0.05;
    PIController _pi_controller;
    bool _was_in_rangiermodus = false;

    // BEMF Pulse Counting
    float EMA_ALPHA = 0.21;
    bool _ema_filter_enabled = true;
    float BEMF_MEA_E = 2.0;
    float BEMF_EST_E = 2.0;
    float BEMF_Q = 0.01;
    const int bemf_threshold = 500;
    volatile int _commutation_pulse_count = 0;
    float _measured_speed_pps = 0.0;
    bool _last_bemf_state = false;
    SimpleKalmanFilter _bemfKalmanFilter;
    bool _kalman_filter_enabled = true;

    // PWM Parameters
#if ARDUINO && !defined(USE_RP2040_LOWLEVEL) && !defined(ARDUINO_ARCH_STM32)
    const int pwm_frequency = 1000;
    const long pwm_period_us = 1000000 / pwm_frequency;
    struct repeating_timer _pwm_timer;
#endif

    // Global Motor & PWM State Variables
    volatile int _target_speed = 0;
    volatile float _current_speed_setpoint = 0.0;
    volatile int _current_pwm = 0;
    volatile bool _motor_forward = true;

    // Acceleration/Deceleration Parameters
    float _acceleration_rate = 0.0; // in speed units per second
    float _deceleration_rate = 0.0; // in speed units per second

    // Startup Kick Parameters
    int _startup_kick_pwm = 0;
    int _startup_kick_duration_ms = 0;
    unsigned long _kick_start_ms = 0;
    bool _is_kick_active = false;

    // Internal state
    unsigned long _last_speed_calc_ms = 0;
    unsigned long _last_update_ms = 0;
    unsigned long _stall_check_start_ms = 0;

#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32)
    static void on_bemf_update_wrapper(int measured_bemf);
    void on_bemf_update(int measured_bemf);
    static XDuinoRails_MotorDriver_Impl* instance; // Static instance for C-style callback
#elif ARDUINO
    static XDuinoRails_MotorDriver_Impl* instance;
    static bool pwm_on_callback(struct repeating_timer *t);
    static int64_t pwm_off_callback(alarm_id_t alarm_id, void *user_data);
#endif
};

XDuinoRails_MotorDriver_Impl* XDuinoRails_MotorDriver_Impl::instance = nullptr;

XDuinoRails_MotorDriver_Impl::XDuinoRails_MotorDriver_Impl(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin)
    : _pwmAPin(pwmAPin), _pwmBPin(pwmBPin), _bemfAPin(bemfAPin), _bemfBPin(bemfBPin),
      _pi_controller(Kp_normal, Ki_normal, max_speed),
      _bemfKalmanFilter(BEMF_MEA_E, BEMF_EST_E, BEMF_Q) {
    instance = this;
}

void XDuinoRails_MotorDriver_Impl::begin() {
#if ARDUINO
    pinMode(_bemfAPin, INPUT);
    pinMode(_bemfBPin, INPUT);

#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32)
    hal_motor_init(_pwmAPin, _pwmBPin, _bemfAPin, _bemfBPin, on_bemf_update_wrapper);
#else
    add_repeating_timer_us(pwm_period_us, pwm_on_callback, NULL, &_pwm_timer);
#endif
#endif
}

void XDuinoRails_MotorDriver_Impl::update() {
#if ARDUINO
    unsigned long current_millis = millis();
    float elapsed_update_time_s = (current_millis - _last_update_ms) / 1000.0;
    _last_update_ms = current_millis;

    // Acceleration/Deceleration Ramp
    if (_current_speed_setpoint < _target_speed) {
        _current_speed_setpoint += _acceleration_rate * elapsed_update_time_s;
        if (_current_speed_setpoint > _target_speed) {
            _current_speed_setpoint = _target_speed;
        }
    } else if (_current_speed_setpoint > _target_speed) {
        _current_speed_setpoint -= _deceleration_rate * elapsed_update_time_s;
        if (_current_speed_setpoint < _target_speed) {
            _current_speed_setpoint = _target_speed;
        }
    }

    // Speed calculation
    if (current_millis - _last_speed_calc_ms >= 100) {
        noInterrupts();
        int pulses = _commutation_pulse_count;
        _commutation_pulse_count = 0;
        interrupts();

        float elapsed_time_s = (current_millis - _last_speed_calc_ms) / 1000.0;
        if (elapsed_time_s > 0) {
            _measured_speed_pps = pulses / elapsed_time_s;
        }
        _last_speed_calc_ms = current_millis;
    }

    // Stall detection
    if (_target_speed > 0 && _measured_speed_pps < stall_speed_threshold_pps) {
        if (_stall_check_start_ms == 0) {
            _stall_check_start_ms = current_millis;
        } else if (current_millis - _stall_check_start_ms >= stall_timeout_ms) {
            setTargetSpeed(0); // Stall detected, stop motor smoothly
        }
    } else {
        _stall_check_start_ms = 0;
    }

#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32)
    hal_motor_set_pwm(_current_pwm, _motor_forward);
#endif
#endif
}

void XDuinoRails_MotorDriver_Impl::setTargetSpeed(int speed) {
#if ARDUINO
    _target_speed = constrain(speed, 0, max_speed);
#else
    _target_speed = speed;
#endif

    // If acceleration/deceleration is not used, jump directly to the target speed.
    if (_acceleration_rate <= 0 && _deceleration_rate <= 0) {
        _current_speed_setpoint = _target_speed;
    }

    // Trigger startup kick if starting from zero and kick is configured
    if (_current_speed_setpoint == 0 && _target_speed > 0 && _startup_kick_pwm > 0 && _startup_kick_duration_ms > 0) {
        _is_kick_active = true;
        _kick_start_ms = millis();
    }
}

int XDuinoRails_MotorDriver_Impl::getTargetSpeed() const {
    return _target_speed;
}

float XDuinoRails_MotorDriver_Impl::getMeasuredSpeedPPS() const {
    return _measured_speed_pps;
}

void XDuinoRails_MotorDriver_Impl::setDirection(bool forward) {
    _motor_forward = forward;
}

bool XDuinoRails_MotorDriver_Impl::getDirection() const {
    return _motor_forward;
}

void XDuinoRails_MotorDriver_Impl::enablePIController(bool enable) {
    _pi_controller_enabled = enable;
}

void XDuinoRails_MotorDriver_Impl::resetPIController() {
    _pi_controller.reset();
}

void XDuinoRails_MotorDriver_Impl::setPIgains(float kp, float ki) {
    // This function could be smarter, e.g. distinguishing between normal and rangier mode
    // For now, it sets both to the same value.
    Kp_normal = kp;
    Ki_normal = ki;
    Kp_rangier = kp;
    Ki_rangier = ki;
    _pi_controller.setGains(kp, ki);
}

void XDuinoRails_MotorDriver_Impl::setFilterParameters(float ema_alpha, float mea_e, float est_e, float q) {
    EMA_ALPHA = ema_alpha;
    BEMF_MEA_E = mea_e;
    BEMF_EST_E = est_e;
    BEMF_Q = q;
    // Re-initialize the Kalman filter with new parameters
    _bemfKalmanFilter = SimpleKalmanFilter(BEMF_MEA_E, BEMF_EST_E, BEMF_Q);
}

void XDuinoRails_MotorDriver_Impl::enableEmaFilter(bool enable) {
    _ema_filter_enabled = enable;
}

void XDuinoRails_MotorDriver_Impl::enableKalmanFilter(bool enable) {
    _kalman_filter_enabled = enable;
}

int XDuinoRails_MotorDriver_Impl::getCurrentPWM() const {
    return _current_pwm;
}

void XDuinoRails_MotorDriver_Impl::setCurrentPWM(int pwm) {
    _current_pwm = pwm;
}

ControllerAction XDuinoRails_MotorDriver_Impl::getControllerAction() const {
    return _pi_controller.getAction();
}

void XDuinoRails_MotorDriver::setAcceleration(float rate) {
    pImpl->setAcceleration(rate);
}

void XDuinoRails_MotorDriver::setDeceleration(float rate) {
    pImpl->setDeceleration(rate);
}

void XDuinoRails_MotorDriver::setStartupKick(int pwm, int duration_ms) {
    pImpl->setStartupKick(pwm, duration_ms);
}

#if defined(USE_RP2040_LOWLEVEL) || defined(ARDUINO_ARCH_STM32)
void XDuinoRails_MotorDriver_Impl::on_bemf_update_wrapper(int measured_bemf) {
    if (instance) {
        instance->on_bemf_update(measured_bemf);
    }
}

void XDuinoRails_MotorDriver_Impl::on_bemf_update(int measured_bemf) {
    static float smoothed_bemf = 0.0;
    static bool filter_initialized = false;
    if (!filter_initialized) {
        smoothed_bemf = measured_bemf;
        filter_initialized = true;
    } else if (_ema_filter_enabled) {
        smoothed_bemf = (EMA_ALPHA * measured_bemf) + ((1.0 - EMA_ALPHA) * smoothed_bemf);
    } else {
        smoothed_bemf = measured_bemf;
    }

    float kalman_filtered_bemf = _kalman_filter_enabled ? _bemfKalmanFilter.updateEstimate(smoothed_bemf) : smoothed_bemf;

    bool current_bemf_state = (kalman_filtered_bemf > bemf_threshold);
    if (current_bemf_state && !_last_bemf_state) {
        _commutation_pulse_count++;
    }
    _last_bemf_state = current_bemf_state;

    // Handle startup kick
    if (_is_kick_active) {
        if (millis() - _kick_start_ms < _startup_kick_duration_ms) {
            _current_pwm = _startup_kick_pwm;
        } else {
            _is_kick_active = false;
        }
    } else if (_pi_controller_enabled) {
        bool is_in_rangiermodus = (_current_speed_setpoint > 0 && _current_speed_setpoint <= rangiermodus_speed_threshold);
        if (is_in_rangiermodus != _was_in_rangiermodus) {
            _pi_controller.reset();
        }
        _was_in_rangiermodus = is_in_rangiermodus;

        _pi_controller.setGains(is_in_rangiermodus ? Kp_rangier : Kp_normal, is_in_rangiermodus ? Ki_rangier : Ki_normal);

        int measured_speed = map(_measured_speed_pps, 0, 500, 0, 255);
        _current_pwm = _pi_controller.calculate(_current_speed_setpoint, measured_speed, _current_pwm);
    }
}

void XDuinoRails_MotorDriver_Impl::setAcceleration(float rate) {
    _acceleration_rate = rate;
}

void XDuinoRails_MotorDriver_Impl::setDeceleration(float rate) {
    _deceleration_rate = rate;
}

void XDuinoRails_MotorDriver_Impl::setStartupKick(int pwm, int duration_ms) {
    _startup_kick_pwm = pwm;
    _startup_kick_duration_ms = duration_ms;
}

#elif ARDUINO && !defined(USE_RP2040_LOWLEVEL)

bool XDuinoRails_MotorDriver_Impl::pwm_on_callback(struct repeating_timer *t) {
    if(instance) {
        int pwm_val = instance->_current_pwm;

        long on_time_us = map(pwm_val, 0, 255, 0, instance->pwm_period_us);

        if (on_time_us > 0) {
            pinMode(instance->_pwmAPin, OUTPUT);
            pinMode(instance->_pwmBPin, OUTPUT);
            if (instance->_motor_forward) {
                digitalWrite(instance->_pwmAPin, HIGH);
                digitalWrite(instance->_pwmBPin, LOW);
            } else {
                digitalWrite(instance->_pwmAPin, LOW);
                digitalWrite(instance->_pwmBPin, HIGH);
            }
        }
        add_alarm_in_us(on_time_us, pwm_off_callback, NULL, true);
    }
    return true;
}

int64_t XDuinoRails_MotorDriver_Impl::pwm_off_callback(alarm_id_t alarm_id, void *user_data) {
    if(instance) {
        pinMode(instance->_pwmAPin, INPUT);
        pinMode(instance->_pwmBPin, INPUT);
        delayMicroseconds(100);

        int bemfA = analogRead(instance->_bemfAPin);
        int bemfB = analogRead(instance->_bemfBPin);
        int measured_bemf = abs(bemfA - bemfB);

        static float smoothed_bemf = 0.0;
        static bool filter_initialized = false;

        if (!filter_initialized) {
            smoothed_bemf = measured_bemf;
            filter_initialized = true;
        } else if (instance->_ema_filter_enabled) {
            smoothed_bemf = (instance->EMA_ALPHA * measured_bemf) + ((1.0 - instance->EMA_ALPHA) * smoothed_bemf);
        } else {
            smoothed_bemf = measured_bemf;
        }

        float kalman_filtered_bemf = instance->_kalman_filter_enabled ? instance->_bemfKalmanFilter.updateEstimate(smoothed_bemf) : smoothed_bemf;

        bool current_bemf_state = (kalman_filtered_bemf > instance->bemf_threshold);
        if (current_bemf_state && !instance->_last_bemf_state) {
            instance->_commutation_pulse_count++;
        }
        instance->_last_bemf_state = current_bemf_state;

        if (instance->_pi_controller_enabled) {
            bool is_in_rangiermodus = (instance->_target_speed > 0 && instance->_target_speed <= instance->rangiermodus_speed_threshold);

            if (is_in_rangiermodus != instance->_was_in_rangiermodus) {
                instance->_pi_controller.reset();
            }
            instance->_was_in_rangiermodus = is_in_rangiermodus;

            if (is_in_rangiermodus) {
                instance->_pi_controller.setGains(instance->Kp_rangier, instance->Ki_rangier);
            } else {
                instance->_pi_controller.setGains(instance->Kp_normal, instance->Ki_normal);
            }
            int measured_speed = map(instance->_measured_speed_pps, 0, 500, 0, 255);
            instance->_current_pwm = instance->_pi_controller.calculate(instance->_target_speed, measured_speed, instance->_current_pwm);
        }
    }
    return 0;
}
#endif

// PIMPL forwarding
XDuinoRails_MotorDriver::XDuinoRails_MotorDriver(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin)
    : pImpl(new XDuinoRails_MotorDriver_Impl(pwmAPin, pwmBPin, bemfAPin, bemfBPin)) {}

XDuinoRails_MotorDriver::~XDuinoRails_MotorDriver() {
    delete pImpl;
}

void XDuinoRails_MotorDriver::begin() {
    pImpl->begin();
}

void XDuinoRails_MotorDriver::update() {
    pImpl->update();
}

void XDuinoRails_MotorDriver::setTargetSpeed(int speed) {
    pImpl->setTargetSpeed(speed);
}

int XDuinoRails_MotorDriver::getTargetSpeed() const {
    return pImpl->getTargetSpeed();
}

float XDuinoRails_MotorDriver::getMeasuredSpeedPPS() const {
    return pImpl->getMeasuredSpeedPPS();
}

void XDuinoRails_MotorDriver::setDirection(bool forward) {
    pImpl->setDirection(forward);
}

bool XDuinoRails_MotorDriver::getDirection() const {
    return pImpl->getDirection();
}

void XDuinoRails_MotorDriver::enablePIController(bool enable) {
    pImpl->enablePIController(enable);
}

void XDuinoRails_MotorDriver::resetPIController() {
    pImpl->resetPIController();
}

void XDuinoRails_MotorDriver::setPIgains(float kp, float ki) {
    pImpl->setPIgains(kp, ki);
}

void XDuinoRails_MotorDriver::setFilterParameters(float ema_alpha, float mea_e, float est_e, float q) {
    pImpl->setFilterParameters(ema_alpha, mea_e, est_e, q);
}

void XDuinoRails_MotorDriver::enableEmaFilter(bool enable) {
    pImpl->enableEmaFilter(enable);
}

void XDuinoRails_MotorDriver::enableKalmanFilter(bool enable) {
    pImpl->enableKalmanFilter(enable);
}

int XDuinoRails_MotorDriver::getCurrentPWM() const {
    return pImpl->getCurrentPWM();
}

void XDuinoRails_MotorDriver::setCurrentPWM(int pwm) {
    pImpl->setCurrentPWM(pwm);
}

ControllerAction XDuinoRails_MotorDriver::getControllerAction() const {
    return pImpl->getControllerAction();
}
