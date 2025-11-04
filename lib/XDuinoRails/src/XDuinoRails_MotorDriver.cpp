#include "XDuinoRails_MotorDriver.h"
#if ARDUINO
#include <Adafruit_NeoPixel.h>
#endif
#include <SimpleKalmanFilter.h>
#include "pi_controller.h"

#ifdef USE_RP2040_LOWLEVEL
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

    int getCurrentPWM() const;
    void setCurrentPWM(int pwm);

    ControllerAction getControllerAction() const;

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
    const float Kp_normal = 0.1;
    const float Ki_normal = 0.1;
    const float Kp_rangier = 0.15;
    const float Ki_rangier = 0.05;
    PIController _pi_controller;
    bool _was_in_rangiermodus = false;

    // BEMF Pulse Counting
    const float EMA_ALPHA = 0.21;
    const float BEMF_MEA_E = 2.0;
    const float BEMF_EST_E = 2.0;
    const float BEMF_Q = 0.01;
    const int bemf_threshold = 500;
    volatile int _commutation_pulse_count = 0;
    float _measured_speed_pps = 0.0;
    bool _last_bemf_state = false;
    SimpleKalmanFilter _bemfKalmanFilter;

    // PWM Parameters
#ifndef USE_RP2040_LOWLEVEL
    const int pwm_frequency = 1000;
    const long pwm_period_us = 1000000 / pwm_frequency;
    struct repeating_timer _pwm_timer;
#endif

    // Global Motor & PWM State Variables
    volatile int _target_speed = 0;
    volatile int _current_pwm = 0;
    volatile bool _motor_forward = true;

    // Internal state
    unsigned long _last_speed_calc_ms = 0;
    unsigned long _stall_check_start_ms = 0;

#if ARDUINO && !defined(USE_RP2040_LOWLEVEL)
    static XDuinoRails_MotorDriver_Impl* instance;
    static bool pwm_on_callback(struct repeating_timer *t);
    static int60_t pwm_off_callback(alarm_id_t alarm_id, void *user_data);
#elif defined(USE_RP2040_LOWLEVEL)
    static void on_bemf_update_wrapper(int measured_bemf);
    void on_bemf_update(int measured_bemf);
    static XDuinoRails_MotorDriver_Impl* instance; // Static instance for C-style callback
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

#ifndef USE_RP2040_LOWLEVEL
    add_repeating_timer_us(pwm_period_us, pwm_on_callback, NULL, &_pwm_timer);
#else
    hal_motor_init(_pwmAPin, _pwmBPin, _bemfAPin, _bemfBPin, on_bemf_update_wrapper);
#endif
#endif
}

void XDuinoRails_MotorDriver_Impl::update() {
#if ARDUINO
    unsigned long current_millis = millis();

    // Speed calculation
    if (current_millis - _last_speed_calc_ms >= 100) {
        noInterrupts();
        int pulses = _commutation_pulse_count;
        _commutation_pulse_count = 0;
        interrupts();

        float elapsed_time_s = (current_millis - _last_speed_calc_ms) / 1000.0;
        _measured_speed_pps = pulses / elapsed_time_s;
        _last_speed_calc_ms = current_millis;
    }

    // Stall detection
    if (_target_speed > 0 && _measured_speed_pps < stall_speed_threshold_pps) {
        if (_stall_check_start_ms == 0) {
            _stall_check_start_ms = current_millis;
        } else if (current_millis - _stall_check_start_ms >= stall_timeout_ms) {
            _target_speed = 0; // Stall detected, stop motor
        }
    } else {
        _stall_check_start_ms = 0;
    }

#ifdef USE_RP2040_LOWLEVEL
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

int XDuinoRails_MotorDriver_Impl::getCurrentPWM() const {
    return _current_pwm;
}

void XDuinoRails_MotorDriver_Impl::setCurrentPWM(int pwm) {
    _current_pwm = pwm;
}

ControllerAction XDuinoRails_MotorDriver_Impl::getControllerAction() const {
    return _pi_controller.getAction();
}

#ifdef USE_RP2040_LOWLEVEL
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
    } else {
        smoothed_bemf = (EMA_ALPHA * measured_bemf) + ((1.0 - EMA_ALPHA) * smoothed_bemf);
    }

    float kalman_filtered_bemf = _bemfKalmanFilter.updateEstimate(smoothed_bemf);

    bool current_bemf_state = (kalman_filtered_bemf > bemf_threshold);
    if (current_bemf_state && !_last_bemf_state) {
        _commutation_pulse_count++;
    }
    _last_bemf_state = current_bemf_state;

    if (_pi_controller_enabled) {
        bool is_in_rangiermodus = (_target_speed > 0 && _target_speed <= rangiermodus_speed_threshold);
        if (is_in_rangiermodus != _was_in_rangiermodus) {
            _pi_controller.reset();
        }
        _was_in_rangiermodus = is_in_rangiermodus;

        _pi_controller.setGains(is_in_rangiermodus ? Kp_rangier : Kp_normal, is_in_rangiermodus ? Ki_rangier : Ki_normal);

        int measured_speed = map(_measured_speed_pps, 0, 500, 0, 255);
        _current_pwm = _pi_controller.calculate(_target_speed, measured_speed, _current_pwm);
    }
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
        } else {
            smoothed_bemf = (instance->EMA_ALPHA * measured_bemf) + ((1.0 - instance->EMA_ALPHA) * smoothed_bemf);
        }

        float kalman_filtered_bemf = instance->_bemfKalmanFilter.updateEstimate(smoothed_bemf);

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

int XDuinoRails_MotorDriver::getCurrentPWM() const {
    return pImpl->getCurrentPWM();
}

void XDuinoRails_MotorDriver::setCurrentPWM(int pwm) {
    pImpl->setCurrentPWM(pwm);
}

ControllerAction XDuinoRails_MotorDriver::getControllerAction() const {
    return pImpl->getControllerAction();
}
