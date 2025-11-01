#ifndef XDUINORAILS_MOTORCONTROL_H
#define XDUINORAILS_MOTORCONTROL_H

#include <Arduino.h>
#include "pico/time.h"
#include <SimpleKalmanFilter.h>
#include "PIController.h"

// Forward declaration of the main class to be used in the callback function pointers
class xDuinoRails_MotorControl;

// Define function pointer types for customizable callbacks
typedef float (*BemfFilterCallback)(int rawBemf, xDuinoRails_MotorControl* controller);
typedef int (*SpeedControllerCallback)(int targetSpeed, int measuredSpeed, xDuinoRails_MotorControl* controller);


class xDuinoRails_MotorControl {
public:
    xDuinoRails_MotorControl();

    // High-Level API
    void begin(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin);
    void setSpeed(int speed); // Speed in pulses per second
    void setDirection(bool forward);
    void stop();
    void loop(); // Manages the high-level state machine (for automated patterns)

    // Low-Level/Advanced API
    void setRawPwm(int dutyCycle);
    int getBemf();
    float getSpeedPPS();
    const volatile int* getBemfBuffer(int* head, int* size);

    // Configuration API
    void setPIgains(float kp, float ki);
    void setFilterCallback(BemfFilterCallback callback);
    void setControllerCallback(SpeedControllerCallback callback);

    // Public members accessible by callbacks
    PIController pi_controller;
    SimpleKalmanFilter bemfKalmanFilter;
    float measured_speed_pps;
    int target_speed;
    volatile int current_pwm;

private:
    // Pins
    int _pwmAPin;
    int _pwmBPin;
    int _bemfAPin;
    int _bemfBPin;

    // State
    bool _forward;

    // Timer
    struct repeating_timer _pwm_timer;

    // BEMF processing
    static const int BEMF_BUFFER_SIZE = 64;
    volatile int _bemf_buffer[BEMF_BUFFER_SIZE];
    volatile int _bemf_buffer_head;
    bool _last_bemf_state;
    volatile int _commutation_pulse_count;

    // Callbacks
    BemfFilterCallback _filterCallback;
    SpeedControllerCallback _controllerCallback;

    // Default callback implementations
    static float defaultFilter(int rawBemf, xDuinoRails_MotorControl* controller);
    static int defaultController(int targetSpeed, int measuredSpeed, xDuinoRails_MotorControl* controller);

    // Timer interrupt callbacks
    static bool pwm_on_callback(struct repeating_timer *t);
    static int64_t pwm_off_callback(alarm_id_t alarm_id, void *user_data);

    void calculateSpeed();

    // High-level state machine
    enum MotorState {
        MOTOR_IDLE,
        MOTOR_DITHER,
        RAMP_UP,
        COAST_HIGH,
        RAMP_DOWN,
        COAST_LOW,
        STOP,
        CHANGE_DIRECTION,
        MOTOR_STALLED
    };
    MotorState _current_state;
    unsigned long _state_start_ms;
    unsigned long _last_ramp_update_ms;

};

#endif // XDUINORAILS_MOTORCONTROL_H
