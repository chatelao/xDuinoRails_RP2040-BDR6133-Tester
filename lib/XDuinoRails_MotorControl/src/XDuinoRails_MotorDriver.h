#ifndef XDUINORAILS_MOTOR_DRIVER_H
#define XDUINORAILS_MOTOR_DRIVER_H

#if ARDUINO
#include <Arduino.h>
#endif

#include "pi_controller.h"
#include <functional>

#if defined(TESTING)
unsigned long millis();
#endif

// Forward declaration of the implementation class
class XDuinoRails_MotorDriver_Impl;
class IKalmanFilter;

class XDuinoRails_MotorDriver {
public:
    using FilterCallback = std::function<float(float)>;
    using ControllerCallback = std::function<int(float, float)>;

    XDuinoRails_MotorDriver(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin);
#if defined(TESTING)
    XDuinoRails_MotorDriver(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin, IKalmanFilter* filter);
#endif
    ~XDuinoRails_MotorDriver();

    void begin();
    void update();

    void setTargetSpeed(int speed);
    int getTargetSpeed() const;
    float getMeasuredSpeedPPS() const;
    float getCurrentSpeedSetpoint() const;

    void setDirection(bool forward);
    bool getDirection() const;

    void enablePIController(bool enable);
    void resetPIController();

    void setPIgains(float kp, float ki);
    void setFilterParameters(float mea_e, float est_e, float q);
    void setStallDetection(bool enabled);

    void setAcceleration(float rate);
    void setDeceleration(float rate);
    void setStartupKick(int pwm, int duration_ms);

    // Filter pipeline methods
    void clearFilters();
    void appendEmaFilter(float alpha = 0.21f);
    void appendKalmanFilter();
    void appendCustomFilter(FilterCallback filter);

    void setCustomController(ControllerCallback controller);

    int getCurrentPWM() const;
    void setCurrentPWM(int pwm);

    ControllerAction getControllerAction() const;

private:
    XDuinoRails_MotorDriver_Impl* pImpl;
};

#endif // XDUINORAILS_MOTOR_DRIVER_H
