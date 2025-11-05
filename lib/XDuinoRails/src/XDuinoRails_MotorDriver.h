#ifndef XDUINORAILS_MOTOR_DRIVER_H
#define XDUINORAILS_MOTOR_DRIVER_H

#if ARDUINO
#include <Arduino.h>
#endif

#include "pi_controller.h"

// Forward declaration of the implementation class
class XDuinoRails_MotorDriver_Impl;

class XDuinoRails_MotorDriver {
public:
    XDuinoRails_MotorDriver(int pwmAPin, int pwmBPin, int bemfAPin, int bemfBPin);
    ~XDuinoRails_MotorDriver();

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

    void setAcceleration(float rate);
    void setDeceleration(float rate);
    void setStartupKick(int pwm, int duration_ms);

    void enableEmaFilter(bool enable);
    void enableKalmanFilter(bool enable);

    int getCurrentPWM() const;
    void setCurrentPWM(int pwm);

    ControllerAction getControllerAction() const;

private:
    XDuinoRails_MotorDriver_Impl* pImpl;
};

#endif // XDUINORAILS_MOTOR_DRIVER_H
