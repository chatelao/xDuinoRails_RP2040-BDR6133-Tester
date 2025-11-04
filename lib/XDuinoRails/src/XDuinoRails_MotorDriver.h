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

    int getCurrentPWM() const;
    void setCurrentPWM(int pwm);

    ControllerAction getControllerAction() const;

private:
    XDuinoRails_MotorDriver_Impl* pImpl;
};

#endif // XDUINORAILS_MOTOR_DRIVER_H
