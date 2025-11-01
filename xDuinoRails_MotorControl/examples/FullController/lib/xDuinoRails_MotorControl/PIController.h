#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <cstdint>

// Forward declaration of the ControllerAction enum
enum ControllerAction {
    ACCELERATING,
    DECELERATING,
    STEADY
};


class PIController {
public:
    PIController(float kp, float ki, int max_output);
    int calculate(int target_speed, int measured_speed, int current_pwm);
    void reset();
    void setGains(float kp, float ki);
    ControllerAction getAction() const;

private:
    float kp_;
    float ki_;
    float integral_error_;
    int max_output_;
    ControllerAction action_;
};

#endif // PI_CONTROLLER_H
