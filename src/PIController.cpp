#include "PIController.h"

// A utility function for constraining a value within a range
template<typename T>
T constrain(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}


PIController::PIController(float kp, float ki, int max_output)
    : kp_(kp), ki_(ki), integral_error_(0.0), max_output_(max_output), action_(STEADY) {}

int PIController::calculate(int target_speed, int measured_speed, int current_pwm) {
    int error = target_speed - measured_speed;

    // Conditional Integration: only accumulate error if the output is not saturated.
    if (current_pwm < max_output_) {
        integral_error_ += error;
    }

    int adjustment = (kp_ * error) + (ki_ * integral_error_);
    int new_pwm = constrain(target_speed + adjustment, 0, max_output_);

    if (new_pwm > current_pwm) {
        action_ = ACCELERATING;
    } else if (new_pwm < current_pwm) {
        action_ = DECELERATING;
    } else {
        action_ = STEADY;
    }

    return new_pwm;
}

void PIController::reset() {
    integral_error_ = 0.0;
}

ControllerAction PIController::getAction() const {
    return action_;
}
