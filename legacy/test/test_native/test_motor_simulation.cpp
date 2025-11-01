#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include "PIController.h"

// Simple motor model
class Motor {
public:
    Motor(float inertia) : inertia_(inertia), speed_(0) {}

    void update(int pwm) {
        speed_ += (pwm - speed_) * inertia_;
    }

    int getSpeed() const {
        return static_cast<int>(speed_);
    }

private:
    float inertia_;
    float speed_;
};

void test_pi_controller_simulation() {
    PIController controller(0.1, 0.1, 255);
    Motor motor(0.1);
    int target_speed = 200;
    int current_pwm = 0;

    std::ofstream results_file("simulation_results.csv");
    results_file << "Timestamp,TargetSpeed,MeasuredSpeed,PWM" << std::endl;

    for (int i = 0; i < 500; ++i) {
        int measured_speed = motor.getSpeed();
        current_pwm = controller.calculate(target_speed, measured_speed, current_pwm);
        motor.update(current_pwm);
        results_file << i << "," << target_speed << "," << measured_speed << "," << current_pwm << std::endl;
    }

    results_file.close();
    assert(motor.getSpeed() > 150);
}

int main() {
    test_pi_controller_simulation();
    return 0;
}
