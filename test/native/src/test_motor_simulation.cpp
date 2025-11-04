#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include "pi_controller.h"

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
    assert(motor.getSpeed() > 150);

    // --- Test Rangiermodus ---
    target_speed = 20; // 10% of 255 is ~25. Let's use 20.
    controller.setGains(0.15, 0.05); // Use the new Rangiermodus gains
    controller.reset(); // Reset integral error for the new phase

    for (int i = 500; i < 1000; ++i) {
        int measured_speed = motor.getSpeed();
        current_pwm = controller.calculate(target_speed, measured_speed, current_pwm);
        motor.update(current_pwm);
        results_file << i << "," << target_speed << "," << measured_speed << "," << current_pwm << std::endl;
    }

    results_file.close();
    // Assert that the speed has settled near the low target speed
    assert(motor.getSpeed() > 10 && motor.getSpeed() < 30);
}

int main() {
    test_pi_controller_simulation();
    return 0;
}
