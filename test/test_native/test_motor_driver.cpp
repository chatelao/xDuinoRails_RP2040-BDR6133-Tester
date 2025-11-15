#include "XDuinoRails_MotorDriver.h"
#include "MockKalmanFilter.h"
#include <cassert>
#include <iostream>

// --- Mock HAL & Arduino functions ---
extern unsigned long mock_millis_count;

void advance_time_ms(unsigned long ms) {
    mock_millis_count += ms;
}

// Provide simple stub implementations for the HAL functions to satisfy the linker.
extern "C" {
    void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, void (*callback)(int)) {}
    void hal_motor_set_pwm(int duty_cycle, bool forward) {}
}

void test_acceleration_deceleration() {
    std::cout << "--- Running Test: test_acceleration_deceleration ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    // Disable stall detection for this test
    driver.setStallDetection(false);

    // --- Test Acceleration ---
    std::cout << "  Testing Acceleration..." << std::endl;
    driver.setAcceleration(50); // 50 speed units per second
    driver.setDeceleration(0);
    driver.setTargetSpeed(100);

    // Simulate updates.
    for (int i = 0; i < 100; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    float current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 1s: " << current_setpoint << std::endl;
    assert(current_setpoint > 48 && current_setpoint < 52);

    // Simulate another second
    for (int i = 0; i < 100; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 2s: " << current_setpoint << std::endl;
    assert(current_setpoint == 100);


    // --- Test Deceleration ---
    std::cout << "  Testing Deceleration..." << std::endl;
    driver.setAcceleration(0);
    driver.setDeceleration(100); // 100 speed units per second
    driver.setTargetSpeed(0);

    // Simulate 0.5 seconds of updates
    for (int i = 0; i < 50; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 0.5s deceleration: " << current_setpoint << std::endl;
    assert(current_setpoint > 48 && current_setpoint < 52);


    // Simulate another 0.5 seconds
    for (int i = 0; i < 50; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 1s deceleration: " << current_setpoint << std::endl;
    assert(current_setpoint == 0);

    std::cout << "--- Test Passed: test_acceleration_deceleration ---" << std::endl;
}

void test_stall_detection() {
    std::cout << "--- Running Test: test_stall_detection ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setTargetSpeed(100);
    driver.update(); // Set the initial target speed

    // Simulate 1.5 seconds of a stalled motor (measured speed is 0)
    for (int i = 0; i < 150; ++i) {
        advance_time_ms(10);
        driver.update();
    }

    int final_speed = driver.getTargetSpeed();
    std::cout << "  Target speed after 1.5s stalled: " << final_speed << std::endl;
    assert(final_speed == 0);
    std::cout << "--- Test Passed: test_stall_detection ---" << std::endl;
}

void test_startup_kick() {
    std::cout << "--- Running Test: test_startup_kick ---" << std::endl;
    mock_millis_count = 0; // Reset time for this test

    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setStartupKick(150, 100); // 150 PWM for 100ms
    driver.setTargetSpeed(100); // This should trigger the kick

    advance_time_ms(10);
    driver.update();
    int pwm = driver.getCurrentPWM();
    std::cout << "  PWM immediately after kick start: " << pwm << std::endl;
    assert(pwm == 150);

    // Simulate 50ms of updates
    advance_time_ms(50);
    driver.update();
    pwm = driver.getCurrentPWM();
    std::cout << "  PWM after 50ms: " << pwm << std::endl;
    assert(pwm == 150);

    // Simulate another 60ms (total 110ms, kick should be over)
    advance_time_ms(60);
    driver.update();
    pwm = driver.getCurrentPWM();
    std::cout << "  PWM after 110ms: " << pwm << std::endl;
    assert(pwm != 150);

    std::cout << "--- Test Passed: test_startup_kick ---" << std::endl;
}


void test_watchdog() {
    std::cout << "--- Running Test: test_watchdog ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setWatchdogTimeout(100); // 100ms timeout
    driver.setTargetSpeed(150);
    driver.update();

    assert(driver.getTargetSpeed() == 150);

    // Advance time by 50ms, watchdog should not trigger
    advance_time_ms(50);
    driver.update();
    std::cout << "  Speed after 50ms: " << driver.getTargetSpeed() << std::endl;
    assert(driver.getTargetSpeed() == 150);

    // Reset watchdog by sending a new speed command
    driver.setTargetSpeed(150);
    advance_time_ms(60);
    driver.update();
    std::cout << "  Speed after another 60ms (with reset): " << driver.getTargetSpeed() << std::endl;
    assert(driver.getTargetSpeed() == 150);

    // Advance time by 110ms, watchdog should trigger
    advance_time_ms(110);
    driver.update();
    std::cout << "  Speed after 110ms (timeout expected): " << driver.getTargetSpeed() << std::endl;
    assert(driver.getTargetSpeed() == 0);

    std::cout << "--- Test Passed: test_watchdog ---" << std::endl;
}

int main() {
    test_acceleration_deceleration();
    test_stall_detection();
    test_startup_kick();
    test_watchdog();
    return 0;
}
