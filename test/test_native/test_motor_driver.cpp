#include "XDuinoRails_MotorControl.h"
#include "MockKalmanFilter.h"
#include <cassert>
#include <iostream>
#include <cmath>

// --- Mock HAL & Arduino functions ---
extern unsigned long mock_millis_count;
void (*global_bemf_callback)(int) = nullptr;

void advance_time_ms(unsigned long ms) {
    mock_millis_count += ms;
}

extern "C" {
    void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, void (*callback)(int)) {
        global_bemf_callback = callback;
    }
    void hal_motor_set_pwm(int duty_cycle, bool forward) {}
}

void test_acceleration_deceleration() {
    // Goal: Verify acceleration and deceleration ramps work as expected.
    // Steps:
    // 1. Set Accel, Set Target. Verify intermediate speed.
    // 2. Verify final speed.
    // 3. Set Decel, Set Target 0. Verify intermediate speed.
    // 4. Verify final speed.
    // Ref: Core Logic - Acceleration
    std::cout << "--- Running Test: test_acceleration_deceleration ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setStallDetection(false);

    std::cout << "  Testing Acceleration..." << std::endl;
    driver.setAcceleration(50);
    driver.setDeceleration(0);
    driver.setTargetSpeed(100);

    for (int i = 0; i < 100; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    float current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 1s: " << current_setpoint << std::endl;
    assert(current_setpoint > 48 && current_setpoint < 52);

    for (int i = 0; i < 100; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 2s: " << current_setpoint << std::endl;
    assert(current_setpoint == 100);

    std::cout << "  Testing Deceleration..." << std::endl;
    driver.setAcceleration(0);
    driver.setDeceleration(100);
    driver.setTargetSpeed(0);

    for (int i = 0; i < 50; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    current_setpoint = driver.getCurrentSpeedSetpoint();
    std::cout << "  Speed setpoint after 0.5s deceleration: " << current_setpoint << std::endl;
    assert(current_setpoint > 48 && current_setpoint < 52);

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
    // Goal: Verify that motor stops if speed is 0 for timeout period.
    // Steps: Set target. Simulate 0 speed. Wait timeout. Verify target becomes 0.
    // Ref: Core Logic - Stall Detection
    std::cout << "--- Running Test: test_stall_detection ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setTargetSpeed(100);
    driver.update();

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
    // Goal: Verify startup kick PWM and duration.
    // Steps: Configure kick. Set target. Verify PWM is Kick PWM. Wait. Verify PWM returns to PI.
    // Ref: Core Logic - Startup Kick
    std::cout << "--- Running Test: test_startup_kick ---" << std::endl;
    mock_millis_count = 0;

    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setStartupKick(150, 100);
    driver.setTargetSpeed(100);

    advance_time_ms(10);
    driver.update();
    int pwm = driver.getCurrentPWM();
    std::cout << "  PWM immediately after kick start: " << pwm << std::endl;
    assert(pwm == 150);

    advance_time_ms(50);
    driver.update();
    pwm = driver.getCurrentPWM();
    std::cout << "  PWM after 50ms: " << pwm << std::endl;
    assert(pwm == 150);

    advance_time_ms(60);
    driver.update();
    pwm = driver.getCurrentPWM();
    std::cout << "  PWM after 110ms: " << pwm << std::endl;
    assert(pwm != 150);

    std::cout << "--- Test Passed: test_startup_kick ---" << std::endl;
}


void test_watchdog() {
    // Goal: Verify watchdog stops motor if no target command received.
    // Steps: Set timeout. Set target. Wait < timeout. Check speed. Wait > timeout. Check speed 0.
    // Ref: Core Logic - Safety
    std::cout << "--- Running Test: test_watchdog ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();

    driver.setWatchdogTimeout(100);
    driver.setTargetSpeed(150);
    driver.update();

    assert(driver.getTargetSpeed() == 150);

    advance_time_ms(50);
    driver.update();
    assert(driver.getTargetSpeed() == 150);

    driver.setTargetSpeed(150);
    advance_time_ms(60);
    driver.update();
    assert(driver.getTargetSpeed() == 150);

    advance_time_ms(110);
    driver.update();
    assert(driver.getTargetSpeed() == 0);

    std::cout << "--- Test Passed: test_watchdog ---" << std::endl;
}

void test_pi_controller_response() {
    // Goal: Verify PI controller calculates PWM based on error.
    // Steps: Set target. Error=Target. Check PWM > 0. Wait. Check PWM increases (Integral).
    // Ref: PI Controller Logic
    std::cout << "--- Running Test: test_pi_controller_response ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();
    driver.setAcceleration(0);
    driver.setStartupKick(0, 0);
    driver.setStallDetection(false);

    driver.setTargetSpeed(100);
    driver.update();
    int pwm1 = driver.getCurrentPWM();
    std::cout << "  PWM 1 (Initial P-only): " << pwm1 << std::endl;
    // With Kp=0.1, Error=100, P=10. PWM = Target + Adj = 100 + 10 = 110.
    assert(pwm1 > 100);

    for(int i=0; i<20; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    int pwm2 = driver.getCurrentPWM();
    std::cout << "  PWM 2 (After Integral): " << pwm2 << std::endl;
    assert(pwm2 > pwm1);

    std::cout << "--- Test Passed: test_pi_controller_response ---" << std::endl;
}

void test_rangiermodus() {
    // Goal: Verify gains switch for low speeds (Rangiermodus).
    // Steps: Test Normal Mode response. Test Rangier Mode response. Compare implied Kp.
    // Ref: Core Logic - Rangiermodus
    std::cout << "--- Running Test: test_rangiermodus ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();
    driver.setAcceleration(0);
    driver.setStartupKick(0, 0);
    driver.setStallDetection(false);

    // Normal Mode: Target 100. (Kp = 0.1)
    // We need to prevent Integral from interfering, so check first update only.
    // But resetPIController happens on mode switch.

    // Test Rangier Mode (Target 20 <= 25.5)
    driver.setTargetSpeed(20);
    driver.update();
    int pwm_rangier = driver.getCurrentPWM();
    // Error = 20.
    // Kp=0.15 -> P = 3.
    // Ki=0.05 -> I = 1. (Integral accumulates immediately: 20 * 0.05 = 1)
    // Adj = 4. PWM = 20 + 4 = 24.
    std::cout << "  Rangier Mode PWM (Target 20): " << pwm_rangier << " (Expected 24)" << std::endl;
    assert(pwm_rangier == 24);

    // Test Normal Mode (Target 50 > 25.5)
    driver.setTargetSpeed(50);
    driver.update();
    int pwm_normal = driver.getCurrentPWM();
    // Error = 50.
    // Kp=0.1 -> P = 5.
    // Ki=0.1 -> I = 5. (Integral accumulates immediately: 50 * 0.1 = 5)
    // Adj = 10. PWM = 50 + 10 = 60.
    std::cout << "  Normal Mode PWM (Target 50): " << pwm_normal << " (Expected 60)" << std::endl;
    assert(pwm_normal == 60);

    std::cout << "--- Test Passed: test_rangiermodus ---" << std::endl;
}

void test_startup_kick_pi_reset() {
    // Goal: Verify PI controller is reset after startup kick to prevent integral windup.
    // Steps:
    // 1. Build up integral error manually (disable kick).
    // 2. Enable kick. Stop motor.
    // 3. Start motor (trigger kick). Wait for kick end.
    // 4. Verify PWM is not influenced by previous integral.
    // Ref: Core Logic - Startup Kick / PI
    std::cout << "--- Running Test: test_startup_kick_pi_reset ---" << std::endl;
    mock_millis_count = 0;
    MockKalmanFilter* mock_filter = new MockKalmanFilter();
    XDuinoRails_MotorDriver driver(0, 1, 2, 3, mock_filter);
    driver.begin();
    driver.setAcceleration(0);
    driver.setStallDetection(false);

    // 1. Wind up integral
    driver.setStartupKick(0, 0);
    driver.setTargetSpeed(100);
    for(int i=0; i<50; ++i) {
        advance_time_ms(10);
        driver.update();
    }
    int pwm_windup = driver.getCurrentPWM();
    std::cout << "  PWM with windup: " << pwm_windup << std::endl;
    assert(pwm_windup > 120); // 100 + 10(P) + Integral

    // 2. Enable Kick and Stop
    driver.setStartupKick(150, 100);
    driver.setTargetSpeed(0);
    driver.update();
    // Speed setpoint should be 0. PWM should be ...?
    // update() -> ramp -> 0. PI -> 0.

    // 3. Start with Kick
    driver.setTargetSpeed(100); // Triggers kick
    advance_time_ms(10);
    driver.update();
    assert(driver.getCurrentPWM() == 150); // Kick active

    // 4. Wait for kick end
    advance_time_ms(110); // Total 120ms
    driver.update();
    int pwm_after_kick = driver.getCurrentPWM();
    std::cout << "  PWM after kick: " << pwm_after_kick << std::endl;

    // Should be P-only (approx 110) + slight integral from 1 cycle?
    // Should NOT be > 120 like before.
    assert(pwm_after_kick < pwm_windup);
    assert(pwm_after_kick >= 110);

    std::cout << "--- Test Passed: test_startup_kick_pi_reset ---" << std::endl;
}

int main() {
    test_acceleration_deceleration();
    test_stall_detection();
    test_startup_kick();
    test_watchdog();
    test_pi_controller_response();
    test_rangiermodus();
    test_startup_kick_pi_reset();
    return 0;
}
