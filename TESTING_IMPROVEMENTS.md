# Plan for Improving Test Coverage

This document outlines a plan to enhance the test coverage of the `XDuinoRails_MotorDriver` library, focusing on the native test suite. The goal is to improve the stability and maintainability of the core motor control logic.

## 1. Implement PI Controller Tests

**Goal:** Verify the correct functionality of the core Proportional-Integral (PI) controller.

**Test Cases:**

*   **Test Proportional and Integral Response:**
    *   Simulate a constant error between the target speed and the measured speed.
    *   Assert that the PWM output from the controller increases over time, demonstrating both the proportional and integral components are working.
*   **Test Response to Negative Error:**
    *   Simulate a scenario where the measured speed is higher than the target speed.
    *   Assert that the PWM output decreases.
*   **Test Controller Disabling:**
    *   Use the `enablePIController(false)` method.
    *   Manually set a PWM value.
    *   Assert that the PWM value does not change after an `update()` call, confirming the PI logic was bypassed.
*   **Test Controller Reset:**
    *   Run the controller for a period with a large error to accumulate a significant integral term.
    *   Call `resetPIController()`.
    *   Assert that the PWM output is immediately reduced to a value primarily determined by the proportional term, proving the integral term was cleared.

## 2. Implement BEMF Filter Control Tests

**Goal:** Verify that the EMA and Kalman filters for BEMF smoothing can be enabled and disabled correctly.

**Test Cases:**

*   **Test Passthrough Mode:**
    *   Disable both the EMA and Kalman filters using `enableEmaFilter(false)` and `enableKalmanFilter(false)`.
    *   Provide a raw BEMF measurement.
    *   Assert that a commutation pulse is counted correctly based on the raw, unfiltered value.
*   **Test EMA Filter Functionality:**
    *   Enable only the EMA filter.
    *   Provide a series of BEMF measurements that, when smoothed, should cross the commutation threshold.
    *   Assert that a pulse is counted only after the smoothed value crosses the threshold, not on the raw value.

## 3. Implement `Rangiermodus` (Shunting Mode) Test

**Goal:** Verify that the PI controller uses a different set of gains when the target speed is within the low-speed "Rangiermodus" range.

**Test Cases:**

*   **Test Gain Switching:**
    *   Create a test-only method to set the normal and shunting mode PI gains to distinct values (e.g., `setGains_TESTONLY(kp_normal, ki_normal, kp_rangier, ki_rangier)`).
    *   **Normal Mode:** Set a target speed above the shunting threshold. Introduce a speed error and measure the resulting change in PWM.
    *   **Shunting Mode:** Set a target speed within the shunting threshold. Introduce the same speed error and measure the resulting change in PWM.
    *   Assert that the change in PWM is different between the two modes, confirming that the correct gains were applied.
*   **Test PI Reset on Transition:**
    *   Establish a high integral term by running the motor at high speed with a large error.
    *   Switch the target speed into the shunting range.
    *   Assert that the PWM value drops significantly on the next update, proving that the PI controller was reset during the transition.

## 4. Implement Test for Startup Kick and PI Controller Interaction

**Goal:** Verify that the PI controller's state is correctly reset after the startup kick completes to ensure a smooth transition to closed-loop control.

**Test Cases:**

*   **Test PI Reset After Kick:**
    *   First, disable the startup kick.
    *   Run the PI controller with a large, sustained error to wind up the integral term, resulting in a high PWM value.
    *   Re-enable the startup kick.
    *   Stop the motor (`setTargetSpeed(0)`), then set a new target speed to trigger the kick.
    *   Wait for the kick duration to elapse.
    *   On the next `update()` cycle, assert that the PWM value is now low and based only on the proportional term, proving the integral term was cleared.

## Status Update
- Tests implemented in `test/test_native/test_motor_driver.cpp` on 2025-05-27.
- Native tests verify PI logic, Rangiermodus, Startup Kick, and Safety features.
