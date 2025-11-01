# Advanced Motor Control Concepts

This document outlines several advanced concepts for improving the performance, robustness, and adaptability of the BEMF-based motor controller. These concepts serve as a foundation for future development and enhancements.

## 1. Handling of Additional Control Cases

Beyond simple start, stop, and stall conditions, a robust motor controller should gracefully handle a variety of real-world scenarios.

### Sudden Load Changes

*   **Scenario:** The motor experiences a rapid change in load, such as a locomotive starting up a steep incline or entering a sharp curve.
*   **Behavior:** The PI controller's primary role is to manage this. A sudden load increase will cause the motor to slow down, increasing the error between the target speed and the actual speed. The integral term of the PI controller will accumulate this error and increase the PWM output to compensate, providing more power to maintain the target speed.
*   **Considerations:** The PI gains (`Kp`, `Ki`) must be well-tuned to provide a response that is quick but does not overshoot and cause oscillation.

### Over-speed Condition

*   **Scenario:** The motor is driven faster than its target speed by an external force, such as a locomotive going down a steep decline.
*   **Behavior:** In this state, the BEMF-derived speed will exceed the setpoint. The PI controller will reduce the PWM output, potentially to zero.
*   **Advanced Behavior (Active Braking):** For more aggressive speed regulation, the controller could implement active braking by briefly shorting the motor terminals (setting both half-bridges low) or reversing the H-bridge polarity for a very short duration. This is more complex and requires careful implementation to avoid damaging the motor or driver. For now, the default behavior is to enter a high-impedance coasting state by setting PWM to zero.

### Sensor Errors

*   **Scenario:** BEMF readings become erratic, noisy, or nonsensical due to issues like a loose connection, severe electrical noise from the commutator, or a fault in the measurement circuit.
*   **Behavior:** The controller should be able to detect invalid readings. A simple approach is to check if the BEMF readings fall within an expected range or if they change more rapidly than is physically possible (a "rate of change" check).
*   **Action:** If an error is detected, the safest action is to transition to a `MOTOR_FAULT` state, shut down the motor, and signal the error via the status LED. This prevents unpredictable behavior.

## 2. Low-Speed Control (Rangiermodus / Shunting Mode)

*   **Problem:** At very low speeds, the BEMF signal is weak and noisy, making it difficult to use for stable closed-loop (PI) control. This can result in jerky or unreliable movement.
*   **Solution:** Implement a special "shunting mode" for low-speed operation.
    *   **Open-Loop Control:** Below a certain speed threshold, disable the PI controller and switch to open-loop control. The controller will output a fixed, pre-determined PWM signal instead of relying on BEMF feedback.
    *   **Low-Frequency PWM:** Using a lower PWM frequency at very low speeds can sometimes provide smoother torque and finer control, helping to overcome motor stiction.

## 3. Auto-Calibration for New Motors

To make the controller adaptable to different motors, an automated calibration routine can be implemented. This routine would run once for a new motor to determine its key characteristics.

### Phase 1: Determine Motor Constants

1.  **Measure Winding Resistance:** While the motor is stopped, apply a short, low-voltage pulse and measure the current to calculate the internal winding resistance. This is useful for more advanced control algorithms.
2.  **Find Breakaway PWM:** Slowly ramp up the PWM duty cycle from zero until the motor starts to turn (detected by a non-zero BEMF reading). This value is the minimum power required to overcome static friction.

### Phase 2: Profile the Speed Curve

1.  **Step through PWM values:** Run the motor at several different PWM duty cycles (e.g., 20%, 40%, 60%, 80%, 100%).
2.  **Record Stable Speed:** At each step, wait for the speed to stabilize and record the resulting BEMF value.
3.  **Generate a Curve:** This data creates a profile of PWM duty cycle vs. motor speed, which is crucial for the PI controller to know the expected speed for a given power input.

## 4. Kalman Filter Tuning

The Kalman filter is essential for smoothing the noisy BEMF measurements to get a reliable speed estimate. Its performance is highly dependent on its tuning parameters, which should be adjusted for each motor.

*   **`e_mea` (Measurement Uncertainty):** This parameter reflects how noisy the BEMF measurements are.
    *   A motor with a 3-pole commutator will generate significant electrical noise, requiring a **higher** `e_mea`.
    *   A coreless motor or one with many commutator segments will be much smoother, allowing for a **lower** `e_mea`.
*   **`e_est` (Estimation Uncertainty):** The initial uncertainty of the filter's estimate. This is less critical but can be tuned.
*   **`q` (Process Variance):** This represents how much the motor's true speed is expected to change between measurements.
    *   A heavy locomotive with high inertia will not change speed quickly, so it should have a **lower** `q`.
    *   A small, light motor with low inertia can change speed rapidly, suggesting a **higher** `q`.

**Tuning as part of Calibration:** An advanced calibration routine could potentially analyze the noise characteristics of the BEMF signal at a steady speed to suggest starting values for `e_mea` and `q`.
