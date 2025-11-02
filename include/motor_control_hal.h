/**
 * @file motor_control_hal.h
 * @brief Hardware Abstraction Layer (HAL) for low-level motor control.
 *
 * This file defines a platform-agnostic interface for hardware-accelerated
 * PWM motor control and BEMF (Back-EMF) measurement. The implementation for
 * a specific microcontroller must be provided separately.
 */
#ifndef MOTOR_CONTROL_HAL_H
#define MOTOR_CONTROL_HAL_H

#include <cstdint>

/**
 * @brief Callback function pointer type for BEMF updates.
 *
 * This function is called from an interrupt context whenever a new
 * differential BEMF measurement is available from the hardware. It is the
 * responsibility of the callee to perform any necessary filtering, processing,
 * and control logic adjustments.
 *
 * @param raw_bemf_value The raw, unfiltered differential BEMF value, calculated
 *                       as the absolute difference between the two ADC readings.
 */
typedef void (*hal_bemf_update_callback_t)(int raw_bemf_value);

/**
 * @brief Initializes the low-level hardware for motor control.
 *
 * This function configures the hardware timers, PWM peripherals, ADC, and DMA
 * for hardware-accelerated motor control and BEMF measurement. It must be
 * called once during the application's setup phase.
 *
 * @param pwm_a_pin The GPIO pin number for PWM channel A (e.g., forward).
 * @param pwm_b_pin The GPIO pin number for PWM channel B (e.g., reverse).
 * @param bemf_a_pin The GPIO pin number for ADC input connected to motor terminal A.
 * @param bemf_b_pin The GPIO pin number for ADC input connected to motor terminal B.
 * @param callback A pointer to a function that will be called from an interrupt
 *                 context with new BEMF data.
 */
void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback);

/**
 * @brief Sets the motor's PWM duty cycle and direction.
 *
 * This function updates the PWM hardware with the new duty cycle. It should
 * be called periodically from the main application loop to reflect the latest
 * output from the motor control algorithm (e.g., a PI controller).
 *
 * @param duty_cycle The desired duty cycle, typically in a range from 0 to 255.
 * @param forward The desired motor direction (true for forward, false for reverse).
 */
void hal_motor_set_pwm(int duty_cycle, bool forward);

#endif // MOTOR_CONTROL_HAL_H
