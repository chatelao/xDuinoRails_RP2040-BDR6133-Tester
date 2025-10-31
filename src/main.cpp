/**
 * @file main.cpp
 * @brief Closed-loop motor controller for a Märklin motor using BEMF feedback.
 *
 * This firmware implements a closed-loop speed control system for a DC motor
 * using a XIAO RP2040 and a BDR-6133 H-bridge driver. Speed is measured by
 * counting back-EMF (BEMF) commutation pulses. A proportional controller
 * adjusts the motor power via a non-blocking, hardware-accelerated PWM signal
 * using the RP2040's timers. The system also includes a state machine to
 * run an automatic test pattern.
 */
#include <Arduino.h>
#include "pico/time.h"

//== Pin Definitions ==
const int pwmAPin = D7;      ///< Pin for PWM Forward direction (connects to InA).
const int pwmBPin = D8;      ///< Pin for PWM Reverse direction (connects to InB).
const int bemfAPin = A3;     ///< ADC pin for BEMF measurement from motor terminal A.
const int bemfBPin = A2;     ///< ADC pin for BEMF measurement from motor terminal B.
const int statusLedPin = LED_BUILTIN; ///< Built-in LED for status indication.

//== Motor Control Parameters ==
const int max_speed = 255;  ///< Maximum target speed value, corresponds to max PWM duty cycle.

//== Stall Detection Parameters ==
const int stall_speed_threshold_pps = 10; ///< If speed is below this (in pulses/sec) while motor is active, it's considered stalled.
const unsigned long stall_timeout_ms = 1000; ///< Time in ms motor must be stalled before triggering the stall state.

//== Proportional Controller ==
const float Kp = 0.1;       ///< Proportional gain for the P-controller.

//== BEMF Pulse Counting ==
const int bemf_threshold = 500; ///< ADC value threshold for detecting a commutation pulse.
volatile int commutation_pulse_count = 0; ///< Counter for BEMF pulses. Volatile for safe access from loop and PWM callback.
float measured_speed_pps = 0.0; ///< Calculated motor speed in pulses per second.
bool last_bemf_state = false;   ///< State of the BEMF signal in the previous cycle to detect rising edge.

//== Software PWM Parameters ==
const int pwm_frequency = 1000; ///< PWM frequency in Hz.
const long pwm_period_us = 1000000 / pwm_frequency; ///< PWM period in microseconds.

//== State Machine for Test Pattern ==
/**
 * @enum MotorState
 * @brief Defines the states for the automatic motor test pattern.
 */
enum MotorState {
    RAMP_UP,      ///< Motor accelerates to max_speed.
    COAST_HIGH,   ///< Motor runs at max_speed for a fixed duration.
    RAMP_DOWN,    ///< Motor decelerates to 10% of max_speed.
    COAST_LOW,    ///< Motor runs at 10% speed for a fixed duration.
    STOP,         ///< Motor is stopped for a fixed duration before reversing.
    MOTOR_STALLED ///< Motor has stalled and is now stopped.
};
MotorState current_state = RAMP_UP; ///< Current state of the machine.
unsigned long state_start_ms = 0;   ///< Timestamp (millis) when the current state was entered.
unsigned long last_ramp_update_ms = 0; ///< Timestamp of the last speed adjustment during a ramp.
const int ramp_step_delay_ms = 20; ///< Delay between speed steps during ramps.

//== Global Motor & PWM State Variables ==
int target_speed = 0; ///< The desired speed for the motor, set by the state machine.
volatile int current_pwm = 0;  ///< The current PWM duty cycle (0-255), calculated by the P-controller. Volatile for interrupt access.
bool forward = true;  ///< Current direction of the motor. true for forward, false for reverse.
struct repeating_timer pwm_timer; ///< Holds the repeating timer instance for the PWM cycle.

/**
 * @brief Timer callback for the PWM OFF phase.
 *
 * This function is scheduled by the ON-phase callback. It coasts the motor,
 * waits, measures BEMF, and runs the P-controller. It returns 0 because it's
 * a one-shot alarm and should not be re-scheduled automatically.
 *
 * @param alarm_id The ID of the alarm that triggered.
 * @return 0
 */
int64_t pwm_off_callback(alarm_id_t alarm_id, void *user_data) {
  // 1. Put H-Bridge into high-impedance state to measure BEMF
  pinMode(pwmAPin, INPUT);
  pinMode(pwmBPin, INPUT);
  // 2. Wait for voltage to stabilize
  delayMicroseconds(100);

  // 3. Perform differential BEMF measurement
  int bemfA = analogRead(bemfAPin);
  int bemfB = analogRead(bemfBPin);
  int measured_bemf = abs(bemfA - bemfB);

  // 4. Detect rising edge of a commutation pulse
  bool current_bemf_state = (measured_bemf > bemf_threshold);
  if (current_bemf_state && !last_bemf_state) {
    // This is accessed by the main loop, so it needs to be atomic.
    // However, we are in an interrupt, so noInterrupts() is not the right tool.
    // For a single-core MCU and a single-instruction increment, it's often okay,
    // but a proper atomic operation would be better in a multi-core scenario.
    commutation_pulse_count++;
  }
  last_bemf_state = current_bemf_state;

  // 5. Run the P-controller
  // Note: Speed calculation is done in the main loop to avoid float math here.
  int measured_speed = map(measured_speed_pps, 0, 500, 0, 255); // Approximate mapping
  int error = target_speed - measured_speed;

  // Calculate new PWM value and ensure it's within bounds
  int new_pwm = constrain(target_speed + (Kp * error), 0, max_speed);
  current_pwm = new_pwm; // Update the volatile PWM value

  return 0; // Does not repeat
}

/**
 * @brief Timer callback for the PWM ON phase.
 *
 * This function is called by a repeating hardware timer. It drives the motor
 * for the ON portion of the PWM cycle and schedules the one-shot OFF-phase
 * callback.
 *
 * @param t A pointer to the repeating_timer_t structure.
 * @return true to continue the repeating timer.
 */
bool pwm_on_callback(struct repeating_timer *t) {
  // Read the latest PWM value. On a 32-bit core, reading a volatile int is atomic.
  int pwm_val = current_pwm;

  // Calculate the ON time for this cycle
  long on_time_us = map(pwm_val, 0, 255, 0, pwm_period_us);

  // Only drive the motor if there is a non-zero ON time
  if (on_time_us > 0) {
    // Set pins to OUTPUT to drive the H-bridge
    pinMode(pwmAPin, OUTPUT);
    pinMode(pwmBPin, OUTPUT);
    if (forward) {
      digitalWrite(pwmAPin, HIGH);
      digitalWrite(pwmBPin, LOW);
    } else {
      digitalWrite(pwmAPin, LOW);
      digitalWrite(pwmBPin, HIGH);
    }
  }

  // Schedule the OFF callback to execute after the ON time.
  // If on_time is 0, this will effectively be scheduled for "now",
  // which is fine. The BEMF measurement will happen immediately.
  add_alarm_in_us(on_time_us, pwm_off_callback, NULL, true);

  return true; // Keep the repeating timer going
}

/**
 * @brief Initializes hardware pins, serial communication, and hardware timers.
 *
 * This function is called once at startup. It sets up ADC and status pins,
 * starts serial, and configures the repeating timer for the PWM cycle.
 */
void setup() {
  pinMode(bemfAPin, INPUT);
  pinMode(bemfBPin, INPUT);
  pinMode(statusLedPin, OUTPUT);
  Serial.begin(9600);

  // Initialize the hardware timer for the PWM base frequency
  add_repeating_timer_us(pwm_period_us, pwm_on_callback, NULL, &pwm_timer);
}

/**
 * @brief Updates the status LED based on the current motor state.
 *
 * Provides visual feedback on the motor's operation:
 * - RAMP_UP: Solid ON
 * - RAMP_DOWN: Slow blink
 * - COAST_HIGH/LOW: Solid ON
 * - STOP: Fast blink
 */
void update_status_light() {
    unsigned long current_millis = millis();
    switch (current_state) {
        case RAMP_UP:
            digitalWrite(statusLedPin, HIGH);
            break;
        case RAMP_DOWN:
            digitalWrite(statusLedPin, (current_millis / 500) % 2);
            break;
        case COAST_HIGH:
        case COAST_LOW:
            digitalWrite(statusLedPin, HIGH);
            break;
        case STOP:
            digitalWrite(statusLedPin, (current_millis / 100) % 2);
            break;
        case MOTOR_STALLED:
            digitalWrite(statusLedPin, (current_millis / 50) % 2); // Very fast blink
            break;
        default:
            digitalWrite(statusLedPin, LOW);
            break;
    }
}

/**
 * @brief Main application loop.
 *
 * This function runs repeatedly and contains the main logic for:
 * - Updating the status light.
 * - Calculating motor speed (PPS) every 100ms.
 * - Managing the high-level state machine for the test pattern.
 * - Executing the low-level, non-blocking software PWM loop.
 */
void loop() {
  update_status_light();
  unsigned long current_millis = millis();
  unsigned long current_micros = micros();

  // --- Speed calculation based on commutation pulses ---
  static unsigned long last_speed_calc_ms = 0;
  if (current_millis - last_speed_calc_ms >= 100) { // Calculate speed every 100ms
    // Atomically read and reset the pulse counter
    noInterrupts();
    int pulses = commutation_pulse_count;
    commutation_pulse_count = 0;
    interrupts();

    float elapsed_time_s = (current_millis - last_speed_calc_ms) / 1000.0;
    measured_speed_pps = pulses / elapsed_time_s;
    last_speed_calc_ms = current_millis;
  }

  // --- Stall Detection Logic ---
  static unsigned long stall_check_start_ms = 0;

  // A stall can only occur if the motor is supposed to be moving but isn't.
  if (target_speed > 0 && measured_speed_pps < stall_speed_threshold_pps) {
    if (stall_check_start_ms == 0) { // If this is the start of a potential stall...
      stall_check_start_ms = current_millis; // ...record the start time.
    } else if (current_millis - stall_check_start_ms >= stall_timeout_ms) {
      // If the condition persists long enough, trigger the stalled state.
      current_state = MOTOR_STALLED;
      state_start_ms = current_millis;
      target_speed = 0; // Stop the motor immediately.
    }
  } else {
    // If the motor is moving as expected or is supposed to be stopped, reset the timer.
    stall_check_start_ms = 0;
  }

  // --- High-level state machine for the test pattern ---
  unsigned long time_in_state = current_millis - state_start_ms;

  switch (current_state) {
    case RAMP_UP:
     if (current_millis - last_ramp_update_ms >= ramp_step_delay_ms) {
      last_ramp_update_ms = current_millis;
      if (target_speed < max_speed) {
        target_speed++;
      } else {
        current_state = COAST_HIGH;
        state_start_ms = current_millis;
      }
     }
     break;

    case COAST_HIGH:
     if (time_in_state >= 3000) {
      current_state = RAMP_DOWN;
      state_start_ms = current_millis;
     }
     break;

    case RAMP_DOWN:
     if (current_millis - last_ramp_update_ms >= ramp_step_delay_ms) {
      last_ramp_update_ms = current_millis;
      if (target_speed > max_speed * 0.1) {
        target_speed--;
      } else {
        current_state = COAST_LOW;
        state_start_ms = current_millis;
      }
     }
     break;

    case COAST_LOW:
     if (time_in_state >= 3000) {
      current_state = STOP;
      state_start_ms = current_millis;
      target_speed = 0; // Ensure motor is stopped
     }
     break;

    case STOP:
     if (time_in_state >= 2000) {
      forward = !forward; // Change direction
      current_state = RAMP_UP;
      state_start_ms = current_millis;
     }
     break;
    case MOTOR_STALLED:
      // Motor is stopped and remains in this state.
      // The LED will blink rapidly to indicate the error.
      // A reset is required to clear this state.
      break;
  }

  // With the hardware timer handling the PWM and BEMF measurement,
  // the main loop is now much simpler and only needs to manage the
  // high-level state machine and speed calculations. The CPU load is
  // significantly reduced.
}
