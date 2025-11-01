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
#include <Adafruit_NeoPixel.h>
#include <SimpleKalmanFilter.h>
#include "PIController.h"

//== Pin Definitions ==
const int pwmAPin = D7;      ///< Pin for PWM Forward direction (connects to InA).
const int pwmBPin = D8;      ///< Pin for PWM Reverse direction (connects to InB).
const int bemfAPin = A3;     ///< ADC pin for BEMF measurement from motor terminal A.
const int bemfBPin = A2;     ///< ADC pin for BEMF measurement from motor terminal B.
// const int statusLedPin = LED_BUILTIN; ///< Built-in LED for status indication.

//== RGB LED Definitions ==
const int NEOPIXEL_POWER_PIN = 11; ///< GPIO pin that powers the NeoPixel.
const int NEOPIXEL_PIN = 12;       ///< GPIO pin for NeoPixel data.
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//== LED Color Definitions ==
const uint32_t COLOR_BLUE = pixel.Color(0, 0, 255);
const uint32_t COLOR_GREEN = pixel.Color(0, 255, 0);
const uint32_t COLOR_RED = pixel.Color(255, 0, 0);
const uint32_t COLOR_YELLOW = pixel.Color(255, 255, 0);
const uint32_t COLOR_PINK = pixel.Color(255, 105, 180);
const uint32_t COLOR_OFF = pixel.Color(0, 0, 0);

//== Motor Control Parameters ==
const int max_speed = 255;  ///< Maximum target speed value, corresponds to max PWM duty cycle.

//== Stall Detection Parameters ==
const int stall_speed_threshold_pps = 10; ///< If speed is below this (in pulses/sec) while motor is active, it's considered stalled.
const unsigned long stall_timeout_ms = 1000; ///< Time in ms motor must be stalled before triggering the stall state.

//== Proportional-Integral Controller ==
volatile bool pi_controller_enabled = true; ///< Global flag to enable/disable the PI controller logic.
const float Kp = 0.1;       ///< Proportional gain for the PI-controller.
const float Ki = 0.1;       ///< Integral gain for the PI-controller.
PIController pi_controller(Kp, Ki, max_speed);

//== BEMF Pulse Counting ==
const float EMA_ALPHA = 0.21;   ///< Alpha for the Exponential Moving Average filter for BEMF smoothing.
// Kalman filter parameters for BEMF smoothing.
// e_mea: Measurement Uncertainty - How much we expect our measurement to vary.
// e_est: Estimation Uncertainty - Can be initialized with the same value as e_mea.
// q: Process Variance - A small number indicating how fast the measurement moves.
// These defaults are tuned for a brushed DC motor with a 3-split commutator,
// which typically has high measurement noise (e_mea) and low process variance (q).
const float BEMF_MEA_E = 2.0; ///< Measurement uncertainty.
const float BEMF_EST_E = 2.0; ///< Estimation uncertainty.
const float BEMF_Q = 0.01;    ///< Process variance.
const int bemf_threshold = 500; ///< ADC value threshold for detecting a commutation pulse.
volatile int commutation_pulse_count = 0; ///< Counter for BEMF pulses. Volatile for safe access from loop and PWM callback.
float measured_speed_pps = 0.0; ///< Calculated motor speed in pulses per second.
bool last_bemf_state = false;   ///< State of the BEMF signal in the previous cycle to detect rising edge.
SimpleKalmanFilter bemfKalmanFilter(BEMF_MEA_E, BEMF_EST_E, BEMF_Q);

//== Software PWM Parameters ==
const int pwm_frequency = 1000; ///< PWM frequency in Hz.
const long pwm_period_us = 1000000 / pwm_frequency; ///< PWM period in microseconds.

//== State Machine for Test Pattern ==
/**
 * @enum MotorState
 * @brief Defines the states for the automatic motor test pattern.
 */
enum MotorState {
    MOTOR_DITHER, ///< Applies a short, alternating signal to overcome stiction.
    RAMP_UP,      ///< Motor accelerates to max_speed.
    COAST_HIGH,   ///< Motor runs at max_speed for a fixed duration.
    RAMP_DOWN,    ///< Motor decelerates to 10% of max_speed.
    COAST_LOW,    ///< Motor runs at 10% speed for a fixed duration.
    STOP,         ///< Motor is stopped for a fixed duration before reversing.
    CHANGE_DIRECTION, ///< Brief delay while changing direction.
    MOTOR_STALLED ///< Motor has stalled and is now stopped.
};
MotorState current_state = MOTOR_DITHER; ///< Current state of the machine.

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

  // 4. Apply EMA filter to smooth the BEMF reading
  static float smoothed_bemf = 0.0;
  static bool filter_initialized = false;

  if (!filter_initialized) {
    smoothed_bemf = measured_bemf;
    filter_initialized = true;
  } else {
    smoothed_bemf = (EMA_ALPHA * measured_bemf) + ((1.0 - EMA_ALPHA) * smoothed_bemf);
  }

  // 4b. Apply Kalman filter for additional smoothing
  // The first call to updateEstimate will initialize the filter.
  float kalman_filtered_bemf = bemfKalmanFilter.updateEstimate(smoothed_bemf);

  // 5. Detect rising edge of a commutation pulse using the final filtered value
  bool current_bemf_state = (kalman_filtered_bemf > bemf_threshold);
  if (current_bemf_state && !last_bemf_state) {
    // This is accessed by the main loop, so it needs to be atomic.
    // However, we are in an interrupt, so noInterrupts() is not the right tool.
    // For a single-core MCU and a single-instruction increment, it's often okay,
    // but a proper atomic operation would be better in a multi-core scenario.
    commutation_pulse_count++;
  }
  last_bemf_state = current_bemf_state;

  // 6. Run the PI-controller if it's enabled
  if (pi_controller_enabled) {
    // Note: Speed calculation is done in the main loop to avoid float math here.
    int measured_speed = map(measured_speed_pps, 0, 500, 0, 255); // Approximate mapping
    current_pwm = pi_controller.calculate(target_speed, measured_speed, current_pwm);
  }

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
  // pinMode(statusLedPin, OUTPUT);
  Serial.begin(9600);

  // Initialize the NeoPixel
  pinMode(NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(50); // Set a reasonable brightness
  pixel.show(); // Initialize all pixels to 'off'

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
    uint32_t color = COLOR_OFF;
    bool blink_state = true; // Solid by default

    // Determine color based on motor state and controller action
    switch (current_state) {
        case STOP:
            color = COLOR_BLUE;
            break;
        case CHANGE_DIRECTION:
            color = COLOR_PINK;
            break;
        case MOTOR_STALLED:
            color = COLOR_RED; // Stalled is a high-priority error color
            break;
        case RAMP_UP:
        case RAMP_DOWN:
        case COAST_HIGH:
        case COAST_LOW:
            // For all moving states, the color depends on the P-controller's action.
            ControllerAction action = pi_controller.getAction();

            switch (action) {
                case ACCELERATING:
                    color = COLOR_GREEN;
                    break;
                case DECELERATING:
                    color = COLOR_RED;
                    break;
                case STEADY:
                    color = COLOR_YELLOW;
                    break;
            }
            break;
    }

    // Determine blink pattern based on high-level state
    switch (current_state) {
        case RAMP_DOWN:
            blink_state = (current_millis / 500) % 2; // Slow blink
            break;
        case CHANGE_DIRECTION:
            blink_state = (current_millis / 100) % 2; // Fast blink
            break;
        case MOTOR_STALLED:
            blink_state = (current_millis / 50) % 2;  // Very fast blink for error
            break;
        default:
            blink_state = true; // Solid for all other states
            break;
    }

    // Set the pixel color
    if (blink_state) {
        pixel.setPixelColor(0, color);
    } else {
        pixel.setPixelColor(0, COLOR_OFF);
    }
    pixel.show();
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
    case MOTOR_DITHER:
      // On first entry, disable the PI controller for direct PWM control.
      if (time_in_state == 0) {
        pi_controller_enabled = false;
      }

      // Run dither sequence for 80ms
      if (time_in_state < 80) {
        // 100 Hz means a 10ms period. 5ms forward, 5ms reverse.
        bool dither_direction = (time_in_state / 5) % 2 == 0;
        forward = dither_direction; // Directly control direction
        current_pwm = max_speed * 0.15; // 15% duty cycle
      } else {
        // Dither complete. Transition to ramp up.
        pi_controller_enabled = true; // Re-enable PI controller
        current_pwm = 0; // Reset PWM before ramping
        target_speed = 0;
        current_state = RAMP_UP;
        state_start_ms = current_millis;
      }
      break;

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
      pi_controller.reset(); // Reset integral error when stopping
     }
     break;

    case STOP:
     if (time_in_state >= 2000) {
      current_state = CHANGE_DIRECTION;
      state_start_ms = current_millis;
     }
     break;

    case CHANGE_DIRECTION:
     if (time_in_state >= 500) { // A brief 500ms delay for the fast blink
        forward = !forward; // Change direction
        pi_controller.reset(); // Reset integral error on direction change
        current_state = MOTOR_DITHER;
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
