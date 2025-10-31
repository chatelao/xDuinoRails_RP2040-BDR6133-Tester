/**
 * @file main.cpp
 * @brief Closed-loop motor controller for a Märklin motor using BEMF feedback.
 *
 * This firmware implements a closed-loop speed control system for a DC motor
 * using a XIAO RP2040 and a BDR-6133 H-bridge driver. Speed is measured by
 * counting back-EMF (BEMF) commutation pulses. A proportional controller
 * adjusts the motor power via a non-blocking, software-generated PWM signal.
 * The system also includes a state machine to run an automatic test pattern.
 */
#include <Arduino.h>

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
int current_pwm = 0;  ///< The current PWM duty cycle (0-255), calculated by the P-controller.
bool forward = true;  ///< Current direction of the motor. true for forward, false for reverse.
unsigned long last_pwm_cycle_us = 0; ///< Timestamp (micros) of the start of the last PWM cycle.
bool bemf_measured_this_cycle = false; ///< Flag to ensure BEMF is measured only once per PWM cycle.

/**
 * @brief Initializes hardware pins and serial communication.
 *
 * This function is called once at startup. It sets the pinMode for all connected
 * pins and starts the serial port at 9600 baud.
 */
void setup() {
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);
  pinMode(bemfAPin, INPUT);
  pinMode(bemfBPin, INPUT);
  pinMode(statusLedPin, OUTPUT);
  Serial.begin(9600);
}

/**
 * @brief Measures BEMF, counts pulses, and runs the P-controller.
 *
 * This function is called during the OFF phase of each software PWM cycle.
 * It performs the following steps:
 * 1. Puts the H-bridge in a high-impedance state (coasting).
 * 2. Waits 100µs for electrical noise to settle.
 * 3. Reads the differential BEMF voltage from the ADC pins.
 * 4. Detects the rising edge of a commutation pulse and increments the counter.
 * 5. Calculates the control error and the new PWM value using the P-controller.
 */
void update_motor_pwm() {
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
    noInterrupts();
    commutation_pulse_count++;
    interrupts();
  }
  last_bemf_state = current_bemf_state;

  // 5. Run the P-controller
  // Note: Speed calculation is done in the main loop to avoid float math here.
  int measured_speed = map(measured_speed_pps, 0, 500, 0, 255); // Approximate mapping
  int error = target_speed - measured_speed;
  current_pwm = constrain(target_speed + (Kp * error), 0, max_speed);

  bemf_measured_this_cycle = true; // Mark BEMF as measured for this cycle
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

  // --- Low-level non-blocking software PWM and BEMF measurement ---
  unsigned long time_since_cycle_start = current_micros - last_pwm_cycle_us;
  long on_time_us = map(current_pwm, 0, 255, 0, pwm_period_us);

  // Check if it's time to start a new PWM cycle
  if (time_since_cycle_start >= pwm_period_us) {
    last_pwm_cycle_us = current_micros;
    time_since_cycle_start = 0; // Reset for the new cycle
    bemf_measured_this_cycle = false;
  }

  // Determine whether we are in the ON or OFF portion of the PWM pulse
  if (time_since_cycle_start < on_time_us) {
    // --- ON Portion: Drive the motor ---
    pinMode(pwmAPin, OUTPUT);
    pinMode(pwmBPin, OUTPUT);
    if (forward) {
      digitalWrite(pwmAPin, HIGH);
      digitalWrite(pwmBPin, LOW);
    } else {
      digitalWrite(pwmAPin, LOW);
      digitalWrite(pwmBPin, HIGH);
    }
  } else {
    // --- OFF Portion: Coast and measure BEMF ---
    if (!bemf_measured_this_cycle) {
      update_motor_pwm(); // This function sets pins to INPUT and measures
      bemf_measured_this_cycle = true;
    }
  }
}
