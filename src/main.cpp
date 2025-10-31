#include <Arduino.h>

// Corrected Pin Assignments
const int pwmAPin = D7;      // PWM Forward (OutA)
const int pwmBPin = D8;      // PWM Reverse (OutB)
const int bemfAPin = A3;     // BEMF A (bEMFA)
const int bemfBPin = A2;     // BEMF B (bEMFB)
const int statusLedPin = LED_BUILTIN; // Status LED

// Motor control parameters
const int max_speed = 255;

// Proportional control gain
const float Kp = 0.1;

// Commutation pulse counting
const int bemf_threshold = 500; // Threshold for detecting a pulse
volatile int commutation_pulse_count = 0; // Use volatile for safe interrupt access
float measured_speed_pps = 0.0; // Pulses per second
bool last_bemf_state = false;

// Software PWM parameters
const int pwm_frequency = 1000; // Hz
const long pwm_period_us = 1000000 / pwm_frequency;

// --- State Machine ---
enum MotorState { RAMP_UP, COAST_HIGH, RAMP_DOWN, COAST_LOW, STOP };
MotorState current_state = RAMP_UP;
unsigned long state_start_ms = 0;
unsigned long last_ramp_update_ms = 0;
const int ramp_step_delay_ms = 20; // Time between speed increments/decrements

// Motor and PWM state
int target_speed = 0;
int current_pwm = 0;
bool forward = true;
unsigned long last_pwm_cycle_us = 0;
bool bemf_measured_this_cycle = false;

void setup() {
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);
  pinMode(bemfAPin, INPUT);
  pinMode(bemfBPin, INPUT);
  pinMode(statusLedPin, OUTPUT);
  Serial.begin(9600);
}

void update_motor_pwm() {
  pinMode(pwmAPin, INPUT);
  pinMode(pwmBPin, INPUT);
  delayMicroseconds(100);

  int bemfA = analogRead(bemfAPin);
  int bemfB = analogRead(bemfBPin);
  int measured_bemf = abs(bemfA - bemfB);

  bool current_bemf_state = (measured_bemf > bemf_threshold);
  if (current_bemf_state && !last_bemf_state) {
    commutation_pulse_count++;
  }
  last_bemf_state = current_bemf_state;

  // The speed calculation will be moved to the main loop
  int measured_speed = map(measured_speed_pps, 0, 500, 0, 255);
  int error = target_speed - measured_speed;
  current_pwm = constrain(target_speed + (Kp * error), 0, max_speed);
  bemf_measured_this_cycle = true;
}

void update_status_light() {
    unsigned long current_millis = millis();
    switch (current_state) {
        case RAMP_UP:
            digitalWrite(statusLedPin, HIGH); // Solid ON during ramp up
            break;
        case RAMP_DOWN:
            // Slow blink (500ms interval)
            digitalWrite(statusLedPin, (current_millis / 500) % 2);
            break;
        case COAST_HIGH:
        case COAST_LOW:
            digitalWrite(statusLedPin, HIGH); // Solid ON during coasting
            break;
        case STOP:
            // Fast blink (100ms interval) during stop
            digitalWrite(statusLedPin, (current_millis / 100) % 2);
            break;
        default:
            digitalWrite(statusLedPin, LOW); // Off otherwise
            break;
    }
}

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
