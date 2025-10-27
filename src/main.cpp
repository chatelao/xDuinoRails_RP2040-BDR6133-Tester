#include <Arduino.h>

// Corrected Pin Assignments
const int pwmAPin = D2;      // PWM Forward (GPIO28)
const int pwmBPin = D3;      // PWM Reverse (GPIO29)
const int bemfAPin = A0;     // BEMF A (GPIO26)
const int bemfBPin = A1;     // BEMF B (GPIO27)
const int statusLedPin = LED_BUILTIN; // Status LED

// Motor control parameters
const int max_speed = 255;
const int half_speed = 127;
const int ramp_duration_ms = 3000;

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

// Non-blocking timers and state variables
unsigned long last_ramp_update_ms = 0;
unsigned long last_pwm_cycle_us = 0;
unsigned long direction_change_start_ms = 0;
int ramp_step_delay_ms = ramp_duration_ms / half_speed;

int target_speed = 0;
int current_pwm = 0;
bool forward = true;
bool ramping_up = true;
bool in_direction_change_delay = false;
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

  if (in_direction_change_delay) {
    // Fast blink (100ms interval)
    digitalWrite(statusLedPin, (current_millis / 100) % 2);
  } else if (!ramping_up && target_speed > 0) {
    // Slow blink (500ms interval)
    digitalWrite(statusLedPin, (current_millis / 500) % 2);
  } else if (target_speed > 0) {
    // Solid ON
    digitalWrite(statusLedPin, HIGH);
  } else {
    // Solid OFF
    digitalWrite(statusLedPin, LOW);
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

  // --- High-level state machine for ramping and direction changes ---
  if (in_direction_change_delay) {
    if (current_millis - direction_change_start_ms >= 1000) {
      in_direction_change_delay = false;
      ramping_up = true;
      forward = !forward;
    }
    return;
  }

  if (current_millis - last_ramp_update_ms >= ramp_step_delay_ms) {
    last_ramp_update_ms = current_millis;

    if (ramping_up) {
      if (target_speed < half_speed) {
        target_speed++;
      } else {
        ramping_up = false;
      }
    } else {
      if (target_speed > 0) {
        target_speed--;
      } else {
        in_direction_change_delay = true;
        direction_change_start_ms = current_millis;
      }
    }
  }

  // --- Low-level non-blocking software PWM and BEMF measurement ---
  long on_time_us = map(current_pwm, 0, 255, 0, pwm_period_us);

  if (current_micros - last_pwm_cycle_us >= pwm_period_us) {
    last_pwm_cycle_us = current_micros;
    bemf_measured_this_cycle = false;

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

  if (!bemf_measured_this_cycle && (current_micros - last_pwm_cycle_us > on_time_us)) {
    update_motor_pwm();
  }
}
