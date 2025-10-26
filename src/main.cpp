#include <Arduino.h>

// Corrected Pin Assignments
const int pwmAPin = D2;      // PWM Forward (GPIO28)
const int pwmBPin = D3;      // PWM Reverse (GPIO29)
const int bemfAPin = A0;     // BEMF A (GPIO26)
const int bemfBPin = A1;     // BEMF B (GPIO27)

// Motor control parameters
const int max_speed = 255;
const int half_speed = 127;
const int ramp_duration_ms = 3000;

// Proportional control gain
const float Kp = 0.5;

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
  Serial.begin(9600);
}

void loop() {
  unsigned long current_millis = millis();
  unsigned long current_micros = micros();

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
    pinMode(pwmAPin, INPUT);
    pinMode(pwmBPin, INPUT);
    bemf_measured_this_cycle = true;
    delayMicroseconds(50);

    // Corrected BEMF measurement: read the terminal that was NOT connected to ground.
    int measured_bemf = forward ? analogRead(bemfAPin) : analogRead(bemfBPin);
    int measured_speed = map(measured_bemf, 0, 1023, 0, 255);
    int error = target_speed - measured_speed;
    current_pwm = constrain(target_speed + (Kp * error), 0, max_speed);
  }
}
