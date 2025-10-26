// Define the pins used for motor control
const int pwmPin = D0;       // PWM output for motor speed
const int dirPin = D1;       // Digital output for motor direction
const int bemfAPin = A0;     // ADC input for back EMF from OutA
const int bemfBPin = A1;     // ADC input for back EMF from OutB

// Motor control parameters
const int max_speed = 255;   // Maximum PWM value
const int half_speed = 127;  // Half speed PWM value
const int ramp_time = 3000;  // Ramp time in milliseconds

// Proportional control gain
const float Kp = 0.5;

// Global variable for motor state
int target_speed = 0;

void setup() {
  // Set the pin modes
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(bemfAPin, INPUT);
  pinMode(bemfBPin, INPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Ramp up in forward direction
  digitalWrite(dirPin, HIGH); // Set direction to forward
  ramp_speed(0, half_speed);

  // Ramp down in forward direction
  ramp_speed(half_speed, 0);

  delay(1000); // Wait for 1 second

  // Ramp up in reverse direction
  digitalWrite(dirPin, LOW); // Set direction to reverse
  ramp_speed(0, half_speed);

  // Ramp down in reverse direction
  ramp_speed(half_speed, 0);

  delay(1000); // Wait for 1 second
}

void ramp_speed(int start_target, int end_target) {
  int step = (start_target < end_target) ? 1 : -1;
  int delay_time = ramp_time / abs(end_target - start_target);

  for (target_speed = start_target; target_speed != end_target; target_speed += step) {
    adjust_motor_speed();
    delay(delay_time);
  }
}

void adjust_motor_speed() {
  // Read back EMF - use the higher of the two readings
  int bemfA = analogRead(bemfAPin);
  int bemfB = analogRead(bemfBPin);
  int measured_bemf = max(bemfA, bemfB);

  // Map the 10-bit ADC reading (0-1023) to the 8-bit PWM range (0-255)
  // This mapping may need calibration based on the actual BEMF voltage range.
  int measured_speed = map(measured_bemf, 0, 1023, 0, 255);

  // Calculate the error
  int error = target_speed - measured_speed;

  // Proportional control: the output is proportional to the error, adjusted
  // from a baseline (the target speed itself is a good baseline).
  int new_pwm = target_speed + (Kp * error);

  // Constrain the PWM value to be within the valid range (0-255)
  new_pwm = constrain(new_pwm, 0, max_speed);

  // Apply the new PWM value to the motor
  analogWrite(pwmPin, new_pwm);

  // Print debugging information
  Serial.print("Target: ");
  Serial.print(target_speed);
  Serial.print(" | Measured: ");
  Serial.print(measured_speed);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | PWM: ");
  Serial.println(new_pwm);
}
