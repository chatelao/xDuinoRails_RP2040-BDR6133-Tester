// Define the pins used for motor control
const int pwmPin = D0;       // PWM output for motor speed
const int dirPin = D1;       // Digital output for motor direction
const int bemfAPin = A0;     // ADC input for back EMF from OutA
const int bemfBPin = A1;     // ADC input for back EMF from OutB

// Motor control parameters
const int max_speed = 255;   // Maximum PWM value
const int half_speed = 127;  // Half speed PWM value
const int ramp_time = 3000;  // Ramp time in milliseconds
const int delay_time = ramp_time / half_speed;

void setup() {
  // Set the pin modes
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(bemfAPin, INPUT);
  pinMode(bemfBPin, INPUT);

  // Set PWM frequency if necessary (RP2040 default is fine)
  // analogWriteFreq(1000);
}

void loop() {
  // Ramp up in forward direction
  digitalWrite(dirPin, HIGH); // Set direction to forward
  for (int speed = 0; speed <= half_speed; speed++) {
    analogWrite(pwmPin, speed);
    delay(delay_time);
  }

  // Ramp down in forward direction
  for (int speed = half_speed; speed >= 0; speed--) {
    analogWrite(pwmPin, speed);
    delay(delay_time);
  }

  delay(1000); // Wait for 1 second

  // Ramp up in reverse direction
  digitalWrite(dirPin, LOW); // Set direction to reverse
  for (int speed = 0; speed <= half_speed; speed++) {
    analogWrite(pwmPin, speed);
    delay(delay_time);
  }

  // Ramp down in reverse direction
  for (int speed = half_speed; speed >= 0; speed--) {
    analogWrite(pwmPin, speed);
    delay(delay_time);
  }

  delay(1000); // Wait for 1 second
}
