#include <Arduino.h>
#include <XDuinoRails.h>

// Pin Definitions
const int pwmAPin = D7;
const int pwmBPin = D8;
const int bemfAPin = A3;
const int bemfBPin = A2;

// Approximation of the maximum speed in pulses per second (PPS) for a target PWM of 255.
// This value is motor-specific and may need adjustment for different hardware.
const float MAX_SPEED_PPS_ESTIMATE = 500.0;

XDuinoRails_MotorDriver motor(pwmAPin, pwmBPin, bemfAPin, bemfBPin);

// Define the search space for the parameters
const float kp_values[] = {0.1, 0.3, 0.6};
const float ki_values[] = {0.05, 0.1, 0.2};
const float ema_alpha_values[] = {0.1, 0.25, 0.4};
const float mea_e_values[] = {1.0, 3.0, 5.0};
const float est_e_values[] = {1.0, 3.0, 5.0};
const float q_values[] = {0.01, 0.05};

// Data structure to hold test results
struct TestResult {
    float kp;
    float ki;
    float ema_alpha;
    float mea_e;
    float est_e;
    float q;
    float score;
    int min_startup_pwm;
};

TestResult best_forward_result = {0, 0, 0, 0, 0, 0, 99999.0, 0};
TestResult best_reverse_result = {0, 0, 0, 0, 0, 0, 99999.0, 0};

// Function Prototypes
float run_test_profile(bool forward);
int find_min_startup_pwm(bool forward);
void run_tuning_for_direction(bool is_forward, TestResult& best_result);
void print_results();


void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    motor.begin();
    Serial.println("Starting motor parameter auto-tuning process...");
    Serial.println("This may take several minutes...");
}

void loop() {
    run_tuning_for_direction(true, best_forward_result);
    run_tuning_for_direction(false, best_reverse_result);

    print_results();

    // Stop the motor and do nothing else
    motor.setTargetSpeed(0);
    while(true) {
        motor.update();
        delay(10);
    }
}

void run_tuning_for_direction(bool is_forward, TestResult& best_result) {
    const char* direction_str = is_forward ? "Forward" : "Reverse";
    Serial.printf("\n--- Starting %s Tuning ---\n", direction_str);

    best_result.min_startup_pwm = find_min_startup_pwm(is_forward);
    Serial.printf("%s minimum startup PWM: %d\n", direction_str, best_result.min_startup_pwm);

    for (float kp : kp_values) {
        for (float ki : ki_values) {
            for (float ema : ema_alpha_values) {
                for (float mea_e : mea_e_values) {
                    for (float est_e : est_e_values) {
                        for (float q : q_values) {
                            motor.setPIgains(kp, ki);
                            motor.setFilterParameters(ema, mea_e, est_e, q);
                            float score = run_test_profile(is_forward);
                            Serial.printf("  Score: %.2f (Kp=%.2f, Ki=%.2f, EMA=%.2f, MEA_E=%.2f, EST_E=%.2f, Q=%.2f)\n",
                                          score, kp, ki, ema, mea_e, est_e, q);

                            if (score < best_result.score) {
                                best_result.score = score;
                                best_result.kp = kp;
                                best_result.ki = ki;
                                best_result.ema_alpha = ema;
                                best_result.mea_e = mea_e;
                                best_result.est_e = est_e;
                                best_result.q = q;
                            }
                        }
                    }
                }
            }
        }
    }
}


float run_test_at_speed(int target_speed) {
    const int settle_time_ms = 1000;
    const int measure_duration_ms = 2000;
    const int sample_interval_ms = 10;
    const int num_samples = measure_duration_ms / sample_interval_ms;

    float speed_samples[num_samples];
    float sum_sq_diff = 0.0;
    float total_overshoot = 0.0;
    long settle_end_time = 0;

    motor.setTargetSpeed(target_speed);
    motor.resetPIController();
    long start_time = millis();

    // Settle time
    while(millis() - start_time < settle_time_ms) {
        motor.update();
        delay(5);
    }

    // Measurement period
    long measure_start_time = millis();
    float sum_speed = 0;
    float max_speed = 0;
    for (int i = 0; i < num_samples; i++) {
        motor.update();
        speed_samples[i] = motor.getMeasuredSpeedPPS();
        sum_speed += speed_samples[i];
        if (speed_samples[i] > max_speed) {
            max_speed = speed_samples[i];
        }
        delay(sample_interval_ms);
    }
    float average_speed = sum_speed / num_samples;

    // Calculate standard deviation (stability)
    for (int i = 0; i < num_samples; i++) {
        sum_sq_diff += pow(speed_samples[i] - average_speed, 2);
    }
    float std_dev = sqrt(sum_sq_diff / num_samples);

    // Calculate overshoot
    float pps_target = map(target_speed, 0, 255, 0, MAX_SPEED_PPS_ESTIMATE);
    float overshoot = (max_speed > pps_target) ? (max_speed - pps_target) : 0.0;

    // Simple scoring: weighted sum of fluctuation and overshoot. Lower is better.
    return (std_dev * 1.0) + (overshoot * 0.5);
}

float run_test_profile(bool forward) {
    motor.setDirection(forward);

    float score_low = run_test_at_speed(50); // Low speed
    float score_med = run_test_at_speed(150); // Medium speed
    float score_high = run_test_at_speed(250); // High speed

    motor.setTargetSpeed(0);
    motor.resetPIController();
    delay(500); // Let motor stop

    // Combine scores (e.g., average, or weighted average)
    return (score_low + score_med + score_high) / 3.0;
}

int find_min_startup_pwm(bool forward) {
    const int pwm_step = 1;
    const int settle_time_ms = 200;
    const float speed_threshold_pps = 5.0;

    motor.setDirection(forward);
    motor.enablePIController(false); // Use direct PWM control

    for (int pwm = 0; pwm <= 255; pwm += pwm_step) {
        motor.setCurrentPWM(pwm);
        long start_time = millis();
        bool motor_moved = false;
        while (millis() - start_time < settle_time_ms) {
            motor.update();
            if (motor.getMeasuredSpeedPPS() > speed_threshold_pps) {
                motor_moved = true;
                break;
            }
            delay(10);
        }
        if (motor_moved) {
            motor.setCurrentPWM(0);
            motor.enablePIController(true);
            delay(500);
            return pwm;
        }
    }

    motor.setCurrentPWM(0);
    motor.enablePIController(true);
    return 255; // Return max if it never moved
}

void print_results() {
    Serial.println("\n--- Auto-tuning Complete ---");
    Serial.println("\n--- Best Forward Parameters ---");
    Serial.printf("Score: %.2f\n", best_forward_result.score);
    Serial.printf("Min Startup PWM: %d\n", best_forward_result.min_startup_pwm);
    Serial.printf("Kp: %.2f\n", best_forward_result.kp);
    Serial.printf("Ki: %.2f\n", best_forward_result.ki);
    Serial.printf("EMA Alpha: %.2f\n", best_forward_result.ema_alpha);
    Serial.printf("Kalman MEA_E: %.2f\n", best_forward_result.mea_e);
    Serial.printf("Kalman EST_E: %.2f\n", best_forward_result.est_e);
    Serial.printf("Kalman Q: %.2f\n", best_forward_result.q);

    Serial.println("\n--- Best Reverse Parameters ---");
    Serial.printf("Score: %.2f\n", best_reverse_result.score);
    Serial.printf("Min Startup PWM: %d\n", best_reverse_result.min_startup_pwm);
    Serial.printf("Kp: %.2f\n", best_reverse_result.kp);
    Serial.printf("Ki: %.2f\n", best_reverse_result.ki);
    Serial.printf("EMA Alpha: %.2f\n", best_reverse_result.ema_alpha);
    Serial.printf("Kalman MEA_E: %.2f\n", best_reverse_result.mea_e);
    Serial.printf("Kalman EST_E: %.2f\n", best_reverse_result.est_e);
    Serial.printf("Kalman Q: %.2f\n", best_reverse_result.q);
}
