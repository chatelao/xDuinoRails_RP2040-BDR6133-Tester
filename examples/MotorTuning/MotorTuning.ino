#include <xDuinoRails_MotorDriver.h>

//== Pin Definitions ==
// IMPORTANT: Make sure these pins match your hardware setup.
const int INA_PIN = D7;
const int INB_PIN = D8;
const int BEMFA_PIN = A3;
const int BEMFB_PIN = A2;

// Instantiate the motor driver library
XDuinoRails_MotorDriver motor(INA_PIN, INB_PIN, BEMFA_PIN, BEMFB_PIN);

// State machine for the tuning process
enum TuningState {
    SETUP,
    TUNING_KALMAN_FORWARD,
    TUNING_PI_FORWARD,
    TUNING_STALL_FORWARD,
    CHANGING_DIRECTION_TO_REVERSE,
    TUNING_KALMAN_REVERSE,
    TUNING_PI_REVERSE,
    TUNING_STALL_REVERSE,
    DONE
};
TuningState currentState = SETUP;
unsigned long stateStartTime = 0;

// Variables for the movement profile and scoring
enum ProfileState {
    PROFILE_IDLE,
    PROFILE_ACCEL_LOW,
    PROFILE_HOLD_LOW,
    PROFILE_ACCEL_HIGH,
    PROFILE_HOLD_HIGH,
    PROFILE_DECEL,
    PROFILE_COMPLETE
};
ProfileState currentProfileState = PROFILE_IDLE;
unsigned long profileStateStartTime = 0;
double stabilityScore = 0;
bool profileRunning = false;

// -- Parameters for the tuning algorithm --
// Test values for Kalman filter's measurement noise
const float kalman_e_mea_values[] = {10.0, 20.0, 40.0, 80.0};
const int num_kalman_e_mea_values = sizeof(kalman_e_mea_values) / sizeof(kalman_e_mea_values[0]);

// Test values for Kalman filter's process noise
const float kalman_q_values[] = {0.01, 0.05, 0.1, 0.2};
const int num_kalman_q_values = sizeof(kalman_q_values) / sizeof(kalman_q_values[0]);

// Test values for PI controller gains
const float pi_kp_values[] = {0.5, 1.0, 2.0, 4.0};
const int num_pi_kp_values = sizeof(pi_kp_values) / sizeof(pi_kp_values[0]);
const float pi_ki_values[] = {5.0, 10.0, 20.0, 30.0};
const int num_pi_ki_values = sizeof(pi_ki_values) / sizeof(pi_ki_values[0]);


// Variables to store the best found parameters
double bestScore = -1.0;
float best_kalman_e_mea_forward = 0.0, best_kalman_q_forward = 0.0;
float best_pi_kp_forward = 0.0, best_pi_ki_forward = 0.0;
float best_kalman_e_mea_reverse = 0.0, best_kalman_q_reverse = 0.0;
float best_pi_kp_reverse = 0.0, best_pi_ki_reverse = 0.0;


// Indices for iterating through the test values
int idx1 = 0;
int idx2 = 0;


// Function prototypes
void startMovementProfile();
double runMovementProfileAndGetScore();


void setup() {
    Serial.begin(115200);
    // Wait for the serial port to be ready.
    while (!Serial) {
        delay(10);
    }

    Serial.println("===================================");
    Serial.println("xDuinoRails: Motor Tuning Sketch");
    Serial.println("===================================");
    Serial.println("This sketch will perform an automatic tuning sequence.");
    Serial.println("Please ensure the motor can run freely without load.");
    Serial.println("The process will take several minutes.");
    Serial.println();

    // Initialize the motor driver
    motor.begin();
    stateStartTime = millis();
}

void loop() {
    // If a movement profile is active, it takes priority.
    // We keep calling its update function until it's complete.
    if (profileRunning) {
        updateMovementProfile();
        return; // Don't run the main state machine until the profile is done.
    }

    // The main motor update must still be called on every loop.
    motor.update();

    unsigned long timeInState = millis() - stateStartTime;

    switch (currentState) {
        case SETUP:
            // Wait for 5 seconds before starting the tuning process
            if (timeInState > 5000) {
                currentState = TUNING_KALMAN_FORWARD;
                stateStartTime = millis();
                Serial.println("Tuning Kalman Filter (Forward)...");
                // Start the first test run
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_KALMAN_FORWARD:
            // This case is entered after a movement profile completes.
            // Check the score from the completed run.
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_kalman_e_mea_forward = kalman_e_mea_values[idx1];
                best_kalman_q_forward = kalman_q_values[idx2];
            }
            Serial.print("  Test complete. Score: ");
            Serial.println(stabilityScore);

            // Move to the next combination of parameters
            idx2++;
            if (idx2 >= num_kalman_q_values) {
                idx2 = 0;
                idx1++;
            }

            // Check if we have tested all combinations
            if (idx1 >= num_kalman_e_mea_values) {
                // Tuning for this stage is done.
                Serial.println("Kalman tuning (Forward) complete.");
                Serial.print("Best Params: e_mea = ");
                Serial.print(best_kalman_e_mea_forward);
                Serial.print(", q = ");
                Serial.println(best_kalman_q_forward);

                // Apply the best settings before moving to the next stage
                motor.setKalmanGains(best_kalman_e_mea_forward, best_kalman_q_forward);

                currentState = TUNING_PI_FORWARD;
                stateStartTime = millis();

                // Reset for the next tuning stage
                bestScore = -1.0;
                idx1 = 0;
                idx2 = 0;

                Serial.println("\nTuning PI Controller (Forward)...");
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            } else {
                // Start the next test run
                Serial.print("Testing e_mea = ");
                Serial.print(kalman_e_mea_values[idx1]);
                Serial.print(", q = ");
                Serial.println(kalman_q_values[idx2]);
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_PI_FORWARD:
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_pi_kp_forward = pi_kp_values[idx1];
                best_pi_ki_forward = pi_ki_values[idx2];
            }
             Serial.print("  Test complete. Score: ");
            Serial.println(stabilityScore);

            idx2++;
            if (idx2 >= num_pi_ki_values) {
                idx2 = 0;
                idx1++;
            }

            if (idx1 >= num_pi_kp_values) {
                Serial.println("PI tuning (Forward) complete.");
                Serial.print("Best Params: Kp = ");
                Serial.print(best_pi_kp_forward);
                Serial.print(", Ki = ");
                Serial.println(best_pi_ki_forward);
                motor.setGains(best_pi_kp_forward, best_pi_ki_forward);
                currentState = CHANGING_DIRECTION_TO_REVERSE;
                stateStartTime = millis();
            } else {
                Serial.print("Testing Kp = ");
                Serial.print(pi_kp_values[idx1]);
                Serial.print(", Ki = ");
                Serial.println(pi_ki_values[idx2]);
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_STALL_FORWARD:
            // Stall tuning is not implemented in this version
            currentState = CHANGING_DIRECTION_TO_REVERSE;
            break;

        case CHANGING_DIRECTION_TO_REVERSE:
            Serial.println("\nChanging direction to REVERSE...");
            motor.setTargetSpeed(0, 500); // Stop the motor
            if (motor.getCurrentSpeed() == 0 && timeInState > 1000) {
                motor.changeDirection();
                currentState = TUNING_KALMAN_REVERSE;
                stateStartTime = millis();
                Serial.println("\nTuning Kalman Filter (Reverse)...");
                idx1 = 0;
                idx2 = 0;
                bestScore = -1.0;
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_KALMAN_REVERSE:
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_kalman_e_mea_reverse = kalman_e_mea_values[idx1];
                best_kalman_q_reverse = kalman_q_values[idx2];
            }
            Serial.print("  Test complete. Score: ");
            Serial.println(stabilityScore);
            idx2++;
            if (idx2 >= num_kalman_q_values) {
                idx2 = 0;
                idx1++;
            }
            if (idx1 >= num_kalman_e_mea_values) {
                Serial.println("Kalman tuning (Reverse) complete.");
                Serial.print("Best Params: e_mea = ");
                Serial.print(best_kalman_e_mea_reverse);
                Serial.print(", q = ");
                Serial.println(best_kalman_q_reverse);
                motor.setKalmanGains(best_kalman_e_mea_reverse, best_kalman_q_reverse);
                currentState = TUNING_PI_REVERSE;
                stateStartTime = millis();
                idx1 = 0;
                idx2 = 0;
                bestScore = -1.0;
                Serial.println("\nTuning PI Controller (Reverse)...");
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            } else {
                Serial.print("Testing e_mea = ");
                Serial.print(kalman_e_mea_values[idx1]);
                Serial.print(", q = ");
                Serial.println(kalman_q_values[idx2]);
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_PI_REVERSE:
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_pi_kp_reverse = pi_kp_values[idx1];
                best_pi_ki_reverse = pi_ki_values[idx2];
            }
            Serial.print("  Test complete. Score: ");
            Serial.println(stabilityScore);
            idx2++;
            if (idx2 >= num_pi_ki_values) {
                idx2 = 0;
                idx1++;
            }
            if (idx1 >= num_pi_kp_values) {
                Serial.println("PI tuning (Reverse) complete.");
                Serial.print("Best Params: Kp = ");
                Serial.print(best_pi_kp_reverse);
                Serial.print(", Ki = ");
                Serial.println(best_pi_ki_reverse);
                motor.setGains(best_pi_kp_reverse, best_pi_ki_reverse);
                currentState = DONE;
                stateStartTime = millis();
            } else {
                Serial.print("Testing Kp = ");
                Serial.print(pi_kp_values[idx1]);
                Serial.print(", Ki = ");
                Serial.println(pi_ki_values[idx2]);
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_STALL_REVERSE:
            // Stall tuning is not implemented in this version
            currentState = DONE;
            break;

        case DONE:
            Serial.println("\n===================================");
            Serial.println("Tuning Complete!");
            Serial.println("Recommended Settings:");
            Serial.println("===================================");

            Serial.println("\n--- FORWARD Motion ---");
            Serial.print("Kalman Filter: e_mea = ");
            Serial.print(best_kalman_e_mea_forward);
            Serial.print(", q = ");
            Serial.println(best_kalman_q_forward);
            Serial.print("PI Controller: Kp = ");
            Serial.print(best_pi_kp_forward);
            Serial.print(", Ki = ");
            Serial.println(best_pi_ki_forward);

            Serial.println("\n--- REVERSE Motion ---");
            Serial.print("Kalman Filter: e_mea = ");
            Serial.print(best_kalman_e_mea_reverse);
            Serial.print(", q = ");
            Serial.println(best_kalman_q_reverse);
            Serial.print("PI Controller: Kp = ");
            Serial.print(best_pi_kp_reverse);
            Serial.print(", Ki = ");
            Serial.println(best_pi_ki_reverse);

            Serial.println("\nTo use these settings, update them in your main sketch.");

            // Halt execution.
            while (true) {
                delay(100);
            }
            break;
    }
}

/**
 * @brief Starts the movement profile for a tuning run.
 */
void startMovementProfile() {
    stabilityScore = 0;
    currentProfileState = PROFILE_ACCEL_LOW;
    profileStateStartTime = millis();
    motor.setTargetSpeed(80, 2000); // Ramp to 30% speed
    profileRunning = true;
    Serial.print("  Running profile... ");
}

/**
 * @brief Manages the execution of the movement profile and calculates the stability score.
 * This function must be called repeatedly until the profile is complete.
 * @return Returns true if the profile is still running, false when it's complete.
 */
bool updateMovementProfile() {
    if (currentProfileState == PROFILE_IDLE || !profileRunning) {
        return false;
    }

    // Call the main motor update function
    motor.update();

    // Calculate the absolute error between target and current speed for the score
    stabilityScore += abs(motor.getTargetSpeed() - motor.getCurrentSpeed());

    unsigned long timeInProfileState = millis() - profileStateStartTime;

    switch (currentProfileState) {
        case PROFILE_ACCEL_LOW:
            if (motor.getCurrentSpeed() >= 75) {
                currentProfileState = PROFILE_HOLD_LOW;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_HOLD_LOW:
            if (timeInProfileState >= 2000) {
                currentProfileState = PROFILE_ACCEL_HIGH;
                profileStateStartTime = millis();
                motor.setTargetSpeed(200, 2000); // Ramp to 80% speed
            }
            break;

        case PROFILE_ACCEL_HIGH:
            if (motor.getCurrentSpeed() >= 195) {
                currentProfileState = PROFILE_HOLD_HIGH;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_HOLD_HIGH:
            if (timeInProfileState >= 3000) {
                currentProfileState = PROFILE_DECEL;
                profileStateStartTime = millis();
                motor.setTargetSpeed(0, 2000); // Ramp to stop
            }
            break;

        case PROFILE_DECEL:
            if (motor.getCurrentSpeed() == 0 && timeInProfileState > 2100) { // Wait until ramp is done
                currentProfileState = PROFILE_COMPLETE;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_COMPLETE:
            profileRunning = false;
            Serial.println("done.");
            return false; // Profile finished

        case PROFILE_IDLE:
             // Should not happen while profileRunning is true
            break;
    }

    return true; // Profile still running
}
