/**
 * @file MotorTuning.ino
 * @brief Beispiel-Sketch zur automatischen Ermittlung optimaler Motor-Regelparameter.
 *
 * Dieser Sketch führt einen mehrstufigen, automatisierten Test durch, um die besten
 * Einstellungen für den Kalman-Filter und den PI-Regler der xDuinoRails_MotorDriver-Bibliothek
 * zu finden. Der Prozess wird für die Vorwärts- und Rückwärtsrichtung getrennt durchgeführt.
 *
 * Ablauf:
 * 1. Der Motor wird einem standardisierten Bewegungsprofil ausgesetzt (Beschleunigen, Halten, Abbremsen).
 * 2. Währenddessen wird ein "Stabilitäts-Score" berechnet, der die Abweichung zwischen Soll- und Ist-Geschwindigkeit misst.
 * 3. Der Sketch testet iterativ verschiedene Kombinationen von Regelparametern.
 * 4. Die Parameter-Kombination mit dem niedrigsten Score (höchste Stabilität) wird als Optimum gespeichert.
 * 5. Am Ende werden die empfohlenen Einstellungen für beide Fahrtrichtungen auf dem Seriellen Monitor ausgegeben.
 *
 * @attention WICHTIG: Für diesen Test muss der Motor frei und ohne Last laufen können!
 *             Der gesamte Vorgang kann mehrere Minuten dauern.
 */
#include <xDuinoRails_MotorDriver.h>

//================================================================================
// PIN-DEFINITIONEN
//================================================================================
// WICHTIG: Stellen Sie sicher, dass diese Pins mit Ihrem Hardware-Aufbau übereinstimmen.
const int INA_PIN = D7;
const int INB_PIN = D8;
const int BEMFA_PIN = A3;
const int BEMFB_PIN = A2;

// Instanziieren der Motortreiber-Bibliothek
XDuinoRails_MotorDriver motor(INA_PIN, INB_PIN, BEMFA_PIN, BEMFB_PIN);

//================================================================================
// HAUPT-ZUSTANDSSTEUERUNG (TUNING-PROZESS)
//================================================================================
// Definiert die einzelnen Phasen des gesamten Tuning-Prozesses.
enum TuningState {
    SETUP,                          // Anfangsphase, wartet kurz vor dem Start.
    TUNING_KALMAN_FORWARD,          // Kalman-Filter für Vorwärtslauf tunen.
    TUNING_PI_FORWARD,              // PI-Regler für Vorwärtslauf tunen.
    TUNING_STALL_FORWARD,           // Platzhalter für Blockiererkennung (vorwärts).
    CHANGING_DIRECTION_TO_REVERSE,  // Motor anhalten und Richtung wechseln.
    TUNING_KALMAN_REVERSE,          // Kalman-Filter für Rückwärtslauf tunen.
    TUNING_PI_REVERSE,              // PI-Regler für Rückwärtslauf tunen.
    TUNING_STALL_REVERSE,           // Platzhalter für Blockiererkennung (rückwärts).
    DONE                            // Prozess abgeschlossen, Ergebnisse anzeigen.
};
TuningState currentState = SETUP;   // Aktueller Zustand im Tuning-Prozess.
unsigned long stateStartTime = 0;   // Zeitstempel für den Beginn des aktuellen Zustands.

//================================================================================
// ZUSTANDSSTEUERUNG (BEWEGUNGSPROFIL)
//================================================================================
// Definiert die Phasen des standardisierten Bewegungsprofils für einen einzelnen Testlauf.
enum ProfileState {
    PROFILE_IDLE,       // Wartet auf Start.
    PROFILE_ACCEL_LOW,  // Auf niedrige Geschwindigkeit beschleunigen.
    PROFILE_HOLD_LOW,   // Niedrige Geschwindigkeit halten.
    PROFILE_ACCEL_HIGH, // Auf hohe Geschwindigkeit beschleunigen.
    PROFILE_HOLD_HIGH,  // Hohe Geschwindigkeit halten.
    PROFILE_DECEL,      // Zum Stillstand abbremsen.
    PROFILE_COMPLETE    // Profil abgeschlossen.
};
ProfileState currentProfileState = PROFILE_IDLE; // Aktueller Zustand im Bewegungsprofil.
unsigned long profileStateStartTime = 0;         // Zeitstempel für den Beginn des Profil-Zustands.
double stabilityScore = 0;                       // Kumulierter Score für den aktuellen Testlauf.
bool profileRunning = false;                     // Flag, ob gerade ein Bewegungsprofil aktiv ist.

//================================================================================
// PARAMETER FÜR DEN TUNING-ALGORITHMUS
//================================================================================
// --- Zu testende Werte für den Kalman-Filter ---
// e_mea: "measurement uncertainty" - Wie stark rauschen die Messwerte? (höher = mehr Glättung)
const float kalman_e_mea_values[] = {10.0, 20.0, 40.0, 80.0};
const int num_kalman_e_mea_values = sizeof(kalman_e_mea_values) / sizeof(kalman_e_mea_values[0]);

// q: "process noise" - Wie stark ändert sich die Geschwindigkeit von selbst? (niedriger bei hoher Trägheit)
const float kalman_q_values[] = {0.01, 0.05, 0.1, 0.2};
const int num_kalman_q_values = sizeof(kalman_q_values) / sizeof(kalman_q_values[0]);

// --- Zu testende Werte für den PI-Regler ---
// Kp: Proportionalanteil - Wie stark wird auf die aktuelle Abweichung reagiert?
const float pi_kp_values[] = {0.5, 1.0, 2.0, 4.0};
const int num_pi_kp_values = sizeof(pi_kp_values) / sizeof(pi_kp_values[0]);
// Ki: Integralanteil - Wie stark werden vergangene Abweichungen korrigiert?
const float pi_ki_values[] = {5.0, 10.0, 20.0, 30.0};
const int num_pi_ki_values = sizeof(pi_ki_values) / sizeof(pi_ki_values[0]);

//================================================================================
// SPEICHER FÜR DIE BESTEN GEFUNDENEN PARAMETER
//================================================================================
double bestScore = -1.0; // Speichert den bisher besten (niedrigsten) Score.
// Vorwärts
float best_kalman_e_mea_forward = 0.0, best_kalman_q_forward = 0.0;
float best_pi_kp_forward = 0.0, best_pi_ki_forward = 0.0;
// Rückwärts
float best_kalman_e_mea_reverse = 0.0, best_kalman_q_reverse = 0.0;
float best_pi_kp_reverse = 0.0, best_pi_ki_reverse = 0.0;

// Indizes zur Iteration durch die Test-Parameter-Arrays.
int idx1 = 0;
int idx2 = 0;

// Funktions-Prototypen
void startMovementProfile();
bool updateMovementProfile();


/**
 * @brief Initialisierung des Programms.
 */
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Auf serielle Verbindung warten.
    }

    Serial.println("===================================");
    Serial.println("xDuinoRails: Motor Tuning Sketch");
    Serial.println("===================================");
    Serial.println("Dieser Sketch fuehrt eine automatische Tuning-Sequenz durch.");
    Serial.println("Bitte stellen Sie sicher, dass der Motor frei und ohne Last laufen kann.");
    Serial.println("Der Prozess wird mehrere Minuten dauern.");
    Serial.println();

    motor.begin(); // Motortreiber initialisieren.
    stateStartTime = millis();
}

/**
 * @brief Hauptschleife des Programms.
 */
void loop() {
    // Wenn ein Bewegungsprofil aktiv ist, hat dessen Aktualisierung Vorrang.
    // Die Haupt-Zustandsmaschine pausiert, bis das Profil abgeschlossen ist.
    if (profileRunning) {
        updateMovementProfile();
        return;
    }

    // Die update()-Methode der Bibliothek muss in jeder Schleife aufgerufen werden.
    motor.update();

    unsigned long timeInState = millis() - stateStartTime;

    // Haupt-Zustandsmaschine für den gesamten Tuning-Prozess
    switch (currentState) {
        case SETUP:
            // Kurze Wartezeit vor Beginn, damit der Benutzer den Seriellen Monitor öffnen kann.
            if (timeInState > 5000) {
                currentState = TUNING_KALMAN_FORWARD;
                stateStartTime = millis();
                Serial.println("Beginne Tuning fuer Kalman-Filter (Vorwaerts)...");
                // Ersten Testlauf starten.
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_KALMAN_FORWARD:
            // Dieser Zustand wird nach Abschluss eines Bewegungsprofils erreicht.
            // Bewerte den Score des abgeschlossenen Laufs.
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_kalman_e_mea_forward = kalman_e_mea_values[idx1];
                best_kalman_q_forward = kalman_q_values[idx2];
            }
            Serial.print("  Test abgeschlossen. Score: ");
            Serial.println(stabilityScore);

            // Gehe zur nächsten Parameter-Kombination.
            idx2++;
            if (idx2 >= num_kalman_q_values) {
                idx2 = 0;
                idx1++;
            }

            // Prüfen, ob alle Kombinationen getestet wurden.
            if (idx1 >= num_kalman_e_mea_values) {
                // Tuning für diese Phase ist abgeschlossen.
                Serial.println("Kalman-Tuning (Vorwaerts) abgeschlossen.");
                Serial.print("Beste Parameter: e_mea = ");
                Serial.print(best_kalman_e_mea_forward);
                Serial.print(", q = ");
                Serial.println(best_kalman_q_forward);

                // Beste gefundene Einstellung für die nächste Phase übernehmen.
                motor.setKalmanGains(best_kalman_e_mea_forward, best_kalman_q_forward);

                // Zur nächsten Tuning-Phase wechseln.
                currentState = TUNING_PI_FORWARD;
                stateStartTime = millis();

                // Variablen für die nächste Phase zurücksetzen.
                bestScore = -1.0;
                idx1 = 0;
                idx2 = 0;

                Serial.println("\nBeginne Tuning fuer PI-Regler (Vorwaerts)...");
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            } else {
                // Nächsten Testlauf starten.
                Serial.print("Teste e_mea = ");
                Serial.print(kalman_e_mea_values[idx1]);
                Serial.print(", q = ");
                Serial.println(kalman_q_values[idx2]);
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_PI_FORWARD:
            // Score des PI-Regler-Tests auswerten.
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_pi_kp_forward = pi_kp_values[idx1];
                best_pi_ki_forward = pi_ki_values[idx2];
            }
            Serial.print("  Test abgeschlossen. Score: ");
            Serial.println(stabilityScore);

            // Nächste PI-Parameter-Kombination.
            idx2++;
            if (idx2 >= num_pi_ki_values) {
                idx2 = 0;
                idx1++;
            }

            if (idx1 >= num_pi_kp_values) {
                // PI-Tuning (vorwärts) abgeschlossen.
                Serial.println("PI-Tuning (Vorwaerts) abgeschlossen.");
                Serial.print("Beste Parameter: Kp = ");
                Serial.print(best_pi_kp_forward);
                Serial.print(", Ki = ");
                Serial.println(best_pi_ki_forward);
                motor.setGains(best_pi_kp_forward, best_pi_ki_forward);

                // Nächste Phase: Richtungswechsel.
                currentState = CHANGING_DIRECTION_TO_REVERSE;
                stateStartTime = millis();
            } else {
                // Nächsten PI-Testlauf starten.
                Serial.print("Teste Kp = ");
                Serial.print(pi_kp_values[idx1]);
                Serial.print(", Ki = ");
                Serial.println(pi_ki_values[idx2]);
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_STALL_FORWARD:
            // Blockiererkennung wird in dieser Version nicht automatisch justiert.
            currentState = CHANGING_DIRECTION_TO_REVERSE;
            break;

        case CHANGING_DIRECTION_TO_REVERSE:
            Serial.println("\nWechsle auf RUECKWAERTSLAUF...");
            motor.setTargetSpeed(0, 500); // Motor sanft stoppen.
            // Warten bis Motor steht, dann Richtung wechseln.
            if (motor.getCurrentSpeed() == 0 && timeInState > 1000) {
                motor.changeDirection();
                currentState = TUNING_KALMAN_REVERSE;
                stateStartTime = millis();

                Serial.println("\nBeginne Tuning fuer Kalman-Filter (Rueckwaerts)...");
                // Variablen für den Rückwärtslauf zurücksetzen.
                idx1 = 0;
                idx2 = 0;
                bestScore = -1.0;
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_KALMAN_REVERSE:
            // Score des Kalman-Tests (rückwärts) auswerten.
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_kalman_e_mea_reverse = kalman_e_mea_values[idx1];
                best_kalman_q_reverse = kalman_q_values[idx2];
            }
            Serial.print("  Test abgeschlossen. Score: ");
            Serial.println(stabilityScore);

            // Nächste Parameter-Kombination.
            idx2++;
            if (idx2 >= num_kalman_q_values) {
                idx2 = 0;
                idx1++;
            }

            if (idx1 >= num_kalman_e_mea_values) {
                // Kalman-Tuning (rückwärts) abgeschlossen.
                Serial.println("Kalman-Tuning (Rueckwaerts) abgeschlossen.");
                Serial.print("Beste Parameter: e_mea = ");
                Serial.print(best_kalman_e_mea_reverse);
                Serial.print(", q = ");
                Serial.println(best_kalman_q_reverse);
                motor.setKalmanGains(best_kalman_e_mea_reverse, best_kalman_q_reverse);

                // Nächste Phase: PI-Tuning (rückwärts).
                currentState = TUNING_PI_REVERSE;
                stateStartTime = millis();
                idx1 = 0;
                idx2 = 0;
                bestScore = -1.0;
                Serial.println("\nBeginne Tuning fuer PI-Regler (Rueckwaerts)...");
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            } else {
                // Nächsten Kalman-Testlauf (rückwärts) starten.
                Serial.print("Teste e_mea = ");
                Serial.print(kalman_e_mea_values[idx1]);
                Serial.print(", q = ");
                Serial.println(kalman_q_values[idx2]);
                motor.setKalmanGains(kalman_e_mea_values[idx1], kalman_q_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_PI_REVERSE:
            // Score des PI-Regler-Tests (rückwärts) auswerten.
            if (bestScore < 0 || stabilityScore < bestScore) {
                bestScore = stabilityScore;
                best_pi_kp_reverse = pi_kp_values[idx1];
                best_pi_ki_reverse = pi_ki_values[idx2];
            }
            Serial.print("  Test abgeschlossen. Score: ");
            Serial.println(stabilityScore);

            // Nächste PI-Parameter-Kombination.
            idx2++;
            if (idx2 >= num_pi_ki_values) {
                idx2 = 0;
                idx1++;
            }

            if (idx1 >= num_pi_kp_values) {
                // PI-Tuning (rückwärts) abgeschlossen.
                Serial.println("PI-Tuning (Rueckwaerts) abgeschlossen.");
                Serial.print("Beste Parameter: Kp = ");
                Serial.print(best_pi_kp_reverse);
                Serial.print(", Ki = ");
                Serial.println(best_pi_ki_reverse);
                motor.setGains(best_pi_kp_reverse, best_pi_ki_reverse);

                // Prozess ist fertig.
                currentState = DONE;
                stateStartTime = millis();
            } else {
                // Nächsten PI-Testlauf (rückwärts) starten.
                Serial.print("Teste Kp = ");
                Serial.print(pi_kp_values[idx1]);
                Serial.print(", Ki = ");
                Serial.println(pi_ki_values[idx2]);
                motor.setGains(pi_kp_values[idx1], pi_ki_values[idx2]);
                startMovementProfile();
            }
            break;

        case TUNING_STALL_REVERSE:
            // Blockiererkennung wird in dieser Version nicht automatisch justiert.
            currentState = DONE;
            break;

        case DONE:
            // Endergebnis ausgeben.
            Serial.println("\n===================================");
            Serial.println("Tuning Abgeschlossen!");
            Serial.println("Empfohlene Einstellungen:");
            Serial.println("===================================");

            Serial.println("\n--- VORWAERTSLAUF ---");
            Serial.print("Kalman-Filter: e_mea = ");
            Serial.print(best_kalman_e_mea_forward);
            Serial.print(", q = ");
            Serial.println(best_kalman_q_forward);
            Serial.print("PI-Regler: Kp = ");
            Serial.print(best_pi_kp_forward);
            Serial.print(", Ki = ");
            Serial.println(best_pi_ki_forward);

            Serial.println("\n--- RUECKWAERTSLAUF ---");
            Serial.print("Kalman-Filter: e_mea = ");
            Serial.print(best_kalman_e_mea_reverse);
            Serial.print(", q = ");
            Serial.println(best_kalman_q_reverse);
            Serial.print("PI-Regler: Kp = ");
            Serial.print(best_pi_kp_reverse);
            Serial.print(", Ki = ");
            Serial.println(best_pi_ki_reverse);

            Serial.println("\nUm diese Einstellungen zu verwenden, uebertragen Sie sie in Ihren Haupt-Sketch.");

            // Programm anhalten.
            while (true) {
                delay(100);
            }
            break;
    }
}

/**
 * @brief Startet das standardisierte Bewegungsprofil für einen einzelnen Testlauf.
 */
void startMovementProfile() {
    stabilityScore = 0; // Score für diesen Lauf zurücksetzen.
    currentProfileState = PROFILE_ACCEL_LOW;
    profileStateStartTime = millis();
    motor.setTargetSpeed(80, 2000); // Sanfter Anstieg auf ca. 30% Geschwindigkeit.
    profileRunning = true;
    Serial.print("  Fahre Bewegungsprofil... ");
}

/**
 * @brief Steuert den Ablauf des Bewegungsprofils und berechnet den Score.
 * @details Diese Funktion wird wiederholt von `loop()` aufgerufen, solange `profileRunning` true ist.
 * @return true, wenn das Profil noch läuft, false, wenn es abgeschlossen ist.
 */
bool updateMovementProfile() {
    if (currentProfileState == PROFILE_IDLE || !profileRunning) {
        return false;
    }

    motor.update();

    // Der Stabilitäts-Score ist die Summe der absoluten Abweichungen zwischen Soll- und Ist-Geschwindigkeit.
    // Ein kleinerer Wert bedeutet, dass der Motor dem Geschwindigkeitsprofil genauer folgt.
    stabilityScore += abs(motor.getTargetSpeed() - motor.getCurrentSpeed());

    unsigned long timeInProfileState = millis() - profileStateStartTime;

    switch (currentProfileState) {
        case PROFILE_ACCEL_LOW: // Beschleunigen auf niedrige Geschwindigkeit.
            if (motor.getCurrentSpeed() >= 75) {
                currentProfileState = PROFILE_HOLD_LOW;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_HOLD_LOW: // Niedrige Geschwindigkeit für 2 Sekunden halten.
            if (timeInProfileState >= 2000) {
                currentProfileState = PROFILE_ACCEL_HIGH;
                profileStateStartTime = millis();
                motor.setTargetSpeed(200, 2000); // Auf ca. 80% Geschwindigkeit beschleunigen.
            }
            break;

        case PROFILE_ACCEL_HIGH: // Beschleunigen auf hohe Geschwindigkeit.
            if (motor.getCurrentSpeed() >= 195) {
                currentProfileState = PROFILE_HOLD_HIGH;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_HOLD_HIGH: // Hohe Geschwindigkeit für 3 Sekunden halten.
            if (timeInProfileState >= 3000) {
                currentProfileState = PROFILE_DECEL;
                profileStateStartTime = millis();
                motor.setTargetSpeed(0, 2000); // Sanft zum Stillstand abbremsen.
            }
            break;

        case PROFILE_DECEL: // Abbremsen zum Stillstand.
            // Warten, bis die Rampe beendet ist und der Motor steht.
            if (motor.getCurrentSpeed() == 0 && timeInProfileState > 2100) {
                currentProfileState = PROFILE_COMPLETE;
                profileStateStartTime = millis();
            }
            break;

        case PROFILE_COMPLETE: // Profil ist fertig.
            profileRunning = false;
            Serial.println("fertig.");
            return false;

        case PROFILE_IDLE:
             // Sollte nicht eintreten, während das Profil läuft.
            break;
    }

    return true; // Profil läuft noch.
}
