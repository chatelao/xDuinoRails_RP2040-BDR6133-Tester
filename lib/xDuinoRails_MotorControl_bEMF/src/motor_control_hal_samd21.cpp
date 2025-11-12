/**
 * @file motor_control_hal_samd21.cpp
 * @brief SAMD21-specific implementation for the motor control HAL.
 *
 * This file provides the concrete implementation of the functions defined in
 * motor_control_hal.h for the SAMD21 microcontroller. It uses the SAMD21's
 * hardware peripherals (TCC, TC, EVSYS, ADC) for efficient, non-blocking motor
 * control and BEMF measurement.
 */

#if defined(USE_SAMD21_LOWLEVEL) && defined(ARDUINO_ARCH_SAMD)

#include "motor_control_hal.h"
#include <Arduino.h>

//== Hardware Timer & BEMF Measurement Parameters ==
const uint PWM_FREQUENCY_HZ = 25000;
const uint BEMF_MEASUREMENT_DELAY_US = 10;

//== Static Globals for Hardware Control ==
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;

// Globals for the ADC ISR state machine
static volatile int bemf_a_reading = -1;
static volatile bool next_adc_is_a = true;

/**
 * @brief ADC Interrupt Service Routine.
 *
 * This ISR is triggered when an ADC conversion, started by the event system,
 * is complete. It implements a simple state machine to alternate sampling
 * between the two BEMF pins.
 */
void ADC_Handler() {
    // Check if the Result Ready interrupt flag is set
    if (ADC->INTFLAG.bit.RESRDY) {
        if (next_adc_is_a) {
            // Store the reading for pin A
            bemf_a_reading = ADC->RESULT.reg;
            // Configure the ADC's positive input to be pin B for the next conversion
            ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[g_bemf_b_pin].ulADCChannelNumber;
            next_adc_is_a = false;
        } else {
            // We just finished reading pin B, so now we have a complete pair.
            int bemf_b_reading = ADC->RESULT.reg;
            // Configure the ADC's positive input back to pin A for the next cycle
            ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[g_bemf_a_pin].ulADCChannelNumber;
            next_adc_is_a = true;

            // If we have a valid reading from pin A, calculate the differential BEMF
            if (bemf_a_reading != -1) {
                int measured_bemf = abs(bemf_a_reading - bemf_b_reading);
                if (bemf_callback) {
                    bemf_callback(measured_bemf);
                }
                // Invalidate the pin A reading until the next full cycle
                bemf_a_reading = -1;
            }
        }
        // Clear the interrupt flag to re-arm it for the next event.
        ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    }
}

/**
 * @brief Initializes the hardware for motor control.
 *
 * This function performs a significant amount of low-level register configuration
 * to achieve a CPU-free, hardware-timed control loop:
 * 1. Configures Generic Clocks (GCLK) to provide a 48MHz clock to the peripherals.
 * 2. Configures TCC0 to generate a 25kHz center-aligned PWM signal.
 * 3. Configures TC3 to act as a one-shot timer for the 10µs BEMF delay.
 * 4. Configures the Event System (EVSYS) to create a chain of triggers:
 *    - TCC0 overflow (end of PWM cycle) -> starts TC3
 *    - TC3 overflow (after 10µs) -> starts ADC conversion
 * 5. Configures the ADC to be triggered by the event system and fire an interrupt on completion.
 */
void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    bemf_callback = callback;

    // --- Pin Muxing for TCC0 ---
    // This is the low-level equivalent of pinPeripheral(pin, PIO_TIMER)
    PORT->Group[g_APinDescription[g_pwm_a_pin].ulPort].PINCFG[g_APinDescription[g_pwm_a_pin].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[g_pwm_a_pin].ulPort].PMUX[g_APinDescription[g_pwm_a_pin].ulPin >> 1].reg |= (g_APinDescription[g_pwm_a_pin].ulPin & 1) ? PORT_PMUX_PMUXO(PIO_TIMER) : PORT_PMUX_PMUXE(PIO_TIMER);
    PORT->Group[g_APinDescription[g_pwm_b_pin].ulPort].PINCFG[g_APinDescription[g_pwm_b_pin].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[g_pwm_b_pin].ulPort].PMUX[g_APinDescription[g_pwm_b_pin].ulPin >> 1].reg |= (g_APinDescription[g_pwm_b_pin].ulPin & 1) ? PORT_PMUX_PMUXO(PIO_TIMER) : PORT_PMUX_PMUXE(PIO_TIMER);

    // --- GCLK Configuration (source a 48MHz clock for peripherals) ---
    // 1. Configure GCLK Generator 4 to use the 48MHz DFLL as a source, with no division.
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
    while (GCLK->STATUS.bit.SYNCBUSY);
    // 2. Route GCLK4 to TCC0/TCC1 and TC2/TC3
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // --- TCC0 for PWM Generation ---
    TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE; // Disable TCC0 to configure it
    while (TCC0->SYNCBUSY.bit.ENABLE);
    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1; // No prescaler, run at 48MHz
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM; // Normal PWM mode
    while (TCC0->SYNCBUSY.bit.WAVE);
    uint32_t period = 48000000 / PWM_FREQUENCY_HZ - 1;
    TCC0->PER.reg = period; // Set PWM period (wrap value)
    while (TCC0->SYNCBUSY.bit.PER);
    TCC0->EVCTRL.reg = TCC_EVCTRL_OVFEO;   // Enable overflow event output
    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;  // Enable TCC0
    while (TCC0->SYNCBUSY.bit.ENABLE);

    // --- TC3 for 10us BEMF Measurement Delay ---
    TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; // Disable TC3 to configure it
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
    // Configure in 16-bit mode with a 16x prescaler (48MHz/16 = 3MHz clock)
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;
    // Configure TC3 to be triggered by an event, and invert the event trigger polarity
    TC3->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI | TC_EVCTRL_OVFEO;
    // Set compare value for 10us delay (30 cycles @ 3MHz = 10us)
    TC3->COUNT16.CC[0].reg = 30;
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
    // Enable one-shot mode
    TC3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);


    // --- Event System ---
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
    // Channel 0: TCC0 Overflow -> TC3 Start
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);
    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC0_OVF) | EVSYS_CHANNEL_CHANNEL(0);
    // Channel 1: TC3 Overflow -> ADC Start Conversion
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(2) | EVSYS_USER_USER(EVSYS_ID_USER_ADC_START);
    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC3_OVF) | EVSYS_CHANNEL_CHANNEL(1);

    // --- ADC ---
    ADC->CTRLA.reg = 0; // Disable ADC to configure it
    while(ADC->STATUS.bit.SYNCBUSY);
    ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1; // Use VDDANA as reference
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;    // No averaging
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0);   // 0 extra sample time
    // Set prescaler and 12-bit resolution
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 | ADC_CTRLB_RESSEL_12BIT;
    // Set the initial input to the first BEMF pin
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[g_bemf_a_pin].ulADCChannelNumber;
    ADC->EVCTRL.bit.STARTEI = 1;      // Enable event-triggered conversion
    ADC->INTENSET.bit.RESRDY = 1;     // Enable result ready interrupt
    ADC->CTRLA.reg = ADC_CTRLA_ENABLE; // Enable ADC
    while(ADC->STATUS.bit.SYNCBUSY);

    // Enable the ADC interrupt in the Nested Vector Interrupt Controller (NVIC)
    NVIC_EnableIRQ(ADC_IRQn);
    // Enable TC3 (must be done last, after event setup)
    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

/**
 * @brief Sets the PWM duty cycle and direction for the motor.
 *
 * @param duty_cycle The PWM duty cycle (0-255).
 * @param forward The direction of rotation.
 */
void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // Map the 8-bit duty cycle to the PWM counter's range
    uint32_t level = map(duty_cycle, 0, 255, 0, TCC0->PER.reg);

    // TCC0 has multiple compare channels (CCx) which map to different output
    // pins (WO[x]). We drive one high and the other low depending on direction.
    // The specific mapping of CC channel to WO pin is hardware-dependent.
    // For QT Py M0: A0 (PA02) is TCC0/WO[0], A1 (PA03) is TCC0/WO[1].
    // So we use CC[0] for PWM A and CC[1] for PWM B.
    if (forward) {
        TCC0->CC[0].reg = level;
        while(TCC0->SYNCBUSY.bit.CC0);
        TCC0->CC[1].reg = 0;
        while(TCC0->SYNCBUSY.bit.CC1);
    } else {
        TCC0->CC[0].reg = 0;
        while(TCC0->SYNCBUSY.bit.CC0);
        TCC0->CC[1].reg = level;
        while(TCC0->SYNCBUSY.bit.CC1);
    }
}

#endif // defined(USE_SAMD21_LOWLEVEL) && defined(ARDUINO_ARCH_SAMD)
