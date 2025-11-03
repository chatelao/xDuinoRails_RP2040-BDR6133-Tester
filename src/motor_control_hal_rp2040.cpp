/**
 * @file motor_control_hal_rp2040.cpp
 * @brief RP2040-specific implementation for the motor control HAL.
 *
 * This file provides the concrete implementation of the functions defined in
 * motor_control_hal.h for the Raspberry Pi RP2040 microcontroller. It uses
 * the Pico-SDK's hardware peripherals (PWM, ADC, DMA) for efficient,
 * non-blocking motor control and BEMF measurement.
 */

#include "motor_control_hal.h"

#if defined(USE_RP2040_LOWLEVEL)

#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

//== Hardware PWM & BEMF Measurement Parameters ==
// Defines the core operational parameters for the hardware-timed control loop.
const uint PWM_FREQUENCY_HZ = 25000; // 25kHz PWM frequency for motor control.
// RP2040 sys clock is 125MHz. PWM counter wraps at this value for 25kHz.
static uint16_t PWM_WRAP_VALUE = (125000000 / PWM_FREQUENCY_HZ) - 1;
// 10us delay after PWM off-time before ADC read, allows BEMF to stabilize.
const uint BEMF_MEASUREMENT_DELAY_US = 10;
// Buffer for DMA to store ADC samples. Must be a power of 2 for the ring buffer.
const uint BEMF_RING_BUFFER_SIZE = 64;

//== Static Globals for Hardware Control ==
// Storage for hardware configuration state that is shared across HAL functions.
static uint dma_channel;
static uint motor_pwm_slice;
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
static hal_bemf_update_callback_t bemf_callback = nullptr;

// Pin definitions, to be set in hal_motor_init
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;

//== Private Interrupt Service Routines and Callbacks ==
// These functions orchestrate the precise timing of BEMF ADC measurements.

// Forward Declarations for the ISRs and timer callbacks used by the HAL.
static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();

/**
 * @brief DMA interrupt handler.
 *
 * This ISR is triggered when the DMA transfer of ADC samples to the ring
 * buffer is complete. It calculates the average differential BEMF, invokes
 * the registered callback, and then restarts the DMA transfer.
 */
static void dma_irq_handler() {
    // Manually clear the DMA interrupt flag to prevent re-triggering.
    dma_hw->ints0 = 1u << dma_channel;

    uint32_t sum_A = 0, sum_B = 0;
    // The DMA filled the buffer with alternating samples of BEMF B and BEMF A.
    // We now iterate through the buffer, summing the values for each pin.
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_B += bemf_ring_buffer[i];     // Even indices are BEMF B
        sum_A += bemf_ring_buffer[i + 1]; // Odd indices are BEMF A
    }
    // Calculate the average for each channel, then find the absolute difference.
    // This differential measurement represents the final BEMF value.
    int measured_bemf = abs((int)(sum_A / (BEMF_RING_BUFFER_SIZE / 2)) - (int)(sum_B / (BEMF_RING_BUFFER_SIZE / 2)));

    // Invoke the callback with the raw BEMF data
    if (bemf_callback) {
        bemf_callback(measured_bemf);
    }

    // Restart DMA transfer for the next cycle
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

/**
 * @brief One-shot timer callback to trigger ADC conversion.
 *
 * This is called by an alarm a short time after the PWM wrap to ensure
 * the motor windings are in a high-impedance state. It starts the ADC,
 * which will then use DMA to fill the buffer.
 */
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    adc_run(true);
    return 0; // Do not reschedule
}

/**
 * @brief PWM wrap interrupt handler.
 *
 * This ISR is triggered at the end of each PWM cycle. It sets a one-shot
 * hardware timer to trigger the ADC measurement after a short delay.
 */
static void on_pwm_wrap() {
    pwm_clear_irq(motor_pwm_slice);
    add_alarm_in_us(BEMF_MEASUREMENT_DELAY_US, delayed_adc_trigger_callback, NULL, true);
}

//== Public HAL Function Implementations ==

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    bemf_callback = callback;

    // --- ADC and DMA Setup ---
    // Configures the ADC and DMA to automatically sample BEMF pins A and B.
    adc_init();
    adc_gpio_init(bemf_b_pin); // Enable ADC functionality on the BEMF B pin.
    adc_gpio_init(bemf_a_pin); // Enable ADC functionality on the BEMF A pin.
    // Set ADC to cycle between BEMF B and A pins. (e.g., A2=ADC2, A3=ADC3)
    // The calculation `pin - 26` converts Arduino pin number (A0=26) to ADC channel.
    adc_set_round_robin((1u << (bemf_b_pin - 26)) | (1u << (bemf_a_pin - 26)));
    adc_select_input(bemf_b_pin - 26); // Start round-robin sequence with BEMF B.
    // Configure the ADC FIFO: enable, DMA-driven, one sample per DREQ, no error byte.
    adc_fifo_setup(true, true, 1, false, false);

    dma_channel = dma_claim_unused_channel(true); // Get a free DMA channel.
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    // Configure DMA: 16-bit transfers from ADC FIFO to a memory buffer.
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false); // Read from same ADC FIFO address.
    channel_config_set_write_increment(&dma_config, true);  // Write to successive buffer addresses.
    channel_config_set_dreq(&dma_config, DREQ_ADC); // Trigger DMA on ADC data ready.
    // Configure the write address to wrap around, creating a 64-entry ring buffer.
    // The '6' means the ring wraps at 2^6 = 64 entries (of 16-bits each).
    channel_config_set_ring(&dma_config, true, 6);

    dma_channel_configure(dma_channel, &dma_config, bemf_ring_buffer, &adc_hw->fifo, BEMF_RING_BUFFER_SIZE, false);
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- PWM Setup for Motor Control ---
    // Configures a PWM slice to drive both motor pins A and B.
    gpio_set_function(g_pwm_a_pin, GPIO_FUNC_PWM); // Assign PWM func to pin A.
    gpio_set_function(g_pwm_b_pin, GPIO_FUNC_PWM); // Assign PWM func to pin B.
    // Both pins must be on the same PWM slice; we find the slice from pin A.
    motor_pwm_slice = pwm_gpio_to_slice_num(g_pwm_a_pin);

    pwm_config motor_pwm_conf = pwm_get_default_config();
    // Set the PWM counter wrap value to achieve the desired PWM_FREQUENCY_HZ.
    pwm_config_set_wrap(&motor_pwm_conf, PWM_WRAP_VALUE);
    pwm_init(motor_pwm_slice, &motor_pwm_conf, true); // Load config and run.

    // --- PWM Interrupt for Synchronization ---
    // Sets up an interrupt that fires every time the PWM counter wraps (resets).
    // This interrupt is the master trigger for the BEMF measurement sequence.
    pwm_clear_irq(motor_pwm_slice);
    pwm_set_irq_enabled(motor_pwm_slice, true); // Enable the wrap interrupt.
    // Set our custom `on_pwm_wrap` function as the exclusive handler.
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true); // Enable the global PWM IRQ.

    // Start DMA, which will wait for the first ADC trigger.
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

//== Public HAL Function Implementations ==

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // Map the 8-bit duty cycle (0-255) to the PWM counter's wrap value.
    uint16_t level = map(duty_cycle, 0, 255, 0, PWM_WRAP_VALUE);

    // For H-bridge control, one pin is driven with PWM, the other is held low.
    if (forward) {
        pwm_set_gpio_level(g_pwm_a_pin, level); // Drive pin A
        pwm_set_gpio_level(g_pwm_b_pin, 0);     // Keep pin B low
    } else {
        pwm_set_gpio_level(g_pwm_a_pin, 0);     // Keep pin A low
        pwm_set_gpio_level(g_pwm_b_pin, level); // Drive pin B
    }
}

#endif // USE_RP2040_LOWLEVEL && ARDUINO_RASPBERRY_PI_PICO
