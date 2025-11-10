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

#if defined(USE_RP2040_LOWLEVEL) && defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

//== Hardware PWM & BEMF Measurement Parameters ==

// PWM frequency for the motor driver, 25kHz is high enough to be inaudible.
const uint PWM_FREQUENCY_HZ = 25000;
// PWM counter wrap value, calculated from the 125MHz system clock.
// Formula: (SystemClock / PWM_Frequency) - 1. This value determines the PWM resolution.
static uint16_t PWM_WRAP_VALUE = (125000000 / PWM_FREQUENCY_HZ) - 1;
// Delay after the PWM cycle before triggering ADC, allows the motor coils' magnetic field to collapse.
const uint BEMF_MEASUREMENT_DELAY_US = 10;
// Ring buffer size for ADC samples. Must be a power of 2 for DMA efficiency.
const uint BEMF_RING_BUFFER_SIZE = 64;

//== Static Globals for Hardware Control ==
static uint dma_channel;     // DMA channel for ADC->memory transfers
static uint motor_pwm_slice; // The RP2040 PWM slice driving the motor
// Volatile is required as this buffer is written by DMA and read by the CPU in an ISR.
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_pwm_a_pin; // Motor PWM A pin
static uint8_t g_pwm_b_pin; // Motor PWM B pin

// Forward Declarations
static void dma_irq_handler();
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data);
static void on_pwm_wrap();

// DMA ISR: triggered when the BEMF ring buffer is full.
// This is the core of the measurement, processing raw ADC data and passing it to the controller.
static void dma_irq_handler() {
    // Clear the DMA interrupt flag for our channel using a bitmask.
    dma_hw->ints0 = 1u << dma_channel;

    uint32_t sum_A = 0, sum_B = 0;
    // De-interleave and sum the ADC samples for each motor terminal (B, A, B, A, ...).
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_B += bemf_ring_buffer[i];
        sum_A += bemf_ring_buffer[i + 1];
    }
    // Calculate the average differential BEMF.
    int measured_bemf = abs((int)(sum_A / (BEMF_RING_BUFFER_SIZE / 2)) - (int)(sum_B / (BEMF_RING_BUFFER_SIZE / 2)));

    if (bemf_callback) {
        bemf_callback(measured_bemf);
    }
    // Restart DMA transfer to refill the buffer for the next cycle.
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

// One-shot hardware timer callback to trigger the ADC conversion.
// This is called after the BEMF_MEASUREMENT_DELAY_US to ensure a stable reading.
static int64_t delayed_adc_trigger_callback(alarm_id_t id, void *user_data) {
    // Start a single ADC conversion sequence that will run until the DMA buffer is full.
    adc_run(true);
    return 0; // Returning 0 prevents the timer from rescheduling.
}

// PWM Wrap ISR: synchronizes ADC measurement with the PWM cycle.
// This is triggered at the end of each PWM cycle.
static void on_pwm_wrap() {
    pwm_clear_irq(motor_pwm_slice);
    // Schedule the ADC trigger to run after the specified delay.
    add_alarm_in_us(BEMF_MEASUREMENT_DELAY_US, delayed_adc_trigger_callback, NULL, true);
}

//== Public HAL Function Implementations ==

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    bemf_callback = callback;

    // --- ADC and DMA Setup ---
    adc_init();
    adc_gpio_init(bemf_b_pin);
    adc_gpio_init(bemf_a_pin);
    // Configure ADC for round-robin sampling. The bitmask `(1u << N)` selects ADC channel N.
    // Arduino pins A0-A3 map to ADC0-3 (GPIO 26-29), so `pin - 26` converts GPIO to ADC channel.
    adc_set_round_robin((1u << (bemf_b_pin - 26)) | (1u << (bemf_a_pin - 26)));
    adc_select_input(bemf_b_pin - 26);
    // Configure the ADC FIFO to generate a DMA request (DREQ) for every sample.
    adc_fifo_setup(true, true, 1, false, false);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); // 16-bit ADC samples
    channel_config_set_read_increment(&dma_config, false);           // Read from same ADC FIFO address
    channel_config_set_write_increment(&dma_config, true);           // Write to sequential buffer addresses
    channel_config_set_dreq(&dma_config, DREQ_ADC);                  // Trigger DMA from ADC
    // Configure a ring buffer write address. The '6' is log2(BEMF_RING_BUFFER_SIZE), i.e., log2(64).
    channel_config_set_ring(&dma_config, true, 6);

    // Apply the DMA config: read from ADC FIFO, write to our ring buffer.
    dma_channel_configure(dma_channel, &dma_config, bemf_ring_buffer, &adc_hw->fifo, BEMF_RING_BUFFER_SIZE, false);
    // Set up the DMA interrupt handler.
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- PWM Setup for Motor Control ---
    gpio_set_function(g_pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(g_pwm_b_pin, GPIO_FUNC_PWM);
    // Both PWM pins must be on the same slice.
    motor_pwm_slice = pwm_gpio_to_slice_num(g_pwm_a_pin);

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_wrap(&motor_pwm_conf, PWM_WRAP_VALUE);
    pwm_init(motor_pwm_slice, &motor_pwm_conf, true);

    // --- PWM Interrupt for Synchronization ---
    pwm_clear_irq(motor_pwm_slice);
    // Enable the interrupt that fires when the PWM counter wraps.
    pwm_set_irq_enabled(motor_pwm_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Start the DMA, which will now wait for the first ADC trigger.
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // Map the 8-bit duty cycle (0-255) to the PWM counter's range.
    uint16_t level = map(duty_cycle, 0, 255, 0, PWM_WRAP_VALUE);

    if (forward) {
        // For forward, PWM is applied to pin A and pin B is held low.
        pwm_set_gpio_level(g_pwm_a_pin, level);
        pwm_set_gpio_level(g_pwm_b_pin, 0);
    } else {
        // For reverse, pin A is held low and PWM is applied to pin B.
        pwm_set_gpio_level(g_pwm_a_pin, 0);
        pwm_set_gpio_level(g_pwm_b_pin, level);
    }
}

#endif // USE_RP2040_LOWLEVEL && ARDUINO_ARCH_RP2040
