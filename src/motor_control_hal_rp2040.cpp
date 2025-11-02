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
const uint PWM_FREQUENCY_HZ = 25000;
static uint16_t PWM_WRAP_VALUE = (125000000 / PWM_FREQUENCY_HZ) - 1;
const uint BEMF_MEASUREMENT_DELAY_US = 10;
const uint BEMF_RING_BUFFER_SIZE = 64;

//== Static Globals for Hardware Control ==
static uint dma_channel;
static uint motor_pwm_slice;
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];
static hal_bemf_update_callback_t bemf_callback = nullptr;

// Pin definitions, to be set in hal_motor_init
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;

// Forward Declarations
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
    dma_hw->ints0 = 1u << dma_channel;

    uint32_t sum_A = 0, sum_B = 0;
    // The ADC round-robin is configured for B(ADC2) then A(ADC3).
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_B += bemf_ring_buffer[i];
        sum_A += bemf_ring_buffer[i + 1];
    }
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
    adc_init();
    adc_gpio_init(bemf_b_pin);
    adc_gpio_init(bemf_a_pin);
    adc_set_round_robin((1u << (bemf_b_pin - 26)) | (1u << (bemf_a_pin - 26))); // Assumes pins are A0-A3 -> ADC0-3
    adc_select_input(bemf_b_pin - 26);
    adc_fifo_setup(true, true, 1, false, false); // DREQ on every sample

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
    channel_config_set_ring(&dma_config, true, 6); // Ring buffer size 64 = 2^6

    dma_channel_configure(dma_channel, &dma_config, bemf_ring_buffer, &adc_hw->fifo, BEMF_RING_BUFFER_SIZE, false);
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- PWM Setup for Motor Control ---
    gpio_set_function(g_pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(g_pwm_b_pin, GPIO_FUNC_PWM);
    motor_pwm_slice = pwm_gpio_to_slice_num(g_pwm_a_pin);

    pwm_config motor_pwm_conf = pwm_get_default_config();
    pwm_config_set_wrap(&motor_pwm_conf, PWM_WRAP_VALUE);
    pwm_init(motor_pwm_slice, &motor_pwm_conf, true);

    // --- PWM Interrupt for Synchronization ---
    pwm_clear_irq(motor_pwm_slice);
    pwm_set_irq_enabled(motor_pwm_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Start DMA, which will wait for the first ADC trigger.
    dma_channel_set_write_addr(dma_channel, bemf_ring_buffer, true);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    uint16_t level = map(duty_cycle, 0, 255, 0, PWM_WRAP_VALUE);
    if (forward) {
        pwm_set_gpio_level(g_pwm_a_pin, level);
        pwm_set_gpio_level(g_pwm_b_pin, 0);
    } else {
        pwm_set_gpio_level(g_pwm_a_pin, 0);
        pwm_set_gpio_level(g_pwm_b_pin, level);
    }
}

#endif // USE_RP2040_LOWLEVEL && ARDUINO_RASPBERRY_PI_PICO
