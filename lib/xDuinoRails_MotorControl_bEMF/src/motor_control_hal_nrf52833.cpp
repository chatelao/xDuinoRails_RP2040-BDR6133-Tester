#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>
#include <nRF52_PWM.h>
#include <nrfx_saadc.h>
#include "nrfx_timer.h"
#include "nrfx_ppi.h"

// PWM instances for motor control.
static nRF52_PWM* pwm_a;
static nRF52_PWM* pwm_b;

// SAADC, PPI, and Timer instances for BEMF measurement.
#define ADC_CHANNELS_IN_USE 2
#define SAADC_BUF_SIZE ADC_CHANNELS_IN_USE
#define SAADC_BUF_COUNT 2

static nrf_saadc_value_t samples[SAADC_BUF_COUNT][SAADC_BUF_SIZE];
static const nrfx_timer_t m_sample_timer = NRFX_TIMER_INSTANCE(1);
static nrf_ppi_channel_t m_timer_saadc_ppi_channel;

// Store pin numbers for later use.
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static hal_bemf_update_callback_t g_bemf_callback;

// Desired PWM frequency in Hz.
const float PWM_FREQUENCY = 20000.0f;

// BEMF measurement interval in milliseconds.
const uint32_t BEMF_MEASUREMENT_INTERVAL_MS = 10;

// Simple function to provide an index to the next input buffer
static uint32_t next_free_buf_index(void) {
    static uint32_t buffer_index = -1;
    buffer_index = (buffer_index + 1) % SAADC_BUF_COUNT;
    return buffer_index;
}

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event) {
    ret_code_t err_code;
    switch (p_event->type) {
        case NRFX_SAADC_EVT_DONE:
            if (g_bemf_callback) {
                int differential_bemf = abs(p_event->data.done.p_buffer[0] - p_event->data.done.p_buffer[1]);
                g_bemf_callback(differential_bemf);
            }
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

static void timer_init(void) {
    nrfx_err_t err_code;
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
    err_code = nrfx_timer_init(&m_sample_timer, &timer_config, NULL);
    APP_ERROR_CHECK(err_code);
    nrfx_timer_extended_compare(&m_sample_timer, NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_ms_to_ticks(&m_sample_timer, BEMF_MEASUREMENT_INTERVAL_MS),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrfx_timer_resume(&m_sample_timer);
}

static void ppi_init(void) {
    nrfx_err_t err_code = nrfx_ppi_channel_alloc(&m_timer_saadc_ppi_channel);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(m_timer_saadc_ppi_channel,
                                       nrfx_timer_event_address_get(&m_sample_timer, NRF_TIMER_EVENT_COMPARE0),
                                       nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_enable(m_timer_saadc_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

static void adc_configure(void) {
    ret_code_t err_code;
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    saadc_adv_config.internal_timer_cc = 0;
    saadc_adv_config.start_on_end = true;

    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err_code);

    static nrfx_saadc_channel_t channel_configs[ADC_CHANNELS_IN_USE];

    // Configure BEMF A pin
    nrfx_saadc_channel_t config_a = NRFX_SAADC_DEFAULT_CHANNEL_SE(g_bemf_a_pin, 0);
    memcpy(&channel_configs[0], &config_a, sizeof(config_a));

    // Configure BEMF B pin
    nrfx_saadc_channel_t config_b = NRFX_SAADC_DEFAULT_CHANNEL_SE(g_bemf_b_pin, 1);
    memcpy(&channel_configs[1], &config_b, sizeof(config_b));

    err_code = nrfx_saadc_channels_config(channel_configs, ADC_CHANNELS_IN_USE);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_advanced_mode_set(0x03, NRF_SAADC_RESOLUTION_14BIT, &saadc_adv_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_set(&samples[next_free_buf_index()][0], SAADC_BUF_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_mode_trigger();
    APP_ERROR_CHECK(err_code);
}

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    g_bemf_callback = callback;

    // Create new PWM instances for each motor direction.
    pwm_a = new nRF52_PWM(g_pwm_a_pin, PWM_FREQUENCY, 0.0f);
    pwm_b = new nRF52_PWM(g_pwm_b_pin, PWM_FREQUENCY, 0.0f);

    // Initialize the PWM instances.
    if (pwm_a) {
        pwm_a->setPWM();
    }
    if (pwm_b) {
        pwm_b->setPWM();
    }

    adc_configure();
    ppi_init();
    timer_init();
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // The duty cycle is a float from 0.0 to 100.0 for the nRF52_PWM library.
    float duty_cycle_float = map(duty_cycle, 0, 255, 0, 100);

    if (forward) {
        pwm_a->setPWM(g_pwm_a_pin, PWM_FREQUENCY, duty_cycle_float);
        pwm_b->setPWM(g_pwm_b_pin, PWM_FREQUENCY, 0.0f);
    } else {
        pwm_a->setPWM(g_pwm_a_pin, PWM_FREQUENCY, 0.0f);
        pwm_b->setPWM(g_pwm_b_pin, PWM_FREQUENCY, duty_cycle_float);
    }
}

#endif // ARDUINO_ARCH_NRF52
