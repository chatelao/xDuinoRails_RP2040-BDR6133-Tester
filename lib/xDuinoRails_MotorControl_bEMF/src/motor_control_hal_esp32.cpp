/**
 * @file motor_control_hal_esp32.cpp
 * @brief ESP32-specific implementation for the motor control HAL.
 */

#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "soc/soc_caps.h"
#include "driver/adc.h"
#include "soc/adc_channel.h"

// PWM frequency for the motor driver
const uint32_t PWM_FREQUENCY_HZ = 25000;
// Timer resolution for PWM
const uint32_t PWM_TIMER_RESOLUTION_HZ = 10 * 1000 * 1000; // 10 MHz
// BEMF measurement cooldown delay
const uint32_t BEMF_MEASUREMENT_DELAY_US = 10;

// ADC DMA configuration
#define ADC_RESULT_BYTE 2
#define ADC_CONV_FRAME_SIZE 100
#define ADC_DMA_BUF_SIZE (ADC_CONV_FRAME_SIZE * ADC_RESULT_BYTE)

// Static Globals for Hardware Control
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static adc_channel_t g_bemf_a_channel;
static adc_channel_t g_bemf_b_channel;
static uint16_t s_dma_buf[ADC_DMA_BUF_SIZE] = {0};

static mcpwm_cmpr_handle_t comparator_a = nullptr;
static mcpwm_cmpr_handle_t comparator_b = nullptr;
static mcpwm_oper_handle_t oper = nullptr;
static mcpwm_timer_handle_t timer = nullptr;
static gptimer_handle_t gptimer = nullptr;

void hal_read_and_process_bemf();

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    bemf_callback = callback;

    // --- MCPWM Setup ---
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PWM_TIMER_RESOLUTION_HZ / PWM_FREQUENCY_HZ,
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    mcpwm_new_operator(&operator_config, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_config, &comparator_a);
    mcpwm_new_comparator(oper, &comparator_config, &comparator_b);

    mcpwm_generator_handle_t generator_a = nullptr;
    mcpwm_generator_config_t gen_config_a = { .gen_gpio_num = g_pwm_a_pin };
    mcpwm_new_generator(oper, &gen_config_a, &generator_a);

    mcpwm_generator_handle_t generator_b = nullptr;
    mcpwm_generator_config_t gen_config_b = { .gen_gpio_num = g_pwm_b_pin };
    mcpwm_new_generator(oper, &gen_config_b, &generator_b);

    // Set actions for generators to produce standard PWM
    mcpwm_generator_set_action_on_timer_event(generator_a, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_a, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(generator_b, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(generator_b, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // --- GPTimer Setup for Cooldown Delay ---
    gptimer_config_t gptimer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    gptimer_new_timer(&gptimer_config, &gptimer);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = BEMF_MEASUREMENT_DELAY_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer);
    // Note: gptimer is not started here. It will be started by the ETM.

    bemf_callback = callback;

    g_bemf_a_channel = (adc_channel_t)ADC1_GPIO32_CHANNEL;
    g_bemf_b_channel = (adc_channel_t)ADC1_GPIO33_CHANNEL;

    adc1_config_channel_atten(g_bemf_a_channel, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(g_bemf_b_channel, ADC_ATTEN_DB_11);

    adc_digi_pattern_table_t adc_pattern[2] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .channel = g_bemf_a_channel,
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        },
        {
            .atten = ADC_ATTEN_DB_11,
            .channel = g_bemf_b_channel,
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        },
    };

    adc_digi_config_t dig_cfg = {
        .conv_limit_en = false,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .adc1_pattern_len = sizeof(adc_pattern) / sizeof(adc_digi_pattern_table_t),
        .adc1_pattern = adc_pattern,
        .dma_eof_num = ADC_CONV_FRAME_SIZE,
        .dma_work_mode = ADC_DMA_WORK_MODE_CIRCULAR,
        .dma_data = (void*)s_dma_buf,
        .dma_size = ADC_DMA_BUF_SIZE,
    };
    adc_digi_controller_config(&dig_cfg);
    adc_digi_start();


    // --- ETM Setup for Hardware Triggering ---
    mcpwm_etm_event_config_t mcpwm_event_config = { .event_type = MCPWM_TIMER_EVENT_FULL };
    esp_etm_event_handle_t mcpwm_timer_event;
    mcpwm_new_etm_event(timer, &mcpwm_event_config, &mcpwm_timer_event);

    gptimer_etm_task_config_t gptimer_start_task_config = { .task_type = GPTIMER_ETM_TASK_START_COUNT };
    esp_etm_task_handle_t gptimer_start_task;
    gptimer_new_etm_task(gptimer, &gptimer_start_task_config, &gptimer_start_task);

    gptimer_etm_event_config_t gptimer_event_config = { .event_type = GPTIMER_EVENT_ALARM };
    esp_etm_event_handle_t gptimer_alarm_event;
    gptimer_new_etm_event(gptimer, &gptimer_event_config, &gptimer_alarm_event);

    // Placeholder for ADC ETM task - to be implemented when ADC ETM driver is available.
    // For now, we will use a software ISR from the GPTimer.
    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_alarm_isr_handler,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, nullptr);

    gptimer_etm_task_config_t gptimer_stop_task_config = { .task_type = GPTIMER_ETM_TASK_STOP_COUNT };
    esp_etm_task_handle_t gptimer_stop_task;
    gptimer_new_etm_task(gptimer, &gptimer_stop_task_config, &gptimer_stop_task);

    esp_etm_channel_config_t etm_config_1 = {};
    esp_etm_channel_handle_t etm_channel_1;
    esp_etm_new_channel(&etm_config_1, &etm_channel_1);
    esp_etm_channel_connect(etm_channel_1, mcpwm_timer_event, gptimer_start_task);
    esp_etm_channel_enable(etm_channel_1);

    esp_etm_channel_config_t etm_config_3 = {};
    esp_etm_channel_handle_t etm_channel_3;
    esp_etm_new_channel(&etm_config_3, &etm_channel_3);
    esp_etm_channel_connect(etm_channel_3, gptimer_alarm_event, gptimer_stop_task);
    esp_etm_channel_enable(etm_channel_3);
}

static bool IRAM_ATTR gptimer_alarm_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    adc_digi_start();
    gptimer_set_raw_count(timer, 0);
    return false;
}

void hal_read_and_process_bemf() {
    uint32_t ret_num = 0;
    adc_digi_output_data_t results[ADC_CONV_FRAME_SIZE];
    uint8_t result = adc_digi_read_bytes((uint8_t*)&results, ADC_CONV_FRAME_SIZE, &ret_num, ADC_MAX_DELAY);

    if (result == ESP_OK) {
        for (int i = 0; i < ret_num; i += 2) {
            uint16_t adc_val_a = results[i].type2.data;
            uint16_t adc_val_b = results[i+1].type2.data;

            if (bemf_callback) {
                bemf_callback(abs(adc_val_a - adc_val_b));
            }
        }
    }
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    uint32_t duty_ticks = (PWM_TIMER_RESOLUTION_HZ / PWM_FREQUENCY_HZ) * (duty_cycle / 255.0f);

    if (forward) {
        mcpwm_comparator_set_compare_value(comparator_a, duty_ticks);
        mcpwm_comparator_set_compare_value(comparator_b, 0);
    } else {
        mcpwm_comparator_set_compare_value(comparator_a, 0);
        mcpwm_comparator_set_compare_value(comparator_b, duty_ticks);
    }
}

#endif // ARDUINO_ARCH_ESP32
