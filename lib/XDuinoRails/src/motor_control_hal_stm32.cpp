#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

// BEMF Measurement Parameters
const uint BEMF_RING_BUFFER_SIZE = 64;

// Static Globals for Hardware Control
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];

// HardwareTimer for PWM
HardwareTimer* pwm_timer;
uint32_t pwm_channel_a;
uint32_t pwm_channel_b;

// DMA and ADC handles
DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

// DMA interrupt handler
extern "C" void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    uint32_t sum_A = 0, sum_B = 0;
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_A += bemf_ring_buffer[i];
        sum_B += bemf_ring_buffer[i + 1];
    }
    int measured_bemf = abs((int)(sum_A / (BEMF_RING_BUFFER_SIZE / 2)) - (int)(sum_B / (BEMF_RING_BUFFER_SIZE / 2)));

    if (bemf_callback) {
        bemf_callback(measured_bemf);
    }
}

void hal_motor_init(uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t bemf_a_pin, uint8_t bemf_b_pin, hal_bemf_update_callback_t callback) {
    g_pwm_a_pin = pwm_a_pin;
    g_pwm_b_pin = pwm_b_pin;
    g_bemf_a_pin = bemf_a_pin;
    g_bemf_b_pin = bemf_b_pin;
    bemf_callback = callback;

    // --- PWM Setup ---
    TIM_TypeDef *pwm_timer_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(g_pwm_a_pin), PinMap_TIM);
    pwm_timer = new HardwareTimer(pwm_timer_instance);
    pwm_channel_a = stm32_timer_get_channel(digitalPinToPinName(g_pwm_a_pin));
    pwm_channel_b = stm32_timer_get_channel(digitalPinToPinName(g_pwm_b_pin));
    pwm_timer->setMode(pwm_channel_a, TIMER_OUTPUT_COMPARE_PWM1, g_pwm_a_pin);
    pwm_timer->setMode(pwm_channel_b, TIMER_OUTPUT_COMPARE_PWM1, g_pwm_b_pin);
    pwm_timer->setOverflow(25000, HERTZ_FORMAT);
    pwm_timer->resume();

    // --- ADC and DMA Setup ---
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.ScanConvMode = ENABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 2;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc);

    sConfig.Channel = stm32_adc_get_channel(digitalPinToPinName(g_bemf_a_pin));
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    sConfig.Channel = stm32_adc_get_channel(digitalPinToPinName(g_bemf_b_pin));
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc.Instance = DMA2_Stream0;
    hdma_adc.Init.Channel = DMA_CHANNEL_0;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc);

    __HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_ADC_Start_DMA(&hadc, (uint32_t*)bemf_ring_buffer, BEMF_RING_BUFFER_SIZE);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    uint32_t overflow = pwm_timer->getOverflow();
    uint32_t ticks = map(duty_cycle, 0, 255, 0, overflow);

    if (forward) {
        pwm_timer->setCaptureCompare(pwm_channel_a, ticks, TICK_COMPARE_FORMAT);
        pwm_timer->setCaptureCompare(pwm_channel_b, 0, TICK_COMPARE_FORMAT);

    } else {
        pwm_timer->setCaptureCompare(pwm_channel_a, 0, TICK_COMPARE_FORMAT);
        pwm_timer->setCaptureCompare(pwm_channel_b, ticks, TICK_COMPARE_FORMAT);
    }
}

void hal_motor_deinit() {
    HAL_ADC_Stop_DMA(&hadc);
    HAL_DMA_DeInit(&hdma_adc);
    HAL_ADC_DeInit(&hadc);
    pwm_timer->pause();
    delete pwm_timer;
}

#endif // ARDUINO_ARCH_STM32
