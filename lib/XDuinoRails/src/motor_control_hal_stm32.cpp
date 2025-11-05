#include "motor_control_hal.h"

#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

// BEMF Measurement Parameters
// The size of the buffer for DMA transfers. Must be an even number.
const uint BEMF_RING_BUFFER_SIZE = 64;

// Static Globals for Hardware Control
static hal_bemf_update_callback_t bemf_callback = nullptr;
static uint8_t g_bemf_a_pin;
static uint8_t g_bemf_b_pin;
static uint8_t g_pwm_a_pin;
static uint8_t g_pwm_b_pin;
// Buffer to store ADC readings from DMA
static volatile uint16_t bemf_ring_buffer[BEMF_RING_BUFFER_SIZE];

// HardwareTimer for PWM
HardwareTimer* pwm_timer;
uint32_t pwm_channel_a;
uint32_t pwm_channel_b;

// DMA and ADC handles
DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;

// DMA interrupt handler for ADC1. On the F446RE, ADC1 is connected to DMA2, Stream 0.
extern "C" void DMA2_Stream0_IRQHandler(void) {
    // This function is called when the DMA transfer is complete.
    // It calls the HAL's DMA interrupt handler to process the transfer.
    HAL_DMA_IRQHandler(&hdma_adc);
}

// This callback is invoked by the HAL when the ADC DMA transfer is complete.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    uint32_t sum_A = 0, sum_B = 0;
    // The ADC is configured to sample BEMF A, then BEMF B, and so on.
    // This loop de-interleaves the DMA buffer and sums the readings for each channel.
    for (uint i = 0; i < BEMF_RING_BUFFER_SIZE; i += 2) {
        sum_A += bemf_ring_buffer[i];
        sum_B += bemf_ring_buffer[i + 1];
    }
    // Calculate the average differential BEMF.
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
    // Find the timer instance associated with the PWM pin.
    TIM_TypeDef *pwm_timer_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(g_pwm_a_pin), PinMap_TIM);
    pwm_timer = new HardwareTimer(pwm_timer_instance);

    // Get the timer channels for the PWM pins.
    pwm_channel_a = stm32_timer_get_channel(digitalPinToPinName(g_pwm_a_pin));
    pwm_channel_b = stm32_timer_get_channel(digitalPinToPinName(g_pwm_b_pin));

    // Configure the timer channels for PWM output.
    pwm_timer->setMode(pwm_channel_a, TIMER_OUTPUT_COMPARE_PWM1, g_pwm_a_pin);
    pwm_timer->setMode(pwm_channel_b, TIMER_OUTPUT_COMPARE_PWM1, g_pwm_b_pin);

    // Set the PWM frequency to 25kHz.
    pwm_timer->setOverflow(25000, HERTZ_FORMAT);
    pwm_timer->resume();

    // --- ADC and DMA Setup ---
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure the ADC peripheral.
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // ADC clock is derived from PCLK2
    hadc.Init.Resolution = ADC_RESOLUTION_12B;          // 12-bit resolution
    hadc.Init.ScanConvMode = ENABLE;                   // Scan multiple channels
    hadc.Init.ContinuousConvMode = ENABLE;             // Continuous conversion
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // No external trigger
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 2;                     // Two channels to convert
    hadc.Init.DMAContinuousRequests = ENABLE;          // Enable DMA requests
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc);

    // Configure the first ADC channel (BEMF A).
    sConfig.Channel = stm32_adc_get_channel(digitalPinToPinName(g_bemf_a_pin));
    sConfig.Rank = 1; // First in the sequence
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    // Configure the second ADC channel (BEMF B).
    sConfig.Channel = stm32_adc_get_channel(digitalPinToPinName(g_bemf_b_pin));
    sConfig.Rank = 2; // Second in the sequence
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    // Enable the DMA controller clock.
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Configure the DMA stream for ADC1.
    hdma_adc.Instance = DMA2_Stream0;
    hdma_adc.Init.Channel = DMA_CHANNEL_0; // ADC1 is on DMA2, Channel 0
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16-bit data
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR; // Continuously fill the buffer
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc);

    // Link the DMA handle to the ADC handle.
    __HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);

    // Configure and enable the DMA interrupt.
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // Start the ADC with DMA.
    HAL_ADC_Start_DMA(&hadc, (uint32_t*)bemf_ring_buffer, BEMF_RING_BUFFER_SIZE);
}

void hal_motor_set_pwm(int duty_cycle, bool forward) {
    // Get the timer's auto-reload value (the maximum counter value).
    uint32_t overflow = pwm_timer->getOverflow();
    // Map the 8-bit duty cycle (0-255) to the timer's counter range.
    uint32_t ticks = map(duty_cycle, 0, 255, 0, overflow);

    if (forward) {
        pwm_timer->setCaptureCompare(pwm_channel_a, ticks, TICK_COMPARE_FORMAT);
        pwm_timer->setCaptureCompare(pwm_channel_b, 0, TICK_COMPARE_FORMAT);

    } else {
        pwm_timer->setCaptureCompare(pwm_channel_a, 0, TICK_COMPARE_FORMAT);
        pwm_timer->setCaptureCompare(pwm_channel_b, ticks, TICK_COMPARE_FORMAT);
    }
}

// De-initializes the hardware used for motor control.
void hal_motor_deinit() {
    // Stop the ADC and DMA.
    HAL_ADC_Stop_DMA(&hadc);
    HAL_DMA_DeInit(&hdma_adc);
    HAL_ADC_DeInit(&hadc);
    // Stop and clean up the PWM timer.
    pwm_timer->pause();
    delete pwm_timer;
}

#endif // ARDUINO_ARCH_STM32
