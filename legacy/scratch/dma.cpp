#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// --- Pin Configuration ---
// Output pin for the motor PWM
const uint MOTOR_PWM_PIN = 25; // Onboard LED for easy testing
// Input pin for BEMF measurement
const uint BEMF_ADC_PIN = 26;  // ADC Channel 0
const uint ADC_CHANNEL = 0;

// --- Timing Configuration ---
#define SYS_CLOCK_HZ 125000000
#define PWM_FREQ 500
#define PWM_CLOCK_DIV 12.5f // 125MHz / 12.5 = 10MHz PWM clock
// 10MHz clock -> 0.1µs per step
// 2000µs period / 0.1µs/step = 20000 steps
#define PWM_WRAP_VALUE (19999) 

// 100µs settling delay / 0.1µs/step = 1000 steps
#define PWM_SETTLE_STEPS (1000) 
// 50µs measurement / 0.1µs/step = 500 steps
#define PWM_MEASURE_STEPS (500) 
// Total cutout = 100µs + 50µs = 150µs
#define PWM_CUTOUT_STEPS (PWM_SETTLE_STEPS + PWM_MEASURE_STEPS) // 1500 steps

// Max 'on' time = 20000 - 1500 = 18500 steps
#define MAX_PWM_ON_STEPS (PWM_WRAP_VALUE + 1 - PWM_CUTOUT_STEPS)

#define SETTLE_DELAY_US (100)
#define ADC_SAMPLE_FREQ (500000) // 500ksps = 2µs per sample
#define ADC_SAMPLE_COUNT (25)    // 25 samples * 2µs/sample = 50µs window

// --- Global Hardware Variables ---
static uint pwm_slice_motor;
static uint motor_pwm_channel = PWM_CHAN_B; // Use Channel B for this trick
static int dma_chan_start_timer;
static int dma_chan_start_adc;
static int dma_chan_read_adc;
static uint32_t timer_arm_cmd = 1u; // Value to write to timer_hw->arm

// --- Data Buffers ---
// DMA will write ADC results here
static uint16_t adc_capture_buffer[ADC_SAMPLE_COUNT];
// Globals for sharing data from ISR to main loop
static volatile bool new_data_ready = false;
static volatile uint16_t avg_bemf = 0;


/**
 * @brief DUMMY CONTROL LOOP
 * This is where your actual motor control logic would go.
 * It's called from the ISR, so it MUST be fast.
 * * @param bemf_value The average BEMF reading from the 50µs window.
 * @return float A requested duty cycle from 0.0 to 1.0.
 */
float run_my_control_logic(uint16_t bemf_value) {
    // For this demo, we'll just return a fixed 50% duty cycle.
    // In a real app, you'd have a PID controller or state machine here.
    return 0.5f; 
}


/**
 * @brief ISR: The Control Loop
 * This function is triggered by DMA_IRQ_0 when the ADC
 * capture (25 samples) is complete at T=150µs.
 */
void control_loop_isr() {
    // 1. Clear the interrupt flag
    dma_irqn_clear_channel_interrupt(DMA_IRQ_0, dma_chan_read_adc);

    // 2. Process the BEMF data (fast averaging)
    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        total += adc_capture_buffer[i];
    }
    avg_bemf = total / ADC_SAMPLE_COUNT; // Update global for logging

    // 3. Run the time-critical control logic
    float duty_cycle_request = run_my_control_logic(avg_bemf); // Request 0.0 to 1.0

    // 4. Scale the request to our available "on" time (18500 steps)
    uint16_t on_time_steps = (uint16_t)(duty_cycle_request * (float)MAX_PWM_ON_STEPS);
    
    // Clamp to max
    if (on_time_steps > MAX_PWM_ON_STEPS) {
        on_time_steps = MAX_PWM_ON_STEPS;
    }

    // 5. Set the final PWM level for the NEXT cycle
    // The B pin is HIGH from LEVEL_A (cutout) to LEVEL_B (cutout + on_time)
    uint16_t new_level_b = PWM_CUTOUT_STEPS + on_time_steps;
    pwm_set_chan_level(pwm_slice_motor, motor_pwm_channel, new_level_b);
    
    // 6. Signal the main loop that data is ready for logging
    new_data_ready = true;
}


void setup_hardware() {
    // --- 1. ADC Setup ---
    adc_init();
    adc_gpio_init(BEMF_ADC_PIN);
    adc_select_input(ADC_CHANNEL);

    // Set ADC clock to 48MHz / 96 = 500 kHz (2µs per sample)
    adc_set_clkdiv((float)clock_get_hz(clk_adc) / ADC_SAMPLE_FREQ - 1);

    adc_fifo_setup(
        true,  // Write each conversion to FIFO
        true,  // Enable DMA DREQ
        1,     // DREQ threshold
        false, // Don't include error bit
        false  // No byte shift
    );
    adc_run(false); // Start in free-running mode, but don't run yet

    // --- 2. Timer Setup (for 100µs delay) ---
    // We'll use TIMER_ALARM[0]
    // Enable DREQ on ALARM 0
    hw_set_bits(&timer_hw->inte, 1u << 0);
    // Timer will be armed by DMA

    // --- 3. PWM Setup (Motor Output & Master Trigger) ---
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    pwm_slice_motor = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);

    pwm_config cfg = pwm_get_default_config();
    // Set 500 Hz clock
    pwm_config_set_clkdiv(&cfg, PWM_CLOCK_DIV);
    pwm_config_set_wrap(&cfg, PWM_WRAP_VALUE);
    pwm_init(pwm_slice_motor, &cfg, false); // Don't start yet

    // THIS IS THE TRICK:
    // Set CHAN_A level to the end of the cutout window (150µs)
    // We don't use CHAN_A pin, just its register.
    pwm_set_chan_level(pwm_slice_motor, PWM_CHAN_A, PWM_CUTOUT_STEPS);

    // Set CHAN_B level (our motor pin) to also be at the cutout.
    // This means 0% duty cycle to start.
    // The B pin will be HIGH from LEVEL_A to LEVEL_B.
    pwm_set_chan_level(pwm_slice_motor, motor_pwm_channel, PWM_CUTOUT_STEPS);
    
    // Enable DREQ on WRAP (T=0)
    // This DREQ will trigger DMA 1 to start the whole chain.
    pwm_set_dreq_enabled(pwm_slice_motor, true);


    // --- 4. DMA Setup ---
    dma_chan_start_timer = dma_claim_unused_channel(true);
    dma_chan_start_adc = dma_claim_unused_channel(true);
    dma_chan_read_adc = dma_claim_unused_channel(true);

    // --- DMA 1: PWM Wrap -> Start 100µs Timer ---
    dma_channel_config c1 = dma_channel_get_default_config(dma_chan_start_timer);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, pwm_get_dreq(pwm_slice_motor)); // Triggered by PWM WRAP
    
    dma_channel_configure(
        dma_chan_start_timer,
        &c1,
        &timer_hw->arm,               // Write to: Arm the timer
        &timer_arm_cmd,               // Read from: A variable holding '1'
        1,                            // Transfer count
        false                         // Don't start yet
    );

    // --- DMA 2: Timer Alarm (T=100µs) -> Start ADC ---
    dma_channel_config c2 = dma_channel_get_default_config(dma_chan_start_adc);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, false);
    channel_config_set_dreq(&c2, DREQ_TIMER_ALARM0); // Triggered by Timer
    channel_config_set_chain_to(&c2, dma_chan_read_adc); // Chain to DMA 3
    
    dma_channel_configure(
        dma_chan_start_adc,
        &c2,
        &adc_hw->cs,                  // Write to: ADC Control/Status reg
        (void*)&adc_hw->cs,           // Read from: ADC CS reg (read-modify-write)
        1,                            // Transfer count
        false                         // Don't start yet
    );
    // Set the bit we want to write: START_MANY (bit 3)
    dma_channel_set_read_addr(dma_chan_start_adc, &adc_hw->cs, false);
    dma_channel_set_write_addr(dma_chan_start_adc, &adc_hw->cs, false);
    dma_channel_set_trans_count(dma_chan_start_adc, 1, false);
    // This is a special "read-modify-write" DMA operation
    dma_hw->ch[dma_chan_start_adc].al1_ctrl = (dma_hw->ch[dma_chan_start_adc].al1_ctrl & ~DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) | (DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS); // Set INCR_WRITE to false
    dma_hw->ch[dma_chan_start_adc].al2_read_addr_trig = (io_rw_32)&adc_hw->cs;
    dma_hw->ch[dma_chan_start_adc].al3_write_addr_trig = (io_rw_32)&adc_hw->cs;
    dma_hw->ch[dma_chan_start_adc].al1_ctrl |= (1u << ADC_CS_START_MANY_LSB); // Force bit 3 high on write
    

    // --- DMA 3: ADC FIFO Ready -> Read ADC to Buffer ---
    dma_channel_config c3 = dma_channel_get_default_config(dma_chan_read_adc);
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_16);
    channel_config_set_read_increment(&c3, false);
    channel_config_set_write_increment(&c3, true);
    channel_config_set_dreq(&c3, DREQ_ADC); // Triggered by ADC FIFO
    
    dma_channel_configure(
        dma_chan_read_adc,
        &c3,
        adc_capture_buffer,           // Write to: Our RAM buffer
        &adc_hw->fifo,                // Read from: ADC FIFO
        ADC_SAMPLE_COUNT,             // Transfer count (25)
        false                         // Don't start yet
    );
    // Enable interrupt on completion
    dma_irqn_set_channel_enabled(DMA_IRQ_0, dma_chan_read_adc, true);


    // --- 5. Interrupt Setup ---
    irq_set_exclusive_handler(DMA_IRQ_0, control_loop_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    // --- 6. Start the System! ---
    // Arm the first DMA channel. It will wait for the first PWM DREQ.
    dma_channel_start(dma_chan_start_timer);
    // Start the PWM. The first WRAP event will fire the whole chain.
    pwm_set_enabled(pwm_slice_motor, true);
}


void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial
    }
    Serial.println("\nRP2040 BEMF Hardware Controller");
    Serial.println("---------------------------------");
    
    setup_hardware();
    
    Serial.println("PWM, ADC, DMA, and Timer chain initialized.");
    Serial.println("System running at 500 Hz.");
}


void loop() {
    // The main loop is only for low-priority tasks, like logging.
    // The entire control loop runs in the ISR.
    
    if (new_data_ready) {
        // Safely print the average BEMF from the last cycle
        Serial.print("BEMF Avg (12-bit): ");
        Serial.println(avg_bemf);
        
        // Clear the flag
        new_data_ready = false;
    }
    
    // You could update control parameters (like target speed) here
    // and the ISR would pick them up on its next run.
    
    delay(100); // Don't spam the serial monitor
}
