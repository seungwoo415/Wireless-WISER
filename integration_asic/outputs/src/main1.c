#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <zephyr/irq.h>
#include <nrfx_pwm.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_pwm.h>
#include <hal/nrf_ppi.h>

// logging 
LOG_MODULE_REGISTER(spect, LOG_LEVEL_INF);
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3

// pins 
#define DAC_SDA_PIN 12
#define DAC_SCL_PIN 11
#define SIPO_CLK_OUT 27
#define SIPO_DATA_OUT 26 
#define CLK_1MHZ_OUT 14 

// hardware indicies 
#define PWM_SIPO_INST_IDX 1
#define PWM_CLK_1MHZ_INST_IDX 2

#define PPI_CH NRF_PPI_CHANNEL0
#define PPI_DIS_CH NRF_PPI_CHANNEL1

uint32_t counter = 0; 

// hardware instances 
nrfx_pwm_t pwm_sipo_instance = NRFX_PWM_INSTANCE(PWM_SIPO_INST_IDX);
nrfx_pwm_t pwm_clk_1mhz_instance = NRFX_PWM_INSTANCE(PWM_CLK_1MHZ_INST_IDX);

// buffer 
static nrf_pwm_values_individual_t sipo_buffer[16]; 

static nrf_pwm_values_common_t clk_1mhz_value = 8;

// pwm sequences 
static nrf_pwm_sequence_t seq_sipo =
{
    .values.p_individual = sipo_buffer,
    .length           = 64, 
    .repeats          = 0,
    .end_delay        = 0
}; 

static nrf_pwm_sequence_t seq_clk_1mhz =
{
    .values.p_common = &clk_1mhz_value,
    .length           = 1, 
    .repeats          = 0,
    .end_delay        = 0
};

// global variables 
static volatile bool sipo_done = false; 
static volatile bool dac_i2c_done = false; 

// interrupt handlers 
static void pwm_sipo_handler(nrfx_pwm_evt_type_t event_type, void * p_context) {
    if (event_type == NRFX_PWM_EVT_FINISHED) {
        sipo_done = true;
        counter++; 
    }

    if (event_type == NRFX_PWM_EVT_END_SEQ0) {
        // This will fire 4 times for your test
        LOG_INF("Step detected via SEQEND0");
        sipo_done = true;
    }
}

static void pwm_sipo_init(void) {
        nrfx_err_t status;
        (void)status; 

        // sipo config 
        nrfx_pwm_config_t pwm_sipo_config = NRFX_PWM_DEFAULT_CONFIG(SIPO_CLK_OUT, SIPO_DATA_OUT, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_sipo_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_sipo_config.top_value = 16; 
        pwm_sipo_config.load_mode = NRF_PWM_LOAD_INDIVIDUAL;

        // enable sipo pwm interupt 
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_SIPO_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_SIPO_INST_IDX), 0, 0);


        status = nrfx_pwm_init(&pwm_sipo_instance, &pwm_sipo_config, pwm_sipo_handler, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);

        nrf_pwm_shorts_set(NRF_PWM1, NRF_PWM_SHORT_SEQEND0_STOP_MASK);
} 

static void pwm_clk_1mhz_init(void) {
        nrfx_err_t status;
        (void)status; 

        // sipo config 
        nrfx_pwm_config_t pwm_clk_1mhz_config = NRFX_PWM_DEFAULT_CONFIG(CLK_1MHZ_OUT, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_clk_1mhz_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_clk_1mhz_config.top_value = 16; 
        pwm_clk_1mhz_config.load_mode = NRF_PWM_LOAD_COMMON;

        status = nrfx_pwm_init(&pwm_clk_1mhz_instance, &pwm_clk_1mhz_config, NULL, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);

}

static void dac_reset() {    
    nrf_gpio_pin_set(DAC_SDA_PIN); 
    __DSB(); 
    k_busy_wait(5); 
    nrf_gpio_pin_set(DAC_SCL_PIN);
    dac_i2c_done = false; 
}


// write the i2c dac data 
static void dac_i2c_write(uint16_t dac2, uint16_t dac1, uint16_t dac0) 
{
    // 1. Concatenate data exactly like: {dac2, dac1, dac0[15:12]}
    // We use a 64-bit int to hold the 36-bit result comfortably
    uint64_t dac_main = 0;
    dac_main |= ((uint64_t)dac2 << 20);           // Top 16 bits
    dac_main |= ((uint64_t)dac1 << 4);            // Middle 16 bits
    dac_main |= ((uint64_t)(dac0 >> 12) & 0x0F);  // Bottom 4 bits

    // define critical section
    unsigned int i2c_key = irq_lock();

    // STATE 0: Idle/Trigger
    // In Verilog: next_dac_data_bit = 1'b0 (Start condition)
    nrf_gpio_pin_clear(DAC_SDA_PIN); 
    k_busy_wait(5); 
    nrf_gpio_pin_clear(DAC_SCL_PIN); 

    // STATE 1: Shifting 36 bits
    // counter_dac starts at 6'b100100 (36 decimal)
    for (int i = 35; i >= 0; i--) {   
        // set data bit 
        nrf_gpio_pin_write(DAC_SDA_PIN, (dac_main >> i) & 0x01); 

        // clock high 
        k_busy_wait(5); 
        nrf_gpio_pin_set(DAC_SCL_PIN); 

        // clock low 
        k_busy_wait(5); 
        nrf_gpio_pin_clear(DAC_SCL_PIN); 
    }

    // Wrap up / Reset State
    // In Verilog: next_dac_clk = 1'b1, next_dac_data_bit = 1'b1
    k_busy_wait(5); 
    nrf_gpio_pin_set(DAC_SCL_PIN); 
    k_busy_wait(5); 
    nrf_gpio_pin_set(DAC_SDA_PIN); 


    irq_unlock(i2c_key);
    
    LOG_INF("DAC Transaction Complete: 0x%09llX", dac_main);
    dac_i2c_done = true; 
} 

void ppi_init(void) {
    // setup disble of ppi when pwm sequence starts
    nrf_ppi_channel_include_in_group(NRF_PPI, PPI_CH, NRF_PPI_CHANNEL_GROUP0);

    nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CH, 
        nrf_pwm_event_address_get(NRF_PWM2, NRF_PWM_EVENT_PWMPERIODEND), 
        nrf_pwm_task_address_get(NRF_PWM1, NRF_PWM_TASK_SEQSTART0)
    );  

    nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_DIS_CH, 
        nrf_pwm_event_address_get(NRF_PWM1, NRF_PWM_EVENT_SEQSTARTED0), 
        nrf_ppi_task_group_disable_address_get(NRF_PPI, NRF_PPI_CHANNEL_GROUP0) // not sure if we have to use group disable task 
    );

    nrf_ppi_channel_enable(NRF_PPI, PPI_DIS_CH);
    
}

void sipo_prepare_buffer(void) {
    for (int i = 0; i < 16; i++) {
        sipo_buffer[i].channel_0 = 8;      // Clock pulse
        sipo_buffer[i].channel_1 = 0x8000; // Data bit
        sipo_buffer[i].channel_2 = 0;      // Empty
        sipo_buffer[i].channel_3 = 0;      // Empty
    }
}

void sipo_trigger_transfer(void) {
    nrfx_pwm_simple_playback(&pwm_sipo_instance, &seq_sipo, 1, NRFX_PWM_FLAG_STOP | NRFX_PWM_FLAG_START_VIA_TASK);
    nrf_ppi_group_enable(NRF_PPI, NRF_PPI_CHANNEL_GROUP0);
}

void clk_1mhz_trigger_transfer(void) {
        nrfx_pwm_simple_playback(&pwm_clk_1mhz_instance, &seq_clk_1mhz, 1, NRFX_PWM_FLAG_LOOP);
}

int main(void)
{
        pwm_clk_1mhz_init(); 
        pwm_sipo_init(); 
        ppi_init();

        // // configure pins 
        // nrf_gpio_cfg_output(DAC_SDA_PIN); 
        // nrf_gpio_cfg_output(DAC_SCL_PIN);

        // dac_reset(); 

        // uint16_t sipo_test_data[8] = {
        //       0xAAAC, 0xAAAA, 0x5555, 0xAAAA, 
        //       0x5555, 0xAAAA, 0x5555, 0xAAAA  
        // }; 

        //sipo_prepare_buffer(sipo_test_data); 

        // uint16_t dac2 = 0x1234; 
        // uint16_t dac1 = 0x5678; 
        // uint16_t dac0 = 0x9000;

        sipo_prepare_buffer(); 

        clk_1mhz_trigger_transfer(); 

        while (1) {
                sipo_done = false; 
                dac_i2c_done = false; 

                //dac_i2c_write(dac2, dac1, dac0); 
                
                sipo_trigger_transfer(); 

                while (!sipo_done) {
                        k_yield(); 
                }

                k_msleep(500); 
        }

}
