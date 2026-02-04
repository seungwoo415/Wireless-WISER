#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <zephyr/irq.h>
#include <nrfx_pwm.h>
#include <zephyr/logging/log.h>

#include <nrfx_gpiote.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>

// logging 
LOG_MODULE_REGISTER(spect, LOG_LEVEL_INF);
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3

// pins 
#define DAC_SDA_PIN 12
#define DAC_SCL_PIN 11
#define SIPO_CLK_OUT 6
#define SIPO_DATA_OUT 26 
#define CLK_1MHZ_OUT 27
#define CLK_BASE_OUT 14

// hardware indicies 
#define PWM_SIPO_INST_IDX 1
#define PWM_CLK_1MHZ_INST_IDX 2
#define PWM_CLK_BASE_INST_IDX 3

// channels 
#define GPIOTE_SIPO_CH 0
#define PPI_SIPO_CH 0
#define PPI_SIPO_GROUP NRF_PPI_CHANNEL_GROUP1

// hardware instances 
nrfx_pwm_t pwm_sipo_instance = NRFX_PWM_INSTANCE(PWM_SIPO_INST_IDX);
nrfx_pwm_t pwm_clk_1mhz_instance = NRFX_PWM_INSTANCE(PWM_CLK_1MHZ_INST_IDX);
nrfx_pwm_t pwm_clk_base_instance = NRFX_PWM_INSTANCE(PWM_CLK_BASE_INST_IDX);

// buffer 
static nrf_pwm_values_grouped_t sipo_buffer[126];
static nrf_pwm_values_common_t clk_1mhz_value = 8;
static nrf_pwm_values_common_t clk_base_value = 400;

// pwm sequences 
static nrf_pwm_sequence_t seq_sipo =
{
    .values.p_grouped = sipo_buffer,
    .length           = 126 * 2, 
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

static nrf_pwm_sequence_t seq_clk_base =
{
    .values.p_common = &clk_base_value,
    .length           = 1, 
    .repeats          = 0,
    .end_delay        = 0
};

// global variables 
static volatile bool sipo_done = false; 
static volatile bool dac_i2c_done = false; 

//interrupt handlers 
static void pwm_sipo_handler(nrfx_pwm_evt_type_t event_type, void * p_context) {
    if (event_type == NRFX_PWM_EVT_FINISHED) {
        nrf_ppi_group_disable(NRF_PPI, PPI_SIPO_GROUP);
        //nrf_ppi_channel_disable(NRF_PPI, PPI_SIPO_CH);
        //nrf_ppi_channel_disable(NRF_PPI, 1);
        nrf_gpiote_event_disable(NRF_GPIOTE, GPIOTE_SIPO_CH);
        sipo_done = true;
    }
}

static void pwm_sipo_init(void) {
        nrfx_err_t status;
        (void)status; 

        // sipo config 
        nrfx_pwm_config_t pwm_sipo_config = NRFX_PWM_DEFAULT_CONFIG(SIPO_CLK_OUT, NRF_PWM_PIN_NOT_CONNECTED, SIPO_DATA_OUT, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_sipo_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_sipo_config.top_value = 32; 
        pwm_sipo_config.load_mode = NRF_PWM_LOAD_GROUPED;

        // enable sipo pwm interupt 
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_SIPO_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_SIPO_INST_IDX), 0, 0);


        status = nrfx_pwm_init(&pwm_sipo_instance, &pwm_sipo_config, pwm_sipo_handler, NULL);
        if (status != NRFX_SUCCESS) LOG_ERR("PWM SIPO Init FAILED with error: %d", status);


} 

static void pwm_clk_1mhz_init(void) {
        nrfx_err_t status;
        (void)status; 

        // sipo config 
        nrfx_pwm_config_t pwm_clk_1mhz_config = NRFX_PWM_DEFAULT_CONFIG(CLK_1MHZ_OUT, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_clk_1mhz_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_clk_1mhz_config.top_value = 16; 
        pwm_clk_1mhz_config.load_mode = NRF_PWM_LOAD_COMMON;

        // enable sipo pwm interupt 
        //IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_CLK_1MHZ_INST_IDX)), IRQ_PRIO_LOWEST,
        //        NRFX_PWM_INST_HANDLER_GET(PWM_CLK_1MHZ_INST_IDX), 0, 0);


        status = nrfx_pwm_init(&pwm_clk_1mhz_instance, &pwm_clk_1mhz_config, NULL, NULL);
        if (status != NRFX_SUCCESS) LOG_ERR("PWM 1MHz Init FAILED with error: %d", status);

}

// 100 khz
static void pwm_clk_base_init(void) {
        nrfx_err_t status;
        (void)status; 

        // base config 
        nrfx_pwm_config_t pwm_clk_base_config = NRFX_PWM_DEFAULT_CONFIG(CLK_BASE_OUT, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_clk_base_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_clk_base_config.top_value = 800; 
        pwm_clk_base_config.load_mode = NRF_PWM_LOAD_COMMON;

        // enable sipo pwm interupt 
        //IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_CLK_1MHZ_INST_IDX)), IRQ_PRIO_LOWEST,
        //        NRFX_PWM_INST_HANDLER_GET(PWM_CLK_1MHZ_INST_IDX), 0, 0);


        status = nrfx_pwm_init(&pwm_clk_base_instance, &pwm_clk_base_config, NULL, NULL);
        if (status != NRFX_SUCCESS) LOG_ERR("PWM Base Init FAILED with error: %d", status);

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

void sipo_prepare_buffer(uint16_t sipo[8]) {
    int buf_idx = 0;

    for (int s = 7; s >= 0; s--) {
        int bits_to_send = (s == 0) ? 14 : 16; // sipo0 only sends 14 bits [15:2]
        uint16_t current_val = sipo[s];

        for (int i = 15; i >= (16 - bits_to_send); i--) {
            if (buf_idx >= 126) break;

            bool bit = (current_val >> i) & 0x01;

            // Group 1: DATA PIN
            // 0x8000 sets the 15th bit (Inverted polarity), ensuring pin stays HIGH or LOW
            sipo_buffer[buf_idx].group_1 = bit ? 0 : 0x8000;

            // Group 0: CLOCK PIN
            // 50% duty cycle (16 out of 32)
            sipo_buffer[buf_idx].group_0 = 16;

            buf_idx++;
        }
    }
} 

void sipo_sync_ppi_init(void){ 
        // configure 1mhz clock output pin first 
        nrf_gpio_cfg(
                CLK_BASE_OUT,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_CONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_S0S1,
                NRF_GPIO_PIN_NOSENSE); 

        // connect gpiote event with pwm task and disable 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, 2, 
                nrfx_pwm_event_address_get(&pwm_clk_base_instance, NRF_PWM_EVENT_SEQSTARTED0), 
                nrfx_pwm_task_address_get(&pwm_clk_1mhz_instance, NRF_PWM_TASK_SEQSTART0)
        ); 

        
        // HITOLO because pwm_sipo begins on a negative edge        
        nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_SIPO_CH, CLK_BASE_OUT, NRF_GPIOTE_POLARITY_HITOLO);

        // initialize ppi group
        nrf_ppi_group_clear(NRF_PPI, PPI_SIPO_GROUP);
        nrf_ppi_channel_include_in_group(NRF_PPI, PPI_SIPO_CH, PPI_SIPO_GROUP);

        // connect gpiote event with pwm task and disable 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_SIPO_CH, 
                nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_SIPO_CH)), 
                nrfx_pwm_task_address_get(&pwm_sipo_instance, NRF_PWM_TASK_SEQSTART0)
        ); 

        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_SIPO_CH,  
                nrf_ppi_task_group_disable_address_get(NRF_PPI, PPI_SIPO_GROUP)
        );

        // nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_SIPO_CH, 
        //         nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_SIPO_CH)), 
        //         nrf_ppi_task_group_disable_address_get(NRF_PPI, PPI_SIPO_GROUP)
        // );     


        // nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_SIPO_CH,  
        //         nrf_ppi_task_group_disable_address_get(NRF_PPI, PPI_SIPO_GROUP)
        // );



        // nrf_ppi_channel_endpoint_setup(NRF_PPI, 3, 
        //         nrfx_pwm_event_address_get(&pwm_clk_1mhz_instance, NRF_PWM_EVENT_SEQEND0), 
        //         nrf_ppi_task_group_disable_address_get(NRF_PPI, PPI_1MHZ_GROUP)
        // );

        // nrf_ppi_channel_enable(NRF_PPI, 3);
        

}

void sipo_trigger_transfer(void) {
    //nrf_ppi_channel_enable(NRF_PPI, PPI_SIPO_CH);
    //nrf_ppi_channel_enable(NRF_PPI, 1);
    nrf_gpiote_event_clear(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_SIPO_CH));

    nrfx_pwm_simple_playback(&pwm_sipo_instance, &seq_sipo, 1, NRFX_PWM_FLAG_START_VIA_TASK | NRFX_PWM_FLAG_STOP);

    nrf_ppi_group_enable(NRF_PPI, PPI_SIPO_GROUP);
    //nrf_ppi_channel_enable(NRF_PPI, PPI_SIPO_CH);
    nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_SIPO_CH);
    
}

void clk_1mhz_trigger_transfer(void) {
        nrf_ppi_channel_enable(NRF_PPI, 2);
}

void clk_base_trigger_transfer(void) {
        nrfx_pwm_simple_playback(&pwm_clk_base_instance, &seq_clk_base, 1, NRFX_PWM_FLAG_LOOP);
}

int main(void)
{
        LOG_INF("START");
        pwm_clk_1mhz_init(); 
        LOG_INF("init 1 MHz clock");
        pwm_clk_base_init();
        LOG_INF("init base MHz clock");
        pwm_sipo_init(); 
        LOG_INF("init sipo MHz clock");
        sipo_sync_ppi_init(); 

        LOG_INF("prepare 1 MHz clock");
        clk_1mhz_trigger_transfer();

        LOG_INF("start global clock");
        clk_base_trigger_transfer(); 
        

        // configure pins 
        nrf_gpio_cfg_output(DAC_SDA_PIN); 
        nrf_gpio_cfg_output(DAC_SCL_PIN);

        dac_reset(); 

        uint16_t sipo_test_data[8] = {
              0xAAAC, 0xAAAA, 0x5555, 0xAAAA, 
              0x5555, 0xAAAA, 0x5555, 0xAAAA  
        }; 

        sipo_prepare_buffer(sipo_test_data); 

        uint16_t dac2 = 0x1234; 
        uint16_t dac1 = 0x5678; 
        uint16_t dac0 = 0x9000; 

        //clk_1mhz_trigger_transfer(); 

        while (1) {
                sipo_done = false; 
                dac_i2c_done = false; 

                dac_i2c_write(dac2, dac1, dac0); 
                
                sipo_trigger_transfer(); 

                while (!sipo_done) {
                        k_yield(); 
                }
                LOG_INF("SIPO Done"); 

                k_msleep(1); 
        }


        

}
