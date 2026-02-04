/**
 * @file outputs.c
 * @author Archie Lee
 * @brief Handles data interface between nrf52840 inputs and ASIC. 
 * @details Implements SIPO, DAC I2C outputs, Base clock, 1MHz clock, and Chip reset.  
 * @version 1.0
 * @date 2026-01-23
 */

/* Includes ------------------------------------------------------------------*/
#include "final_patch.h" 

LOG_MODULE_DECLARE(final_patch, LOG_LEVEL_INF);

/* Interrupt Handlers ------------------------------------------------------------------*/ 
static void pwm_sipo_handler(nrfx_pwm_evt_type_t event_type, void * p_context) {
    if (event_type == NRFX_PWM_EVT_FINISHED) {
        sipo_done = true; 
    }
}

/* Peripheral Initializations ------------------------------------------------------------------*/
void pwm_sipo_init(void) {
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
        NRFX_ASSERT(status == NRFX_SUCCESS);

        nrf_pwm_int_enable(NRF_PWM1, NRF_PWM_INT_SEQEND0_MASK | NRF_PWM_INT_STOPPED_MASK);

        nrf_pwm_shorts_set(NRF_PWM1, NRF_PWM_SHORT_SEQEND0_STOP_MASK);
} 

void pwm_clk_1mhz_init(void) {
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

void pwm_clk_base_init(void) {
        nrfx_err_t status;
        (void)status;

        // sipo config 
        nrfx_pwm_config_t pwm_clk_base_config = NRFX_PWM_DEFAULT_CONFIG(CLK_BASE_OUT, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
        pwm_clk_base_config.base_clock = NRF_PWM_CLK_16MHz; 
        pwm_clk_base_config.top_value = 6400;  // 4176
        pwm_clk_base_config.load_mode = NRF_PWM_LOAD_COMMON;

        status = nrfx_pwm_init(&pwm_clk_base_instance, &pwm_clk_base_config, NULL, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);
    
}

void ppi_outputs_init(void) {

    nrf_gpio_cfg(
                CLK_BASE_OUT,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_CONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_S0S1,
                NRF_GPIO_PIN_NOSENSE);

    nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_BASE_CLK_CH, CLK_BASE_OUT, NRF_GPIOTE_POLARITY_LOTOHI);

    nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_SIPO_CH, 
        nrf_gpiote_event_address_get(NRF_GPIOTE, NRF_GPIOTE_EVENT_IN_0), 
        nrf_pwm_task_address_get(NRF_PWM1, NRF_PWM_TASK_SEQSTART1)
    ); 

    nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_SIPO_CH, 
        (uint32_t)&NRF_PPI->TASKS_CHG[PPI_SIPO_CH].DIS);

    nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CLK_1MHZ_CH, 
        nrf_gpiote_event_address_get(NRF_GPIOTE, NRF_GPIOTE_EVENT_IN_0), 
        nrf_pwm_task_address_get(NRF_PWM2, NRF_PWM_TASK_SEQSTART1)
    ); 

    nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_CLK_1MHZ_CH, 
        (uint32_t)&NRF_PPI->TASKS_CHG[PPI_CLK_1MHZ_CH].DIS);

    nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_BASE_CLK_CH);
    
}

/* Output Initializations ------------------------------------------------------------------*/
void dac_init() {
    nrf_gpio_cfg_output(DAC_SDA_PIN); 
    nrf_gpio_cfg_output(DAC_SCL_PIN);
    __DSB();
    nrf_gpio_pin_set(DAC_SDA_PIN); 
    __DSB(); 
    k_busy_wait(5); 
    nrf_gpio_pin_set(DAC_SCL_PIN);
    dac_i2c_done = false;
    done_wr_dac = 0;
}

/* Output Resets ------------------------------------------------------------------*/ 
void dac_reset() {    
    nrf_gpio_pin_set(DAC_SDA_PIN); 
    __DSB(); 
    k_busy_wait(5); 
    nrf_gpio_pin_set(DAC_SCL_PIN);
    dac_i2c_done = false; 
    done_wr_dac = 0;
}


/* Outputs Transfer Functions ------------------------------------------------------------------*/
void dac_i2c_write(const uint8_t *data, uint16_t len) 
{   
    LOG_INF("DAC transfer");
    if (len < 5) {
        LOG_ERR("DAC data too short"); 
        return; 
    }
    LOG_INF("DAC transfer start"); 
    // 1. Concatenate data exactly like: {dac2, dac1, dac0[15:12]}
    // We use a 64-bit int to hold the 36-bit result comfortably
    uint64_t dac_main = 0;

    dac_main = ((uint64_t)data[0] << 28) |
               ((uint64_t)data[1] << 20) |
               ((uint64_t)data[2] << 12) |
               ((uint64_t)data[3] << 4)  |
               ((uint64_t)(data[4] >> 4) & 0x0F);

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

/* sipo functions */
void prepare_sipo_buffer(const uint8_t *sipo_data) {
        //uint32_t data_value = 0x88888; 

        // uint8_t data_value[16] = {
        //     0xAA, 0xAA, 0xAA, 0xAA, // Bytes 0-3  (101010...)
        //     0x55, 0x55, 0x55, 0x55, // Bytes 4-7  (010101...)
        //     0xFF, 0xFF, 0x00, 0x00, // Bytes 8-11 (Solid High then Solid Low)
        //     0x12, 0x34, 0x56, 0x78  // Bytes 12-15 (Random Pattern)
        // };

        for (uint8_t i = 0; i < SIPO_DATA_LEN; i++) {
                // Extract bit from MSB down to LSB
                //bool bit = (data_value >> (20 - 1 - i)) & 0x01;

                // 1. Calculate which byte we are in (0 to 15)
                int byte_idx = i / 8;
                
                // 2. Calculate which bit inside that byte (7 down to 0 for MSB-first)
                int bit_idx = 7 - (i % 8);

                // 3. Extract the bit
                bool bit = (sipo_data[byte_idx] >> bit_idx) & 0x01;

                /// Data:
                if (!bit) {
                        // Force "Always High" by setting duty cycle > top_value
                        // This prevents the PWM counter from ever "resetting" the pin
                        sipo_buffer[i].group_1 = 0x8000; 
                } else {
                        sipo_buffer[i].group_1 = 0;  
                }

                sipo_buffer[i].group_0 = 16;
        }
}

void sipo_trigger_transfer(void) {
    nrf_pwm_task_trigger(NRF_PWM1, NRF_PWM_TASK_STOP);
    k_busy_wait(10);

    nrfx_pwm_simple_playback(&pwm_sipo_instance, &seq_sipo, 1, NRFX_PWM_FLAG_STOP | NRFX_PWM_FLAG_START_VIA_TASK);
    nrf_ppi_channel_enable(NRF_PPI, PPI_SIPO_CH);
    LOG_INF("PPI channel enabled"); 
}

/* start global clock */
void clk_1mhz_trigger_transfer(void) {
        nrfx_pwm_simple_playback(&pwm_clk_1mhz_instance, &seq_clk_1mhz, 1, NRFX_PWM_FLAG_LOOP | NRFX_PWM_FLAG_START_VIA_TASK);
        nrf_ppi_channel_enable(NRF_PPI, PPI_CLK_1MHZ_CH);
}

void clk_base_trigger_transfer(void) {
        nrfx_pwm_simple_playback(&pwm_clk_base_instance, &seq_clk_base, 1, NRFX_PWM_FLAG_LOOP);
}

void chip_reset(void) {
    unsigned int ch_rst_key = irq_lock();

    nrf_gpio_pin_clear(CH_RST);
    nrf_gpio_pin_set(CH_RST); 

    // high for 1kHz
    k_busy_wait(1000);

    nrf_gpio_pin_clear(CH_RST);

    irq_unlock(ch_rst_key);
}
