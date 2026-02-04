#include "final_patch.h" 
#include "inputs.h"
#include "outputs.h"
#include "rtc.h" 

LOG_MODULE_REGISTER(final_patch, LOG_LEVEL_INF);

// hardware instances 
const nrfx_timer_t timer_sramout_inst = NRFX_TIMER_INSTANCE(TIMER_SRAMOUT_INST_IDX); 
const nrfx_timer_t timer_dataout_inst = NRFX_TIMER_INSTANCE(TIMER_DATAOUT_INST_IDX);
const nrfx_pwm_t pwm_sipo_instance = NRFX_PWM_INSTANCE(PWM_SIPO_INST_IDX);
const nrfx_pwm_t pwm_clk_base_instance = NRFX_PWM_INSTANCE(PWM_CLK_BASE_INST_IDX);
const nrfx_pwm_t pwm_clk_1mhz_instance = NRFX_PWM_INSTANCE(PWM_CLK_1MHZ_INST_IDX); 

// global variables 
volatile uint32_t sramout_buffer[4]; 
volatile uint32_t dataout_buffer = 0; 
volatile int sramout_count = 0;
volatile int dataout_count = 0; 
 
nrf_pwm_values_grouped_t sipo_buffer[SIPO_DATA_LEN]; 
nrf_pwm_values_common_t clk_base_value = 3200;
nrf_pwm_values_common_t clk_1mhz_value = 8 | 0x8000;
volatile uint32_t rtc_overflow_count = 0;
volatile bool sipo_done = false; 
volatile bool dac_i2c_done = false;

volatile uint32_t ble_al_counter = 0;
volatile uint32_t ble_dl_counter = 0;
volatile uint8_t done_wr_sipo = 0;
volatile uint8_t done_wr_dac = 0;

struct k_work sramout_work;
struct k_work dataout_work;

// pwm sequences 
nrf_pwm_sequence_t seq_sipo =
{
    .values.p_grouped = sipo_buffer,
    .length           = SIPO_DATA_LEN * 2, 
    .repeats          = 0,
    .end_delay        = 0
}; 

nrf_pwm_sequence_t seq_clk_base =
{
    .values.p_common = &clk_base_value,
    .length           = 1, 
    .repeats          = 0,
    .end_delay        = 0
};

nrf_pwm_sequence_t seq_clk_1mhz =
{
    .values.p_common = &clk_1mhz_value,
    .length           = 1, 
    .repeats          = 0,
    .end_delay        = 0
}; 

void sramout_work_handler(struct k_work *work)
{
    // All the bit-shifting and BLE notification logic we wrote earlier
    int count = sramout_count;
    
    process_sramout(sramout_buffer, count); 
}

void dataout_work_handler(struct k_work *work)
{   
    uint64_t ts = get_current_timestamp(); 
    uint32_t val = dataout_buffer;
    int count = dataout_count;

    process_dataout(val, count, ts);  
}


void initialize_patch(void) {

        // pin initalizations 
        nrf_gpio_pin_clear(CH_RST);
        nrf_gpio_cfg_output(CH_RST);

        nrf_gpio_cfg_input(SRAMOUT_DATA_IN, NRF_GPIO_PIN_NOPULL); 
        nrf_gpio_cfg_input(SRAMOUT_CLK_IN, NRF_GPIO_PIN_NOPULL);

        nrf_gpio_cfg_input(DATAOUT_DATA_IN, NRF_GPIO_PIN_NOPULL); 
        nrf_gpio_cfg_input(DATAOUT_CLK_IN, NRF_GPIO_PIN_NOPULL);

        // inputs 
        gpiote_inputs_init(); 
        timer_timeout_init(); 
        timer_counter_init();
        ppi_inputs_init(); 

        // outputs 
        pwm_clk_base_init();
        pwm_clk_1mhz_init(); 
        pwm_sipo_init(); 
        ppi_outputs_init(); 
        dac_init(); 

        // trigger global clocks 
        clk_base_trigger_transfer();
        k_usleep(100);
        clk_1mhz_trigger_transfer(); 

        // initialize workers 
        k_work_init(&sramout_work, sramout_work_handler);
        k_work_init(&dataout_work, dataout_work_handler);

        // initialize bluetooth 
        int err;
        err = bt_enable(NULL); 
        if (err) {
            LOG_ERR("Bluetooth init failed \n"); 
            return; 
        } 
        LOG_INF("BT success");

        // start advertising 
        // err = bt_le_adv_start(BT_LE_ADV_CONN, NULL, 0, NULL, 0); 
        patch_ad_start();  
        LOG_INF("Advertising successfully started");

}


int main(void)
{
        LOG_INF("Start Patch \n"); 
        
        // initialize patch 
        initialize_patch(); 

        // patch is idle 
        while (1) {
                k_sleep(K_MSEC(5000));
        }
}
