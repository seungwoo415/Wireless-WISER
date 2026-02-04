#include <zephyr/kernel.h>

#include <nrfx_gpiote.h> 
#include <nrfx_timer.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
// #include <zephyr/sys/printk.h> 
#include <nrfx_spim.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(nrfx_sample, LOG_LEVEL_INF);

// pin configurations 
#define CLK_IN 27
#define DATA_IN 26

#define CLK_OUT 14
#define DATA_OUT 13

// hardware index 
#define GPIOTE_INST_IDX 0
#define TIMER_TIMEOUT_INST_IDX 1 
#define SPIM_INST_IDX 1

// hardware instances 
#define GPIOTE_NODE DT_NODELABEL(gpiote0)
static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(GPIOTE_INST_IDX);

static const nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_TIMEOUT_INST_IDX); 

static const nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

// channels 
static uint8_t gpiote_channel; 
#define GPIOTE_CH 0
#define PPI_CH NRF_PPI_CHANNEL0 

// pin mask 
#define DATA_PIN_MASK (1 << DATA_IN)

// global variables 
volatile uint32_t rx_buffer = 0; 
volatile int bit_count = 0; 
volatile bool data_ready = false; 
volatile bool spim_done = false; 

// data buffers 
static uint8_t tx_buf[4] = {
    0x88, 0x88, 0xAA, 0xAA // 10001000 8 bits 
}; 

// https://docs.zephyrproject.org/latest/kernel/services/interrupts.html 
ISR_DIRECT_DECLARE(gpiote_fast_handler)
{
    // clear the event manually in a Direct ISR
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    // optimized LSB-first bit placement 
    if (NRF_P0->IN & (1 << DATA_IN)) {
        rx_buffer |= (1 << bit_count);
    }

    bit_count++;

    return 0; 
}

void timer_handler(nrf_timer_event_t event_type, void * p_context) {
        if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // packet is finished 
                if (bit_count > 0) {
                data_ready = true;
                }
        }
}

static void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context){
        if (p_event->type == NRFX_SPIM_EVENT_DONE)
        {
                spim_done = true; 
        }
} 

static void spim_init(void) {
        nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG(CLK_OUT, DATA_OUT, NRF_SPIM_PIN_NOT_CONNECTED, NRF_SPIM_PIN_NOT_CONNECTED);

        cfg.frequency = NRFX_MHZ_TO_HZ(1);       // 1 MHz
        cfg.mode      = NRF_SPIM_MODE_0;         // CPOL=0, CPHA=0
        cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

        nrfx_err_t err = nrfx_spim_init(&spim_inst, &cfg, spim_handler, NULL);
        if (err != NRFX_SUCCESS) {
                 LOG_INF("SPIM init failed");
        }
        LOG_INF("SPIM initialized");

}

static void gpiote_init(void) {

        nrfx_gpiote_init(&gpiote_inst, 0);

        // nrfx_gpiote_channel_alloc(&gpiote_inst, &gpiote_channel); 
        
        // static nrfx_gpiote_trigger_config_t trig_config = {
        //         .trigger = NRFX_GPIOTE_TRIGGER_LOTOHI,
        //         .p_in_channel = &gpiote_channel
        // };

        // static nrfx_gpiote_handler_config_t hndl_config = {
        //         .handler = gpiote_handler
        // };

        // static const nrfx_gpiote_input_pin_config_t config =
        // {
        //         .p_pull_config    = NULL,
        //         .p_trigger_config = &trig_config,
        //         .p_handler_config = &hndl_config
        // };

        // nrfx_gpiote_input_configure(&gpiote_inst, CLK_IN, &config); 
        // nrfx_gpiote_trigger_enable(&gpiote_inst, CLK_IN, true);

        NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)   |
                            (CLK_IN                        << GPIOTE_CONFIG_PSEL_Pos)   |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;

        nrf_gpio_cfg_input(DATA_IN, NRF_GPIO_PIN_NOPULL);
} 

static void timer_init(void) {
        // timeout 
        uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
        nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
        config.bit_width = NRF_TIMER_BIT_WIDTH_32;
        
        nrfx_err_t err = nrfx_timer_init(&timer_inst, &config, timer_handler); 
        if (err != NRFX_SUCCESS) {
                LOG_INF("Timer init failed");
        }
        LOG_INF("Timer init succeeded");
        
        nrfx_timer_clear(&timer_inst);

        uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, 5000000); // 5s
        nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

        // turn timer off initially 
        nrf_timer_task_trigger(timer_inst.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(timer_inst.p_reg, NRF_TIMER_TASK_CLEAR);
} 

static void ppi_init(void) {
        // low to high --> start timer 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[0], 
                nrfx_timer_task_address_get(&timer_inst, NRF_TIMER_TASK_CLEAR)
        ); 

        // low to high --> clear timer 
        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_CH, nrfx_timer_task_address_get(&timer_inst, NRF_TIMER_TASK_START)); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_CH); 
}

int main(void)
{       
        // enable instruction cache
        NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;

        IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, gpiote_fast_handler, 0);
        irq_enable(GPIOTE_IRQn);

        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX),
        0,
        0
        );

        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_TIMEOUT_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_TIMEOUT_INST_IDX), 0, 0);

        // initialize hardware 
        gpiote_init(); 

        timer_init(); 

        ppi_init(); 

        spim_init(); 

        // xfer start 
        nrfx_spim_xfer_desc_t xfer = {
                .p_tx_buffer = tx_buf,
                .tx_length   = sizeof(tx_buf),
                .p_rx_buffer = NULL,
                .rx_length   = 0
        };

        spim_done = false;

        nrfx_err_t err = nrfx_spim_xfer(&spim_inst, &xfer, 0);
        if (err != NRFX_SUCCESS) {
                LOG_INF("SPIM xfer failed"); 
        }
        LOG_INF("SPIM xfer succeeded");

        while(!spim_done) {k_usleep(10);}

        // idle 
        while (!data_ready){ 
               k_msleep(1);  
        }
        LOG_INF("Packet: 0x%08X (Bits: %d)\n", rx_buffer, bit_count);

}
