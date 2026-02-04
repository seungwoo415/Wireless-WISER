#include <zephyr/kernel.h>

#include <nrfx_gpiote.h> 
#include <nrfx_timer.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
#include <nrfx_spim.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <hal/nrf_egu.h>
#include "test.h" 
//#include "test_spim.h"
//#include <nrfx_pwm.h>
LOG_MODULE_REGISTER(inputs, LOG_LEVEL_INF);

// pin configurations 
#define SRAMOUT_CLK_IN 27
#define SRAMOUT_DATA_IN 26
#define DATAOUT_CLK_IN 14
#define DATAOUT_DATA_IN 13

// hardware index 
#define GPIOTE_INST_IDX 0
#define TIMER_SRAMOUT_INST_IDX 0
#define TIMER_DATAOUT_INST_IDX 1

// hardware instances 
//#define GPIOTE_NODE DT_NODELABEL(gpiote0)
//static nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(GPIOTE_INST_IDX);

static const nrfx_timer_t timer_sramout_inst = NRFX_TIMER_INSTANCE(TIMER_SRAMOUT_INST_IDX); 
static const nrfx_timer_t timer_dataout_inst = NRFX_TIMER_INSTANCE(TIMER_DATAOUT_INST_IDX);

// channels 
#define GPIOTE_SRAMOUT_CH 1
#define GPIOTE_DATAOUT_CH 2
#define PPI_SRAMOUT_CH NRF_PPI_CHANNEL0 
#define PPI_DATAOUT_CH NRF_PPI_CHANNEL1
#define PPI_EGU_CH NRF_PPI_CHANNEL2

// pin mask 
#define SRAMOUT_PIN_MASK (1 << SRAMOUT_DATA_IN)
#define DATAOUT_PIN_MASK (1 << DATAOUT_DATA_IN)

// global variables 
//volatile uint32_t sramout_buffer = 0; 
volatile uint32_t sramout_buffer[4];
volatile uint32_t dataout_buffer = 0; 
volatile int sramout_count = 0;
volatile int dataout_count = 0; 
volatile bool sramout_ready = false; 
volatile bool dataout_ready = false;

// https://docs.zephyrproject.org/latest/kernel/services/interrupts.html 
// __attribute__((section(".ramfunc")))
// ISR_DIRECT_DECLARE(gpiote_fast_handler)
// {
//     NRF_GPIOTE->EVENTS_IN[1] = 0;

//     if (NRF_P0->IN & (1 << SRAMOUT_DATA_IN)) {
//         sramout_buffer |= (1 << sramout_count);
//     }
//     sramout_count++;

//     return 0;
// }

uint32_t b[4];
volatile uint32_t *capture_ptr; 
volatile uint32_t raw_capture[113];

__attribute__((section(".ramfunc")))
ISR_DIRECT_DECLARE(gpiote_fast_handler)
{
//     NRF_GPIOTE->EVENTS_IN[1] = 0;
//     uint32_t pin_state = NRF_P0->IN;

//     if (sramout_count < 64) {
//         if ((pin_state >> SRAMOUT_DATA_IN) & 1) {
//             part1 |= ((uint64_t)1 << sramout_count);
//         }
//     } else if (sramout_count < 128) {
//         if ((pin_state >> SRAMOUT_DATA_IN) & 1) {
//             part2 |= ((uint64_t)1 << (sramout_count - 64));
//         }
//     }

//     sramout_count++;
//     return 0;

        // 1. SNAPSHOT FIRST (Cycle 0)
        uint32_t val = NRF_P0->IN; 

        // 2. Clear hardware event
        NRF_GPIOTE->EVENTS_IN[1] = 0;

        // 3. Simple Store (The CPU loves this)
        *capture_ptr++ = val;
        sramout_count++;

        __DSB();
        return 1;

}

__attribute__((section(".ramfunc")))
ISR_DIRECT_DECLARE(egu_fast_handler) {
    NRF_GPIOTE->EVENTS_IN[2] = 0;
    NRF_EGU0->EVENTS_TRIGGERED[0] = 0;

    if (NRF_P0->IN & (1 << DATAOUT_DATA_IN)) {
        dataout_buffer |= (1 << dataout_count);
    }
    dataout_count++;
    return 0;
}

void timer_sramout_handler(nrf_timer_event_t event_type, void * p_context) {
        if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // packet is finished 
                if (sramout_count > 0) {
                sramout_ready = true;
                }
        }
}

void timer_dataout_handler(nrf_timer_event_t event_type, void * p_context) {
        if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // packet is finished 
                if (dataout_count > 0) {
                dataout_ready = true;
                }
        }
}

static void gpiote_init(void) {

        // enable instruction cache
        NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;

        IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, gpiote_fast_handler, IRQ_ZERO_LATENCY);
        irq_enable(GPIOTE_IRQn);

        IRQ_DIRECT_CONNECT(SWI0_EGU0_IRQn, 0, egu_fast_handler, IRQ_ZERO_LATENCY);
        irq_enable(SWI0_EGU0_IRQn);

        // sramout channel 
        NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)   |
                            (SRAMOUT_CLK_IN                        << GPIOTE_CONFIG_PSEL_Pos)   |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk;

        nrf_gpio_cfg_input(SRAMOUT_DATA_IN, NRF_GPIO_PIN_NOPULL);

        // egu for dataout 
        NRF_EGU0->EVENTS_TRIGGERED[0] = 0;
        NRF_EGU0->INTENSET = EGU_INTENSET_TRIGGERED0_Msk;

        // dataout channel 
        NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)   |
                            (DATAOUT_CLK_IN                        << GPIOTE_CONFIG_PSEL_Pos)   |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
        
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN2_Msk;

        nrf_gpio_cfg_input(DATAOUT_DATA_IN, NRF_GPIO_PIN_NOPULL);

        // connect dataout with egu 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_EGU_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[2], 
                (uint32_t)&NRF_EGU0->TASKS_TRIGGER[0]
        ); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_EGU_CH);

} 

static void timer_init(void) {
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_SRAMOUT_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_SRAMOUT_INST_IDX), 0, 0);
        
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_DATAOUT_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_DATAOUT_INST_IDX), 0, 0);

        // sramout timeout 
        uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_sramout_inst.p_reg);
        nrfx_timer_config_t sramout_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
        sramout_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
        
        nrfx_err_t err = nrfx_timer_init(&timer_sramout_inst, &sramout_config, timer_sramout_handler); 
        if (err != NRFX_SUCCESS) {
                LOG_INF("sramout timer init failed");
        }
        LOG_INF("sramout timer init succeeded");
        
        nrfx_timer_clear(&timer_sramout_inst);

        uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_sramout_inst, 5000000); // 1s
        nrfx_timer_extended_compare(&timer_sramout_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

        // turn timer off initially 
        nrf_timer_task_trigger(timer_sramout_inst.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(timer_sramout_inst.p_reg, NRF_TIMER_TASK_CLEAR);

        // dataout timeout 
        base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_dataout_inst.p_reg);
        nrfx_timer_config_t dataout_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
        dataout_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
        
        err = nrfx_timer_init(&timer_dataout_inst, &dataout_config, timer_dataout_handler); 
        if (err != NRFX_SUCCESS) {
                LOG_INF("dataout timer init failed");
        }
        LOG_INF("dataout timer init succeeded");
        
        nrfx_timer_clear(&timer_dataout_inst);

        desired_ticks = nrfx_timer_us_to_ticks(&timer_dataout_inst, 9000000); // 1s
        nrfx_timer_extended_compare(&timer_dataout_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

        // turn timer off initially 
        nrf_timer_task_trigger(timer_dataout_inst.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(timer_dataout_inst.p_reg, NRF_TIMER_TASK_CLEAR);
} 

static void ppi_init(void) {
        // low to high --> start timer 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_SRAMOUT_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[1], 
                nrfx_timer_task_address_get(&timer_sramout_inst, NRF_TIMER_TASK_CLEAR)
        ); 

        // low to high --> clear timer 
        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_SRAMOUT_CH, nrfx_timer_task_address_get(&timer_sramout_inst, NRF_TIMER_TASK_START)); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_SRAMOUT_CH); 

        // low to high --> start timer 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_DATAOUT_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[2], 
                nrfx_timer_task_address_get(&timer_dataout_inst, NRF_TIMER_TASK_CLEAR)
        ); 

        // low to high --> clear timer 
        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_DATAOUT_CH, nrfx_timer_task_address_get(&timer_dataout_inst, NRF_TIMER_TASK_START)); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_DATAOUT_CH);
}

void process_capture_to_b4(void) {
    // 1. Clear the destination buffer first!
    memset(b, 0, sizeof(b));

    for (int i = 0; i < 113; i++) {
        // 2. Extract ONLY bit 26 from the i-th snapshot
        uint32_t bit = (raw_capture[i] >> 26) & 1;

        // 3. Pack the bit into b[4]
        // (i >> 5) selects the word: 0, 1, 2, or 3
        // (i & 31) selects the bit position inside that word
        if (bit) {
            b[i >> 5] |= (1U << (i & 31));
        }
    }
}

void print_b4_hex(void) {
    LOG_INF("b[4] Data (Hex): ");
    for (int i = 0; i < 4; i++) {
        LOG_INF("0x%08X ", b[i]);
    }
}

int main(void)
{       
        // initialize hardware 
        gpiote_init(); 

        timer_init(); 

        ppi_init(); 
        
        // uint8_t my_sipo_data[16] = {
        //         0xAA, 0x55, 0xAA, 0x55, // Bytes 0-3
        //         0xAA, 0x55, 0xAA, 0x55, // Bytes 4-7
        //         0xAA, 0x55, 0xAA, 0x55, // Bytes 8-11
        //         0xAA, 0x55, 0xAA, 0x55  // Bytes 12-15
        // };

        // uint8_t my_sipo_data[16] = {
        //         0xAA, 0x55, 0xAA, 0x55, // Bytes 0-3
        //         0xAA, 0x55, 0xAA, 0x55,
        //         0xAA, 0x55, 0xAA, 0x55,
        //         0xAA, 0x55, 0xAA, 0x55
        // };

        // uint8_t my_sipo_data[4] = {
        //         0xAA, 0x55, 0xAA, 0x55 // Bytes 0-3
        // };

        uint8_t my_sipo_data[16] = {0xFF, 0x00, 0xAA, 0x88, 0xAA, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};

        capture_ptr = &raw_capture[0];

        // initialize test 
        buffer_init(my_sipo_data); 

        pwm_init();

        k_msleep(100);

        sramout_count = 0;
        nrfx_pwm_simple_playback(&pwm_instance, &seq, 1, NRFX_PWM_FLAG_STOP);

        while(!pwm_done) {k_usleep(10);}

        while (!sramout_ready){ 
               k_msleep(1);  
        }

        process_capture_to_b4(); 

        print_b4_hex(); 
        LOG_INF("Total Bits Captured: %d", sramout_count);

        //LOG_INF("sramout: 0x%08X (Bits: %d)\n", sramout_buffer, sramout_count);

}
