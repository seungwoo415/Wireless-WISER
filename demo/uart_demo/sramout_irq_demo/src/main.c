#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/sys/printk.h>
//#include <zephyr/usb/usb_device.h>
#include <hal/nrf_timer.h>
//#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <zephyr/irq.h>

#define SRAM_BITS 112
#define SRAM_BYTES (SRAM_BITS / 8) 

// define the I/O pins 
#define SRAM_CLK_IN   27
#define SRAM_DATA_IN  26
#define SRAM_CLK_OUT 14
#define SRAM_DATA_OUT 13

// gpiote channel 
#define GPIOTE_CH 0
#define GPIOTE_INST_IDX 0
#define SPIM_INST_IDX 1
#define PPI_CH NRF_PPI_CHANNEL0
#define PPI_CH1 NRF_PPI_CHANNEL1


#define GPIOTE_NODE DT_NODELABEL(gpiote0)
static nrfx_gpiote_t const gpiote_inst = NRFX_GPIOTE_INSTANCE(GPIOTE_INST_IDX);

static uint8_t in_channel;

// k_timer for timeout 
static struct k_timer clk_stop_timer;
#define CLOCK_TIMEOUT_US 2000

// data buffers 
static uint8_t tx_buf1[14] = {
    0x88, 0x88, 0x88, 0x88, // 10001000.. 112 bits 
    0x88, 0x88, 0x88, 0x88,
    0x88, 0x88, 0x88, 0x88, 
    0x88, 0x88
}; 

static uint8_t tx_buf2[1] = {
    0x8F // 10001111 8 bits 
}; 

// is the data transfer done? 
static volatile bool done_rd_SRAM = false; 

// from BLE 
static volatile bool data_rd_reset = false; 

// counter + rx buffer 
static volatile uint32_t counter_sram = 0; 
static volatile uint8_t sram_out[112];

static volatile bool capture_enable = false;
// static volatile uint32_t status; 

static const nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

static volatile bool spim_done = false; 

static uint32_t idx; 

static void clk_stop_handler(struct k_timer *timer)
{
    capture_enable = false;
    done_rd_SRAM = true;
    nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0);
    counter_sram = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0);
    //printk("timer handler"); 
} 

static void clk_timer_init(void)
{
    k_timer_init(&clk_stop_timer, clk_stop_handler, NULL);
}


static void spim_handler(nrfx_spim_evt_t const * evt, void * ctx) {
        if (evt->type == NRFX_SPIM_EVENT_DONE) {
                spim_done = true;
        }
 }

/* SPIM test */
static void spim_test_init(void) {
    // spim config
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SRAM_CLK_OUT,
                                                              SRAM_DATA_OUT,
                                                              NRF_SPIM_PIN_NOT_CONNECTED,
                                                              NRF_SPIM_PIN_NOT_CONNECTED);
    spim_config.mode      = NRF_SPIM_MODE_0;         // CPOL=0, CPHA=0
    spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spim_config.frequency = NRFX_MHZ_TO_HZ(0.25);      // 1 MHz

    // initialize spim 
    nrfx_err_t err = nrfx_spim_init(&spim_inst, &spim_config, spim_handler, NULL);
    if (err != NRFX_SUCCESS) {
        printk("SPIM init failed\n");
        return 0;
    }
    printk("SPIM initialized\n");

} 

static void spim_test_transfer1(void) {
    // spim transfer buffer information 
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(tx_buf1,
                                                              sizeof(tx_buf1),
                                                              NULL,
                                                              0);

    // send spim data 
    nrfx_err_t err = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    if (err != NRFX_SUCCESS) {
        printk("SPIM first xfer failed \n");
    }
    printk("SPIM first xfer \n");
}

static void spim_test_transfer2(void) {
    // spim transfer buffer information 
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(tx_buf2,
                                                              sizeof(tx_buf2),
                                                              NULL,
                                                              0);

    // send spim data 
    nrfx_err_t err = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    if (err != NRFX_SUCCESS) {
        printk("SPIM second xfer failed\n");
    }
    printk("SPIM second xfer");
} 

// static inline uint8_t get_bit(uint16_t idx)
// {
//     if (idx >= counter_sram) return 0;
//     return sram_out[counter_sram - 1 - idx];
// }

void gpiote_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{       
        if (!capture_enable) return;

        // nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0);
        // uint32_t idx = nrf_timer_cc_get(NRF_TIMER1,
        //                                 NRF_TIMER_CC_CHANNEL0);

        // if (idx > 0 && idx <= SRAM_BITS) {
        //         sram_out[idx - 1] = nrf_gpio_pin_read(SRAM_DATA_IN) & 1;
        // }

        sram_out[idx++] = nrf_gpio_pin_read(SRAM_DATA_IN) & 1;
        

        // restart timeout 
        k_timer_start(&clk_stop_timer, K_USEC(CLOCK_TIMEOUT_US), K_NO_WAIT);

}


/* GPIOTE */
// static void gpiote_init(void) {
//         IRQ_CONNECT(
//         NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(0)),
//         IRQ_PRIO_LOWEST,
//         gpiote_handler,
//         0,
//         0
//         ); 
        

//         nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_CH, SRAM_CLK_IN, NRF_GPIOTE_POLARITY_LOTOHI);
//         nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_CH); 

//         nrf_gpiote_int_enable(NRF_GPIOTE, NRF_GPIOTE_INT_IN0_MASK); 

//         nrf_gpio_cfg_input(SRAM_DATA_IN, NRF_GPIO_PIN_NOPULL); 
// }

static void gpiote_init(void) {

        nrfx_gpiote_init(&gpiote_inst, 0);

        nrfx_gpiote_channel_alloc(&gpiote_inst, &in_channel); 
        
        static nrfx_gpiote_trigger_config_t trig_config = {
                .trigger = NRFX_GPIOTE_TRIGGER_LOTOHI,
                .p_in_channel = &in_channel
        };

        static nrfx_gpiote_handler_config_t hndl_config = {
                .handler = gpiote_handler
        };

        static const nrfx_gpiote_input_pin_config_t config =
        {
                .p_pull_config    = NULL,
                .p_trigger_config = &trig_config,
                .p_handler_config = &hndl_config
        };

        nrfx_gpiote_input_configure(&gpiote_inst, SRAM_CLK_IN, &config); 
        nrfx_gpiote_trigger_enable(&gpiote_inst, SRAM_CLK_IN, true);

        nrf_gpio_cfg_input(SRAM_DATA_IN, NRF_GPIO_PIN_NOPULL);
}

static void timer_init(void) {
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

        nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_COUNTER); 
        nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_16);

        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);
}

static void ppi_init(void) {
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CH, 
                nrfx_gpiote_in_event_address_get(&gpiote_inst, SRAM_CLK_IN), 
                nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_COUNT)
        ); 

        // nrf_ppi_channel_endpoint_setup(
        //         NRF_PPI, PPI_CH1,
        // nrfx_gpiote_in_event_address_get(&gpiote_inst, SRAM_CLK_IN),
        // nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0)
        // );

        // nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CH_EGU,
        //         nrfx_gpiote_in_event_address_get(gpiote_inst, nrf_gpiote_in_event_get(GPIOTE_CH)),
        //         nrf_egu_task_address_get(NRF_EGU0, NRF_EGU_TASK_TRIGGER0)
        // );

        nrf_ppi_channel_enable(NRF_PPI, PPI_CH);
        // nrf_ppi_channel_enable(NRF_PPI, PPI_CH1);
        // nrf_ppi_channel_enable(NRF_PPI, PPI_CH_EGU);
}

static void sramout_reset(void) {

        capture_enable = false;
        counter_sram = 0; 
        done_rd_SRAM = false; 
        idx = 0; 
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);
        capture_enable = true;

}

int main(void)
{   
// int ret;

    /* -------- USB console -------- */
//     ret = usb_enable(NULL);
//     if (ret) {
//         return 0;
//     }
      printk("SPIM test start \n");

    IRQ_CONNECT(DT_IRQN(GPIOTE_NODE), DT_IRQ(GPIOTE_NODE, priority), nrfx_isr,
    		    NRFX_CONCAT(nrfx_gpiote_, GPIOTE_INST_IDX, _irq_handler), 0);

    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX),
        0,
        0
    );
    

    gpiote_init(); 
    
    timer_init(); 

    ppi_init(); 

    clk_timer_init();

    spim_test_init();


    // first round 
    data_rd_reset = true; 
    if (data_rd_reset) {
        sramout_reset(); 
        data_rd_reset = false; 
    }
//     spim_test_transfer1(); 

//     while (1) {
//         if (spim_done) {
//             spim_done = false;
             
//             /* Re-trigger transfer continuously */
//             spim_test_transfer1(); 
//         }

//         /* Sleep until SPIM interrupt fires */
//         k_cpu_idle();
//     }
    spim_test_transfer1(); 
    if (!spim_done) {
        k_cpu_idle();
    }
    spim_done = false; 
    printk("DATA pin raw read: %d\n",
       nrf_gpio_pin_read(SRAM_DATA_IN));
    
    printk("checking data...\n");

    while (!done_rd_SRAM) {
        k_msleep(300); 
        printk("counter_sram: %d \n", counter_sram);
    }
    printk("First counter: %d \n", counter_sram); // 112 bits 
    k_msleep(300);
    
    for (int b = 0; b < 14; b++) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (sram_out[b*8 + i] & 1) << (7 - i);
    }
    printk("%02X ", byte);
}
printk("\n");



    // second round
//     data_rd_reset = true; 
//     if (data_rd_reset) {
//         sramout_reset(); 
//         data_rd_reset = false; 
//     }                                                            

//     spim_test_transfer2();
//     if (!spim_done) {
//         k_cpu_idle();
//     }
//     spim_done = false; 
    
//     printk("checking data 2...");
//     while (!done_rd_SRAM) {
//         k_msleep(300);
//         printk("counter_sram 2: %d \n", counter_sram); 
//     }
//     printk("Second counter: %d \n", counter_sram); // 8 bits
        

}
