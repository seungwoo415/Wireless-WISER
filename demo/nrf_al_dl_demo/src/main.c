#include <zephyr/kernel.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_egu.h>
#include <cmsis_core.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <nrfx_spim.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define GPIOTE_CH 0
#define GPIOTE_OUT_CH1 1

#define PPI_CH NRF_PPI_CHANNEL0
#define PPI_CH_EGU NRF_PPI_CHANNEL1
#define PPI_CH_PULSE NRF_PPI_CHANNEL2

#define PIN_IN 14
#define PIN_OUT 26
#define PIN_OUT2 27
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define EGU0_IRQ DT_IRQN(DT_NODELABEL(egu0))

volatile bool egu_fired = false;
static uint32_t counter; 

#define SPIM_INST_IDX 1
static const nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);
volatile bool spim_done = false; 

// data buffers 
static uint8_t tx_buf1[5] = {
    0x88, 0x88, 0x88, 0x88, // 10001000.. 112 bits 
    0x88
}; 

// not needed for print 
// void EGU0_IRQHandler(void)
// {
//     if (nrf_egu_event_check(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED0)) {
//         nrf_egu_event_clear(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED0);

//         egu_fired = true;

//     }
// }

static void gpiote_init(void) {
        nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_CH, PIN_IN, NRF_GPIOTE_POLARITY_LOTOHI);
        nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_CH); 
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
                nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_CH)), 
                nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_COUNT)
        ); 

        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_CH_EGU,
                nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_CH)),
                nrf_egu_task_address_get(NRF_EGU0, NRF_EGU_TASK_TRIGGER0)
        );

        nrf_ppi_channel_enable(NRF_PPI, PPI_CH);
        nrf_ppi_channel_enable(NRF_PPI, PPI_CH_EGU);
}
// not needed for print
// static void egu_init(void)
// {    
//     IRQ_CONNECT(EGU0_IRQ, 3, EGU0_IRQHandler, NULL, 0);
//     irq_enable(EGU0_IRQ);


//     nrf_egu_int_enable(NRF_EGU0, EGU_INTENSET_TRIGGERED0_Msk);
// } 

static void spim_handler(nrfx_spim_evt_t const * evt, void * ctx) {
        if (evt->type == NRFX_SPIM_EVENT_DONE) {
                spim_done = true;
        }
 }

/* SPIM test */
static void spim_test_init(void) {
    // spim config
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(PIN_OUT,
                                                              PIN_OUT2,
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


int main(void)
{       
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX),
        0,
        0
        );
        // enable usb 
        int err; 
        // err = usb_enable(NULL); 
        // if (err) {
        //         printk("usb_enable failed"); 
        // }
        // printk("usb_enable success\n");

        if (!device_is_ready(led.port)) {
		return -1;
	    }

        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	    if (err < 0) {
		    return -1;
	    }
        gpio_pin_set_dt(&led, 1);
        k_msleep(200);
        gpio_pin_set_dt(&led, 0);
        k_msleep(200); 

        // input initialize 
        gpiote_init(); 

        timer_init();
        
        // not needed for print
        //egu_init();

        ppi_init(); 


        // test clock initialize 
        spim_test_init(); 
        spim_test_transfer1(); 
        k_msleep(500); 

        if (spim_done) {
            nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0);
            counter = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0);
        }
        // not needed for print
        // if (egu_fired) {
        //             egu_fired = false;
        //             for (uint32_t i = 0; i < counter; i++) {
        //                     gpio_pin_set_dt(&led, 1);
        //                     k_msleep(200);
        //                     gpio_pin_set_dt(&led, 0); 
        //                     k_msleep(200);
        //             }
        // }
        printk("Counter: %u \n", counter); 

}
