#include <zephyr/kernel.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_egu.h>
#include <cmsis_core.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

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
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define EGU0_IRQ DT_IRQN(DT_NODELABEL(egu0))

volatile bool egu_fired = false;
static uint32_t counter; 

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

// test pulse 
static void pulse_gpiote_init(void)
{
    nrf_gpiote_task_configure(
        NRF_GPIOTE,
        GPIOTE_OUT_CH1,
        PIN_OUT,
        NRF_GPIOTE_POLARITY_TOGGLE,
        NRF_GPIOTE_INITIAL_VALUE_LOW
    );

    nrf_gpiote_task_enable(NRF_GPIOTE, GPIOTE_OUT_CH1);
}

static void pulse_timer_init(void)
{
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);

    nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(NRF_TIMER0, 4);
    nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, 100000); 
    nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1, 120000);
    nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL2, 140000);
    nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL3, 160000);

    //nrf_timer_shorts_enable(NRF_TIMER0, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);
    nrf_timer_shorts_enable(NRF_TIMER0, NRF_TIMER_SHORT_COMPARE3_STOP_MASK);
//     nrf_timer_shorts_enable(
//     NRF_TIMER0,
//     NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK
//     );

}

static void pulse_ppi_init(void)
{
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        PPI_CH_PULSE,
        nrf_timer_event_address_get(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0),
        nrf_gpiote_task_address_get(NRF_GPIOTE, NRF_GPIOTE_TASK_OUT_1)
    );

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        NRF_PPI_CHANNEL3,
        nrf_timer_event_address_get(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1),
        nrf_gpiote_task_address_get(NRF_GPIOTE, NRF_GPIOTE_TASK_OUT_1)
    );

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        NRF_PPI_CHANNEL4,
        nrf_timer_event_address_get(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE2),
        nrf_gpiote_task_address_get(NRF_GPIOTE, NRF_GPIOTE_TASK_OUT_1)
    );

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        NRF_PPI_CHANNEL5,
        nrf_timer_event_address_get(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE3),
        nrf_gpiote_task_address_get(NRF_GPIOTE, NRF_GPIOTE_TASK_OUT_1)
    );

    nrf_ppi_channel_enable(NRF_PPI, PPI_CH_PULSE);
    nrf_ppi_channel_enable(NRF_PPI, NRF_PPI_CHANNEL3);
    nrf_ppi_channel_enable(NRF_PPI, NRF_PPI_CHANNEL4);
    nrf_ppi_channel_enable(NRF_PPI, NRF_PPI_CHANNEL5);
}

int main(void)
{
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
        pulse_gpiote_init();
        pulse_timer_init();
        pulse_ppi_init();
        k_msleep(200);
        nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);


        k_msleep(600);
        // Capture counter value
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0);
        counter = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0);
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
