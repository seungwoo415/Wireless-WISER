#include <zephyr/kernel.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

 #define GPIOTE_OUT_CH 1
 #define PPI_CH_PULSE NRF_PPI_CHANNEL2
 #define PIN_OUT 14

static void gpiote_out_init(void)
{
    nrf_gpiote_task_configure(
        NRF_GPIOTE,
        GPIOTE_OUT_CH,
        PIN_OUT,
        NRF_GPIOTE_POLARITY_TOGGLE,
        NRF_GPIOTE_INITIAL_VALUE_LOW
    );

    nrf_gpiote_task_enable(NRF_GPIOTE, GPIOTE_OUT_CH);
}

static void pulse_timer_init(void)
{
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);

    nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(NRF_TIMER0, 0);
    nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, 1000); 

    nrf_timer_shorts_enable(
        NRF_TIMER0,
        NRF_TIMER_SHORT_COMPARE0_STOP_MASK
    );

}

static void pulse_ppi_init(void)
{
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        PPI_CH_PULSE,
        nrf_timer_event_address_get(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0),
        nrf_gpiote_task_address_get(NRF_GPIOTE, NRF_GPIOTE_TASK_OUT_1)
    );

    nrf_ppi_channel_enable(NRF_PPI, PPI_CH_PULSE);
}


int main(void)
{       
        // enable usb 
        int err; 
        err = usb_enable(NULL); 
        if (err) {
                printk("usb_enable failed"); 
        }
        printk("usb_enable success\n");
        gpiote_out_init();
        pulse_timer_init();
        pulse_ppi_init();
        nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);
        while (1) {
                k_sleep(K_FOREVER); 
        }
}
