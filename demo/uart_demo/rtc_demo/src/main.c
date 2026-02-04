#include <zephyr/kernel.h> 
#include <hal/nrf_rtc.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(main);

// overflow counter 
volatile uint32_t overflow_count = 0; 


// This function runs automatically every 8.5 minutes
void RTC2_IRQHandler(void) {
    // Check if the OVERFLOW event actually happened
    if (nrf_rtc_event_check(NRF_RTC2, NRF_RTC_EVENT_OVERFLOW)) {
        overflow_count++; 
        // Clear the event so the interrupt doesn't keep firing
        nrf_rtc_event_clear(NRF_RTC2, NRF_RTC_EVENT_OVERFLOW);
    }
}

void init_timestamp_rtc(void) { 
        // maximum resolution 
        nrf_rtc_prescaler_set(NRF_RTC2, 0);
        
        nrf_rtc_int_enable(NRF_RTC2, NRF_RTC_INT_OVERFLOW_MASK);
        //IRQ_CONNECT(RTC0_IRQn, 1, rtc_handler, NULL, 0);
        NVIC_EnableIRQ(RTC2_IRQn);

        // reset the clock 
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);

        // start the clock
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
}

uint64_t get_current_timestamp(void) {
    uint32_t before, low, after;

    // Keep reading until we are sure an overflow didn't happen DURING the read
    do {
        before = overflow_count;
        low    = nrf_rtc_counter_get(NRF_RTC2);
        after  = overflow_count;
    } while (before != after);

    return ((uint64_t)before << 24) | low;
}

int main(void)
{
        init_timestamp_rtc(); 

        while(1) {
                k_msleep(1000);
                uint64_t time = get_current_timestamp(); 
                LOG_INF("time: %" PRIu64, time); 
        }       

        return 0; 
}
