/**
 * @file rtc.c
 * @author Archie Lee
 * @brief Handles RTC timer logic for dataout timestamp. 
 * @details Implements RTC timer initialization, RTC timer handler, and timestamp return.  
 * @version 1.0
 * @date 2026-02-03
 */

/* Includes ------------------------------------------------------------------*/
#include "final_patch.h"

// logging 
LOG_MODULE_DECLARE(final_patch, LOG_LEVEL_INF);

/* RTC Handler ------------------------------------------------------------------*/
// This function runs automatically every 8.5 minutes
static void RTC2_IRQHandler(void) {
    // Check if the OVERFLOW event actually happened
    if (nrf_rtc_event_check(NRF_RTC2, NRF_RTC_EVENT_OVERFLOW)) {
        rtc_overflow_count++; 
        // Clear the event so the interrupt doesn't keep firing
        nrf_rtc_event_clear(NRF_RTC2, NRF_RTC_EVENT_OVERFLOW);
    }
}

/* Start RTC Timer Function ------------------------------------------------------------------*/
void start_timestamp_rtc(void) { 
        // maximum resolution 
        nrf_rtc_prescaler_set(NRF_RTC2, 0);
        
        nrf_rtc_int_enable(NRF_RTC2, NRF_RTC_INT_OVERFLOW_MASK);
        NVIC_EnableIRQ(RTC2_IRQn);

        // reset the clock 
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_CLEAR);

        // start the clock
        nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
}

/* Get Current Timestamp ------------------------------------------------------------------*/
uint64_t get_current_timestamp(void) {
    uint32_t before, low, after;

    // Keep reading until we are sure an overflow didn't happen DURING the read
    do {
        before = rtc_overflow_count;
        low    = nrf_rtc_counter_get(NRF_RTC2);
        after  = rtc_overflow_count;
    } while (before != after);

    return ((uint64_t)before << 24) | low;
}
