/**
 * @file rtc.h
 * @author Archie Lee
 * @brief  Header file for RTC timer logic for dataout timestamp. 
 * @details Defines RTC timer initialization, RTC timer handler, and timestamp return.  
 * @version 1.0
 * @date 2026-02-03
 */

#ifndef RTC_H
#define RTC_H


/* --- Function Prototypes --- */

/**
 * @brief Begins timestamp timer for dataout.
 */
void start_timestamp_rtc(void); 

/**
 * @brief Returns timestamp for dataout.
 */
uint64_t get_current_timestamp(void);

#endif /* FINAL_PATCH_H */