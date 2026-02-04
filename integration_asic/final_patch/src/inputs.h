/**
 * @file inputs.h
 * @author Archie Lee
 * @brief Header file for data interface between nrf52840 inputs and ASIC.
 * @details Defines the public structures and function prototypes for 
 * SRAMOUT, DATAOUT, AL, and DL.
 * @version 1.0
 * @date 2026-01-23
 */

#ifndef INPUTS_H
#define INPUTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- Function Prototypes --- */

/** @brief Initializes GPIOTE and EGU for high-speed input sampling. */
void gpiote_inputs_init(void);

/** @brief Initializes timers used for sampling timeouts. */
void timer_timeout_init(void);

/** @brief Configures PPI to link GPIOTE events to Timer/EGU tasks. */
void ppi_inputs_init(void);

/** @brief Initializes timers used as counters (sampling AL/DL). */
void timer_counter_init(void);

/** @brief Resets AL counter. */
void reset_al_counter(void);

/** @brief Resets DL counter. */
void reset_dl_counter(void);

/** @brief Resets sramout. */
void reset_sramout(void);

/** @brief Resets dataout. */
void reset_dataout(void);

/** @brief Updates AL counter value. */
void update_al_counter(void);

/** @brief Updates DL counter value. */
void update_dl_counter(void);

/** @brief Processes sramout data. */
void process_sramout(volatile uint32_t *buffer, int count); 

/** @brief Processes dataout data. */
void process_dataout(uint32_t dataout, int count, uint64_t timestamp); 

/** @brief Returns sramout bit. */
bool get_sram_bit(volatile uint32_t *buffer, int bit_idx); 

#ifdef __cplusplus
}
#endif

#endif /* INPUTS_H */