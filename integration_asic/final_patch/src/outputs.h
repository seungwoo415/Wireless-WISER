/**
 * @file outputs.h
 * @author Archie Lee
 * @brief Header file for data interface between nrf52840 outputs and ASIC.
 * @details Defines the public structures and function prototypes for 
 * SIPO, DAC I2C, Base clock, 1MHz clock, and Chip reset.
 * @version 1.0
 * @date 2026-01-23
 */

#ifndef OUTPUTS_H
#define OUTPUTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- Function Prototypes --- */

/**
 * @brief Initializes the PWM for SIPO data transmission.
 * Configures grouped load mode and connects the IRQ handler.
 */
void pwm_sipo_init(void);

/**
 * @brief Initializes a 1MHz clock source using a PWM instance.
 */
void pwm_clk_1mhz_init(void);

/**
 * @brief Initializes the base reference clock for the system.
 */
void pwm_clk_base_init(void);

/**
 * @brief Initializes the DAC I2C interface to an idle state.
 */
void dac_init(void);

/**
 * @brief Resets the DAC I2C interface to an idle state.
 */
void dac_reset(void);

/**
 * @brief Resets the SIPO interface to an idle state.
 */
void sipo_reset(void); 

/**
 * @brief Bit-bangs 36 bits of data to the I2C DAC.
 * @param dac2 Top 16 bits
 * @param dac1 Middle 16 bits
 * @param dac0 Bottom 16 bits (shifts to use MSB 4 bits)
 */
void dac_i2c_write(const uint8_t *data, uint16_t len);

/**
 * @brief Configures PPI and GPIOTE to link clock events to PWM tasks.
 */
void ppi_outputs_init(void);

/**
 * @brief Fills the SIPO buffer with test patterns or specific data.
 */
void prepare_sipo_buffer(const uint8_t *sipo_data);  

/**
 * @brief Enables PPI and triggers the SIPO data playback.
 */
void sipo_trigger_transfer(void);

/**
 * @brief Starts the 1MHz clock output.
 */
void clk_1mhz_trigger_transfer(void);

/**
 * @brief Starts the base reference clock output.
 */
void clk_base_trigger_transfer(void);

/**
 * @brief Resets the chip.
 */
void chip_reset(void); 

#ifdef __cplusplus
}
#endif

#endif /* OUTPUTS_H */