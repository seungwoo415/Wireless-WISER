#ifndef TEST_H
#define TEST_H

#include <zephyr/kernel.h>
#include <nrfx_pwm.h>

// pins     
#define CLK_OUT 14
#define DATA_OUT 13 

/* * Function Prototypes
 * This allows main.c to "see" the functions defined in test.c
 */

/** @brief Initializes the PWM peripheral and connects interrupts */
void pwm_init(void);

/** @brief Fills the duty cycle buffer with 20-bit data */
void buffer_init(const uint8_t *sipo_data);

/* * Extern Variables
 * If you need to check the status of 'pwm_done' in your main loop
 */
extern bool pwm_done;
extern nrf_pwm_sequence_t seq;
extern nrfx_pwm_t pwm_instance;

#endif // TEST_H