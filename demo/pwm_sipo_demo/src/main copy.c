

// #include <nrfx_example.h>
#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>

#include <zephyr/kernel.h>

#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3
#include <nrfx_log.h>
#include <zephyr/logging/log.h>

/**
 * @defgroup nrfx_pwm_grouped_example Grouped mode PWM example
 * @{
 * @ingroup nrfx_pwm_examples
 *
 * @brief Example showing basic functionality of nrfx_pwm driver for sequences loaded in grouped
 *        mode.
 *
 * @details Application initializes nrfx_pmw driver. It starts two-sequence playback on LEDs and
 *          replays this sequence @ref NUM_OF_LOOPS times. The @ref pwm_handler() is executed with
 *          relevant log message after every loop. Additionally, it changes SEQ1 each time it is
 *          called.
 */

// pins 
#define CLK_OUT 14
#define DATA_OUT 26 

/** @brief Symbol specifying PWM instance to be used. */
#define PWM_INST_IDX 2

// pwm instance 
nrfx_pwm_t pwm_instance = NRFX_PWM_INSTANCE(PWM_INST_IDX); 

// global variables 
static bool pwm_done = false; 

/**
 * @brief Sequence default configuration in NRF_PWM_LOAD_GROUPED mode.
 *
 * This configuration sets up sequence with the following options:
 * - end delay: 0 PWM periods
 * - length: actual number of 16-bit values in the array pointed by @p _pwm_val
 * - repeats: VALUE_REPEATS
 *
 * @param[in] _pwm_val pointer to an array with duty cycle values.
 */
// #define SEQ_CONFIG(_pwm_val)                            \
// {                                                       \
//     .values.p_grouped = _pwm_val,                       \
//     .length           = 2 * NRFX_ARRAY_SIZE(_pwm_val),  \
//     .repeats          = VALUE_REPEATS,                  \
//     .end_delay        = 0                               \
// }

// buffer 
static nrf_pwm_values_grouped_t m_duty_cycles[126]; 

/** @brief Array containing sequences to be used in this example. */
// static nrf_pwm_sequence_t seq[] =
// {
//     SEQ_CONFIG(m_duty_cycles)
// };

// Set length to 40 (20 bits * 2 values per bit)
static nrf_pwm_sequence_t seq =
{
    .values.p_grouped = m_duty_cycles,
    .length           = 126 * 2, 
    .repeats          = 0,
    .end_delay        = 0
}; 
/**
 * @brief Function for handling PWM driver events.
 *
 * @param[in] event_type PWM event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function.
 */
static void pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
{
        if (event_type == NRFX_PWM_EVT_FINISHED) pwm_done = true;  
} 

static void pwm_init(void) {
        nrfx_err_t status;
        (void)status;

        nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(CLK_OUT, NRF_PWM_PIN_NOT_CONNECTED, DATA_OUT, NRF_PWM_PIN_NOT_CONNECTED);
        config.base_clock = NRF_PWM_CLK_16MHz; 
        config.top_value = 32; 
        config.load_mode = NRF_PWM_LOAD_GROUPED;
        status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);
} 

static void buffer_init(void) {
        uint32_t data_value = 0x88888; 

        for (uint8_t i = 0; i < 20; i++) {
                // Extract bit from MSB down to LSB
                bool bit = (data_value >> (20 - 1 - i)) & 0x01;

                /// Data:
                if (!bit) {
                        // Force "Always High" by setting duty cycle > top_value
                        // This prevents the PWM counter from ever "resetting" the pin
                        m_duty_cycles[i].group_1 = 0x8000; 
                } else {
                        m_duty_cycles[i].group_1 = 0;  
                }

                m_duty_cycles[i].group_0 = 8;
        }
}

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */
int main(void)
{
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX), 0, 0);


    //NRFX_EXAMPLE_LOG_INIT();

    NRFX_LOG_INFO("Starting nrfx_pwm example for sequences loaded in grouped mode.");
    //NRFX_EXAMPLE_LOG_PROCESS(); 
    
    buffer_init(); 

    pwm_init(); 

    nrfx_pwm_simple_playback(&pwm_instance, &seq, 1, NRFX_PWM_FLAG_STOP);

    while (1)
    {
        if (pwm_done) {
            pwm_done = false;

            k_msleep(50); 

            nrfx_pwm_simple_playback(&pwm_instance, &seq, 1, NRFX_PWM_FLAG_STOP);
        }

        // Yield to Zephyr so other threads can run
        k_yield(); 
    }
}

/** @} */
