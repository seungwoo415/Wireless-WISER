#include "test.h" 
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>


//LOG_MODULE_REGISTER(inputs, LOG_LEVEL_INF);

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


/** @brief Symbol specifying PWM instance to be used. */
#define PWM_INST_IDX 2

// pwm instance 
nrfx_pwm_t pwm_instance = NRFX_PWM_INSTANCE(PWM_INST_IDX); 

// global variables 
bool pwm_done = false; 

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
nrf_pwm_values_grouped_t m_duty_cycles[113]; 

/** @brief Array containing sequences to be used in this example. */
// static nrf_pwm_sequence_t seq[] =
// {
//     SEQ_CONFIG(m_duty_cycles)
// };

// Set length to 40 (20 bits * 2 values per bit)
nrf_pwm_sequence_t seq =
{
    .values.p_grouped = m_duty_cycles,
    .length           = 226, 
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
void pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
{
        if (event_type == NRFX_PWM_EVT_FINISHED) pwm_done = true;  
} 

void pwm_init(void) {
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX), 0, 0);

        nrfx_err_t status;
        (void)status;

        nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(CLK_OUT, NRF_PWM_PIN_NOT_CONNECTED, DATA_OUT, NRF_PWM_PIN_NOT_CONNECTED);
        config.base_clock = NRF_PWM_CLK_4MHz; 
        config.top_value = 16; 
        config.load_mode = NRF_PWM_LOAD_GROUPED;
        status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);
} 

// void buffer_init(void) {
//         uint32_t data_value = 0x8888AAAA9999999999999999999999 99 99; 

//         for (uint8_t i = 0; i < 32; i++) {
//                 // Extract bit from MSB down to LSB
//                 bool bit = (data_value >> i) & 0x01;
 
//                 /// Data:
//                 if (!bit) {
//                         // Force "Always High" by setting duty cycle > top_value
//                         // This prevents the PWM counter from ever "resetting" the pin
//                         m_duty_cycles[i].group_1 = 0x8000; 
//                 } else {
//                         m_duty_cycles[i].group_1 = 0;  
//                 }

//                 m_duty_cycles[i].group_0 = 8;
//         }
// }

/* sipo functions */
void buffer_init(const uint8_t *sipo_data) {
        //uint32_t data_value = 0x88888; 

        // uint8_t data_value[16] = {
        //     0xAA, 0xAA, 0xAA, 0xAA, // Bytes 0-3  (101010...)
        //     0x55, 0x55, 0x55, 0x55, // Bytes 4-7  (010101...)
        //     0xFF, 0xFF, 0x00, 0x00, // Bytes 8-11 (Solid High then Solid Low)
        //     0x12, 0x34, 0x56, 0x78  // Bytes 12-15 (Random Pattern)
        // };

        for (uint8_t i = 0; i < 113; i++) {
                // Extract bit from MSB down to LSB
                //bool bit = (data_value >> (20 - 1 - i)) & 0x01;

                // 1. Calculate which byte we are in (0 to 15)
                int byte_idx = i / 8;
                
                // 2. Calculate which bit inside that byte (7 down to 0 for MSB-first)
                //int bit_idx = 7 - (i % 8);
                int bit_idx = (i % 8);

                // 3. Extract the bit
                bool bit = (sipo_data[byte_idx] >> bit_idx) & 0x01;

                /// Data:
                if (!bit) {
                        // Force "Always High" by setting duty cycle > top_value
                        // This prevents the PWM counter from ever "resetting" the pin
                        m_duty_cycles[i].group_1 = 16; 
                } else {
                        m_duty_cycles[i].group_1 = 0;  
                }

                m_duty_cycles[i].group_0 = 8;
        }
}

