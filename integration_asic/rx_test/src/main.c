

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
#define CLK_OUT 14 // SCK
#define DATA_OUT 26 // 9

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
static nrf_pwm_values_grouped_t sramout_buffer[112]; 
static nrf_pwm_values_grouped_t dataout_buffer[26];

/** @brief Array containing sequences to be used in this example. */
// static nrf_pwm_sequence_t seq[] =
// {
//     SEQ_CONFIG(m_duty_cycles)
// };

// Set length to 40 (20 bits * 2 values per bit)
static nrf_pwm_sequence_t seq_sramout =
{
    .values.p_grouped = sramout_buffer,
    .length           = 112 * 2, 
    .repeats          = 0,
    .end_delay        = 0
}; 

static nrf_pwm_sequence_t seq_dataout =
{
    .values.p_grouped = dataout_buffer,
    .length           = 26 * 2, 
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
        config.pin_inverted[0] = false;
        config.pin_inverted[1] = false;
        config.pin_inverted[2] = false;
        config.pin_inverted[3] = false;
        config.base_clock = NRF_PWM_CLK_8MHz; 
        config.top_value = 32; 
        config.load_mode = NRF_PWM_LOAD_GROUPED;
        status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, NULL);
        NRFX_ASSERT(status == NRFX_SUCCESS);
} 

void prepare_sramout_buffer(uint16_t sramout[7]) {
    int buf_idx = 0;

    for (int s = 6; s >= 0; s--) {
        //int bits_to_send = (s == 0) ? 14 : 16; // sipo0 only sends 14 bits [15:2]
        uint16_t current_val = sramout[s];

        for (int i = 15; i >= 0; i--) {
            if (buf_idx >= 112) break;

            bool bit = (current_val >> i) & 0x01;

            // Group 1: DATA PIN
            // 0x8000 sets the 15th bit (Inverted polarity), ensuring pin stays HIGH or LOW
            sramout_buffer[buf_idx].group_1 = bit ? 0 : 0x8000;

            // Group 0: CLOCK PIN
            // 50% duty cycle (16 out of 32)
            sramout_buffer[buf_idx].group_0 = 16;

            buf_idx++;
        }
    }
} 

void prepare_dataout_buffer(uint16_t dataout[2]) {
    int buf_idx = 0;

    for (int s = 1; s >= 0; s--) {
        //int bits_to_send = (s == 0) ? 14 : 16; // sipo0 only sends 14 bits [15:2]
        uint16_t current_val = dataout[s];

        for (int i = 15; i >= 0; i--) {
            if (buf_idx >= 26) break;

            bool bit = (current_val >> i) & 0x01;

            // Group 1: DATA PIN
            // 0x8000 sets the 15th bit (Inverted polarity), ensuring pin stays HIGH or LOW
            dataout_buffer[buf_idx].group_1 = bit ? 0 : 0x8000;

            // Group 0: CLOCK PIN
            // 50% duty cycle (16 out of 32)
            dataout_buffer[buf_idx].group_0 = 16;

            buf_idx++;
        }
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
    
//     uint16_t sipo_test_data[8] = {
//               0xAAAC, 0xAAAA, 0x5555, 0xAAAA, 
//               0x5555, 0xAAAA, 0x5555, 0xAAAA  
//     }; 

    uint16_t sramout_test_buffer[7] = { 
        0x8888, // sipo[1]
        0x5678, // sipo[2]
        0x1234, // sipo[3]
        0x5555, // sipo[4]
        0xAAAA, // sipo[5]
        0x8888, // sipo[6]
        0x8888  // sipo[7]: Bits 15-0 are sent (1001 1001 1001 1001) -> SENT FIRST
    };

    uint16_t dataout_test_buffer[2] = { 
        0x01AA, // sipo[1]
        0x8888, // sipo[2]
    };


    prepare_sramout_buffer(sramout_test_buffer); 

    prepare_dataout_buffer(dataout_test_buffer); 

    pwm_init(); 

    nrfx_pwm_simple_playback(&pwm_instance, &seq_sramout, 1, NRFX_PWM_FLAG_STOP);

    //nrfx_pwm_simple_playback(&pwm_instance, &seq_dataout, 1, NRFX_PWM_FLAG_STOP);
    
    while (!pwm_done) k_yield(); 

    LOG_INF("test done"); 
}

/** @} */
