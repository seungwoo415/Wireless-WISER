#include <zephyr/kernel.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

#define TEST_CLK_PIN   14
#define TEST_PERIOD    9000      // 1000 us = 1 kHz toggle
#define TEST_TOGGLES   20

static const nrfx_timer_t test_timer = NRFX_TIMER_INSTANCE(0);
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

static void test_timer_init(void)
{
    nrfx_err_t err;

    /* ---------- GPIOTE INIT ---------- */
    if (!nrfx_gpiote_init_check(&gpiote)) {
        err = nrfx_gpiote_init(&gpiote, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
        NRFX_ASSERT(err == NRFX_SUCCESS);
    }

    uint8_t gpiote_ch;
    err = nrfx_gpiote_channel_alloc(&gpiote, &gpiote_ch);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    const nrfx_gpiote_output_config_t out_cfg = {
        .drive = NRF_GPIO_PIN_S0S1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull = NRF_GPIO_PIN_NOPULL,
    };

    const nrfx_gpiote_task_config_t task_cfg = {
        .task_ch = gpiote_ch,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };

    err = nrfx_gpiote_output_configure(&gpiote, TEST_CLK_PIN, &out_cfg, &task_cfg);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    nrfx_gpiote_out_task_enable(&gpiote, TEST_CLK_PIN);

    /* ---------- TIMER INIT ---------- */
    nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_1MHz);
    tcfg.mode = NRF_TIMER_MODE_TIMER;
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err = nrfx_timer_init(&test_timer, &tcfg, NULL);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    nrfx_timer_clear(&test_timer);

    nrfx_timer_extended_compare(&test_timer,
                             NRF_TIMER_CC_CHANNEL0,
                             TEST_PERIOD,
                             NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                             false);
    nrfx_timer_compare(&test_timer, NRF_TIMER_CC_CHANNEL1,
                      TEST_PERIOD * TEST_TOGGLES, false);

    /* ---------- GPPI ---------- */
    uint8_t gppi_toggle;
    uint8_t gppi_stop;

    nrfx_gppi_channel_alloc(&gppi_toggle);
    nrfx_gppi_channel_alloc(&gppi_stop);

    // COMPARE0 → GPIOTE TOGGLE
    nrfx_gppi_channel_endpoints_setup(
        gppi_toggle,
        nrfx_timer_compare_event_address_get(&test_timer, NRF_TIMER_CC_CHANNEL0),
        nrfx_gpiote_out_task_address_get(&gpiote, TEST_CLK_PIN)
    );

    // COMPARE1 → TIMER STOP
    nrfx_gppi_channel_endpoints_setup(
        gppi_stop,
        nrfx_timer_compare_event_address_get(&test_timer, NRF_TIMER_CC_CHANNEL1),
        nrfx_timer_task_address_get(&test_timer, NRF_TIMER_TASK_STOP)
    );

    nrfx_gppi_channels_enable(BIT(gppi_toggle) | BIT(gppi_stop));
    //nrfx_gppi_channels_enable(BIT(gppi_toggle));

    nrfx_timer_enable(&test_timer);
}

int main(void)
{
    test_timer_init();

    while (1) {
        __WFE();   // CPU sleeps, hardware runs
    }
}
