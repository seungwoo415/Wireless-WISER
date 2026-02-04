#include <zephyr/kernel.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

#define PULSE_PIN        14
#define PULSE_START_US   100000    // 100 ms
#define PULSE_WIDTH_US   200000    // 200 ms

static const nrfx_timer_t pulse_timer = NRFX_TIMER_INSTANCE(0);
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

static void pulse_init(void)
{
    nrfx_err_t err;

    /* -------- GPIOTE -------- */
    if (!nrfx_gpiote_init_check(&gpiote)) {
        err = nrfx_gpiote_init(&gpiote,
                               NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
        NRFX_ASSERT(err == NRFX_SUCCESS);
    }

    uint8_t ch;
    err = nrfx_gpiote_channel_alloc(&gpiote, &ch);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    const nrfx_gpiote_output_config_t out_cfg = {
        .drive = NRF_GPIO_PIN_S0S1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull = NRF_GPIO_PIN_NOPULL,
    };

    const nrfx_gpiote_task_config_t task_cfg = {
        .task_ch = ch,
        .polarity = NRF_GPIOTE_POLARITY_NONE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };

    err = nrfx_gpiote_output_configure(&gpiote,
                                       PULSE_PIN,
                                       &out_cfg,
                                       &task_cfg);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    // 🔴 THIS WAS THE MISSING LINE
    nrfx_gpiote_out_task_enable(&gpiote, PULSE_PIN);

    /* -------- TIMER -------- */
    nrfx_timer_config_t tcfg =
        NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_1MHz);
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err = nrfx_timer_init(&pulse_timer, &tcfg, NULL);
    NRFX_ASSERT(err == NRFX_SUCCESS);

    nrfx_timer_clear(&pulse_timer);

    nrfx_timer_compare(&pulse_timer,
                       NRF_TIMER_CC_CHANNEL0,
                       PULSE_START_US,
                       false);

    nrfx_timer_compare(&pulse_timer,
                       NRF_TIMER_CC_CHANNEL1,
                       PULSE_START_US + PULSE_WIDTH_US,
                       false);

    /* -------- GPPI -------- */
    uint8_t gppi_set, gppi_clr;
    nrfx_gppi_channel_alloc(&gppi_set);
    nrfx_gppi_channel_alloc(&gppi_clr);

    nrfx_gppi_channel_endpoints_setup(
        gppi_set,
        nrfx_timer_compare_event_address_get(&pulse_timer,
                                             NRF_TIMER_CC_CHANNEL0),
        nrfx_gpiote_set_task_address_get(&gpiote, PULSE_PIN)
    );

    nrfx_gppi_channel_endpoints_setup(
        gppi_clr,
        nrfx_timer_compare_event_address_get(&pulse_timer,
                                             NRF_TIMER_CC_CHANNEL1),
        nrfx_gpiote_clr_task_address_get(&gpiote, PULSE_PIN)
    );

    nrfx_gppi_channels_enable(BIT(gppi_set) | BIT(gppi_clr));

    nrfx_timer_enable(&pulse_timer);
}

int main(void)
{
    pulse_init();

    while (1) {
        __WFE();
    }
}
