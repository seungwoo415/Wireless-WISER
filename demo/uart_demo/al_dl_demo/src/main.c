#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>

/* ===== PIN DEFINITIONS ===== */
#define TEST_CLK_PIN 7 // Test clocks
#define DL_PIN  26   // ASIC DL clock 26
#define AL_PIN  27  // ASIC AL clock 27

#define DL_CC NRF_TIMER_CC_CHANNEL0
#define AL_CC NRF_TIMER_CC_CHANNEL0

#define TEST_TOGGLES 20 // expect 10 counter
#define PERIOD_US 1000

/* ===== TIMER INSTANCES ===== */
static const nrfx_timer_t dl_timer = NRFX_TIMER_INSTANCE(0);
static const nrfx_timer_t al_timer = NRFX_TIMER_INSTANCE(1); 
static const nrfx_timer_t test_timer = NRFX_TIMER_INSTANCE(2);

// only one GPIOTE instance multiple GPIOTE channels 
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

// gpiote channels
static uint8_t dl_ch, al_ch;

// gppi channels 
static uint8_t gppi_dl;
static uint8_t gppi_al;

// to print out counter value 
volatile bool dl_updated = false; 
volatile bool al_updated = false; 

// gpiote handler for printing out counter values 
// static void gpiote_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void * p_context) {
//         if (pin == DL_PIN) dl_updated = true; 
//         else if (pin == AL_PIN) al_updated = true; 
// }

static void test_timer_init() {
    nrf_gpio_cfg_output(TEST_CLK_PIN);
    nrf_gpio_pin_clear(TEST_CLK_PIN);  // known start LOW

    for (int i = 0; i < TEST_TOGGLES; i++) {
        nrf_gpio_pin_toggle(TEST_CLK_PIN);
        k_busy_wait(PERIOD_US);
    }

    // force known final state
    nrf_gpio_pin_clear(TEST_CLK_PIN);
}

int main(void)
{
    // enable usb 
    int err; 
    err = usb_enable(NULL); 
    if (err) {
        printk("usb_enable failed"); 
    }
    printk("usb_enable success\n");
    // initialize gpiote driver 
    if (!nrfx_gpiote_init_check(&gpiote)) {
        nrfx_gpiote_init(&gpiote, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY); 
    }
    
    // allocate gpiote channels  
    nrfx_gpiote_channel_alloc(&gpiote, &dl_ch); 
    nrfx_gpiote_channel_alloc(&gpiote, &al_ch); 

    // trigger config for input pin config 
    static nrfx_gpiote_trigger_config_t dl_trigger_cfg = {
        .trigger = NRFX_GPIOTE_TRIGGER_LOTOHI, 
        .p_in_channel = &dl_ch, 
    };

    static nrfx_gpiote_trigger_config_t al_trigger_cfg = {
        .trigger = NRFX_GPIOTE_TRIGGER_LOTOHI, 
        .p_in_channel = &al_ch, 
    };

    // gpiote handler config 
    static nrfx_gpiote_handler_config_t gpiote_handler_cfg = {
        .handler = NULL, 
        .p_context = NULL, 
    };

    // configure the input 
    static nrfx_gpiote_input_pin_config_t input_cfg = {
        .p_pull_config = NRF_GPIO_PIN_NOPULL, 
        .p_handler_config = &gpiote_handler_cfg, 
    }; 

    // configure trigger for output pin 
    input_cfg.p_trigger_config = &dl_trigger_cfg;
    nrfx_gpiote_input_configure(&gpiote, DL_PIN, &input_cfg); 
    input_cfg.p_trigger_config = &al_trigger_cfg;
    nrfx_gpiote_input_configure(&gpiote, AL_PIN, &input_cfg); 
    
    /* ---------- TIMER INIT (COUNTER MODE) ---------- */
    // initialize timer driver 
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_FREQ_1MHz);
    timer_cfg.mode      = NRF_TIMER_MODE_COUNTER;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    nrfx_timer_init(&dl_timer, &timer_cfg, NULL);
    nrfx_timer_init(&al_timer, &timer_cfg, NULL);
    
    // clear the timer 
    nrfx_timer_clear(&dl_timer);
    nrfx_timer_clear(&al_timer);
    
    // enable the timer 
    nrfx_timer_enable(&dl_timer);
    nrfx_timer_enable(&al_timer);

    /* ---------- GPPI INIT ---------- */ 
    // get addresses to connect GPPI 
    //uint32_t dl_evt = nrfx_gpiote_in_event_address_get(&gpiote, DL_PIN); 
    //uint32_t al_evt = nrfx_gpiote_in_event_address_get(&gpiote, AL_PIN); 

    //uint32_t dl_cnt = nrfx_timer_task_address_get(&dl_timer, NRF_TIMER_TASK_COUNT); 
    //uint32_t al_cnt = nrfx_timer_task_address_get(&al_timer, NRF_TIMER_TASK_COUNT);
    
    // allocate the PPI channels 
    nrfx_gppi_channel_alloc(&gppi_dl);
    nrfx_gppi_channel_alloc(&gppi_al);

    // assign gppi channel endpoints 
    nrfx_gppi_channel_endpoints_setup(gppi_dl, nrfx_gpiote_in_event_address_get(&gpiote, dl_ch), nrfx_timer_task_address_get(&dl_timer, NRF_TIMER_TASK_COUNT)); 
    nrfx_gppi_channel_endpoints_setup(gppi_al, nrfx_gpiote_in_event_address_get(&gpiote, al_ch), nrfx_timer_task_address_get(&al_timer, NRF_TIMER_TASK_COUNT));

    // enable trigger for output pin 
    nrfx_gpiote_trigger_enable(&gpiote, DL_PIN, true);
    nrfx_gpiote_trigger_enable(&gpiote, AL_PIN, true);

    // enable the gppi channels 
    nrfx_gppi_channels_enable(BIT(gppi_dl) | BIT(gppi_al));  

    k_sleep(K_MSEC(500));
    nrfx_timer_clear(&dl_timer);
    test_timer_init(); 
    printk("test timer initialized\n");
    k_sleep(K_MSEC(10));
    uint32_t dl_count = nrfx_timer_capture(&dl_timer, DL_CC); 
    printk("DL count =  %u\n", dl_count); 

    uint32_t al_count = nrfx_timer_capture(&al_timer, AL_CC);
    printk("AL count =  %u\n", al_count);

    /* ---------- MAIN LOOP ---------- */
    while(1){

        k_sleep(K_MSEC(1)); // sleep, CPU not involved in counting
    }
}
