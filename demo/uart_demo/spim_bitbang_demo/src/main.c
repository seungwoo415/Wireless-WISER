#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
//#include <zephyr/usb/usb_device.h>

#include <nrfx_spim.h>
#include <hal/nrf_gpio.h>

/* ---------------- Pins (Feather nRF52840 – CHANGE IF NEEDED) ---------------- */
#define PIN_SCK   13
#define PIN_MOSI  14
#define PIN_MISO  NRF_SPIM_PIN_NOT_CONNECTED
#define PIN_CS    NRF_SPIM_PIN_NOT_CONNECTED   // NO CS

/* ---------------- SPIM instance ---------------- */
#define SPIM_INST_IDX 1
static const nrfx_spim_t spim = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

/* ---------------- State flag ---------------- */
static volatile bool spim_done = false;

/* ---------------- TX buffer: 10001000 pattern ---------------- */
static uint8_t tx_buf[2] = {
    0x88, 0x88
};

/* ---------------- SPIM interrupt handler ---------------- */
static void spim_handler(nrfx_spim_evt_t const * p_evt, void * p_context)
{
//     if (p_evt->type == NRFX_SPIM_EVENT_DONE) {
//         spim_done = true;
//     }
}

static void sipo_transfer_20_bits(void) {
        nrfx_spim_xfer_desc_t xfer = {
                .p_tx_buffer = tx_buf,
                .tx_length   = sizeof(tx_buf),
                .p_rx_buffer = NULL,
                .rx_length   = 0
        };

        spim_done = false;

        uint32_t saved_sck_pin  = spim.p_reg->PSEL.SCK;
        uint32_t saved_mosi_pin = spim.p_reg->PSEL.MOSI;

        nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer, 0); 
        // if (err != NRFX_SUCCESS) {
        //         printk("SPIM xfer failed\n");
        // }
        // printk("SPIM first xfer\n");

        while (!nrf_spim_event_check(spim.p_reg, NRF_SPIM_EVENT_END));

        // while (!spim_done) {
        //         k_busy_wait(1); 
        // }

        // k_busy_wait(2);

        spim.p_reg->ENABLE = 0;

        spim.p_reg->PSEL.SCK  = 0xFFFFFFFF; 
        spim.p_reg->PSEL.MOSI = 0xFFFFFFFF; 

        for (int i = 0; i < 4; i++) {
                // Set MOSI data bit (MSB first)
                if (i%2) {
                        nrf_gpio_pin_set(PIN_MOSI);
                } else {
                        nrf_gpio_pin_clear(PIN_MOSI);
                }

                // Pulse the SCK
                nrf_gpio_pin_set(PIN_SCK);
                for(int n=0; n<20; n++) { __NOP(); } // Adjust for your 1MHz requirement
                //k_busy_wait(1);
                nrf_gpio_pin_clear(PIN_SCK);
                for(int n=0; n<15; n++) { __NOP(); }
                //k_busy_wait(1);
        }
        
        spim.p_reg->PSEL.SCK  = saved_sck_pin;
        spim.p_reg->PSEL.MOSI = saved_mosi_pin; 
        spim.p_reg->ENABLE = 7;
        printk("20-bit transfer (16 HW + 4 SW) done\n");

}

int main(void)
{
//     int ret;

//     /* -------- USB console -------- */
//     ret = usb_enable(NULL);
//     if (ret) {
//         return 0;
//     }
//     k_msleep(500);
    printk("SPIM test start\n");

    //enable interrupt 
    IRQ_CONNECT(
        NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)),
        IRQ_PRIO_LOWEST,
        NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX),
        0,
        0
    );

    /* -------- SPIM configuration -------- */
    nrfx_spim_config_t cfg =
        NRFX_SPIM_DEFAULT_CONFIG(PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS);

    cfg.frequency = NRFX_MHZ_TO_HZ(1);       // 1 MHz
    cfg.mode      = NRF_SPIM_MODE_0;         // CPOL=0, CPHA=0
    cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

    nrfx_err_t err = nrfx_spim_init(&spim, &cfg, spim_handler, NULL);
    if (err != NRFX_SUCCESS) {
        printk("SPIM init failed\n");
        return 0;
    }
    printk("SPIM initialized");
 

    /* -------- Main loop -------- */
    while (1) {

        sipo_transfer_20_bits(); 
        k_msleep(100);

    }
}
