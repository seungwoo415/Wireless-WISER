#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include "test_spim.h" 

LOG_MODULE_DECLARE(inputs, LOG_LEVEL_INF);

#define SPIM_INST_IDX 1
const nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

volatile bool spim_done = false; 

// data buffers 
uint8_t tx_buf[4] = {
    0x88, 0x88, 0xAA, 0xAA  // 10001000 8 bits 
}; 

void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context){
        if (p_event->type == NRFX_SPIM_EVENT_DONE)
        {
                spim_done = true; 
        }
} 

void spim_init(void) {
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX),
            0,
            0
        );
        nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG(CLK_OUT, DATA_OUT, NRF_SPIM_PIN_NOT_CONNECTED, NRF_SPIM_PIN_NOT_CONNECTED);

        cfg.frequency = NRFX_MHZ_TO_HZ(1);       // 1 MHz
        cfg.mode      = NRF_SPIM_MODE_0;         // CPOL=0, CPHA=0
        cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

        nrfx_err_t err = nrfx_spim_init(&spim_inst, &cfg, spim_handler, NULL);
        if (err != NRFX_SUCCESS) {
                 LOG_INF("SPIM init failed");
        }
        LOG_INF("SPIM initialized");

}