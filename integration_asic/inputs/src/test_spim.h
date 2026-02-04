#ifndef TEST_SPIM_H
#define TEST_SPIM_H

#include <zephyr/kernel.h>
#include <nrfx_spim.h>

#define CLK_OUT 14
#define DATA_OUT 13

// Share these with main.c
extern const nrfx_spim_t spim_inst;
extern volatile bool spim_done;
extern uint8_t tx_buf[4];

void spim_init(void);
void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context);

#endif