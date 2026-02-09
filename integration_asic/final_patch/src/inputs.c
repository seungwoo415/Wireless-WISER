/**
 * @file inputs.c
 * @author Archie Lee
 * @brief Handles data interface between nrf52840 inputs and ASIC. 
 * @details Implements SRAMOUT, DATAOUT, AL, and DL inputs.  
 * @version 1.0
 * @date 2026-01-23
 */

/* Includes ------------------------------------------------------------------*/
#include "final_patch.h"

LOG_MODULE_DECLARE(final_patch, LOG_LEVEL_INF);

// dataout fifo 
//K_MSGQ_DEFINE(fifo_data_out, sizeof(uint32_t), 32, 4);
K_MSGQ_DEFINE(fifo_data_out, sizeof(struct dataout_ble_packet), 32, 4);

/* ISR ------------------------------------------------------------------*/
// https://docs.zephyrproject.org/latest/kernel/services/interrupts.html 
__attribute__((section(".ramfunc")))
ISR_DIRECT_DECLARE(gpiote_fast_handler)
{
    NRF_GPIOTE->EVENTS_IN[1] = 0;

    uint32_t count = sramout_count;

    if (NRF_P0->IN & (1 << SRAMOUT_DATA_IN)) {
        // Shifting a constant 1UL is faster than variable math
        sramout_buffer[count >> 5] |= (1UL << (count & 31));
    }
        sramout_count = count + 1; 

    return 0;
}

__attribute__((section(".ramfunc")))
ISR_DIRECT_DECLARE(egu_fast_handler) {
    NRF_GPIOTE->EVENTS_IN[2] = 0;
    NRF_EGU0->EVENTS_TRIGGERED[0] = 0;

    if (NRF_P0->IN & (1 << DATAOUT_DATA_IN)) {
        dataout_buffer |= (1 << dataout_count);
    }
    dataout_count++;
    return 0;
}


/* Timeout Handlers ------------------------------------------------------------------*/
static void timer_sramout_handler(nrf_timer_event_t event_type, void * p_context) {
        if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // packet is finished 
                if (sramout_count > 0) {
                        k_work_submit(&sramout_work); 
                }
        }
}

static void timer_dataout_handler(nrf_timer_event_t event_type, void * p_context) {
        if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        // packet is finished 
                if (dataout_count > 0) {
                        k_work_submit(&dataout_work); 
                }
        }
} 


/* Peipheral Initializations ------------------------------------------------------------------*/
void gpiote_inputs_init(void) {

        // enable instruction cache
        NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;

        IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, gpiote_fast_handler, IRQ_ZERO_LATENCY);
        irq_enable(GPIOTE_IRQn);

        IRQ_DIRECT_CONNECT(SWI0_EGU0_IRQn, 0, egu_fast_handler, IRQ_ZERO_LATENCY);
        irq_enable(SWI0_EGU0_IRQn);

        // sramout channel 
        NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)   |
                            (SRAMOUT_CLK_IN                        << GPIOTE_CONFIG_PSEL_Pos)   |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk;

        nrf_gpio_cfg_input(SRAMOUT_DATA_IN, NRF_GPIO_PIN_NOPULL);

        // egu for dataout 
        NRF_EGU0->EVENTS_TRIGGERED[0] = 0;
        NRF_EGU0->INTENSET = EGU_INTENSET_TRIGGERED0_Msk;

        // dataout channel 
        NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)   |
                            (DATAOUT_CLK_IN                        << GPIOTE_CONFIG_PSEL_Pos)   |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
        
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN2_Msk;

        nrf_gpio_cfg_input(DATAOUT_DATA_IN, NRF_GPIO_PIN_NOPULL);

        // connect dataout with egu 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_EGU_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[2], 
                (uint32_t)&NRF_EGU0->TASKS_TRIGGER[0]
        ); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_EGU_CH);

        // counter gpiote enable 
        nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_AL_CH, AL_DATA_IN, NRF_GPIOTE_POLARITY_LOTOHI);
        nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_AL_CH);

        nrf_gpiote_event_configure(NRF_GPIOTE, GPIOTE_DL_CH, DL_DATA_IN, NRF_GPIOTE_POLARITY_LOTOHI);
        nrf_gpiote_event_enable(NRF_GPIOTE, GPIOTE_DL_CH);

} 

void timer_timeout_init(void) {
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_SRAMOUT_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_SRAMOUT_INST_IDX), 0, 0);
        
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_DATAOUT_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(TIMER_DATAOUT_INST_IDX), 0, 0);

        // sramout timeout 
        uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_sramout_inst.p_reg);
        nrfx_timer_config_t sramout_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
        sramout_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
        
        nrfx_err_t err = nrfx_timer_init(&timer_sramout_inst, &sramout_config, timer_sramout_handler); 
        if (err != NRFX_SUCCESS) {
                LOG_INF("sramout timer init failed");
        }
        LOG_INF("sramout timer init succeeded");
        
        nrfx_timer_clear(&timer_sramout_inst);

        uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_sramout_inst, 20); // 1s
        nrfx_timer_extended_compare(&timer_sramout_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

        // turn timer off initially 
        nrf_timer_task_trigger(timer_sramout_inst.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(timer_sramout_inst.p_reg, NRF_TIMER_TASK_CLEAR);

        // dataout timeout 
        base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_dataout_inst.p_reg);
        nrfx_timer_config_t dataout_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
        dataout_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
        
        err = nrfx_timer_init(&timer_dataout_inst, &dataout_config, timer_dataout_handler); 
        if (err != NRFX_SUCCESS) {
                LOG_INF("dataout timer init failed");
        }
        LOG_INF("dataout timer init succeeded");
        
        nrfx_timer_clear(&timer_dataout_inst);

        desired_ticks = nrfx_timer_us_to_ticks(&timer_dataout_inst, 20); // 1s
        nrfx_timer_extended_compare(&timer_dataout_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

        // turn timer off initially 
        nrf_timer_task_trigger(timer_dataout_inst.p_reg, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(timer_dataout_inst.p_reg, NRF_TIMER_TASK_CLEAR);
} 

void timer_counter_init(void) {
        // AL counter 
        nrf_timer_task_trigger(TIMER_AL_INST, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(TIMER_AL_INST, NRF_TIMER_TASK_CLEAR);

        nrf_timer_mode_set(TIMER_AL_INST, NRF_TIMER_MODE_COUNTER); 
        nrf_timer_bit_width_set(TIMER_AL_INST, NRF_TIMER_BIT_WIDTH_32);

        nrf_timer_task_trigger(TIMER_AL_INST, NRF_TIMER_TASK_START);

        // DL counter 
        nrf_timer_task_trigger(TIMER_DL_INST, NRF_TIMER_TASK_STOP);
        nrf_timer_task_trigger(TIMER_DL_INST, NRF_TIMER_TASK_CLEAR);

        nrf_timer_mode_set(TIMER_DL_INST, NRF_TIMER_MODE_COUNTER); 
        nrf_timer_bit_width_set(TIMER_DL_INST, NRF_TIMER_BIT_WIDTH_32);

        nrf_timer_task_trigger(TIMER_DL_INST, NRF_TIMER_TASK_START);
}

void ppi_inputs_init(void) {
        // low to high --> start timer 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_SRAMOUT_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[1], 
                nrfx_timer_task_address_get(&timer_sramout_inst, NRF_TIMER_TASK_CLEAR)
        ); 

        // low to high --> clear timer 
        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_SRAMOUT_CH, nrfx_timer_task_address_get(&timer_sramout_inst, NRF_TIMER_TASK_START)); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_SRAMOUT_CH); 

        // low to high --> start timer 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_DATAOUT_CH, 
                (uint32_t)&NRF_GPIOTE->EVENTS_IN[2], 
                nrfx_timer_task_address_get(&timer_dataout_inst, NRF_TIMER_TASK_CLEAR)
        ); 

        // low to high --> clear timer 
        nrf_ppi_fork_endpoint_setup(NRF_PPI, PPI_DATAOUT_CH, nrfx_timer_task_address_get(&timer_dataout_inst, NRF_TIMER_TASK_START)); 

        nrf_ppi_channel_enable(NRF_PPI, PPI_DATAOUT_CH);

        // counters 
        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_AL_CH, 
                nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_AL_CH)), 
                nrf_timer_task_address_get(TIMER_AL_INST, NRF_TIMER_TASK_COUNT)
        );

        nrf_ppi_channel_enable(NRF_PPI, PPI_AL_CH);

        nrf_ppi_channel_endpoint_setup(NRF_PPI, PPI_DL_CH, 
                nrf_gpiote_event_address_get(NRF_GPIOTE, nrf_gpiote_in_event_get(GPIOTE_DL_CH)), 
                nrf_timer_task_address_get(TIMER_DL_INST, NRF_TIMER_TASK_COUNT)
        );

        nrf_ppi_channel_enable(NRF_PPI, PPI_DL_CH);
}


/* Reset Functions ------------------------------------------------------------------*/
void reset_al_counter(void) {
        nrf_timer_task_trigger(TIMER_AL_INST, NRF_TIMER_TASK_CLEAR);
        ble_al_counter = 0; 
}

void reset_dl_counter(void) {
        nrf_timer_task_trigger(TIMER_DL_INST, NRF_TIMER_TASK_CLEAR);
        ble_dl_counter = 0;
} 

void reset_sramout(void) {
        done_rd_SRAM = 0; 
        sramout_count = 0; 
        memset((void *)sramout_buffer, 0, sizeof(sramout_buffer));
        memset(&ble_sramout_packet, 0, sizeof(ble_sramout_packet));
} 

void reset_dataout(void) {
        dataout_count = 0; 
        dataout_buffer = 0; 
        rtc_overflow_count = 0; 
        start_timestamp_rtc(); 
} 

/* AL/DL Update Functions ------------------------------------------------------------------*/
void update_al_counter(void) {
        nrf_timer_task_trigger(TIMER_AL_INST, NRF_TIMER_TASK_CAPTURE0);
        ble_al_counter = nrf_timer_cc_get(TIMER_AL_INST, NRF_TIMER_CC_CHANNEL0);
}

void update_dl_counter(void) {
        nrf_timer_task_trigger(TIMER_DL_INST, NRF_TIMER_TASK_CAPTURE0);
        ble_dl_counter = nrf_timer_cc_get(TIMER_DL_INST, NRF_TIMER_CC_CHANNEL0);
}


/* SRAMOUT and DATAOUT Process Functions  ------------------------------------------------------------------*/
bool get_sram_bit(volatile uint32_t *buffer, int bit_idx) {
    // word 0: bits 0-31, word 1: bits 32-63, etc.
    return (buffer[bit_idx >> 5] >> (bit_idx & 31)) & 0x01;
} 

void process_sramout(volatile uint32_t *buffer, int count) {

    //memset(&ble_sramout_packet, 0, sizeof(ble_sramout_packet));

    LOG_INF("sramout: 0x%08X %08X %08X %08X \n", 
         __RBIT(buffer[0]), __RBIT(buffer[1]), __RBIT(buffer[2]), __RBIT(buffer[3]));
    LOG_INF("count: %u \n", count); 

    // --- REPLICATING FPGA LOGIC CONDITIONS ---
    // FPGA: (SRAM_out[0]==1'b1 && counter_SRAM==16'd1)
    if (count == 2 && get_sram_bit(buffer, 0)) {
        done_rd_SRAM = 1;
    }
    // FPGA: (SRAM_out[8]==1'b0 && counter_SRAM==16'd9)
    else if (count == 10 && !get_sram_bit(buffer, 8)) {
        done_rd_SRAM = 1;
    }
    // FPGA: (SRAM_out[110]==1'b0 && counter_SRAM==16'd111)
    // Note: C count 111 is the 112th bit
    else if (count == 112 && !get_sram_bit(buffer, 110)) {
        done_rd_SRAM = 1;
    }

    // buffer[0] has bits that arrived first 
    uint32_t b0 = __RBIT(buffer[0]);
    uint32_t b1 = __RBIT(buffer[1]);
    uint32_t b2 = __RBIT(buffer[2]);
    uint32_t b3 = __RBIT(buffer[3]);

    memset(&ble_sramout_packet, 0, sizeof(ble_sramout_packet));
    if (done_rd_SRAM) {
        // Slice the 128-bit buffer into the 16-bit FPGA chunks
        ble_sramout_packet.SRAM_out6 = (uint16_t)(b0 >> 16);  // bits 0-15 reversed
        ble_sramout_packet.SRAM_out5 = (uint16_t)(b0 & 0xFFFF); // bits 16-31 reversed
        ble_sramout_packet.SRAM_out4 = (uint16_t)(b1 >> 16);  // bits 32-47 reversed
        ble_sramout_packet.SRAM_out3 = (uint16_t)(b1 & 0xFFFF); // bits 48-63 reversed
        ble_sramout_packet.SRAM_out2 = (uint16_t)(b2 >> 16);  // bits 64-79 reversed
        ble_sramout_packet.SRAM_out1 = (uint16_t)(b2 & 0xFFFF); // bits 80-95 reversed
        ble_sramout_packet.SRAM_out0 = (uint16_t)(b3 >> 16);

        bt_gatt_notify(current_conn, &spect_svc.attrs[8], &ble_sramout_packet, sizeof(ble_sramout_packet));
    } 

    done_rd_SRAM = 0; 
    sramout_count = 0; 
    memset((void *)sramout_buffer, 0, sizeof(sramout_buffer));

    LOG_INF("SRAM Data (LSB-Packed): \n");
    LOG_INF("sram_out6 :   0x%04X \n", ble_sramout_packet.SRAM_out6);
    LOG_INF("sram_out5:  0x%04X \n", ble_sramout_packet.SRAM_out5);
    LOG_INF("sram_out4:  0x%04X \n", ble_sramout_packet.SRAM_out4);
    LOG_INF("sram_out3: (96-111): 0x%04X \n", ble_sramout_packet.SRAM_out3);
    LOG_INF("sram_out2 :   0x%04X \n", ble_sramout_packet.SRAM_out2);
    LOG_INF("sram_out1:  0x%04X \n", ble_sramout_packet.SRAM_out1);
    LOG_INF("sram_out0:  0x%04X \n", ble_sramout_packet.SRAM_out0);
    LOG_INF("Total Bits: %d \n", sramout_count);
 
} 


int ble_notification_dataout(struct dataout_ble_packet *pkt, uint16_t len) {
        return bt_gatt_notify(current_conn, &spect_svc.attrs[15], pkt, len); 
}

void process_dataout(uint32_t dataout, int count, uint64_t timestamp) {
    // FPGA: counter_data0 == 16'd25 (which is the 26th bit)

    if (count == 26) {
        struct dataout_ble_packet new_pkt; 

        new_pkt.timestamp = timestamp; 
        
        // Extract the 26 bits. 
        // Note: dataout_buffer[25:0] matches your uint32_t bits 0 to 25.
        new_pkt.dataout = __RBIT(dataout) >> 6; // Mask only 26 bits

        // If you want to send this to your BLE FIFO:
        k_msgq_put(&fifo_data_out, &new_pkt, K_NO_WAIT);
        dataout_count = 0; 
        dataout_buffer = 0;

        uint32_t num_fifo = k_msgq_num_used_get(&fifo_data_out);
        if (num_fifo >= 5) {
                struct dataout_ble_packet send_pkt;
                for (int i = 0; i < 5; i++) {
                        k_msgq_get(&fifo_data_out, &send_pkt, K_NO_WAIT); 
                        int err = ble_notification_dataout(&send_pkt, sizeof(send_pkt)); 
                        if (err) LOG_ERR("dataout sending failed"); 
                }
        }
        
        LOG_INF("DataOut Packet Processed: 0x%08X", new_pkt.dataout);
    } else {
        // This handles cases where noise or partial packets occur
        LOG_WRN("DataOut rejected: count was %d, expected 26", count);
    }
}
