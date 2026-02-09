/**
 * @file final_patch.h
 * @author Archie Lee
 * @brief Header file for patch.
 * @details Defines includes, pin configurations, peripheral instances,  
 * peripheral channels, and global variables.
 * @version 1.0
 * @date 2026-01-23
 */

#ifndef FINAL_PATCH_H
#define FINAL_PATCH_H

/* Includes ------------------------------------------------------------------*/
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <nrfx_pwm.h>
#include <nrfx_timer.h>

#include <hal/nrf_pwm.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_egu.h>
#include <hal/nrf_rtc.h>

#include <zephyr/irq.h>

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// pin configurations 
#define SRAMOUT_CLK_IN 27 //27 
#define SRAMOUT_DATA_IN 26 //26
#define DATAOUT_CLK_IN 4 //4
#define DATAOUT_DATA_IN 5 //5
#define AL_DATA_IN 24
#define DL_DATA_IN 15

#define DAC_SDA_PIN 12
#define DAC_SCL_PIN 11
#define SIPO_CLK_OUT 8
#define SIPO_DATA_OUT 6
#define CLK_1MHZ_OUT 14 
#define CLK_BASE_OUT 13
#define CH_RST NRF_GPIO_PIN_MAP(1, 8)

#define BLE_LED NRF_GPIO_PIN_MAP(1, 10)

// hardware instance indicies 
#define GPIOTE_INST_IDX 0

#define TIMER_SRAMOUT_INST_IDX 4
#define TIMER_DATAOUT_INST_IDX 1
#define TIMER_AL_INST NRF_TIMER2 
#define TIMER_DL_INST NRF_TIMER3

#define PWM_CLK_BASE_INST_IDX 0
#define PWM_SIPO_INST_IDX 1
#define PWM_CLK_1MHZ_INST_IDX 2

// gpiote channels 
#define GPIOTE_BASE_CLK_CH 0 
#define GPIOTE_SRAMOUT_CH 1
#define GPIOTE_DATAOUT_CH 2
#define GPIOTE_AL_CH 3
#define GPIOTE_DL_CH 4 

// ppi channels 
#define PPI_SRAMOUT_CH NRF_PPI_CHANNEL0 
#define PPI_DATAOUT_CH NRF_PPI_CHANNEL1
#define PPI_EGU_CH NRF_PPI_CHANNEL2
#define PPI_SIPO_CH NRF_PPI_CHANNEL3
#define PPI_CLK_1MHZ_CH NRF_PPI_CHANNEL4 
#define PPI_AL_CH NRF_PPI_CHANNEL5 
#define PPI_DL_CH NRF_PPI_CHANNEL6

// pin mask 
#define SRAMOUT_PIN_MASK (1 << SRAMOUT_DATA_IN)
#define DATAOUT_PIN_MASK (1 << DATAOUT_DATA_IN)

// data length 
#define SIPO_DATA_LEN 126

// hardware instances
extern const nrfx_timer_t timer_sramout_inst; 
extern const nrfx_timer_t timer_dataout_inst;
extern const nrfx_pwm_t pwm_sipo_instance;
extern const nrfx_pwm_t pwm_clk_base_instance;
extern const nrfx_pwm_t pwm_clk_1mhz_instance;

// global variables 
extern volatile uint32_t sramout_buffer[4]; 
extern volatile uint32_t dataout_buffer; 
extern volatile int sramout_count;
extern volatile int dataout_count; 
extern nrf_pwm_values_grouped_t sipo_buffer[SIPO_DATA_LEN]; 
extern nrf_pwm_values_common_t clk_base_value;
extern nrf_pwm_values_common_t clk_1mhz_value;
extern volatile uint32_t rtc_overflow_count; 

// dataout fifo buffer 
extern struct k_msgq fifo_data_out;

// done signals 
extern volatile bool sipo_done; 
extern volatile bool dac_i2c_done;

extern volatile uint32_t ble_al_counter;
extern volatile uint32_t ble_dl_counter;
extern volatile uint8_t done_wr_sipo;
extern volatile uint8_t done_wr_dac; 

extern volatile uint8_t done_rd_SRAM; 

// ble data 
struct __packed sramout_ble_packet {
    uint16_t SRAM_out0;
    uint16_t SRAM_out1;
    uint16_t SRAM_out2;
    uint16_t SRAM_out3;
    uint16_t SRAM_out4;
    uint16_t SRAM_out5;
    uint16_t SRAM_out6;
}; 

extern struct sramout_ble_packet ble_sramout_packet;

struct __packed dataout_ble_packet {
    uint32_t dataout; 
    uint64_t timestamp; 
}; 

extern struct bt_conn *current_conn;

extern const struct bt_gatt_service_static spect_svc;

extern const struct bt_data ad[2]; 

// pwm sequences 
extern nrf_pwm_sequence_t seq_sipo;   
extern nrf_pwm_sequence_t seq_clk_base;
extern nrf_pwm_sequence_t seq_clk_1mhz; 

// workers 
extern struct k_work sramout_work;
extern struct k_work dataout_work;

#ifdef __cplusplus
}
#endif

#endif /* FINAL_PATCH_H */