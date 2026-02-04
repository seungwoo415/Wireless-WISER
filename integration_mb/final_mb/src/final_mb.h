/**
 * @file final_mb.h
 * @author Archie Lee
 * @brief Header file for motherboard.
 * @details Defines includes, data buffers, devices, and global variables.
 * @version 1.0
 * @date 2026-02-03
 */

#ifndef FINAL_PATCH_H
#define FINAL_PATCH_H

/* Includes ------------------------------------------------------------------*/
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of Patches ------------------------------------------------------------------*/
#define NUM_PATCHES 1

/* Global Data Buffers ------------------------------------------------------------------*/
extern uint16_t data_deq_0; 
extern uint16_t data_deq_1; 
extern uint32_t al_counter; 
extern uint32_t dl_counter; 
extern struct sramout_ble_packet ble_sramout_packet;
extern uint16_t dac2, dac1, dac0; 
extern uint16_t sipo7, sipo6, sipo5, sipo4, sipo3, sipo2, sipo1, sipo0;

extern uint8_t done_wr_sipo; 
extern uint8_t done_wr_dac; 
extern uint16_t SRAM_rd_reset; 
extern uint16_t dlal_reset; 
extern uint16_t data_rd_reset;
extern uint16_t sipo_reset; 

extern int board_address;

/* UART  ------------------------------------------------------------------*/
extern const struct device *uart; 

/* BLE Variables ------------------------------------------------------------------*/
extern struct bt_conn *current_conn;
extern const struct bt_data ad[2]; 
extern struct bt_le_scan_param scan_param;

struct __packed sramout_ble_packet {
    uint16_t SRAM_out0;
    uint16_t SRAM_out1;
    uint16_t SRAM_out2;
    uint16_t SRAM_out3;
    uint16_t SRAM_out4;
    uint16_t SRAM_out5;
    uint16_t SRAM_out6;
};

struct __packed dataout_ble_packet {
    uint32_t dataout; 
    uint64_t timestamp; 
}; 

/* K_WORK  ------------------------------------------------------------------*/
struct command_work_t {
    struct k_work work;
    char payload[128];
};

extern struct command_work_t process_command_work;

#ifdef __cplusplus
}
#endif

#endif /* FINAL_MB_H */