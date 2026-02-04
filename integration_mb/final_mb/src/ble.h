/**
 * @file ble.h
 * @author Archie Lee
 * @brief Header file for BLE motherboard logic.
 * @details Defines the public structures and function prototypes for 
 * BLE connection, scanning, communication, and subscriptions.
 * @version 1.0
 * @date 2026-02-03
 */

#ifndef BLE_H
#define BLE_H

#ifdef __cplusplus
extern "C" {
#endif


/* --- UUID --- */
extern struct bt_uuid_128 spect_uuid;
extern struct bt_uuid_128 sipo_uuid;  
extern struct bt_uuid_128 dac_uuid;
extern struct bt_uuid_128 sramout_uuid;
extern struct bt_uuid_128 al_uuid;
extern struct bt_uuid_128 dl_uuid;
extern struct bt_uuid_128 dataout_uuid;
extern struct bt_uuid_128 syscmd_uuid;

/* --- Handle Variables --- */
extern uint16_t sipo_data_handle; 
extern uint16_t sipo_notify_handle;
extern uint16_t dac_data_handle; 
extern uint16_t dac_notify_handle; 
extern uint16_t sramout_data_handle;
extern uint16_t sramout_notify_handle;  
extern uint16_t al_data_handle; 
extern uint16_t dl_data_handle; 
extern uint16_t dataout_data_handle; 
extern uint16_t syscmd_data_handle;

/* --- Function Prototypes --- */

/**
 * @brief Triggers a GATT Read for AL counter.
 */
void al_read(void);

/**
 * @brief Triggers a GATT Read for the DL counter.
 */
void dl_read(void);

/**
 * @brief Triggers a GATT Read for Dataout.
 */
void dataout_read(void);

/**
 * @brief Packs 126-bit SIPO configuration and writes it to the Patch.
 */
void write_sipo(uint16_t s7, uint16_t s6, uint16_t s5, uint16_t s4, 
                uint16_t s3, uint16_t s2, uint16_t s1, uint16_t s0); 

/**
 * @brief Packs 36-bit DAC I2C data and writes it to the Patch.
 */
void write_dac(uint16_t d2, uint16_t d1, uint16_t d0); 

/**
 * @brief Writes system command to the patch.
 */
void write_sys_cmd(uint8_t command); 

/**
 * @brief Creates connection when device is found.
 */ 
void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */