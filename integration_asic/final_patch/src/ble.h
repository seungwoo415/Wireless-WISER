/**
 * @file ble.h
 * @author Archie Lee
 * @brief Header file for BLE patch logic.
 * @details Defines the public structures and function prototypes for 
 * BLE connection, advertisement, communication, and GATT server.
 * @version 1.0
 * @date 2026-01-23
 */

#ifndef BLE_H
#define BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- UUID --- */
#define SPECT_BASE_UUID BT_UUID_128_ENCODE(0x7231db4c, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)

extern struct bt_uuid_128 spect_uuid;
extern struct bt_uuid_128 sipo_uuid;  
extern struct bt_uuid_128 dac_uuid;
extern struct bt_uuid_128 sramout_uuid;
extern struct bt_uuid_128 al_uuid;
extern struct bt_uuid_128 dl_uuid;
extern struct bt_uuid_128 dataout_uuid;
extern struct bt_uuid_128 syscmd_uuid;

/* --- Function Prototypes --- */
/**
 * @brief Begins advertisement.
 */
void patch_ad_start(void);
 
#ifdef __cplusplus
}
#endif

#endif /* BLE_H */