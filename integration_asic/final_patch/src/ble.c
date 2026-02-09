/**
 * @file ble.c
 * @author Archie Lee
 * @brief Handles patch BLE logic. 
 * @details Implements BLE connection, advertisement, communication, and GATT server.  
 * @version 1.0
 * @date 2026-01-23
 */

/* Includes ------------------------------------------------------------------*/
#include "final_patch.h" 
#include "inputs.h"
#include "outputs.h" 

// logging 
LOG_MODULE_REGISTER(spect, LOG_LEVEL_INF);
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3

/* UUID ------------------------------------------------------------------*/
#define SPECT_BASE_UUID BT_UUID_128_ENCODE(0x7231db4c, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)

static struct bt_uuid_128 spect_uuid = BT_UUID_INIT_128(SPECT_BASE_UUID);
static struct bt_uuid_128 sipo_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4d, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)); 
static struct bt_uuid_128 dac_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4e, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 sramout_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4f, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 al_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db50, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 dl_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db51, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)); 
static struct bt_uuid_128 dataout_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db52, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 syscmd_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db53, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));

const struct bt_data ad[] = {
    /* Set advertising flags: General discoverable, BR/EDR not supported */
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    
    /* Advertise the 128-bit Service UUID so the Motherboard can filter for it */
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
        0xee, 0x47, 0x81, 0x34, 0x84, 0x2b, 0x9f, 0xbe, 
        0xf7, 0x4b, 0xed, 0x67, 0x4c, 0xdb, 0x31, 0x72),
    
};

/* Global Variables ------------------------------------------------------------------*/
struct bt_conn *current_conn;
struct sramout_ble_packet ble_sramout_packet;

/* BLE Connection Functions ------------------------------------------------------------------*/
void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)\n", err);
        return;
    }

    LOG_INF("Connected! Motherboard is now talking to us.\n");

    // erase for actual patch 
    nrf_gpio_pin_set(BLE_LED);
    
    // Save the connection reference
    current_conn = bt_conn_ref(conn);
} 

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    // erase for actual patch 
    nrf_gpio_pin_clear(BLE_LED); 
    
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    
    // CRITICAL: Start advertising again so the Motherboard can reconnect!
    // bt_le_adv_start(BT_LE_ADV_CONN, NULL, 0, NULL, 0);
    patch_ad_start();
}

// Register the callbacks with the system
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

/* BLE Callback Functions ------------------------------------------------------------------*/
static ssize_t sipo_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 16) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    
    sipo_reset(); 
    reset_sramout();  

    const uint8_t *data = (const uint8_t *)buf; 

    // LOG_INF("First byte: 0x%02X \n", data[0]);
    // LOG_INF("Second byte: 0x%02X \n", data[1]);
    // LOG_INF("Second byte: 0x%02X \n", data[2]);
    // LOG_INF("Sixth byte: 0x%02X \n", data[6]);
    // LOG_INF("Twelvth byte: 0x%02X \n", data[12]);
    // LOG_INF("Last byte: 0x%02X \n", data[15]);

    // prepare sipo_buffer 
    prepare_sipo_buffer(data); 

    // send the sipo data 
    sipo_trigger_transfer(); 

    // start the sipo transfer for now lets assume sending the sipo data means sipo_trigger 
    LOG_INF("SIPO Data Received! Sending the SIPO data with first byte: 0x%02X and last byte: 0x%02X", data[0], data[len-1]); 

    while(!sipo_done) {
        k_usleep(10); 
    }

    // sipo is done 
    done_wr_sipo = 1; 

    // notify motherboard that sipo is done 
    bt_gatt_notify(conn, attr, (const void *)&done_wr_sipo, sizeof(done_wr_sipo)); 

    LOG_INF("SIPO done! Sent done_wr_sipo to motherboard \n");

    return len;
} 

// dac write callback 
static ssize_t dac_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 5) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Copy incoming wireless data to our local buffer
    //memcpy(ble_dac_buffer, buf, len);

    const uint8_t *data = (const uint8_t *)buf;

    // send dac data 
    dac_reset(); 
    dac_i2c_write(data, len); 

    // start the dac transfer for now lets assume sending the dac data means dac_trigger 
    LOG_INF("DAC Data Received! Sending the DAC data with first byte: 0x%02X and last byte: 0x%02X", data[0], data[len-1]); 

    // dac is done 
    done_wr_dac = 1; 
                    
    // notify motherboard that sipo is done 
    bt_gatt_notify(conn, attr, &done_wr_dac, sizeof(done_wr_dac));

    LOG_INF("DAC done! Sent done_wr_dac to motherboard \n");

    return len;
} 

// sramout read callback 
static ssize_t sramout_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{

    // This helper function handles the offset and length logic for you 
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &ble_sramout_packet, sizeof(ble_sramout_packet));
    
} 

// al read callback 
static ssize_t al_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{

    update_al_counter(); 
    
    // This helper function handles the offset and length logic for you 
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &ble_al_counter, sizeof(ble_al_counter));
} 

// dl read callback 
static ssize_t dl_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{

    update_dl_counter();
 
    // This helper function handles the offset and length logic for you 
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &ble_dl_counter, sizeof(ble_dl_counter));
}

// dataout read callback 
static ssize_t dataout_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    
    // uint32_t val;

    // // Attempt to pop ONE word from the FIFO
    // if (k_msgq_get(&fifo_data_out, &val, K_NO_WAIT) == 0) {
    //     // Prepare the 4-byte buffer
    //     uint8_t temp_buf[4];
    //     temp_buf[0] = (val >> 24) & 0xFF;
    //     temp_buf[1] = (val >> 16) & 0xFF;
    //     temp_buf[2] = (val >> 8) & 0xFF;
    //     temp_buf[3] = val & 0xFF;

    //     LOG_INF("Dataout read request: Popping 0x%08X", val);
    //     return bt_gatt_attr_read(conn, attr, buf, len, offset, temp_buf, sizeof(temp_buf));
    // } else {
    //     // FIFO is empty, return 0 or an error code
    //     LOG_WRN("Dataout read request: FIFO Empty");
    //     return 0; 
    // }

    struct dataout_ble_packet pkt; 
    if (k_msgq_get(&fifo_data_out, &pkt, K_NO_WAIT) == 0){
        LOG_INF("Dataout read request: Popping 0x%08X", pkt.dataout);
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
    } else {
        // FIFO is empty, return 0 or an error code
        LOG_WRN("Dataout read request: FIFO Empty");
        return 0; 
    }
}

// sys command callback 
static ssize_t syscmd_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    uint8_t command = ((uint8_t *)buf)[0];
    
    switch (command) {
        case 0x01: // reset sipo
            done_wr_sipo = 0; 
            //memset((void*)sipo_buffer, 0, sizeof(sipo_buffer));
            LOG_INF("reset sipo \n");
            break;
        
        case 0x02: // reset dac
            dac_reset(); 
            //memset((void*)dac_buffer, 0, sizeof(dac_buffer));
            LOG_INF("reset dac \n"); 
            break; 
        
        case 0x03: // reset sramout   
            reset_sramout(); 
            LOG_INF("reset sramout \n"); 
            break; 
        
        case 0x04: // reset aldl
            reset_al_counter(); 
            reset_dl_counter();  
            LOG_INF("reset aldl \n");  
            break; 
        
        case 0x05: // reset dataout + start rtc timer
            reset_dataout(); 
            LOG_INF("reset dataout \n"); 
            break;
        
        case 0x06: // reset chip 
            LOG_INF("reset chip \n"); 
            chip_reset(); 
            break;
        
        case 0x07: // reset global
            LOG_INF("reset global \n"); 
            break;

        default:
            LOG_INF("Unknown Command: 0x%02X\n", command);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return len; 

}

void patch_ad_start(void) {
    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Adv start failed (err %d)", err);
    }
}

/* GATT Server ------------------------------------------------------------------*/
BT_GATT_SERVICE_DEFINE(spect_svc,
        BT_GATT_PRIMARY_SERVICE(&spect_uuid),
        BT_GATT_CHARACTERISTIC(&sipo_uuid.uuid,
                    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,    // Motherboard can write
                    BT_GATT_PERM_WRITE,    // Requires write permission
                    NULL, sipo_write_cb, NULL), 
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
        BT_GATT_CHARACTERISTIC(&dac_uuid.uuid,
                    BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,    // Motherboard can write
                    BT_GATT_PERM_WRITE,    // Requires write permission
                    NULL, dac_write_cb, NULL), 
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
        BT_GATT_CHARACTERISTIC(&sramout_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    sramout_read_cb, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
        BT_GATT_CHARACTERISTIC(&al_uuid.uuid,
                    BT_GATT_CHRC_READ,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    al_read_cb, NULL, NULL),
        BT_GATT_CHARACTERISTIC(&dl_uuid.uuid,
                    BT_GATT_CHRC_READ,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    dl_read_cb, NULL, NULL),
        BT_GATT_CHARACTERISTIC(&dataout_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    dataout_read_cb, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        BT_GATT_CHARACTERISTIC(&syscmd_uuid.uuid,
                    BT_GATT_CHRC_WRITE,    
                    BT_GATT_PERM_WRITE,    
                    NULL, syscmd_write_cb, NULL)
);


