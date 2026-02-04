#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// logging 
LOG_MODULE_REGISTER(spect, LOG_LEVEL_INF);
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3

// define unique IDs (UUIDs) 
// base UUID 
#define SPECT_BASE_UUID BT_UUID_128_ENCODE(0x7231db4c, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)

static struct bt_uuid_128 spect_uuid = BT_UUID_INIT_128(SPECT_BASE_UUID);
static struct bt_uuid_128 sipo_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4d, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)); 
static struct bt_uuid_128 dac_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4e, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 sramout_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db4f, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 al_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db50, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 dl_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db51, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)); 
static struct bt_uuid_128 dataout_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db52, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));
static struct bt_uuid_128 syscmd_uuid  = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7231db53, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee));

// buffers 
static uint8_t sipo_buffer[16]; 
static uint8_t dac_buffer[6]; 
static uint8_t SRAM_out_buffer[14] = {0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF};  // might need to change this 
static uint32_t al_counter, dl_counter;  
static uint8_t dataout_buffer[4]; 

// fifo buffer 
K_MSGQ_DEFINE(fifo_data_out, sizeof(uint32_t), 32, 4);

// done signals 
static uint8_t done_wr_sipo = 0; 
static uint8_t done_wr_dac = 0;
static uint8_t sipo_trigger = 0;

static struct bt_conn *current_conn;

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)\n", err);
        return;
    }

    LOG_INF("Connected! Motherboard is now talking to us.\n");
    
    // Save the connection reference
    current_conn = bt_conn_ref(conn);
} 

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    
    // CRITICAL: Start advertising again so the Motherboard can reconnect!
    bt_le_adv_start(BT_LE_ADV_CONN, NULL, 0, NULL, 0);
}

// Register the callbacks with the system
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

// sipo write callback 
static ssize_t sipo_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len > sizeof(sipo_buffer)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Copy incoming wireless data to our local buffer
    memcpy(sipo_buffer, buf, len);

    const uint8_t *data = (const uint8_t *)buf;

    // start the sipo transfer for now lets assume sending the sipo data means sipo_trigger 
    LOG_INF("SIPO Data Received! Sending the SIPO data with first byte: 0x%02X and last byte: 0x%02X", data[0], data[len-1]); 

    // sipo is done 
    done_wr_sipo = 1; 

    // notify motherboard that sipo is done 
    bt_gatt_notify(conn, attr, &done_wr_sipo, sizeof(done_wr_sipo)); 

    LOG_INF("SIPO done! Sent done_wr_sipo to motherboard \n");

    return len;
} 

// dac write callback 
static ssize_t dac_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len > sizeof(dac_buffer)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // Copy incoming wireless data to our local buffer
    memcpy(dac_buffer, buf, len);

    const uint8_t *data = (const uint8_t *)buf;

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
    return bt_gatt_attr_read(conn, attr, buf, len, offset, SRAM_out_buffer, sizeof(SRAM_out_buffer));
} 

// al read callback 
static ssize_t al_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    
    // This helper function handles the offset and length logic for you 
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &al_counter, sizeof(al_counter));
} 

// dl read callback 
static ssize_t dl_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    
    // This helper function handles the offset and length logic for you 
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &dl_counter, sizeof(dl_counter));
}

// dataout read callback 
static ssize_t dataout_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    
    uint32_t val;
    
    // Attempt to pop ONE word from the FIFO
    if (k_msgq_get(&fifo_data_out, &val, K_NO_WAIT) == 0) {
        // Prepare the 4-byte buffer
        uint8_t temp_buf[4];
        temp_buf[0] = (val >> 24) & 0xFF;
        temp_buf[1] = (val >> 16) & 0xFF;
        temp_buf[2] = (val >> 8) & 0xFF;
        temp_buf[3] = val & 0xFF;

        LOG_INF("Read request: Popping 0x%08X", val);
        return bt_gatt_attr_read(conn, attr, buf, len, offset, temp_buf, sizeof(temp_buf));
    } else {
        // FIFO is empty, return 0 or an error code
        LOG_WRN("Read request: FIFO Empty");
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
            memset((void*)sipo_buffer, 0, sizeof(sipo_buffer));
            LOG_INF("reset sipo \n");
            break;
        
        case 0x02: // reset dac
            done_wr_dac = 0; 
            memset((void*)dac_buffer, 0, sizeof(dac_buffer));
            LOG_INF("reset dac \n"); 
            break; 
        
        case 0x03: // reset sramout  
            memset((void*)SRAM_out_buffer, 0, sizeof(SRAM_out_buffer)); 
            LOG_INF("reset sramout \n"); 
            break; 
        
        case 0x04: // reset aldl
            al_counter = 0; 
            dl_counter = 0; 
            LOG_INF("reset aldl \n");  
            break; 
        
        case 0x05: // reset dataout 
            LOG_INF("reset dataout \n"); 
            break; 

        // case 0x02: // trigger sipo
        //     printk("trigger sipo\n");
        //     break;

        default:
            LOG_INF("Unknown Command: 0x%02X\n", command);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return len; 

}

// register the service and characteristics
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
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    al_read_cb, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        BT_GATT_CHARACTERISTIC(&dl_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    dl_read_cb, NULL, NULL),
        BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        BT_GATT_CHARACTERISTIC(&dataout_uuid.uuid,
                    BT_GATT_CHRC_READ,    // Motherboard can read
                    BT_GATT_PERM_READ,    // Requires read permission
                    dataout_read_cb, NULL, NULL),
        BT_GATT_CHARACTERISTIC(&syscmd_uuid.uuid,
                    BT_GATT_CHRC_WRITE,    
                    BT_GATT_PERM_WRITE,    
                    NULL, syscmd_write_cb, NULL)
);

// sramout pseudo processor 
static void sramout_processor(void) {
    if (!current_conn) {
        LOG_ERR("no connection"); 
        return; 
    }

    LOG_INF("processing SRAMout input"); 
    
    k_usleep(10); 
    LOG_INF("processing SRAMout input done");  

    // notify the motherboard 
    bt_gatt_notify(current_conn, &spect_svc.attrs[8], SRAM_out_buffer, sizeof(SRAM_out_buffer));
    LOG_INF("sramout notification sent"); 
    
}

int main(void) {
        
        LOG_INF("Starting patch example");
        
        int err; 

        // initialize bluetooth 
        err = bt_enable(NULL); 
        if (err) {
            LOG_ERR("Bluetooth init failed \n"); 
            return 0; 
        } 
        LOG_INF("BT success");
        // start advertising 
        err = bt_le_adv_start(BT_LE_ADV_CONN, NULL, 0, NULL, 0); 
        if (err) {
            LOG_ERR("Advertising failed to start \n"); 
            return 0; 
        } 

        LOG_INF("Advertising successfully started");

        al_counter = 20; 
        dl_counter = 500; 

        uint32_t dataout1 = 20; 
        uint32_t dataout2 = 60;
        k_msgq_put(&fifo_data_out, &dataout1, K_NO_WAIT);
        k_msgq_put(&fifo_data_out, &dataout2, K_NO_WAIT);

        while (1) {
            if (current_conn) {
                sramout_processor(); 
                al_counter++; 
                bt_gatt_notify(current_conn, &spect_svc.attrs[11], &al_counter, sizeof(al_counter));
                dl_counter++; 
                bt_gatt_notify(current_conn, &spect_svc.attrs[14], &dl_counter, sizeof(dl_counter));
            }
            k_sleep(K_SECONDS(5)); 
        }
        
}

