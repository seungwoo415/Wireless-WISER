#include "final_mb.h" 
#include "ble.h"
#include "uart.h"

// logging
LOG_MODULE_REGISTER(final_mb, LOG_LEVEL_INF);

const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
        BT_UUID_128_ENCODE(0x7231db4c, 0x67ed, 0x4bf7, 0xbe9f, 0x2b84348147ee)),
};

struct bt_le_scan_param scan_param = {
    .type     = BT_LE_SCAN_TYPE_PASSIVE,
    .options  = BT_LE_SCAN_OPT_NONE,
    .interval = BT_GAP_MS_TO_SCAN_INTERVAL(100), 
    .window   = BT_GAP_MS_TO_SCAN_WINDOW(50),
};

// UART device 
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm));

// process_command work handler 
struct command_work_t process_command_work;

void process_command_work_handler(struct k_work *work_item)
{
    struct command_work_t *cmd = CONTAINER_OF(work_item, struct command_work_t, work);
    
    // Now we call process_command in a safe, low-priority thread context
    process_command(cmd->payload, uart); 
}

// initialize the motherboard
void initialize_mb(void) {
    int err;

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    // START SCANNING HERE
    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        return;
    }

    printk("Scanning started... Looking for Patch.\n");

    // initialize workers 
    k_work_init(&process_command_work.work, process_command_work_handler);
}

int main(void)
{
        initialize_mb(); 

        initialize_uart(); 

        k_msleep(5000); 

        // if (current_conn && al_data_handle != 0) {
        //         LOG_INF("Starting AL Read...");
        //         al_read(); 
        // } else {
        //         LOG_ERR("AL Read skipped: Not connected or handle not found");
        // }

        // k_msleep(1000);

        // if (current_conn && dl_data_handle != 0) {
        //         LOG_INF("Starting DL Read...");
        //         dl_read(); 
        // } 

        // k_msleep(200); 

        // write_sipo(0xF000, 0x0000, 0x0000, 0x0000, 
        //                 0x0000, 0x0000, 0x000F, 0x0000);
        

        while(1) {
                k_sleep(K_FOREVER); 
        }
        
        return 0; 
}
