#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

// external flash partitions
//#define NVS_PARTITION		 external_storage_partition 
//#define NVS_PARTITION_DEVICE	 FIXED_PARTITION_DEVICE(NVS_PARTITION)
//#define NVS_PARTITION_OFFSET	 FIXED_PARTITION_OFFSET(NVS_PARTITION) 

/* Get the node identifier for the partition label in your overlay */
#define NVS_PARTITION_NODE DT_NODELABEL(external_storage_partition)

/* Use the DT_FLASH_AREA macros to get the device and offset */
#define NVS_PARTITION_DEVICE DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(NVS_PARTITION_NODE))
#define NVS_PARTITION_OFFSET DT_REG_ADDR(NVS_PARTITION_NODE)

// id for nvs (each data == id) 
#define FS_ID 1

static struct nvs_fs fs = {
	.flash_device = NVS_PARTITION_DEVICE,
	.offset = NVS_PARTITION_OFFSET,
        .sector_size = 4096, 
        .sector_count = 32,
};

int main(void) {
        int err; 
        
        // enable USB 
        err= usb_enable(NULL);
        if (err) {
                printk("USB init failed: %d\n", err);
                return 0;
        }
        k_msleep(500);
        printk("USB console initialized.\n");
        k_msleep(5000);

        if (!device_is_ready(fs.flash_device)) {
                printk("Flash device %s is not ready\n", fs.flash_device->name);
                return 0;
        }

        err = nvs_mount(&fs); 
        if (err) {
                printk("nvs_mount failed"); 
                return 0; 
        }

        // array to be stored 
        int arr[5] = {85, 92, 78, 95, 88};
        
        // write the array 
        err = nvs_write(&fs, FS_ID, arr, sizeof(arr)); 
        if (err < 0) {
                printk("nvs_write failed");
        } 
        
        int read_buf[5]; 
        /// read the array
        err = nvs_read(&fs, FS_ID, read_buf, sizeof(read_buf)); 
        if (err < 0) {
                printk("nvs_read failed");
        } else {
                printk("read data1: %d %d %d %d %d", arr[0], arr[1], arr[2],  arr[3], arr[4]); 
        }

        // modify the array
        arr[0] = 20;

        // write the array 
        err = nvs_write(&fs, FS_ID, arr, sizeof(arr)); 
        if (err < 0) {
                printk("nvs_write failed");
        }

        /// read the array
        err = nvs_read(&fs, FS_ID, read_buf, sizeof(read_buf)); 
        if (err < 0) {
                printk("nvs_read failed");
        } else {
                printk("modified read data2: %d %d %d %d %d", arr[0], arr[1], arr[2],  arr[3], arr[4]); 
        }

        return 0; 

}