#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(uart_demo, LOG_LEVEL_INF);

static const struct device *uart;

static uint8_t rx_buffer[64]; 
static char sipo_done[] = "SIPO_DONE\n"; 
static char dac_done[] = "DAC_DONE\n";
static char sramout[128]; 

uint16_t sipo0, sipo1, sipo2, sipo3, sipo4, sipo5, sipo6, sipo7; 
uint16_t dac0, dac1, dac2; 
uint16_t done_wr_sipo, done_wr_dac; 
int board_address; 
int popdata; 

struct __packed dataout_ble_packet {
    uint32_t dataout; 
    uint64_t timestamp; 
};

static void process_command(char *buf, const struct device *dev) {
        if (strlen(buf) < 1) return; 

        switch(buf[0]) {
                case 'B': 
                        if (strncmp(buf, "B_ADDR:", 7) == 0) board_address = atoi(buf + 7); 
                        break; 
                case 'C':  
                        if (strcmp(buf, "CLK_TOGG") == 0) LOG_INF("CLK_TOGG");  
                        else if (strcmp(buf, "CH_RST") == 0) LOG_INF("CH_RST");  // needed
                        break; 
                case 'D': 
                        if (strncmp(buf, "DAC:", 4) == 0) LOG_INF("DAC");  // make sure to call reset_dac() first
                        // else if (strcmp(buf, "DAC_RST") == 0) reset_dac();
                        else if (strcmp(buf, "DLAL_RST") == 0) LOG_INF("DLAL_RST");
                        else if (strcmp(buf, "DATA_RD_RST") == 0) LOG_INF("DATA_RD_RST"); // think more
                        break; 
                case 'R': 
                        if (strcmp(buf, "RST_GLOBAL") == 0) LOG_INF("RST_GLOBAL");
                        break;
                case 'S': 
                        if (strncmp(buf, "SIPO:", 5) == 0) LOG_INF("SIPO"); // make sure to call reset_sipo() and reset_sramout() first 
                        // else if (strcmp(buf, "SIPO_RST") == 0) reset_sipo(); 
                        //else if (strcmp(buf, "SRAM_RD_RST") == 0) reset_sramout(); 
                        break;  
                default: 
                        LOG_WRN("Invalid GUI command"); 
                        break; 
        }
}

static void uart_cb(const struct device *dev, void *user_data)
{
    static int rx_ptr = 0;
    if (!uart_irq_update(dev)) return;

    if (uart_irq_rx_ready(dev)) {
        uint8_t byte;
        while (uart_fifo_read(dev, &byte, 1) > 0) {
            if (byte == '\n') {
                rx_buffer[rx_ptr] = '\0'; // Terminate string
                
                if (rx_ptr > 0) process_command(rx_buffer, dev);  
                rx_ptr = 0; // Reset for next command

            } else if (rx_ptr < (sizeof(rx_buffer) - 1)) {
                rx_buffer[rx_ptr++] = byte;
            }
        }
    }
}

static void uart_init(void) {
        /* Enable USB serial */
        int ret = usb_enable(NULL); 
        if (ret) {
                //printk("USB init failed: %d\n", ret);
                return 0;
        }

        uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm)); 
        if (!device_is_ready(uart)) {
                printk("CDC ACM not ready\n");
                return;
        }

        int dtr = 0;  
        while (!dtr) {
                uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr); 
                k_msleep(10);
        }

        // set callback function
        uart_irq_callback_set(uart, uart_cb);

        // enable uart rx interrupt 
        uart_irq_rx_enable(uart);

        LOG_INF("USB Serial Ready. Run Python script");

}

int main(void)
{       
        uart_init(); 

        struct dataout_ble_packet pkt1;
        struct dataout_ble_packet pkt; 
        pkt1.dataout = 60; 
        pkt1.timestamp = 20; 
        memcpy(&pkt, &pkt1, sizeof(pkt));

        // I need to process dataout --> need to change for more than one patch
        char tx_buf[64];

        //LOG_INF("USB Serial Ready. Run Python script");

        int current_board = 0; 

        while(1) {
                int len = snprintf(tx_buf, sizeof(tx_buf), "DATAOUT:%d,%u,%" PRIu64 "\n", 
                           current_board, pkt.dataout, pkt.timestamp);

                int sent = 0;
                while (sent < len) {
                int ret = uart_fifo_fill(uart, (uint8_t *)tx_buf + sent, len - sent);
                if (ret > 0) sent += ret;
                }
                pkt.dataout++; 
                pkt.timestamp++; 

                current_board = (current_board == 0) ? 1 : 0;
                
                k_msleep(2500);
        }

        return 0;
}
