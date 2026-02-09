/**
 * @file uart.c
 * @author Archie Lee
 * @brief Handles USB over UART logic to communicate with PC GUI. 
 * @details Implements initializing UART, UART callback, and processing input UART.   
 * @version 1.0
 * @date 2026-02-03
 */

/* Includes ------------------------------------------------------------------*/
#include "final_mb.h"

// logging 
LOG_MODULE_DECLARE(final_mb, LOG_LEVEL_INF);

/* Variables  ------------------------------------------------------------------*/
static uint8_t rx_buffer[128]; 
int board_address; 

/* UART Handle Functions ------------------------------------------------------------------*/
static void handle_sipo(char *buf) {
        // SIPO:s7,s6,...,s0
        int len = sscanf(buf + 5, "%hx,%hx,%hx,%hx,%hx,%hx,%hx,%hx", 
                           &sipo7, &sipo6, &sipo5, &sipo4, &sipo3, &sipo2, &sipo1, &sipo0);

        done_wr_sipo = 0; 
        // sramout_reset(); make patch reset when the sipo data arrives
        // sipo_reset(); make patch reset when the sipo data arrives  
        if (len == 8) {
                write_sipo(sipo7, sipo6, sipo5, sipo4, sipo3, sipo2, sipo1, sipo0); 
        }
        
}

static void handle_dac(char *buf) {
        // DAC:dac2,dac1,dac0
        int len = sscanf(buf + 4, "%hx,%hx,%hx", &dac2, &dac1, &dac0);

        done_wr_dac = 0; 
        // dac_reset(); make patch reset when the dac data arrives
        if (len == 3) {
                 write_dac(dac2, dac1, dac0); 
        }
        
}

/* UART Process Function ------------------------------------------------------------------*/
void process_command(char *buf, const struct device *dev) {
        if (buf == NULL || buf[0] == '\0') return;

        switch(buf[0]) {
                case 'B': 
                        if (strncmp(buf, "B_ADDR:", 7) == 0) board_address = atoi(buf + 7); 
                        break; 
                case 'C':  
                        //if (strcmp(buf, "CLK_TOGG") == 0) clk_toggle(); // don't think this is needed
                        if (strcmp(buf, "CH_RST") == 0) write_sys_cmd(6); // needed
                        break; 
                case 'D': 
                        if (strncmp(buf, "DAC:", 4) == 0) handle_dac(buf); // make sure to call reset_dac() first
                        // else if (strcmp(buf, "DAC_RST") == 0) reset_dac();
                        else if (strcmp(buf, "DLAL_RST") == 0) write_sys_cmd(4); 
                        else if (strcmp(buf, "DLAL_RQST") == 0) dl_read(); 
                        else if (strcmp(buf, "DATA_RD_RST") == 0) write_sys_cmd(5); // think more
                        break; 
                case 'R': 
                        if (strcmp(buf, "RST_GLOBAL") == 0) write_sys_cmd(7);
                        break;
                case 'S': 
                        if (strncmp(buf, "SIPO:", 5) == 0) handle_sipo(buf); // make sure to call reset_sipo() and reset_sramout() first 
                        // else if (strcmp(buf, "SIPO_RST") == 0) reset_sipo(); 
                        //else if (strcmp(buf, "SRAM_RD_RST") == 0) reset_sramout(); 
                        break;  
                default: 
                        LOG_WRN("Invalid GUI command"); 
                        break; 
        }
}

/* UART Callback Function ------------------------------------------------------------------*/
static void uart_cb(const struct device *dev, void *user_data)
{
    static int rx_ptr = 0;
    if (!uart_irq_update(dev)) return;

    if (uart_irq_rx_ready(dev)) {
        uint8_t byte;
        while (uart_fifo_read(dev, &byte, 1) > 0) {
            if (byte == '\n' || byte == '\r') {
                if (rx_ptr > 0){
                        rx_buffer[rx_ptr] = '\0'; // Terminate string
                        strncpy(process_command_work.payload, rx_buffer, sizeof(process_command_work.payload));
                        k_work_submit(&process_command_work.work);  
                } 
                rx_ptr = 0; // Reset for next command

            } else if (rx_ptr < (sizeof(rx_buffer) - 1)) {
                rx_buffer[rx_ptr++] = byte;
            }
        }
    }
}

/* Initialize UART Function ------------------------------------------------------------------*/
void initialize_uart(void) {
    /* Enable USB serial */
        int ret = usb_enable(NULL); 
        if (ret) {
                LOG_ERR("CDC ACM not ready\n");
                return;
        }
 
        if (!device_is_ready(uart)) {
                LOG_ERR("CDC ACM not ready\n");
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
}