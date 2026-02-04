#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h> 

static const struct device *uart;

// receive data
static uint8_t foo;

static bool tx_pending; 

/* UART ISR callback */
static void uart_cb(const struct device *dev, void *user_data)
{
    // prepare for ready functions 
    if (!uart_irq_update(dev)) return; 

    if (uart_irq_rx_ready(dev)) {

        int len = uart_fifo_read(dev, &foo,  1);
        if (len > 0) {
            foo++; 
            uart_irq_tx_enable(dev);
            tx_pending = true; 
        }
    }

    /* Return back to the terminal */
    if (uart_irq_tx_ready(dev)) {
        uart_fifo_fill(dev, &foo, 1);
        tx_pending = false; 
        uart_irq_tx_disable(dev);
    }
}

int main(void)
{
    int ret;

    // declare usb device node 
    uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm)); 
    if (!device_is_ready(uart)) {
        printk("CDC ACM not ready\n");
        return 0;
    }

    /* Enable USB serial */
    ret = usb_enable(NULL); 
    if (ret) {
        printk("USB init failed: %d\n", ret);
        return 0;
    }

    // data terminal ready: control signal sent by the pc to say that it is ready 
    int dtr = 0;  
    while (!dtr) {
        uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr); 
        k_msleep(10);
    }

   // set callback function 
   uart_irq_callback_user_data_set(uart, uart_cb, &foo);

   // enable uart rx interrupt 
   uart_irq_rx_enable(uart); 

   while (1) {
    k_sleep(K_FOREVER); 
   }

}
