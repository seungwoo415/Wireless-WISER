#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

static const struct device *uart;

/* UART ISR callback */
static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {

        int len = uart_fifo_read(dev, &c, 1);
        if (len > 0) {
            printk("Received: %c\n", c);

            /* Echo back to terminal */
            uart_fifo_fill(dev, &c, 1);

            /* Test command */
            if (c == 'h') {
                char msg[] = "hello!\r\n";
                uart_fifo_fill(dev, msg, strlen(msg));
            }
        }
    }
}

int main(void)
{
    int ret;

    /* Enable USB serial */
    ret = usb_enable(NULL); 
    if (ret) {
        printk("USB init failed: %d\n", ret);
        return 0;
    }

    k_sleep(K_MSEC(300));

    uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm)); 

    if (!device_is_ready(uart)) {
        printk("CDC ACM not ready\n");
        return 0;
    }

    /* Wait for macOS terminal to open tty.usbmodemXXX */
    uint32_t dtr = 0; 
    while (uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr) == 0) {
        if (dtr) break;
        k_sleep(K_MSEC(100));
    }

    printk("Host connected.\n");

    /* Enable UART interrupt mode */
    uart_irq_callback_user_data_set(uart, uart_cb, NULL);
    uart_irq_rx_enable(uart);

    /* Send a startup message */
    char msg[] = "Connected. Type something:\r\n";
    uart_fifo_fill(uart, msg, strlen(msg));

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
