/**
 * @file uart.h
 * @author Archie Lee
 * @brief Header file for USB over UART logic to communicate with PC GUI. 
 * @details Defines initializing UART, UART callback, and processing input UART.   
 * @version 1.0
 * @date 2026-02-03
 */

#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- Function Prototypes --- */
/**
 * @brief Initializes UART on Motherboard.
 */
void initialize_uart(void); 

/**
 * @brief Processes UART inputs from PC GUI.
 */
void process_command(char *buf, const struct device *dev); 

#ifdef __cplusplus
}
#endif

#endif /* UART_H */