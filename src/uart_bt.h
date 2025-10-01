#ifndef UART_BT_H
#define UART_BT_H


#include <stdint.h>


// Initialise Bluetooth UART (USART1, PA9/PA10)
void uart_bt_init(void);

/*
// Thread-safe printf wrapper for Bluetooth UART
int __wrap_printf_bt(char *fmt, ...);
*/

// Low-level I/O putchar/getchar for Bluetooth UART



// Interrupt handlers (used internally)
void _uart_bt_rxne_isr(void);
void _uart_bt_txe_isr(void);
void _uart_bt_error(void);


#endif // UART_BT_H