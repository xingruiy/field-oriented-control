#pragma once

/* Interactive debug CLI over USART3 (115200 baud, PB10/PB11).
 *
 * RX uses single-byte interrupt mode (HAL_UART_Receive_IT). Characters are
 * accumulated into a line buffer; the command is dispatched when CR or LF is
 * received. The full line is echoed back on dispatch.
 * Enable local echo in your terminal (e.g. picocom --echo) to see input live.
 *
 * cli_init()    — arm RX interrupt, print banner.
 * cli_process() — call from main loop; non-blocking, dispatches completed lines.
 * cli_print()   — blocking UART transmit of a NUL-terminated string (also used
 *                 by the fault layer to report asynchronously).
 *
 * Type 'help' at the prompt for the full command list.
 */

void cli_init(void);
void cli_process(void);
void cli_print(const char *str);
