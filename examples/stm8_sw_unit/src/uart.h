#pragma once

#include <stdint.h>
/* 
UARTDIV = fSYSCLK / BaudRate
Baud Rate Table
At 16 MHz system clock
Baud	UARTDIV(Hex)	        BRR2 (hi4:lo4)	        BRR1	Notes
9600	1667    (0x0683)	    (0x0:0x3) → 0x03	    0x68	Accurate
19200	833     (0x0341)	    (0x0:0x1) → 0x01	    0x34	Accurate
38400	417     (0x01A1)	    (0x0:0x1) → 0x01	    0xA1	Accurate
57600	278     (0x0116)	    (0x0:0x6) → 0x06	    0x11	Small error
115200	139     (0x008B)	    (0x0:0xB) → 0x0B	    0x08	Small error (~0.3%)

At 12 MHz system clock
Baud	UARTDIV	        BRR2 (hi4:lo4)	        BRR1	Notes
9600	1250 (0x04E2)    (0x0:0x2) → 0x02	0x4E	Exact
19200	625  (0x0271)	 (0x0:0x1) → 0x01	0x27	Exact
38400	313  (0x0139)	 (0x0:0x9) → 0x09	0x12	Error ~0.16%
57600	208  (0x00D0)	 (0x0:0x0) → 0x00	0x0D	Error ~0.8%
115200	104  (0x0068)	 (0x0:0x8) → 0x08	0x06	Error ~2% (usable)

at 1MHz
Baud	UARTDIV	            BRR2 (hi4:lo4)	        BRR1
9600	104 (0x0068)	    (0x0:0x8) → 0x08	    0x06
 */
void uart_init(uint32_t f_cpu, uint32_t baud);
void uart_putc(char c);
char uart_getc(void);
void uart_puts(const char* s);

void delay_ms(uint16_t ms); // crude ~ms delay at 16 MHz

#define Serial_begin(baud) uart_init(F_CPU, baud);
