// #include "stm8s.h"
// #include "mcu_defs.h"
#include "uart.h"
#include "stm8_regs.h"
#include "stm8_intrinsics.h"

// Define named constant for UART1_CR1 configuration: 8 data bits, no parity
#ifndef UART1_CR1_M_8DATA_BITS_NO_PARITY
#define UART1_CR1_M_8DATA_BITS_NO_PARITY 0x00
#endif

// -------------------------
// UART1 initialization
// -------------------------
// void uart_init(uint32_t f_cpu, uint32_t baud) {
//  // uint16_t brr;

//   // UARTDIV = f_cpu / (16 * baud)
//   //brr = (uint16_t)((f_cpu + (8UL * baud)) / (16UL * baud));

//   //UART1_BRR2 =  (uint8_t)(((brr >> 8) & 0xF0) | (brr & 0x0F));
//   //UART1_BRR1 =  (uint8_t)((brr >> 4) & 0xFF);
//   // 16MHz HSI
//   UART1_BRR1 = 0x06;//stm8s003f3 9600 baud
//   UART1_BRR2 = 0x88; 

//   UART1_CR1 = UART1_CR1_M_8DATA_BITS_NO_PARITY;      // 8 data bits, no parity
//   // Clear STOP1 and STOP2 bits to configure 1 stop bit (STOP[1:0] = 00)
//   UART1_CR3 &= ~(UART1_CR3_STOP1 | UART1_CR3_STOP2); // 1 stop
//   UART1_CR2 = UART1_CR2_TEN | UART1_CR2_REN;         // enable TX+RX
// }

/**
 * @brief Initialize UART1
 * @note if you do not change the clock settings, the default HSI 16 MHz
 * But CLK_CKDIVR default = 0x18 → CPU clock = HSI ÷ 8 = 2 MHz so you need to set CLK_CKDIVR = 0x00 for 16 MHz
 *
 * @param f_cpu CPU frequency in Hz
 * @param baud Baud rate in bps
 */
void uart_init(uint32_t f_cpu, uint32_t baud)
{
    uint16_t div = (uint16_t)((f_cpu + (baud >> 1)) / baud);

    UART1_BRR2 = (uint8_t)((div >> 8) & 0xF0) | (div & 0x0F);
    UART1_BRR1 = (uint8_t)(div >> 4);

    UART1_CR1 = 0x00; // 8 data, no parity
    UART1_CR3 = 0x00; // 1 stop bit
    UART1_CR2 = UART1_CR2_TEN | UART1_CR2_REN;
}

// -------------------------
// Send one byte (blocking)
// -------------------------
void uart_putc(char c) {
  while (!(UART1_SR & UART1_SR_TXE)) {
    // wait for TX buffer empty
  }
  UART1_DR = (uint8_t)c;
}

// -------------------------
// Receive one byte (blocking)
// -------------------------
char uart_getc(void) {
  while (!(UART1_SR & UART1_SR_RXNE)) {
    // wait for data available
  }
  return (char)UART1_DR;
}

// -------------------------
// Send a string (blocking)
// -------------------------
void uart_puts(const char* s) {
  while (*s) {
    uart_putc(*s++);
  }
}

/// @brief Crude delay loop, ~ms at 16 MHz. if the clock was 1/2 of that, it would take twice as long
/// @param ms 
void delay_ms(uint16_t ms) {
  for (uint16_t i = 0; i < ms; i++) {
    for (uint16_t j = 0; j < 4000; j++) {
      nop(); // crude ~1 ms delay at 16 MHz
    }
  }
}