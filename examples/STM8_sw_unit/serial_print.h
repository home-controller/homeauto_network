#pragma once
#include <stdint.h>

// String
#define uart_print_s(s) uart_puts(s)
//void uart_print_s(const char *s);
void uart_println_s(const char *s);

// Numbers
void uart_print_i16(int16_t v);
void uart_println_i16(int16_t v);

void uart_print_u16(uint16_t v);
void uart_println_u16(uint16_t v);

void uart_print_i32(int32_t v);
void uart_println_i32(int32_t v);

void uart_print_u32(uint32_t v);
void uart_println_u32(uint32_t v);


void uart_print_hex8(uint8_t v);
void uart_println_hex8(uint8_t v);

void uart_print_hex16(uint16_t v);
void uart_println_hex16(uint16_t v);

void uart_print_bin8(uint8_t v);
void uart_println_bin8(uint8_t v);

void uart_print_bin16(uint16_t v);
void uart_println_bin16(uint16_t v);

// Misc
void uart_println(void); // just "\r\n"

#define Serial_print_s(s) uart_print_s(s)
#define Serial_println_s(s) uart_println_s(s)
#define Serial_print_i16(n) uart_print_i16(n)
#define Serial_println_i16(n) uart_println_i16(n)
#define Serial_print_i(n) uart_print_i16(n)
#define Serial_println_i(n) uart_println_i16(n)
