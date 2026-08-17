#include "uart.h"
#include "serial_print.h"

// --- Basic string print ---
// void uart_print_s(const char *s) {
//     while (*s) uart_putc(*s++);
// }

void uart_println_s(const char *s) {
    uart_print_s(s);
    uart_println();
}

void uart_println(void) {
    uart_putc('\r');
    uart_putc('\n');
}

// --- Integer (signed 16-bit) ---
void uart_print_i16(int16_t v) {
    char buf[7]; // sign + 5 digits + null
    char *p = buf + sizeof(buf) - 1;
    *p = '\0';

    int neg = (v < 0);
    uint16_t val = neg ? -v : v;

    do {
        *--p = '0' + (val % 10);
        val /= 10;
    } while (val);

    if (neg) *--p = '-';

    uart_puts(p);
}

void uart_println_i16(int16_t v) {
    uart_print_i16(v);
    uart_println();
}

// --- Integer (unsigned 16-bit) ---
void uart_print_u16(uint16_t v) {
    char buf[6];
    int i = 0;

    if (v == 0) { uart_putc('0'); return; }

    while (v > 0 && i < (int)(sizeof(buf) - 1)) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i--) uart_putc(buf[i]);
}

void uart_println_u16(uint16_t v) {
    uart_print_u16(v);
    uart_println();
}

// --- Integer (unsigned 32-bit) ---
/*
 * uart_print_u32
 * Prints an unsigned 32-bit integer as a decimal string.
 * Buffer is sized for 10 decimal digits plus null terminator.
 * Note: uart_puts expects a null-terminated string.
 */
void uart_print_u32(uint32_t v) {
    char buf[11]; // 10 digits + null terminator
    char *p = buf + sizeof(buf) - 1;
    *p = '\0';

    do {
        *--p = '0' + (v % 10);
        v /= 10;
    } while (v);

    uart_puts(p);
}

void uart_println_u32(uint32_t v) {
    uart_print_u32(v);
    uart_println();
}


// --- Integer (signed 32-bit) ---
void uart_print_i32(int32_t v) {
    char buf[12]; // -2147483648\0
    int i = 0;

    if (v == 0) { uart_putc('0'); return; }

    if (v < 0) { uart_putc('-'); v = -v; }

    while (v > 0 && i < (int)(sizeof(buf) - 1)) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i--) uart_putc(buf[i]);
}

void uart_println_i32(int32_t v) {
    uart_print_i32(v);
    uart_println();
}

// --- Hex helpers ---
void uart_print_hex8(uint8_t v) {
    const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[(v >> 4) & 0xF]);
    uart_putc(hex[v & 0xF]);
}

void uart_println_hex8(uint8_t v) {
    uart_print_hex8(v);
    uart_println();
}

void uart_print_hex16(uint16_t v) {
    uart_print_hex8((v >> 8) & 0xFF);
    uart_print_hex8(v & 0xFF);
}

void uart_println_hex16(uint16_t v) {
    uart_print_hex16(v);
    uart_println();
}

// --- Binary (8-bit) ---
void uart_print_bin8(uint8_t v) {
    for (int i = 7; i >= 0; --i) {
        uart_putc((v & (1 << i)) ? '1' : '0');
    }
}

void uart_println_bin8(uint8_t v) {
    uart_print_bin8(v);
    uart_println();
}

// --- Binary (16-bit) ---
void uart_print_bin16(uint16_t v) {
    for (int i = 15; i >= 0; --i) {
        uart_putc((v & (1 << i)) ? '1' : '0');
    }
}

void uart_println_bin16(uint16_t v) {
    uart_print_bin16(v);
    uart_println();
}
