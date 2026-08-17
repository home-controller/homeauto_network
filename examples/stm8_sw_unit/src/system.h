#pragma once
#include <stdint.h>
#include "stm8_regs.h"

/**
 * @brief Initialize STM8 system clock.
 *
 * - Sets CPU clock prescaler to 1 (CLK_CKDIVR = 0x00).
 * - Leaves default HSI (16 MHz) as system clock source.
 *
 * After calling this, F_CPU should be defined as 16000000UL.
 */

// Fallback: If no external crystal value was passed during compilation, assume 12MHz
#ifndef HSE_VALUE
#define HSE_VALUE 12000000UL
#endif

// Readable enums for HSI clock speeds (F_CPU)
typedef enum {
    HSI_16MHZ = 0x00, // No dividers
    HSI_8MHZ  = 0b00001000, // HSIDIV = 2, CPUDIV = 1
    HSI_4MHZ  = 0b00010000, // HSIDIV = 4, CPUDIV = 1
    HSI_2MHZ  = 0b00011000, // HSIDIV = 8, CPUDIV = 1 (STM8 Default Reset state)
    HSI_1MHZ  = 0b00011001  // HSIDIV = divide by 8, CPUDIV = divide by 2 for total divide by 16 → 1 MHz
} HSI_Speed_t;

// Readable enums for HSE clock speeds (F_CPU depends entirely on CPUDIV)
typedef enum {
    HSE_DIV1   = 0x00, // F_CPU = HSE_VALUE / 1
    HSE_DIV2   = 0x01, // F_CPU = HSE_VALUE / 2
    HSE_DIV4   = 0x02, // F_CPU = HSE_VALUE / 4
    HSE_DIV8   = 0x03, // F_CPU = HSE_VALUE / 8
    HSE_DIV16  = 0x04  // F_CPU = HSE_VALUE / 16
} HSE_Speed_t;

// Source Switching (Defensive checks built-in)
void system_switch_to_hsi(void);
void system_switch_to_hse(void);

// Speed Control
void system_set_hsi_speed(HSI_Speed_t speed);
void system_set_hse_speed(HSE_Speed_t speed);

// Helper to check what we are running right now
uint8_t system_get_current_source(void);

void system_clock_init_16MHz(void);
void system_clock_init_1MHz(void);


void clock_init_external_12MHz(void);

