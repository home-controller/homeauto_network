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
void system_clock_init_16MHz(void);
void system_clock_init_1MHz(void);


void clock_init_external_12MHz(void);

