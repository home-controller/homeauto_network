#include "system.h"

void system_clock_init_16MHz(void) {
  // Reset clock divider → CPU runs at full HSI 16 MHz
  CLK_CKDIVR = 0x00;

  // Optional: could also check CLK_CMSR == 0xE1 (HSI selected)
  // or switch to HSE if you want external crystal later
}

void system_clock_init_1MHz(void) {
  // Reset clock divider → CPU runs at full HSI 16 MHz
  CLK_CKDIVR = 4; // CPUDIV = ÷16 → 1 MHz

  // Optional: could also check CLK_CMSR == 0xE1 (HSI selected)
  // or switch to HSE if you want external crystal later
}
/*



HSI = 16 000 000 Hz. Two prescalers affect it:
    HSIDIV (÷1, ÷2, ÷4, ÷8) – applied only when HSI is the source.
    CPUDIV (÷1, ÷2, ÷4, ÷8, ÷16, ÷32, ÷64, ÷128) – always applied to the selected source.

(Combined divisor = 2^n, n = 0..10)
n (total exponent)	Combined divisor	fCPU = 16,000,000 / 2^n
0	1	    16,000,000 Hz
1	2	    8,000,000 Hz
2	4	    4,000,000 Hz
3	8	    2,000,000 Hz
4	16	    1,000,000 Hz
5	32	    500,000 Hz
6	64	    250,000 Hz
7	128	    125,000 Hz
8	256	    62,500 Hz
9	512	    31,250 Hz
10	1024	15,625 Hz

HSIDIV applies only to HSI. If you switch to HSE, HSIDIV is ignored.

when using your 12 MHz external crystal (HSE = 12 MHz)

(CPUDIV only; divisor = 2^n, n = 0..7)
n (CPUDIV exponent)	Divisor	fCPU = 12,000,000 / 2^n
0	1	12,000,000 Hz
1	2	6,000,000 Hz
2	4	3,000,000 Hz
3	8	1,500,000 Hz
4	16	750,000 Hz
5	32	375,000 Hz
6	64	187,500 Hz
7	128	93,750 Hz
*/

// -------------------------
// Switch system clock to external HSE 12 MHz crystal
void clock_init_external_12MHz(void) {
  // *** TEMPORARILY DISABLE CSS FOR DEBUGGING ***
  CLK_CSSR &= ~CLK_CSSR_CSSEN;
  // Enable external crystal
  CLK_ECKR |= CLK_ECKR_HSEEN;

  // Wait until crystal is stable
  while (!(CLK_ECKR & CLK_ECKR_HSERDY))
    ;

  // Small extra delay (~1 ms) to make sure oscillator is fully stable
  for (volatile int i = 0; i < 8000; i++)
    ;

  // Switch system clock source to HSE
  CLK_SWR = 0xB4; // 0xB4 = HSE selected
  while (CLK_CMSR != 0xB4)
    ; // wait for switch complete

  // No prescaler
  CLK_CKDIVR = 0x00; // CPU clock = HSE = 12 MHz
}
// --- Switch to external 12 MHz crystal ---
void switch_to_hse(void) {
  CLK_SWCR |= (1 << 1);     // SWEN = 1 (enable switching)
  CLK_SWR = CLK_SRC_HSE;    // request HSE
  while (CLK_CMSR != CLK_SRC_HSE) {
    // wait until switch finished
  }
}
// --- Switch back to internal 16 MHz RC ---
void switch_to_hsi(void) {
  CLK_SWCR |= (1 << 1);     // SWEN = 1 (enable switching)
  CLK_SWR = CLK_SRC_HSI;    // request HSI
  while (CLK_CMSR != CLK_SRC_HSI) {
    // wait until switch finished
  }
}