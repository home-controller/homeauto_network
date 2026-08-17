#include "system.h"
#include "stm8_intrinsics.h"
#include "stm8_regs.h"
#include <iso646.h>

uint8_t system_get_current_source(void) {
  return CLK_CMSR; // 0xE1 = HSI, 0xB4 = HSE, 0xD2 = LSI
}
void system_switch_to_hsi(void) {
  // 1. Defensive Check: If we are already running on HSI, skip everything
  if (system_get_current_source() == CLK_SRC_HSI) { return; }

  // 2. Fire up the Internal RC if it was sleeping
  CLK_ICKR |= CLK_ICKR_HSIEN; // HSIEN = 1

  // 3. Wait block: Stall safely until the HSI hardware reports ready
  while (!(CLK_ICKR & CLK_ICKR_HSIRDY))
    nop(); // Wait for HSIRDY = 1

  // 4. Execute safe clock switch
  CLK_SWCR |= CLK_SWCR_SWEN; // SWEN = 1 (Switch Enable),  Enable switching execution
  CLK_SWR = CLK_SRC_HSI;     // Target = HSI
  while (CLK_CMSR != CLK_SRC_HSI)
    nop();                    // Wait until switch complete
  CLK_SWCR &= ~CLK_SWCR_SWEN; // Clear SWEN Switch Enable flag
}

void system_switch_to_hse(void) {
  // 1. Defensive Check: Skip if we are already safely on HSE
  if (system_get_current_source() == CLK_SRC_HSE) { return; }

  // Disable the Clock Security System (CSS) detection before shifting
  CLK_CSSR &= ~CLK_CSSR_CSSEN;

  // 2. Turn on the External Crystal Oscillator hardware
  CLK_ECKR |= CLK_ECKR_HSEEN;

  // 3. Wait block: Stall safely until the external crystal hardware stabilizes
  while (!(CLK_ECKR & CLK_ECKR_HSERDY)) {
    nop();
  }

  // Physical loop buffer to allow noisy crystal harmonics to settle perfectly
  for (volatile int i = 0; i < 8000; i++) {
    nop();
  }

  // 4. Fire the clock switch hardware sequencer
  CLK_SWCR |= CLK_SWCR_SWEN; // Enable switching execution
  CLK_SWR = CLK_SRC_HSE;     // Select target clock source

  // Wait until the Master Status Register confirms the migration
  while (CLK_CMSR != CLK_SRC_HSE) {
    nop();
  }

  CLK_SWCR &= ~CLK_SWCR_SWEN; // Clear Switch Enable flag
}

void system_set_hsi_speed(HSI_Speed_t speed) {
  // Ensure we are operating on the internal oscillator first
  system_switch_to_hsi();
  CLK_CKDIVR = (uint8_t)speed;
}

void system_set_hse_speed(HSE_Speed_t speed) {
  // Ensure we are operating on the external crystal first
  system_switch_to_hse();

  // For HSE, HSIDIV bits are ignored, but clean it up out of good practice
  CLK_CKDIVR = (uint8_t)speed;
}

/**
 * @brief Initialize system clock to 16 MHz using internal HSI oscillator.
 *
 */
void system_clock_init_16MHz(void) {
  system_set_hsi_speed(HSI_16MHZ); // Set HSI to 16 MHz
}

void system_clock_init_1MHz(void) {
  // Reset clock divider → CPU runs at full HSI 16 MHz
  // CLK_CKDIVR = 4; // CPUDIV = ÷16 → 1 MHz
  system_set_hsi_speed(HSI_1MHZ); // Set HSI to 1 MHz

  // Optional: could also check CLK_CMSR == 0xE1 (HSI selected)
  // or switch to HSE if you want external crystal later
}
/*



HSI = 16 000 000 Hz. Two prescalers affect it:
    HSIDIV (÷1, ÷2, ÷4, ÷8) – applied only when HSI is the source.
    CPUDIV (÷1, ÷2, ÷4, ÷8, ÷16, ÷32, ÷64, ÷128) – always applied to the selected source.
    note: they are both bit fields in the same register (CLK_CKDIVR), so you can set them together.

(Combined divisor = 2^n, n = 0..10)
n (total exponent)	Combined divisor	fCPU = 16,000,000 / 2^n
0	                  1           	    16,000,000 Hz
1	                  2	                8,000,000 Hz
2	                  4	                4,000,000 Hz
3	                  8	                2,000,000 Hz
4	                  16	              1,000,000 Hz
5                   32	              500,000 Hz
6	                  64	              250,000 Hz
7	                  128	              125,000 Hz    CPUDIV can get you to here, without HSIDIV, i.e. external crystal or with HSEDIV at 1
8	                  256	              62,500 Hz
9	                  512	              31,250 Hz
10	                1024	            15,625 Hz

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
  uint16_t timeout = 0xFFFF; // Safety timeout

  CLK_CSSR &= ~CLK_CSSR_CSSEN;
  // Enable external crystal
  CLK_ECKR |= CLK_ECKR_HSEEN;

  // Wait until crystal is stable
  while (!(CLK_ECKR & CLK_ECKR_HSERDY) and (--timeout > 0)) {
    nop();
  }
  if (timeout == 0) {
    // CRYSTAL FAILED OR MISSING!
    // Fall back to 16MHz internal clock safely
    system_clock_init_16MHz();
    return;
  }

  // Small extra delay (~1 ms) to make sure oscillator is fully stable
  for (volatile int i = 0; i < 8000; i++) {
    nop();
  }
  // --- THE FIX: Turn on the switch execution engine ---
  CLK_SWCR |= 0x02; // Set SWEN = 1 (Switch Enable)

  // Switch system clock source to HSE
  CLK_SWR = 0xB4; // 0xB4 = HSE selected
  while (CLK_CMSR != 0xB4)
    ; // wait for switch complete

  // No prescaler
  CLK_CKDIVR = 0x00; // CPU clock = HSE = 12 MHz

  // Clear Switch Enable bit now that we are done
  CLK_SWCR &= ~0x02;
}

// --- Switch to external 12 MHz crystal ---
void switch_to_hse(void) {
  CLK_SWCR |= (1 << 1);  // SWEN = 1 (enable switching)
  CLK_SWR = CLK_SRC_HSE; // request HSE
  while (CLK_CMSR != CLK_SRC_HSE) {
    // wait until switch finished
  }
}
// --- Switch back to internal 16 MHz RC ---
void switch_to_hsi(void) {
  CLK_SWCR |= (1 << 1);  // SWEN = 1 (enable switching)
  CLK_SWR = CLK_SRC_HSI; // request HSI
  while (CLK_CMSR != CLK_SRC_HSI) {
    // wait until switch finished
  }
}

/// @brief Calibrates the HSI clock using the precise 12 MHz HSE crystal
/// @warning Totally untested, AI-generated code
/// @note this requires hardwiring a wire from the HSI output to the CCO pin (PC4).
void calibrate_hsi_from_hse(void) {
    // 1. Switch the system clock to the precise 12 MHz HSE crystal
    system_switch_to_hse(); 
    
    // 2. Output the HSI clock (16 MHz) to the CCO pin (PC4)
    // We set CCOSEL = HSI in the CLK_CCOR register
    CLK_CCOR = 0x01 | 0x10; // Enable CCO and select HSI as source
    
    // 3. Configure TIM2 Channel 1 to capture the period of the CCO signal
    TIM2_CCMR1 = 0x01; // CC1 channel configured as input, mapped on TI1FP1
    TIM2_CCER1 = 0x01; // Capture enabled on rising edge
    TIM2_CR1   = 0x01; // Start TIM2 (running at 12 MHz from the HSE crystal)
    
    uint8_t best_trim = CLK_HSITRIMR;
    uint16_t best_error = 0xFFFF;
    
    // 4. Successive approximation: Test different trim values
    // The trim register typically uses the 3 or 4 least significant bits
    for (uint8_t temp_trim = 0; temp_trim < 16; temp_trim++) {
        CLK_HSITRIMR = temp_trim;
        
        // Wait for the clock to settle
        for (volatile int i = 0; i < 500; i++);
        
        // Capture two consecutive edges to measure the HSI period
        while (!(TIM2_SR1 & 0x02)); // Wait for first capture
        uint16_t t1 = (TIM2_CNTRH << 8) | TIM2_CNTRL;
        
        while (!(TIM2_SR1 & 0x02)); // Wait for second capture
        uint16_t t2 = (TIM2_CNTRH << 8) | TIM2_CNTRL;
        
        uint16_t measured_period = t2 - t1;
        
        // TARGET CALCULATION:
        // Timer runs at 12 MHz (HSE). Target HSI is 16 MHz.
        // Expected timer ticks per HSI cycle = 12,000,000 / 16,000,000 = 0.75 ticks.
        // To get better resolution, you can configure the CCO prescaler 
        // to divide HSI by 64, making the target ticks = 64 * (12/16) = 48 ticks.
        
        uint16_t target_ticks = 48; 
        uint16_t error = (measured_period > target_ticks) ? 
                         (measured_period - target_ticks) : 
                         (target_ticks - measured_period);
                         
        if (error < best_error) {
            best_error = error;
            best_trim = temp_trim;
        }
    }
    
    // 5. Apply the optimal trim value
    CLK_HSITRIMR = best_trim;
    
    // 6. Clean up: Turn off CCO and TIM2 to save power
    CLK_CCOR = 0x00; 
    TIM2_CR1 = 0x00;
}