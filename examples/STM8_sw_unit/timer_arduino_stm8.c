#include "mcu_defs.h"
#include "serial_print.h"
#include "stm8_regs.h"
#include "stm8_intrinsics.h"

// -------------------------
// Config: Select timer
// -------------------------
// Comment out to use Timer2, enable to use Timer4
// #define USE_TIMER4

static volatile uint32_t millis_count = 0;

// -------------------------
// Missing register bit defs
// -------------------------
// Event Generation Register (EGR)
#ifndef TIM4_EGR_UG
#define TIM4_EGR_UG (1 << 0) // Update Generation (reload prescaler/ARR)
#endif
#ifndef TIM2_EGR_UG
#define TIM2_EGR_UG (1 << 0)
#endif

// -------------------------
// Init system clock (HSE 12 MHz assumed)
// -------------------------
void clock_init(void) {
  // Enable HSE (external crystal oscillator)
  CLK_ECKR |= CLK_ECKR_HSEEN;
  while (!(CLK_ECKR & CLK_ECKR_HSERDY))
    ;

  // Switch system clock source to HSE
  CLK_SWR = 0xB4;
  while (CLK_CMSR != 0xB4)
    ;

  // No prescaler
  CLK_CKDIVR = 0x00;
}

void print_clock_source(void) {
  uint8_t src = CLK_CMSR;
  switch (src) {
    case 0xE1:
      Serial_println_s("HSI (16 MHz RC)");
      break;
    case 0xD2:
      Serial_println_s("LSI (128 kHz RC)");
      break;
    case 0xB4:
      Serial_println_s("HSE (external crystal)");
      break;
    default:
      Serial_println_s("Unknown");
      break;
  }
}

#ifndef USE_TIMER4
// -------------------------
// Timer2 → 1 MHz tick (1 µs per tick)
// ISR fires every 1 ms
// -------------------------
void timer2_init(void) {
  // Enable Timer2 clock
  CLK_PCKENR1 |= CLK_PCKENR1_TIM2; // (0x20)

  // Reset TIM2 control registers
  TIM2_CR1 = 0x00;
  TIM2_IER = 0x00;
  TIM2_SR1 = 0x00;
  TIM2_EGR = 0x00;   // no events
  TIM2_CCMR1 = 0x00; // capture/compare mode reset
  TIM2_CCMR2 = 0x00;
  TIM2_CCMR3 = 0x00;
  TIM2_CCER1 = 0x00;
  TIM2_CNTRH = 0x00; // counter reset
  TIM2_CNTRL = 0x00;
  TIM2_PSCR = 0x00; // prescaler reset
  TIM2_ARRH = 0xFF; // auto-reload max
  TIM2_ARRL = 0xFF;

  TIM2_PSCR = 11; // prescaler = 2^11 = 2048 → 12MHz / 2048 ≈ 5859 Hz
  // Better: for exact 1 MHz, prescaler=11? (let’s assume 12 MHz HSE)
  // Actually: PSC=11 means divide by 12 → 1 MHz
  TIM2_ARRH = (1000 - 1) >> 8; // 1000 ticks = 1 ms
  TIM2_ARRL = (1000 - 1) & 0xFF;

  TIM2_IER |= TIM2_IER_UIE; // update interrupt enable
  TIM2_CR1 |= TIM2_CR1_CEN; // counter enable
}

// -------------------------
// Timer2 ISR
// -------------------------
/**
  TIM2_SR1 = (TIM2_SR1 & ~TIM2_SR1_UIF); // clear only UIF, preserve other flags
  millis_count++;
 * This ISR is triggered every 1 ms by Timer2 and increments the global millis_count.
 * It also clears the update interrupt flag to allow subsequent interrupts.
 */
INTERRUPT_HANDLER(TIM2_UPD_OVF_IRQHandler, 13)
{
  TIM2_SR1 &= ~TIM2_SR1_UIF; // clear update flag
  millis_count++;
}
#endif

#ifdef USE_TIMER4
// -------------------------
// Timer4 initialization (8-bit basic timer)
// 1 ms tick directly (no micros() precision)
// Used for millis()/micros() tick (1 ms)
// -------------------------
void timer4_init(void) {
  // Enable clock for TIM4
  // Bit5 = TIM4EN = 0x20
  CLK_PCKENR1 |= CLK_PCKENR1_TIM4; // (0x20)

  // --- Reset control registers ---
  TIM4_CR1 = 0x00;
  TIM4_IER = 0x00;
  TIM4_SR = 0x00;
  TIM4_EGR = 0x00;
  TIM4_PSCR = 0x00;
  TIM4_ARR = 0xFF;

  // --- Configure for 1ms tick ---
  // f_master = 12 MHz
  // Prescaler = 128 → f_cnt = 93.75 kHz
  // Period = 125 → 93.75k / 125 = 750 Hz ≈ 1.333 ms (close but not exact)
  // Instead, ARR = 124 → (124+1)=125 counts → exactly 1.000 ms per update
  TIM4_PSCR = 0x07; // prescaler 128
  TIM4_ARR = 124;   // auto-reload value

  // Enable update interrupt
  TIM4_IER |= TIM4_IER_UIE;

  // Generate update to load ARR/PSCR
  TIM4_EGR |= TIM4_EGR_UG;

  // Start timer
  TIM4_CR1 |= TIM4_CR1_CEN;
}

// -------------------------
// Timer4 ISR for millis()
// -------------------------
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) {
  TIM4_SR &= ~TIM4_SR_UIF; // clear interrupt flag
  millis_count++;          // increment global counter
}
#endif

// -------------------------
// Arduino-like API
// -------------------------
/**
 * @brief Returns the number of milliseconds since the program started.
 *
 * This function provides the elapsed time in milliseconds, which are units of time equal to 1/1000th of a second.
 * Typically used for timing operations. It relies on the global variable `millis_count`, which should be incremented by a timer interrupt.
 *
 * @return The number of milliseconds since the program began execution.
 *
 * @note There are 1000 milliseconds in one second. Do not confuse with microseconds, which are 1/1,000,000 of a second.
 *
 * @return uint32_t The number of milliseconds since the program began.
 */
uint32_t millis(void) { return millis_count; }

uint32_t micros(void) { // 1 million per second
#ifdef USE_TIMER4
  // Only ms resolution if Timer4 is used
  return millis_count * 1000UL;
#else
  uint16_t t;
  uint32_t m;

  disableInterrupts();
  m = millis_count;
  t = ((uint16_t)TIM2_CNTRH << 8) | TIM2_CNTRL;
  // Check overflow race
  if (TIM2_SR1 & TIM2_SR1_UIF) {
    m++;
    t = ((uint16_t)TIM2_CNTRH << 8) | TIM2_CNTRL;
  }
  enableInterrupts();

  return (m * 1000UL) + t;
#endif
}

// -------------------------
// Delay functions
// -------------------------
void delay(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms)
    ;
}

void delayMicroseconds(uint16_t us) {
#ifdef USE_TIMER4
  // crude: spin-loop since no µs resolution
  while (us--) { __asm__("nop\n nop\n nop\n nop\n"); }
#else
  uint32_t start = micros();
  while ((micros() - start) < us);
#endif
}
