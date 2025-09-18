#ifndef STM8_INTRINSICS_H
#define STM8_INTRINSICS_H

// ---------------------------------------------------------
// Compiler-specific intrinsics and interrupt declaration
// ---------------------------------------------------------

#if defined(__SDCC__)   // Small Device C Compiler
  #define enableInterrupts()   __asm__("rim\n")
  #define disableInterrupts()  __asm__("sim\n")
  #define wfi()                __asm__("wfi\n")
  #define nop()                __asm__("nop\n")
  #define INTERRUPT_HANDLER(name, irq)  void name(void) __interrupt(irq)

#elif defined(__CSMC__) // Cosmic compiler
  #define enableInterrupts()   _rim_()
  #define disableInterrupts()  _sim_()
  #define wfi()                _wfi_()
  #define nop()                _nop_()
  #define INTERRUPT_HANDLER(name, irq)  @far @interrupt void name(void)

#elif defined(__IAR_SYSTEMS_ICC__) // IAR Embedded Workbench
  #include <intrinsics.h>
  #define enableInterrupts()   __enable_interrupt()
  #define disableInterrupts()  __disable_interrupt()
  #define wfi()                __wait_for_interrupt()
  #define nop()                __no_operation()
  #define INTERRUPT_HANDLER(name, irq)  __interrupt void name(void)

#else
  #warning "Unknown STM8 compiler: define intrinsics manually"
  #define enableInterrupts()
  #define disableInterrupts()
  #define wfi()
  #define nop()
  #define INTERRUPT_HANDLER(name, irq)  void name(void)
#endif

#endif // STM8_INTRINSICS_H
