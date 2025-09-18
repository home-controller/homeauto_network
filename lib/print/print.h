#ifndef _hn_h
#define _hn_h

#ifdef STM8
#define io_println(S) uart_println_s(S)
#define io_print(S) uart_puts(S)//Serial.print(F(S))
#define io_print_n16(N) uart_print_i16(N)  // output a number
#define io_print_n32(N) uart_print_i32(N)  // output a number
#define io_println_n16(N) uart_println_i16(N)  // output a number
#define io_println_n32(N) uart_println_i32(N)  // output a number


#else
#ifndef noMcu_buildflag
#define io_println(S) Serial.println(F(S))
#define io_print(S) Serial.print(F(S))
#define io_print_n(N) Serial.print(N)  // output a number
#else
#include <stdio.h>
#define io_println(S) \
  printf(S);          \
  printf("\n");
#define io_print_n(N) printf(N)  // output a number
#define io_print(S) printf(S);
#endif
#endif

#define printS(A) io_print(A);
#define printI(A) io_print_n16(A);
#define printI32(A) io_print_n32(A);
#define printlnS(A) io_println(A);
#define printlnI(A) io_print_n16(A);
#define printlnI32(A) io_print_n32(A);
#endif