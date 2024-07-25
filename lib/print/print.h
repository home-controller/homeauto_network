#ifndef _hn_h
#define _hn_h

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

#define printS(A) io_print(A);
#define printN(A) io_print_(A);
#endif