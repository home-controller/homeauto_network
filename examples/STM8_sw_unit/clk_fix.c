#include "stm8s.h"

/* Override HSE_VALUE */
#undef HSE_VALUE
#define HSE_VALUE 12000000UL  // 12 MHz crystal
const uint8_t HSIDivFactor[4] = {1, 2, 4, 8};

/* Minimal replacement for CLK_GetClockFreq */
uint32_t CLK_GetClockFreq(void)
{
    uint32_t clockfrequency = 0;
    CLK_Source_TypeDef clocksource;
    uint8_t tmp = 0, presc = 0;

    clocksource = (CLK_Source_TypeDef)CLK->CMSR;

    if (clocksource == CLK_SOURCE_HSI)
    {
        tmp = (uint8_t)(CLK->CKDIVR & CLK_CKDIVR_HSIDIV);
        tmp = (uint8_t)(tmp >> 3);
        presc = HSIDivFactor[tmp];
        clockfrequency = HSI_VALUE / presc;
    }
    else if (clocksource == CLK_SOURCE_LSI)
    {
        clockfrequency = LSI_VALUE;
    }
    else  // HSE
    {
        clockfrequency = HSE_VALUE;
    }

    return clockfrequency;
}
