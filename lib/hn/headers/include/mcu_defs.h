#pragma once

#ifdef STM8
//#include <stm8s.h>
#include <stdint.h> 
#include <stm8_regs.h>

typedef uint8_t byte;
typedef uint16_t word;
typedef uint8_t boolean;
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

#define DigitalWriteTime 4   // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5    // forums says 4.78µs but I think than includes the for loop
#define ReadBitsLoopMicros 3 // this is for each time though the loop that checks 8 time per bit so a value of 3 would be 24µs per bit

#define EEPROM_UNINITIALIZED 0x00
#define EEPROM_ADDR_ID 0x00 // address in EEPROM to store board ID

#elif defined(ARDUINO)
#include <Arduino.h>

#define DigitalWriteTime 4   // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5    // forums says 4.78µs but I think than includes the for loop
#define ReadBitsLoopMicros 3 // this is for each time though the loop that checks 8 time per bit so a value of 3 would be 24µs per bit

#define EEPROM_UNINITIALIZED 0xFF
#endif