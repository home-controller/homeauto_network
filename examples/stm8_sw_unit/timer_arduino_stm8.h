#ifndef ARDUINO_STM8_H
#define ARDUINO_STM8_H

//#include "stm8s.h"

// typedef enum { LOW = 0, HIGH = 1 } PinLevel;
// typedef enum { INPUT, OUTPUT } PinMode;

// // Basic wrappers
// void pinMode(GPIO_TypeDef* port, uint8_t pin, PinMode mode);
// void digitalWrite(GPIO_TypeDef* port, uint8_t pin, PinLevel val);
// PinLevel digitalRead(GPIO_TypeDef* port, uint8_t pin);
void delayMicroseconds(uint16_t us);
void delay(uint16_t ms);


void print_clock_source(void);
void clock_init(void);

void timer2_init(void);
uint32_t millis(void);
uint32_t micros(void);

#endif
