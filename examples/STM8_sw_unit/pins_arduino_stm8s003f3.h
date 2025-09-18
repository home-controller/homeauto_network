#pragma once
//#include "iostm8s003f3.h" // for GPIO_TypeDef etc.


// Encode port + pin into a single 16-bit value
#define PINDEF(port, pin) (((port) << 8) | (pin))

enum {
  PORT_A = 0,
  PORT_B = 1,
  PORT_C = 2,
  PORT_D = 3,
};

// Lookup table for decoding
static GPIO_TypeDef* const port_table[] = { GPIOA, GPIOB, GPIOC, GPIOD };

// --- Digital pin mapping (D0..D15 style) ---
// Following your table

#define D2 PINDEF(PORT_A, 3)  // PA3  / SS / 1wire option
#define D3 PINDEF(PORT_B, 5)  // PB5  / SDA
#define D4 PINDEF(PORT_B, 4)  // PB4  / SCL
#define D5 PINDEF(PORT_C, 3)  // PC3  / HA net in
#define D6 PINDEF(PORT_C, 4)  // PC4  / HA net out / A0
#define D7 PINDEF(PORT_C, 5)  // PC5  / SCK
#define D8 PINDEF(PORT_C, 6)  // PC6  / MOSI
#define D9 PINDEF(PORT_C, 7)  // PC7  / MISO
#define D10 PINDEF(PORT_D, 1) // PD1  / SWIM
#define D11 PINDEF(PORT_D, 2) // PD2  / Light SW1 / A1
#define D12 PINDEF(PORT_D, 3) // PD3  / Light SW2 / A2
#define D13 PINDEF(PORT_D, 4) // PD4  / Light SW3
#define D14 PINDEF(PORT_D, 5) // PD5  / TX / A3
#define D15 PINDEF(PORT_D, 6) // PD6  / RX / A4

// Aliases for Arduino-style pins
#define SS D2
#define SDA D3
#define SCL D4
#define SCK D7
#define MOSI D8
#define MISO D9
#define RX D15
#define TX D14

// --- Analog pins (A0..A4) ---
#define A0 D6  // PC4 / AIN2
#define A1 D11 // PD2 / AIN3
#define A2 D12 // PD3 / AIN4
#define A3 D14 // PD5 / AIN5
#define A4 D15 // PD6 / AIN6

// --- Wrapper functions (Arduino-like) ---
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0

inline void pinMode(uint16_t pinDef, uint8_t mode) {
  GPIO_TypeDef* port = port_table[(pinDef >> 8) & 0xFF];
  uint8_t pin = pinDef & 0xFF;

  if (mode == OUTPUT) {
    port->DDR |= (1 << pin);  // output
    port->CR1 |= (1 << pin);  // push-pull
  } else {                    // INPUT
    port->DDR &= ~(1 << pin); // input
    port->CR1 |= (1 << pin);  // pull-up
  }
}

inline void digitalWrite(uint16_t pinDef, uint8_t val) {
  GPIO_TypeDef* port = port_table[(pinDef >> 8) & 0xFF];
  uint8_t pin = pinDef & 0xFF;

  if (val) port->ODR |= (1 << pin);
  else port->ODR &= ~(1 << pin);
}

inline int digitalRead(uint16_t pinDef) {
  GPIO_TypeDef* port = port_table[(pinDef >> 8) & 0xFF];
  uint8_t pin = pinDef & 0xFF;

  return (port->IDR & (1 << pin)) ? HIGH : LOW;
}
