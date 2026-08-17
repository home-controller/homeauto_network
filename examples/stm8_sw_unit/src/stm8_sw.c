// file: src/stm8_sw.c 
/*
 Code for a stm8 pcb in the light switch using the "simple home network" to:
 monitor up to a 3 gang 2 way switch
 maybe 1-wire temp sensor
 May add touch


 Circuit:
 * Ethernet shield attached with SPI to pins 10, 11, 12, 13 + 9 for reset
 * Output for relays on pins 3, 4 See relays.h tab
 * switch pins Connected to switch directly 14(A0), 15(A1), 16(A2), A3(17)
 A6(20),A7(21) Set to pullup, A6 and A7 can't be pullup on some chips
 *
 * ================This one is us=======================
 * + switch controller network             2,3,4       +
 * =====================================================
 *
 * //  1-wire                                5,6,7
 * //  SPISerial Peripheral Interface        (8,9 select 2 SPI slaves,can be any
 pins)10,11,12,13
 * //  GPIO output, relay, Led, etc.     8, D3(3), 9, A3(17) Over lap with
 above.
 * //  I2C                            A4(18), A5(19)
 * //  leave D3,9 to last to test pwm. Going to need PWM multiplex or second
 arduino or mega.
 * //  A6 & A7 are analogRead(); only, Can't use pinMode(A6,INPUT_PULLUP). Need
 to add a pull up resistor in hardware.


stm8s003f3 – 20-pin TSSOP20
-----------------------------------------------------------
STM8 pin   #   Arduino  PCB                         Notes
-----------------------------------------------------------
PD4        1   D13      Light Switch 3 (SW3)   TIM2_CH1, ADC_ETR
PD5        2   D14/A3   UART1_TX, AIN5
PD6        3   D15/A4   UART1_RX, AIN6, TIM1_CH1
NRST       4            Reset
OSCIN      5            Crystal input
OSCOUT     6            Crystal output
VSS        7            Ground
VCAP       8            1.8 V regulator capacitor
VDD        9            +5 V supply
PA3        10  D2/SS    SPI_NSS, TIM2_CH3, 1wire temp through solder jumper
PB5        11  D3/SDA   I2C                 I²C SDA, TIM1_BKIN
PB4        12  D4/SCL   I2C                 I²C SCL
PC3        13  D5       HA network in       TIM1_CH3, TLI, TIM1_CH1N
PC4        14  D6/A0    HA network out      CLK_CCO, TIM1_CH4, AIN2, TIM1_CH2N
PC5        15  D7/SCK   SPI                 SPI_SCK, TIM2_CH1
PC6        16  D8/MOSI  SPI                 SPI_MOSI, TIM1_CH1
PC7        17  D9/MISO  SPI                 SPI_MISO, TIM1_CH2
PD1        18  D10      SWIM debug interface
PD2        19  D11/A1   Light Switch 1 (SW1)        AIN3, TIM2_CH3
PD3        20  D12/A2   Light Switch 2 (SW2)            AIN4, TIM2_CH2, BEEP
-----------------------------------------------------------


Analog pins (Arduino A0–A4):
    A0 → PC4 / AIN2 (pin 14)
    A1 → PD2 / AIN3 (pin 19)
    A2 → PD3 / AIN4 (pin 20)
    A3 → PD5 / AIN5 (pin 2)
    A4 → PD6 / AIN6 (pin 3)

   I²C pins: PB4=SCL, PB5=SDA.
   SPI pins: PC5=SCK, PC6=MOSI, PC7=MISO, PA3=SS.
   UART1: TX=PD5, RX=PD6.

1-wire
   PA3/D2 temp through solder jumper
   Other wire header/plug connector gos to JP



 */

// #undef F_CPU
// #define F_CPU 12000000UL

// #include <Arduino.h>        // gives you millis(), delay(), etc.
// #include <EEPROM.h>         // sduino EEPROM functions
// #include <HardwareSerial.h> // gives you Serial_xxx macros
// #include "common_defs.h"
// #include "eeprom.h"
#include "system.h"
// #include "pins_arduino_stm8s003f3.h" // pin definitions and pinMode(), digitalWrite(), digitalRead()
#include "print.h"
// #include "serial_print.h"
// #include "timer_arduino_stm8.h" // delay(), delayMicroseconds()
#include "stm8_intrinsics.h"
#include "stm8_regs.h"
#include "uart.h"

#define t A1

#define LOG_INTERVAL_MS 3600000UL // 1 hour in milliseconds
#define MINUTE_INTERVAL_MS 1000UL * 60UL

// // Map enum → readable name
// static const char* clkname(CLK_Source_TypeDef s) {
//   switch (s) {
//     case CLK_SOURCE_HSI:
//       return "HSI";
//     case CLK_SOURCE_LSI:
//       return "LSI";
//     case CLK_SOURCE_HSE:
//       return "HSE";
//     default:
//       return "?";
//   }
// }

// // Print current SYSCLK source
// void print_clock_source(void) {
//   CLK_Source_TypeDef src = CLK_GetSYSCLKSource();
//   Serial_print_s(clkname(src));
//   Serial_println_s(" (SYSCLK source)");
// }

// // Switch system clock to external 12 MHz crystal
// void clock_init(void) {
//   // Enable external high-speed oscillator (HSE)
//   CLK_HSECmd(ENABLE);
//   while (CLK_GetFlagStatus(CLK_FLAG_HSERDY) == RESET) {
//     // wait until HSE is stable
//   }

//   // No prescaler → run core at 12 MHz
//   // CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
//   // CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);

//   // Switch SYSCLK to HSE
//   CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
// }
void print_clock_source(void) {
  uint8_t src = CLK_CMSR;
  switch (src) {
    case 0xE1:
      uart_puts("HSI (16 MHz RC)\r\n");
      break;
    case 0xD2:
      uart_puts("LSI (128 kHz RC)\r\n");
      break;
    case 0xB4:
      uart_puts("HSE (external crystal)\r\n");
      break;
    default:
      uart_puts("Unknown\r\n");
      break;
  }
}

void setup(void) {
  // uint8_t id;

  // 1. Start at 16 MHz
  system_clock_init_16MHz();
  // Serial_begin(9600);
  // uart_init(12000000UL, 9600); // init UART1 at 9600 baud, 12 MHz HSE
  uart_init(16000000UL, 9600); // init UART1 at 9600 baud, 16 MHz HSI
  delay_ms(1000);
  uart_puts("Hello 16MHz internal clock!\r\n");
  print_clock_source();
  delay_ms(100);
  while (!(UART1_SR & 0x40));

  // 2. Switch to 1 MHz (CPU = 1MHz, Master = 16MHz)
  system_clock_init_1MHz();
  uart_init(16000000UL, 9600); // already initialized at 16 MHz, so no need to re-init
  uart_puts("Hello 1MHz internal clock!\r\n");
  print_clock_source();
  // FIX: CPU is 16x slower, so divide the delay by 16 to get ~1 real second
  delay_ms(1000 / 16);

  uart_puts(".\r\n"); // ruffly check timings as seral output has timestamps per line.
  delay_ms(10000 / 16);
  uart_putc('.');
  while (!(UART1_SR & 0x40));

  // 3. Switch to 12 MHz external crystal (CPU = 12MHz, Master = 12MHz)
  clock_init_external_12MHz();
  uart_init(12000000UL, 9600); // init UART1 at 9600 baud, 12 MHz HSE
  delay_ms(1000);
  uart_puts("Hello 12MHz external clock!\r\n");
  print_clock_source();

  system_clock_init_16MHz();
  uart_puts("Hello bare-metal sw!\r\n");
  uart_init(16000000UL, 9600);
  uart_puts("Back to 16MHz internal clock!\r\n");
  while (1) {
    uart_putc('t');
    print_clock_source();
    delay_ms(1000);
  }

  // printS("F_CPU (compile) = ");
  // printI(F_CPU);
  // print_clock_source();
  // timer2_init();
  // // printS(" (compile)");
  // // printS("CLK_GetClockFreq() = ");
  // // printI32(CLK_GetClockFreq());

  // // printS("CLK->CKDIVR = 0x");
  // // printI((int)CLK->CKDIVR);
  // // print millis baseline
  // printS("millis() at start = ");
  // printI32(millis());

  // print_clock_source();
  // printlnS("STM8 Serial started");

  // printS("F_CPU = ");
  // printI((int)(F_CPU / 1000000UL)); // in MHz Serial_println_ul(F_CPU);
  // printlnS("MHz");

  // // Read ID from EEPROM
  // id = EEPROM_read(EEPROM_ADDR_ID);

  // if (id == EEPROM_UNINITIALIZED) {
  //   // Not yet set → assign new ID
  //   id = 42;
  //   EEPROM_write(EEPROM_ADDR_ID, id);
  //   printS("Board ID not set. Assigning: ");
  //   printI((int)id);
  // } else {
  //   printS("Board ID from EEPROM: ");
  //   printI((int)id);
  // }
  // print_clock_source();
  // printS("readBit() = ");
  // printI((int)readBit());
}

void loop(void) {
  // static int l = 0;
  // static uint32_t lastMillis = 0;
  // static uint32_t lastMillisC = 0;
  // static uint16_t c = 0;
  // uint32_t now = millis();

  // if (now - lastMillis >= LOG_INTERVAL_MS) {
  //   lastMillis += LOG_INTERVAL_MS;
  //   printS("Elapsed hours: ");
  //   printI((int)(lastMillis / 3600000UL));
  // }
  // if (now - lastMillisC >= MINUTE_INTERVAL_MS * 10) {
  //   lastMillisC += MINUTE_INTERVAL_MS;
  //   printS("Looping: ");
  //   printI(l);
  //   l++;
  //   // c = 0;
  // }
  // digitalWrite(1, 1);
  delay_ms(100);
}
int main(void) {
  setup();
  while (1) {
    loop();
  }
}