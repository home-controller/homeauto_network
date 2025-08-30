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

 stm8s003f3 20 pin TSSOP20 pinout
        pin
PD4     1       Light Switch Gang 3 (SW3)
PD5     2       UART1 TX
PD6     3       UART1 RX




 */

#include <Arduino.h>        // gives you millis(), delay(), etc.
#include <EEPROM.h>         // sduino EEPROM functions
#include <HardwareSerial.h> // gives you Serial_xxx macros

#define t A1

#define EEPROM_ADDR_ID 0x00 // address in EEPROM to store board ID

void setup(void) {
    uint8_t id;

    Serial_begin(9600);
    Serial_println_s("STM8 Serial started");

    // Read ID from EEPROM
    id = EEPROM_read(EEPROM_ADDR_ID);

    if (id == 0xFF) {
        // Not yet set → assign new ID
        id = 42;
        EEPROM_write(EEPROM_ADDR_ID, id);
        Serial_print_s("Board ID not set. Assigning: ");
        Serial_println_i((int)id);
    } else {
        // Serial_print_s("Board ID from EEPROM: ");
        Serial_println_i((int)id);
    }
}

void loop(void) {
    Serial_println_s("Looping...");
    delay(1000);
}
