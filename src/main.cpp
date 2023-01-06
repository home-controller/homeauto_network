/*
 A simple home network Libary

 A slow network for communication on a Arduino GPIO pins with the protocol modelled
 after the CAN network but cut down for less bits per command and much slower bit
 rate. It needs to be slower as no line driver chip and without the
 protocol dedicated hardware and the MCU running other stuff there will be delays
 in detecting line high low transitions.

 Circuit:
 * Ethernet shield attached with SPI to pins 10, 11, 12, 13 + 9 for reset
 * Output for relays on pins 3, 4 See relays.h tab
 */

#include <defs.h>
#include <gpioSwitchInput.h>
#include <hn.h>

#include "/home/jmnc2/.platformio/packages/toolchain-atmelavr/avr/include/avr/wdt.h"

#define serial_speed 38400

#define pinIO_no_of_switches 6               // setup the number of gpio's used
#define pinIO_inPins A7, A6, A0, A1, A2, A3  // in sa main.h

// byte pinIO_Max_switches = pinIO_no_of_switches;
byte pinIO_switchState[pinIO_no_of_switches];
byte pinIO_pinsA_in[pinIO_no_of_switches] = {pinIO_inPins};

void gotInputPin(byte ioType, byte i, byte offset, byte count, byte state) {  // Callback when a switch changes
    Serial.print(F("ioType:"));
    Serial.print(ioType);
    Serial.print(F(", i:"));
    Serial.print(i);
    Serial.print(F(", count:"));
    Serial.print(count);
    Serial.print(F(", state:"));
    Serial.println(state);
}

gpioSwitchInputC gpioIn(pinIO_no_of_switches, 0, pinIO_switchState, pinIO_pinsA_in);

SlowHomeNet hNet();

void setup() {
    // byte id, i;
    Serial.begin(serial_speed);
    while (!Serial) {
        ;  // wait for serial port to connect. Needed for native USB port only
        delay(100);
    }
    Serial.println();
    Serial.println(F("Serial connected"));
    Serial.print(F("Board type: "));
    Serial.println(F(board_name));

    wdt_enable(WDTO_8S);

    Serial.println(F("No internal pullup mode for A6 & A7"));
    Serial.print(F("A0 = "));
    Serial.print(A0);
    Serial.print(F(", A7 = "));
    Serial.println(A7);  // Serial.print( F("D1 = "));Serial.print(D1);
                         // SetUpInputs();// Wall switches
    Serial.print(F("main.cpp: End of Setup(). Line No.: "));
    Serial.println(__LINE__);
    
    gpioIn.SetCallback(&gotInputPin);
}

void loop() {
    wdt_reset();
    gpioIn.SwitchesExe();  // Func is debounced
}
