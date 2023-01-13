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
 * switch pins Connected to switch directly 14(A0), 15(A1), 16(A2), A3(17) A6(20),A7(21) Set to pullup A6 and A7 can't be pullup
 * switch controller network             2,3,4
 * //  1-wire                                5,6,7
 * //  SPISerial Peripheral Interface        (8,9 select 2 SPI slaves,can be any pins)10,11,12,13
 * //  GPIO output, relay, Led, etc.     8, D3(3), 9, A3(17) Over lap with above.
 * //  I2C                            A4(18), A5(19)
 * //  leave D3,9 to last to test pwm. Going to need PWM multiplex or second arduino or mega.
 * //  A6 & A7 are analogRead(); only, Can't use pinMode(A6,INPUT_PULLUP). Need to add a pull up resistor in hardware.
 */

#include <defs.h>
#include <gpioSwitchInput.h>
#include <hn.h>

#include "/home/jmnc2/.platformio/packages/toolchain-atmelavr/avr/include/avr/wdt.h"

#define serial_speed 38400
#define homeNetPin 2
//#define pinIO_no_of_switches 6               // setup the number of gpio's used
//#define pinIO_inPins A7, A6, A0, A1, A2, A3  // in sa main.h
#define pinIO_no_of_switches 4               // setup the number of gpio's used
#define pinIO_inPins A0, A1, A2, A3  // in sa main.h

// byte pinIO_Max_switches = pinIO_no_of_switches;
byte pinIO_switchState[pinIO_no_of_switches];
byte pinIO_pinsA_in[pinIO_no_of_switches] = {pinIO_inPins};

SlowHomeNet hNet(homeNetPin);
unsigned long loopTimer;
word loopCount;

void gotInputPin(byte ioType, byte i, byte offset, byte count, byte state) {  // Callback when a switch changes
    Serial.print(F("ioType:"));
    Serial.print(ioType);
    Serial.print(F(", i:"));
    Serial.print(i);
    Serial.print(F(", count:"));
    Serial.print(count);
    Serial.print(F(", state:"));
    Serial.println(state);
    hNet.send(state,0);
}

gpioSwitchInputC gpioIn(pinIO_no_of_switches, 0, pinIO_switchState, pinIO_pinsA_in);


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
    // Serial.print(F("A0 = "));
    // Serial.print(A0);
    // Serial.print(F(", A7 = "));
    // Serial.println(A7);  // Serial.print( F("D1 = "));Serial.print(D1);
    //                      // SetUpInputs();// Wall switches
    gpioIn.SetUpInputs();
    Serial.print(F("main.cpp: End of Setup(). Line No.: "));
    Serial.println(__LINE__);

    gpioIn.SetCallback(&gotInputPin);

    loopCount = 0;
    loopTimer = micros();
}

void loop() {
    byte r;
    wdt_reset();
    gpioIn.SwitchesExe();  // Func is debounced
    hNet.exc();
    wdt_disable();
    r=hNet.receiveMonitor();// the chip will reset when the watch dog timer expires.
    wdt_enable(WDTO_8S);
    if (r==0){Serial.print(F("Message received, r = "));Serial.println(r);}
    loopCount++;

    if (loopCount >= 1000) {
        loopCount = 0;
        Serial.print(F("time for 1000 loops through the main loop"));
        Serial.println(micros() - loopTimer);
    }
}
