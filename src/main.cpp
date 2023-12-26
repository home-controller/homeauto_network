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
 *
 * ================This one is us=======================
 * + switch controller network             2,3,4       +
 * =====================================================
 *
 * //  1-wire                                5,6,7
 * //  SPISerial Peripheral Interface        (8,9 select 2 SPI slaves,can be any pins)10,11,12,13
 * //  GPIO output, relay, Led, etc.     8, D3(3), 9, A3(17) Over lap with above.
 * //  I2C                            A4(18), A5(19)
 * //  leave D3,9 to last to test pwm. Going to need PWM multiplex or second arduino or mega.
 * //  A6 & A7 are analogRead(); only, Can't use pinMode(A6,INPUT_PULLUP). Need to add a pull up resistor in hardware.
 */

// #include <defs/src/defs.h>
#include <defs.h>
// #include "../../libraries/defs/src/defs.h"
#ifdef OLED_I2C
#include <Adafruit_SSD1306.h>
#define WIRE Wire

#endif

#include <gpioSwitchInput.h>

#include "hn.h"

/* Including the watchdog timer header file. */
#include "/home/jmnc2/.platformio/packages/toolchain-atmelavr/avr/include/avr/wdt.h"

#define serial_speed 38400
#define homeNetPin 2
// #define pinIO_no_of_switches 6               // setup the number of gpio's used
// #define pinIO_inPins A7, A6, A0, A1, A2, A3  // in sa main.h
#define pinIO_no_of_switches 4       // setup the number of gpio's used
#define pinIO_inPins A0, A1, A2, A3  // in sa main.h

// byte pinIO_Max_switches = pinIO_no_of_switches;
byte pinIO_switchState[pinIO_no_of_switches];
byte pinIO_pinsA_in[pinIO_no_of_switches] = {pinIO_inPins};

SlowHomeNet hNet(homeNetPin);
unsigned long loopTimer;
word loopCount;

/**
 * `gotInputPin` is called when a switch changes
 *
 * @param ioType The type of input.  This is always 0 for a switch.
 * @param i The index of the input pin that changed.
 * @param offset The offset of the pin in the array of pins.
 * @param count The number of times the switch has changed state.
 * @param state The state of the switch.  0 for off, 1 for on.
 */
void gotInputPin(byte ioType, byte i, byte offset, byte count, byte state) {  // Callback when a switch changes
    Serial.print(F("ioType:"));
    Serial.print(ioType);
    Serial.print(F(", i:"));
    Serial.print(i);
    Serial.print(F(", count:"));
    Serial.print(count);
    Serial.print(F(", state:"));
    Serial.println(state);
    Serial.print(F("Sending state on network(pin "));
    Serial.print(hNet.pin());
    Serial.println(").");
    hNet.send(state, 0);
}

gpioSwitchInputC gpioIn(pinIO_no_of_switches, 0, pinIO_switchState, pinIO_pinsA_in);

#define STRINGIFY_(b) #b
#define STRINGIFY(b) STRINGIFY_(b)

#ifdef OLED_I2C
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);
#endif
// #define q__ @"

/**
 * The function `setup()` is called once when the program starts. It sets up the serial port, the
 * watchdog timer, and the input pins
 */
void setup() {
    // byte id, i;
    Serial.begin(serial_speed);
    while (!Serial) {
        ;  // wait for serial port to connect. Needed for native USB port only
        delay(100);
    }
    Serial.println();
    Serial.println(F("Serial connected"));
    Serial.println(F("program version: " STRINGIFY(VERSION)));
    Serial.print(F("Board type: "));
    Serial.println(F(board_name));

    wdt_enable(WDTO_8S);

    Serial.println(F("No internal pullup mode for A6 & A7"));
#if defined gpioDebugSetup
    Serial.println(F("Debug flag \"gpioDebugSetup\" set"));
#endif

#if defined send_buildflag
    Serial.println(F("Debug flag \"send_buildflag\" set"));
#else
#if defined receive_buildflag
    Serial.println(F("Flag \"receive_buildflag\" set"));
#else
    Serial.println(F("No flag for \"send_buildflag\" or \"receive_buildflag\" set"
#endif
#endif

#ifdef OLED_I2C
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Address 0x3C for 128x32
    Serial.println("OLED begun");
    // Show image buffer on the display hardware.
    // Since the buffer is initalized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(1000);
#endif

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

/**
 * The function is called every time through the main loop. It checks the switches, executes the
 * network, and checks for a message
 */
void loop() {
#ifdef receive_buildflag
    byte r;
#endif
    static byte c = 0;
    wdt_reset();
    gpioIn.SwitchesExe();  // Func is debounced
    hNet.exc();
#ifdef receive_buildflag
    wdt_disable();
    r = hNet.receiveMonitor();  // the chip will reset when the watch dog timer expires.
    wdt_enable(WDTO_8S);
    if (r == 0) {
        Serial.print(F("Message received, r = "));
        Serial.println(r);
    } else {
        Serial.print(F("Error receiving message, r = "));
        Serial.println(r);
    }
#endif
#ifdef send_buildflag

#endif

    loopCount++;

    if (loopCount >= 50000 and (c <= 5)) {  // maxsize of a word is 65535 so if over tha will never pass
        loopCount = 0;
        Serial.print(F("Count: "));
        Serial.print(c);
        Serial.print(F(", Time for 50,000 loops through the main loop: "));
        Serial.print(micros() - loopTimer);
        Serial.print(F("µs / 488 Enough time to check each bit "));
        Serial.print((micros() - loopTimer) / 488);
        Serial.println(F(" times."));
        loopTimer = micros();
        c++;
    }
}
