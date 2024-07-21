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
 * switch pins Connected to switch directly 14(A0), 15(A1), 16(A2), A3(17) A6(20),A7(21) Set to pullup, A6 and A7 can't be pullup on some chips
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

#include <EEPROM.h>
// #include "defs.h"
#include <defs.h>
// #include "../../libraries/defs/src/defs.h"
#include <gpioSwitchInput.h>
#include "hn.h"
/* Including the watchdog timer header file. */
#include "/home/jmnc2/.platformio/packages/toolchain-atmelavr/avr/include/avr/wdt.h"

#ifdef OLED_I2C
// #include <Adafruit_SSD1306.h>
// #define WIRE Wire
// Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);
#include <Adafruit_SH110X.h>

#define i2c_Address 0x3c  // initialize with the I2C addr 0x3C Typically eBay OLED's
#define OLED_RESET -1     //   QT-PY / XIAO
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#endif

#define serial_speed 38400
#define homeNetPin 2
// #define pinIO_no_of_switches 6               // setup the number of gpio's used
// #define pinIO_inPins A7, A6, A0, A1, A2, A3  // in sa main.h
#define pinIO_no_of_switches 4       // setup the number of gpio's used
#define pinIO_inPins A0, A1, A2, A3  // in sa main.h

#define NextBoardId 28
#define eepromIdAddr 250

// byte pinIO_Max_switches = pinIO_no_of_switches;
byte pinIO_switchState[pinIO_no_of_switches];
byte pinIO_pinsA_in[pinIO_no_of_switches] = {pinIO_inPins};

SlowHomeNet hNet(homeNetPin);
unsigned long loopTimer;
word loopCount;

/**
 * @brief Callback function called when a switch changes
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
  Serial.print(hNet.getPinNo());
  Serial.println(").");
  hNet.send(state, 0);
}

gpioSwitchInputC gpioIn(pinIO_no_of_switches, 0, pinIO_switchState, pinIO_pinsA_in);

#define STRINGIFY_(b) #b
#define STRINGIFY(b) STRINGIFY_(b)

// #define q__ @"

/**
 * The function `setup()` is called once when the program starts. It sets up the
 * serial port, the watchdog timer, and the input pins
 */
void setup() {
  byte id;
  byte NBId = NextBoardId;
  id = EEPROM.read(eepromIdAddr);
  // id=0xFF; hard set of board id
#if defined hard_set_boardID
  Serial.println(F("Hard setting board ID to passed C flag value"));
    id == 0xFF);
    NBId = hard_set_boardID;
#elif defined suggest_boardID
  if (id == 0xFF) Serial.println(F("Next board ID passed as a C flag"));
  NBId = suggest_boardID;
#endif
    if (id == 0xFF) {
      Serial.print(F("Board ID not set, setting board ID to:"));

      Serial.println(NBId);
      EEPROM.update(eepromIdAddr, NBId);
      id = NBId;
    } else {
      Serial.print(F("Got board ID: "));
      Serial.println(id);
    }
    // byte id, i;
    Serial.begin(serial_speed);
    while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
      delay(100);
    }
    Serial.println();
    Serial.println(F("Serial connected"));
    Serial.println(F("program version: " STRINGIFY(VERSION)));  // VERSION is build flag in platformio.ini
    Serial.print(F("Board type: "));
    Serial.println(F(board_name));

    wdt_enable(WDTO_8S);
    if ((Nano_board == board_type) or (Uno_board == board_type)) { Serial.println(F("No internal pullup mode for A6 & A7")); }
    if (Pro_Micro_board == board_type) { Serial.println(F("Pro Micro has missing pins including AREF, A4, A5, SS, 11, 12, and 13.")); }
#if defined gpioDebugSetup
    Serial.println(F("Debug flag \"gpioDebugSetup\" set"));
#endif
    Serial.print(F("Home_net pin: "));
    Serial.println(homeNetPin);

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
    //   display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
    display.begin(i2c_Address, true);  // Address 0x3C default
    display.cp437(true);

    Serial.println("OLED begun");
    // Show image buffer on the display hardware.
    // Since the buffer is initalized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(3000);  // watchdog timer will reset the MCU if the delay is for to long.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println("Start Network testing");
    display.display();
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
 * The function is called every time through the main loop. It checks the
 * switches, executes the network, and checks for a message
 */
void loop() {
  static unsigned long sendCTime = millis();
  static unsigned long sendLastTime = sendCTime - 10000;
  static boolean oneTime = false;
  if (!oneTime) {
    Serial.print(F("Pin number = "));
    Serial.println(hNet.getPinNo());
    oneTime = true;
    Serial.print(F("Pin state = "));
    if (digitalRead(hNet.getPinNo()) == LOW) Serial.println("LOW");
    else Serial.println("HIGH");
    byte ta[2]{1, 7};
    byte crc = hNet.Crc4(ta, 2);
    Serial.print(F("CRC for [1,7] = "));
    Serial.println(crc);
  }
#ifdef receive_buildflag
  byte r;
#endif
  static byte c = 0;
  wdt_reset();
  gpioIn.SwitchesExe();  // Func is debounced
  hNet.exc();            // TODO: doing nothing for now
#ifdef receive_buildflag
  /// really need to
  wdt_disable();

  /*  // debug loop______________________#=========================
   byte a_l;  // pin changes stored.
   unsigned long
       a[20];  // for storing pin changes in millis for for the message. although the is a max of 59bit some are always at a set value and only use 2 date bytes max for now.
   byte a2[20];
   boolean pinLevel, lastLev;
   // Serial.print('.');
   byte p = hNet.getPinNo();
   pinLevel = digitalRead(p);
   if (pinLevel == LOW) {
     lastLev = LOW;
     a_l = 0;
     sendCTime = micros();
     sendLastTime = sendCTime;
     unsigned long fistTime;
     fistTime = sendCTime;
     while ((sendCTime - fistTime) < 1000000)  // Check for about 1 second.
     {
       pinLevel = digitalRead(p);
       if (lastLev != pinLevel) {
         if (a_l <= 20) {
           a[a_l] = sendCTime - sendLastTime;
           a2[a_l] = lastLev;
           sendLastTime = sendCTime;
         }
         lastLev = pinLevel;
         if (a_l < 255) a_l++;
       }
       sendCTime = micros();
     }
     Serial.print(F("Pin toggled between low/high "));
     Serial.print(a_l + 1);
     Serial.println(F(" times"));
     if (a_l >= 1) {
       Serial.print(F("The first pull LOW pulse lasted approx "));
       Serial.print(a[0]);
       Serial.print(F(" microseconds, Microseconds for subsequent changes: "));
       byte i;
       for (i = 1; ((i <= a_l) and (i < 20)); i++) {
         if (i > 1) Serial.print(", ");
         Serial.print(a[i]);
       }
       Serial.println();
     }
   } */
  Serial.println(F("calling receiveMonitor"));
  r = hNet.receiveMonitor();  // the chip will reset when the watch dog timer expires.
  wdt_enable(WDTO_8S);
  if (r == 0) {
    byte a[5], rtr, dl, ml;
    Serial.print(F("Message received with no error(0)"));
    //Serial.print(r);
    Serial.print(F(", buffer len: "));
    Serial.println(hNet.recCount());
    Serial.println(F("calling getFromBuf"));
    byte t = hNet.getFromBuf(a, rtr, ml, dl);
    Serial.print(F("buffer len after pop: "));
    Serial.print(hNet.recCount());
    Serial.print(F(", getFromBuf(): "));
    Serial.print(t);
    Serial.print(F(", message bytes: "));
    Serial.print(ml);
    Serial.print(F(", data bytes: "));
    Serial.print(dl);
    Serial.print(F(", Remote Transmission Request: "));
    Serial.print(rtr);
    byte lc = 0;
    while ((ml + dl) > lc) {
      Serial.print(F(", data ["));
      Serial.print(lc);
      Serial.print(F("] = "));
      Serial.print(a[lc]);
      // Serial.println();
      lc++;
    }
    Serial.println();
  } else {
    Serial.print(F("Error receiving message, error = "));
    Serial.println(r);
  }
#endif
#ifdef send_buildflag
  sendCTime = millis();
  if ((sendCTime - sendLastTime) >= 15000) {
    static byte sc = 0;
    sendLastTime = sendCTime;
    delay(2000);
    sc = hNet.send(1, (byte)7);
    byte sc2 = hNet.sendW(2, 0xFFFF);
    Serial.print(F("Message sent "));
    Serial.print(sc);
    Serial.print(F(", Second Message sent "));
    Serial.print(sc2);
    if (sc == Error_AckError) Serial.print(F(": A unit signaled an Ack error, likely CRC fail. "));
    Serial.println();
  }
#endif

  loopCount++;

  if (loopCount >= 5000 and (c <= 5)) {  // maxsize of a word is 65535 so if over that will never pass
    loopCount = 0;
    if (c == 0) Serial.println(F("Time per pulse is µs / 488(board rate, bits per second) = 1e6 / 488 = 2048µs per bit."));
    Serial.print(F("Count: "));
    Serial.print(c);
    Serial.print(F(", "));
    Serial.print(micros() - loopTimer);
    Serial.print("µs.");
    Serial.print(F(" for 5,000 loops through loop(): Time to check each bit "));
    Serial.print((micros() - loopTimer) / 2048);
    Serial.println(F(" times."));
    loopTimer = micros();
    c++;
  }
}
