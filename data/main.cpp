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

#include <avr/wdt.h>
#include <Arduino.h>

#include <defs.h>
#include <I.h>

void setup()
{
  // byte id, i;
  Serial.begin(38400);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.println(F("Serial connected"));
  Serial.print(F("Board type: ")); Serial.println(F(board_name));

  wdt_enable(WDTO_8S);
  
  Serial.println(F("No internal pullup mode for A6 & A7"));
  Serial.print(F("A0 = "));
  Serial.print(A0);
  Serial.print(F(", A7 = "));
  Serial.println(A7); // Serial.print( F("D1 = "));Serial.print(D1);
                      // SetUpInputs();// Wall switches
  Serial.print(F("main.cpp: End of Setup(). Line No.: "));
  Serial.println(__LINE__);
}

void loop()
{
  wdt_reset();
  SwitchesExe(); // Func is debounced
}
