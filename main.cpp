/*
 A simple home network Libary

 A slow network for communication on a Arduino GPIO pins with the protocol modeled
 after the CAN network but cut down for less bits per command and much slower bit
 rate. It needs to be slower as no line driver chip and without the 
 proticol deicated hardware and the MCU running other stuff there will be delaiys
 in detecting line high low transsitions.

 Circuit:
 * Ethernet shield attached with SPI to pins 10, 11, 12, 13 + 9 for reset
 * Output for relays on pins 3, 4 See relays.h tab
 */

#include <avr/wdt.h>


void setup() {
  //byte id, i;
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.println(F("Serial connected") );
  Serial.print(F("Board type: ") );
  Serial.print( eth_type );
  if (arduino_type == Uno_board) {Serial.print(" Uno "); }
  else if (arduino_type == Nano_board) {Serial.print(" Nano, "); }
  if (eth_chip == _W5100){ Serial.println("W5100"); }
  else if (eth_chip == _W5500){ Serial.println("W5500"); }
  else {Serial.println();}
  
  
   wdt_enable(WDTO_8S);
  // MQTT setup.
  //Serial.println(F("Call MQTT_setup") );
 // wdt_disable();
  Serial.println( F("No internal pullup mode for A6 & A7"));
  Serial.print( F("A0 = "));Serial.print(A0); Serial.print( F(", A7 = "));Serial.println(A7);// Serial.print( F("D1 = "));Serial.print(D1);
 // SetUpInputs();// Wall switches
  Serial.print( F("main.cpp: End of Setup(). Line No.: "));Serial.println(__LINE__);
}

void loop() {
  wdt_reset();
    SwitchesExe(); //Func is bebounced
}
