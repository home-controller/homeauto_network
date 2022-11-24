#ifndef _hn_h
#define _hn_h
/**
 * @file hn.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-09
 * 
 * @copyright Copyright (c) 2022
 * 
 * hn is short fot Home Network here. 
 * Not to be confused with Ethernet.
 * This will be kind of like a very cut down and slow CAN network. So there will be no need for network hardware.
 * 
 * Lets go with:
 * 1: A pull down pulse to say I am about to start sending.
 * 2: Then 8 bits of data
 * 3: Then a parity. Also makes it so each 9 bits + parity(10 bits total) sent have to have at least 1 each of having the line HIGH and LOW.
 * 4: For now use extra bit for expecting acknowledgement back, OK RESEND etc.
 * 5: TODO Maybe do checksums
 * 6: Todo: should probably use CAN style, add a inverted bit if long sequence of high or low bits instead of relying on parity bit.
 */

/**
 * xxxB- this is used by digital pin 8 to digital pin 13
 * xxxC-this is used by analog pins
 * xxxD- this is used by digital pin 0 to digital pin 7
 * 
 * DDRx makes pins input or output
 * PORTx makes pin LOW or HIGH
 * PINx this is for reading the state of input pins
 * 
 * DDRB, PORTB, PINB-each with 8 bits for pins D8 to D13   = PB0 to PB5    = PCINT0 to PCINT5 (bit 6(PB6) and 7 must be high because they are for crystal)
 * DDRC, PORTC, PINC-each with 8 bits for pins A0 to A5(7) = PC0 to PC5(7) = PCINT8 to PCINT13(15)(we can control only 6, pins 6 and 7 are available on other boards).
 * DDRD, PORTD, PIND-each with 8 bits for pin D0 to D7     = PD0 to PD7    = PCINT16 to PCINT23
 * 
 * PCICR last 3 bit enable pin change interrupts for pin registers, bit 0 for register B (D8-D13 = PCINT0 to PCINT5) and so on.
 * PCIFR flage pins changed, as above with bit 0 for reg b, bit 1 for reg c(A0to A5) and bit 2 for pins D0 to D7(PORTD)
 * PCMSK0, PCMSK1 and PCMSK2 are the 8 bit bitmask register for the above pin back registers. PCMSK0 for bank B etc, with 1 bit per pin.
 * 
 * 
 * 
 */
#define pinIntEn 1<<2
//                      76543210
#define pinD_intMsk = 0x00000100 // pin D2
 
#include <Arduino.h>


#define MaxInUseHigh
class SlowHomeNet {
  public:
    void attachIntToPin(byte pin);
    SlowHomeNet(uint8_t l, uint8_t pinNo);//class setup procedure, auto called
  private:
    volatile boolean lock = false;
    #define _pinReg PIND // read PIND for pins D0 to D7 states
    #define _pinMask 0b00000100;//Mask for third pin in reg. i.e. on PIND mask for D2
    #define _hn_int_pin 2
    word lineSpeed = 488;//maybe should chang from 600 to 488 as this allows shifting right 11 to give the approx no of bits from timing in us while and gives the mod
    word bitPulseLength = 1e6 / lineSpeed; // in microseconds 1,666.66...  488 ~= 1e6/2048
      //1e6 is the number of microseconds in a second
      /*
       * Max in use high is 9. 9 High data bits, Then parity would be low. Add 1/2 pulse for timing errors.
       * Forgot you can have 8 1s then high parity then 9 x 1s for next data then 0 parity. Total of 18 high in a row if sending 2 bytes of data.
       * As we are trying to go as slow as posable without making to noticeable a delay between turning the switch and the light coming on stick 
       * to sensing 1 byte at a time before dropping the line for maxInuseHigh + a bit to give others a chance.
       * TODO Maybe do like CAN and add a bit of the opposite logic level after x bits of the same logic level then drop it at the other end. 
       *      This guarantees a max line high or low while sending data without any worry about the bits being sent.
       * before we have CAN style it is to painfully without a stop bit. Adding one pulse after the parity the opposite of it so we always get
       * our date without having to check, also makes error checking easier.
       */
    word maxInuseHigh = (9 * bitPulseLength) + (bitPulseLength >> 2);    //  Only true when sending only 1 byte of data at a time.
    word maxInuseLow = maxInuseHigh + bitPulseLength;  // Max bits pulled low is 10. Pull low 1 tic to show start then could be 9 lows for data then high for parity.

    unsigned long lastTime;// In micros. 1/million of a second
    volatile unsigned long CurrentTime;
    byte bitPos = 0;
    byte dFlags = 0; // parity bit is b00000001, ack is b00000010
    byte overflowCount = 0;// to many bits sent without ensuring line change at end of 11 bits
    byte lastState = 1;
    
    byte dataIn = 0;
/*
 * 1 byte will Do for now. might do for most commands? 3 bit command() + 5 for data. eg. command = turn on light, data = light number.
 * B1xxyyyyy directly (turn on xx=01) (turn off xx=00) (toggle xx=10) light no. up to yyyyy=b11111=32
 * B1xxyyyyy when xx=11 set curent temp with yyyyy being a packed temp code
 * B0xxyyyyy group (turn on xx=01) (turn off xx=00) (toggle xx=10) group no. up to yyyyy=b11111=32. each group can be 32 bits(4 bytes) in eeprom
 * 16 commands(buff size) should be plenty? If you go mad flicking switches while spamming the web page you only have yourself to blame :P
 * Needs to be 2^x as I use (bufLength - 1) bitand (head + length) to get the mod in the code :)
 * Maybe make the commands and data the same size as standard CAN, Would also mean you could just change to CAN 
 * network with not much more than a libary change to the code.
 */


 #define bufIndexBits 4  
 #define bufLength (2 << bufIndexBits) // 2^4 gives the 16 for bufLength 
 #define bufMsk (bufLength - 1) //B1111 // = 2^4 - 1
    byte buff[bufLength];// 2^x so can use shift for math.
    byte headP = 0;
    byte lengthB = 0;

    byte parityErrorCount = 0;// If parity fail int this and discard. Not put in buffer.
    
    void IntCallback();// Store 3 line change timings and discared any short enough to be bounce or a line spike. Although would this be a thing?

  
};

#endif
