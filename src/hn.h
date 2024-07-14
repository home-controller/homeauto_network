#ifndef _hn_h
#define _hn_h

/*
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

#include <Arduino.h>
#include <circular_buf.h>
// #include "../../libraries/circular_buf/src/circular_buf.h"

#define MaxInUseHigh
#define CRCError
#define SOFBits 2  // The number of SOF (Start of Frame) bits.
#if SOFBits > 1
#define SOFValue 0b01  // if the number of bit is greater than 1 pull low for (SOFBits - 1) bits then 1 hight bit.
#else
#define SOFValue 0  // else pull low for 1 bit.
#endif
#define maxDataSize 4         // the maximum data frame size in bytes, the is separate for the message frame.
#define maxMessageSize 1      // The maximum massage size in bytes, TODO: can only be 1 at the min.
#define _pinReg PIND          // read PIND for pins D0 to D7 states
#define _pinMask 0b00000100;  // Mask for third pin in reg. i.e. on PIND mask for D2
#define _hn_int_pin 2
#define DataLengthBitsLn 3  // the number of bits storing the message and data frame length in code.

#define Error_NoError 0         //  0,  Successfully sent and received Ack.
#define Error_LineError 1       //  1,  line error.
#define Error_NoRoomInBuffer 3  //  3,  Not enough or no room to store the info needed in the buffer.
#define Error_AckError 16       //  16, A unit signaled an Ack error, it failed to receive the message. For example CRC failed.
#define Error_LostPriority 17   //  17, Higher priority message being sent, received in buffer.

///  18, could be network SOF mismatch on different units,
/// or network down or not reading all incoming messages properly
/// or not checking for if in middle of message for example at program start
/// or if receiving messages and only sending them and not checking for line free.
#define Error_NetworkProblem 18
#define Error_UnhandledDataSize 19     // 19 unhandled data size.
#define Error_AnotherUnit_AckError 20  // Another unit signaled a receive error, i.e. it failed it's CRC check.
#define Error_EOFCodeStored 21         // End of frame error code stored in private class var: endOfFrameError
#define Error_LineErrorFrameStart 22

class SlowHomeNet {
 public:
  // +++++++++++++++++ Setup +++++++++++++++++++++++++++++++++++++++++
  void attachIntToPin(byte pin);
  byte getPinNo();
  explicit SlowHomeNet(byte pin);  // class setup procedure, auto called

  //+++++++++++++++++ Receive ++++++++++++++++++++++++++++++++++++++++
  void exc();  // Need to call each time though the main loop.
  byte receiveMonitor();

  /// @brief Get the number of bytes of received date stored in the receive buffer.
  /// @return bytes in buffer.
  byte recCount() { return buf.getLength(); }
  byte nextIndex() { return buf.nextIndex(); }

  //+++++++++++++++++ Send ++++++++++++++++++++++++++++++++++++++++
  byte setDataArray(byte command, byte data);
  byte setDataArray(byte command, word data);
  byte setDataArray(byte command, uint32_t data, byte l);
  byte sendHelper(byte RTR, byte mLen, byte dLen);
  byte send(byte command, byte data);

  //+++++++++++++++++++++++++ Misc ++++++++++++++++++++++++++++++++++++

  byte Crc4(uint8_t *addr, uint8_t len);
  byte Crc4buf(uint8_t i);

  /// @brief Get the value in the queue i items back front the head of the queue. No range checking.
  /// @param i if i = 0 then the first item at the head of the queue, else i bytes back from the head
  /// @return byte of data from the buffer.
  byte peek(byte i) { return buf.peek(i); }

  /* While for sending we can send the message and wait for it to be send without to much problems. At least if there is
   * not to much line contention and we are not in to much of a rush to do other stuff.
   *
   * But for receiving just sitting and monitoring the line and blocking everything else will likely be a problem so we need a few more
   * options:
   * 1: If our main loop is fast enough to catch any message while it is still on the
   */

  // unit testing stuff
#ifdef UnitTest
  // To read a bit on the line we divide the time for 1 bit by 8 and check at each point in time. this is the point in time
  // 1 is the start and 8 is the end. We can use this to provide dummy test values for testing.
  byte inBitPos;
#endif

 private:
  // wouldn't bother with storing this in SRAM but for to keep the ISR faster.
  uint8_t pin_bit_msk;  // = digitalPinToBitMask(pin);
  uint8_t pin_port;     // = digitalPinToPort(pin);
  // for the MEGA the type may need to be changed to uint16_t
  volatile uint8_t *pin_DDR_reg;  // = portModeRegister(port);
  volatile uint8_t *port_IO_reg;  // volatile uint8_t *out = portOutputRegister(port);
  byte networkPin;
  word bitPulseLength = 2048;  //  1 bit takes 2048 microseconds (~= 1e6 / lineSpeed;) (microsecond = 1 millionth of a second).

  word lineSpeed = 1e6 / bitPulseLength;  // giving a line speed of 488 bits per second.
                                          // Changed from 600 to 488 as this allows shifting right 11 to divide by 2048.
                                          // so number or bits can be given by t >> 11 and the remaining time by t bitand (2048 - 1)

  /// For storing any error codes from other units.
  /// TODO: As well as storing end of frame error codes could also store any code added mid frame by pulling low for 6 connective bits
  /// this type off lower level error handling is not implemented yet nd my not be.
  byte endOfFrameError = 0;

  // To be acuate 1e6/2048 would be a line speed of of  488 + 9 ∕ 32 bits per second. P.S. 1e6 = 1 million, the number of microseconds in a second
  // This is truncated to an int, giving 488  bits per second line speed
  // With delays for code execution and differing clock speeds on different processors on the line etc. timings will probably not be that acuate anyway.
  //
  // As lineSpeed is probably only used for the UI or people reading here it's accuracy probably don't matter much anyway.

  /*
   * Gives max number of *miliseconds* the line can be high while in use and sending a message,
   * With 4 bytes of date max is maybe 42 high bits, see readme. Just under a 10th of a second?
   * TODO Maybe do like CAN and add a bit of the opposite logic level after x bits of the same logic level then drop it at the other end.
   *      This guarantees a max line high or low while sending data without any worry about the bits being sent.
   *      For example if 8 bit in a row are the same logic level then insert an opposite bit.
   *      This can seem superfluous if the 9th bit would of changed anyway but we need to do it like this so the receiver can know to delete the bit.
   * before we have CAN style it is to painfully without a stop bit. Adding one pulse after the parity the opposite of it so we always get
   * our date without having to check, also makes error checking easier.
   */
  word maxInuseHigh =
      ((42 * bitPulseLength) + (bitPulseLength >> 2)) / 1000 + 1;  // = 88 = (42 x 2048 + 2048 >> 1) /1000 + 1 = (86,016 + 1024)/1000 + 1 = 87,040 /1000 +1 = 88.04=88
  // Only true when sending only 4 byte of data. 42 x 2048 + 2048 >> 1 = 87,041
  // +1 to round up, as dividing by 1000 is unlikely to be a whole number and int math always rounds down.

  // uint32_t maxInuseLow = maxInuseHigh + bitPulseLength;  // Max bits pulled low is 10. Pull low 1 tic to show start then could be 9 lows for data then high for parity.
  word maxInuseLow = maxInuseHigh + bitPulseLength / 1000;  // Max bits pulled low is 10. Pull low 1 tic to show start then could be 9 lows for data then high for parity.
  // byte size_of = sizeof(maxInuseLow);
  word WaitForLineTimeout = 400;  // 4/10th of a second in millisecond (1e-3). different from more accurate timings that are in microseconds (1e-6)

  unsigned long lastTime;  // In micros. 1/million of a second
  volatile unsigned long CurrentTime;

  /// @brief pitPos is the bit position of the last received bit, any lead-in bit(s) are not counted. Or stuffed bits.
  /// TODO: If only used in IntCallback(); might be better as a "static" type.
  byte bitPos = 0;                 /// used in IntCallback(); (when using pin change interrupts)
  byte bufIndexPartMessageAt = 0;  /// @brief The buffer array index for the start of the message we are part way through receiving and storing.
  byte dFlags = 0;                 // parity bit is b00000001, ack is b00000010
  byte overflowCount = 0;          // to many bits sent without ensuring pin level change at end of 5 bits
  byte lastState = 1;
  byte dataArray[maxDataSize + maxMessageSize];  // beside using this to send different size messages and data, the CRC function wants it all in one array.
  byte RTRLenCode;                               // The RTR in the high bit plus the length code.

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

#define DigitalWriteTime 4  // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5   // forums says 4.78µs but I think than includes the for loop

  byte parityErrorCount = 0;  // If parity fail int this and discard. Not put in buffer.

  circular_bufC buf;

  boolean monitorLinePinForChange(byte pulses, byte level);
  byte sendBits(byte bits, byte numberOfBits);
  boolean checkPinInput();
  byte readBits(byte bits);

  byte getPulseNo(byte pulses, byte level);
  byte getDataLen(byte l);
  byte getMessageLen(byte l);
  byte getMessageDataLen(byte l);
  byte getLenCode(byte mLen, byte dLen);

  byte pushDataLen(byte l, byte RTR);
  byte pushMessageId(byte m);

  byte sendStartOfFrame();
  byte sendRTR(byte v);
  byte sendDataLen(byte v, byte);
  byte sendMessageId(byte v);
  byte sendData(byte v);
  word sendData(word v);
  byte sendCRC(byte v);
  byte sendAck(byte v);
  byte sendEndOfFrame();

  boolean getNetwork();
  byte checkSOF();
  byte receiveRest(byte bitPos);

  void IntCallback();  // Store 3 line change timings and discared any short enough to be bounce or a line spike. Although would this be a thing?
};

#endif
