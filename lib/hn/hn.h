/**
 * @file hn.cpp
 * @author Joseph (you@domain.com)
 * @brief A communication protocol for a slow home network, including
 * functions for sending and receiving data packets, calculating CRC checksums,
 * and handling line contention.
 * @version 0.1.3
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025
 *
 * @details  * A slow Home Network using 1 or 2 GPIO pins. I am writing this for
 * sending messages to/from wired light switches with a MCU in the switch box
 * There is a 4 wire low voltage cable to each switch, for power and messages.
 *
 * For more detail about the protocol etc. see: DESIGN.md and the README.md
 *
 * hn is short fot Home Network here.
 */

#include "sharedVarsConf.h"

/// @brief A wired network using IO pins on an MCU. Slow with minimal hardware requirements.
/// @details This has some similarities with the CAN network but is much slower and don't need separate controller and transceiver chips.
/// @details I am using it to send messages between units on light switches and the units turning the lights on and off.
/// @details Although to avoid backfeed you may need a transistor a few resistors and 2 IO pins
class SlowHomeNet
{
  public:
    // +++++++++++++++++ Setup +++++++++++++++++++++++++++++++++++++++++
    void attachIntToPin(byte pin);
    byte getPinNo();
    explicit SlowHomeNet(byte pin);             // class setup procedure, auto called
    SlowHomeNet(byte pin, byte addDelayToSend); // class setup procedure, auto called

    //+++++++++++++++++ Receive ++++++++++++++++++++++++++++++++++++++++
    void exc(); // Need to call each time though the main loop.
    byte receiveMonitor();
    byte getFromBuf(byte a[], byte& RTR, byte& mLen, byte& dLen);

    /// @brief Get the number of bytes of received date stored in the receive buffer.
    /// @return bytes in buffer.
    byte recCount() { return buf.getLength(); }
    byte nextIndex() { return buf.nextIndex(); }

    //+++++++++++++++++ Send ++++++++++++++++++++++++++++++++++++++++
    byte setDataArray(byte command);
    byte setDataArray(byte command, byte data);
    byte setDataArray(byte command, word data);
    byte setDataArray(byte command, uint32_t data, byte l);
    byte sendHelper(byte RTR = 0, byte mLen = 1, byte dLen = 0, boolean lineFreeCheck = true);
    byte send(byte command);
    byte send(byte command, byte data);
    byte sendW(byte command, word data);

    //+++++++++++++++++++++++++ Misc ++++++++++++++++++++++++++++++++++++

    static inline byte getDataLen(byte l);
    static inline byte getMessageLen(byte l);
    static inline byte getMessageDataLen(byte l);
    static inline byte getLenCode(byte mLen, byte dLen);

    static inline byte Crc4(uint8_t* addr, uint8_t len);
    static inline byte CRC8bits(byte crc, uint8_t prev_crc = 0);
    inline byte Crc4buf(uint8_t i);

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
    uint8_t pin_bit_msk; // = digitalPinToBitMask(pin);
    uint8_t pin_port;    // = digitalPinToPort(pin);
    // for the MEGA the type may need to be changed to uint16_t
    volatile uint8_t* pin_DDR_reg; // = portModeRegister(port);
    volatile uint8_t* port_IO_reg; // volatile uint8_t *out = portOutputRegister(port);
    byte networkPin;
    word bitPulseLength = PulseLength; //2048; //  1 bit takes 2048 microseconds (~= 1e6 / lineSpeed;) (microsecond = 1 millionth of a second).

    word lineSpeed = 1e6 / bitPulseLength; // giving a line speed of 488 bits per second.
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
    word maxInuseHigh = ((42 * bitPulseLength) + (bitPulseLength >> 2)) / 1000 +
                        1; // = 88 = (42 x 2048 + 2048 >> 1) /1000 + 1 = (86,016 + 1024)/1000 + 1 = 87,040 /1000 +1 = 88.04=88
    // Only true when sending only 4 byte of data. 42 x 2048 + 2048 >> 1 = 87,041
    // +1 to round up, as dividing by 1000 is unlikely to be a whole number and int math always rounds down.

    // uint32_t maxInuseLow = maxInuseHigh + bitPulseLength;  // Max bits pulled low is 10. Pull low 1 tic to show start then could be 9 lows for data then
    // high for parity.
    word maxInuseLow =
      maxInuseHigh + bitPulseLength / 1000; // Max bits pulled low is 10. Pull low 1 tic to show start then could be 9 lows for data then high for parity.
    // byte size_of = sizeof(maxInuseLow);
    // word WaitForLineTimeout = 400;  // 4/10th of a second in millisecond (1e-3). different from more accurate timings that are in microseconds (1e-6)
    
    /// Line in use etc. see "#defines LineFree" etc. above.
    /// @note the "lineState" var is also used in the ISR
    byte lineState = LineUnmonitored; 
    

    //===========================================ISR vars=======================================
    // moved here from having as static in func as I think(?) that would limit to 1 pin.

    unsigned long lastTime; // In micros. 1/million of a second
    volatile unsigned long CurrentTime;

    // BitPos is the bit position of the last received bit, any lead-in bit(s) are not counted. Or stuffed bits.
    /// TODO: If only used in IntCallback(); might be better as a "static" type.
    /// @todo would using static function vars limit us to using 1 line/pin?
    // byte lastState = 1;  // used by the ISR, leaving this as a separate var as this could still be used at the same time as the send/receive funcs

    byte dFlags = 0; // parity bit is b00000001, ack is b00000010
    bool bufferOverflow = false;
    byte bitsCount = 0;
    byte bitsStore = 0;
    byte lastState;
    boolean expectStuffedBit =
      false; /// @brief After 5 bits in a row of the same level and if we are in the right part of the frame this is set to true so the next bit is removed
    byte messageLen = 0;
    

    byte bitPos, overflowCount; /// relay not sure about some of the vars as it maybe the same uses as above
    /// @todo I think the whole ISR function is probably far from working and needs rewriting

    //------------------------------------ISR vars-------------------------------------

    /// @note // not used??
    // byte bufIndexPartMessageAt = 0;  /// @brief The buffer array index for the start of the message we are part way through receiving and storing.

    /**
     * @brief bitCountUnchanged & lastBitLevel are used to add a bit of oppsite level when 5 or more bits sent are the same level and to remove
     * the added bit when receiving. This is for the normally called functions, ISR use different vars above.
     *
     * @details The values are tracked in the functions:
     * readBits(byte bits) when reading.
     * sendBits(byte bits, byte numberOfBits, boolean stuffBitOverride) when sending.
     * sendStartOfFrame() initializes the values. for sending
     * checkSOF() initializes the values. for reading
     * Maybe readBits should be changed so it only removes opersite bits after 5 consecutive bits, this way it could also read EOF and maybe long SOFs
     */
    byte bitCountUnchanged = 0;  /// The the number of bits of the same level sent. (Used to insert the opset bit if gets to 5.)
    boolean lastBitLevel = HIGH; /// the line level of the last bit sent or received
    byte
      dataArray[MaxMessageSize + MaxDataSize]; // beside using this to send different size messages and data, the CRC function wants it all in one array.
    byte RTRLenCode;                           // The RTR in the high bit plus the length code. Class var.
    byte mHandled;                             // This message was handled by a different unit.
    word lineMinGapMs = 100;                   // wait needed between sending 2 messages when checking in main loop
    // TODO should maybe use #ifdef to remove uneeded vars? When not needing to wait between messages etc.

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

#define DigitalWriteTime 4   // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5    // forums says 4.78µs but I think than includes the for loop
#define ReadBitsLoopMicros 3 // this is for each time though the loop that checks 8 time per bit so a value of 3 would be 24µs per bit

    byte parityErrorCount = 0; // If parity fail int this and discard. Not put in buffer.

    circular_bufC buf;

    boolean monitorLinePinForChange(byte pulses, byte level);
    boolean monitorLinePinForChangeMs(word ms, byte level = 1);
    boolean checkPinInput();
    byte checkLineFreeState(boolean wait, word timeout);
    byte readBit();
    byte readBits(byte bits);

    // byte getPulseNo(byte pulses, byte level);

    byte pushDataLen(byte l, byte RTR);
    byte pushMessageId(byte m);

    // Private send data to line functions
    boolean sendBitH();
    void sendBitL();
    byte sendBits(byte bits, byte numberOfBits, boolean stuffBitOverride = false);

    byte sendStartOfFrame();
    byte sendRTR(byte v);
    byte sendDataLen(byte v);
    byte sendMessageId(byte v);
    byte sendData(byte v);
    word sendData(word v);
    byte sendCRC(byte v);
    byte sendAck(byte v);
    byte sendEndOfFrame();

    boolean getNetwork();
    byte checkSOF();
    byte receiveRest(byte bitPos);

    void IntCallback(); // Store 3 line change timings and discared any short enough to be bounce or a line spike. Although would this be a thing?
};

#endif
