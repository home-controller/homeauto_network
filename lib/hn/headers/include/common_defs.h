#pragma once

#include "mcu_defs.h"
#include "config.h"

#define AllowPullLineLowForBusy false

#define dominantLineLevel LOW   // The line is pulled LOW by the sending unit, i.e. the dominant level on the line when sending a message.
#define recessiveLineLevel HIGH // The line is pulled HIGH by the receiving unit, i.e. the recessive level on the line when not sending a message.

// #define CRCError
// moved to config.h #define SOFBits 2       /// @brief The number of SOF (Start of Frame) bits.
#define FrameInfoBits 4 /// RTR (Remote Transmission Request) bit + 3 bits for the data length
#define CRCBits 5       // 4 CRC bits + 1 Delimiter
#define AckBits 5
#define EOFBits 7
#define InterframeSpaceBits 3 // 3 bits of spacing between frames, this is not counted in the total frame bits.
#define TotalFrameBits (SOFBits + FrameInfoBits + CRCBits + AckBits + EOFBits) // = 2+4+5+4+7 = 22 Total frame bits not counting and message or data bits.
#define BeforeMessageBits (SOFBits + FrameInfoBits)
#define BeforeEOFBits (FrameInfoBits + CRCBits + AckBits) /// @brief This is not counting any SOF bits

#define MaxInUseHighBits 9


#if SOFBits > 1
#define SOFValue 0b01 // if the number of bit is greater than 1 pull low for (SOFBits - 1) bits then 1 hight bit.
#else
#define SOFValue 0 // else pull low for 1 bit.
#endif
///
#define MaxDataSize 4        // the maximum data frame size in bytes, the is separate for the message frame. Can only be 0,1,2,4,8,16,32 byte
#define MaxMessageSize 1     // The maximum massage size in bytes.
#define DataLengthBitsLn 3                          // the number of bits storing the message and data frame length in code.
enum {maxMessageIdBits = (8 * MaxMessageSize)}; // Maximum message ID bits, currently only

#define PulseLength 2048 // 1 bit takes 2048 microseconds (~= 1e6 / 488 = 2049.18) (microsecond = 1 millionth of a second).
#define BitsPerSecond                                                                                                                                     \
  488 // this is approx [microseconds in a second]1e6 / 1e6/2048 [1e6/2048=488.28]
      // used 488 & 2048 as can then shift right 11 to divide by 2048
      // and the number of bit in a given pulse length can be given by:
      // bits = t >> 11 and the remaining time by t bitand (2048 - 1)

// Max number of high bits while sending a message should be 7( see MaxInUseHighBits above) but as bit stuffing is not implemented
// yet maybe 52(from DESIGN.md).
// That would give a max high time of: 52 / BitsPerSecond = 52/488 ≅ 0.1 seconds = 100 miliseconds
// 7 bits would be 7/488 ≅ 14 miliseconds
#define LineCheckTimeout 500 // 1/2 second. This is just waiting for a message to end so if 1/2 a seconds passes there is must be a problem somewhere.
#define WaitForLineTimeout (SOFBits + MaxMessageSize + MaxDataSize) // This needs to wait for the message to be sent not just the line level to change.


#define LineUnmonitored 0 // there is no ISR etc. keeping track of the line state
#define LineFree 1        // The ISR or function keeping track of incoming messages has marked the line as free.
#define LineInuse 2       // the line is in use. You may need to call exc(); etc. for this to be up to date.
#define LineMinGap 3      // Make sure there is a gap of at least lineMinGapMs (class var)

// When using a function to check the line each time through the main loop you will likely need this delay. I needed about 100ms for the test in main.c
/// @note this is kind of duplicated in inTimer1.cpp

#define Error_NoError 0   //  0,  Successfully sent and received Ack.
#define Error_LineError 1 //  1,  line error.
#define Error_NoRoomInBuffer                                                                                                                              \
  3                           //  3,  Not enough or no room to store the info needed in the buffer. Message received will be overwritten with next message.
#define Error_AckError 16     //  16, A unit signaled an Ack error, it failed to receive the message. For example CRC failed.
#define Error_LostPriority 17 //  17, Higher priority message being sent, received in buffer.
#define Error_CRCError 33     // CRC received not the same as the 1 from calculating it from the received message+data.

///  18, could be network SOF mismatch on different units,
/// or network down or not reading all incoming messages properly
/// or not checking for if in middle of message for example at program start
/// or if receiving messages and only sending them and not checking for line free.
#define Error_NetworkProblem 18
#define Error_UnhandledDataSize 19      // 19 unhandled data size.
#define Error_UnhandledMessageIdBits 20 // 34 unhandled message ID bits, i.e. not 8 or 16 bits.
#define Error_AnotherUnit_AckError 21   // Another unit signaled a receive error, i.e. it failed it's CRC check.
#define Error_EOFCodeStored 22          // End of frame error code stored in private class var: endOfFrameError
#define Error_LineErrorFrameStart 23
/// @brief Not expected value, i.e. the line was not high or low when it should have been. Maybe bit stuffing or a line error.
/// @details This is used to signal that the line was not in the expected state when it
/// was expected to be high or low. This can happen if the line is shorted,
/// if the line is not being monitored correctly, or if there is a protocol error.
/// @note Could be: Protocol error, line error, maybe (reading interrupted by another ISR) or (interrupts disabled for too long) etc.
#define Error_BitNotExpectedValue 24
#define Error_AlreadyProcessingMessage 25 // 25, Already processing a message, i.e. the ISR is already processing a message.

#define Error_NoMessageStoredToRetrieve 30
#define Error_MessageNotInBuffer 31
#define Error_DataNotInBuffer 32
#define Error_Array_to_small 33
#define Error_Code_logic_Bug 40
