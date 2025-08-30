/**
 * @file sharedVarsConf.h
 * @brief Configuration and shared data structures for the HomeNet protocol.
 *
 * This header defines protocol-level constants, macros, and the main shared data structure
 * used for communication between the ISR and the main loop in an Arduino-based home automation network.
 * It provides protocol timing, frame structure, error codes, and the `SharedData` struct, which
 * encapsulates all state and data fields required for frame reception, transmission, and error handling.
 *
 * Key Features:
 * - Protocol bit timing and frame field definitions.
 * - Error code macros for various protocol and buffer conditions.
 * - `FrameState` enum for protocol-level state machine.
 * - `SharedData` struct for ISR/main loop shared variables, including frame fields, CRC, ACK, and status flags.
 * - Compatibility macros for both Arduino and non-Arduino builds.
 *
 * @note This file is intended for use in projects implementing the HomeNet protocol on Arduino or similar microcontrollers.
 *       It is designed to be included in both ISR and main application code, ensuring consistent protocol state management.
 */
#ifndef _SharedVarsConf_h
#define _SharedVarsConf_h

#include "circular_buf.h"
#ifndef noMcu_buildflag
#include <Arduino.h>

#else
#include <cstdint>
typedef uint8_t byte;
typedef uint16_t word;
typedef uint8_t boolean;
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#endif
// #include "../../libraries/circular_buf/src/circular_buf.h"

extern circular_bufC bufISR1;

// --- Configuration ---
#define INPUT_SIGNAL_PIN 2 // Connect your incoming digital signal to Digital Pin 2 (PD2)

// For my pump controller board the network is on pins 6 for output and 7 for input
/*
#define homeNetPin 6 /// @brief pin for network, a transistor to switch the network to Ground(LOW)
/// @brief pin for network, Connected to the line through a high value resistor mainly to prevent back feeding
/// the MCU when it is off and the line is on.
#define homeNetPinInput 7
 */

///  @brief Allow the line to be pulled low by a unit that is busy, i.e. not ready to receive a message. This is useful if you have only one controller and
///  want to wait for it to be ready before sending a message.
#define AllowPullLineLowForBusy false

#define dominantLineLevel LOW   // The line is pulled LOW by the sending unit, i.e. the dominant level on the line when sending a message.
#define recessiveLineLevel HIGH // The line is pulled HIGH by the receiving unit, i.e. the recessive level on the line when not sending a message.

// #define CRCError
#define SOFBits 2       /// @brief The number of SOF (Start of Frame) bits.
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
#define _pinReg PIND         // read PIND for pins D0 to D7 states
#define _pinMask 0b00000100; // Mask for third pin in reg. i.e. on PIND mask for D2
#define _hn_int_pin 2
#define DataLengthBitsLn 3                          // the number of bits storing the message and data frame length in code.
const byte maxMessageIdBits = (8 * MaxMessageSize); // Maximum message ID bits, currently only

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
/// When using a function to check the line each time through the main loop you will likely need this delay. I needed about 100ms for the test in main.c
#define WaitForLineTimeout (SOFBits + MaxMessageSize + MaxDataSize) // This needs to wait for the message to be sent not just the line level to change.

#define DigitalWriteTime 4   // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5    // forums says 4.78µs but I think than includes the for loop
#define ReadBitsLoopMicros 3 // this is for each time though the loop that checks 8 time per bit so a value of 3 would be 24µs per bit

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

/// @brief Protocol-level state machine for frame processing.
/// @details This enum represents the logical states of a frame as it is received or transmitted according to the protocol,
/// distinct from the physical line states (such as High, Low, etc.).
enum class FrameState : uint8_t {
  Line_State_UNKNOWN, // there is no ISR etc. keeping track of the line state
  Line_IDLE,
  RECEIVING_SOF,
  RECEIVING_RTR,         // Remote Transmission Request bit
  RECEIVING_DATA_LENGTH, // Data Length bits
  RECEIVING_MESSAGE_ID,  // Message ID bits
  RECEIVING_DATA,
  RECEIVING_CRC,
  RECEIVING_CRC_Delimiter,  //
  RECEIVING_ACK_CRC_PASSED, // Where you write your ACK
  RECEIVING_ACK_CRC_FAILED, // The line is pulled low by the unit that will handle the message.
                            // RECEIVING_ACK_DELIMITER,
  RECEIVING_ACK_HANDLED,    // The line is pulled low by the unit that will handle the message.
  // RECEIVING_ACK_HANDLED_DELIMITER, // The line is pulled low by the unit that will handle the message.
  RECEIVING_EOF,
  RECEIVING_INTERFRAME_SPACE, // The line is left high for 3 bits before the next message.
  MESSAGE_COMPLETE,
  Line_DISABLED, // The line is disabled, i.e. it is pulled low. This can mean a line error(shorted to GND etc.) or a unit has pulled it low to signal it
                 // is not ready for a message(if only 1 controller this can be away to tell the sending units to wait).
  Line_Error,    // The line is in an error state, e.g. it is shorted to GND or VCC.
  ERROR_FRAME_DETECTED, // If you implement error detection
  ERROR_Logic_code_Bug  // If you detect a logic error or bug in the code, i.e. the code is not working as expected.
};

/// @brief Shared data structure for the HomeNet protocol.
/// @details This structure contains variables that are shared between the ISR and the main loop.
/// It is used to manage the state of the protocol, including frame reception, transmission, and error handling.
/// The variables are declared as volatile to ensure that they are correctly handled in the ISR context
/// and to prevent optimization issues that could lead to incorrect behavior.
/// For some of the variables, rather that needing to be shared they just need separate copies for if you need
/// more than one network on different pins
struct SharedData { // @brief Shared data structure for the HomeNet protocol. As well sharing need a separate copy for each network on different pins.
  // === Variables for ISR to manage frame reception ===
  volatile FrameState currentFrameState;
  volatile byte bitsRead = 0;                   // Number of bits read in the current frame(only counts 1 bit from SOF)
  volatile uint8_t currentBitIndex;             // Global bit index within the current field
  volatile byte isr_processing_message = false; // Flag to prevent re-entrancy
  // volatile uint8_t currentByteIndex;         // Byte index within data field or buffer
  // volatile uint8_t bitsInCurrentAccumulator; // To accumulate bits into a byte
  //  --- Volatile Variables (accessed by ISR and main loop) ---
  volatile byte previous_stable_state;                       // The last lineLevel we confirmed after debouncing
  volatile unsigned int current_stable_sample_count = 0;     // Counts consecutive stable samples for the current lineLevel
  volatile unsigned int unstable_change_detection_count = 0; // Counts samples that differ from previous_stable_state
  volatile byte current_pin_read;                            // The current pin read value, used to check if the line is stable or not

  /// @brief Number of bits that should be read in the IDR before calling the helper function. 0
  /// @details This will get the ISR to read this and only this number of bits before calling the helper function.
  /// This is useful to avoid reading too many bits at once, which could cause issues with the protocol.
  /// Used in the ack and EOF fields, Fixed length fields without having bit stuffing.
  /// @note 0 means no limit, i.e. read all bits of the same level. (should be no more than 5 in frames with bit stuffing.)
  /// @warning Bits read this way will not allow for timing errors between unit in the same way.
  volatile byte maxBitRead = 0;

  // Frame Fields (simplified for a custom protocol)
  volatile byte RTRBit;            // Remote Transmission Request bit for the current frame
  volatile uint8_t dataLengthCode; // Example: 3-bit DLC (0-7)
  volatile uint8_t dataLength;     // Actual data length in bytes (0-8)
  volatile uint8_t messageIdBits;  // Number of bits in the message ID
#if MaxMessageSize == 1
  volatile uint8_t messageID; // Message ID (up to 8 bits)
#elif MaxMessageSize == 2
  volatile uint16_t messageID; // Message ID (up to 16 bits)
#endif
  /// MaxDataSize(defaults to 4) bytes for data.
  /// This is just for the data, to store in a buffer will also need space for the message ID and the Control Field
  volatile uint8_t dataPayload[MaxDataSize];

  // CRC Field
  volatile uint8_t receivedCRC;   // The CRC value received from the sender (8-bit)
  volatile uint8_t calculatedCRC; // The CRC calculated by this receiver (8-bit)
  volatile bool crcCheckPassed;   // Flag for CRC status

  // ACK Fields
  volatile bool ackPassed; // Flag for all units receiving (with Ack handling implemented) passed the CRC check.
  // volatile bool ackError;    // Flag to indicate if an ACK error was received
  // This record is used for receiving only so no sending vars needed.
  volatile bool canHandleMessage;
  volatile byte canHandleMessageMsk; // A mask for the messageID. if mask > 0 then it is bitand with the message id before the check.
  volatile byte canHandleMessageId;  // if using a > 8 bit message ID this will only mach with ID that will fit into a byte.
  // On MCUs with more memory this could be changed to an array.
  /// @todo add maybe a #define for an array size option here with 16 bit IDs option to.

  // Flags for main loop communication
  volatile bool newFrameReceived;
  volatile byte frameError;             // General error flag (e.g., CRC mismatch, form error, timeout)
  volatile unsigned long lastBitTimeUs; // Timestamp for bit timing checks (micros() recommended)

  // Var to handle bit stuffing
  // volatile byte bitsOfSameLevel; // Count of consecutive bits at the same level (for bit stuffing)

  /// The level of the bits of the same level (HIGH or LOW) + 1 or 0 if not expected.
  /// @note Bits are stuffed in fields that have bit stuffing: 1 bit from SOF(last bit), RTR, Data Length, Message ID, Data Payload.
  /// @warning Every field after the data field does not have bit stuffing i.e.: CRC, ACK, EOF, Interframe Space.
  /// @warning Also only the last bit of SOF has bit stuffing.
  volatile byte stuffedBitExpected;

  // --- Other shared variables ---
  // float temperatureCelsius;
  // int systemStatus;

  /// @note Maybe change the bool(s) to use individual bits of a byte.
};

#endif