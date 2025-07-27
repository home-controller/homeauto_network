#include "inTimer1.h"
// #include <Arduino.h>
// #include <avr/interrupt.h>
// #include <avr/io.h>

// --- Configuration ---

/**
 * @brief Global instance of SharedData used for network-related shared state.
 *
 * This object provides a shared data structure for managing and exchanging
 * information between different components of the network module.
 *
 * @note Ensure thread-safety or interrupt-safety if accessed from multiple contexts.
 */
SharedData g_network1_data;

// SharedData g_network2_data;

// Queue for decoded bits (a simple array for demonstration)
/// @testing: move this elsewhere when finished testing, maybe to a class or struct
#define DECODED_BUFFER_SIZE 100
volatile char decodedBitsBuffer[DECODED_BUFFER_SIZE];
volatile int decodedBitHead = 0; // Where to add new bits
volatile int decodedBitTail = 0; // Where to read bits from

void setupTimer1()
{
    cli(); // Disable global interrupts while configuring timer

    // Clear Timer1 control registers and counter
    TCCR1A = 0; // Set entire TCCR1A register to 0
    TCCR1B = 0; // Set entire TCCR1B register to 0
    TCNT1 = 0;  // Initialize counter value to 0

    // Set Compare Match Register A (OCR1A) for the desired sample interval
    OCR1A = OCR1A_VALUE;

    // Set CTC mode (Clear Timer on Compare Match)
    // WGM12 bit in TCCR1B
    TCCR1B |= (1 << WGM12);

    // Set Timer1 prescaler to 64 (CS11 and CS10 bits)
    TCCR1B |= (1 << CS11) | (1 << CS10); // Or (0b11 << CS10); as discussed

    // Enable Timer1 Compare Match A interrupt
    // OCIE1A bit in TIMSK1
    TIMSK1 |= (1 << OCIE1A);

    resetMessageState();                                                // Initialize shared data structure for the network
    g_network1_data.currentFrameState = FrameState::Line_State_UNKNOWN; // At start the line is in an unknown state

    sei(); // Re-enable global interrupts
}

// --- Interrupt Service Routine for Timer1 Compare Match A ---
ISR(TIMER1_COMPA_vect)
{
    ///@todo keep track of line state if it is in use or not and waiting for new frame(started reading part way through a frame, receive error etc.)
    /// Started to do this but not for any Error handling yet.
    ///@todo The global vars will need to be moved to the struc SharedData or similar if we want to use more than one network on different pins.
    g_network1_data.current_pin_read = digitalRead(INPUT_SIGNAL_PIN);
    /// @todo Maybe change this to only check if the line HIGH and idle or not, and if it is LOW then call the processDecodedDuration function.
    if (g_network1_data.current_pin_read == g_network1_data.previous_stable_state) {
        // Pin lineLevel is the same as the last *confirmed* stable lineLevel
        if (g_network1_data.current_stable_sample_count > MaxInUseHighBits * 8)
            return; // If the stable count exceeds the threshold, line is Idle, disabled or in error state and this is already handled below.
        g_network1_data.unstable_change_detection_count = 0; // Reset any pending "unstable" change detection
        if (g_network1_data.current_stable_sample_count >= MaxInUseHighBits * 8) {
            if (HIGH == g_network1_data.current_pin_read) {
                g_network1_data.currentFrameState = FrameState::Line_IDLE; // Reset to IDLE if line is stable
            } else {
                if (AllowPullLineLowForBusy) {
                    g_network1_data.currentFrameState = FrameState::Line_DISABLED; // Set to DISABLED if line is pulled low by a unit that is busy
                } else {
                    g_network1_data.currentFrameState = FrameState::Line_Error; // Set to ERROR if line is pulled low and not allowed
                }
            }

        } else {
            g_network1_data.current_stable_sample_count++; // Continue counting the current stable duration
        }
    } else {
        // Pin lineLevel is different from the last *confirmed* stable lineLevel
        g_network1_data.unstable_change_detection_count++;
        ///@note if we ever want to use this to show the line state we will need to add an array or something to keep track of all line level changes
        if (g_network1_data.unstable_change_detection_count >= SamplesToCountAsBit) { // changed from DEBOUNCE_THRESHOLD as need to know total bits of same
                                                                                      // level for bit stuffing in processDecodedDuration().
            // We have enough consecutive samples agreeing on the new lineLevel
            // This is a confirmed lineLevel change!

            // Process the *previous* stable duration
            if (g_network1_data.current_stable_sample_count > 0) { // Only process if there was a duration
                if (g_network1_data.isr_processing_message > 0) {
                    // If we are already processing a message, skip this call to avoid re-entrancy issues
                    if ((g_network1_data.currentFrameState == FrameState::RECEIVING_EOF) or
                        (g_network1_data.currentFrameState == FrameState::RECEIVING_INTERFRAME_SPACE)) {
                        g_network1_data.isr_processing_message++; // Increment the processing message count
                        processDecodedDuration(g_network1_data.current_stable_sample_count, g_network1_data.previous_stable_state, false);

                    } else {
                        g_network1_data.frameError = Error_AlreadyProcessingMessage; // Set error flag for already processing message
                        g_network1_data.currentFrameState = FrameState::Line_Error;  // Reset to unknown
                        return;
                    }

                } else {
                    g_network1_data.isr_processing_message = true; // Set flag to prevent re-entrancy
                    // change to done in function interrupts();  // Enable interrupts before processing to allow other ISRs to run
                    byte lev = g_network1_data.previous_stable_state;
                    byte c = g_network1_data.current_stable_sample_count;
                    g_network1_data.previous_stable_state = g_network1_data.current_pin_read;
                    g_network1_data.current_stable_sample_count = 0;     // Reset stable counter for the new lineLevel
                    g_network1_data.unstable_change_detection_count = 0; // Reset unstable counter
                    processDecodedDuration(c, lev, true);
                    noInterrupts();
                    g_network1_data.isr_processing_message = false; // Reset flag after processing
                    return;                                         // Return early after processing the duration
                }
            }

            // Update to the new stable lineLevel
            g_network1_data.previous_stable_state = g_network1_data.current_pin_read;
            g_network1_data.current_stable_sample_count = 0;     // Reset stable counter for the new lineLevel
            g_network1_data.unstable_change_detection_count = 0; // Reset unstable counter
        }
        // Else: not enough consecutive samples yet, keep counting unstable_change_detection_count
    }
}

/**
 * @brief Resets the state variables used for message processing.
 *
 * This function initializes all relevant state variables to their default values,
 * preparing the system to process a new message. It resets counters, message identifiers,
 * CRC values, and other related fields to ensure a clean state.
 *
 * @note Does not reset the currentFrameState, which should be handled separately
 */
static void resetMessageState()
{
    g_network1_data.bitsRead = 0;
    // g_network1_data.currentByteIndex = 0;
    g_network1_data.currentBitIndex = 0;
    // g_network1_data.bitsInCurrentAccumulator = 0;
    //  g_network1_data.messageAndDataLength = 0;
    g_network1_data.calculatedCRC = 0;
    g_network1_data.RTRBit = 0;
    g_network1_data.messageID = 0;
    g_network1_data.dataLength = 0;
    g_network1_data.stuffedBitExpected = false; // Reset stuffed bit expected state
    // g_network1_data.messageIdBits = g_network1_data.; // Reset to default
    // g_network1_data.currentFrameState = FrameState::Line_State_UNKNOWN; // Reset frame state to IDLE
}
// --- Helper function to process the duration of a decoded segment ---

/**
 * @brief Processes a decoded signal duration and updates the frame parsing state.
 *
 * @attention Only call when there are enough samples to process a bit, i.e. SamplesToCountAsBit or more.
 *
 * @details This function interprets a duration (in samples) at a given logic level (HIGH or LOW)
 * as a sequence of protocol bits, handling the parsing of Start of Frame (SOF), RTR (Remote Transmission Request),
 * frame data length, message ID, and data payload bits according to the SlowHomeNet protocol.
 * It also manages bit-stuffing, CRC calculation, and enqueues decoded bits for further processing.
 *
 * The function is designed to be called from a timer interrupt or similar context where
 * signal transitions are measured and decoded in real-time.
 *
 * @param num_samples The number of samples representing the duration of the current logic level.
 * @param lineLevel The logic level (HIGH or LOW) of the signal during the duration.
 *
 * @note 1. This function assumes global or static state for frame parsing (e.g., bitsRead, frameDataLength, messageId, etc.).
 * @note 2. Handles SOF detection, RTR bit, frame info bits, message ID, and data payload bits.
 * @note 3. Performs CRC calculation for message ID bits.
 * @note 4. Enqueues decoded bits for further processing.
 * @note 5. The calling ISR handles the Unknown line state and transitions to IDLE or ERROR states at the minute. Maybe should be moved to here though?
 * @todo Implement bit-stuffing handling.
 */
void processDecodedDuration(unsigned int num_samples, bool lineLevel, bool interrupts)
{
    byte x;
    if (interrupts) interrupts(); // Enable interrupts so other ISR can run if needed, e.g. for Serial output or other ISRs.
    // We also allow this function to be called again before returning if we are in the EOF or Interframe Space state.
    // this allows us to do some longer processing before returning after setting the State to RECEIVING_EOF or RECEIVING_INTERFRAME_SPACE.

    // Calculate the approximate number of 'bits' this duration represents
    // Using roundf for floating-point rounding
    int num_bits = num_samples >> 3;  // Integer division by 8
    if ((num_samples & 0b111) >= 4) { // Check if remainder is 4 or more (i.e., >= 0.5)
        num_bits++;
    }
    if (g_network1_data.stuffedBitExpected > 0) {
        /// @note: ISR should only call this function if there are bits to process, so removed checking for num_bits < 1 bits.
        num_bits--;
        if (lineLevel != (g_network1_data.stuffedBitExpected--)) {
            // If the line level is not what we expected, we have a stuffed bit error
            g_network1_data.frameError = Error_BitNotExpectedValue;     // Set frame error flag
            g_network1_data.currentFrameState = FrameState::Line_Error; // Reset to unknown state
            return; // Exit early as we can't process further. For debugging might be handy to try and decode the rest of the code.
        }
        g_network1_data.stuffedBitExpected = false;
        if (num_bits < 1) return; // If we have no more bits to process, return early
    }
    if (num_bits < 1) {
        /// @note should probably get rid of this check as it is not needed, as the ISR should only call this function if there are bits to process.
        // If we have less than 1 bit, we can't process it
        /// @todo This is an error as bit stuffing relies on this function being called with the correct number of bits.
        g_network1_data.frameError = true;                          // Set frame error flag
        g_network1_data.currentFrameState = FrameState::Line_Error; // Reset to unknown state
        return;                                                     // if the pulse is too short, just ignore it and return.
    }
    // handle bit stuffing.
    // Will need to be reset for fields that do not have bit stuffing, e.g.
    // SOF(only one/last bit counts), CRC, ACK, EOF, Interframe Space
    /// @note set g_network1_data.stuffedBitExpected to the line level + 1. = 0 if not expected, 1 if expected to be LOW, 2 if expected to be HIGH.
    if (num_bits >= 5) {
        g_network1_data.stuffedBitExpected = lineLevel + 1; // If we have 5 or more bits, we expect a stuffed bit next time this function is called
    } // If we have 5 or more bits, we expect a stuffed bit next time this function is called (if in bit stuffed field)
    else {
        g_network1_data.stuffedBitExpected = false;
    } // If less than 5 bits, no stuffed bit expected

    // SOF (Start of Frame) detection SOFBits
    switch (g_network1_data.currentFrameState) {
            //        case FrameState::Line_State_UNKNOWN: handled in calling function ISR
        case FrameState::Line_IDLE:
            // To be in the IDLE state, we should have a stable lineLevel of HIGH for at least MaxInUseHighBits * 8 samples(9 bits)
            /// And as this is only called when the lineLevel is LOW, we can move to receiving SOF directly
            resetMessageState(); // Reset the message state for a new frame
            g_network1_data.currentFrameState = FrameState::RECEIVING_SOF;
        case FrameState::RECEIVING_SOF:

            if (SOFBits == 1) {
                // If SOF is a single bit, we can directly check the line level
                if (lineLevel != LOW) { // Only checking for errors here.
                    // If we read a '1' when expecting '0', reset and wait for new frame
                    /// @note We should never get here unless there is a bug in the logic as to be in RECEIVING_SOF we should have read a LOW bit
                    /// Should we even have this "if (lineLevel == LOW) check"?
                    /// Maybe if the ISR gets blocked by some other ISR and/or interrupts get disabled for too long? Or the timer is disabled and
                    /// reenabled without resetting the state? Still could just rely on the CRC check but guess that might send the CRC Ack in the wrong
                    /// place and mess up the message for other units?

                    // The change from Line_State_UNKNOWN to Line_IDLE will call resetMessageState(); so not needed here.
                    g_network1_data.currentFrameState = FrameState::Line_State_UNKNOWN;
                    return; // Nothing stored yet so can return and wait for next message
                }
            } else { // If SOF is multiple bits, drop the leading LOW bits until we find a HIGH bit
                if (lineLevel == LOW) {
                    // Discard any number of leading LOW bits until we find a HIGH bit. Meaning if SOF length is > 1 it can be any number of leading LOW
                    // bits. g_network1_data.bitsOfSameLevel = 0; // read 1 bit of SOF g_network1_data.sameLevelLevel = LOW; // Set the level of the bits
                    // of the same level
                    g_network1_data.stuffedBitExpected = false; // no stuffed bits in leading LOWs in SOF
                    return;                                     // Return early as we are still waiting for the first HIGH bit of SOF
                }
            }
            g_network1_data.bitsRead = 1; // We have read the last bit of SOF
            g_network1_data.currentFrameState = FrameState::RECEIVING_RTR;
            num_bits--;
            if (num_bits < 1) return; // If we have no more bits to process, return early
        case FrameState::RECEIVING_RTR:
            g_network1_data.bitsRead = 2; // We have read the last bit of SOF
            g_network1_data.currentFrameState = FrameState::RECEIVING_DATA_LENGTH;
            g_network1_data.currentBitIndex = 0; // Reset current bit index for the next field
            num_bits--;
            if (num_bits < 1) return; // If we have no more bits to process, return early

        case FrameState::RECEIVING_DATA_LENGTH: // Data Length bits are 3 bits.
            if (g_network1_data.currentBitIndex < 3) {
                x = 3 - g_network1_data.currentBitIndex; // Calculate how many bits we need to read for the data length
                if (x > num_bits) {
                    x = num_bits; // If we don't have enough bits, read only what we can
                }
                g_network1_data.dataLengthCode = (g_network1_data.dataLength << x);
                if (lineLevel) {
                    g_network1_data.dataLengthCode |= ((1 << x) - 1); // Set the last x bits to 1 if lineLevel is HIGH
                }
                g_network1_data.bitsRead += x;        // Update bits read
                g_network1_data.currentBitIndex += x; // Update current bit index
                num_bits -= x;                        // Decrease the number of bits left to process
            }
            if (g_network1_data.currentBitIndex >= 3) {
                // We have read all 3 bits of the data length
                g_network1_data.dataLength = SlowHomeNet::getDataLen(g_network1_data.dataLengthCode); // Convert length code to number of bits
                if (g_network1_data.dataLength > MaxDataSize) {
                    // If data length is more than MaxDataSize(defaults to 4), set error state
                    g_network1_data.frameError = Error_UnhandledDataSize; // Set frame error flag
                    g_network1_data.currentFrameState = FrameState::Line_Error;
                    return;
                }
                g_network1_data.messageIdBits =
                  SlowHomeNet::getMessageLen(g_network1_data.dataLengthCode) * 8; // Get the number of bits for the message ID
                g_network1_data.currentFrameState = FrameState::RECEIVING_MESSAGE_ID;
                g_network1_data.currentBitIndex = 0; // Reset current bit index for the next field
                g_network1_data.messageID = 0;       // Reset message ID for the next field
            }
            if (num_bits < 1) return; // If we have no more bits to process, return early

        case FrameState::RECEIVING_MESSAGE_ID:
            // Read the message ID bits
            if (g_network1_data.currentBitIndex < g_network1_data.messageIdBits) {
                x = g_network1_data.messageIdBits - g_network1_data.currentBitIndex; // Calculate how many bits we need to read for the message ID
                if (x > num_bits) {
                    x = num_bits; // If we don't have enough bits, read only what we can
                }
                g_network1_data.messageID = (g_network1_data.messageID << x);
                if (lineLevel) {
                    g_network1_data.messageID |= ((1 << x) - 1); // Set the last x bits to 1 if lineLevel is HIGH
                }
                g_network1_data.bitsRead += x;        // Update bits read
                g_network1_data.currentBitIndex += x; // Update current bit index
                num_bits -= x;                        // Decrease the number of bits left to process
            }
            if (g_network1_data.currentBitIndex >= g_network1_data.messageIdBits) {
                // We have read all bits of the message ID
                if (g_network1_data.messageIdBits == 8) {
                    g_network1_data.calculatedCRC = SlowHomeNet::CRC8bits((byte)(g_network1_data.messageID), 0);
                } else if (g_network1_data.messageIdBits == 16) {
                    g_network1_data.calculatedCRC = SlowHomeNet::CRC8bits(highByte(g_network1_data.messageID), 0);
                    g_network1_data.calculatedCRC |= SlowHomeNet::CRC8bits(lowByte(g_network1_data.messageID), g_network1_data.calculatedCRC);
                } else {
                    // Unsupported messageIdBits length, set error state
                    g_network1_data.frameError = Error_UnhandledMessageIdBits;
                    g_network1_data.currentFrameState = FrameState::Line_Error;
                    return;
                }
                g_network1_data.currentFrameState = FrameState::RECEIVING_DATA;
                g_network1_data.currentBitIndex = 0; // Reset current bit index for the next field
            }
            if (num_bits < 1) return; // If we have no more bits to process, return early

        // Handle the data payload bits
        // Note: Data length is already set, so we know how many bytes to read
        // If data length is 0, we skip to CRC directly
        case FrameState::RECEIVING_DATA:
            if (g_network1_data.dataLength > 0) {
                // We have data to read
                x = g_network1_data.dataLength * 8 - g_network1_data.currentBitIndex; // Calculate how many bits we need to read
                if (x > num_bits) {
                    x = num_bits; // If we don't have enough bits, read only what we can
                }
                for (int i = 0; i < x; i++) {
                    g_network1_data.dataPayload[g_network1_data.currentByteIndex] <<= 1; // Shift left to make space for the new bit
                    if (lineLevel) {
                        g_network1_data.dataPayload[g_network1_data.currentByteIndex] |= 1; // Set the last bit to 1 if lineLevel is HIGH
                    }
                    g_network1_data.bitsRead++;        // Update bits read
                    g_network1_data.currentBitIndex++; // Update current bit index
                    if (++g_network1_data.currentByteIndex >= g_network1_data.dataLength) {
                        break; // Stop if we have read all bytes of data
                    }
                }
                num_bits -= x; // Decrease the number of bits left to process
            }
            if (g_network1_data.currentBitIndex >> 3 >= g_network1_data.dataLength) {
                // We have read all data bytes, move to CRC state
                g_network1_data.receivedCRC = 0; // Reset received CRC for the next field
                g_network1_data.currentFrameState = FrameState::RECEIVING_CRC;
                g_network1_data.currentBitIndex = 0; // Reset current bit index for the next field
            }
            if (num_bits < 1) return; // If we have no more bits to process, return early
        case FrameState::RECEIVING_CRC:
            // Read the CRC bits
            if (g_network1_data.currentBitIndex < 8) {
                x = 8 - g_network1_data.currentBitIndex; // Calculate how many bits we need to read for the CRC
                if (x > num_bits) {
                    x = num_bits; // If we don't have enough bits, read only what we can
                }
                g_network1_data.receivedCRC <<= x; // Shift left to make space for the new bits
                if (lineLevel) {
                    g_network1_data.receivedCRC |= ((1 << x) - 1); // Set the last x bits to 1 if lineLevel is HIGH
                }
                g_network1_data.bitsRead += x;        // Update bits read
                g_network1_data.currentBitIndex += x; // Update current bit index
                num_bits -= x;                        // Decrease the number of bits left to process
            }
            if (g_network1_data.currentBitIndex >= 8) {
                // We have read all bits of the CRC, now we can check it
                if (g_network1_data.calculatedCRC != g_network1_data.receivedCRC) {
                    // CRC mismatch, set error state
                    g_network1_data.frameError = Error_CRCError; // Set frame error flag
                    g_network1_data.currentFrameState = FrameState::RECEIVING_ACK_CRC_FAILED;
                } else {
                    // We have read all bits of the CRC, move to ACK state
                    g_network1_data.currentFrameState = FrameState::RECEIVING_ACK_CRC_PASSED;
                }
                g_network1_data.currentBitIndex = 0; // Reset current bit index for the next field
            }
            if (num_bits < 1) return; // If we have no more bits to process, return early

        case FrameState::RECEIVING_ACK_CRC_PASSED:
        ///@todo As this needs a reply we need away for the ISR to always call at this point.
            // We have passed the CRC check, now we can handle the ACK
    }

    // --- Simple Circular Buffer for Decoded Bits ---
    void enqueueDecodedBit(char bitValue)
    {
        int nextHead = (decodedBitHead + 1) % DECODED_BUFFER_SIZE;
        if (nextHead != decodedBitTail) { // Check if buffer is not full
            decodedBitsBuffer[decodedBitHead] = bitValue;
            decodedBitHead = nextHead;
        } else {
            // Serial.println("Buffer overflow!"); // Debugging for buffer full
        }
    }