#include "hn.h"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// --- Configuration ---
#define INPUT_SIGNAL_PIN 2 // Connect your incoming digital signal to Digital Pin 2 (PD2)

// Timer configuration
const unsigned long SAMPLE_INTERVAL_US = 256; // Target sampling interval in microseconds
const int TIMER1_PRESCALER = 64;              // Prescaler for Timer1
const unsigned int OCR1A_VALUE = ((F_CPU / TIMER1_PRESCALER) * (SAMPLE_INTERVAL_US / 1000000.0)) - 1;

// Decoding parameters
const int SAMPLES_PER_BIT = 8;    // Number of samples that represent one 'bit' duration (2048us / 256us)
const int DEBOUNCE_THRESHOLD = 2; // Number of consecutive samples to confirm a lineLevel change

// --- Volatile Variables (accessed by ISR and main loop) ---
volatile byte previous_stable_state;                       // The last lineLevel we confirmed after debouncing
volatile unsigned int current_stable_sample_count = 0;     // Counts consecutive stable samples for the current lineLevel
volatile unsigned int unstable_change_detection_count = 0; // Counts samples that differ from previous_stable_state
volatile byte bitsRead = 0;                                // Number of bits read in the current frame
volatile byte lineState = LineUnmonitored;                 // Current line state (0 = idle, 1 = active, 2 = error, etc.) See "#defines LineFree" in hn.h
///@note lineState is also a class var in SlowHomeNet, so if this is converted to a class we will only need one of these.
volatile byte messageAndDataLength = 0;                    // Length of the message and data frame in bits
volatile byte crcRunningValue = 0;                         // Running CRC value for the current frame
volatile byte RTRBit;                                      // Remote Transmission Request bit for the current frame
volatile byte messageId = 0;                               // Initialize message ID
volatile byte frameDataLength = 0;                         // Length of the data in the current frame
volatile byte messageIdBits = 8;                           // Number of bits in the message ID
const byte maxDataLength = 4;                              // Maximum data size in bytes (can be adjusted as needed)
volatile byte dataArray[maxDataLength + 1];                // Array to hold message and data, +1 for command byte

// Queue for decoded bits (a simple array for demonstration)
#define DECODED_BUFFER_SIZE 100
volatile char decodedBitsBuffer[DECODED_BUFFER_SIZE];
volatile int decodedBitHead = 0; // Where to add new bits
volatile int decodedBitTail = 0; // Where to read bits from

// --- Function Prototypes ---
void processDecodedDuration(unsigned int num_samples, byte lineLevel);
void enqueueDecodedBit(char bitValue);

// --- Setup ---
void setup()
{
    Serial.begin(115200); // Faster baud rate for better debugging output
    while (!Serial)
        ; // Wait for Serial Monitor to open for some boards (e.g., Leonardo, ESP32)
    Serial.println("Starting robust digital pulse decoder...");

    pinMode(INPUT_SIGNAL_PIN, INPUT); // Set the input pin

    // Initial read of the pin lineLevel (assuming it's stable at startup)
    previous_stable_state = digitalRead(INPUT_SIGNAL_PIN);
    Serial.print("Initial stable lineLevel: ");
    Serial.println(previous_stable_state == HIGH ? "HIGH" : "LOW");

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

    sei(); // Re-enable global interrupts
}

// --- Main Loop ---
void loop()
{
    // Process decoded bits from the buffer in the main loop
    // This avoids doing heavy Serial.print in the ISR, which can cause issues.
    while (decodedBitHead != decodedBitTail) {
        Serial.print(decodedBitsBuffer[decodedBitTail]);
        decodedBitTail = (decodedBitTail + 1) % DECODED_BUFFER_SIZE;
    }
}

// --- Interrupt Service Routine for Timer1 Compare Match A ---
ISR(TIMER1_COMPA_vect)
{
    ///@todo keep track of line state if it is in use or not and waiting for new frame(started reading part way through a frame, receive error etc.)
    byte current_pin_read = digitalRead(INPUT_SIGNAL_PIN);

    if (current_pin_read != previous_stable_state) {
        // Pin lineLevel is different from the last *confirmed* stable lineLevel
        unstable_change_detection_count++;

        if (unstable_change_detection_count >= DEBOUNCE_THRESHOLD) {
            // We have enough consecutive samples agreeing on the new lineLevel
            // This is a confirmed lineLevel change!

            // Process the *previous* stable duration
            if (current_stable_sample_count > 0) { // Only process if there was a duration
                processDecodedDuration(current_stable_sample_count, previous_stable_state);
            }

            // Update to the new stable lineLevel
            previous_stable_state = current_pin_read;
            current_stable_sample_count = 0;     // Reset stable counter for the new lineLevel
            unstable_change_detection_count = 0; // Reset unstable counter
        }
        // Else: not enough consecutive samples yet, keep counting unstable_change_detection_count
    } else {
        // Pin lineLevel is the same as the last *confirmed* stable lineLevel
        unstable_change_detection_count = 0; // Reset any pending "unstable" change detection
        current_stable_sample_count++;       // Continue counting the current stable duration
    }
}

/**
 * @brief Resets the state variables used for message processing.
 *
 * This function initializes all relevant state variables to their default values,
 * preparing the system to process a new message. It resets counters, message identifiers,
 * CRC values, and other related fields to ensure a clean state.
 */
static void resetMessageState()
{
    bitsRead = 0;
    messageAndDataLength = 0;
    crcRunningValue = 0;
    RTRBit = 0;
    messageId = 0;
    frameDataLength = 0;
    messageIdBits = 8; // Reset to default
}
// --- Helper function to process the duration of a decoded segment ---
void processDecodedDuration(unsigned int num_samples, byte lineLevel)
{
    static byte bitsOfSameLevel;
    static bool sameLevelLevel;
    static byte runningCRC;
    // Calculate the approximate number of 'bits' this duration represents
    // Using roundf for floating-point rounding
    int num_bits = num_samples >> 3;  // Integer division by 8
    if ((num_samples & 0b111) >= 4) { // Check if remainder is 4 or more (i.e., >= 0.5)
        num_bits++;
    }
    if (num_bits < 1) {
        // If we have less than 1 bit, we can't process it
        return; // if the pulse is too short, just ignore it and return. maybe we should flag this as an error?
    }
    //@todo: forgot about handling bit stuffing, so we need to handle that here too.

    // SOF (Start of Frame) detection SOFBits
    if (bitsRead < SOFBits) { // SOFBits is the number of bits for Start of Frame
        // Handle Start of Frame detection
        if (SOFBits == 1) {
            if (lineLevel == LOW) { // SOF is 0 for single bit
                bitsRead++;
            } else { // SOF is 1 for single bit
                bitsRead = 0;
                bitsOfSameLevel = 0; // Reset bitsOfSameLevel for the new frame
                resetMessageState(); // Reset and wait for new frame if we read a '1' when expecting '0'
                // nothing stored yet so can return and wait for next message
                return;
            }
        }
        if (bitsRead + num_bits >= SOFBits) { // Last bit of SOF, plus maybe RTR bit etc.
            // If we have read enough bits to complete the SOF
            if (lineLevel == LOW) {
                // Should be HIGH for last bit of SOF in multi-bit SOF
                bitsRead = 0;
                return; // Reset bitsRead if we read a '0' when expecting '1'
            } else {
                bitsRead++;
                num_bits--;
                if (num_bits < 1) return; // If we have no more bits to process, return early)
            }
        } else {
            // Still reading SOF bits
            if (lineLevel == LOW) {
                bitsRead += num_bits; // SOF is 0 for multi-bit
                return;               // Return early as we have used up the received bits
            } else {
                // error condition, we expected a LOW but got HIGH
                bitsRead = 0; // Reset bitsRead if we read a '1' when expecting '0'
                // nothing stored yet so can return and wait for next message
                return;
            }
        }
    }

    // Read RTR (Remote Transmission Request) bit
    if (bitsRead < SOFBits + 1) { // RTR (Remote Transmission Request) bit
        // If we are still reading SOF bits, return early
        RTRBit = lineLevel bitand 0b1; // Set RTR bit based on state
        bitsRead++;
        num_bits--;
        frameDataLength = 0;      // Reset frameDataLength for the new frame
        if (num_bits < 1) return; // If we have no more bits to process, return early
    }
    // If we are here, we have read the SOF and RTR bits

    // Process the data length code bits after SOF and RTR
    while (bitsRead < SOFBits + FrameInfoBits) { // FrameInfoBits is the number of bits for message ID and data length
        // We are still reading Data length bits
        frameDataLength = (frameDataLength << 1) | (lineLevel & 0b1);
        bitsRead++;
        num_bits--;
        if (bitsRead >= SOFBits + FrameInfoBits) {
            // convert length code to number of bits
            // Data length bits are 0=0,1=1,2=2,3=4,4=8,5=16,6=32,7=64
            frameDataLength = SlowHomeNet::getDataLen(frameDataLength);
            messageIdBits = SlowHomeNet::getMessageLen(frameDataLength) * 8; // Get the number of bits for the message ID
        }
        if (num_bits < 1) return; // If we have no more bits to process, return early
    }
    // If we are here, we have read the SOF, RTR, and data length bits

    // Read the message ID bits
    byte TOTAL_TARGET_BITS = SOFBits + FrameInfoBits + messageIdBits; // The total number of bits you're trying to collect
    byte bits_to_take_from_current_block;
    if (bitsRead < TOTAL_TARGET_BITS) {
        // Calculate how many bits from the *current* 'num_bits' run we actually need
        // to fulfill our 'TOTAL_TARGET_BITS' requirement.
        bits_to_take_from_current_block = min((int)num_bits, TOTAL_TARGET_BITS - bitsRead);
        if (bits_to_take_from_current_block > 0) { // Only process if we actually need bits from this block
            // Left-shift messageId to make space for the new block of bits
            messageId <<= bits_to_take_from_current_block;

            if (lineLevel) { // If the current line level is HIGH (i.e., bit is 1)
                // Generate a mask of '1's for the number of bits we're taking.
                // E.g., if bits_to_take_from_current_block = 3, (1 << 3) - 1 = 0b111
                uint16_t ones_mask = (1 << bits_to_take_from_current_block) - 1;
                messageId |= ones_mask; // OR in the block of ones
            }
            // If lineLevel is 0, no ORing is needed after the left shift, as it effectively appends zeros.

            // Update counters
            bitsRead += bits_to_take_from_current_block;
            num_bits -= bits_to_take_from_current_block;
            if (bitsRead >= SOFBits + FrameInfoBits + messageIdBits) {
                // @todo: work out CRC for message ID bits.
                if (messageIdBits == 8) {
                    runningCRC = SlowHomeNet::CRC8bits(lowByte(messageId));
                } else { // Only 8 and 16 bits are currently supported for message ID
                    runningCRC = SlowHomeNet::CRC8bits(lowByte(messageId));
                    runningCRC <<= 8;
                    runningCRC |= SlowHomeNet::CRC8bits(highByte(messageId));
                }
            }
            if (num_bits < 1) return; // Consume from the current run
        }
    }

    // Now read the data bits next if data length is greater than 0
    if (frameDataLength > 0) {
        TOTAL_TARGET_BITS = SOFBits + FrameInfoBits + messageIdBits + frameDataLength * 8; // The total number of bits you're trying to collect
        if (bitsRead < TOTAL_TARGET_BITS) {
            byte bits_to_take_from_current_block = min((int)num_bits, TOTAL_TARGET_BITS - bitsRead);
            if (bits_to_take_from_current_block > 0) {
                // Left-shift messageId to make space for the new block of bits
                messageId <<= bits_to_take_from_current_block;

                if (lineLevel) { // If the current line level is HIGH (i.e., bit is 1)
                    // Generate a mask of '1's for the number of bits we're taking.
                    // E.g., if bits_to_take_from_current_block = 3, (1 << 3) - 1 = 0b111
                    uint16_t ones_mask = (1 << bits_to_take_from_current_block) - 1;
                    messageId |= ones_mask; // OR in the block of ones
                }
                // If lineLevel is 0, no ORing is needed after the left shift, as it effectively appends zeros.

                // Update counters
                bitsRead += bits_to_take_from_current_block;
                num_bits -= bits_to_take_from_current_block;
                if (num_bits < 1) return; // Consume from the current run
            }
        }
    }

    // Debugging output within the ISR (use sparingly, ideally buffer for main loop)
    // Serial.print("Detected ");
    // Serial.print(lineLevel == HIGH ? "HIGH" : "LOW");
    // Serial.print(" for ");
    // Serial.print(num_samples * SAMPLE_INTERVAL_US);
    // Serial.print("us (");
    // Serial.print(num_samples);
    // Serial.print(" samples). Decoded as ");
    // Serial.print(num_bits);
    // Serial.println(" bits.");

    // Enqueue the decoded bits based on the lineLevel and number of bits
    // For a simple HIGH=1, LOW=0 protocol:
    char bitToEnqueue = (lineLevel == HIGH) ? '1' : '0';
    for (int i = 0; i < num_bits; i++) {
        enqueueDecodedBit(bitToEnqueue);
    }
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