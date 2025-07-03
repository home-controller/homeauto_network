#include <avr/io.h>
#include <avr/interrupt.h>

#define BUFFER_SIZE 128
#define MIN_PULSE_WIDTH 200  // Adjust this value as needed for debounce
#define DATA_PIN 0
#define DEBOUNCE_TIME 1000   // Debounce time in microseconds. A pulse/bit is 2048 microseconds

/**
 * @class TimerDecoder
 * @brief A class for decoding digital signals using timer and pin change interrupts with debounce logic.
 *
 * @details
 * The TimerDecoder class is designed to capture and decode digital bit streams from a specified data pin
 * on an Arduino-like microcontroller. It uses Timer1 for precise timing and a pin change interrupt to
 * detect state changes on the data pin. The class implements debounce logic to filter out spurious transitions
 * and stores the timing and state information in a circular event buffer for later processing.
 *
 * Key Features:
 * - Configures Timer1 for timing and sets up pin change interrupts for the data pin.
 * - Implements software debounce to ensure only stable state changes are recorded.
 * - Uses a circular buffer to store time deltas and pin states for decoding.
 * - Provides methods to enable or disable decoding.
 * - Processes buffered events in the timer interrupt handler, where protocol-specific decoding logic can be implemented.
 *
 * Usage:
 * 1. Instantiate the TimerDecoder class.
 * 2. Call begin() to initialize timers and interrupts.
 * 3. Use enableDecoding() and disableDecoding() to control decoding.
 * 4. Implement protocol decoding logic in handleTimerInterrupt().
 *
 * Assumptions:
 * - Constants such as DATA_PIN, DEBOUNCE_TIME, MIN_PULSE_WIDTH, and BUFFER_SIZE are defined elsewhere.
 * - The class is intended for use in an embedded environment with direct register access.
 *
 * Thread Safety:
 * - The class uses volatile qualifiers for variables shared between interrupt and main contexts.
 *
 * @author Joseph
 * @date 1 July 2025
 */
class TimerDecoder {
public:
    TimerDecoder() : decoding_enabled(0), last_timestamp(0), last_stable_state(0), last_debounce_time(0) {
        event_buffer.head = 0;
        event_buffer.tail = 0;
    }

    void begin() {
        // Configure Timer1
        TCCR1B |= (1 << CS11);     // Start Timer1 with prescaler 8 (2 MHz clock)
        OCR1A = 65535;             // Set OCR1A to 65535 for ~30.52 Hz interrupt frequency
        TIMSK1 |= (1 << OCIE1A);   // Enable Timer1 compare match interrupt

        // Configure Pin Change Interrupt for DATA_PIN
        PCICR |= (1 << PCIE0);     // Enable pin change interrupts for PCINT0-7
        PCMSK0 |= (1 << DATA_PIN); // Enable pin change interrupt on DATA_PIN

        sei();                     // Enable global interrupts
    }

    void enableDecoding() {
        decoding_enabled = 1;
    }

    void disableDecoding() {
        decoding_enabled = 0;
    }

    /**
     * @brief Handles the change in the state of a pin with debounce logic.
     *
     * This function is called whenever there is a change in the state of the specified pin.
     * 
     * @details
     * It implements debounce logic to ensure that only stable state changes are recorded.
     * The function calculates the time difference between the current and last state change,
     * and if the state remains stable for a specified debounce time, it records the state
     * change and the time difference in an event buffer.
     *
     * The function uses the following global variables:
     * - `last_timestamp`: The timestamp of the last recorded state change.
     * - `last_stable_state`: The last stable state of the pin.
     * - `last_debounce_time`: The timestamp of the last debounce check.
     * - `event_buffer`: A circular buffer to store the time differences and states.
     *
     * The function reads the current state of the pin, checks if it has changed, and if so,
     * resets the debounce timer. If the state remains stable for the debounce time, it records
     * the state change and the time difference in the event buffer, ensuring that the buffer
     * does not overflow.
     * 
     * @note 1. This function assumes that the global variables and constants such as `TCNT1`,
     * `PINB`, `DATA_PIN`, `DEBOUNCE_TIME`, `MIN_PULSE_WIDTH`, and `BUFFER_SIZE` are defined
     * elsewhere in the code.
     *
     * @note 2. Rough Estimate of Instruction Count, approximate range of 10-20 instructions (this is a Copilot estimate).
     * 
     * 3. Estimated Execution Time: The `handlePinChange()` function is estimated to execute
     * in approximately 0.9375 to 1.875 microseconds on an Arduino Uno running at 16 MHz. This
     * is a rough estimate and the actual execution time may vary based on the specific 
     * implementation and compiler optimizations.
     */
    void handlePinChange() {
        uint16_t current_timestamp = TCNT1;
        uint16_t delta = current_timestamp - last_timestamp; // Calculate time difference

        uint8_t current_state = (PINB & (1 << DATA_PIN)) ? 1 : 0; // Read DATA_PIN state

        if (current_state != last_stable_state) { // State has changed
            last_debounce_time = current_timestamp; // Reset debounce timer
        }

        if ((current_timestamp - last_debounce_time) >= DEBOUNCE_TIME) { // Stable for debounce time
            if (current_state != last_stable_state) { // State has changed
                last_stable_state = current_state; // Update stable state

                if (delta >= MIN_PULSE_WIDTH) { // Debounce: Ignore quick changes
                    uint8_t next_head = (event_buffer.head + 1) % BUFFER_SIZE;
                    if (next_head != event_buffer.tail) { // Check for buffer overflow
                        event_buffer.time_delta[event_buffer.head] = delta;
                        event_buffer.state[event_buffer.head] = current_state; // Record stable state
                        event_buffer.head = next_head;
                    }
                    last_timestamp = current_timestamp;
                }
            }
        }
    }

    void handleTimerInterrupt() {
        if (!decoding_enabled) return; // Skip decoding if disabled

        while (event_buffer.tail != event_buffer.head) {
            uint16_t delta = event_buffer.time_delta[event_buffer.tail]; // Get time delta
            uint8_t state = event_buffer.state[event_buffer.tail];       // Get pin state
            event_buffer.tail = (event_buffer.tail + 1) % BUFFER_SIZE;   // Move tail forward

            // Decode the bit stream here based on delta and state

            // The Start of frame is 2 bits that should be 0b01
            // RTR (Remote Transmission Request) is 1 bit
            // the date length is 3 bits
            // The message ID is 8 bits
            // The data is 0 to 32 bits
            // The CRC is 4 bits
            // The Ack is 2 bits that should be 0b01
            // The message handled bit is 2 bits with the delimiter bit
            // the extra delimiter bit is 1 bit, this is pulled low before the end of frame.
            // The End of frame is 7 bits that should be 0b1111111
            // there should be 3 bits of spacing between frames next.
        }
    }

private:
    struct {
        uint16_t time_delta[BUFFER_SIZE];
        uint8_t state[BUFFER_SIZE];
        uint8_t head;
        uint8_t tail;
    } event_buffer;

    volatile uint8_t decoding_enabled;
    volatile uint16_t last_timestamp;
    volatile uint8_t last_stable_state;
    volatile uint16_t last_debounce_time;
};

TimerDecoder timerDecoder;

ISR(TIMER1_COMPA_vect) {
    timerDecoder.handleTimerInterrupt();
}

ISR(PCINT0_vect) {
    timerDecoder.handlePinChange();
}

int main(void) {
    timerDecoder.begin();

    while (1) {
        // Time-sensitive code runs here
    }
}