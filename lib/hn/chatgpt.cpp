#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define BUFFER_SIZE 32
#define MIN_PULSE_WIDTH 2    // Debounce threshold in timer ticks
#define DATA_PIN PB2         // Default to D2 (PB2 on Arduino Uno)

// Circular buffer to store time deltas and pin states
volatile struct {
    uint16_t time_delta[BUFFER_SIZE];  // Time difference between pin changes
    uint8_t state[BUFFER_SIZE];       // Pin state (high or low)
    uint8_t head;                     // Write index
    uint8_t tail;                     // Read index
} event_buffer = {0};

volatile uint16_t last_timestamp = 0; // Last recorded timer value
volatile uint8_t decoding_enabled = 1; // Flag to enable/disable decoding

// Pin Change Interrupt Service Routine for DATA_PIN
ISR(PCINT0_vect) {
    uint16_t current_timestamp = TCNT1; // Read current timer value
    uint16_t delta = current_timestamp - last_timestamp; // Calculate time difference

    if (delta >= MIN_PULSE_WIDTH) {  // Debounce: Ignore quick changes
        uint8_t next_head = (event_buffer.head + 1) % BUFFER_SIZE;
        if (next_head != event_buffer.tail) {  // Check for buffer overflow
            event_buffer.time_delta[event_buffer.head] = delta;
            event_buffer.state[event_buffer.head] = (PINB & (1 << DATA_PIN)) ? 1 : 0;  // Read DATA_PIN state
            event_buffer.head = next_head;
        }
        last_timestamp = current_timestamp;
    }
}

// Timer1 Compare Match Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
    if (!decoding_enabled) return; // Skip decoding if disabled

    while (event_buffer.tail != event_buffer.head) {
        uint16_t delta = event_buffer.time_delta[event_buffer.tail]; // Get time delta
        uint8_t state = event_buffer.state[event_buffer.tail];       // Get pin state
        event_buffer.tail = (event_buffer.tail + 1) % BUFFER_SIZE;   // Move tail forward

        // Decode the bit stream here based on delta and state
    }
}

// Enable decoding
void enable_decoding(void) {
    decoding_enabled = 1;
}

// Disable decoding
void disable_decoding(void) {
    decoding_enabled = 0;
}

int main(void) {
    // Configure Timer1
    TCCR1B |= (1 << CS11);     // Start Timer1 with prescaler 8 (2 MHz clock)
    OCR1A = 65535;              // Set OCR1A to 65535 for ~30.52 Hz interrupt frequency
    TIMSK1 |= (1 << OCIE1A);   // Enable Timer1 compare match interrupt

    // Configure Pin Change Interrupt for DATA_PIN
    PCICR |= (1 << PCIE0);     // Enable pin change interrupts for PCINT0-7
    PCMSK0 |= (1 << DATA_PIN); // Enable pin change interrupt on DATA_PIN

    sei();                     // Enable global interrupts

    while (1) {
        // Time-sensitive code runs here
    }
}
