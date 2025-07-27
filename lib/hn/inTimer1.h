#include "hn.h"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// --- Configuration ---

// Timer configuration
const unsigned long SAMPLE_INTERVAL_US = 256; // Target sampling interval in microseconds
const int TIMER1_PRESCALER = 64;              // Prescaler for Timer1
const unsigned int OCR1A_VALUE = ((F_CPU / TIMER1_PRESCALER) * (SAMPLE_INTERVAL_US / 1000000.0)) - 1;

// Decoding parameters
const int SAMPLES_PER_BIT = 8;    // Number of samples that represent one 'bit' duration (2048us / 256us)
const int DEBOUNCE_THRESHOLD = 2; // Number of consecutive samples to confirm a lineLevel change
const byte SamplesToCountAsBit = 4; // Minimum number of samples to count as a valid bit (4 samples = 1/2 bit at 2048us)

// --- Function Prototypes ---
void processDecodedDuration(unsigned int num_samples, byte lineLevel);
void enqueueDecodedBit(char bitValue);
void setupTimer1();
