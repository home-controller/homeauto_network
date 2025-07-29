//#include "../lib/hn/hn.h"
#include "hn.h"
#include <Arduino.h>
// #include <avr/interrupt.h>
// #include <avr/io.h>
#include <inTimer1.h>
// --- Setup ---
void setup()
{
    Serial.begin(115200); // Faster baud rate for better debugging output
    while (!Serial)
        ; // Wait for Serial Monitor to open for some boards (e.g., Leonardo, ESP32)
    Serial.println("Starting robust digital pulse decoder...");

    pinMode(INPUT_SIGNAL_PIN, INPUT); // Set the input pin

    // Initial read of the pin lineLevel (assuming it's stable at startup)
    //previous_stable_state = digitalRead(INPUT_SIGNAL_PIN);
    //Serial.print("Initial stable lineLevel: ");
    //Serial.println(previous_stable_state == HIGH ? "HIGH" : "LOW");
    setupTimer1();
}

// --- Main Loop ---
void loop()
{
    // Process decoded bits from the buffer in the main loop
    // This avoids doing heavy Serial.print in the ISR, which can cause issues.
    // while (decodedBitHead != decodedBitTail) {
    //     Serial.print(decodedBitsBuffer[decodedBitTail]);
    //     decodedBitTail = (decodedBitTail + 1) % DECODED_BUFFER_SIZE;
    // }
}
