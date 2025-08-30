// #include "../lib/hn/hn.h"
#include "hn.h"
#include <Arduino.h>
// #include <avr/interrupt.h>
// #include <avr/io.h>
#include <inTimer1.h>
// --- Setup ---

#define serial_speed 115200 // 38400

// #define pinIO_no_of_switches 6               // setup the number of gpio's used
// #define pinIO_inPins A7, A6, A0, A1, A2, A3  // in sa main.h
#define pinIO_no_of_switches 4      // setup the number of gpio's used
#define pinIO_inPins A0, A1, A2, A3 // in sa main.h

#define NextBoardId 28
#define eepromIdAddr 250

// byte pinIO_Max_switches = pinIO_no_of_switches;
byte pinIO_switchState[pinIO_no_of_switches];
byte pinIO_pinsA_in[pinIO_no_of_switches] = { pinIO_inPins };

circular_bufC bufISR1;
void setup()
{
    Serial.begin(115200); // Faster baud rate for better debugging output
    while (!Serial)
        ; // Wait for Serial Monitor to open for some boards (e.g., Leonardo, ESP32)
    Serial.println("Starting robust digital pulse decoder...");

    pinMode(INPUT_SIGNAL_PIN, INPUT); // Set the input pin

    // Initial read of the pin lineLevel (assuming it's stable at startup)
    // previous_stable_state = digitalRead(INPUT_SIGNAL_PIN);
    // Serial.print("Initial stable lineLevel: ");
    // Serial.println(previous_stable_state == HIGH ? "HIGH" : "LOW");
    setupTimer1();
}

// --- Main Loop ---
/**
 * @brief 
 * 
 */
void loop()
{
    if (bufISR1.len() > 0) {
        Serial.print("Started reading message: ");
        while (bufISR1.len() < bufISR1.peek() + 1) { // wait for message to be ready
            /// @todo Should only wait long enough for the message to be sent and print an error if it takes too long
            Serial.print(".");
        }
        Serial.println();
        Serial.print("Finished reading message: ");
        for (int i = 0; i < bufISR1.len(); i++) {
            Serial.print(bufISR1.pull());
            Serial.print(" ");
        }
        Serial.println();
    }
}
