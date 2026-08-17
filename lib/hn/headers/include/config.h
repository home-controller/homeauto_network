#pragma once

#define SOFBits 2 /// @brief The number of SOF (Start of Frame) bits.

#ifdef STM8S003F3_sw_unit_v1

#define HDNetworkPinRX D5 // Home auto network input pin
#define HDNetworkPinTX D6 // Home auto network output pin
#define serial_speed 9600

#elif defined(ARDUINO)
#define serial_speed 115200
#define HDNetworkPinRX 2 // Home auto network input pin
#define HDNetworkPinTX 6 // Home auto network output pin

#endif

#define LineMinGapMs 100 // Make sure there is a gap of at least lineMinGapMs ms (class var)

/// @brief Number of pins used for the home automation network. each line. 1 for IO on same pin 2 for separate RX, TX pin, Can then use a high value
/// resistor on RX pin to stop/limit backfeeding/powering MCU high when not sending. TX can also use a transistor to pull the line low(no backfeeding).
#define HDnetworkPins 2
