#include "hn.h"
/*
 * slow Home Network.
 * 1: The line is pullup so to send data first pull it low.
 * 2: If when sending stuff the line is pulled down when you have released it that means someone else was sending something to, so collision.
 * 3: When collision the one trying to send the high pulse gives up and waits for the line to be free.
 * This only works if we go slow enough that when we pull LOW all points on the line are LOW at the same time, for the majority of the pulse length.
 * If you go to fast someone down the line can have sent the pulse and moved on to the next bit before you even see it.
 * Also there is bounce from the line end so it can look like someone else sent a pulse when it is your pulse coming back.
 * And other problems with speed so the idea is to go some slow that all that is no problem.
 *
 * Although I am no expert and just read a wiki page.
 *
 * This will work like a very cut-down CAN bus with no need for the extra modules. Hopefully. :)
 * very loosely
 */
void SlowHomeNet::attachIntToPin(byte pin) {
}

SlowHomeNet::SlowHomeNet(byte pin) {
    networkPin = pin;
    pinMode(pin, INPUT_PULLUP);  // should add some external resistor to make stronger pullup. be safer to use 2 pins and pull down through
                                 // a transistor, easy to replace if you shore the line high.
                                 // Would one of the solid state fuses be fast enough to save the pin?
    pin_bit_msk = digitalPinToBitMask(pin);
    pin_port = digitalPinToPort(pin);            // PORTB;
    pin_DDR_reg = portModeRegister(pin_port);    // port to the address of DDRx. The DDR(Data Direction Register)s are registers which determine if the digital pin is output mode or input mode.
    port_IO_reg = portOutputRegister(pin_port);  // PORTB, PORTC, PORTD etc. registers for bi-directional I/O
}

/**
 * @brief Send bits on the home network IO pin.
 *
 * @details Send bits while checking for line contention. High is pullup and collision is detected when another unit pull low while we
 * are trying to send a High bit. We should then stop sending and read the message being sent. This function just stops and returns the bit pos
 * that was LOW when we expected High. Also not sure if should be using INPUT_PULLUP or if the line should be pulled up with a resistor on the
 * line wire. Should maybe find out the line impedance and add the appropriate resistor at each end?
 *
 * @param bits Bits to send, need to send msb first so lowest number priority works.
 * @param numberOfBits Number of bits to send, max 8 as "bits" is a byte.
 * @return byte. 0 for success or else the bit number the collision was on.
 */
byte SlowHomeNet::sendBits(byte bits, byte numberOfBits) {
    byte x, y;
    for (x = numberOfBits; x > 0; x--) {
        if (((bits >> (x - 1)) bitand 0b1) == 1) {
            pinMode(networkPin, INPUT_PULLUP);
            delayMicroseconds((bitPulseLength - DigitalWriteTime) >> 2);
            for (y = 1; y <= 3; y++) {
                if (digitalRead(networkPin) == LOW) { return x + 1; }  // return pos of collision. Can then continue to read incoming higher priority message.
                delayMicroseconds((bitPulseLength >> 2) - DigitalReadTime);
            }
        } else {
            pinMode(networkPin, OUTPUT);
            digitalWrite(networkPin, LOW);
            delayMicroseconds((bitPulseLength >> 2) - DigitalWriteTime);
        }
    }
    if (!checkPinInput()) pinMode(networkPin, INPUT_PULLUP);
    return 0;
}

boolean SlowHomeNet::checkPinInput() {
    return ((*pin_DDR_reg & pin_bit_msk) == 0);  // set bit is output.
}

byte SlowHomeNet::send(byte command, byte data) {
    byte sent, dataSent, crc;
    byte crcBuf[2];
    if (getNetwork()) {
        sent = sendBits(command, 8);
        if (sent == 0) {
            sendBits(0, 1);                    // Data Frame (RTR = 0) or a Remote-Request Frame (RTR = 1).
                                               // send  control field
            if (data > 0) {                    // send 1 byte of data. control field = 0b01. no extended or res bits yet at least.
                if (sendBits(0b01, 2) == 0) {  // send 1 byte of data.
                    dataSent = sendBits(data, 8);
                    if (dataSent == 0) {  // byte of data sent
                        crcBuf[0] = command;
                        crcBuf[1] = data;
                        crc = Crc4(crcBuf, 2);
                    } else {  // bus contention at data bit dataSent
                        // read remainder of message
                        if (lengthB >= (bufLength - 3)) return 3;
                        bitPos = 8 + 3 + dataSent;
                        buff[headP + lengthB + 1] = command;
                        buff[headP + lengthB + 2] = (data >> (8 - dataSent)) bitand (byte)(~0b1);
                        buff[headP + lengthB + 0] = 0b001;  // command with 1 byte of data. If it was more we would of had priority.
                        lengthB += 3;
                        return 17;  // Higher priority message being sent. Maybe read it.
                    }

                } else {
                    // another unit is sending command with no extra data.
                    if (lengthB >= (bufLength - 2)) return 3;  // out of buff space for input.
                    buff[headP + lengthB + 0] = 0b000;
                    buff[headP + lengthB + 1] = command;
                    lengthB += 2;
                    bitPos = 8 + 3;
                    return 17;  // Higher priority message being sent. Maybe read it.
                }
            } else {
                crc = Crc4(&crc, 1);
            }
            crc = sendBits(crc, 4);
            if (crc == 0) {                             // next send CRC delimiter
                if (sendBits(1, 4) != 0) { return 1; }  // not sure about flagging this as a line error as this is just a delimiter.
                if (sendBits(1, 4) != 0) {              // Not success for sending 1 means Ack from other unit.
                    // successfully sent and received Ack.
                    return 0;  // We do not have to wait for the Ack delimiter and the end of frame bits as they are just high bits.
                } else {
                    // No Ack
                    return 16;  // warn 16 = no Ack. No other uint handed the message.
                }
            } else {       /* error as the crc should be right even if 2 or more units are sending the same message */
                return 1;  // error 1 = line error.
            }
        } else {
            // read rest of higher priority message other unit is sending.
        }
    } else {
        // can't acquire network.

        return 18;  // warn: network buzzy or down.
    }
    return byte();
}

/**
 * @brief monitor if line changes level in the time of "pulses * bitPulseLength".
 *
 * @param pulses Number of bitPulseLength pulses of time to monitor the line for.
 * @return boolean return true if the line level changed else false.
 */
boolean SlowHomeNet::monitorLinePinForChange(byte pulses, byte level) {
    byte x, c;
    if (level > 1) level = digitalRead(networkPin);
    for (x = 1; x <= 8 * pulses; x++) {
        if (digitalRead(networkPin) != level) {  // if line level changed for to checks in a row return true. This is to allow for nosy line spikes.
                                                 // not sure if this is a good idea or not. maybe just add a filter to the line would be better.
            if (c > 0) return true;
            c++;
        } else {
            if (c > 0) c--;
        }
        delayMicroseconds((bitPulseLength >> 3) - DigitalReadTime);  // Shift left 3 is same as divide by 8.(each shift left divides by 2)
        // We should really work out how many clock cycles are used in the loop [DigitalReadTime] and sub that from the delay to be accurate.
        // arduino forum says 4.78µs in a for loop for digitalRead so subtracting 5 as a guess for the Arduino.
    }
    return false;
}

/**
 * @brief Check network IO pin and send first line pulse.
 *
 * @details Checks if the pin is high for the 'maxInuseHigh' time and if not waits for upto "WaitForLineTimeout" for in to become free.
 * if the line is/becomes free send the start pulse and return true else return false.
 *
 * @return boolean true for success, false for timeout etc.
 */
boolean SlowHomeNet::getNetwork() {
    unsigned long timeOutStart;
    timeOutStart = millis();
    do {
        if (monitorLinePinForChange(maxInuseHigh + 1, 1) == false) {
            digitalWrite(networkPin, LOW);
            delayMicroseconds((bitPulseLength >> 2) - DigitalWriteTime);
            pinMode(networkPin, INPUT_PULLUP);
            return true;
        }
    } while ((millis() - timeOutStart) < WaitForLineTimeout);
    return false;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++ Reed/receive/ watch line/pin +++++++++++++++++++++++++++++++++

/**
 * @brief get the number of bits of the same level on the line upto the max of 'pulses'
 * 
 * @param pulses The max number of bits of the same level to wait for, Max of 0xFE.(but maybe should be 8 of something)
 * @param level The level of bits to check for, 3
 * @return byte, The number of bits of at least 1/2 a pulse length or 0xFF for timeout.
 */
byte SlowHomeNet::getPulseNo(byte pulses, byte level) {
    byte x, c;
    c = 0;
    if (level > 1) level = digitalRead(networkPin);
    for (x = 1; x <= (8 * pulses)+4; x++) {//+4 is for in-case the timing is out by upto 1/2 a pulse
        if (digitalRead(networkPin) != level) {  // if line level changed for two checks in a row return true. This is to allow for nosy line spikes.
                                                 // not sure if this is a good idea or not. maybe just add a filter to the line would be better.
            if (c > 0) {
                c = x bitand 0b111;                     // c = remainder of x div 8
                if (c > ((bitPulseLength >> 1) + 2)) {  // if remainder > (bitPulseLength / 2).
                                                        // The +2 is for the extra wait for line spike check and maybe the same for the last call
                                                        // and shouldn't hurt as a pulse should be more than 1/2 length anyway. Although not sure about the need to check at all.
                    c = x >> 3 + 1;                     // c= x div 8 + 1
                } else {
                    c = x >> 3;  // c = x div 8
                }
                return c;
            }
            c++;
        } else {
            if (c > 0) c--;
        }
        delayMicroseconds((bitPulseLength >> 3) - DigitalReadTime);
        // Shift left 3 is same as divide by 8.(each shift left divides by 2) 488>>3 = 61, 488=0b111101000
        // Todo more accurate value for for loop code execution time i.e. DigitalReadTime.
        // arduino forum says 4.78µs in a for loop for digitalRead so subtracting 5 as a guess for the Arduino.
    }
    return 0xff;// timeout, the line is staying at the same line level for longer/(more bits) than 'pulses'
}

/**
 * @brief Monitor the line for input.
 *
 * @details As this stays here until a message is received only good for testing or if there are no other inputs from switches etc.
 * Although I guess there could be stuff on interrupts.
 *
 * @return byte Returns 0 for success, Message is stored in the buffer.
 */
byte SlowHomeNet::receiveMonitor() {  // should I add a timeout?
    byte line, c;
    do {
        line = getPulseNo(1, HIGH);
        if (line == 1) {
            c++;
        } else {
            c = 0;
        }
    } while ((line == 0) or (c <= 2));
}

void SlowHomeNet::IntCallback() {  // expects 11 bit: 8 data 1 ack, 1 parity & 1
                                   // low bit at start.
    unsigned long t;
    word mod_t;
    byte state =
        _pinReg & _pinMask;  // LOW = 0 but HIGH value will = the mask not 1.
    // word dTemp = 0;
    //  if last state change time > 9 bits + 1/3 bit margin and was low then reset
    //  and wait for new start. 2: if was high then start counting.

    CurrentTime =
        micros();  // not sure if should try and use the registers strait?
    t = CurrentTime - lastTime;
    // If time since last called less then 1/2 pulse time ignore call
    if (t < (bitPulseLength >> 1)) {
        return;
    }
    t = CurrentTime - lastTime;
    lastTime = CurrentTime;
    mod_t =
        t & 0x7ff;  //= 11 bit mask (0x7ff = 2048 - 1 = 2^11 - 1 = 0b11111111111)
    t = t >> 11;
    if (mod_t > (bitPulseLength >> 1))
        t++;  // t now = the number of bits sent with the last line pulse length.

    if (t + bitPos > 11) {
        if (overflowCount < 0xFF)
            overflowCount++;
        bitPos = 0;
    } else {
        if (bitPos == 0) {
            dataIn = 0;
            dFlags = 0;
        }
        bitPos += t;
        dataIn = dataIn << t;
        if (lastState == 0) {  // if line high add the t high bits (1s).  // if state
                               // is low last state should be high.
            // parity is only about odd or even number of high bits so only changes on
            // high pulses.
            dFlags = (t + (dFlags & B1)) &
                     B1;             // parity = last bit of parity + t i.e. = (parity + t) bitand
                                     // b00000001. hence count of all hight bits including the
                                     // parity will always be even i.e. last bit = 0
            dataIn |= (1 << t) - 1;  // e.g. t=3 becomes (1 << 3) sub 1 = B1000 sub 1 =
                                     // B111. Also |= should be equivalent to += here.
        }                            // else leave as already set to 0s with the shift left.
        // if(bitPos > 11){should never get here as would mean the interrupt was
        // delayed by a pulse length. (I hope :P)}
        if (bitPos >= 11) {  // Should never be greater than 11.
            if ((dFlags & B1) == 0) {
                dataIn = dataIn >> 2;
                buff[(headP + lengthB) & bufMsk] = (byte)dataIn;
                lengthB++;
            } else {  // parity error
                if (parityErrorCount < 0xFF)
                    parityErrorCount++;
                bitPos = 0;
            }
        }
    }
    lastState = state;
}

//---------------------------------------------------- Write ----------------------------------

/**
 * @brief taken strait from https://github.com/PaulStoffregen/OneWire/blob/master/OneWire.cpp
 * Can be bothered to change it but think I will xor the high and low nibbles together and just send 4 bits of CRC.
 */

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
    0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OneWireCrc8(const uint8_t *addr, uint8_t len) {
    uint8_t crc = 0;

    while (len--) {
        crc = *addr++ ^ crc;  // just re-using crc as intermediate
        crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
              pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
    }

    return crc;
}

byte SlowHomeNet::Crc4(uint8_t *addr, uint8_t len) {
    byte crc;
    crc = OneWireCrc8(addr, len);
    return (((crc >> 4) xor (crc)) bitand 0b1111);
}