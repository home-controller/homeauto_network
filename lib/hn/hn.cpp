/**
 * The code defines a communication protocol for a slow home network, including
 * functions for sending and receiving data packets, calculating CRC checksums,
 * and handling line contention.
 *
 * @param pin The code you provided seems to be implementing a custom
 * communication protocol for a slow home network using a single pin. It
 * includes functions for sending and receiving bits, handling collisions,
 * calculating CRC checksums, and managing network communication.
 */

#include <hn.h>
/*
 * slow Home Network.
 * 1: The line is pullup so to send data first pull it low.
 * 2: If when sending stuff the line is pulled down when you have released it
 * that means someone else was sending something to, so collision. 3: When
 * collision the one trying to send the high pulse gives up and waits for the
 * line to be free. This only works if we go slow enough that when we pull LOW
 * all points on the line are LOW at the same time, for the majority of the
 * pulse length. If you go to fast someone down the line can have sent the
 * pulse and moved on to the next bit before you even see it. Also there is
 * bounce from the line end so it can look like someone else sent a pulse when
 * it is your pulse coming back. And other problems with speed so the idea is
 * to go some slow that all that is no problem.
 *
 * Although I am no expert and just read a wiki page.
 *
 * This will work like a very cut-down CAN bus with no need for the extra
 * modules. Hopefully. :) very loosely
 */

void SlowHomeNet::attachIntToPin(byte pin) {}

/// @brief Slow home network.
/// @param pin The MCU pit to use.
SlowHomeNet::SlowHomeNet(byte pin) : networkPin{pin} {
  // networkPin = pin;
#ifndef noMcu_buildflag
  pinMode(pin,
          INPUT_PULLUP);  // should add some external resistor to make stronger pullup. be safer to use 2 pins and pull down through a transistor, easy to replace if you shore
                          // the line high. Would one of the solid state fuses be fast enough to save the pin?
  pin_bit_msk = digitalPinToBitMask(pin);
  pin_port = digitalPinToPort(pin);            // PORTB;
  pin_DDR_reg = portModeRegister(pin_port);    // port to the address of DDRx. The DDR(Data Direction Register)s are registers which
                                               // determine if the digital pin is output mode or input mode.
  port_IO_reg = portOutputRegister(pin_port);  // PORTB, PORTC, PORTD etc. registers for bi-directional I/O
  CurrentTime = micros();
  lastTime = CurrentTime - bitPulseLength;
#endif
}

void SlowHomeNet::exc() {
  // Todo / This could either handle stuff stored from the pin change
  // interrupt . Or message stored during failed send.
  // Todo: / Or if the first pull low pulse is long enough this
  // could read the incoming message. When resend is reliable could even wait
  // for next send if busy.
  // Todo / Or if you know you are going to be busy you
  // could pull the line low :P to use this without interrupts would probably
  // have to increase the time of start pulse bit length.
}

/**
 * @brief Send bits on the home network IO pin, stop sending on line collision.
 *
 * @details Send bits while checking for line contention. High is a resistor pullup and
 * collision is detected when another unit pulls low while we are trying to send
 * a High bit. We should then stop sending and read the message being sent.
 * This function just stops and returns the bit pos that was LOW when we
 * expected High. Also not sure if should be using INPUT_PULLUP or if the line
 * should be pulled up with a resistor on the line wire. Should maybe find out
 * the line impedance and add the appropriate resistor at each end?
 *
 * @param bits Bits to send, need to send msb first so lowest number priority
 * works. Although if numberOfBits is less than 8 bits then, high end bit(s) will be discarded.
 * @param numberOfBits Number of bits to send, max 8 as "bits" is a byte.
 * @return byte. 0 for success or else the bit number the collision was on.
 */
byte SlowHomeNet::sendBits(byte bits, byte numberOfBits) {
#ifndef noMcu_buildflag
  byte x, y;
  for (x = numberOfBits; x > 0; x--) {
    if (((bits >> (x - 1)) bitand 0b1) == 1) {
      pinMode(networkPin, INPUT_PULLUP);
      delayMicroseconds((bitPulseLength - DigitalWriteTime) >> 2);
      for (y = 1; y <= 3; y++) {
        if (digitalRead(networkPin) == LOW) {
          return x + 1;
        }  // return pos of collision. Can then continue to read
           // incoming higher priority message.
        delayMicroseconds((bitPulseLength >> 2) - DigitalReadTime);
      }
    } else {
      pinMode(networkPin, OUTPUT);
      digitalWrite(networkPin, LOW);
      delayMicroseconds(bitPulseLength - DigitalWriteTime);
    }
  }
  // if (!checkPinInput()) pinMode(networkPin, INPUT_PULLUP);
  return 0;
#endif
}

/**
 * @brief  Checks if a pin is set as an input or
 * output.
 *
 * @return A boolean value that indicates whether the pin
 * is set as an input or not. Returns true(= 1) if pin is set to input. It checks if the pin's corresponding bit in the
 * pin data direction register (`pin_DDR_reg`) is clear (equal to 0), which
 * means the pin is set as an input.
 */
boolean SlowHomeNet::checkPinInput() { return ((*pin_DDR_reg & pin_bit_msk) == 0); }  ///< If bit = 0 then pin is input.

/// @return returns the pin this network is using.
byte SlowHomeNet::getPinNo() { return networkPin; }
// void SlowHomeNet::pin(byte p) {
//     networkPin=p;
// }

/// @brief If there is room to store the whole message+date in the buffer, then setup some vars and push the number of bytes that will be stored in the buffer.
/// Also stores the RTR in the high bit.
/// @param l length code, if RTR high bit is present it will be ANDed out.
/// @param RTR If this is a Remote Transmission Request, can be 0b1 or 0b10000000 etc.
/// @return 0 for success or 3 if not enough room in buffer.
byte SlowHomeNet::pushDataLen(byte l, byte RTR = 0) {
  byte t;
  t = getDataLen(l);  // t needs to be the number of bytes stored including the message id. the high bit will store the RTR bit.
  t += getMessageLen(l);
  if (buf.space() < (t + 1)) return 3;  // if there is not room to store all message plus, data plus, len byte then quit trying :)
  if (RTR > 0) l |= (0b1 << 7);
  bufIndexPartMessageAt = nextIndex();
  buf.push(l);  // command with 1 byte of data. If it was more we would of had priority.
  return 0;
}

/// @brief as pushDataLen() does setup and checks for buffer room this just pushes the message.
/// @param m message id.
/// @return 0 for success.
byte SlowHomeNet::pushMessageId(byte m) {
  // byte t = sendBits(m, 8);

  buf.push(m);  // command with 1 byte of data. If it was more we would of had priority.
  return 0;
}

/// @brief Try to send the SOF bits. The calling function should make sure the line is not part way through a message.
/// @return This should return the SOF bits.
/// if not there is a problem with missing messages, SOF length mismatch between units or other hardware/software problems.
byte SlowHomeNet::sendStartOfFrame() {
  byte r;
  r = sendBits(SOFValue, SOFBits);
  if (r == 0) return SOFValue;
  else return 0;  // The line High at the end of the pull LOWs is the only thing that can change.
}

/// @brief Try to send the RTR(Remote Transmission Request) bit. While following message priority.
/// @param v the bit to send.
/// @return If another unit has priority(sending 0 when we send 1) return their value else return v
byte SlowHomeNet::sendRTR(byte v) {
  // v = v bitand 0b1;
  if (sendBits(v, 1) > 0) {
    return 0;
  } else return v;
}

/// @brief Send the code for the length of data being sent.
/// @param v The data length code.
/// @return the code sent on the bus, not necessarily form this node.
byte SlowHomeNet::sendDataLen(byte v) {
  byte sb = sendBits(v, DataLengthBitsLn);
  if (sb > 0) {
    // As we lost out to a higher priority message we are in receiving mode now.
    byte x = (1 << (DataLengthBitsLn - sb)) - 1;
    v = v bitand x;
    x = 3 - sb;
    bitSet(v, x);
    byte r = readBits(DataLengthBitsLn - sb);
    return (r bitor v);
  } else {
    return v;
  }
}

/// @brief Sends 1 byte of data.
/// @param v 8 bits of data to send.
/// @return The data sent on the bus, not necessarily form this node.
byte SlowHomeNet::sendMessageId(byte v) {
  // v = v bitand 0b1;
  byte sb = sendBits(v, 8);
  if (sb > 0) {
    byte x = (1 << (8 - sb)) - 1;  // for example if dataSent = 2 this will give x = 0b00111111
    v = v bitand x;
    bitSet(v, (8 - sb));
    byte r = readBits(8 - sb);
    return (r bitor v);
  } else return v;
}

byte SlowHomeNet::sendData(byte v) { return sendMessageId(v); }

word SlowHomeNet::sendData(word v) {
  word b1 = sendMessageId(highByte(v));  // try to send fist send high byte.
  if (b1 == highByte(v)) { return ((b1 << 8) + sendMessageId(lowByte(v))); }
  return ((b1 << 8) + readBits(8));
}

/// @brief Send the CRC. If we get to to the point of sending the CRC it should only fail if there is a line error.
/// @param v The CRC to send.
/// @return The CRC sent on the bus, not necessarily form this node.
byte SlowHomeNet::sendCRC(byte v) {  // this shouldn't fail as all the date is already sent. Also sends the CRC delimiter
  // v = v bitand 0b1;
  byte sb = sendBits(v, 4);
  if (sb > 0) {                    // If this happens there is a line error or code bug.
    byte x = (1 << (4 - sb)) - 1;  // for example if dataSent = 2 this will give x = 0b00111111
    v = v bitand x;
    bitSet(v, 4 - sb);
    byte r = readBits(4 - sb);
    sendBits(1, 1);
    return (r bitor v);
  } else {
    sendBits(1, 1);
    return v;
  }
}

/// @brief  Send the Ack bit plus the Ack delimiter bit.
/// @param v 1 or 0 for the ack bit, 0 indicates crc error.
/// @return 0 for v successful sent, 1 for line pulled low when v = 1, indicating some other unit had a receiving error.
byte SlowHomeNet::sendAck(byte v) {
  byte r = sendBits(v, 1);
  sendBits(1, 1);
  if (r == 0) return 0;
  else return 1;
}

/// @brief Send end of frame, and check for EOF error code(not decided yet)
/// @return 0 for no errors, 1 for errors. return value is temporary
/// error handling is not decided yet. For now storing in endOfFrameError the remaining EOF bits after the first pul low.
byte SlowHomeNet::sendEndOfFrame() {
  byte sb = sendBits(0xFF, 7);
  if (sb > 0) {
    // TODO error code, not implemented yet.
    // Not even sure if the error should be here or force the message to stop(by pulling low for 6 pulses) as soon as the error is found.
    // error should probably be handled at a lower level as a frame error
    byte r = readBits(7 - sb);  /// read the remaining end of frame bits. //TODO should this just always read 8 bits instead for 8 bit error code
    endOfFrameError = r;
    return 1;  /// TODO maybe this is the error code, not decided on error handling yet though.
  }
  endOfFrameError = 0;
  return 0;
}

/// @brief Store a byte command in the dataArray[] before calling send. The command byte starts at dataArray[0]
/// @param command The 1 byte message ID to be stored in dataArray[0]
/// @return 0, The code for the number of bytes stored(1 message byte).
byte SlowHomeNet::setDataArray(byte command) {
  dataArray[0] = command;
  return 0;
}

/// @brief Store a byte command and data in the dataArray[] before calling send. The data byte is at dataArray[1]
/// @param command The 1 byte message ID to be stored in dataArray[0]
/// @param data This needs to be of type byte as function overloading is used.
/// @return 1, The code for the number of bytes stored(1 message and 1 data).
byte SlowHomeNet::setDataArray(byte command, byte data) {
  dataArray[0] = command;
  dataArray[1] = data;
  return 1;
}

/// @brief Store 1 byte message and 2 bytes in the dataArray before calling send. High order byte at dataArray[0]
/// @param data The data to store to then send. This needs to be of type byte as function overloading is used.
/// @return 2: The code for the number of bytes stored(1 message and 2 data).
byte SlowHomeNet::setDataArray(byte command, word data) {
  if (sizeof(dataArray) >= 3) {
    dataArray[0] = command;
    dataArray[1] = highByte(data);
    dataArray[2] = lowByte(data);
    return 2;
  } else return 0;
}

/// @brief Store up to 4 bytes in the dataArray before calling send.
/// @param data the data to store to then send.
/// @param l the number of bytes to send. The high order bytes are stored first at dataArray[0]
/// @return The number of bytes stored.
byte SlowHomeNet::setDataArray(byte command, uint32_t data, byte l) {
  byte x;
  if (l > maxDataSize + maxMessageSize) l = maxDataSize + maxMessageSize;
  dataArray[0] = command;
  for (x = (l + 1) - 1; x >= 0; x--) {  // the + 1 is for the command length in bytes
    dataArray[x] = ((byte)(data bitand 0xFF));
    if (x > 0) data = data >> 8;
  }
  return l;
}

/// @brief  For sending message id and data packets over a network, should only be called when not already receiving a message
/// @param command A byte Representing the command byte(or message Id) that you want to send over the network.
/// @details It specifies the type of command or operation you want to perform. This command byte is used to communicate instructions
/// or actions between different units on the network
/// @param data A byte of data to send.
/// @return 0 for success or an error code, see sendHelper() function or look at error code #defines in top of header file
byte SlowHomeNet::send(byte command, byte data) { return sendHelper(0, 1, setDataArray(command, data)); };

/// @brief  For sending message with no data, should only be called when not already receiving a message
/// @param command A byte Representing the command byte(or message Id) that you want to send over the network.
/// @details It specifies the type of command or operation you want to perform. This command byte is used to communicate instructions
/// or actions between different units on the network
/// @return 0 for success or an error code, see sendHelper() function or look at error code #defines in top of header file
byte SlowHomeNet::send(byte command) { return sendHelper(0, 1, setDataArray(command)); };

/// @brief  Send message id and data(16 bits), should only be called when not already receiving a message
/// @param command A byte Representing the command byte(or message Id) that you want to send over the network.
/// @param data A word(uint16_t) of data to send.
/// @return 0 for success or an error code, see sendHelper() function or look at error code #defines in top of header file
byte SlowHomeNet::sendW(byte command, word data) {
  byte t = setDataArray(command, data);
  // Serial.print(F("Setup array and return length code: "));
  // Serial.print(t);
  return sendHelper(0, 1, 2);
};

/**
 * @brief For sending message id and data packets over a network, should only be called when not already receiving a message
 *
 * @details handling various scenarios such as sending the message framwork, message id, data transmission and CRC calculation.
 * TODO: needs expanding to handle more than (0 or 1) byte of data.
 *
 * @param RTR Remote Transmission Request, 0 = sending message, 1 = request another unit to send a message.
 * Defaults to send message (= 0).
 *
 * @param mLen Message length in bytes.
 *
 * @param dLen Data length in bytes.
 *
 * @return The function `SlowHomeNet::send` returns different values based on
 * the outcome of the communication process. Here are the possible return
 * values and their meanings:
 *  0,  Successfully sent and received Ack.
 *  1,  line error.
 *  16, warn no Ack. No other uint handed the message.
 *  17, Higher priority message being sent, received in buffer.
 *  18, network SOF mismatch on different units, network down or not reading all incoming messages properly and checking for in middle of message.
 *  19, unhandled data size.
 *
 */
byte SlowHomeNet::sendHelper(byte RTR = 0, byte mLen = 1, byte dLen = 0) {
  byte sent, crc, t, dataLenCode, i;
  // byte crcBuf[2];

  if (sendStartOfFrame() == SOFValue) {  // Try to send start of frame.

    // Send RTR (Remote Transmission Request).
    t = sendRTR(RTR);
    if (t != RTR) {
      RTRLenCode = (t << 7);
      receiveRest(1);  // TODO: test this function.
      return 17;       // Received message in buffer
    }

    // next handle data length
    dataLenCode = getLenCode(mLen, dLen);
    dLen = getDataLen(dataLenCode);     // As getLenCode() give a default code for invalid values this should kind of fix an invalid value for dLen
    mLen = getMessageLen(dataLenCode);  // same as above.

    sent = sendDataLen(dataLenCode);
    if (sent != dataLenCode) {  // lost bus priority.
      // t = pushDataLen(sent, RTR);  // push RTR and message/data length.
      // if (t > 0) return t;         // No room in buffer, returning.
      // Next fetch rest of message.
      RTRLenCode |= sent;
      receiveRest(4);  // TODO: test this function.
      return 17;       // Received message in buffer
    }

    // TODO handle date more than 1 byte. ATM it will always be 0 or 1 byte though.
    // Send message byte(s?)
    for (i = 0; i < mLen; i++) {
      sent = sendMessageId(dataArray[0]);  // TODO: Update to work with more than 1 byte message lengths.
      if (sent != dataArray[i]) {
        // t = pushDataLen(dataLenCode);
        // if (t > 0) return t;
        // buf.push(sent);
        dataArray[i] = sent;
        receiveRest(4 + 8 + (8 * i));
        return 17;  // Received message in buffer
      }
    }

    // Send data byte(s?)
    for (i = mLen; i < mLen + dLen; i++) {
      sent = sendData(dataArray[i]);
      if (sent != dataArray[i]) {
        dataArray[i] = sent;
        // t = pushDataLen(dataLenCode);
        // if (t > 0) return t;
        // byte x;
        // for (x = 0; x < i; x++) buf.push(dataArray[x]);  // push message plus data bytes befor this on one if any.
        // buf.push(sent);
        receiveRest(4 + 8 + (8 * (i + 1)));  // 1 RTR bit + 3 length bits, 8 bits of message ID, (8 * (i + 1)) data bits from this for loop including 8 for this time.
        return 17;                           // Received message in buffer
      }
    }

    // send CRC
    crc = Crc4(dataArray, mLen + dLen);
    sent = sendCRC(crc);  // if this fails we have a line error or bug in the code.

    // send Ack
    if (sent != crc) {           // CRC fail so send Ack failure.
      sent = sendBits(0b01, 2);  // Ack fail plus Ack delimiter.
      // If sent is different it means another unit sent the same message but somehow we received a different CRC.
      // Or there is some other line error or code bug.
      //  TODO we could check for delimiter error here.
      sendEndOfFrame();
      return Error_AckError;
    } else {
      sent = sendBits(0b1, 1);
      if (sent != 0) {  // Another unit signaled a receive error.
        sendEndOfFrame();
        return Error_AnotherUnit_AckError;
      }
      sent = sendBits(0b1, 1);  // TODO Not bothering to check for delimiter errors
    }

    // Send end of frame
    sent = sendEndOfFrame();
    if (sent != 0) { return 21; }
  } else {
    /// SOF send fail, this shouldn't happen, code or line fault
    /// TODO: Do more here.
    return Error_NetworkProblem;
  }
  // Serial.print(F(", dataLenCode sent: "));
  // Serial.print(dataLenCode);
  // Serial.print(F(", mLen sent: "));
  // Serial.print(mLen);
  // Serial.print(F(", dLen sent: "));
  // Serial.println(dLen);
  return 0;
}
/**
 * @brief Receive remaining message frame, see @@details for more.
 *
 * @details Note, this is expecting that a message is already being sent on the bus so it will not wait for the start bit as it should already be sent
 * Any bits stored in the buffer must already be shifted to the right place as this will just or the new bits in.
 *
 * If bitPos > (RTR + length code bits) then checks for buffer space else presumes the calling func has already done it.
 *
 * @param pitPos is the bit position of the last received bit, any lead-in bit(s) are not counted.
 * @return Byte, 0 for no errors else an error code, see Error_ code #defines in header file.
 */
byte SlowHomeNet::receiveRest(byte bitPos) {
  byte r, crc, t, i, ack, return_error;
  byte mLen;  // length in bytes.
  byte dLen;  // length in byt.
  return_error = 0;
  // get RTR bit if needed. RTR = (Remote Transmission Request).
  if (bitPos == 0) RTRLenCode = readBits(1) << 7;  // RTRLenCode is a private class var.
  // byte rtrBit = RTRLenCode;

  // get any remaining RTR and dataLength bits.
  if (bitPos < (DataLengthBitsLn + 1)) {  // FDLBits = frame data+message length code bit number in frame.
    if (bitPos > 1) {
      r = readBits((DataLengthBitsLn + 1) - bitPos);
      RTRLenCode |= r;  // RTRLenCode is a class var
    } else RTRLenCode |= readBits(DataLengthBitsLn);
  }
  /// Message len in bytes
  // byte readDataLenField = RTRLenCode; only needed for debugging
  mLen = getMessageLen(RTRLenCode);
  dLen = getDataLen(RTRLenCode);
  byte mLenBits = mLen << 3;  // length in bits.
  byte dLenBits = dLen << 3;  // length in bits.
  // get any remaining message ID  bits
  if (bitPos < (4 + mLenBits)) {
    if (bitPos > 4) {  // If already read some of the Message ID field then finish any part byte and setup to read the rest.
      r = readBits(8 - ((bitPos - (DataLengthBitsLn + 1)) bitand 0b111));  // x bitand 0b111, gives the remainder after dividing by 8 i.e. the remaining bits of 1 byte
      t = ((bitPos - (DataLengthBitsLn + 1)) >> 3);                        // shift right 3 = divide by 8;
      dataArray[t] = dataArray[t] bitor r;
    } else t = 0;  // else setup to read all the Message ID field.
    for (i = t; i < mLen; i++) {
      // uint32_t st, et; // Used to debug timings for readBits
      // st = micros();
      r = readBits(8);
      // et = micros();
      if (i > (0)) Serial.print(F("\n\r Error in for loop(get message)"));
      dataArray[i] = r;
    }
  }

  // Get data
  if (bitPos < (4 + mLenBits + dLenBits)) {
    if (bitPos > (4 + mLenBits)) {                     // If already read some of the Message ID field then finish any part byte and setup to read the rest.
      t = bitPos - (DataLengthBitsLn + 1 + mLenBits);  // the number of bit of of date field already read.
      r = readBits(t bitand 0b111);                    // Any part byte left to read.
      t >>= 3;                                         // shift right 3 = divide by 8; To give the number of full bytes.
    } else {                                           // else setup to read all data field.
      t = mLen;                                        // Where the first byte of data is stored in the array.
    }
    for (i = t; i < (mLen + dLen); i++) {
      // if (i > (1)) Serial.print(F("\n\r Error in for loop(get data)"));
      r = readBits(8);
      dataArray[i] = r;
    }
  }

  // get CRC
  if (bitPos <= (4 + ((mLenBits + dLenBits) >> 3))) crc = readBits(4);
  else {
    Serial.print(F(" Error with CRC read. shouldn't' get here"));
    return_error = Error_CRCError;  // should only happen if we send the same message + data as another but end up with a different CRC
                                    // So line error or bug in code.
  }
  byte crcCalc = Crc4(dataArray, mLen + dLen);
  readBits(1);  // CRC delimiter. Delimiter is high.
                // if (Crc4buf(bufSI) == crc) {
  if (crc == crcCalc) {
    // Check if anyone failed CRC while also waiting for the 2 bits to be sent.
    ack = sendAck(1);
    if (ack != 0) {  // another unit had CRC fail on receive.
      return_error = Error_AckError;
    }
  } else {
    sendAck(0);
    return_error = Error_AckError;
    Serial.print(F("\n\rFail crc. crc = 0b"));
    Serial.print(crc, BIN);
    Serial.print('(');
    Serial.print(crc);
    Serial.print(')');
    Serial.print(F(", crcCalc = "));
    Serial.print(crcCalc, BIN);
    Serial.print(", ");
  }

  // bufSI = buf.nextIndex();
  // tl = buf.getLength();
  // r = readBits(8);

  // Serial.print(F("Length code read: 0b"));
  // Serial.print(readDataLenField, BIN);
  // Serial.print(F(", Read 8 bits time: "));
  // Serial.print(et - st);
  // Serial.print(F(", RTR bit: "));
  // Serial.print(rtrBit, BIN);
  // Serial.print(F("\n\r RTRLenCode: "));
  // Serial.print(RTRLenCode, BIN);
  // Serial.print('[');
  // Serial.print(RTRLenCode);
  // Serial.println(']');
  // Serial.print(F(", Message len: "));
  // Serial.print(mLen, BIN);
  // Serial.print(F(", Data len: 0b"));
  // Serial.println(dLen, BIN);
  // Serial.print(F("RTR: 0b"));
  // Serial.print(RTRLenCode >> 7, BIN);
  // Serial.print(F(", Length code: 0b"));
  // Serial.print(RTRLenCode bitand 0b01111111, BIN);
  // Serial.print(F(", Id: "));
  // Serial.print(dataArray[0], BIN);
  // // Serial.println();
  // for (i = 1; i < (dLen + 1); i++) {
  //   Serial.print(F(", Data byte: ["));
  //   Serial.print(i);
  //   Serial.print("] = 0b");
  //   Serial.print(dataArray[i], BIN);
  // }
  // Serial.print(F(", crc: "));
  // Serial.print(crc);
  return return_error;
}

/**
 * @brief monitor if line changes level in the time of "pulses * bitPulseLength".
 *
 * @param pulses Number of bitPulseLength pulses of time to monitor the line for.
 * @param level
 * @return boolean return true if the line level changed else false.
 * @todo some CAN standards check the level of the pulse 87.5 percent along the
 * pulse length this gives any ringing time to settle.
 */
boolean SlowHomeNet::monitorLinePinForChange(byte pulses, byte level = 1) {
  byte x, c = 0;
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
 * @brief Check network IO pin and send first pull low bit. If success the pin will be left pulled LOW.
 *
 * TODO: For now this is ok but in most cases every unit should be receiving all messages so should already
 * know if a message is being sent on the network, so should maybe use sendBits() instead. Maybe only use
 * this after unit turn on or if have been not monitoring the line for some reason.
 *
 * @details Checks if the pin is high for the 'maxInuseHigh' time and if not
 * waits for upto "WaitForLineTimeout" for in to become free. if the line
 * is/becomes free send the start pulse and return true else return false.
 *
 * @return boolean true for success, false for timeout etc.
 */
boolean SlowHomeNet::getNetwork() {
  unsigned long timeOutStart;
  timeOutStart = millis();
  do {
    if (monitorLinePinForChange(MaxInUseHighBits, 1) == false) {
      digitalWrite(networkPin, LOW);
      delayMicroseconds(bitPulseLength - DigitalWriteTime);
      // pinMode(networkPin, INPUT_PULLUP); // why did I ever add this?
      return true;
    }
  } while ((millis() - timeOutStart) < WaitForLineTimeout);
  return false;
}

/// @brief Convert data length code to data length in bytes, this don't count the message id.
/// @param l Length code, only the 3 lower bit are used.
/// @return Length in bytes, shift left 3 to get bits. Max bytes of data is 4 bytes.
/// @details the space in the frame could be up to bytes but limiting to not hog line
byte SlowHomeNet::getDataLen(byte l) {
  l = l bitand 0b111;  // the length code can also have the RTR in the high bit.
  if (l <= 2) return l;
  if (l == 3) return 4;
  if (l == 4) return 8;
  if (l == 5) return 16;
  if (l == 6) return 32;
  if (l == 7) return 64;
  // error undefined for code greater than 3, return max size.
  return 4;
}

/// @brief Convert data length code to message id length in bytes, this don't count the data bytes.
/// @param l Length code, only the 3 lower bit are used.
/// @return Length in bytes, shift left 3 to get bits. So far should always be 4 or less.
byte SlowHomeNet::getMessageLen(byte l) {
  l >>= 3;
  l = l bitand 0b111;  // the length code can also have the RTR in the high bit.
  if (l <= 0) return 1;
  if (l <= 1) return 2;
  if (l <= 2) return 4;
  // if (l <= 3) return 8;

  // error undefined for code greater than 3, return max size.
  return 4;
}

/// @brief Convert data length code to message id + data, length in bytes.
/// @param l Length code, only the 3 lower bit are used.
/// @return Length in bytes.
byte SlowHomeNet::getMessageDataLen(byte l) { return (getMessageLen(l) + getDataLen(l)); }

/// @brief Get the message plus data length code
/// @param mLen The message length
/// @param dLen The data length;
/// @return The length code. If no code to fit params then return Max length.
byte SlowHomeNet::getLenCode(byte mLen, byte dLen) {
  byte dLenPart, mLenPart;
  if (dLen > maxDataSize) dLen = maxDataSize;
  if (mLen > maxMessageSize) mLen = maxMessageSize;

  if (dLen <= 2) dLenPart = dLen;
  else if (dLen <= 4) dLenPart = 3;
  else if (dLen <= 8) dLenPart = 4;
  else if (dLen <= 16) dLenPart = 5;
  else if (dLen <= 32) dLenPart = 6;
  else if (dLen <= 64) dLenPart = 7;
  else dLenPart = 7;

  if (mLen <= 1) mLenPart = 0;
  else if (mLen <= 2) mLenPart = 1;
  else if (mLen <= 4) mLenPart = 2;
  else if (mLen <= 8) mLenPart = 3;
  else if (mLen <= 16) mLenPart = 4;
  else if (mLen <= 32) mLenPart = 5;
  else if (mLen <= 64) mLenPart = 6;
  else mLenPart = 6;

  // Although the above handles up to 64 bytes of message ID + another 64 bytes of data lets limit it to 2 bytes of message Id pluse 4 bytes of Data.
  if (mLenPart > 2) mLenPart = 2;
  if (dLenPart > 3) dLenPart = 3;
  return ((mLenPart << 3) bitor dLenPart);
}

/// @brief Get the first message stored in the buffer, this will remove this message from the buffer.
/// @param a and array to return the Message id and the data if any.
/// @param RTR Return 1 if this is a Remote Transmission Request
/// @param mLen Used to return the Message length in bytes
/// @param dLen Used to return the data length in bytes
/// @return 0 for no problems else the error code.
byte SlowHomeNet::getFromBuf(byte a[], byte &RTR, byte &mLen, byte &dLen) {
  byte lc, i, p;
  if (recCount() < 1) return Error_NoMessageStoredToRetrieve;
  lc = buf.pull();
  p = 1;
  RTR = lc >> 7;
  mLen = getMessageLen(lc);
  dLen = getDataLen(lc);
  //  Serial.print(F("[lc: "));
  //   Serial.print(lc);
  //   Serial.print("]\n");
  //   Serial.print(F(", [dLen: "));
  //   Serial.print(dLen);
  //   Serial.print("]\n");
  for (i = 0; i < mLen; i++) {
    if (recCount() < 1) return Error_MessageNotInBuffer;
    a[i] = buf.pull();
    p++;
  }
  for (i = mLen; i < (mLen + dLen); i++) {
    if (recCount() < 1) return Error_DataNotInBuffer;
    a[i] = buf.pull();
    p++;
  }
  // Serial.print(F("[pull: "));
  // Serial.print(p);
  // Serial.print(']');
  return 0;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++ Reed/receive/ watch line/pin +++++++++++++++++++++++++++++++++

/**
 * @brief Read 'bits' number of bits from the line, Max 8
 *
 * @param bits The number of bits to read upto a max of 8.
 * @return byte The value read i.e. if bit read were 1,0,1,1 that would be 0b1011= 11.
 */
byte SlowHomeNet::readBits(byte bits) {
  byte bitCount, x, c, cl, cc, out;
  static byte y = 1;  // This is part of the one 8th of the of a bit pulse in a for loop. setting to 0 takes 1 off and setting to 2 adds 1.
                      // The can be used to adjust the timings to try and fix small timing mismatches.

  boolean level, levelC;
  level = digitalRead(networkPin);  // this always needs to be LOW or HIGH i.e. 1 or 0 as it it used with 'bitor' to set the last bit.
  if (bits > 8) bits = 8;
  out = 0;
  for (bitCount = 1; bitCount <= bits; bitCount++) {  // loop through the bits given by 'bits'
    c = 0;                                            // count of high pulse in middle of bit pulse
    cc = 0;                                           // Count of level opposite of expected towards the end of the bit.
    cl = 0;
    for (x = y; x <= 8; x++) {  // split each pulse into 8 and check the levels.
      y = 1;
#ifdef UnitTest
      inBitPos = x;
#endif

      levelC = digitalRead(networkPin);
      if (x <= 2) {  // first 1/4 of bit pulse
        if (levelC == HIGH) cl++;
      }
      if (x > 6) {  // last 1/4 of bit pulse
        if (levelC == HIGH) cc++;
      } else {  // middle part of bit pulse
        if (levelC == HIGH) c++;
      }
      delayMicroseconds(((bitPulseLength >> 3) - DigitalReadTime) - (ReadBitsLoopMicros));  // 11 12 13 15
      // Shift left 3 is same as divide by 8.(each shift left divides by 2) 488>>3 = 61, 488=0b111101000
      // Todo more accurate value for for loop code execution time i.e. DigitalReadTime.
      // arduino forum says 4.78µs in a for loop for digitalRead so subtracting 5 as a guess for the Arduino.
    }
    // try to correct timing errors
    if (c >= 3) {
      level = HIGH;  // pulse is split into 8 and 3 out of the middle 4 checks are HIGH
      if (cc == 0) y = 2;
      else if (cl == 1) delayMicroseconds(((bitPulseLength >> 3) - DigitalReadTime) - (ReadBitsLoopMicros));
    } else {
      level = LOW;
      if (cc >= 2) y = 2;
      else if (cl >= 2) delayMicroseconds(((bitPulseLength >> 3) - DigitalReadTime) - (ReadBitsLoopMicros));
    }
    out = ((out << 1) bitor level);
    level = levelC;  // if (levelC != level) level = levelC;
  }
  // Serial.print(F("pin: "));Serial.print(networkPin);
  // Serial.print(F(", level: "));Serial.print(level);
  // Serial.print(F(", bits: "));Serial.print(bits);
  // Serial.print(F(", out: "));Serial.print(out);
  // Serial.println();

  return out;
}

// /**
//  *
//  * TOdo This should be removed later.
//  *
//  * @brief get the number of bits of the same level on the line upto the max of 'pulses'
//  *
//  * @param pulses The max number of bits of the same level to wait for, Max of 0xFE.(but maybe should be 8 of something)
//  * @param level The level of bits to check for, 3
//  * @return byte, The number of bits of at least 1/2 a pulse length or 0xFF for timeout.
//  */
// byte SlowHomeNet::getPulseNo(byte pulses, byte level) {
//     byte x, c;
//     c = 0;
//     if (level > 1) level = digitalRead(networkPin);
//     else {
//         for (x = 1; x <= 4; x++) {
//             if (digitalRead(networkPin) != level) {
//                 delayMicroseconds((bitPulseLength >> 3) - DigitalReadTime);
//             } else
//                 break;
//         }
//     }
//     for (x = 1; x <= (8 * pulses) + 4; x++) {    //+4 is for in-case the timing is out by upto 1/2 a pulse
//         if (digitalRead(networkPin) != level) {  // if line level changed for two checks in a row return true. This is to allow for nosy line spikes.
//                                                  // not sure if this is a good idea or not. maybe just add a filter to the line would be better.
//             if (c > 0) {
//                 c = x bitand 0b111;                     // c = remainder of x div 8
//                 if (c > ((bitPulseLength >> 1) + 2)) {  // if remainder > (bitPulseLength / 2).
//                                                         // The +2 is for the extra wait for line spike check and maybe the same for the last call
//                                                         // and shouldn't hurt as a pulse should be more than 1/2 length anyway. Although not sure about the need to
//                                                         check at all.
//                     c = x >> 3 + 1;                     // c= x div 8 + 1
//                 } else {
//                     c = x >> 3;  // c = x div 8
//                 }
//                 return c;
//             }
//             c++;
//         } else {
//             if (c > 0) c--;
//         }
//         delayMicroseconds((bitPulseLength >> 3) - DigitalReadTime);
//         // Shift left 3 is same as divide by 8.(each shift left divides by 2) 488>>3 = 61, 488=0b111101000
//         // Todo more accurate value for for loop code execution time i.e. DigitalReadTime.
//         // arduino forum says 4.78µs in a for loop for digitalRead so subtracting 5 as a guess for the Arduino.
//     }
//     return 0xff;  // timeout, the line is staying at the same line level for longer/(more bits) than 'pulses'
// }

/// @brief Read start of frame field, This is mostly to skip past the start of frame field.
/// TODO: This is expecting the pull low for start of frame and will never return until it gets 1;
/// @return 0 for success or else an error code.
byte SlowHomeNet::checkSOF() {
  byte r, line;
  // Check for line going low. This is expecting the pull low for the start of the frame so not checking for middle of frame or anything like that.
  do { line = digitalRead(networkPin); } while (line == 1);  // permanent blocking, maybe change to an if // While line is pulled high. i.e. no network activity.
  r = readBits(SOFBits);
  if (r != SOFValue) {
#ifdef hn_debug
    Serial.print(F("\r\n Unexpected start of frame value: 0b"));
    Serial.println(r, BIN);
#endif
    return Error_LineErrorFrameStart;
  }
  return 0;
}

/**
 * @brief Blocking! Monitor the line for input.
 *
 * @details As this stays here until a message is received only good for
 * testing or if there are no other inputs from switches etc. Although I guess
 * there could be stuff on interrupts.
 * The first byte in the buffer holds the data length and if RTR bit
 *
 * @return byte Returns 0 for success, Message is stored in the buffer.
 */
byte SlowHomeNet::receiveMonitor() {  // should I add a timeout?
  byte i, t;

  // uint32_t stRTR, stCode, stM, stD, stCRC, etCRC;
  // byte rtr, lCode, m, d, crc;

  // Check start of frame and wait for start of RTR
  t = checkSOF();

  /* stRTR = micros();
  rtr = readBits(1);
  stCode = micros();
  lCode = readBits(3);

  stM = micros();
  m = readBits(8);

  stD = micros();
  d = readBits(8);
  stCRC = micros();
  crc = readBits(4);
  etCRC = micros();

  Serial.print(F("\n\rRTR:"));
  Serial.print(rtr, BIN);
  Serial.print(", Code:");
  Serial.print(lCode, BIN);
  Serial.print(", m:");
  Serial.print(m, BIN);
  Serial.print(", d:");
  Serial.print(d, BIN);
  Serial.print(", crc:");
  Serial.println(crc, BIN);                                                                                   // Serial.print(','); 0b1110
  Serial.println(F("Time for 1 bit rtr: 2048, 2:4096, 3:6144, 4:8192, 5:10240, 6:12288, 7:14336, 8:16384"));  //
  Serial.print(F("Time 1 bit: "));

  Serial.print(stCode - stRTR);
  Serial.print(F(", 3 bit code: "));
  Serial.print(stM - stCode);
  Serial.print(F(", 8 bit m: "));
  Serial.print(stD - stM);
  Serial.print(F(", 8 bit d: "));
  Serial.print(stCRC - stD);
  Serial.print(F(", 4 bit crc: "));
  Serial.println(etCRC - stCRC);
 */
  i = receiveRest(0);  // Get data length.
  if (i == 0) {
    i = pushDataLen(RTRLenCode, RTRLenCode >> 7);
    if (i != 0) {
      Serial.print(F("Error receiveMonitor: "));
      Serial.println(i);
      return i;
    } else {
      byte dLen = getMessageDataLen(RTRLenCode);
#ifdef hn_debug
      Serial.print(F("\n\rgetMessageDataLen(): "));
      Serial.println(dLen);
#endif
      for (i = 0; i < dLen; i++) { buf.push(dataArray[i]); }
    }

  } else return i;
  if (t != 0) {
    Serial.print(F("Start of frame Error: "));
    Serial.println(t);
  }
  return 0;
}

void SlowHomeNet::IntCallback() {  // expects 11 bit: 8 data 1 ack, 1 parity & 1
// low bit at start.
#ifndef noMcu_buildflag
  unsigned long t;
  word mod_t;
  byte state = _pinReg & _pinMask;  // LOW = 0 but HIGH value will = the mask not 1.
  // word dTemp = 0;
  //  if last state change time > 9 bits + 1/3 bit margin and was low then
  //  reset and wait for new start. 2: if was high then start counting.

  CurrentTime = micros();  // not sure if should try and use the registers strait?
  t = CurrentTime - lastTime;
  // If time since last called less then 1/2 pulse time ignore call
  if (t < (bitPulseLength >> 1)) { return; }
  t = CurrentTime - lastTime;
  lastTime = CurrentTime;
  mod_t = t & 0x7ff;  //= 11 bit mask (0x7ff = 2048 - 1 = 2^11 - 1 = 0b11111111111)
  t = t >> 11;
  if (mod_t > (bitPulseLength >> 1)) t++;  // t now = the number of bits sent with the last line pulse length.

  if (t + bitPos > 11) {
    if (overflowCount < 0xFF) overflowCount++;
    bitPos = 0;
  } else {
    if (bitPos == 0) {
      dataIn = 0;
      dFlags = 0;
    }
    bitPos += t;
    dataIn = dataIn << t;
    if (lastState == 0) {  // if line high add the t high bits (1s).  // if
      // state is low last state should be high.
      // parity is only about odd or even number of high bits so only
      // changes on high pulses.
      dFlags = (t + (dFlags & B1)) & B1;  // parity = last bit of parity + t i.e. = (parity + t)
      // bitand b00000001. hence count of all hight bits including
      // the parity will always be even i.e. last bit = 0
      dataIn |= (1 << t) - 1;  // e.g. t=3 becomes (1 << 3) sub 1 = B1000 sub 1 =
                               // B111. Also |= should be equivalent to += here.
    }  // else leave as already set to 0s with the shift left.
    // if(bitPos > 11){should never get here as would mean the interrupt was
    // delayed by a pulse length. (I hope :P)}
    if (bitPos >= 11) {  // Should never be greater than 11.
      if ((dFlags & B1) == 0) {
        dataIn = dataIn >> 2;
        buf.push(dataIn);
      } else {  // parity error
        if (parityErrorCount < 0xFF) parityErrorCount++;
        bitPos = 0;
      }
    }
  }
  lastState = state;
#endif
}

//---------------------------------------------------- Write ----------------------------------

/**
 * @brief taken strait from
 * https://github.com/PaulStoffregen/OneWire/blob/master/OneWire.cpp Can't be
 * bothered to change it but think I will xor the high and low nibbles together
 * and just send 4 bits of CRC.
 */

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
                                                  0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8, 0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OneWireCrc8(const uint8_t *addr, uint8_t len, uint8_t crc = 0) {
  // uint8_t crc = 0;

  while (len--) {
    crc = *addr++ ^ crc;  // just re-using crc as intermediate
    crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^ pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
  }

  return crc;
}

/**
 * @brief  The function `Crc4` calculates a 4-bit CRC checksum using the
 * OneWireCrc8 algorithm.
 *
 * @param addr The `addr` parameter is a pointer to an array of bytes ( data),
 * which points to the data for which the CRC (Cyclic Redundancy Check)
 * calculation is being performed.
 * @param len Represents the length of the data in bytes that you want to
 * calculate the CRC (Cyclic Redundancy Check) for.
 *
 * @return The function `SlowHomeNet::Crc4` is returning the lower 4 bits of
 * the result of the XOR operation between the upper 4 bits and the lower 4
 * bits of the CRC value calculated by the `OneWireCrc8` function.
 */
byte SlowHomeNet::Crc4(uint8_t *addr, uint8_t len) {
  byte crc;
  crc = OneWireCrc8(addr, len);
  return (((crc >> 4) xor (crc)) bitand 0b1111);
}

/// @brief works out the CRC from the frame stored in the buffer, handles wraparound & getting the frame data length from [i]
/// @param i Index into the array the circular buffer is stored in where the first byte of the frame is stored. i.e. the data length and options.
/// @return the CRC.
byte SlowHomeNet::Crc4buf(uint8_t i) {
  byte crc, l;
  l = buf.getBufArrayElement(i) bitand 0b11;
  if (l == 3) l = 4;  //[0,8,16,32] bits
  l++;                // command id byte.
  i = buf.bufIndex(i + 1);
  if (i + (l - 1) >= buf.maxLen()) {                   // if buf wraps around from the end to the back to the beginning.
    crc = OneWireCrc8(buf.bufP(i), buf.maxLen() - i);  // 8-7=1 or 8-5=3 with l=2
    crc = OneWireCrc8(buf.bufP(0), l - (buf.maxLen() - i), crc);
  } else {
    crc = OneWireCrc8(buf.bufP(i), l);
  }
  return (((crc >> 4) xor (crc)) bitand 0b1111);
}

//======================================================hardware dependent fuctions below here================================================

// Hardware dependent pin/line manipulation functions. May need to be different depending on the board e.g. for arduino nano or RP2040
// should also probably include a test dummy function for testing unit code.