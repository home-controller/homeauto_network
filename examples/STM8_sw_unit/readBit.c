#include <Arduino.h> 

#define bitPulseLength 2048 // 1 bit takes 2048 microseconds (~= 1e6 / 488 = 2049.18) (microsecond = 1 millionth of a second).
#define DigitalWriteTime 4   // forums says 4.5µs but I think than includes the for loop
#define DigitalReadTime 5    // forums says 4.78µs but I think than includes the for loop
#define ReadBitsLoopMicros 3 // this is for each time though the loop that checks 8 time per bit so a value of 3 would be 24µs per bit
#define NetworkPinIn 5

#define and &&
#define bitand &
#define or ||
#define bitor |
#define xor ^


byte readBit() {
  byte cStart = 0; // count of high pulse at start of bit pulse
  byte cMid = 0;   // count of high pulse in middle of bit pulse
  byte cEnd = 0;   // count of high pulse towards the end of the bit.
  byte pulsePoint;

  static byte startAdjust = 1; // Normally when set to 1 the for loop will check the bit pulse 8
                               // points in time bitPulseLength/8 microseconds apart. If set to 0
                               // will check an extra time so time taken is [ bitPulseLength +
                               // bitPulseLength/8 ] If set to 2 will check 7 times so time taken is
                               // [ bitPulseLength - bitPulseLength/8 ]

  boolean level;  // the worked out level form takeing the average level at
                  // mutiple points in the bit pulse.
  boolean levelC; // level read from the GPIO pin

  // Serial.print(F("bitPulseLength = "));
  // Serial.println(((bitPulseLength >> 3) - DigitalReadTime) -
  // (ReadBitsLoopMicros));
  if (startAdjust == 0) {                                                                // No need to check end of pervious bit so skip ahead here first.
    delayMicroseconds(((bitPulseLength >> 3) - DigitalReadTime) - (ReadBitsLoopMicros)); // Skip ahead as we are likely behind
    startAdjust = 1;
  }

  // check the bit pulse at normally 8 points, but can be 7 to adjust timing
  for (pulsePoint = startAdjust; pulsePoint <= 8; pulsePoint++) { // split each pulse into 8 and check the levels.
#ifdef UnitTest
    inBitPos = x;
#endif

    levelC = digitalRead(NetworkPinIn);
    /// maybe cEnd, cMid would work better using more or less of the checks.
    if (pulsePoint <= 3) { // first 3/8 of bit pulse, first 3 out of 8
      // checks(can be 2 out of 7 if y is 2)
      if (levelC == HIGH) cStart++;
    } else if (pulsePoint >= 6) { // last 3/8 of bit pulse
      if (levelC == HIGH) cEnd++;
    }
    if ((pulsePoint >= 3) and (pulsePoint <= 6)) { // middle part of bit pulse
      if (levelC == HIGH) cMid++;
    }
    delayMicroseconds(((bitPulseLength >> 3) - DigitalReadTime) - (ReadBitsLoopMicros)); // 11 12 13 15
                                                                                         // Shift left 3 is same as divide by 8.(each shift left divides by
                                                                                         // 2) 488>>3 = 61, 488=0b111101000 Todo more accurate value for
                                                                                         // for loop code execution time i.e. DigitalReadTime. arduino
                                                                                         // forum says 4.78µs in a for loop for digitalRead so subtracting
                                                                                         // 5 as a guess for the Arduino.
  }

  if (cMid >= 3) level = HIGH;
  else level = LOW;

  // try to correct timing errors, if not
  if (startAdjust == 1) { // Check full 8 points have been checked. This means we can't
    // speedup 2 bits in a row we could instead correct for cStart
    // missing 1 check by adding 1 to it if cMid>=3
    if (level == HIGH) { // pulse is split into 8 and at least 3 out of the
                         // middle 4 checks are HIGH
      if (cStart < 2)    // If 2 out of the 3 checks at the start are different from
                         // the middle, start is likely still part of last bit.
        startAdjust = 0;
      if (cEnd <= 1)   // cEnd should be 3. So line noise or reading part of
                       // next bit if lower.
        startAdjust++; // Next bit don't check at fist point in bit as
                       // probably already missed it.
    } else {
      if (cEnd >= 2)     // Should be 0. Alow 1 for noise or slight timeing mismatch
        startAdjust = 2; // Next bit don't check at fist point in bit as
                         // probably already missed it.
      if (cStart >= 2)   // Not used else to alow bad noise to cancel out. Although at
                         // that point probably unreadable anyway.
        startAdjust--;   // This bit is likely still being sent, so wait a
                         // bit before reading next one.
    }
  } else {
    startAdjust = 1;
  }
  return level;
}