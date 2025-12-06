# Design document

- [Design document](#design-document)
  - [Design](#design)
    - [The message frame](#the-message-frame)
      - [Also note:](#also-note)
    - [Interframe Space](#interframe-space)
      - [By default the minimum length in bits is](#by-default-the-minimum-length-in-bits-is)
    - [Maximum consecutive bits of the same value](#maximum-consecutive-bits-of-the-same-value)
      - [Fields that have bit stuffing:](#fields-that-have-bit-stuffing)
      - [Fields that do not have bit stuffing:](#fields-that-do-not-have-bit-stuffing)
    - [CRC Error checking](#crc-error-checking)
    - [Timings and Transmission speed](#timings-and-transmission-speed)
    - [Minimal needed to work for controlling lights with switches and temp](#minimal-needed-to-work-for-controlling-lights-with-switches-and-temp)
    - [Read bus](#read-bus)
      - [Mcu pins used:](#mcu-pins-used)
      - [Different methods to receive the bits sent on the line](#different-methods-to-receive-the-bits-sent-on-the-line)
        - [Not using timers/interrupts](#not-using-timersinterrupts)
        - [Using timers/interrupts](#using-timersinterrupts)
      - [Use Pin change interrupt to read the message](#use-pin-change-interrupt-to-read-the-message)
  - [Can protocol web pages](#can-protocol-web-pages)
    - [Cable lengths](#cable-lengths)
    - [Other maximum cable lengths are (these values are approximate)](#other-maximum-cable-lengths-are-these-values-are-approximate)
    - [Maximum cable length at bit rate](#maximum-cable-length-at-bit-rate)
  - [Checking for duplicate Board ID](#checking-for-duplicate-board-id)
  - [Bus line hardware, protection. pull-up values etc](#bus-line-hardware-protection-pull-up-values-etc)
  - [Current test circuit](#current-test-circuit)

## Design

### The message frame

```fixed width text
|SF|R|lll|mmmmmmmm|ddddddd16ddddddd|CCCCCCCC|D|A|D|A|DD|eeeeeee|iii
|01|?| 3 | 8,16 or32 bits          |    8   |l|1|1|1|2 |7 high | 3 | number of bits.
|01|?|1??|????????|????????????????|????????|1|?|1|?|10|1111111|111| the bits value.
```

- bits[1 Or more] SOF(start of frame) Bit(s) A pull down pulse to say I am about to start sending. There is a #define for number of bits, to make checking each time through main loop more reliable. If set to more than 1 bit the last bit is high after the pulled low bit(s), to help with timings as if checking in the main loop for example might not know when the pull low started. A controller could also use Pulling the line LOW and then Sending the last HIGH bit to take control of the network.
- bits[1] RTR (Remote Transmission Request).
  - RTR = 0: for date frame. or RTR=1 for: "Remote-Request Frame".
  - We could add a spare bit here but as this is just a software protocol it shouldn't matter much if we change it unlike CAN where a load of hardware chips would no longer work.
- bits[3] Data length in bytes 0=1(bits 8),1=2(16),2=4(32),3=8(64). 2^[0..3] bytes. Extra bit for future expansion
- bits[8,16,32,64] bits. 1, 2, 4 or 8 bytes of message data.
- bits[8] CRC field. For now CRC in only on data bytes. note CAN is 15 bits. Changed to 8 bits from 4
- bits[1]: CRC delimiter. Delimiter is high. Maybe should be inverse of preceding bit?
- bits[1] Ack bit. Seems I had this backwards. Now like CAN this is pulled low by any unit that passes the CRC it indicate at least one unit received it fine. @note: In can errors are indicated by 6 dominant bits to stop the message.
- bits[1] Ack delimiter bit (this high to?)
- bits[1] Ack bit. This is pulled low by any unit that can handle the message i.e. if the message was light switch turned on then this unit will turn on the light.
- bits[1] Ack delimiter bit, need this so the replying unit has some timing leeway
- bits[1] Extra dominant(pulled low) delimiter bit. Needed to use pin change ISR as without this all CRC + Ack bits could be high, therefore there could be no pin change after the message is sent.Also a lot of bit in a row could be high, As the other delimiter bits snd the EOF should be high there should always be pin changes after the message, although should wait for the rest of the frame before sending anymore messages. 
- bits[7] EOF 7 bit end of fame.
- bits[3] Interframe Space. Most(all??) CAN controllers seem to add a delay of 3 bits between sending frames to give the controllers time for housekeeping etc.

#### Also note:

- For the collision detection to work properly and the smallest number to have priority the MSB(most significant bit) needs to be sent first.
- Max at one level is 5 after that 1 bit is added at the opposite level but this
can add to the length of time needed to send a frame.
  - This does not include the Ack, EOF field and 'Interframe Space' at the end of the frame.
  - For more detail see below
- Changed to adding a low bit before the EOF. 
- At max there is 9 high bits in a row if CRC + all Ack are high.
  - Maybe change CRC delimiter to inverse of the last CRC bit. Be back to
/// a Max of 7 then

### Interframe Space

- [x] Added this from Microchip CAN controller?

Below is copied from microchips MCP2515-Family-Data-Sheet-DS20001801K.pdf

>The interframe space separates a preceding frame (of any type) from a subsequent data or remote frame.
The interframe space is composed of at least three recessive bits, called the ‘Intermission’. This allows
nodes time for internal processing before the start of the next message frame. After the intermission, the
bus line remains in the recessive state (Bus Idle) until the next transmission starts.

#### By default the minimum length in bits is

| bits | description|
|:----:|------------|
|2    | bits for start of frame. Pull low and then 1 high bit. The first bit(s) is to give time for MCUs to wake from sleep etc. leading bits can be increased if more time is needed. No bit stuffing on leading bits.|
|1    | bit for RTR (Remote Transmission Request)|
| 3   | for length: 3
| 8+  | for message data: 8 to 128 bits (or 1 to 16 bytes) most messages will be 1 or 2 bytes
| 9   | bits [8+1] for CRC + 1 for delimiter. No bit stuffing after CRC. Uses optimized 1-wire CRC. 1-wire uses a 2x16 array of bytes table of values 
| 2   | bits [1+1] ack, Any unit on line will pull the Ack bit low on receiving Error|
| 2   | bits 1+1 Ack (message handled) This can be used to resend the message later if no unit turned on the light etc. maybe it was busy.
| 1   | bit for a pull low bit before EOF. Added this in case we want a simplified receive just using pin-change int with no Ack and not waiting for end of frame
| 7   | bits for end of frame.
| 3   | for Interframe Space.

  
So 38 bits in total and this is not counting any bit stuffing

2+1+3+8+(8+1)+(1+1)+(1+1)+1+7+3 = 38 but if there are 5 bits of the same value in a row
more will be added.(if you don't care about the EOF and maybe ack would be 23 bits)

For max bit we have the above + any data bits
so:  38 + 32 = 70, or 45 not counting the 7 at end. Can't be bothered to work out
how many of these can be high in a row but 45/5 gives 9 more bits also some
data length codes may increase the id or date length even more.

So minimum time used on the line for one message is 38/488 ≈ 0.077 seconds.
For 32 bits of data could use up to 70+8 pulses or 78/488 ≈ 0.16 seconds.

Maybe we could use 6 bits pulled low to interrupt long low priority messages! As long as error handling is handled nicely it shouldn't even need extra code :D and we should hopefully know the message length at this point.

### Maximum consecutive bits of the same value

1. [x] On a lower level limit the max consecutive bits of the same value sent to have max time of having the line HIGH and LOW to make the timing more forgiving. Should probably use CAN style, add a inverted bit if long sequence(5 for CAN) of high or low bits instead of relying on parity bit.
- [x] Added low bit before EOF to bring this back to 7 max 
     - TODO  If the Ack bits are high the last 4+7=11 bits will be high as no bit stuffing in the EOF 7 bits.
- [x] Decided to remove bit stuffing in the Ack. 
    - The sender will not know the final receiving value of the Ack bits as other units overwrite them to acknowledge the message or signal it didn't receive it correctly.
    - Left bit stuffing in CRC as it is now 8 bits long and no reason we can't

#### Fields that have bit stuffing:

- Last bit of SOF
- RTR 
- Length code
- command
- Data
- CRC

> [!CAUTION]
   **SOF** is 01 by default. Even if the length is increase to more than 5 leading 0(dominant) bits, still no bit stuffing on the leading low bits as the whole point of adding the longer SOF is so other units can still pick up the message late if they are busy. If they start checking when the SOF is already part sent it simplifies things if the first high bit is always the last bit of the SOF.
  
 > [!TIP]
 Last high bit of SOF(1) + RTR(1) + length field(3) =5 bits, so no bits stuffed in here but could count towards bit stuffing in the following fields


#### Fields that do not have bit stuffing:

- all the leading low bits of a multi bit SOF(all but the last 1 bit) [The leading Low bits can vary in length]
- CRC Delimiter
- ACK Field (ACK slot and ACK delimiter)
- second ACK Field (ACK slot and ACK delimiter)
- Low delimiter before EOF
- EOF (7 bits)
- Interframe Space (2 bits)

> [!IMPORTANT]
> any leading LOW bits of a longer than 1 bit *SOF* do not have bit stuffing as the point of longer SOF is so receiving units have more time to check if a message is about to be sent and so will not know how meany bits are already sent.

> [!NOTE]
> CAN has a Max consecutive bits of the same level of 5 bits and anything more is used to set an error. So if one unit gets a CRC error it can pull the line low for 6 bits to cancel the send and set an error thus keeping all units in sync.

- [x] Bit stuffing removed from ack fields.
- [x] Add extra dominant delimiter bit before EOF, to make pin change IRC reading work better.

### CRC Error checking

- Decided to change to 8 bit CRC
- CRC is computed on command and data bytes

> [!WARNING]
  > Some code may still not be updated from 4 to 8 bit CRC. 
  > And now I have used all 5 different Alerts :)

### Timings and Transmission speed

- Using a bit timing length of 2048µs gives a lines speed of approx 488 bit/s for the bandwidth.
- The number of high or low bits can then be calculated with shift left(11 = div 2048) and bitwise AND, no need for MCU div. Could go 2 or 4 time faster but if the MCU is trying to use onewire etc. at the same time I was thinking the slower the better. Want to keep the timing code as fast as possible as some of it needs to be in an ISR.
- At 488 bit/s and with 1 message taking 20 bits min and 59 max message, time is approx 24th of a second min and approx one 8th of a second slowest.
- There are 32,768 instruction cycles in 2048 microseconds on an Arduino Uno running at 16MHz.
  - 2048 µs=2048×10−6 s=0.002048 seconds
  - Number of cycles = Clock speed (cycles/second) × Time (seconds) So
  - Number of cycles = 16,000,000 cycles/s×0.002048 s = 32,768

### Minimal needed to work for controlling lights with switches and temp

- [x] Send a simple command with 0 or 1 byte of data(with out CRC or handling higher priority incoming messages)
- [x] handle the rest of the data lengths. Tested with 0,1,2 bytes of data.
- [x] Implement crc
- [ ]   each unit on the line will pull the Ack bit low on CRC fail
- [x] Add an additional Ack bit for units that can handel a message.
- [ ]   Acknowledgment frame bit set for messages that this unit can deal with.
- [ ] Acknowledgment option by sending back the crc checksum.
- [ ] Maybe add some more of the CAN error checking in the 7 bit end frame.
- [ ] At the moment if you send messages to fast after each other the reviving part messes up.
- - [ ] TODO: Need to add code to check for line free before sending code. This kind of needs [Maximum consecutive bits](#maximum-consecutive-bits-of-the-same-value)
- - [ ] TODO: Maybe speed up receiving code and make sure it receives all the message frames so the receiving function don't return while the message ending part of the frame is still being send for example
- - [ ] TODO Add code to try and make sure we do not start receiving a message in the middle of a frame.

### Read bus
#### Mcu pins used:
- 1 MCU pin
  - Downsides
    - Hard to stop back powering the board from the can line if not all units on the line always turn on(and off) at the same time.
    - The MCU has little protection from line nose etc.
- 2 MCU pins
  - Plus 
    - With a resistor and Zener Should stop back powering the board at lest enough to not drag the line down.
    - Makes level shifting the voltage easier
    - As will use a a transistor for send can use a stronger pullup for the line. (more nodes/ more robust?)
    - Could use a can transceiver chip if you really want to, might as well use can at than point though

#### Different methods to receive the bits sent on the line
- [x] Added an option to increase the start pull-down length so it would be long enough that it would stay low for 1 time through the main loop. then you would not need to use interrupts to read. With the ack bit implemented the sender would resend so would not have to catch if doing more than normal in the main loop. Would also need to add 1 high bit at end of SOF (Start of Frame).
- [x] Some CAN standards check the level of the pulse 87.5 percent along the pulse length, this gives any reflections/ringing time to settle, see: <http://www.bittiming.can-wiki.info/>

##### Not using timers/interrupts
- [ ] Receive only just turning off a relay etc.

- [ ] Receive only plus:
  - Could also send replies when asked, maybe temp for example. If the only accepted message is temp for example it really wouldn't matter if any other messages sent on the line are missed as would just be a repeat anyway.
  - Also could do simple checks while monitoring the line, just checking for switch change for example.
    - As long as the check don't take more time than say 1/2 the time to send a bit on the line
    - As that is over 16,000 one cycle MCU instructions, as long as we are not doing things like writing to the terminal etc. we could do a fair bit.
  - Although if not doing a lot likely fine to use IRC to.

- [ ] Check each time through the main loop.
  - if the main loop takes under 16,000 one cycle machine code instructions of time this should sort of work
  - Or if we rely on any messages being resent if missed
  - Or do not care much if messages are missed.

##### Using timers/interrupts
- [ ] Use ISR on pin change to just store starting time then disable pin interrupts and enable general interrupts and then just call the readMessage() function. 
  - This might play badly with other time critical stuff though.

- [ ] Implement pin change interrupt line reading.
- [ ] Alternative first interrupt sets up a timer. Could even use pin change interrupt to correct timing at guaranteed bit change points.
- [ ] TODO: interrupt version, a way to tun off the intercept when doing time sensitive stuff. Will need at least Ack for this.

#### Use Pin change interrupt to read the message

This it for using the pin change interrupt to keep track of the timings and not the on pin change stay in the ISR until the message is red option.

- [ ] On first change set a var with the current time. maybe use the hardware timer Reg to save time in the ISR.
- Each time through the main loop the receivingMessage and time elapsed since last pin change can be checked and used to reset the vars if needed.
- [ ] On subsequent pin changes store the time elapsed since last change.
- If we run out of buffer space because they are not being removed fast enough handle that.
- As the max number of bits send of the same value should be no more than 8? shifting right and storing as a byte should be enough?
- Or would it be better to store the number of bits and the level in a byte?
- Storing the number of bits rather then converting to the message value works better as 0 can than be use to show the start of the next message.
- We also need to keep the time spent in the ISR to the minimum.
- [ ] Each time through the main loop remove any messages from the buffer.
- [ ] If only using "Pin change interrupt" replying in the Ack fields will be hard. Options:
  1. Don't bother.
  1. while in the ISR in CRC field, stay in until after both ACKs. Could disable "Pin change interrupt" and make sure other interupt are enabled, would likely need to for things like Delay to still work.
  1. tweak the Frame so the sender sends a low pulse long enough to trigger the "Pin change interrupt", this could be the last part(maybe 1/4?) of the previous delimiter bit and/or/both the start of the Ack bit. Or could add another low bit before the Ack. Or just change the delimiter bit to be low. Apparently in CAN if triggers an error if LOW? Not sure why.
- Should only compleat messages be removed from the buffer, or compleat bytes or compleat messages.
- I think I will go with for now having an array for 1 message and moving the message to the array as each field or byte is received.
- so each time through the main loop(or timmer interrupt):
   1. Check if we are receiving a message 0b111
   1. If we are check if the time since last pin change is > max for 8 bits and if so reset vars and maybe rase an error.
   1. Move message bits from buffer to message if there is room and we have enough.
   1. Once we have a full message handle it.

## Can protocol web pages

- <https://www.kvaser.com/can-protocol-tutorial/>
- <https://copperhilltech.com/blog/controller-area-network-can-bus-tutorial-message-frame-format/>

### Cable lengths

<details>
  <summary>Maximum Cable Length</summary>

At a speed of 1 MBit/s, a maximum cable length of about 40 meters (130 ft.) can be used. This is because the arbitration scheme requires that the wave front of the signal be able to propagate to the most remote node and back again before the bit is sampled. In other words, the cable length is restricted by the speed of light. A proposal to increase the speed of light has been considered but was turned down because of its inter-galactic consequences.

### Other maximum cable lengths are (these values are approximate)

- 100 meters (330 ft) at 500 kBit/s
- 200 meters (650 ft) at 250 kBit/s
- 500 meters (1600 ft) at 125 kBit/s
- 6 kilometers (20000 ft) at 10 kBit/s

If opto-couplers are used to provide galvanic isolation, the maximum bus length is decreased accordingly. Hint: use fast opto-couplers, and look at the delay through the device, not at the specified maximum bit rate.

### Maximum cable length at bit rate

|Bit rate|Max cable length|
|---|---:|
|1Mbit/s |25m|
|800Kbit/s  |50m|
|500Kbit/s  |100m|
|250Kbit/s  |250m|
|125Kbit/s  |500m|
|50Kbit/s  |1000m|

</details>

## Checking for duplicate Board ID

- [ ] TODO: option to check if anyone else has the same ID 1 time per boot. this would be done the first time the board receives user input i.e. when a switch is used so it is unlikely for any other boards to check at the same time.
- [ ] TODO: single-wire CAN uses a 9.09k ohm pull-up resistor <details>  <summary>Single wire CAN bus example from chip data sheet</summary>
![image](docs/images/can_network_single_wire_example.jpg)
see https://www.onsemi.com/pdf/datasheet/ncv7356-d.pdf for data sheet.</details>
If we add a resistor to the IO pin to limit the current in case of short to ground(or to v++ if using pull-up) we could then use the voltage on the bus pin to check for unit id duplicating at the time of the first user event.

## Bus line hardware, protection. pull-up values etc

- [ ] TODO: For testing I will use a 220 ohm resistor on the IO pin so any code error will not kill the chips as this should limit the current to 23mA.
- [ ] TODO: for testing the pull-up wil be the MCUs internal pull-up.
- [ ] TODO: The above 2(maybe more) will need to be checked if they create problems with slope speed and voltage check ranges.
- [ ] TODO: Most of the above should be options.
- [ ] TODO: Add a slope shaping circuit something similar to the example from the Onsemi date-sheet above maybe.
- [ ] TODO: Check how robust the internal (20kΩ ??) and 220Ω resistors are.

## Current test circuit

- Line pullup resistor = 1kΩ
- IO pin to line resistor = 28Ω This to  protect the IO pin encase of line short etc.
- [ ] TODO: There is a problem with backfeed trying to power the chip through the IO pin when a unit is turn off(unpowered), when using the ATmega328P(and most other chips). This means if one unit is off it pulls the line low all the time.
  - [ ] TODO: Add/Change the code to have the option to use 2 IO pins, 1 to read to line with a high enough resistor in line so it will not pull the line low when the unit is un-powered. And the other pin can be used when sending messages by pulling the line low through and transistor. N-Channel MOSFET or Opto etc.
  - [ ] TODO: While we are at it change the code so it can go high to pull the line low for an NPN transistor etc.
