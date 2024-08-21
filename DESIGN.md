# Design document

## Design

### Lets go with

1. For the collision detection to work properly and the smallest number to have priority the MSB(most significant bit) needs to be sent first.

2. bits[1 Or more] SOF(start of frame) Bit(s) A pull down pulse to say I am about to start sending. There is a #define for number of bits, to make checking each time through main loop more reliable. If set to more than 1 bit the last bit is high after the pulled low bit(s), to help with timings as if checking in the main loop for example might not know when the pull low started.
3. bits[1] RTR (Remote Transmission Request).
    1. RTR = 0: for date frame. or RTR=1 for: "Remote-Request Frame".
    2. We could add a spare bit here but as this is just a software protocol it shouldn't matter much if we change it unlike CAN where a load of hardware IC would not longer work.
4. bits[3] Data length in bytes 0=0,1=1,2=2,3=4. Extra bit for future expansion
5. bits[8] command id. Maybe this should be moved down 2 rows?
6. bits[0,8,16,32] bits, Then optional 8,16 or 32 bits of data.
7. bits[4] CRC field. For now CRC in only on command and data bytes. note CAN is 15 bits. we are using 4 bits for now
8. bits[1]: CRC delimiter. Delimiter is high.
9. bits[1] Ack bit. Like CAN this is pulled low by any unit that fails with the CRC it indicate a line error, even if the unit interested receives it fine.
10. bits[1] Ack delimiter bit (this high to?)
11. bits[1] Ack bit. This is pulled low by any unit that can handle the message i.e. if the message was light switch turned on then this unit will turn on the light.
12. bits[1] Ack delimiter bit, need this so the replying unit has some timing leeway
13. bits[7] : 10: 7 bit end of fame.

```fixed width text
|SF|R|ccc|mmmmmmmm|ddddddd16ddddddd|CCCC|D|A|D|eeeeeee|
|01|?| 3 | 8bits  |0,8,16 or32 bits| 4  |l|1|1|7 high | number of bits.
|01|?|1??|????????|????????????????|????|1|?|1|1111111| the bits value.

Max at one level is 5 after that 1 bit is added at the opposite level but this can add to the length of time needed to send a frame. 

By default the minimum bit length is:
2 for start of frame, 3 for length, 8 for message id, 0 data, 4+1 CRC, 1+1 ack & 7 end of frame. Also plan to add 2 more for message handled.
so:
2+3+8+(4+1)+(1+1)+7 = 27 but if there are 5 bits of the same value in a row more will be added.(if you don't care about the EOF and maybe ack would be 18-20bits)

for max bit we have the above + any data bits
so:  27+ = 49, or 42 not counting the 7 at end.

So minimum number of bits for a message is 20 with no date and not waiting for end of frame.
```

### Maximum consecutive bits of the same value

* [ ] Todo On a lower level limit the max consecutive bits of the same value sent to have max time of having the line HIGH and LOW to make the timing more forgiving. Should probably use CAN style, add a inverted bit if long sequence(5 for CAN) of high or low bits instead of relying on parity bit.

* CAN has a Max consecutive bits of the same level of 5 bits and anything more is used to set an error. So if one unit gets a CRC error it can pull the line low for 6 bits to cancel the send and set an error thus keeping all units in sync.
* Using a bit timing length of 2048µs gives a lines speed of approx 488 bit/s for the bandwidth.
* The number of high or low bits can then be calculated with shift left(11 = div 2048) and bitwise AND, no need for MCU div. Could go 2 or 4 time faster but if the MCU is trying to use onewire etc. at the same time I was thinking the slower the better. Want to keep the timing code as fast as possible as some of it needs to be in an ISR.
* At 488 bit/s and with 1 message taking 20 bits min and 59 max message, tine is approx 24th of a second min and approx one 8th of a second slowest.

### Minimal needed to work for controlling lights with switches and temp.

* [x] Send a simple command with 0 or 1 byte of data(with out CRC or handling higher priority incoming messages)
* [x] handle the rest of the data lengths. Tested with 0,1,2 bytes of data.
* [x] Implement crc
* [ ]   each unit on the line will pull the Ack bit low on CRC fail
* [ ] Add an additional Ack bit for units that can handel a message.
* [ ]   Acknowledgment frame bit set for messages that this unit can deal with.
* [ ] Acknowledgment option by sending back the crc checksum.
* [ ] Maybe add some more of the CAN error checking in the 7 bit end frame.
* [ ] At the min if you send messages to fast after each other the reviving part messes up.
* * [ ] TODO: Need to add code to check for line free before sending code. This kind of needs [Maximum consecutive bits](#maximum-consecutive-bits-of-the-same-value)
* * [ ] TODO: Maybe speed up receiving code and make sure it receives all the message frames so the receiving function don't return while the message ending part of the frame is still being send for example
* * [ ] TODO Add code to try and make sure we don not start receiving a message in the middle of a frame.

### Read bus

* [ ] Read option with ISR on pin change to just store starting time then disable pin interrupts and enable general interrupts and then just call the readMessage() function. This might play badly with other time critical stuff though.
* [ ] Options to have receiving unit(s) use extra wire with interrupt or a week pull-down with 1 controller so the controller can check each time through the main loop.
* [x] Maybe have an option to increase the start pull-down length so it would be long enough that it would stay low for 1 time through the main loop. then you would not need to use interrupts to read. With the ack bit implemented the sender would resend so would not have to catch if doing more than normal in the main loop. Would also need to add 1 high bit at end of SOF (Start of Frame).
* [ ] Implement pin change interrupt line reading.
* [ ] Interrupt to start then continue with timing subsequent pin changes?
* [ ] Alternative first interrupt sets up a timer. Could even use pin change interrupt to correct timing at guaranteed bit change points.
* [ ] TODO: interrupt version, a way to tun off the intercept when doing time sensitive stuff. Will need at least Ack for this.
* [ ] TODO: Some can standards check the level of the pulse 87.5 percent along the pulse length, this gives any reflections/ringing time to settle, see: <http://www.bittiming.can-wiki.info/>

## Can protocol web pages

* <https://www.kvaser.com/can-protocol-tutorial/>
* <https://copperhilltech.com/blog/controller-area-network-can-bus-tutorial-message-frame-format/>

<details>
  <summary>Maximum Cable Length</summary>

At a speed of 1 MBit/s, a maximum cable length of about 40 meters (130 ft.) can be used. This is because the arbitration scheme requires that the wave front of the signal be able to propagate to the most remote node and back again before the bit is sampled. In other words, the cable length is restricted by the speed of light. A proposal to increase the speed of light has been considered but was turned down because of its inter-galactic consequences.

### Other maximum cable lengths are (these values are approximate)

* 100 meters (330 ft) at 500 kBit/s
* 200 meters (650 ft) at 250 kBit/s
* 500 meters (1600 ft) at 125 kBit/s
* 6 kilometers (20000 ft) at 10 kBit/s

If opto-couplers are used to provide galvanic isolation, the maximum bus length is decreased accordingly. Hint: use fast opto-couplers, and look at the delay through the device, not at the specified maximum bit rate.
</details>

## Checking for duplicate Board ID

* [ ] TODO: option to check if anyone else has the same ID 1 time per boot. this would be done the first time the board receives user input i.e. when a switch is used so it is unlikely for any other boards to check at the same time.
* [ ] TODO: single-wire CAN uses a 9.09k ohm pull-up resistor <details>  <summary>Single wire CAN bus example from chip data sheet</summary>
![image](docs/images/can_network_single_wire_example.jpg)
see https://www.onsemi.com/pdf/datasheet/ncv7356-d.pdf for data sheet.</details>
If we add a resistor to the IO pin to limit the current in case of short to ground(or to v++ if using pull-up) we could then use the voltage on the bus pin to check for unit id duplicating at the time of the first user event.

## Bus line hardware, protection. pull-up values etc

* [ ] TODO: For testing I will use a 220 ohm resistor on the IO pin so any code error will not kill the chips as this should limit the current to 23mA.
* [ ] TODO: for testing the pull-up wil be the MCUs internal pull-up.
* [ ] TODO: The above 2(maybe more) will need to be checked if they create problems with slope speed and voltage check ranges.
* [ ] TODO: Most of the above should be options.
* [ ] TODO: Add a slope shaping circuit something similar to the example from the Onsemi date-sheet above maybe.
* [ ] TODO: Check how robust the internal (20kΩ ??) and 220Ω resistors are.

## Current test circuit

* Line pullup resistor = 1kΩ
* IO pin to line protection resistor = 28Ω
* [ ] TODO: There is a problem with backfeed trying to power the chip through the IO pin when a unit is turn off(unpowered), when using the ATmega328P(and most other chips). This means if one unit is off it pulls the line low all the time.
  * [ ] TODO: Add/Change the code to have the option to use 2 IO pins, 1 to read to line with a high enough resistor in line so it will not pull the line low when the unit is un-powered. And the other pin can be used when sending messages by pulling the line low through and transistor. N-Channel MOSFET or Opto etc.
  * [ ] TODO: While we are at it change the code so it can go high to pull the line low for an NPN transistor etc.
