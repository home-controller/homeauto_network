# Homeauto Network

## Still in Afa, unfinished code

This is a slow send/receive network(no master/slave) with collision detection and handling. A bit like CAN but way slower and cut-down and no need for extra hardware.

Collision detection works like 1-wire and CAN with the smallest number having priority and not even knowing there was a collision, the other unit will switch to reading mode and read the message. It can then try to send the message at a later time. For now any unit can Ack a message it can deal with. But maybe this should be changed to Ack if crc is checked and then send another message if the command it dealt with.

The idea is for a wired basic and slow network so you do not have to worry to much about end reflection and having different HIGH and LOW states at different points on the line/wire. Being slow should also help with any timing problems and be more tolerant of other stuff like web pages or mqtt hogging the processor and/or interrupts

## hn is short fot Home Network here

* Not to be confused with Ethernet.
* This will be kind of like a very cut down and slow CAN network. So there will be no need for network hardware.

### Lets go with

1. bits[1 Maybe more] A pull down pulse to say I am about to start sending.

2. bits[8] command id
3. bit3[1] RTR (Remote Transmission Request). RTR = 0: for date frame 4. or RTR=1 for: "Remote-Request Frame"
4. bits[2] data length in bytes 0=0,1=1,2=2,3=4
5. bits[0,8,16,32] bits Then optional 8,16 or 32 bits of data.
6. bits[?] CRC field. CAN is 15 bits but should be lot less here.
7. bits[1]: CRC delimiter
8. bits[1] Ack bit
9. bits[1] Ack delimiter bit
10. bits[7] : 10: 7 bit end of fame.

|s||||||||||||||||||||||||

[ ] : todo On a lower level limit the max consecutive bits of the same value sent to have max time of having the line HIGH and LOW to make the timing more forgiving.

* Using 488 bit/s for the bandwidth. The number of high or low bits can then be calculated with shift left(11 = div 2048) and bitwise AND, no need for MCU div. Could 2 or 4 time faster but if the MCU is trying to use onewire etc. at the same time I was thinking the slower better. Want to keep the timing code as fast as possible as some of it needs to be in an ISR.

* [ ] Send a simple command with 0 or 1 byte of data(with out CRC or handling higher priority incoming messages)
* [ ] Implement crc
* [ ] handle the rest of the data lengths.
* [ ] Should probably use CAN style, add a inverted bit if long sequence of high or low bits instead of relying on parity bit.
* [ ] Acknowledgment frame bit set for messages that this unit can deal with.
* [ ] Acknowledgment option by sending back the crc checksum.
* [ ] Maybe add some more of the CAN error checking in the 7 bit end frame.

### Read bus

* [ ] Read option with ISR on pin change to just store starting time then disable pin interrupts and enable general interrupts and then just call the readMessage() function. This might play badly with other time critical stuff though.
* [ ] Options to have receiving unit(s) use extra wire with interrupt or a week pull-down with 1 controller so the controller can check each time through the main loop.
* [ ] Maybe have an option to increase the start pull-down length so it would be long enough that it would stay low for 1 time through the main loop. then you would not need to use interrupts to read. With the ack bit implemented the sender would resend so would not have to catch if doing more than normal in the main loop. Would also need to add 1 high bit at end of SOF (Start of Frame).
* [ ] Implement pin change interrupt line reading.
* [ ] Interrupt to start then continue with timing subsequent pin changes
* [ ] Alternative first interrupt sets up a timer. Could even use pin change interrupt to correct timing at guaranteed bit change points.
* [ ] TODO: interrupt version away to tun off the intercept when doing time sensitive stuff. Will need at least Ack for this.

## can protocol web pages

* <https://www.kvaser.com/can-protocol-tutorial/>
* <https://copperhilltech.com/blog/controller-area-network-can-bus-tutorial-message-frame-format/>

<details>
  <summary>Maximum Cable Length</summary>

At a speed of 1 Mbit/s, a maximum cable length of about 40 meters (130 ft.) can be used. This is because the arbitration scheme requires that the wave front of the signal be able to propagate to the most remote node and back again before the bit is sampled. In other words, the cable length is restricted by the speed of light. A proposal to increase the speed of light has been considered but was turned down because of its inter-galactic consequences.

Other maximum cable lengths are (these values are approximate):

    100 meters (330 ft) at 500 kbit/s
    200 meters (650 ft) at 250 kbit/s
    500 meters (1600 ft) at 125 kbit/s
    6 kilometres (20000 ft) at 10 kbit/s

If optocouplers are used to provide galvanic isolation, the maximum bus length is decreased accordingly. Hint: use fast optocouplers, and look at the delay through the device, not at the specified maximum bit rate.
</details>
