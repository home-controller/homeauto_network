# Homeauto Network

## This code was copied from the Ethernet relay controller so there might still be some comments etc. referring to that that need to be rewritten or removed. I separated this code as it is needed for more of my Homeauto projects than the Ethernet Relay one

## hn is short fot Home Network here

* Not to be confused with Ethernet.
* This will be kind of like a very cut down and slow CAN network. So there will be no need for network hardware.
*
* Lets go with:

* [x]  1: A pull down pulse to say I am about to start sending.
* 1: 8 bits command id
* 2: 1 bit RTR (Remote Transmission Request)
* 3: 2 bits data length in bytes 0=0,1=1,2=2,3=4
* 4: Then optional 8,16 or 32 bits of data.
* 5: ToDo CRC field. Can is 15 bits but should be lot less here.
* 6: CRC delimiter
* 7: Ack bit
* 8: Ack delimiter bit
* 9: 7 bit end of fame.
* 10: todo On a lower level limit the max consecutive bits of the same value sent to have max time of having the line HIGH and LOW to make the timing more forgiving.

* Using 488 bit/s for the bandwidth. The number of high or low bits can then be calculated with shift left(11 = div 2048) and bitwise AND, no need for MCU div. Could 2 or 4 time faster but if the MCU is trying to use onewire etc. at the same time I was thinking the slower better. Want to keep the timing code as fast as possible as some of it needs to be in an ISR.

* [ ]  5: TODO Maybe do crc
* [ ]  6. Todo: should probably use CAN style, add a inverted bit if long sequence of high or low bits instead of relying on parity bit.
* [ ]   7. Todo Options to have receiving unit(s) use an interrupt or a week pull-down with 1 controller so the controller can check each time through the main loop.
* [ ]   8. Todo: Should have an option for acknowledgment by sending back the crc checksum.
* [ ]   9. TODO: interrupt version away to tun off the intercept when doing time sensitive stuff. Will need at least Ack for this.

## can protocol web pages

* <https://www.kvaser.com/can-protocol-tutorial/>
*

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
