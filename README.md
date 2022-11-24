# Homeauto Network.
### This code was copied from the Ethernet relay controller so there might still be some comments etc. referring to that that need to be rewritten or removed. I separated this code as it is needed for more of my Homeauto projects that the Ethernet Relay one.

## hn is short fot Home Network here. 
 * Not to be confused with Ethernet.
 * This will be kind of like a very cut down and slow CAN network. So there will be no need for network hardware.
 * 
 * Lets go with:
 * 1: A pull down pulse to say I am about to start sending.
 * 2: Then 8 bits of data
 * 3: Then a parity. Also makes it so each 9 bits + parity(10 bits total) sent have to have at least 1 each of having the line HIGH and LOW.
 * 4: For now use extra bit for expecting acknowledgement back, OK RESEND etc.
 * 5: - [x] TODO Maybe do checksums
  6. Todo: should probably use CAN style, add a inverted bit if long sequence of high or low bits instead of relying on parity bit.
  7. Todo Options to have receiving unit(s) use an interrupt or a week pull-down with 1 controller so the controller can check each time through the main loop.
  8. Todo: Should have an option for acknowledgment by sending back the crc checksum.
  9. TODO: interrupt version away to tun off the intercept when doing time sensitive stuff. Will need at least Ack for this.



