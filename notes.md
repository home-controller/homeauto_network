# Current working on

## move completed stuff from here to changelog and remove

- [ ] Add dominant ack bit before EOF to the sending and receiving code.
- [ ] get pin change ISR working
- [ ] Check sending function to se if brocken
- [ ] Check blocking receive to see if still working, Blocking receive is probably all a switch unit needs, but I guess for the same reason the ISR wold work fine to
- Would using a timer to check the line level work better than pin change interrupt?
- For now I think I may just ignore everything after the CRC until we get the next message.
- Although it may be better to have 2 versions with a cut down version for if we can't spend time in ISR? should probably find out if we can get it to work with 1-wire & Ethernet modules etc. and how fussy they are about having delays.
- I guess it would also depend on if we have some sort of acknowledgment message. then we could just stop this ISR when using time sensitive stuff. 
- We could also just pull the line low in the case where we have 1 controller and it is busy.
- 1-wire looks like it has a max of 70 Microsecond + 2 reg writes in reset with interrupts disabled.
- So for now make sure to check for errors with sending and resend if not acknowledged same with 1-wire etc.