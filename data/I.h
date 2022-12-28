 /**
 * @file I.h
 * @author jmnc2 (you@domain.com)
 * @brief handles local input MCU pins.(connected to a switch.)
 * @version 0.1
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022
 *
 * example use:
 * at top of main.h or .ino etc. add
 *   1: #define pinIO_no_of_switches 5 //setup the number of gpio's used
 *   2: #define pinIO_inPins A7,A6,A0,A1,A2 in sa main.h
 *   3: byte pinIO_switchState[pinIO_no_of_switches]; byte pinIO_pinsA_in[pinIO_no_of_switches];
 * Then in main loop.
 *   1: pinIO_SwitchesExe(); needs to be called in the main loop, hopefully at least every 1/4 second or may mess up switch flick count.
 *
 */
#ifndef _in_h
#define _in_h
/*
 * Switches connected directly to a pin
 */

#include <Arduino.h>

// TODO: move setup vars to extern and define in main.c or s.c  ?
//              0  1  2  3  4
// #define inPins A7,A6,A0,A1,A2
// #define no_of_switches 5
#define in_initHLa = B00000011 //{HIGH,HIGH}// the value to use to for on.

/*
 * simple count meaning:
 * 1: tog
 * 2, 3: all hard on/off
 */

/** Check for changes no more than 1/8 of a second, If sending mqtt or web page may be longer?. Should be good for debounce?
 * If same switch changes again in less than 2 seconds add one to switch state to max of 8.
 * switchState[i]  : 0b0000 0000
 *  state          : 0b0000 0001  Fully updated state, including updated mqtt etc. 0 = off 1 = on
 *  current state  : 0b0000 0010  State at last check. Current switch state but count may still be updating etc. 0 = off 1 = on
 *  change count   : 0b0000 1100  Number of changes within 0.1 and 2 seconds between changes. max changes 8, maybe 7. Use difference of bit 1 and 2 as first bit
 *  time         : 0b1111 0000 time in 1/8 seconds since last change. Within approx 1/8 second
 **/
extern byte pinIO_Max_switches;
extern byte pinIO_switchState[]; // 0 = off, 1 = on

extern byte pinIO_pinsA_in[];
// extern byte switchOnVal;// eg 0 for pull the relay LOW to turn on, 1 for pull high. Pull opposite to turn off. Or should it be set to input to turn off?

// extern byte switchInfo[no_of_switches][3];

// extern byte groupsSateA[];

void SetUpInputs();
boolean checkInput();
void SwitchesExe();
void groupSet(byte i, byte relays9_16, byte relays1_8, byte opts = 0);
void setSwitchGroups(byte switchN, byte quick_i, byte normal_i, byte count2_i, byte count3_i, byte count4_i);

// callback func.
/**
 * @brief Callback func for when a local IO input/switch pin has changed.
 * 
 * @param ioType the IO type: ioLocalPin or ioLocalPinVal defied in defs.h
 * @param i index of pin in pinIO_pinsA_in[i]
 * @param count switch flick count.
 * @param state new state of the switch IO pin.
 * @param quick true if still counting flicks
 */
extern void gotInputPin(byte ioType, byte i, byte count, byte state);
void debugSwitch(byte i);

#endif
