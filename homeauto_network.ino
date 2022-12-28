#include <I.h>

//#define _pString_debug
/*
  Web Server / MQTT client to control relays.

Most everything moved to main.cpp
So works with platformio as well

 
IO.h = Low level IO to things like i2c expanders, the arduino's pins & shift registers etc.
       there should be 1 function each for digital read, write & mode that calls pinMode(), digitalWrite etc. or the 
       appropriate functions for i2c etc that works with all the IO based on a byte pin value
       uses only arduino libs.

O.h = Higher level  output. e.g. output: relays for lights
       1: Keeps track of relay state and how to turn a relay on/off etc.
       2: Sends an mqtt message if a relay is updated
       uses: mqtt

I.h and O.h = Higher level input and output. e.g. input: switches, e.g. output: relays for lights
       1: Keeps track of current state. On, off etc.
       2: make sure any change in state is not checked again for a bit to debounce switches etc.
       3: keep track of quick switches, slower than debounce but faster than a second or 2 for extra functionality. 
          Turning all light on for example.
       4: handle keeping track of what the switch will do when switched.
       5: group handling.


html.h =
       Server for the web page on port 80.
       This can turn the output ie. relays on and off and shows their curent settings.
       Should also show the state of switches etc.
          uses:
            O.h for turning the relays on and off.

mqtt.h: 
       send and receive mqtt messages.
       uses 
          calls functions from O.h to control relays etc.
          receives calls from I.h to send messages to the mqtt server.
              this will include any updates from html.h

buf.h:
      Buffer needed the slow custom network. Not finished.

defs.h
      board defs, things like pin No. their usage, amount of memory of each type etc.
      Depends:
        Only #defines so only needs the standard automaticaly included defines.
        
hn.h:
       slow Home Network. For exa,ple so MCU's behind a wall switch can send information and commands back to other units.
       Should work similar to CAN, so easy to replace with CAN network modules.
       Work in progress.

pStr.h
     String handling, strings similar to pascal with the string length as the first byte.
     Also manage/define any strings than might be needed in more than 1 unit/file/lib
      Depends:
        <Arduino.h>
        defs.h


s.h
    Some of the setup moved from main to keep main short and tidy.
 
 */
