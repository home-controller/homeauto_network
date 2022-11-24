

#include "buf.h"
#include <Arduino.h>

#include <EEPROM.h>


circular_buf::circular_buf(byte size){
    byte a[size];
    length=size;
    current_i=0;
    last_i=0;
    count=0;
}

void circular_buf::push(uint8_t b){
 /** word i = 0;
  find_addr(str_count);//index start at 0 so count will give addrs past end of last string.
  //if(t_addr
  while( (last_pos_addr + i <= max_addr) and (s[i]) != 0 ){
    EEPROM.update(last_pos_addr+i+1, s[i]);
  }
  EEPROM.update(last_pos_addr, i);
  str_count++;
  */
}

uint8_t circular_buf::get(byte i){
  //byte x, l;
  // if(i<= str_count){
  //   find_addr(i);
  //   l = EEPROM.read(last_pos_addr);
  //   s[0] = l;
  //   for(x=1;x<=l; x++){
  //     s[x] = EEPROM.read(last_pos_addr+x);
  //   }
  // } else {
  //   s[0] = 0;
  // }
  ;
}