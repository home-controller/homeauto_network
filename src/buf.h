// used buf as is short. buffer to store bytes in memory
#ifndef _buf_h
#define _buf_h
#include <Arduino.h>


class circular_buf {//store strings in mcu RAM
  public:
    circular_buf(byte size);
    void push(uint8_t b);
    uint8_t pull();
    uint8_t peek();
    void set(byte i, uint8_t b);
    uint8_t get(byte i);
  private:
    byte a[16];
    byte length;
    byte current_i;
    word last_i;
    byte count;
};



#endif
