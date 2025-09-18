#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

// Single-byte access
uint8_t EEPROM_read(uint16_t addr);
void    EEPROM_write(uint16_t addr, uint8_t data);   // calls update internally
void    EEPROM_update(uint16_t addr, uint8_t data);

// Multi-byte access
void    EEPROM_write_word(uint16_t addr, uint16_t value);
uint16_t EEPROM_read_word(uint16_t addr);

void    EEPROM_write_block(uint16_t addr, const void *src, uint16_t len);
void    EEPROM_read_block(uint16_t addr, void *dst, uint16_t len);

#endif
