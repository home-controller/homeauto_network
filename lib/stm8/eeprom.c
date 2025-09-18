#include "stm8_regs.h"
#include "eeprom.h"

#define EEPROM_BASE 0x4000

// Unlock EEPROM for write/erase
static void EEPROM_unlock(void) {
    FLASH_DUKR = 0xAE;
    FLASH_DUKR = 0x56;
    while (!(FLASH_IAPSR & (1 << 3))) {
        // wait for DUL (Data EEPROM unlocked)
    }
}

uint8_t EEPROM_read(uint16_t addr) {
    volatile uint8_t *ptr = (uint8_t *)(EEPROM_BASE + addr);
    return *ptr;
}

void EEPROM_update(uint16_t addr, uint8_t data) {
    volatile uint8_t *ptr = (uint8_t *)(EEPROM_BASE + addr);
    if (*ptr != data) {
        EEPROM_unlock();
        *ptr = data;
        while (!(FLASH_IAPSR & (1 << 0))) {
            // wait for EOP (End of programming)
        }
    }
}

void EEPROM_write(uint16_t addr, uint8_t data) {
    EEPROM_update(addr, data); // for safety
}

void EEPROM_write_word(uint16_t addr, uint16_t value) {
    EEPROM_update(addr, (uint8_t)(value & 0xFF));
    EEPROM_update(addr + 1, (uint8_t)(value >> 8));
}

uint16_t EEPROM_read_word(uint16_t addr) {
    uint16_t low  = EEPROM_read(addr);
    uint16_t high = EEPROM_read(addr + 1);
    return (high << 8) | low;
}

void EEPROM_write_block(uint16_t addr, const void *src, uint16_t len) {
    const uint8_t *p = (const uint8_t *)src;
    for (uint16_t i = 0; i < len; i++) {
        EEPROM_update(addr + i, p[i]);
    }
}

void EEPROM_read_block(uint16_t addr, void *dst, uint16_t len) {
    uint8_t *p = (uint8_t *)dst;
    for (uint16_t i = 0; i < len; i++) {
        p[i] = EEPROM_read(addr + i);
    }
}
