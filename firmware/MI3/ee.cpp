
// ============================================================================
//
// ee.cpp   - A simple EEPROM library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "ee.h"

EE::EE() {
}

// Public Methods

void EE::begin() {
  EEAR = 0;
  EEDR = 0;
  EECR = 0;
}

void EE::end() {
}

// write 8-bit value from eeprom
void EE::put(uint8_t addr, uint8_t data) {
  while (EECR & 0x03);
  EEAR = addr;
  EEDR = data;
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
}

// read 8-bit value from eeprom
uint8_t EE::get(uint8_t addr) {
  while (EECR & 0x03);
  EEAR = addr;
  EECR |= (1 << EERE);
  return EEDR;
}

// write 32-bit value from eeprom
void EE::put32(uint8_t addr, uint32_t data) {
  uint8_t x = 0;
  for (uint8_t i=0; i<4; i++) {
    x = (data & 0x000000ff);
    put(addr++, x);
    data = data>>8;
  }
}

// read 32-bit value from eeprom
uint32_t EE::get32(uint8_t addr) {
  uint32_t x = 0;
  addr +=3;
  for (uint8_t i=0; i<4; i++) {
    x |= get(addr--);
    if (i<3) x = x<<8;
  }
  return x;
}

// write callsign to eeprom
void EE::putstr(uint8_t addr, char* s) {
  char ch = ' ';
  uint8_t i = 0;
  while (ch) {
    ch = s[i++];
    put(addr++, ch);
  }
  put(addr, 0);
}

// read callsign from eeprom
void EE::getstr(uint8_t addr, char* s) {
  char ch = ' ';
  uint8_t i = 0;
  while (ch) {
    ch = get(addr++);
    s[i++] = ch;
  }
}

