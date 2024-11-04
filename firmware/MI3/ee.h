
// ============================================================================
//
// ee.h   - A simple EEPROM library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>

#ifndef EE_H
#define EE_H

class EE {
  public:
    EE();
    void begin();
    void end();
    void     put(uint8_t addr, uint8_t data);
    uint8_t  get(uint8_t addr);
    void     put32(uint8_t addr, uint32_t data);
    uint32_t get32(uint8_t addr);
    void     putstr(uint8_t addr, char* s);
    void     getstr(uint8_t addr, char* s);
};

#endif

