
// ============================================================================
//
// i2c.cpp   - I2C library
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "i2c.h"

I2C::I2C() {
}

// Public Methods

void I2C::begin() {
  sbi(PORTC, 4);
  sbi(PORTC, 5);
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / 400000) - 16) / 2;
  TWCR = _BV(TWEN) | _BV(TWEA);
}

void I2C::end() {
  TWCR = 0;
}

void I2C::write(uint8_t address, uint8_t registerAddress, uint8_t data) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  sendByte(data);
  stop();
}

void I2C::write(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(data[i]);
  stop();
}

void I2C::writezeros(uint8_t address, uint8_t registerAddress, uint8_t numberBytes) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  for (uint8_t i = 0; i < numberBytes; i++) sendByte(0);
  stop();
}

uint8_t I2C::read(uint8_t address, uint8_t registerAddress) {
  start();
  sendAddress(SLA_W(address));
  sendByte(registerAddress);
  start();
  sendAddress(SLA_R(address));
  receiveByte();
  stop();
  return(TWDR);
}

// Private Methods

uint8_t I2C::start() {
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START)) {
    return(0);
  }
  if (TWI_STATUS == LOST_ARBTRTN) {
    uint8_t bufferedStatus = TWI_STATUS;
    lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS);
}

uint8_t I2C::sendAddress(uint8_t i2cAddress) {
  TWDR = i2cAddress;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK)) {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK)) {
    stop();
    return(bufferedStatus);
  } else {
    lockUp();
    return(bufferedStatus);
  }
}

uint8_t I2C::sendByte(uint8_t i2cData) {
  TWDR = i2cData;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  if (TWI_STATUS == MT_DATA_ACK) {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (TWI_STATUS == MT_DATA_NACK) {
    stop();
    return(bufferedStatus);
  } else {
    lockUp();
    return(bufferedStatus);
  }
}

uint8_t I2C::receiveByte() {
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  if (TWI_STATUS == LOST_ARBTRTN) {
    uint8_t bufferedStatus = TWI_STATUS;
    lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS);
}

uint8_t I2C::stop() {
  TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  while ((TWCR & (1<<TWSTO)));
  return(0);
}

void I2C::lockUp() {
  TWCR = 0; //releases SDA and SCL lines to high impedance
  TWCR = _BV(TWEN) | _BV(TWEA); //reinitialize TWI
}

I2C I2c = I2C();

