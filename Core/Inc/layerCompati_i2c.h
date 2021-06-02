#ifndef __LAYERCOMPATI_I2C_H__
#define __LAYERCOMPATI_I2C_H__

#include "main.h"

int i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t I2C_ReadOneByte(uint8_t I2C_Addr, uint8_t addr);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t num, uint8_t* data);

#endif
