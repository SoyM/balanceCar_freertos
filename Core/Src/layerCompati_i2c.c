#include "layerCompati_i2c.h"

int i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    return HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10) != HAL_OK ? 1 : 0;
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10) != HAL_OK ? 1 : 0;
}

uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &b, 1, 10);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &b, 1, 10);
}

uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b;
    if (HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &b, 1, 10) != 0)
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &b, 1, 10);
    }
    else
    {
        return 1;
    }
}

uint8_t I2C_ReadOneByte(uint8_t I2C_Addr, uint8_t addr)
{
    uint8_t b;
    HAL_I2C_Mem_Read(&hi2c1, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &b, 1, 10);
    return b;
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t num, uint8_t* data)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, num, 10);
}
