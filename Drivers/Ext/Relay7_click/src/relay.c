#include "relay.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

uint8_t relay_init(I2C_HandleTypeDef *hi2c) {
    uint8_t config = 0x00; // All pins as outputs
    if (HAL_I2C_Mem_Write(hi2c, RELAY7_I2C_ADDRESS << 1, RELAY7_CMD_CONFIGURATION_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, HAL_MAX_DELAY) != HAL_OK) {
        return RELAY_I2C_ERROR;
    }
    return RELAY_OK;
}