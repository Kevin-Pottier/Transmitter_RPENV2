#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>



enum  {
    RELAY_OK = 0,
    RELAY_ERROR = 1,
    RELAY_I2C_ERROR = 2,
    RELAY_INVALID_RELAY_NUMBER = 3,
    RELAY_INVALID_STATE = 4
} relay_error_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t state; // Each bit represents the state of a relay (1 = ON, 0 = OFF)
} relay_t;

#define RELAY7_I2C_ADDRESS (0x112) 

/** Relay 7 Command Registers */
#define RELAY7_CMD_INPUT_PORT_REG 0x00
#define RELAY7_CMD_OUTPUT_PORT_REG 0x01
#define RELAY7_CMD_POLARITY_INVERSION_REG 0x02
#define RELAY7_CMD_CONFIGURATION_REG 0x03


/** Relays are set on port 0, 1, 2 and 3 of the GPIO expander */
#define RELAY_1 0x0
#define RELAY_2 0x1
#define RELAY_3 0x2
#define RELAY_4 0x3


uint8_t relay_init(I2C_HandleTypeDef *hi2c);
uint8_t relay_set_state(relay_t *relay);
uint8_t relay_get_state(relay_t *relay);
uint8_t relay_toggle(relay_t *relay);
uint8_t relay_set_all(uint8_t value);
uint8_t relay_get_all(void);

#endif