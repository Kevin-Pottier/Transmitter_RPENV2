#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

/**
 * @brief Relay 7 description register.
 * @details Specified register for description of Relay 7 Click driver.
 */
#define RELAY7_REG_INPUT_PORT            0x00
#define RELAY7_REG_OUTPUT_PORT           0x01
#define RELAY7_REG_POLARITY_INVERSION    0x02
#define RELAY7_REG_CONFIGURATION         0x03

/**
 * @brief Relay 7 port expander default configuration data.
 * @details Specified default configuration data of Relay 7 Click driver.
 */
#define RELAY7_DEFAULT_CONFIG            0xF0

/**
 * @brief Relay 7 pin mask data values.
 * @details Specified pin mask data values of Relay 7 Click driver.
 */
#define RELAY7_PIN_MASK_NONE             0x00
#define RELAY7_PIN_MASK_P0               0x01
#define RELAY7_PIN_MASK_P1               0x02
#define RELAY7_PIN_MASK_P2               0x04
#define RELAY7_PIN_MASK_P3               0x08
#define RELAY7_ALL_PIN                   0x0F

/**
 * @brief Relay 7 pin logic state setting.
 * @details Specified pin logic state setting of Relay 7 Click driver.
 */
#define RELAY7_PIN_STATE_LOW             0x00
#define RELAY7_PIN_STATE_HIGH            0x01

/**
 * @brief Relay 7 pin bitmask data values.
 * @details Specified pin bitmask data values of Relay 7 Click driver.
 */
#define RELAY7_SEL_REL1                  1
#define RELAY7_SEL_REL2                  2
#define RELAY7_SEL_REL3                  3
#define RELAY7_SEL_REL4                  4

/**
 * @brief Relay 7 device address setting.
 * @details Specified setting for device slave address selection of
 * Relay 7 Click driver.
 */
#define RELAY7_DEVICE_ADDRESS            (0x70<<1)

typedef enum {
    RELAY_OK = 0,
    RELAY_I2C_ERROR,
    RELAY_INVALID_RELAY_NUMBER,
} relay_status_t;

typedef enum
{
    RELAY7_STATE_OPEN = 0,
    RELAY7_STATE_CLOSE = 1

} relay7_relay_state_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_address;
    uint8_t relay_state;  // Bitmask to hold the state of the relays
} relay_gpio_expander_t;

typedef struct {
    relay_gpio_expander_t expander;
} relay7_t;

relay_status_t relay_init(relay_gpio_expander_t *expander);
relay_status_t relay_test_i2c_connection(relay_gpio_expander_t *expander);
relay_status_t relay_set_relay_on(relay7_t *relay, uint8_t relay_number);
relay_status_t relay_set_relay_off(relay7_t *relay, uint8_t relay_number);
relay_status_t relay_set_relay_state(relay7_t *relay, relay7_relay_state_t state, uint8_t relay_number);
relay_status_t relay_set_all(relay7_t *relay, relay7_relay_state_t state);
relay_status_t relay_write_register(relay_gpio_expander_t *expander, uint8_t reg, uint8_t value);
relay_status_t relay_read_register(relay_gpio_expander_t *expander, uint8_t reg, uint8_t *value);

#endif // _RELAY_H_