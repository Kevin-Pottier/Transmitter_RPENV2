#include "relay.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

relay_status_t relay_init(relay_gpio_expander_t *expander) {
  // Initialize the I2C expander with default configuration with C0, C1, C2 & C3
  // pins as outputs whereas C4, C5, C6 & C7 pins as inputs
  uint8_t config_data = RELAY7_DEFAULT_CONFIG;

  if (HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address,
                        RELAY7_REG_CONFIGURATION, I2C_MEMADD_SIZE_8BIT,
                        &config_data, 1, HAL_MAX_DELAY) != HAL_OK) {    
    return RELAY_I2C_ERROR;
  } // Expander configured

  if (HAL_I2C_Mem_Read(expander->hi2c, expander->i2c_address,
                       RELAY7_REG_CONFIGURATION, I2C_MEMADD_SIZE_8BIT,
                       &config_data, 1, HAL_MAX_DELAY) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  if (config_data != RELAY7_DEFAULT_CONFIG) {
    return RELAY_I2C_ERROR;

  } // Expander configuration read back and verified
 uint8_t all_on = 0x00;
  HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address,
                        RELAY7_REG_OUTPUT_PORT, I2C_MEMADD_SIZE_8BIT, &all_on,
                        1, HAL_MAX_DELAY); // All relays on

  HAL_I2C_Mem_Read(expander->hi2c, expander->i2c_address,
                   RELAY7_REG_OUTPUT_PORT, I2C_MEMADD_SIZE_8BIT,
                   &expander->relay_state, 1, HAL_MAX_DELAY); //State of relays read

  return RELAY_OK;
}

relay_status_t relay_set_relay_on(relay7_t *relay, uint8_t relay_number) {
  if(relay_number < RELAY7_SEL_REL1 ||
     relay_number > RELAY7_SEL_REL4) {
    return RELAY_INVALID_RELAY_NUMBER;
  }
  relay_read_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, &relay->expander.relay_state);

  relay->expander.relay_state = ( relay->expander.relay_state | (1 << (relay_number - 1) ) );
  
  relay_write_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, relay->expander.relay_state);
  
  return RELAY_OK;
}

relay_status_t relay_set_relay_off(relay7_t *relay, uint8_t relay_number) {
  if(relay_number < RELAY7_SEL_REL1 ||
     relay_number > RELAY7_SEL_REL4) {
    return RELAY_INVALID_RELAY_NUMBER;
  }
  relay_read_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, &relay->expander.relay_state);

  relay->expander.relay_state = ( relay->expander.relay_state & ~(1 << (relay_number - 1) ) );
  
  relay_write_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, relay->expander.relay_state);
  
  return RELAY_OK;
}

relay_status_t relay_set_relay_state(relay7_t *relay, relay7_relay_state_t state, uint8_t relay_number) {
  if(state != RELAY7_STATE_OPEN && state != RELAY7_STATE_CLOSE) {
    return RELAY_INVALID_RELAY_NUMBER;
  }

  if(relay_number < RELAY7_SEL_REL1 ||
     relay_number > RELAY7_SEL_REL4) {
    return RELAY_INVALID_RELAY_NUMBER;
  }
  relay_read_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, &relay->expander.relay_state);
  if(state == RELAY7_STATE_CLOSE) {
    relay_set_relay_off(relay, relay_number);
  } else {
    relay_set_relay_on(relay, relay_number);
  }
  return RELAY_OK;
}

relay_status_t relay_set_all(relay7_t *relay, relay7_relay_state_t state) {
  if(state != RELAY7_STATE_OPEN && state != RELAY7_STATE_CLOSE) {
    return RELAY_INVALID_RELAY_NUMBER;
  }

  relay_read_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, &relay->expander.relay_state);
  if(state == RELAY7_STATE_CLOSE) {
    relay->expander.relay_state = 0x00; // All relays closed
  } else {
    relay->expander.relay_state = 0x0F; // All relays open
  }
  relay_write_register(&relay->expander, RELAY7_REG_OUTPUT_PORT, relay->expander.relay_state);
  return RELAY_OK;
}

relay_status_t relay_write_register(relay_gpio_expander_t *expander,
                                    uint8_t reg, uint8_t value) {

  if (HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address, reg, I2C_MEMADD_SIZE_8BIT,
                        &value, 1, HAL_MAX_DELAY) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }

  if(reg == RELAY7_REG_OUTPUT_PORT) {
    expander->relay_state = value; // Update the local state if output register is written
  }

  return RELAY_OK;
}

relay_status_t relay_read_register(relay_gpio_expander_t *expander, uint8_t reg,
                                   uint8_t *value) {
  if(HAL_I2C_Mem_Read(expander->hi2c, expander->i2c_address, reg, I2C_MEMADD_SIZE_8BIT,
                      value, 1, HAL_MAX_DELAY) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  return RELAY_OK;
}

relay_status_t relay_test_i2c_connection(relay_gpio_expander_t *expander) {
  uint8_t cfg = 0x00; // P7..P4 en sortie, P3..P0 en entrée
  HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address,
                    RELAY7_REG_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &cfg, 1,
                    HAL_MAX_DELAY);

  HAL_Delay(10);

  uint8_t out = 0xFF; // activer P7..P4
  HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address,
                    RELAY7_REG_OUTPUT_PORT, I2C_MEMADD_SIZE_8BIT, &out, 1,
                    HAL_MAX_DELAY);

  HAL_Delay(10);

  uint8_t check;
  HAL_I2C_Mem_Read(expander->hi2c, expander->i2c_address,
                   RELAY7_REG_OUTPUT_PORT, I2C_MEMADD_SIZE_8BIT, &check, 1,
                   HAL_MAX_DELAY);
  // check devrait renvoyer 0xF0

  if (check != out) {
    return RELAY_I2C_ERROR;
  }
  return RELAY_OK;
}