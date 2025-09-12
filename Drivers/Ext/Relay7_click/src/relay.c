#include "relay.h"
#include "stm32l4xx_hal_i2c.h"
#include <stdint.h>

relay_status_t relay_init(relay_gpio_expander_t *expander) {
  // Initialize the I2C expander with default configuration with C0, C1, C2 & C3
  // pins as outputs whereas C4, C5, C6 & C7 pins as inputs
  uint8_t config_data = RELAY7_DEFAULT_CONFIG;

  if (relay_write_register(expander, RELAY7_REG_CONFIGURATION, config_data) !=
      RELAY_OK) {
    return RELAY_I2C_ERROR;
  } // Expander configured

  if (relay_read_register(expander, RELAY7_REG_INPUT_PORT, &config_data) !=
      RELAY_OK) {
    return RELAY_I2C_ERROR;
  }
  if (config_data != RELAY7_DEFAULT_CONFIG) {
    return RELAY_I2C_ERROR;
  } // Expander configuration read back and verified

  return RELAY_OK;
}

relay_status_t relay_set_relay_state(relay7_t *relay,
                                     relay7_relay_state_t state) {

  if (relay->relay_number < 1 || relay->relay_number > 4) {
    return RELAY_INVALID_RELAY_NUMBER;
  }

  // Write the updated state back to the output port
  switch (relay->relay_number) {
  case 1:
    state = (state == RELAY7_STATE_CLOSE)
                ? (relay->relay_state | RELAY7_PIN_MASK_P0)
                : (relay->relay_state & ~RELAY7_PIN_MASK_P0);
    break;
  case 2:
    state = (state == RELAY7_STATE_CLOSE)
                ? (relay->relay_state | RELAY7_PIN_MASK_P1)
                : (relay->relay_state & ~RELAY7_PIN_MASK_P1);
    break;
  case 3:
    state = (state == RELAY7_STATE_CLOSE)
                ? (relay->relay_state | RELAY7_PIN_MASK_P2)
                : (relay->relay_state & ~RELAY7_PIN_MASK_P2);
    break;
  case 4:
    state = (state == RELAY7_STATE_CLOSE)
                ? (relay->relay_state | RELAY7_PIN_MASK_P3)
                : (relay->relay_state & ~RELAY7_PIN_MASK_P3);
    break;
  default:
    return RELAY_INVALID_RELAY_NUMBER;
  }
  // Use HAL_I2C_Mem_Write to write the state to the output port register

  HAL_I2C_Mem_Write(relay->expander.hi2c, relay->expander.i2c_address,
                    RELAY7_REG_OUTPUT_PORT, 1, (uint8_t *)&state, 1,
                    HAL_MAX_DELAY);

  relay->relay_state = state;

  return RELAY_OK;
}

relay_status_t relay_write_register(relay_gpio_expander_t *expander,
                                    uint8_t reg, uint8_t value) {

  if (HAL_I2C_Master_Seq_Transmit_IT(expander->hi2c, expander->i2c_address,
                                     &expander->i2c_address, 1,
                                     I2C_FIRST_FRAME) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }

  if (HAL_I2C_Master_Seq_Transmit_IT(expander->hi2c, expander->i2c_address,
                                     &reg, 1, I2C_NEXT_FRAME) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  if (HAL_I2C_Master_Seq_Transmit_IT(expander->hi2c, expander->i2c_address,
                                     &value, 1, I2C_LAST_FRAME) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  return RELAY_OK;
}

relay_status_t relay_read_register(relay_gpio_expander_t *expander, uint8_t reg,
                                   uint8_t *value) {
  if (HAL_I2C_Master_Transmit(expander->hi2c, expander->i2c_address, &reg, 1,
                              HAL_MAX_DELAY) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  if (HAL_I2C_Master_Receive(expander->hi2c, expander->i2c_address, value, 1,
                             HAL_MAX_DELAY) != HAL_OK) {
    return RELAY_I2C_ERROR;
  }
  return RELAY_OK;
}

relay_status_t relay_test_i2c_connection(relay_gpio_expander_t *expander) {
  uint8_t cfg = 0x0F; // P7..P4 en sortie, P3..P0 en entrée
  HAL_I2C_Mem_Write(expander->hi2c, expander->i2c_address,
                    RELAY7_REG_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &cfg, 1,
                    HAL_MAX_DELAY);

  HAL_Delay(10);

  uint8_t out = 0xF0; // activer P7..P4
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