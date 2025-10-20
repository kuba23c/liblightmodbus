/*
 * modbus_config.h
 *
 *  Created on: Feb 2, 2025
 *      Author: kuba2
 */

#ifndef MODBUS_PORTS_STM32_MODBUS_CONFIG_H_
#define MODBUS_PORTS_STM32_MODBUS_CONFIG_H_

#include "main.h"

/************* weak functions - for overwrite *******************/

/**
 * @brief called when exception occurred in modbus response
 * @param function - function id
 * @param code - error code
 */
void modbus_exception(uint8_t function, uint8_t code, const char *const name, uint8_t id);

/**
 * @brief Check if register allows reads
 * @param reg_index - register index to check
 * @return 	true - reads are allowed
 * 			false - reads are not allowed
 */
bool modbus_is_reg_read(uint16_t reg_index);

/**
 * @brief Check if register allows writes
 * @param reg_index - register index to check
 * @return 	true - writes are allowed
 * 			false - writes are not allowed
 */
bool modbus_is_reg_write(uint16_t reg_index);

/**
 * @brief Read value from register
 * @param reg_index - register index
 * @return register value
 */
uint16_t modbus_reg_read(uint16_t reg_index);

/**
 * @brief Write new value to register
 * @param reg_index - register index
 * @param value - new register value
 */
void modbus_reg_write(uint16_t reg_index, uint16_t value);

#endif /* MODBUS_PORTS_STM32_MODBUS_CONFIG_H_ */
