/*
 * modbus_config.c
 *
 *  Created on: Feb 2, 2025
 *      Author: kuba2
 */

#include "modbus_config.h"

__weak void modbus_error(const char *const error_string) {
	UNUSED(error_string);
}

__weak void modbus_exception(uint8_t function, uint8_t code) {
	UNUSED(function);
	UNUSED(code);
}

__weak bool modbus_is_reg_read(uint16_t reg_index) {
	UNUSED(reg_index);
	return (true);
}

__weak bool modbus_is_reg_write(uint16_t reg_index) {
	UNUSED(reg_index);
	return (true);
}

__weak uint16_t modbus_reg_read(uint16_t reg_index) {
	UNUSED(reg_index);
	return (0);
}

__weak void modbus_reg_write(uint16_t reg_index, uint16_t value) {
	UNUSED(reg_index);
	UNUSED(value);
}
