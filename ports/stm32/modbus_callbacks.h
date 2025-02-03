/*
 * modbus_callbacks.h
 *
 *  Created on: Feb 2, 2025
 *      Author: kuba2
 */

#ifndef MODBUS_PORTS_STM32_MODBUS_CALLBACKS_H_
#define MODBUS_PORTS_STM32_MODBUS_CALLBACKS_H_

#include "main.h"
#include "modbus_port.h"

void modbus_error_callback(const char *const error_string);
ModbusError modbus_exception_callback(const ModbusSlave *status,
		uint8_t function, ModbusExceptionCode code);
ModbusError modbus_register_callback(const ModbusSlave *status,
		const ModbusRegisterCallbackArgs *args,
		ModbusRegisterCallbackResult *out);

#endif /* MODBUS_PORTS_STM32_MODBUS_CALLBACKS_H_ */
