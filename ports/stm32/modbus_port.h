/**
 * file: modbus_port.h
 * Author: kuba23c
 * Description: Lightmodbus static allocator port
 */

#ifndef _MODBUS_PORT_H
#define _MODBUS_PORT_H

#define LIGHTMODBUS_SLAVE
#define LIGHTMODBUS_F03S
#define LIGHTMODBUS_F06S
#define LIGHTMODBUS_F16S

#include "lightmodbus.h"
#include <stdbool.h>

#define STATIC_BUFFER_SIZE 256

typedef struct {
	uint32_t illegal_function;
	uint32_t illegal_address;
	uint32_t illegal_value;
	uint32_t slave_failure;
	uint32_t acknowledge;
	uint32_t negative_acknowledge;
} modbus_exceptions_t;

typedef struct {
	ModbusSlave slave;
	ModbusErrorInfo err;
	uint8_t buffer[STATIC_BUFFER_SIZE];
	const char *name;
	uint8_t id;
	modbus_exceptions_t exceptions;
} modbus_t;

bool modbus_port_init(modbus_t *const modbus, const char *const name, uint8_t id);
const modbus_exceptions_t* modbus_port_get_exceptions(const modbus_t *const modbus);

#endif
