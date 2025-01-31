#ifndef _MODBUS_PORT_H
#define _MODBUS_PORT_H

#define LIGHTMODBUS_SLAVE
#define LIGHTMODBUS_F03S
#define LIGHTMODBUS_F06S
#define LIGHTMODBUS_F16S

#include "lightmodbus.h"
#include <stdbool.h>

#define STATIC_BUFFER_SIZE 256
#define MODBUS_RESPONSE_REGISTER_MAX	125

typedef void (*modbus_error_handler)(const char *const error_string);

typedef struct {
	ModbusSlave slave;
	ModbusErrorInfo err;
	uint8_t buffer[STATIC_BUFFER_SIZE];
	modbus_error_handler error_handler;
} modbus_t;

bool modbus_port_init(modbus_t *const modbus, ModbusRegisterCallback registerCallback, ModbusSlaveExceptionCallback exceptionCallback, modbus_error_handler error_handler);

#endif
