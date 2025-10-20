/**
 * file: modbus_port.c
 * Author: kuba23c
 * Description: Lightmodbus static allocator port
 */

#include "modbus_port.h"
#define LIGHTMODBUS_IMPL
#include "lightmodbus.h"

#include <string.h>
#include "modbus_callbacks.h"

static ModbusError modbusStaticAllocator(ModbusBuffer *buffer, uint16_t size, void *context) {
	if (context == NULL) {
		buffer->data = NULL;
		return (MODBUS_ERROR_ALLOC);
	}
	modbus_t *modbus = (modbus_t*) context;
	if (!size) {
		buffer->data = NULL;
		memset(modbus->buffer, 0, STATIC_BUFFER_SIZE);
		return (MODBUS_OK);
	} else {
		if (size > STATIC_BUFFER_SIZE) {
			buffer->data = NULL;
			return (MODBUS_ERROR_ALLOC);
		} else {
			buffer->data = modbus->buffer;
			return (MODBUS_OK);
		}
	}
}

bool modbus_port_init(modbus_t *const modbus, const char *const name, uint8_t id) {
	modbus->name = name;
	modbus->id = id;
	memset(&modbus->exceptions, 0, sizeof(modbus_exceptions_t));
	modbus->err = modbusSlaveInit(&(modbus->slave), modbus_register_callback, modbus_exception_callback, modbusStaticAllocator, modbusSlaveDefaultFunctions,
			modbusSlaveDefaultFunctionCount);
	if (!(modbusIsOk(modbus->err))) {
		return (true);
	}
	modbusSlaveSetUserPointer(&(modbus->slave), modbus);
	return (false);
}

const modbus_exceptions_t* modbus_port_get_exceptions(const modbus_t *const modbus) {
	return ((modbus_exceptions_t*) &modbus->exceptions);
}
