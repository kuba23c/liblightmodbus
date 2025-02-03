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

static ModbusError modbusStaticAllocator(ModbusBuffer *buffer, uint16_t size,
		void *context) {
	if (context == NULL) {
		buffer->data = NULL;
		return (MODBUS_ERROR_ALLOC);
	}

	if (!size) {
		buffer->data = NULL;
		memset(context, 0, STATIC_BUFFER_SIZE);
		return (MODBUS_OK);
	} else {
		if (size > STATIC_BUFFER_SIZE) {
			buffer->data = NULL;
			return (MODBUS_ERROR_ALLOC);
		} else {
			buffer->data = context;
			return (MODBUS_OK);
		}
	}
}

bool modbus_port_init(modbus_t *const modbus) {
	modbus->err = modbusSlaveInit(&(modbus->slave), modbus_register_callback,
			modbus_exception_callback, modbusStaticAllocator,
			modbusSlaveDefaultFunctions, modbusSlaveDefaultFunctionCount);
	if (!(modbusIsOk(modbus->err))) {
		modbus_error_callback("Modbus slave init error.");
		return (true);
	}
	modbusSlaveSetUserPointer(&(modbus->slave), &(modbus->buffer));
	return (false);
}
