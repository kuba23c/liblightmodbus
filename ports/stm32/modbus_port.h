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
	ModbusSlave slave;
	ModbusErrorInfo err;
	uint8_t buffer[STATIC_BUFFER_SIZE];
} modbus_t;

bool modbus_port_init(modbus_t *const modbus);

#endif
