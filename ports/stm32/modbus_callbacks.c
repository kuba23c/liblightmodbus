/*
 * modbus_callbacks.c
 *
 *  Created on: Feb 2, 2025
 *      Author: kuba2
 */
#include "modbus_callbacks.h"
#include "modbus_config.h"

ModbusError modbus_exception_callback(const ModbusSlave *status, uint8_t function, ModbusExceptionCode code) {
	modbus_t *modbus = (modbus_t*) modbusSlaveGetUserPointer(status);
	switch (code) {
	case MODBUS_EXCEP_ILLEGAL_FUNCTION:
		modbus->exceptions.illegal_function++;
		break;
	case MODBUS_EXCEP_ILLEGAL_ADDRESS:
		modbus->exceptions.illegal_address++;
		break;
	case MODBUS_EXCEP_ILLEGAL_VALUE:
		modbus->exceptions.illegal_value++;
		break;
	case MODBUS_EXCEP_SLAVE_FAILURE:
		modbus->exceptions.slave_failure++;
		break;
	case MODBUS_EXCEP_ACK:
		modbus->exceptions.acknowledge++;
		break;
	case MODBUS_EXCEP_NACK:
		modbus->exceptions.negative_acknowledge++;
		break;
	default:
		break;
	}
	modbus_exception(function, code, modbus->name, modbus->id);
	return (MODBUS_OK);
}

ModbusError modbus_register_callback(const ModbusSlave *status, const ModbusRegisterCallbackArgs *args, ModbusRegisterCallbackResult *out) {
	UNUSED(status);
	uint16_t index = (args->index);

	switch (args->query) {
	case MODBUS_REGQ_R_CHECK:
		if (args->type != MODBUS_HOLDING_REGISTER) {
			out->exceptionCode = MODBUS_EXCEP_ILLEGAL_FUNCTION;
		} else if (modbus_is_reg_read(index)) {
			out->exceptionCode = MODBUS_EXCEP_NONE;
		} else {
			out->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
		}
		break;
	case MODBUS_REGQ_W_CHECK:
		if (args->type != MODBUS_HOLDING_REGISTER) {
			out->exceptionCode = MODBUS_EXCEP_ILLEGAL_FUNCTION;
		} else if (modbus_is_reg_write(index)) {
			out->exceptionCode = MODBUS_EXCEP_NONE;
		} else {
			out->exceptionCode = MODBUS_EXCEP_ILLEGAL_ADDRESS;
		}
		break;
	case MODBUS_REGQ_R:
		if (args->type == MODBUS_HOLDING_REGISTER) {
			out->value = modbus_reg_read(index);
		} else {
			out->value = 0;
		}
		break;
	case MODBUS_REGQ_W:
		if (args->type == MODBUS_HOLDING_REGISTER) {
			modbus_reg_write(index, args->value);
		}
		break;
	default:
		out->exceptionCode = MODBUS_EXCEP_ILLEGAL_FUNCTION;
		out->value = 0;
		break;
	}
	return (MODBUS_OK);
}
