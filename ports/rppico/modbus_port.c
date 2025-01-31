#include "modbus_port.h"
#define LIGHTMODBUS_IMPL
#include "lightmodbus.h"

#include <string.h>

static ModbusError modbusStaticAllocator(ModbusBuffer *buffer, uint16_t size, void *context)
{
    if (context == NULL)
    {
        buffer->data = NULL;
        return MODBUS_ERROR_ALLOC;
    }

    if (!size)
    {
        buffer->data = NULL;
        memset(context, 0, STATIC_BUFFER_SIZE);
        return MODBUS_OK;
    }
    else
    {
        if (size > STATIC_BUFFER_SIZE)
        {
            buffer->data = NULL;
            return MODBUS_ERROR_ALLOC;
        }
        else
        {
            buffer->data = context;
            return MODBUS_OK;
        }
    }
}

bool modbus_port_init(modbus_t *const modbus, ModbusRegisterCallback registerCallback, ModbusSlaveExceptionCallback exceptionCallback, modbus_error_handler error_handler)
{
    modbus->error_handler = error_handler;
    modbus->err = modbusSlaveInit(
        &(modbus->slave),
        registerCallback,
        exceptionCallback,
        modbusStaticAllocator,
        modbusSlaveDefaultFunctions,
        modbusSlaveDefaultFunctionCount);
    if (!(modbusIsOk(modbus->err)))
    {
        if (modbus->error_handler != NULL)
        {
            modbus->error_handler("Modbus slave init error.");
        }
        return true;
    }
    modbusSlaveSetUserPointer(&(modbus->slave), &(modbus->buffer));
    return false;
}