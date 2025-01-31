#ifndef _TCP_PORT_H
#define _TCP_PORT_H

#include "modbus_port.h"

#define MODBUS_TCP_PORT_DEFAULT 502
#define MODBUS_TCP_REC_MESSAGE_MAX_SIZE 260 // 7+1+252

typedef void (*modbus_debug_handler)(const char *const debug_string, size_t debug_string_len);
typedef struct
{
    modbus_t modbus;
    uint8_t socket;
    uint16_t port;
    uint8_t interrupts;
    uint8_t clear_interrupts;
    uint8_t receive_buffer[MODBUS_TCP_REC_MESSAGE_MAX_SIZE * 4];
    uint16_t receive_buffer_len;
    uint8_t send_buffer[MODBUS_TCP_REC_MESSAGE_MAX_SIZE];
    uint16_t send_buffer_len;
    modbus_debug_handler modbus_debug;
} modbus_tcp_t;

void modbus_tcp_poll(modbus_tcp_t *const modbus_tcp);
uint8_t modbus_tcp_init(modbus_tcp_t *const modbus_tcp, ModbusRegisterCallback registerCallback, ModbusSlaveExceptionCallback exceptionCallback, uint8_t modbus_tcp_socket, uint16_t modbus_tcp_port, modbus_error_handler error_handler, modbus_debug_handler modbus_debug);

#endif