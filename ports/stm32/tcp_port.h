/**
 * file: tcp_port.h
 * Author: kuba23c
 * Description: Lightmodbus TCP port for STM32 + CMSIS RTOSv2 + FreeRTOS + LwIP
 *
 * IMPORTANT: required LwRB v3.2.0
 */

#ifndef _TCP_PORT_H
#define _TCP_PORT_H

#include "modbus_port.h"

typedef struct {
	uint8_t clients_connected;
	uint8_t clients_max;
	uint32_t clients_rejected;
	uint32_t clients_accepted;
	uint32_t clients_timeouts;
	uint32_t clients_errors;
	uint32_t clients_closed;
	uint32_t messages_received;
	uint32_t messages_sent;
	uint32_t messages_ok;
	uint32_t messages_nok;
	uint32_t messages_ring_buffer_full;
	uint8_t listeners_active;
	uint8_t listeners_max;
	uint32_t listeners_rejected;
	uint32_t listeners_tcp_stack_error;
	uint32_t listeners_opened;
	uint32_t listeners_closed;
	uint32_t listeners_errors;
	uint32_t modbus_internal_errors;
} modbus_tcp_stats_t;

void modbus_tcp_init(void);
bool modbus_tcp_start(void);
bool modbus_tcp_stop(void);
const modbus_tcp_stats_t* modbus_tcp_stats(void);
void modbus_tcp_clear_stats(void);
bool modbus_tcp_is_active(void);

#endif
