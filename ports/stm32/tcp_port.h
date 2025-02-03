/**
 * file: tcp_port.h
 * Author: kuba23c
 * Description: Lightmodbus TCP port for STM32 + CMSIS RTOSv2 + FreeRTOS + LwIP
 */

#ifndef _TCP_PORT_H
#define _TCP_PORT_H

#include "modbus_port.h"

uint8_t modbus_tcp_init(void);
void modbus_tcp_deinit(void);

#endif
