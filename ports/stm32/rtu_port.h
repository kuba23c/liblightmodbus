/**
 * file: rtu_port.h
 * Author: kuba23c
 * Description: Lightmodbus RTU port for STM32 + CMSIS RTOSv2 + FreeRTOS
 */

#ifndef _RTU_PORT_H
#define _RTU_PORT_H

#include "main.h"
#include "modbus_port.h"
#include "usart.h"

typedef enum {
	MODBUS_0,
	MODBUS_1200,
	MODBUS_2400,
	MODBUS_4800,
	MODBUS_9600,
	MODBUS_19200,
	MODBUS_57600,
	MODBUS_115200,
} modbus_baudrates_t;

typedef struct {
	uint32_t messages_received;
	uint32_t messages_sent;
	uint32_t messages_ok;
	uint32_t messages_nok;
	uint32_t uart_errors;
	uint32_t timer_errors;
	uint32_t modbus_internal_errors;
} modbus_rtu_stats_t;

void modbus_rtu_repeating_timer_callback(TIM_HandleTypeDef *htim);
bool modbus_rtu_start(osThreadId_t task_handle, uint8_t slave_address, TIM_HandleTypeDef *repeating_timer, UART_HandleTypeDef *uart,
		modbus_baudrates_t baudrate, uint16_t uart_dir_pin, GPIO_TypeDef *uart_dir_port, uint32_t poll_timeout);
bool modbus_rtu_stop(void);
bool modbus_rtu_is_active(void);
const modbus_rtu_stats_t* modbus_rtu_get_stats(void);
void modbus_rtu_clear_stats(void);
bool modbus_rtu_poll(void);

#endif
