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
#include "rtu_config.h"

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
	uint32_t unknown_state;
} modbus_rtu_stats_t;

uint32_t modbus_baudrate_2_number(modbus_baudrates_t baudrate);
modbus_baudrates_t modbus_number_2_baudrate(uint32_t number);
void modbus_rtu_repeating_timer_callback(TIM_HandleTypeDef *htim);
bool modbus_rtu_start(uint8_t id, osThreadId_t task_handle, uint8_t slave_address, TIM_HandleTypeDef *repeating_timer, UART_HandleTypeDef *uart,
		modbus_baudrates_t baudrate, uint16_t uart_dir_pin, GPIO_TypeDef *uart_dir_port, uint32_t poll_timeout);
bool modbus_rtu_stop(void);
bool modbus_rtu_is_active(void);
const modbus_rtu_stats_t* MODBUS_RTU_GetStats(void);
const modbus_exceptions_t* modbus_rtu_get_exceptions(void);
void modbus_rtu_clear_stats(void);
void modbus_rtu_clear_exceptions(void);
#if !TASK_CUSTOM_EVENT_HANDLING
bool modbus_rtu_poll(void);
#else
void modbus_rtu_poll(const uint32_t *const events);
#endif
#endif
