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

void modbus_rtu_repeating_timer_callback(TIM_HandleTypeDef *htim);
bool modbus_rtu_init(osThreadId_t task_handle,
		uint8_t slave_address,
		TIM_HandleTypeDef *repeating_timer,
		UART_HandleTypeDef *uart,
		modbus_baudrates_t baudrate,
		uint16_t uart_dir_pin,
		GPIO_TypeDef *uart_dir_port,
		ModbusRegisterCallback registerCallback,
		ModbusSlaveExceptionCallback exceptionCallback,
		modbus_error_handler error_handler,
		uint32_t poll_timeout);
bool modbus_rtu_deinit(void);
bool modbus_rtu_poll(void);

#endif
