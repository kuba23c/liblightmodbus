#ifndef _RTU_PORT_H
#define _RTU_PORT_H

#include "main.h"
#include "modbus_port.h"

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

#define MODBUS_RTU_REC_MESSAGE_MAX_SIZE 256 // 1+1+252+2

typedef struct {
	UART_HandleTypeDef *uart;
	uint16_t uart_dir_pin;
	GPIO_TypeDef *uart_dir_port;
	modbus_baudrates_t baudrate;
} modbus_rtu_uart_t;

typedef struct {
	volatile uint8_t enable;
	volatile uint32_t timeout;
	volatile uint32_t cnt;
} modbus_rtu_timer_t;

typedef struct {
	TIM_HandleTypeDef *repeating_timer;
	volatile modbus_rtu_timer_t timer_1_5;
	volatile modbus_rtu_timer_t timer_3_5;
} modbus_rtu_timers_t;

typedef enum {
	MODBUS_RTU_READ_READY = 0x01,
	MODBUS_RTU_EMIT_READY = 0x02,
} modbus_rtu_events_t;

typedef enum {
	MODBUS_RTU_NONE,
	MODBUS_RTU_INIT,
	MODBUS_RTU_IDLE,
	MODBUS_RTU_RECEPTION,
	MODBUS_RTU_WAIT,
	MODBUS_RTU_EMISSION,
} modbus_rtu_states_t;

typedef uint8_t (*modbus_rtu_notify_wait_t)(uint32_t* const);
typedef uint8_t (*modbus_rtu_notify_t)(uint32_t);
typedef uint8_t (*modbus_rtu_notify_from_isr_t)(uint32_t);
typedef struct {
	modbus_t modbus;
	uint8_t slave_address;
	modbus_rtu_uart_t uart;
	modbus_rtu_timers_t timers;
	uint32_t events;
	volatile modbus_rtu_states_t state;
	volatile uint8_t receive_buffer[MODBUS_RTU_REC_MESSAGE_MAX_SIZE];
	volatile uint16_t receive_buffer_len;
	volatile uint8_t send_buffer[MODBUS_RTU_REC_MESSAGE_MAX_SIZE];
	volatile uint16_t send_cnt;
	volatile uint16_t send_buffer_len;
	osThreadId_t task_handle;
} modbus_rtu_t;

void modbus_rtu_repeating_timer_callback(TIM_HandleTypeDef *htim);
bool modbus_rtu_init(osThreadId_t task_handle, uint8_t slave_address, TIM_HandleTypeDef *repeating_timer, UART_HandleTypeDef *uart, modbus_baudrates_t baudrate, uint16_t uart_dir_pin, GPIO_TypeDef *uart_dir_port, ModbusRegisterCallback registerCallback, ModbusSlaveExceptionCallback exceptionCallback, modbus_error_handler error_handler);

bool modbus_rtu_deinit(void);
bool modbus_rtu_poll(void);

#endif
