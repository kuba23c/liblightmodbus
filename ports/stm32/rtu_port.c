/**
 * file: rtu_port.c
 * Author: kuba23c
 * Description: Lightmodbus RTU port for STM32 + CMSIS RTOSv2 + FreeRTOS
 */

#include "rtu_port.h"
#include "modbus_callbacks.h"

#define MODBUS_MAX_WAIT 500 // 500ms
#define MODBUS_DIRECTION_RX GPIO_PIN_RESET
#define MODBUS_DIRECTION_TX GPIO_PIN_SET
#define MODBUS_MAX_SLAVE_ADDRESS	247

#define TIMER_15_TIMEOUT	15	// 750/50
#define TIMER_35_TIMEOUT	35	// 1750/50
// 1 char = (11 * 1000000)/(50 * baudrate)
// 1.5 char = (3 * 11 * 1000000)/(2 * 50 * baudrate)
// 3.5 char = (7 * 11 * 1000000)/(2 * 50 * baudrate)
#define TIMER_15_COEFFICIENT	((uint32_t) 330000)
#define TIMER_35_COEFFICIENT	((uint32_t) 770000)

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

typedef struct {
	modbus_t modbus;
	uint8_t slave_address;
	modbus_rtu_uart_t uart;
	modbus_rtu_timers_t timers;
	uint32_t events;
	uint32_t poll_timeout;
	volatile modbus_rtu_states_t state;
	volatile uint8_t receive_buffer[MODBUS_RTU_REC_MESSAGE_MAX_SIZE];
	volatile uint16_t receive_buffer_len;
	volatile uint8_t send_buffer[MODBUS_RTU_REC_MESSAGE_MAX_SIZE];
	volatile uint16_t send_cnt;
	volatile uint16_t send_buffer_len;
	osThreadId_t task_handle;
	modbus_rtu_stats_t stats;
	bool is_active;
} modbus_rtu_t;

typedef enum {
	MODBUS_IRQ_ALL_OFF,
	MODBUS_IRQ_RX_ON,
	MODBUS_IRQ_TX_ON,
	MODBUS_IRQ_MAX
} modbus_irq_t;

typedef enum {
	MODBUS_VALUE_0 = 0,
	MODBUS_VALUE_1200 = 1200,
	MODBUS_VALUE_2400 = 2400,
	MODBUS_VALUE_4800 = 4800,
	MODBUS_VALUE_9600 = 9600,
	MODBUS_VALUE_19200 = 19200,
	MODBUS_VALUE_57600 = 57600,
	MODBUS_VALUE_115200 = 115200,
} modbus_baudrates_value_t;

static volatile modbus_rtu_t modbus_rtu = { 0 };
static volatile uint8_t dummy_read = 0;
static volatile uint32_t uart_isr_status = 0;
static volatile modbus_irq_t current_irq_state = MODBUS_IRQ_ALL_OFF;

static bool notify_wait(void) {
	modbus_rtu.events = osThreadFlagsWait(MODBUS_RTU_READ_READY | MODBUS_RTU_EMIT_READY, osFlagsWaitAny, modbus_rtu.poll_timeout);

	if (modbus_rtu.events == (uint32_t) osErrorTimeout) {
		modbus_rtu.events = 0;
		return (true);
	}
	if (modbus_rtu.events == MODBUS_RTU_READ_READY || modbus_rtu.events == MODBUS_RTU_EMIT_READY
			|| modbus_rtu.events == (MODBUS_RTU_READ_READY | MODBUS_RTU_EMIT_READY)) {
		return (false);
	}
	modbus_rtu.events = 0;
	return (true);

}

static bool notify_from_isr(modbus_rtu_events_t event_to_set) {
	osThreadFlagsSet(modbus_rtu.task_handle, (uint32_t) event_to_set);
	return (false);
}

uint32_t modbus_baudrate_2_number(modbus_baudrates_t baudrate) {
	switch (baudrate) {
	case MODBUS_1200:
		return ((uint32_t) MODBUS_VALUE_1200);
	case MODBUS_2400:
		return ((uint32_t) MODBUS_VALUE_2400);
	case MODBUS_4800:
		return ((uint32_t) MODBUS_VALUE_4800);
	case MODBUS_9600:
		return ((uint32_t) MODBUS_VALUE_9600);
	case MODBUS_19200:
		return ((uint32_t) MODBUS_VALUE_19200);
	case MODBUS_57600:
		return ((uint32_t) MODBUS_VALUE_57600);
	case MODBUS_115200:
		return ((uint32_t) MODBUS_VALUE_115200);
	default:
		return ((uint32_t) MODBUS_VALUE_115200);
	}
}

modbus_baudrates_t modbus_number_2_baudrate(uint32_t number) {
	switch (number) {
	case MODBUS_VALUE_1200:
		return (MODBUS_1200);
	case MODBUS_VALUE_2400:
		return (MODBUS_2400);
	case MODBUS_VALUE_4800:
		return (MODBUS_4800);
	case MODBUS_VALUE_9600:
		return (MODBUS_9600);
	case MODBUS_VALUE_19200:
		return (MODBUS_19200);
	case MODBUS_VALUE_57600:
		return (MODBUS_57600);
	case MODBUS_VALUE_115200:
		return (MODBUS_115200);
	default:
		return (MODBUS_115200);
	}
}

static void wait_while_uart_busy(void) {
	uint32_t tim1 = osKernelGetTickCount();
	uint32_t tim2 = osKernelGetTickCount();

	do {
		if (modbus_rtu.uart.uart->Instance->ISR & UART_FLAG_TC) {
			break;
		}
		tim2 = osKernelGetTickCount();
		if (tim2 < tim1) {
			tim1 = tim2;
		} else if ((tim2 - tim1) > MODBUS_MAX_WAIT) {
			break;
		}
	} while (true);

}

static void send_byte(void) {
	modbus_rtu.send_cnt++;
	modbus_rtu.uart.uart->Instance->TDR = (uint32_t) (modbus_rtu.send_buffer[modbus_rtu.send_cnt - 1]);
}

static void modbus_rtu_uart_irq_set(modbus_irq_t irq_state) {
	if (current_irq_state == irq_state || irq_state >= MODBUS_IRQ_MAX) {
		return;
	}
	current_irq_state = irq_state;

	switch (irq_state) {
	case MODBUS_IRQ_ALL_OFF:
		wait_while_uart_busy();
		__HAL_UART_DISABLE_IT(modbus_rtu.uart.uart, UART_IT_TXE);
		__HAL_UART_DISABLE_IT(modbus_rtu.uart.uart, UART_IT_RXNE);
		dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
		if (modbus_rtu.uart.uart_dir_port != NULL) {
			HAL_GPIO_WritePin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_RX);
			while (HAL_GPIO_ReadPin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_RX) {
			}
		}
		break;
	case MODBUS_IRQ_RX_ON:
		wait_while_uart_busy();
		__HAL_UART_DISABLE_IT(modbus_rtu.uart.uart, UART_IT_TXE);
		if (modbus_rtu.uart.uart_dir_port != NULL) {
			HAL_GPIO_WritePin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_RX);
			while (HAL_GPIO_ReadPin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_RX) {
			}
		}
		dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
		__HAL_UART_ENABLE_IT(modbus_rtu.uart.uart, UART_IT_RXNE);
		break;
	case MODBUS_IRQ_TX_ON:
		__HAL_UART_DISABLE_IT(modbus_rtu.uart.uart, UART_IT_RXNE);
		dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
		if (modbus_rtu.uart.uart_dir_port != NULL) {
			HAL_GPIO_WritePin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_TX);
			while (HAL_GPIO_ReadPin(modbus_rtu.uart.uart_dir_port, modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_TX) {
			}
		}
		send_byte();
		__HAL_UART_ENABLE_IT(modbus_rtu.uart.uart, UART_IT_TXE);
		break;
	default:
		break;
	}
}

/**
 * @brief Call this every 50us in timer interrupt
 *
 * @param htim timer handler
 */
void modbus_rtu_repeating_timer_callback(TIM_HandleTypeDef *htim) {
	if (htim != modbus_rtu.timers.repeating_timer) {
		return;
	}
	if (modbus_rtu.timers.timer_1_5.enable) {
		modbus_rtu.timers.timer_1_5.cnt++;
		if (modbus_rtu.timers.timer_1_5.cnt > modbus_rtu.timers.timer_1_5.timeout) {
			modbus_rtu.timers.timer_1_5.cnt = 0;
			modbus_rtu.timers.timer_1_5.enable = 0;
			switch (modbus_rtu.state) {
			case MODBUS_RTU_RECEPTION:
				modbus_rtu.state = MODBUS_RTU_WAIT;
				notify_from_isr(MODBUS_RTU_READ_READY);
				break;
			default:
				break;
			}
		}
	}
	if (modbus_rtu.timers.timer_3_5.enable) {
		modbus_rtu.timers.timer_3_5.cnt++;
		if (modbus_rtu.timers.timer_3_5.cnt > modbus_rtu.timers.timer_3_5.timeout) {
			modbus_rtu.timers.timer_3_5.cnt = 0;
			modbus_rtu.timers.timer_3_5.enable = 0;
			switch (modbus_rtu.state) {
			case MODBUS_RTU_INIT:
				modbus_rtu.state = MODBUS_RTU_IDLE;
				break;
			case MODBUS_RTU_WAIT:
				modbus_rtu.state = MODBUS_RTU_EMISSION;
				notify_from_isr(MODBUS_RTU_EMIT_READY);
				break;
			case MODBUS_RTU_EMISSION:
				modbus_rtu.state = MODBUS_RTU_IDLE;
				modbus_rtu_uart_irq_set(MODBUS_IRQ_RX_ON);
				break;
			default:
				break;
			}
		}
	}
}

static bool modbus_rtu_timers_init(void) {
	modbus_rtu.timers.timer_1_5.cnt = 0;
	modbus_rtu.timers.timer_1_5.enable = 0;
	modbus_rtu.timers.timer_3_5.cnt = 0;
	modbus_rtu.timers.timer_3_5.enable = 0;

	switch (modbus_rtu.uart.baudrate) {
	case MODBUS_1200:
	case MODBUS_2400:
	case MODBUS_4800:
	case MODBUS_9600:
	case MODBUS_19200:
		modbus_rtu.timers.timer_1_5.timeout = (uint32_t) (TIMER_15_COEFFICIENT / modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
		modbus_rtu.timers.timer_3_5.timeout = (uint32_t) (TIMER_35_COEFFICIENT / modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
		break;
	case MODBUS_57600:
	case MODBUS_115200:
		modbus_rtu.timers.timer_1_5.timeout = TIMER_15_TIMEOUT;
		modbus_rtu.timers.timer_3_5.timeout = TIMER_35_TIMEOUT;
		break;
	default:
		return (true);
		break;
	}
	if (HAL_TIM_Base_Start_IT(modbus_rtu.timers.repeating_timer) != HAL_OK) {
		return (true);
	}

	return (false);
}

static void modbus_rtu_timers_reset(void) {
	modbus_rtu.timers.timer_1_5.cnt = 0;
	modbus_rtu.timers.timer_3_5.cnt = 0;
}

static void modbus_rtu_timers_enable(void) {
	modbus_rtu.timers.timer_1_5.cnt = 0;
	modbus_rtu.timers.timer_3_5.cnt = 0;
	modbus_rtu.timers.timer_1_5.enable = 1;
	modbus_rtu.timers.timer_3_5.enable = 1;
}

static void modbus_rtu_timer_3_5_enable(void) {
	modbus_rtu.timers.timer_3_5.cnt = 0;
	modbus_rtu.timers.timer_3_5.enable = 1;
}

static void modbus_rtu_timers_disable(void) {
	modbus_rtu.timers.timer_1_5.cnt = 0;
	modbus_rtu.timers.timer_3_5.cnt = 0;
	modbus_rtu.timers.timer_1_5.enable = 0;
	modbus_rtu.timers.timer_3_5.enable = 0;
}

void UART7_IRQHandler(void) {
	uart_isr_status = modbus_rtu.uart.uart->Instance->ISR;

	if (uart_isr_status & UART_FLAG_TXE) {
		if (modbus_rtu.state == MODBUS_RTU_EMISSION) {
			if (modbus_rtu.send_cnt < modbus_rtu.send_buffer_len) {
				send_byte();
			} else {
				memset((uint8_t*) modbus_rtu.send_buffer, 0, sizeof(modbus_rtu.send_buffer));
				modbus_rtu.send_buffer_len = 0;
				modbus_rtu.send_cnt = 0;
				modbus_rtu_uart_irq_set(MODBUS_IRQ_ALL_OFF);
				modbus_rtu_timer_3_5_enable();
			}
		}
	}

	if (uart_isr_status & UART_FLAG_RXNE) {
		if (modbus_rtu.receive_buffer_len < sizeof(modbus_rtu.receive_buffer)) {
			switch (modbus_rtu.state) {
			case MODBUS_RTU_INIT:
				modbus_rtu_timer_3_5_enable();
				dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
				break;
			case MODBUS_RTU_IDLE:
				modbus_rtu.receive_buffer[modbus_rtu.receive_buffer_len] = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
				modbus_rtu.receive_buffer_len++;
				modbus_rtu.state = MODBUS_RTU_RECEPTION;
				modbus_rtu_timers_enable();
				break;
			case MODBUS_RTU_RECEPTION:
				modbus_rtu_timers_reset();
				modbus_rtu.receive_buffer[modbus_rtu.receive_buffer_len] = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
				modbus_rtu.receive_buffer_len++;
				break;
			case MODBUS_RTU_WAIT:
				modbus_rtu_timer_3_5_enable();
				dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
				break;
			default:
				dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
				break;
			}
		} else {
			modbus_rtu_timers_disable();
			dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
			memset((uint8_t*) modbus_rtu.receive_buffer, 0, sizeof(modbus_rtu.receive_buffer));
			modbus_rtu.receive_buffer_len = 0;
			modbus_rtu.state = MODBUS_RTU_IDLE;
		}
	}

	if (uart_isr_status & UART_FLAG_ORE) {
		memset((uint8_t*) modbus_rtu.send_buffer, 0, sizeof(modbus_rtu.send_buffer));
		modbus_rtu.send_buffer_len = 0;
		modbus_rtu.send_cnt = 0;
		modbus_rtu_uart_irq_set(MODBUS_IRQ_ALL_OFF);

		modbus_rtu_timers_disable();
		dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
		memset((uint8_t*) modbus_rtu.receive_buffer, 0, sizeof(modbus_rtu.receive_buffer));
		modbus_rtu.receive_buffer_len = 0;
		modbus_rtu.state = MODBUS_RTU_IDLE;

		dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
		SET_BIT(modbus_rtu.uart.uart->Instance->ICR, USART_ICR_ORECF);
		modbus_rtu_uart_irq_set(MODBUS_IRQ_RX_ON);
	}
}

static uint8_t modbus_rtu_uart_init(void) {
	if (HAL_UART_DeInit(modbus_rtu.uart.uart)) {
		return (1);
	}
	modbus_rtu.uart.uart->Init.BaudRate = modbus_baudrate_2_number(modbus_rtu.uart.baudrate);
	modbus_rtu.uart.uart->Init.WordLength = UART_WORDLENGTH_8B;
	modbus_rtu.uart.uart->Init.StopBits = UART_STOPBITS_1;
	modbus_rtu.uart.uart->Init.Parity = UART_PARITY_NONE;
	modbus_rtu.uart.uart->Init.Mode = UART_MODE_TX_RX;
	modbus_rtu.uart.uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	modbus_rtu.uart.uart->Init.OverSampling = UART_OVERSAMPLING_16;
	modbus_rtu.uart.uart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	modbus_rtu.uart.uart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	modbus_rtu.uart.uart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(modbus_rtu.uart.uart)) {
		return (1);
	}
	if (HAL_UARTEx_SetTxFifoThreshold(modbus_rtu.uart.uart,
	UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		return (1);
	}
	if (HAL_UARTEx_SetRxFifoThreshold(modbus_rtu.uart.uart,
	UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		return (1);
	}
	if (HAL_UARTEx_DisableFifoMode(modbus_rtu.uart.uart) != HAL_OK) {
		return (1);
	}
	return (0);
}

/**
 * @brief call this in main modbus task to start modbus rtu
 *
 * @param task_handle main modbus task handle
 * @param slave_address address of this device
 * @param repeating_timer repeating timer handle
 * @param uart uart handle
 * @param baudrate baudrate from modbus_baudrates_t
 * @param uart_dir_pin uart direction pin
 * @param uart_dir_port uart direction port
 * @param registerCallback function callback for register operation
 * @param exceptionCallback function callback for exceptions
 * @param error_handler function callback for driver errors
 * @param poll_timeout poll timeout
 * @return true - ok, false - some error occurred
 */
bool modbus_rtu_start(uint8_t id, osThreadId_t task_handle, uint8_t slave_address, TIM_HandleTypeDef *repeating_timer, UART_HandleTypeDef *uart,
		modbus_baudrates_t baudrate, uint16_t uart_dir_pin, GPIO_TypeDef *uart_dir_port, uint32_t poll_timeout) {
	if (task_handle == NULL || repeating_timer == NULL || uart == NULL || slave_address == 0 || slave_address > MODBUS_MAX_SLAVE_ADDRESS) {
		return (false);
	}
	if (modbus_rtu.is_active) {
		return (false);
	} else {
		if (modbus_port_init((modbus_t*) &(modbus_rtu.modbus), "RTU", id)) {
			modbus_rtu.stats.modbus_internal_errors++;
			return (false);
		}
		modbus_rtu.task_handle = task_handle;
		modbus_rtu.slave_address = slave_address;
		modbus_rtu.uart.uart = uart;
		modbus_rtu.uart.baudrate = baudrate;
		modbus_rtu.uart.uart_dir_pin = uart_dir_pin;
		modbus_rtu.uart.uart_dir_port = uart_dir_port;
		modbus_rtu.timers.repeating_timer = repeating_timer;
		modbus_rtu.poll_timeout = poll_timeout;
		if (modbus_rtu_timers_init()) {
			modbus_rtu.stats.timer_errors++;
			return (false);
		}
		if (modbus_rtu_uart_init()) {
			modbus_rtu.stats.uart_errors++;
			return (false);
		}
		modbus_rtu.state = MODBUS_RTU_INIT;
		modbus_rtu_uart_irq_set(MODBUS_IRQ_RX_ON);
		modbus_rtu_timer_3_5_enable();
		modbus_rtu.is_active = true;
		return (true);
	}
}

/**
 * @brief call this function to stop modbus rtu
 *
 * @return true - ok, false - error occurred
 */
bool modbus_rtu_stop(void) {
	if (modbus_rtu.is_active) {
		modbus_rtu_uart_irq_set(MODBUS_IRQ_ALL_OFF);
		if (HAL_TIM_Base_Stop_IT(modbus_rtu.timers.repeating_timer) != HAL_OK) {
			modbus_rtu.stats.timer_errors++;
			return (false);
		}
		if (HAL_UART_DeInit(modbus_rtu.uart.uart) != HAL_OK) {
			modbus_rtu.stats.uart_errors++;
			return (false);
		}
		modbus_rtu.task_handle = NULL;
		modbus_rtu.slave_address = 0;
		modbus_rtu.uart.uart = NULL;
		modbus_rtu.uart.baudrate = 0;
		modbus_rtu.uart.uart_dir_pin = 0;
		modbus_rtu.uart.uart_dir_port = NULL;
		modbus_rtu.timers.repeating_timer = NULL;
		modbus_rtu.poll_timeout = 0;
		modbus_rtu.state = MODBUS_RTU_NONE;
		modbus_rtu.is_active = false;
		return (true);
	} else {
		return (false);
	}
}

bool modbus_rtu_is_active(void) {
	return (modbus_rtu.is_active);
}

const modbus_rtu_stats_t* modbus_rtu_get_stats(void) {
	return ((modbus_rtu_stats_t*) &modbus_rtu.stats);
}

const modbus_exceptions_t* modbus_rtu_get_exceptions(void) {
	return ((modbus_exceptions_t*) &modbus_rtu.modbus.exceptions);
}

void modbus_rtu_clear_stats(void) {
	memset((modbus_rtu_stats_t*) &modbus_rtu.stats, 0, sizeof(modbus_rtu_stats_t));
}

static void on_read_ready(void) {
	modbus_rtu.modbus.err = modbusParseRequestRTU((ModbusSlave*) &(modbus_rtu.modbus.slave), modbus_rtu.slave_address, (uint8_t*) modbus_rtu.receive_buffer,
			modbus_rtu.receive_buffer_len);
	memset((uint8_t*) modbus_rtu.receive_buffer, 0, sizeof(modbus_rtu.receive_buffer));
	modbus_rtu.receive_buffer_len = 0;
	modbus_rtu.stats.messages_received++;
	if (modbusIsOk(modbus_rtu.modbus.err)) {
		modbus_rtu.stats.messages_ok++;
		const uint8_t *send_buffer_pointer = modbusSlaveGetResponse((ModbusSlave*) &(modbus_rtu.modbus.slave));
		modbus_rtu.send_buffer_len = modbusSlaveGetResponseLength((ModbusSlave*) &(modbus_rtu.modbus.slave));
		memcpy((uint8_t*) modbus_rtu.send_buffer, send_buffer_pointer, modbus_rtu.send_buffer_len);
		modbusSlaveFreeResponse((ModbusSlave*) &(modbus_rtu.modbus.slave));
		modbus_rtu.stats.messages_sent++;
	} else {
		modbus_rtu.stats.messages_nok++;
	}
}

static void on_emit_ready(void) {
	if (modbusIsOk(modbus_rtu.modbus.err)) {
		modbus_rtu_uart_irq_set(MODBUS_IRQ_TX_ON);
	} else {
		modbus_rtu.state = MODBUS_RTU_IDLE;
	}
}

/**
 * @brief call this in main modbus task, it will block task
 *
 * @return true - process ongoing, false - timeout
 */
bool modbus_rtu_poll(void) {
	if (notify_wait()) {
		if (READ_BIT(modbus_rtu.uart.uart->Instance->ISR, USART_ISR_ORE)) {
			dummy_read = (uint8_t) (modbus_rtu.uart.uart->Instance->RDR);
			SET_BIT(modbus_rtu.uart.uart->Instance->ICR, USART_ICR_ORECF);
			__HAL_UART_ENABLE_IT(modbus_rtu.uart.uart, UART_IT_RXNE);
		}
		return (false);
	}
	if ((modbus_rtu.events & MODBUS_RTU_READ_READY) && (modbus_rtu.events & MODBUS_RTU_EMIT_READY)) {
		on_read_ready();
		on_emit_ready();
	} else if (modbus_rtu.events & MODBUS_RTU_READ_READY) {
		on_read_ready();
	} else if (modbus_rtu.events & MODBUS_RTU_EMIT_READY) {
		on_emit_ready();
	} else {
		modbus_rtu.stats.unknown_state++;
	}
	return (true);
}
