#include "rtu_port.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "modbus_port.h"

#define MODBUS_UART uart0
#define MODBUS_UART_IRQ UART0_IRQ
#define MODBUS_MAX_WAIT 500000 // 500ms
#define MODBUS_DIRECTION_RX false
#define MODBUS_DIRECTION_TX true

typedef enum
{
    MODBUS_IRQ_ALL_OFF,
    MODBUS_IRQ_RX_ON,
    MODBUS_IRQ_TX_ON,
    MODBUS_IRQ_MAX,
} modbus_irq_t;

static volatile modbus_rtu_t modbus_rtu = {0};
static volatile modbus_irq_t current_irq_state = MODBUS_IRQ_ALL_OFF;
static volatile uint8_t dummy_read = 0;
static volatile uint32_t uart_isr_status = 0;

static bool notify_wait(void)
{
    modbus_rtu.events = 0;
    if (xTaskNotifyWait(0, ULONG_MAX, (uint32_t *)&(modbus_rtu.events), portMAX_DELAY) != pdTRUE) // previous was ULONG_MAX
    {
        return true;
    }
    return false;
}

static bool notify(modbus_rtu_events_t event_to_set)
{
    if (xTaskNotify(modbus_rtu.task_handle, (uint32_t)event_to_set, eSetBits) != pdTRUE)
    {
        return true;
    }
    return false;
}

static bool notify_from_isr(modbus_rtu_events_t event_to_set)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xTaskNotifyFromISR(modbus_rtu.task_handle, (uint32_t)event_to_set, eSetBits, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return true;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return false;
}

static uint32_t modbus_baudrate_2_number(modbus_baudrates_t baudrate)
{
    switch (baudrate)
    {
    case MODBUS_0:
        return (uint32_t)0;
    case MODBUS_1200:
        return (uint32_t)1200;
    case MODBUS_2400:
        return (uint32_t)2400;
    case MODBUS_4800:
        return (uint32_t)4800;
    case MODBUS_9600:
        return (uint32_t)9600;
    case MODBUS_19200:
        return (uint32_t)19200;
    case MODBUS_57600:
        return (uint32_t)57600;
    case MODBUS_115200:
        return (uint32_t)115200;
    default:
        return (uint32_t)115200;
    }
}

static void wait_while_uart_busy(void)
{
    uint32_t flag_register = 0;
    absolute_time_t tim1 = get_absolute_time();
    absolute_time_t tim2 = get_absolute_time();

    do
    {
        flag_register = uart_get_hw(MODBUS_UART)->fr;
        if (!(flag_register & UART_UARTFR_BUSY_BITS))
        {
            break;
        }
        tim2 = get_absolute_time();
        if (absolute_time_diff_us(tim1, tim2) > MODBUS_MAX_WAIT)
        {
            break;
        }
    } while (true);
}

static void send_byte(void)
{
    modbus_rtu.send_cnt++;
    uart_get_hw(MODBUS_UART)->dr = (uint32_t)(modbus_rtu.send_buffer[modbus_rtu.send_cnt - 1]);
}

static void modbus_rtu_uart_irq_set(modbus_irq_t irq_status)
{
    if (irq_status == current_irq_state || irq_status >= MODBUS_IRQ_MAX)
    {
        return;
    }
    current_irq_state = irq_status;

    switch (irq_status)
    {
    case MODBUS_IRQ_ALL_OFF:
        wait_while_uart_busy();
        uart_set_irq_enables(MODBUS_UART, false, false);
        gpio_put(modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_RX);
        while (gpio_get(modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_RX)
        {
        }
        break;
    case MODBUS_IRQ_RX_ON:
        wait_while_uart_busy();
        uart_set_irq_enables(MODBUS_UART, false, false);
        gpio_put(modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_RX);
        while (gpio_get(modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_RX)
        {
        }
        uart_set_irq_enables(MODBUS_UART, true, false);
        break;
    case MODBUS_IRQ_TX_ON:
        uart_set_irq_enables(MODBUS_UART, false, false);
        gpio_put(modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_TX);
        while (gpio_get(modbus_rtu.uart.uart_dir_pin) != MODBUS_DIRECTION_TX)
        {
        }
        uart_set_irq_enables(MODBUS_UART, false, true);
        send_byte();
        break;
    default:
        break;
    }
}

static bool modbus_rtu_repeating_timer_callback(repeating_timer_t *rt)
{
    if (modbus_rtu.timers.timer_1_5.enable)
    {
        modbus_rtu.timers.timer_1_5.cnt++;
        if (modbus_rtu.timers.timer_1_5.cnt > modbus_rtu.timers.timer_1_5.timeout)
        {
            modbus_rtu.timers.timer_1_5.cnt = 0;
            modbus_rtu.timers.timer_1_5.enable = 0;
            switch (modbus_rtu.state)
            {
            case MODBUS_RTU_RECEPTION:
                modbus_rtu.state = MODBUS_RTU_WAIT;
                notify_from_isr(MODBUS_RTU_READ_READY);
                break;
            default:
                break;
            }
        }
    }
    if (modbus_rtu.timers.timer_3_5.enable)
    {
        modbus_rtu.timers.timer_3_5.cnt++;
        if (modbus_rtu.timers.timer_3_5.cnt > modbus_rtu.timers.timer_3_5.timeout)
        {
            modbus_rtu.timers.timer_3_5.cnt = 0;
            modbus_rtu.timers.timer_3_5.enable = 0;
            switch (modbus_rtu.state)
            {
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
    return true; // have to return true
}

static bool modbus_rtu_timers_init(void)
{
    modbus_rtu.timers.timer_1_5.cnt = 0;
    modbus_rtu.timers.timer_1_5.enable = 0;
    modbus_rtu.timers.timer_3_5.cnt = 0;
    modbus_rtu.timers.timer_3_5.enable = 0;

    switch (modbus_rtu.uart.baudrate)
    {
    case MODBUS_1200:
    case MODBUS_2400:
    case MODBUS_4800:
    case MODBUS_9600:
    case MODBUS_19200:
        // 1 char = (11 * 1000000)/(50 * baudrate)
        // 1.5 char = (3 * 11 * 1000000)/(2 * 50 * baudrate)
        // 3.5 char = (7 * 11 * 1000000)/(2 * 50 * baudrate)
        modbus_rtu.timers.timer_1_5.timeout = (uint32_t)((uint32_t)330000 / modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
        modbus_rtu.timers.timer_3_5.timeout = (uint32_t)((uint32_t)770000 / modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
        break;
    case MODBUS_57600:
    case MODBUS_115200:
        modbus_rtu.timers.timer_1_5.timeout = 15; // 750/50
        modbus_rtu.timers.timer_3_5.timeout = 35; // 1750/50
        break;
    default:
        return true;
        break;
    }
    if (add_repeating_timer_us(-50, modbus_rtu_repeating_timer_callback, NULL, (repeating_timer_t *)&(modbus_rtu.timers.repeating_timer)) == false)
    {
        return true;
    }

    return false;
}

static void modbus_rtu_timers_reset(void)
{
    modbus_rtu.timers.timer_1_5.cnt = 0;
    modbus_rtu.timers.timer_3_5.cnt = 0;
}

static void modbus_rtu_timers_enable(void)
{
    modbus_rtu.timers.timer_1_5.cnt = 0;
    modbus_rtu.timers.timer_3_5.cnt = 0;
    modbus_rtu.timers.timer_1_5.enable = 1;
    modbus_rtu.timers.timer_3_5.enable = 1;
}

static void modbus_rtu_timer_1_5_enable(void)
{
    modbus_rtu.timers.timer_1_5.cnt = 0;
    modbus_rtu.timers.timer_1_5.enable = 1;
}

static void modbus_rtu_timer_3_5_enable(void)
{
    modbus_rtu.timers.timer_3_5.cnt = 0;
    modbus_rtu.timers.timer_3_5.enable = 1;
}

static void modbus_rtu_timers_disable(void)
{
    modbus_rtu.timers.timer_1_5.cnt = 0;
    modbus_rtu.timers.timer_3_5.cnt = 0;
    modbus_rtu.timers.timer_1_5.enable = 0;
    modbus_rtu.timers.timer_3_5.enable = 0;
}

static void uart_isr_handler(void)
{
    uart_isr_status = uart_get_hw(MODBUS_UART)->mis;

    if (uart_isr_status & UART_UARTMIS_TXMIS_BITS)
    {
        if (modbus_rtu.state == MODBUS_RTU_EMISSION)
        {
            if (modbus_rtu.send_cnt < modbus_rtu.send_buffer_len)
            {
                send_byte();
            }
            else
            {
                uart_get_hw(MODBUS_UART)->icr = UART_UARTICR_TXIC_BITS;
                memset((uint8_t *)modbus_rtu.send_buffer, 0, sizeof(modbus_rtu.send_buffer));
                modbus_rtu.send_buffer_len = 0;
                modbus_rtu.send_cnt = 0;
                modbus_rtu_uart_irq_set(MODBUS_IRQ_ALL_OFF);
                modbus_rtu_timer_3_5_enable();
            }
        }
        else
        {
            uart_get_hw(MODBUS_UART)->icr = UART_UARTICR_TXIC_BITS;
        }
    }

    if (uart_isr_status & UART_UARTMIS_RXMIS_BITS)
    {
        if (modbus_rtu.receive_buffer_len < sizeof(modbus_rtu.receive_buffer))
        {
            switch (modbus_rtu.state)
            {
            case MODBUS_RTU_INIT:
                modbus_rtu_timer_3_5_enable();
                dummy_read = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
                break;
            case MODBUS_RTU_IDLE:
                modbus_rtu.receive_buffer[modbus_rtu.receive_buffer_len] = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
                modbus_rtu.receive_buffer_len++;
                modbus_rtu.state = MODBUS_RTU_RECEPTION;
                modbus_rtu_timers_enable();
                break;
            case MODBUS_RTU_RECEPTION:
                modbus_rtu_timers_reset();
                modbus_rtu.receive_buffer[modbus_rtu.receive_buffer_len] = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
                modbus_rtu.receive_buffer_len++;
                break;
            case MODBUS_RTU_WAIT:
                modbus_rtu_timer_3_5_enable();
                dummy_read = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
                break;
            default:
                dummy_read = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
                break;
            }
        }
        else
        {
            modbus_rtu_timers_disable();
            dummy_read = (uint8_t)(uart_get_hw(MODBUS_UART)->dr);
            memset((uint8_t *)modbus_rtu.receive_buffer, 0, sizeof(modbus_rtu.receive_buffer));
            modbus_rtu.receive_buffer_len = 0;
            modbus_rtu.state = MODBUS_RTU_IDLE;
        }
    }
}

static uint8_t modbus_rtu_uart_init(void)
{
    if (!uart_is_enabled(MODBUS_UART))
    {
        uart_init(MODBUS_UART, modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
        gpio_set_function(modbus_rtu.uart.uart_rx_pin, GPIO_FUNC_UART);
        gpio_set_function(modbus_rtu.uart.uart_tx_pin, GPIO_FUNC_UART);
        uart_set_hw_flow(MODBUS_UART, false, false);
        uart_set_fifo_enabled(MODBUS_UART, false);

        irq_set_exclusive_handler(MODBUS_UART_IRQ, uart_isr_handler);
        irq_set_priority(MODBUS_UART_IRQ, 0);
        irq_set_enabled(MODBUS_UART_IRQ, true);

        gpio_init(modbus_rtu.uart.uart_dir_pin);
        gpio_set_dir(modbus_rtu.uart.uart_dir_pin, GPIO_OUT);
        // gpio_pull_down(MODBUS_UART_DIR_PIN);
        gpio_put(modbus_rtu.uart.uart_dir_pin, MODBUS_DIRECTION_RX);
    }

    uart_set_baudrate(MODBUS_UART, modbus_baudrate_2_number(modbus_rtu.uart.baudrate));
    uart_set_format(MODBUS_UART, 8, 1, UART_PARITY_NONE);
    return 0;
}

bool modbus_rtu_init(TaskHandle_t task_handle,
                     uint8_t slave_address,
                     modbus_baudrates_t baudrate,
                     uint8_t uart_rx_pin,
                     uint8_t uart_tx_pin,
                     uint8_t uart_dir_pin,
                     ModbusRegisterCallback registerCallback,
                     ModbusSlaveExceptionCallback exceptionCallback,
                     modbus_error_handler error_handler)
{
    if (task_handle == NULL || registerCallback == NULL || exceptionCallback == NULL || error_handler == NULL || slave_address == 0 || slave_address > 247)
    {
        return true;
    }
    if (modbus_port_init((modbus_t *)&(modbus_rtu.modbus), registerCallback, exceptionCallback, error_handler))
    {
        return true;
    }
    modbus_rtu.task_handle = task_handle;
    modbus_rtu.slave_address = slave_address;
    modbus_rtu.uart.baudrate = baudrate;
    modbus_rtu.uart.uart_rx_pin = uart_rx_pin;
    modbus_rtu.uart.uart_tx_pin = uart_tx_pin;
    modbus_rtu.uart.uart_dir_pin = uart_dir_pin;
    if (modbus_rtu_timers_init())
    {
        return 1;
    }

    if (modbus_rtu_uart_init())
    {
        return 1;
    }

    modbus_rtu.state = MODBUS_RTU_INIT;
    modbus_rtu_uart_irq_set(MODBUS_IRQ_RX_ON);
    modbus_rtu_timer_3_5_enable();

    return 0;
}

bool modbus_rtu_deinit(void)
{
    if (cancel_repeating_timer((repeating_timer_t *)&(modbus_rtu.timers.repeating_timer)) == true)
    {
        modbus_rtu_uart_irq_set(MODBUS_IRQ_ALL_OFF);
        modbus_rtu.task_handle = NULL;
        modbus_rtu.uart.baudrate = 0;
        modbus_rtu.uart.uart_rx_pin = 0;
        modbus_rtu.uart.uart_tx_pin = 0;
        modbus_rtu.uart.uart_dir_pin = 0;
        return false;
    }
    else
    {
        return true;
    }
}

static void on_read_ready(void)
{
    modbus_rtu.modbus.err = modbusParseRequestRTU((ModbusSlave *)&(modbus_rtu.modbus.slave), modbus_rtu.slave_address, (uint8_t *)modbus_rtu.receive_buffer, modbus_rtu.receive_buffer_len);
    memset((uint8_t *)modbus_rtu.receive_buffer, 0, sizeof(modbus_rtu.receive_buffer));
    modbus_rtu.receive_buffer_len = 0;
    if (modbusIsOk(modbus_rtu.modbus.err))
    {
        const uint8_t *send_buffer_pointer = modbusSlaveGetResponse((ModbusSlave *)&(modbus_rtu.modbus.slave));
        modbus_rtu.send_buffer_len = modbusSlaveGetResponseLength((ModbusSlave *)&(modbus_rtu.modbus.slave));
        memcpy((uint8_t *)modbus_rtu.send_buffer, send_buffer_pointer, modbus_rtu.send_buffer_len);
        modbusSlaveFreeResponse((ModbusSlave *)&(modbus_rtu.modbus.slave));
    }
}

static void on_emit_ready(void)
{
    if (modbusIsOk(modbus_rtu.modbus.err))
    {
        modbus_rtu_uart_irq_set(MODBUS_IRQ_TX_ON);
    }
    else
    {
        modbus_rtu.state = MODBUS_RTU_IDLE;
    }
}

bool modbus_rtu_poll(void)
{
    if (notify_wait())
    {
        return true;
    }
    if (modbus_rtu.events & MODBUS_RTU_READ_READY && modbus_rtu.events & MODBUS_RTU_EMIT_READY)
    {
        on_read_ready();
        on_emit_ready();
    }
    else if (modbus_rtu.events & MODBUS_RTU_READ_READY)
    {
        on_read_ready();
    }
    else if (modbus_rtu.events & MODBUS_RTU_EMIT_READY)
    {
        on_emit_ready();
    }
    else
    {
        modbus_rtu.modbus.error_handler("MODBUS RTU wrong state");
    }
    return false;
}