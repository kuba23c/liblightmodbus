#include "tcp_port.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "socket.h"

static char *mb_tcp_listening = "MB-TCP Listening...\r\n";
static char *mb_tcp_disconnected = "MB-TCP Disconnected !!!\r\n";
static char *mb_tcp_timeout = "MB-TCP Timeout !!!\r\n";
static char *mb_tcp_busy = "Modbus BUSY \r\n";
static char print_poll_buffer[200] = {0};
static uint32_t buffer_size = 0;

static uint8_t modbus_tcp_socket_init(modbus_tcp_t *const modbus_tcp)
{
    modbus_tcp->receive_buffer_len = 0;
    modbus_tcp->send_buffer_len = 0;
    memset(modbus_tcp->receive_buffer, 0, sizeof(modbus_tcp->receive_buffer));
    memset(modbus_tcp->send_buffer, 0, sizeof(modbus_tcp->send_buffer));

    if (socket(modbus_tcp->socket, Sn_MR_TCP, modbus_tcp->port, SF_TCP_NODELAY | SF_IO_NONBLOCK) != modbus_tcp->socket)
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket open fail.");
        }
        return 1;
    }
    uint8_t timeout = 1;
    setsockopt(modbus_tcp->socket, SO_KEEPALIVEAUTO, &timeout);
    if (listen(modbus_tcp->socket) != SOCK_OK)
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket listen fail.");
        }
        return 1;
    }
#ifdef MY_DEBUG
    modbus_tcp->modbus_debug(mb_tcp_listening, strlen(mb_tcp_listening));
#endif
    return 0;
}

static uint8_t modbus_tcp_on_connect(modbus_tcp_t *const modbus_tcp)
{
#ifdef MY_DEBUG
    uint8_t destination_ip[4] = {0};
    getsockopt(modbus_tcp->socket, SO_DESTIP, destination_ip);
    buffer_size = sprintf(print_poll_buffer, "MB-TCP Accepted new client %d.%d.%d.%d\r\n",
                          destination_ip[0],
                          destination_ip[1],
                          destination_ip[2],
                          destination_ip[3]);
    modbus_tcp->modbus_debug(print_poll_buffer, buffer_size);
    memset(print_poll_buffer, 0, buffer_size);
#endif
    return 0;
}

static uint8_t modbus_tcp_on_disconnect(modbus_tcp_t *const modbus_tcp)
{
#ifdef MY_DEBUG
    modbus_tcp->modbus_debug(mb_tcp_disconnected, strlen(mb_tcp_disconnected));
#endif
    // disconnect(modbus_tcp->socket); //TODO it should not be used here as at this point we are already disconnected
    if (modbus_tcp_socket_init(modbus_tcp))
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket init fail.");
        }
        return 1;
    }
    return 0;
}

static uint16_t modbus_tcp_buffer_find_data(modbus_tcp_t *const modbus_tcp)
{
    if (!(modbus_tcp->receive_buffer_len))
    {
        return 0;
    }
    if (modbus_tcp->send_buffer_len != 0)
    {
        return 0;
    }
    if (modbus_tcp->receive_buffer[2] != 0 || modbus_tcp->receive_buffer[3] != 0 || modbus_tcp->receive_buffer[4] != 0 || modbus_tcp->receive_buffer[5] > 254)
    {
        memset(modbus_tcp->receive_buffer, 0, sizeof(modbus_tcp->receive_buffer));
        modbus_tcp->receive_buffer_len = 0;
        return 0;
    }
    return (uint16_t)((modbus_tcp->receive_buffer[5]) + 6);
}

static uint8_t modbus_tcp_on_timeout(modbus_tcp_t *const modbus_tcp)
{
#ifdef MY_DEBUG
    modbus_tcp->modbus_debug(mb_tcp_timeout, strlen(mb_tcp_timeout));
#endif
    close(modbus_tcp->socket);
    if (modbus_tcp_socket_init(modbus_tcp))
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket init fail.");
        }
        return 1;
    }
    return 0;
}

static void modbus_tcp_on_sent(modbus_tcp_t *const modbus_tcp)
{
    if (modbus_tcp->send_buffer_len == 0)
    {
        uint16_t data_len = modbus_tcp_buffer_find_data(modbus_tcp);

        if (data_len)
        {
#ifdef MY_DEBUG
            buffer_size = sprintf(print_poll_buffer, "MB-TCP Received: ");
            for (uint8_t i = 0; i < data_len; i++)
            {
                buffer_size += sprintf(print_poll_buffer + buffer_size, "%.02X", modbus_tcp->receive_buffer[i]);
                if (i == 6)
                {
                    buffer_size += sprintf(print_poll_buffer + buffer_size, " : ");
                }
                else if (i % 2)
                {
                    buffer_size += sprintf(print_poll_buffer + buffer_size, ".");
                }
            }
            buffer_size += sprintf(print_poll_buffer + buffer_size, "\r\n");
            modbus_tcp->modbus_debug(print_poll_buffer, buffer_size);
            memset(print_poll_buffer, 0, buffer_size);
#endif
            modbus_tcp->modbus.err = modbusParseRequestTCP(&(modbus_tcp->modbus.slave), modbus_tcp->receive_buffer, data_len);
            if (modbusIsOk(modbus_tcp->modbus.err))
            {
                const uint8_t *send_buffer_pointer = modbusSlaveGetResponse(&(modbus_tcp->modbus.slave));
                modbus_tcp->send_buffer_len = modbusSlaveGetResponseLength(&(modbus_tcp->modbus.slave));
                memcpy(modbus_tcp->send_buffer, send_buffer_pointer, modbus_tcp->send_buffer_len);
                modbusSlaveFreeResponse(&(modbus_tcp->modbus.slave));
                if (data_len > modbus_tcp->receive_buffer_len)
                {
                    // we should never reach here
                    memset(modbus_tcp->receive_buffer, 0, sizeof(modbus_tcp->receive_buffer));
                    modbus_tcp->receive_buffer_len = 0;
                }
                else
                {
                    memmove(modbus_tcp->receive_buffer, &(modbus_tcp->receive_buffer[data_len]), modbus_tcp->receive_buffer_len - data_len);
                    modbus_tcp->receive_buffer_len -= data_len;
                }
            }
            else
            {
                memset(modbus_tcp->receive_buffer, 0, sizeof(modbus_tcp->receive_buffer));
                modbus_tcp->receive_buffer_len = 0;
            }
        }
    }

    if (modbus_tcp->send_buffer_len)
    {
        int32_t res = send(modbus_tcp->socket, modbus_tcp->send_buffer, modbus_tcp->send_buffer_len);
        if (res == SOCK_BUSY)
        {
#ifdef MY_DEBUG
            modbus_tcp->modbus_debug(mb_tcp_busy, strlen(mb_tcp_busy));
#endif
            return;
        }
#ifdef MY_DEBUG
        buffer_size = sprintf(print_poll_buffer, "MB-TCP Send: ");
        for (uint8_t i = 0; i < modbus_tcp->send_buffer_len; i++)
        {
            buffer_size += sprintf(print_poll_buffer + buffer_size, "%.02X", modbus_tcp->send_buffer[i]);

            if (i == 6)
            {
                buffer_size += sprintf(print_poll_buffer + buffer_size, " : ");
            }
            else if (i % 2)
            {
                buffer_size += sprintf(print_poll_buffer + buffer_size, ".");
            }
        }
        buffer_size += sprintf(print_poll_buffer + buffer_size, "\r\n");
        modbus_tcp->modbus_debug(print_poll_buffer, buffer_size);
        memset(print_poll_buffer, 0, buffer_size);
#endif
        memset(modbus_tcp->send_buffer, 0, sizeof(modbus_tcp->send_buffer));
        modbus_tcp->send_buffer_len = 0;
    }
}

static uint8_t modbus_tcp_on_receive(modbus_tcp_t *const modbus_tcp)
{
    if (modbus_tcp->receive_buffer_len > sizeof(modbus_tcp->receive_buffer))
    {
        // we should never reach here
        memset(modbus_tcp->receive_buffer, 0, sizeof(modbus_tcp->receive_buffer));
        modbus_tcp->receive_buffer_len = 0;
        modbus_tcp->send_buffer_len = 0;
        memset(modbus_tcp->send_buffer, 0, sizeof(modbus_tcp->send_buffer));
        return 1;
    }
    else if (modbus_tcp->receive_buffer_len == sizeof(modbus_tcp->receive_buffer))
    {
        return 1;
    }
    else
    {
        uint16_t receive_len = 0;
        uint16_t available_space = sizeof(modbus_tcp->receive_buffer) - modbus_tcp->receive_buffer_len;
        getsockopt(modbus_tcp->socket, SO_RECVBUF, &receive_len);
        if (receive_len > 0)
        {
            if (receive_len > available_space)
            {
                receive_len = available_space;
            }
            int32_t real_len = recv(modbus_tcp->socket, &(modbus_tcp->receive_buffer[modbus_tcp->receive_buffer_len]), receive_len);

            if (real_len > 0)
            {
                modbus_tcp->receive_buffer_len += real_len;
                if (modbus_tcp->send_buffer_len == 0)
                {
                    modbus_tcp_on_sent(modbus_tcp);
                }
            }
        }

        return 0;
    }
}

void modbus_tcp_poll(modbus_tcp_t *const modbus_tcp)
{
    modbus_tcp->interrupts = 0;
    modbus_tcp->clear_interrupts = 0;

    ctlsocket(modbus_tcp->socket, CS_GET_INTERRUPT, &(modbus_tcp->interrupts));

    if (modbus_tcp->interrupts & SIK_CONNECTED)
    {
        if (!(modbus_tcp_on_connect(modbus_tcp)))
        {
            modbus_tcp->clear_interrupts |= SIK_CONNECTED;
        }
    }
    if (modbus_tcp->interrupts & SIK_DISCONNECTED)
    {
        if (!(modbus_tcp_on_disconnect(modbus_tcp)))
        {
            modbus_tcp->clear_interrupts |= SIK_DISCONNECTED;
        }
    }
    if (modbus_tcp->interrupts & SIK_RECEIVED)
    {
        if (!(modbus_tcp_on_receive(modbus_tcp)))
        {
            modbus_tcp->clear_interrupts |= SIK_RECEIVED;
        }
    }
    if (modbus_tcp->interrupts & SIK_TIMEOUT)
    {
        if (!(modbus_tcp_on_timeout(modbus_tcp)))
        {
            modbus_tcp->clear_interrupts |= SIK_TIMEOUT;
        }
    }
    if (modbus_tcp->interrupts & SIK_SENT)
    {
        modbus_tcp_on_sent(modbus_tcp);
    }

    ctlsocket(modbus_tcp->socket, CS_CLR_INTERRUPT, &(modbus_tcp->clear_interrupts));
}

uint8_t modbus_tcp_init(modbus_tcp_t *const modbus_tcp,
                        ModbusRegisterCallback registerCallback,
                        ModbusSlaveExceptionCallback exceptionCallback,
                        uint8_t modbus_tcp_socket,
                        uint16_t modbus_tcp_port,
                        modbus_error_handler error_handler,
                        modbus_debug_handler modbus_debug)
{
    modbus_tcp->modbus_debug = modbus_debug;
    if (modbus_port_init(&(modbus_tcp->modbus), registerCallback, exceptionCallback, error_handler))
    {
        return 1;
    }

    if (modbus_tcp_socket > 7)
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket > 7.");
        }
        return 1;
    }
    modbus_tcp->socket = modbus_tcp_socket;
    if (modbus_tcp_port)
    {
        modbus_tcp->port = modbus_tcp_port;
    }
    else
    {
        modbus_tcp->port = MODBUS_TCP_PORT_DEFAULT;
    }

    if (modbus_tcp_socket_init(modbus_tcp))
    {
        if (modbus_tcp->modbus.error_handler != NULL)
        {
            modbus_tcp->modbus.error_handler("Modbus TCP socket init fail.");
        }
        return 1;
    }
    return 0;
}