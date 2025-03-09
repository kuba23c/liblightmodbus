/**
 * file: tcp_port.c
 * Author: kuba23c
 * Description: Lightmodbus TCP port for STM32 + CMSIS RTOSv2 + FreeRTOS + LwIP
 */

#include "tcp_port.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lwip.h"
#include "tcp.h"
#include "modbus_callbacks.h"

#define MODBUS_TCP_PORT_DEFAULT 502
#define MODBUS_TCP_REC_MESSAGE_MAX_SIZE 260 // 7+1+252
#define MODBUS_TCP_SEND_MESSAGE_MAX_SIZE 260
#define MODBUS_TCP_REC_MULT 4
#define MODBUS_TCP_MAX_CLIENTS 2
#define MODBUS_TCP_MAX_IDLE_SEC	3

typedef struct {
	modbus_t modbus;
	struct tcp_pcb *client_pcb;
	uint8_t idle_cnt;
} modbus_tcp_client_t;

typedef struct {
	struct tcp_pcb *listener_pcb;
	modbus_tcp_client_t clients[MODBUS_TCP_MAX_CLIENTS];
	struct tcpip_callback_msg *create_listener;
	struct tcpip_callback_msg *delete_listener;
} modbus_tcp_t;

static modbus_tcp_t modbus_tcp = { 0 };

#define MODBUS_TCP_KEEP_IDLE 3000
#define MODBUS_TCP_KEEP_INTVL 1000
#define MODBUS_TCP_KEEP_CNT 3

static void modbus_tcp_client_clean(modbus_tcp_client_t *client) {
	memset(client, 0, sizeof(modbus_tcp_client_t));
}

/** Function prototype for tcp error callback functions. Called when the pcb
 * receives a RST or is unexpectedly closed for any other reason.
 *
 * @note The corresponding pcb is already freed when this callback is called!
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param err Error code to indicate why the pcb has been closed
 *            ERR_ABRT: aborted through tcp_abort or by a TCP timer
 *            ERR_RST: the connection was reset by the remote host
 */
static void modbus_tcp_client_on_error(void *arg, err_t err) {
	UNUSED(err);
	modbus_tcp_client_t *client = (modbus_tcp_client_t*) arg;
	modbus_tcp_client_clean(client);
}

/** Function prototype for tcp error callback functions. Called when the pcb
 * receives a RST or is unexpectedly closed for any other reason.
 *
 * @note The corresponding pcb is already freed when this callback is called!
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param err Error code to indicate why the pcb has been closed
 *            ERR_ABRT: aborted through tcp_abort or by a TCP timer
 *            ERR_RST: the connection was reset by the remote host
 */
static void modbus_tcp_listener_on_error(void *arg, err_t err) {
	UNUSED(err);
	struct tcp_pcb **ptcp_pcb = (struct tcp_pcb**) arg;
	*ptcp_pcb = NULL;
}

/** Function prototype for tcp receive callback functions. Called when data has
 * been received.
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param tpcb The connection pcb which received data
 * @param p The received data (or NULL when the connection has been closed!)
 * @param err An error code if there has been an error receiving
 *            Only return ERR_ABRT if you have called tcp_abort from within the
 *            callback function!
 */
static err_t modbus_tcp_on_receive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
	modbus_tcp_client_t *client = (modbus_tcp_client_t*) arg;

	if (err) {
		modbus_tcp_client_clean(client);
		tcp_abort(tpcb);
		return (ERR_ABRT);
	}
	if (p == NULL) {
		modbus_tcp_client_clean(client);
		if (tcp_close(tpcb) != ERR_OK) {
			tcp_abort(tpcb);
			return (ERR_ABRT);
		}
		return (ERR_OK);
	}
	struct pbuf *p_temp = p;
	uint16_t len = p->tot_len;

	while (p_temp) {
		client->modbus.err = modbusParseRequestTCP(&(client->modbus.slave), (uint8_t*) p_temp->payload, p_temp->len);
		if (modbusIsOk(client->modbus.err)) {
			const uint8_t *send_buffer_pointer = modbusSlaveGetResponse(&(client->modbus.slave));
			uint16_t send_buffer_len = modbusSlaveGetResponseLength(&(client->modbus.slave));
			tcp_write(tpcb, send_buffer_pointer, send_buffer_len,
			TCP_WRITE_FLAG_COPY);
			modbusSlaveFreeResponse(&(client->modbus.slave));
		}
		p_temp = p_temp->next;
	}
	pbuf_free(p);
	tcp_recved(tpcb, len);
	return (ERR_OK);
}

/** Function prototype for tcp poll callback functions. Called periodically as
 * specified by @see tcp_poll.
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param tpcb tcp pcb
 * @return ERR_OK: try to send some data by calling tcp_output
 *            Only return ERR_ABRT if you have called tcp_abort from within the
 *            callback function!
 */
static err_t modbus_tcp_on_poll(void *arg, struct tcp_pcb *tpcb) {
	modbus_tcp_client_t *client = (modbus_tcp_client_t*) arg;

	client->idle_cnt++;
	if (client->idle_cnt >= MODBUS_TCP_MAX_IDLE_SEC) {
		modbus_tcp_client_clean(client);
		if (tcp_close(tpcb) != ERR_OK) {
			tcp_abort(tpcb);
			return (ERR_ABRT);
		}
	}
	return (ERR_OK);
}

/** Function prototype for tcp accept callback functions. Called when a new
 * connection can be accepted on a listening pcb.
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param newpcb The new connection pcb
 * @param err An error code if there has been an error accepting.
 *            Only return ERR_ABRT if you have called tcp_abort from within the
 *            callback function!
 */
static err_t modbus_tcp_on_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
	UNUSED(arg);
	UNUSED(err);
	err_t result = ERR_ABRT;
	for (int i = 0; i < MODBUS_TCP_MAX_CLIENTS; ++i) {
		if (modbus_tcp.clients[i].client_pcb == NULL) {
			if (!modbus_port_init(&(modbus_tcp.clients[i].modbus))) {
				newpcb->keep_idle = MODBUS_TCP_KEEP_IDLE;
				newpcb->keep_intvl = MODBUS_TCP_KEEP_INTVL;
				newpcb->keep_cnt = MODBUS_TCP_KEEP_CNT;
				tcp_nagle_disable(newpcb);
				modbus_tcp.clients[i].client_pcb = newpcb;
				modbus_tcp.clients[i].idle_cnt = 0;
				tcp_arg(modbus_tcp.clients[i].client_pcb, &modbus_tcp.clients[i]);
				tcp_err(modbus_tcp.clients[i].client_pcb, modbus_tcp_client_on_error);
				tcp_poll(modbus_tcp.clients[i].client_pcb, modbus_tcp_on_poll, 2);
				tcp_recv(modbus_tcp.clients[i].client_pcb, modbus_tcp_on_receive);
				result = ERR_OK;
			}
			break;
		}
	}
	if (result == ERR_ABRT) {
		tcp_abort(newpcb);
	}
	return (result);
}

static void modbus_tcp_create_listener(void *ctx) {
	UNUSED(ctx);
	if (modbus_tcp.listener_pcb == NULL) {
		modbus_tcp.listener_pcb = tcp_new();
		assert_param(modbus_tcp.listener_pcb != NULL);
		assert_param(tcp_bind(modbus_tcp.listener_pcb, IP4_ADDR_ANY, MODBUS_TCP_PORT_DEFAULT) == ERR_OK);
		tcp_err(modbus_tcp.listener_pcb, modbus_tcp_listener_on_error);
		modbus_tcp.listener_pcb = tcp_listen(modbus_tcp.listener_pcb);
		assert_param(modbus_tcp.listener_pcb != NULL);
		tcp_arg(modbus_tcp.listener_pcb, &modbus_tcp.listener_pcb);
		tcp_accept(modbus_tcp.listener_pcb, modbus_tcp_on_accept);
	}
}

static void modbus_tcp_delete_listener(void *ctx) {
	UNUSED(ctx);
	if (modbus_tcp.listener_pcb) {
		if (tcp_close(modbus_tcp.listener_pcb) != ERR_OK) {
			tcp_abort(modbus_tcp.listener_pcb);
		}
	}
}

/**
 * @brief Init modbus tcp
 * @return 0 - OK, 1 - NOK
 */
uint8_t modbus_tcp_init(void) {
	if (modbus_tcp.create_listener == NULL) {
		modbus_tcp.create_listener = tcpip_callbackmsg_new(modbus_tcp_create_listener, NULL);
		assert_param(modbus_tcp.create_listener != NULL);
		if (modbus_tcp.delete_listener == NULL) {
			modbus_tcp.delete_listener = tcpip_callbackmsg_new(modbus_tcp_delete_listener, NULL);
			assert_param(modbus_tcp.delete_listener != NULL);
		}
		assert_param(tcpip_callbackmsg_trycallback(modbus_tcp.create_listener) == ERR_OK);
		return (0);
	} else {
		assert_param(tcpip_callbackmsg_trycallback(modbus_tcp.create_listener) == ERR_OK);
		return (1);
	}
}

/**
 * @brief Deinit modbus tcp
 * @return 0 - OK, 1 - NOK
 */
void modbus_tcp_deinit(void) {
	assert_param(tcpip_callbackmsg_trycallback(modbus_tcp.delete_listener) == ERR_OK);
}
