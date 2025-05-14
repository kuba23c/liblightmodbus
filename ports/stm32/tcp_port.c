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
	modbus_tcp_stats_t stats;
	bool inited;
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
	modbus_tcp.stats.clients_errors++;
	if (modbus_tcp.stats.clients_connected) {
		modbus_tcp.stats.clients_connected--;
	}
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
	modbus_tcp.stats.listeners_errors++;
	if (modbus_tcp.stats.listeners_active) {
		modbus_tcp.stats.listeners_active--;
	}
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
		modbus_tcp.stats.clients_closed++;
		modbus_tcp_client_clean(client);
		if (tcp_close(tpcb) != ERR_OK) {
			tcp_abort(tpcb);
			return (ERR_ABRT);
		} else {
			if (modbus_tcp.stats.clients_connected) {
				modbus_tcp.stats.clients_connected--;
			}
			modbus_tcp.stats.clients_closed++;
		}
		return (ERR_OK);
	}
	struct pbuf *p_temp = p;
	uint16_t len = p->tot_len;

	while (p_temp) {
		modbus_tcp.stats.messages_received++;
		client->modbus.err = modbusParseRequestTCP(&(client->modbus.slave), (uint8_t*) p_temp->payload, p_temp->len);
		if (modbusIsOk(client->modbus.err)) {
			modbus_tcp.stats.messages_ok++;
			const uint8_t *send_buffer_pointer = modbusSlaveGetResponse(&(client->modbus.slave));
			uint16_t send_buffer_len = modbusSlaveGetResponseLength(&(client->modbus.slave));
			tcp_write(tpcb, send_buffer_pointer, send_buffer_len, TCP_WRITE_FLAG_COPY);
			modbus_tcp.stats.messages_sent++;
			modbusSlaveFreeResponse(&(client->modbus.slave));
		} else {
			modbus_tcp.stats.messages_nok++;
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
		modbus_tcp.stats.clients_timeouts++;
		modbus_tcp_client_clean(client);
		if (tcp_close(tpcb) != ERR_OK) {
			tcp_abort(tpcb);
			return (ERR_ABRT);
		} else {
			if (modbus_tcp.stats.clients_connected) {
				modbus_tcp.stats.clients_connected--;
			}
			modbus_tcp.stats.clients_closed++;
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
			} else {
				modbus_tcp.stats.modbus_internal_errors++;
			}
			break;
		}
	}
	if (result == ERR_ABRT) {
		tcp_abort(newpcb);
		modbus_tcp.stats.clients_rejected++;
	} else {
		modbus_tcp.stats.clients_accepted++;
		modbus_tcp.stats.clients_connected++;
	}
	return (result);
}

static void modbus_tcp_create_listener(void *ctx) {
	UNUSED(ctx);
	if (modbus_tcp.listener_pcb == NULL) {
		modbus_tcp.listener_pcb = tcp_new();
		if (modbus_tcp.listener_pcb == NULL) {
			modbus_tcp.stats.listeners_tcp_stack_error++;
			return;
		}
		if (tcp_bind(modbus_tcp.listener_pcb, IP4_ADDR_ANY, MODBUS_TCP_PORT_DEFAULT) != ERR_OK) {
			modbus_tcp.stats.listeners_tcp_stack_error++;
			tcp_close(modbus_tcp.listener_pcb);
			return;
		}
		tcp_err(modbus_tcp.listener_pcb, modbus_tcp_listener_on_error);
		modbus_tcp.listener_pcb = tcp_listen(modbus_tcp.listener_pcb);
		if (modbus_tcp.listener_pcb == NULL) {
			modbus_tcp.stats.listeners_tcp_stack_error++;
			return;
		}
		tcp_arg(modbus_tcp.listener_pcb, &modbus_tcp.listener_pcb);
		tcp_accept(modbus_tcp.listener_pcb, modbus_tcp_on_accept);
		modbus_tcp.stats.listeners_active++;
		modbus_tcp.stats.listeners_opened++;
	} else {
		modbus_tcp.stats.listeners_rejected++;
	}
}

static void modbus_tcp_delete_listener(void *ctx) {
	UNUSED(ctx);
	if (modbus_tcp.listener_pcb) {
		if (tcp_close(modbus_tcp.listener_pcb) != ERR_OK) {
			tcp_abort(modbus_tcp.listener_pcb);
		} else {
			if (modbus_tcp.stats.listeners_active) {
				modbus_tcp.stats.listeners_active--;
			}
			modbus_tcp.stats.listeners_closed++;
		}
	}
}

/**
 * @brief Init modbus tcp
 * call only once
 */
void modbus_tcp_init(void) {
	if (!modbus_tcp.inited) {
		modbus_tcp.inited = true;
		modbus_tcp.stats.clients_max = MODBUS_TCP_MAX_CLIENTS;
		modbus_tcp.stats.listeners_max = 1;
		modbus_tcp.create_listener = tcpip_callbackmsg_new(modbus_tcp_create_listener, NULL);
		assert_param(modbus_tcp.create_listener != NULL);
		modbus_tcp.delete_listener = tcpip_callbackmsg_new(modbus_tcp_delete_listener, NULL);
		assert_param(modbus_tcp.delete_listener != NULL);
	}
}

/**
 * @brief Start modbus tcp
 */
bool modbus_tcp_start(void) {
	return (tcpip_callbackmsg_trycallback(modbus_tcp.create_listener) == ERR_OK);
}

/**
 * @brief Stop modbus tcp
 */
bool modbus_tcp_stop(void) {
	return (tcpip_callbackmsg_trycallback(modbus_tcp.delete_listener) == ERR_OK);
}

/**
 * @brief get stats of modbus tcp
 * @return pointer to stats
 */
const modbus_tcp_stats_t* modbus_tcp_stats(void) {
	return (&modbus_tcp.stats);
}

/**
 * @brief clear modbus stats
 * Do it only when modbus tcp is NOT active
 */
void modbus_tcp_clear_stats(void) {
	memset(&modbus_tcp.stats, 0, sizeof(modbus_tcp_stats_t));
}

/**
 * @brief check if modbus tcp is active
 * It is NOT set in modbus_tcp_start but some time later when lwip stack will pick up a message,
 * so you have to wait a litte after modbus_tcp_start call
 * @return
 */
bool modbus_tcp_is_active(void) {
	if (modbus_tcp.stats.listeners_active) {
		return (true);
	} else {
		return (false);
	}
}
