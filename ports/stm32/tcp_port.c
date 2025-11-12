/**
 * file: tcp_port.c
 * Author: kuba23c
 * Description: Lightmodbus TCP port for STM32 + CMSIS RTOSv2 + FreeRTOS + LwIP
 *
 * IMPORTANT: required LwRB v3.2.0
 */

#include "tcp_port.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "lwip.h"
#include "tcp.h"
#include "modbus_callbacks.h"
#include "lwrb.h"

#define MODBUS_TCP_REC_MESSAGE_MAX_SIZE 260 // 7+1+252
#define MODBUS_TCP_REC_MESSAGE_MIN_SIZE 8
#define MODBUS_TCP_SEND_MESSAGE_MAX_SIZE 260
#define MODBUS_TCP_REC_MULT 4
#define MODBUS_TCP_MAX_IDLE_SEC	3
#define MODBUS_TCP_LEN_HIGHER_BYTE	4
#define MODBUS_TCP_LEN_LOWER_BYTE	5
#define MODBUS_TCP_HEADER_LEN		6
#define MODBUS_TCP_RING_BUFFER_SIZE	((uint32_t)(3 * 1024)) // you can tweak this value till have messages_ring_buffer_full

typedef struct {
	modbus_t modbus;
	struct tcp_pcb *client_pcb;
	uint8_t idle_cnt;
	uint8_t buff[MODBUS_TCP_REC_MESSAGE_MAX_SIZE];
	uint32_t buff_len;
	uint8_t ring_buff[MODBUS_TCP_RING_BUFFER_SIZE];
	lwrb_t lwrb;
	bool response_sent;
	bool is_ok;
} modbus_tcp_client_t;

typedef struct {
	struct tcp_pcb *listener_pcb;
	modbus_tcp_client_t clients[MODBUS_TCP_MAX_CLIENTS];
	struct tcpip_callback_msg *create_listener;
	struct tcpip_callback_msg *delete_listener;
	modbus_tcp_stats_t stats;
	uint16_t port;
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
		modbus_tcp.stats.clients_closed++;
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

static bool handle_modbus_data(modbus_tcp_client_t *const client, struct tcp_pcb *tpcb, const uint8_t *const buff, uint16_t len) {
	client->modbus.err = modbusParseRequestTCP(&(client->modbus.slave), buff, len);
	if (modbusIsOk(client->modbus.err)) {
		modbus_tcp.stats.messages_received++;
		modbus_tcp.stats.messages_ok++;
		const uint8_t *send_buffer_pointer = modbusSlaveGetResponse(&(client->modbus.slave));
		uint16_t send_buffer_len = modbusSlaveGetResponseLength(&(client->modbus.slave));

		if (lwrb_get_free(&(client->lwrb)) < send_buffer_len) {
			modbus_tcp.stats.messages_ring_buffer_full++;
			modbusSlaveFreeResponse(&(client->modbus.slave));
			return false;
		} else {
			lwrb_write(&(client->lwrb), send_buffer_pointer, send_buffer_len);
			modbus_tcp.stats.messages_sent++;
			modbusSlaveFreeResponse(&(client->modbus.slave));
			client->is_ok = true;
			return true;
		}
	} else {
		client->is_ok = false;
		client->buff_len = 0;
		modbus_tcp.stats.messages_received++;
		modbus_tcp.stats.messages_nok++;
		return false;
	}
}

static bool handle_normal_data(modbus_tcp_client_t *const client, struct tcp_pcb *tpcb, uint8_t **const payload, uint16_t *const len) {
	uint16_t id_and_pdu_len = (((uint16_t) (*payload)[MODBUS_TCP_LEN_HIGHER_BYTE]) << 8) | ((uint16_t) (*payload)[MODBUS_TCP_LEN_LOWER_BYTE]);
	uint16_t real_len = MODBUS_TCP_HEADER_LEN + id_and_pdu_len;
	if (real_len > MODBUS_TCP_REC_MESSAGE_MAX_SIZE) {
		client->is_ok = false;
		client->buff_len = 0;
		modbus_tcp.stats.messages_received++;
		modbus_tcp.stats.messages_nok++;
		return false;
	}
	if (real_len > *len) {
		if (client->is_ok && client->buff_len == 0) {
			memcpy(client->buff, *payload, *len);
			client->buff_len = *len;
		} else {
			client->is_ok = false;
			client->buff_len = 0;
			modbus_tcp.stats.messages_received++;
			modbus_tcp.stats.messages_nok++;
		}
		return false;
	}
	if (!handle_modbus_data(client, tpcb, *payload, real_len)) {
		return false;
	}
	*len -= real_len;
	*payload += real_len;
	return true;
}

static bool handle_prev_data(modbus_tcp_client_t *client, struct tcp_pcb *tpcb, uint8_t **const payload, uint16_t *const len) {
	uint16_t to_copy_bytes = MODBUS_TCP_REC_MESSAGE_MAX_SIZE - client->buff_len;
	if (to_copy_bytes > *len) {
		to_copy_bytes = *len;
	}
	memcpy(client->buff + client->buff_len, *payload, to_copy_bytes);
	uint16_t id_and_pdu_len = (((uint16_t) client->buff[MODBUS_TCP_LEN_HIGHER_BYTE]) << 8) | ((uint16_t) client->buff[MODBUS_TCP_LEN_LOWER_BYTE]);
	uint16_t real_len = MODBUS_TCP_HEADER_LEN + id_and_pdu_len;
	if (real_len > MODBUS_TCP_REC_MESSAGE_MAX_SIZE) {
		client->is_ok = false;
		client->buff_len = 0;
		modbus_tcp.stats.messages_received++;
		modbus_tcp.stats.messages_nok++;
		return false;
	}
	if (real_len > client->buff_len + *len) {
		client->is_ok = false;
		client->buff_len = 0;
		modbus_tcp.stats.messages_received++;
		modbus_tcp.stats.messages_nok++;
		return false;
	}
	if (!handle_modbus_data(client, tpcb, client->buff, real_len)) {
		return false;
	}
	uint16_t bytes_to_remove = real_len - client->buff_len;
	*len -= bytes_to_remove;
	*payload += bytes_to_remove;
	client->buff_len = 0;
	return true;
}

static void send_data_from_ring_buffer(modbus_tcp_client_t *client, struct tcp_pcb *tpcb) {
	lwrb_sz_t to_send = lwrb_get_linear_block_read_length(&(client->lwrb));
	if (to_send) {
		uint32_t available_free_space = tcp_sndbuf(tpcb);
		if (available_free_space < to_send) {
			to_send = available_free_space;
		}
		client->idle_cnt = 0;
		assert_param(tcp_write(tpcb, lwrb_get_linear_block_read_address(&(client->lwrb)), to_send, 0) == ERR_OK);
		client->response_sent = true;
	} else {
		client->response_sent = false;
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

	if (err || client->client_pcb != tpcb) {
		modbus_tcp_client_clean(client);
		if (p != NULL) {
			pbuf_free(p);
		}
		tcp_abort(tpcb);
		return (ERR_ABRT);
	}
	if (p == NULL) {
		modbus_tcp_client_clean(client);
		if (tcp_close(tpcb) != ERR_OK) {
			tcp_abort(tpcb);
			return (ERR_ABRT);
		} else {
			if (modbus_tcp.stats.clients_connected) {
				modbus_tcp.stats.clients_connected--;
				modbus_tcp.stats.clients_closed++;
			}
		}
		return (ERR_OK);
	}

	client->idle_cnt = 0;
	for (struct pbuf *p_temp = p; p_temp != NULL; p_temp = p_temp->next) {
		uint8_t *payload = (uint8_t*) p_temp->payload;
		uint16_t len = p_temp->len;
		while (len) {
			if (client->buff_len + len < MODBUS_TCP_REC_MESSAGE_MIN_SIZE) {
				if (client->is_ok && client->buff_len == 0) {
					memcpy(client->buff, payload, len);
					client->buff_len = len;
				} else {
					client->is_ok = false;
					client->buff_len = 0;
					modbus_tcp.stats.messages_received++;
					modbus_tcp.stats.messages_nok++;
				}
				break;
			}
			if (client->buff_len) {
				if (!handle_prev_data(client, tpcb, &payload, &len)) {
					if (!handle_normal_data(client, tpcb, &payload, &len)) {
						break;
					}
				}
			} else {
				if (!handle_normal_data(client, tpcb, &payload, &len)) {
					break;
				}
			}
		}
	}
	if (!client->response_sent) {
		send_data_from_ring_buffer(client, tpcb);
	}
	tcp_recved(tpcb, p->tot_len);
	pbuf_free(p);
	return (ERR_OK);
}

/** Function prototype for tcp sent callback functions. Called when sent data has
 * been acknowledged by the remote side. Use it to free corresponding resources.
 * This also means that the pcb has now space available to send new data.
 *
 * @param arg Additional argument to pass to the callback function (@see tcp_arg())
 * @param tpcb The connection pcb for which data has been acknowledged
 * @param len The amount of bytes acknowledged
 * @return ERR_OK: try to send some data by calling tcp_output
 *            Only return ERR_ABRT if you have called tcp_abort from within the
 *            callback function!
 */
static err_t modbus_tcp_on_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
	modbus_tcp_client_t *client = (modbus_tcp_client_t*) arg;

	lwrb_skip(&(client->lwrb), len);
	send_data_from_ring_buffer(client, tpcb);
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
				modbus_tcp.stats.clients_closed++;
			}
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
			if (!modbus_port_init(&(modbus_tcp.clients[i].modbus), "TCP", i + 1)) {
				newpcb->keep_idle = MODBUS_TCP_KEEP_IDLE;
				newpcb->keep_intvl = MODBUS_TCP_KEEP_INTVL;
				newpcb->keep_cnt = MODBUS_TCP_KEEP_CNT;
				tcp_nagle_disable(newpcb);
				modbus_tcp.clients[i].client_pcb = newpcb;
				modbus_tcp.clients[i].idle_cnt = 0;
				modbus_tcp.clients[i].is_ok = false;
				modbus_tcp.clients[i].buff_len = 0;
				modbus_tcp.clients[i].response_sent = false;
				lwrb_init(&(modbus_tcp.clients[i].lwrb), modbus_tcp.clients[i].ring_buff, MODBUS_TCP_RING_BUFFER_SIZE);
				tcp_arg(modbus_tcp.clients[i].client_pcb, &modbus_tcp.clients[i]);
				tcp_err(modbus_tcp.clients[i].client_pcb, modbus_tcp_client_on_error);
				tcp_poll(modbus_tcp.clients[i].client_pcb, modbus_tcp_on_poll, 2);
				tcp_sent(modbus_tcp.clients[i].client_pcb, modbus_tcp_on_sent);
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
		if (tcp_bind(modbus_tcp.listener_pcb, IP4_ADDR_ANY, modbus_tcp.port) != ERR_OK) {
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
	modbus_tcp.listener_pcb = NULL;
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
bool modbus_tcp_start(uint16_t port) {
	modbus_tcp.port = port;
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
const modbus_tcp_stats_t* MODBUS_TCP_GetStats(void) {
	return (&modbus_tcp.stats);
}

const modbus_exceptions_t* modbus_tcp_get_exceptions(uint8_t client_id) {
	if (client_id == 0 || client_id > MODBUS_TCP_MAX_CLIENTS) {
		return (NULL);
	} else {
		return ((modbus_exceptions_t*) &modbus_tcp.clients[client_id - 1].modbus.exceptions);
	}
}
/**
 * @brief clear modbus stats
 * Do it only when modbus tcp is NOT active
 */
void modbus_tcp_clear_stats(void) {
	memset(&modbus_tcp.stats, 0, sizeof(modbus_tcp_stats_t));
}

void modbus_tcp_clear_exceptions(uint8_t client_id) {
	if (client_id == 0 || client_id > MODBUS_TCP_MAX_CLIENTS) {
		return;
	} else {
		memset((modbus_exceptions_t*) &modbus_tcp.clients[client_id - 1].modbus.exceptions, 0, sizeof(modbus_exceptions_t));
	}
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
