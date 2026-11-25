/** @file tcp.h
 * @brief Host stub of the lwIP raw TCP API used by mqtt_utk.c. The test
 *        implements each function: it records tcp_write() output, keeps
 *        the registered callbacks and feeds broker bytes (FR15).
 *        Not the real lwIP header.
 */
#ifndef CT_LWIP_TCP_H
#define CT_LWIP_TCP_H
#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/pbuf.h"

#define TCP_WRITE_FLAG_COPY (0x01U)
/* lwIP default for TCP_SND_BUF 2 * 1460 (lwipopts.h). */
#define TCP_SND_QUEUELEN (8U)

struct netif;

/** Opaque connection; the test owns the single instance. */
struct tcp_pcb
{
    u8_t id;
};

typedef err_t (*tcp_recv_fn)(void * p_arg, struct tcp_pcb * p_pcb,
                             struct pbuf * p_buf, err_t err);
typedef err_t (*tcp_connected_fn)(void * p_arg, struct tcp_pcb * p_pcb,
                                  err_t err);
typedef void (*tcp_err_fn)(void * p_arg, err_t err);
typedef err_t (*tcp_sent_fn)(void * p_arg, struct tcp_pcb * p_pcb,
                             u16_t len);

/** @brief New connection, or NULL when the test simulates no memory. */
struct tcp_pcb * tcp_new(void);
/** @brief Set the callback argument. */
void tcp_arg(struct tcp_pcb * p_pcb, void * p_arg);
/** @brief Register (or clear with NULL) the receive callback. */
void tcp_recv(struct tcp_pcb * p_pcb, tcp_recv_fn recv);
/** @brief Register (or clear with NULL) the error callback. */
void tcp_err(struct tcp_pcb * p_pcb, tcp_err_fn err);
/** @brief Register (or clear with NULL) the sent callback. */
void tcp_sent(struct tcp_pcb * p_pcb, tcp_sent_fn sent);
/** @brief Bind the connection to one interface. */
void tcp_bind_netif(struct tcp_pcb * p_pcb, struct netif const * p_netif);
/** @brief Start connecting; the test calls connected() later. */
err_t tcp_connect(struct tcp_pcb * p_pcb, ip_addr_t const * p_ip,
                  u16_t port, tcp_connected_fn connected);
/** @brief Queue bytes for sending; the test records them. */
err_t tcp_write(struct tcp_pcb * p_pcb, void const * p_data, u16_t len,
                u8_t flags);
/** @brief Send queued bytes now. */
err_t tcp_output(struct tcp_pcb * p_pcb);
/** @brief Close gracefully. */
err_t tcp_close(struct tcp_pcb * p_pcb);
/** @brief Abort (RST) after a failed close. */
void tcp_abort(struct tcp_pcb * p_pcb);
/** @brief Reopen the receive window by len bytes. */
void tcp_recved(struct tcp_pcb * p_pcb, u16_t len);
/** @brief Free bytes in the send buffer (a macro in lwIP). */
u16_t tcp_sndbuf(struct tcp_pcb const * p_pcb);
/** @brief Segments queued for sending (a macro in lwIP). */
u16_t tcp_sndqueuelen(struct tcp_pcb const * p_pcb);

#endif /* CT_LWIP_TCP_H */

/*** end of file ***/

