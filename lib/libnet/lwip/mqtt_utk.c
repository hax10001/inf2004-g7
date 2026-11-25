/** @file mqtt_utk.c
 * @brief Network-owner MQTT transport with bounded TX and RX buffers.
 *
 * The only MQTT/lwIP interface of the firmware. mqtt_utk_poll() runs in the
 * CYW43 service after lwIP is initialised; application tasks only offer and
 * take bounded copies through the b1_mqtt_*() functions. Each queue has one
 * producer and one consumer.
 */
#include "modules/telemetry/buddy1_mqtt.h"
#include "mqtt_utk.h"
#include <string.h>

#define B1_QUEUE_SIZE 8u
/* ACK is priority and may burst while historical telemetry drains. */
#define B1_ACK_QUEUE_SIZE 8u
/* STOP has its own queue so a full command queue cannot suppress it. */
#define B1_STOP_QUEUE_SIZE 2u

/** @brief Full memory barrier between queue producer and consumer. */
static inline void b1_barrier (void)
{
#if defined(__GNUC__)
    __sync_synchronize();
#endif
}

/** One queued MQTT payload: an encoded frame and its topic. */
typedef struct
{
    uint8_t bytes[B1_MAX_FRAME];
    uint8_t len;
    uint8_t topic_ack;
} b1_queued_t;

static b1_queued_t g_txq[B1_QUEUE_SIZE];
static b1_queued_t g_rxq[B1_QUEUE_SIZE];
static b1_queued_t g_ackq[B1_ACK_QUEUE_SIZE];
static b1_queued_t g_stopq[B1_STOP_QUEUE_SIZE];
static volatile uint8_t g_tx_head;
static volatile uint8_t g_tx_tail;
static volatile uint8_t g_rx_head;
static volatile uint8_t g_rx_tail;
static volatile uint8_t g_ack_head;
static volatile uint8_t g_ack_tail;
static volatile uint8_t g_stop_head;
static volatile uint8_t g_stop_tail;
static volatile uint8_t g_mqtt_link_ready;

/** @brief Copy a payload into a ring; -1 if the ring is full. */
static int32_t offer (b1_queued_t * p_queue, volatile uint8_t * p_head,
                      volatile uint8_t const * p_tail,
                      uint8_t const * p_bytes, uint8_t n, uint8_t topic_ack,
                      uint8_t capacity)
{
    uint8_t const next = (uint8_t)((*p_head + 1u) % capacity);

    if (next == *p_tail)
    {
        return -1;
    }
    p_queue[*p_head].len = n;
    p_queue[*p_head].topic_ack = topic_ack;
    memcpy(p_queue[*p_head].bytes, p_bytes, n);
    b1_barrier();
    *p_head = next;
    return 0;
}

/** @brief Take the oldest entry of a ring; 0 if it is empty. */
static int32_t take (b1_queued_t const * p_queue,
                     volatile uint8_t const * p_head,
                     volatile uint8_t * p_tail, b1_queued_t * p_out,
                     uint8_t capacity)
{
    if (*p_head == *p_tail)
    {
        return 0;
    }
    b1_barrier();
    *p_out = p_queue[*p_tail];
    b1_barrier();
    *p_tail = (uint8_t)((*p_tail + 1u) % capacity);
    return 1;
}

/** @brief Queue a frame for sending; -1 if the queue is full. */
int32_t b1_mqtt_submit (b1_frame_t const * p_frame)
{
    uint8_t buffer[B1_MAX_FRAME];
    size_t const n = b1_frame_encode(p_frame, buffer, sizeof buffer);
    int32_t result;

    if (0u == n)
    {
        return -1;
    }
    if (B1_TYPE_ACK == p_frame->type)
    {
        result = offer(g_ackq, &g_ack_head, &g_ack_tail, buffer, (uint8_t)n,
                       1u, B1_ACK_QUEUE_SIZE);
    }
    else
    {
        result = offer(g_txq, &g_tx_head, &g_tx_tail, buffer, (uint8_t)n,
                       0u, B1_QUEUE_SIZE);
    }
    return (0 != result) ? -1 : 0;
}

/** @brief Take a received command: 1 frame, 0 none, -1 bad frame. */
int32_t b1_mqtt_receive_command (b1_frame_t * p_frame)
{
    b1_queued_t queued;

    if ((0 == take(g_stopq, &g_stop_head, &g_stop_tail, &queued,
                   B1_STOP_QUEUE_SIZE)) &&
        (0 == take(g_rxq, &g_rx_head, &g_rx_tail, &queued, B1_QUEUE_SIZE)))
    {
        return 0;
    }
    return (0 == b1_frame_decode(queued.bytes, queued.len, p_frame)) ? 1 : -1;
}


/** @brief 1 once subscribed to the command topic, else 0. */
uint8_t b1_mqtt_connected (void)
{
    return __atomic_load_n(&g_mqtt_link_ready, __ATOMIC_ACQUIRE);
}

/** @brief 1 when connected and the telemetry queue has room. */
uint8_t b1_mqtt_can_send (void)
{
    /* The telemetry task is the only producer, so a free slot stays free. */
    uint8_t const next = (uint8_t)((g_tx_head + 1u) % B1_QUEUE_SIZE);

    return (uint8_t)((0u != b1_mqtt_connected()) && (next != g_tx_tail));
}

/** @brief Publish the link state read by b1_mqtt_connected(). */
static void publish_link_status (uint8_t ready)
{
    __atomic_store_n(&g_mqtt_link_ready, ready, __ATOMIC_RELEASE);
}

#if defined(B1_ENABLE_MQTT) && TM_WIFI_NETIF
#include "lwip/tcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#if defined(__has_include)
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#elif __has_include("buddy1_broker_local.h")
#include "modules/telemetry/buddy1_broker_local.h"
#endif
#endif
#ifndef B1_BROKER_IPV4
#error "Provide a private buddy1_broker_local.h configuration"
#endif
#ifndef B1_BROKER_PORT
#define B1_BROKER_PORT 1883
#endif
#ifndef B1_MQTT_USER
#define B1_MQTT_USER ""
#endif
#ifndef B1_MQTT_PASSWORD
#define B1_MQTT_PASSWORD ""
#endif
#ifndef B1_MQTT_TOPIC_ROOT
#define B1_MQTT_TOPIC_ROOT "g7/b1/robot"
#endif
#ifndef B1_MQTT_CLIENT_ID
#define B1_MQTT_CLIENT_ID "g7-b1-car01"
#endif
#define B1_TOPIC_TELEM B1_MQTT_TOPIC_ROOT "/telemetry"
#define B1_TOPIC_COMMAND B1_MQTT_TOPIC_ROOT "/command"
#define B1_TOPIC_ACK B1_MQTT_TOPIC_ROOT "/ack"
#define B1_TOPIC_STATUS B1_MQTT_TOPIC_ROOT "/availability"
/* Plain-text copy of the serial console (USB-console builds only). */
#define B1_TOPIC_LOG B1_MQTT_TOPIC_ROOT "/log"
/* Log chunks: whole lines where possible, at most this many bytes. */
#define B1_LOG_CHUNK 240u
/* Room for the topic string in front of a log chunk. */
#define B1_LOG_TOPIC_ROOM 40u
/* Send a partial chunk after this long, so a quiet log still arrives. */
#define B1_LOG_FLUSH_MS 100u
/* Average and burst log rate; stops a sample dump flooding the link. */
#define B1_LOG_BYTES_PER_SEC 8000u
#define B1_LOG_BURST 2048u
/* TCP send buffer kept free for ACKs and telemetry: logs never fill it. */
#define B1_LOG_TCP_RESERVE 1024u
/* "[netlog] lost N bytes" note: prefix, suffix and its buffers. */
#define B1_LOST_PREFIX "[netlog] lost "
#define B1_LOST_PREFIX_LEN 14u
#define B1_LOST_SUFFIX " bytes\n"
#define B1_LOST_SUFFIX_LEN 7u
#define B1_LOST_NOTE_CAP 48u
#define B1_LOST_DIGITS_CAP 12u
#define B1_LOST_DIGITS_MAX 11
#define B1_DECIMAL_BASE 10u

#define B1_TXBUF_CAP 320u
#define B1_RXBUF_CAP 256u
/* Largest MQTT body written; keeps the packet inside B1_TXBUF_CAP. */
#define B1_MAX_BODY 300u
/* Link loss: the broker answers each PINGREQ at once, so three ping
 * periods with nothing received means the link is gone and the controller
 * stops the motors. Measured uplink stalls are at most about 4.3 s. */
#define B1_PING_PERIOD_MS 5000u
#define B1_LINK_SILENCE_MS 15000u
/* TCP connect or CONNACK not completed within this time: retry. */
#define B1_CONNECT_TIMEOUT_MS 10000u
/* Retry delay after tcp_new() fails, and the reconnect backoff range. */
#define B1_ALLOC_RETRY_MS 2000u
#define B1_BACKOFF_MAX_SEC 16u
#define B1_MS_PER_SEC 1000u

/* MQTT 3.1.1 control packet first bytes and fields. */
#define MQTT_CONNECT 0x10u
#define MQTT_PUBLISH 0x30u
#define MQTT_PUBLISH_RETAIN 0x31u
#define MQTT_SUBSCRIBE 0x82u
#define MQTT_PINGREQ 0xc0u
#define MQTT_TYPE_SHIFT 4u
#define MQTT_TYPE_CONNACK 2u
#define MQTT_TYPE_PUBLISH 3u
#define MQTT_TYPE_SUBACK 9u
#define MQTT_TYPE_PINGRESP 13u
#define MQTT_PUBLISH_FLAGS_MASK 7u
#define MQTT_PROTOCOL_LEVEL 4u
#define MQTT_KEEPALIVE_SEC 30u
/* Clean session, retained QoS 0 Last Will; user and password flags. */
#define MQTT_CONNECT_FLAGS 0x26u
#define MQTT_FLAG_USER 0x80u
#define MQTT_FLAG_PASSWORD 0x40u
/* Remaining length: 7 bits per byte, continuation bit, at most 4 bytes
 * (5 including the fixed header byte); bodies over 200 bytes refused. */
#define MQTT_VARLEN_BASE 128u
#define MQTT_VARLEN_MASK 127u
#define MQTT_VARLEN_MORE 0x80u
#define MQTT_VARLEN_BYTES 4u
#define MQTT_VARLEN_END 5u
#define MQTT_RX_BODY_MAX 200u
/* Strings are a 16-bit big-endian length and the bytes. */
#define MQTT_STRING_MAX 65535u
#define MQTT_STRING_HEADER 2u
#define MQTT_BYTE_SHIFT 8u
/* Size limits checked before CONNECT. */
#define MQTT_USER_MAX 32u
#define MQTT_PASSWORD_MAX 64u
#define MQTT_CLIENT_ID_MAX 32u
#define MQTT_WILL_TOPIC_MAX 60u
/* Packet buffers. */
#define MQTT_CONNECT_CAP 256u
#define MQTT_PUBLISH_CAP 145u
#define MQTT_SUBSCRIBE_CAP 100u
#define MQTT_ONLINE "online"
#define MQTT_ONLINE_LEN 6u
/* Received packet field positions. */
#define MQTT_CONNACK_LEN 2u
#define MQTT_CONNACK_CODE 1u
#define MQTT_SUBACK_MIN 3u
#define MQTT_SUBACK_CODE 2u

static struct tcp_pcb * gp_pcb;
static uint8_t g_connected;
static uint8_t g_subscribed;
static uint8_t g_tcp_ready;
static uint8_t g_rx[B1_RXBUF_CAP];
static uint16_t g_rxlen;
static uint32_t g_next_connect_ms;
static uint32_t g_last_rx_ms;
static uint32_t g_last_ping_ms;
static uint16_t g_packet_id = 1u;
static uint8_t g_backoff = 1u;

/** @brief Encode an MQTT remaining length; returns its size in bytes. */
static size_t mqtt_varlen (uint8_t * p_out, size_t n)
{
    size_t pos = 0;

    do
    {
        uint8_t v = (uint8_t)(n % MQTT_VARLEN_BASE);

        n /= MQTT_VARLEN_BASE;
        if (0u != n)
        {
            v |= MQTT_VARLEN_MORE;
        }
        p_out[pos++] = v;
    } while ((0u != n) && (pos < MQTT_VARLEN_BYTES));
    return pos;
}

/** @brief Write a length-prefixed MQTT string; 0 if it is too long. */
static size_t mqtt_string (uint8_t * p_out, char const * p_str)
{
    size_t const n = strlen(p_str);

    if (n > MQTT_STRING_MAX)
    {
        return 0;
    }
    p_out[0] = (uint8_t)(n >> MQTT_BYTE_SHIFT);
    p_out[1] = (uint8_t)n;
    memcpy(p_out + MQTT_STRING_HEADER, p_str, n);
    return n + MQTT_STRING_HEADER;
}

/** @brief Send one MQTT packet (first byte, remaining length, body). */
static int32_t mqtt_write (uint8_t type, uint8_t const * p_data, size_t n)
{
    uint8_t packet[B1_TXBUF_CAP];
    uint8_t vl[MQTT_VARLEN_BYTES];
    size_t offset = 0;
    size_t vn;
    err_t result;

    if ((NULL == gp_pcb) || (0u == g_tcp_ready) || (n > B1_MAX_BODY))
    {
        return -1;
    }
    vn = mqtt_varlen(vl, n);
    packet[offset++] = type;
    memcpy(packet + offset, vl, vn);
    offset += vn;
    if ((offset + n) > sizeof packet)
    {
        return -1;
    }
    memcpy(packet + offset, p_data, n);
    offset += n;
    result = tcp_write(gp_pcb, packet, (u16_t)offset, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != result)
    {
        return -1;
    }
    result = tcp_output(gp_pcb);
    if (ERR_OK != result)
    {
        return -1;
    }
    return 0;
}

/** @brief Forget the session after a close: link down, queues dropped. */
static void reset_session (void)
{
    publish_link_status(0u);
    g_tcp_ready = 0;
    g_connected = 0;
    g_subscribed = 0;
    g_rxlen = 0;
    /* Never replay a queued START on reconnect. */
    g_rx_tail = g_rx_head;
    g_stop_tail = g_stop_head;
    g_next_connect_ms = sys_now() + ((uint32_t)g_backoff * B1_MS_PER_SEC);
    if (g_backoff < B1_BACKOFF_MAX_SEC)
    {
        g_backoff = (uint8_t)(g_backoff * 2u);
    }
}

/** @brief Close the connection and schedule a reconnect with backoff. */
static void discard (void)
{
    if (NULL != gp_pcb)
    {
        struct tcp_pcb * const p_old = gp_pcb;

        gp_pcb = NULL;
        tcp_arg(p_old, NULL);
        tcp_recv(p_old, NULL);
        tcp_err(p_old, NULL);
        tcp_sent(p_old, NULL);
        if (ERR_OK != tcp_close(p_old))
        {
            tcp_abort(p_old);
        }
    }
    reset_session();
}

/** @brief lwIP error callback: the PCB is already freed. */
static void tcp_error (void * p_arg, err_t err)
{
    (void)p_arg;
    (void)err;
    gp_pcb = NULL;
    reset_session();
}

/** @brief Send CONNECT with the retained "offline" Last Will. */
static int32_t connect_mqtt (void)
{
    uint8_t p[MQTT_CONNECT_CAP];
    size_t n = 0;
    char const * const p_user = B1_MQTT_USER;
    char const * const p_pass = B1_MQTT_PASSWORD;
    uint8_t flags = MQTT_CONNECT_FLAGS;

    if ('\0' != p_user[0])
    {
        flags |= MQTT_FLAG_USER;
    }
    if ('\0' != p_pass[0])
    {
        flags |= MQTT_FLAG_PASSWORD;
    }
    n += mqtt_string(p + n, "MQTT");
    p[n++] = MQTT_PROTOCOL_LEVEL;
    p[n++] = flags;
    p[n++] = 0u;
    p[n++] = MQTT_KEEPALIVE_SEC;
    if ((strlen(p_user) > MQTT_USER_MAX) ||
        (strlen(p_pass) > MQTT_PASSWORD_MAX) ||
        (strlen(B1_MQTT_CLIENT_ID) > MQTT_CLIENT_ID_MAX) ||
        (strlen(B1_TOPIC_STATUS) > MQTT_WILL_TOPIC_MAX))
    {
        return -1;
    }
    n += mqtt_string(p + n, B1_MQTT_CLIENT_ID);
    n += mqtt_string(p + n, B1_TOPIC_STATUS);
    n += mqtt_string(p + n, "offline");
    if ('\0' != p_user[0])
    {
        n += mqtt_string(p + n, p_user);
    }
    if ('\0' != p_pass[0])
    {
        n += mqtt_string(p + n, p_pass);
    }
    return mqtt_write(MQTT_CONNECT, p, n);
}

/** @brief Publish one queued frame on the telemetry or ACK topic. */
static int32_t mqtt_publish (b1_queued_t const * p_queued)
{
    uint8_t p[MQTT_PUBLISH_CAP];
    size_t n = 0;
    char const * const p_topic =
        (0u != p_queued->topic_ack) ? B1_TOPIC_ACK : B1_TOPIC_TELEM;

    n += mqtt_string(p + n, p_topic);
    if ((n + p_queued->len) > sizeof p)
    {
        return -1;
    }
    memcpy(p + n, p_queued->bytes, p_queued->len);
    n += p_queued->len;
    return mqtt_write(MQTT_PUBLISH, p, n);
}

/* Console copy kept by the USB console (lib/libtm); absent in UART-console
 * builds, where the log topic stays silent. The prototypes match lib/libtm,
 * which uses plain int and unsigned types. */
extern int tm_netlog_take (unsigned char * p_out, int max,
                           unsigned int * p_dropped) __attribute__((weak));
extern int tm_netlog_pending (void) __attribute__((weak));
static uint8_t g_log_buf[B1_LOG_CHUNK];
static uint16_t g_log_len;
static uint32_t g_log_last_ms;
static uint32_t g_log_tokens = B1_LOG_BURST;
static uint32_t g_log_token_ms;
static uint32_t g_log_lost;

/** @brief Write the "[netlog] lost N bytes" note; returns its length. */
static size_t log_lost_note (char * p_note, uint32_t lost)
{
    uint32_t v = lost;
    char digits[B1_LOST_DIGITS_CAP];
    int32_t k = 0;
    size_t len;

    do
    {
        digits[k++] = (char)('0' + (v % B1_DECIMAL_BASE));
        v /= B1_DECIMAL_BASE;
    } while ((0u != v) && (k < B1_LOST_DIGITS_MAX));
    memcpy(p_note, B1_LOST_PREFIX, B1_LOST_PREFIX_LEN);
    len = B1_LOST_PREFIX_LEN;
    while (k > 0)
    {
        p_note[len++] = digits[--k];
    }
    memcpy(p_note + len, B1_LOST_SUFFIX, B1_LOST_SUFFIX_LEN);
    return len + B1_LOST_SUFFIX_LEN;
}

/** @brief Send buffered console text on the log topic (lowest priority).
 *
 * Called only when no ACK is waiting. Sends whole lines, keeping a partial
 * line unless it fills the chunk or has waited B1_LOG_FLUSH_MS. Bytes lost
 * (ring full or a refused write) are reported in-band as one line.
 */
static void log_poll (uint32_t now)
{
    uint8_t p[B1_LOG_CHUNK + B1_LOG_TOPIC_ROOM];
    size_t n = 0;
    size_t send;
    unsigned int dropped = 0u;
    char note[B1_LOST_NOTE_CAP];
    size_t note_len = 0;

    if ((NULL == tm_netlog_take) || (NULL == tm_netlog_pending))
    {
        return;
    }
    g_log_tokens += ((now - g_log_token_ms) * B1_LOG_BYTES_PER_SEC) /
                    B1_MS_PER_SEC;
    g_log_token_ms = now;
    if (g_log_tokens > B1_LOG_BURST)
    {
        g_log_tokens = B1_LOG_BURST;
    }
    if (g_log_len < B1_LOG_CHUNK)
    {
        g_log_len = (uint16_t)(g_log_len +
                               tm_netlog_take(g_log_buf + g_log_len,
                                              (int)(B1_LOG_CHUNK - g_log_len),
                                              &dropped));
        g_log_lost += dropped;
    }
    if ((0u == g_log_len) && (0u == g_log_lost))
    {
        g_log_last_ms = now;
        return;
    }
    if ((g_log_len < B1_LOG_CHUNK) &&
        ((now - g_log_last_ms) < B1_LOG_FLUSH_MS))
    {
        return;
    }
    if ((g_log_tokens < g_log_len) || (NULL == gp_pcb) ||
        (tcp_sndbuf(gp_pcb) <
         (B1_LOG_CHUNK + B1_LOG_TOPIC_ROOM + B1_LOG_TCP_RESERVE)) ||
        (tcp_sndqueuelen(gp_pcb) >= (TCP_SND_QUEUELEN / 2u)))
    {
        return;
    }
    /* Up to the last newline; a chunk with none goes out whole. */
    send = g_log_len;
    for (size_t i = g_log_len; i > 0u; i--)
    {
        if ('\n' == g_log_buf[i - 1u])
        {
            send = i;
            break;
        }
    }
    if (0u != g_log_lost)
    {
        note_len = log_lost_note(note, g_log_lost);
    }
    n += mqtt_string(p + n, B1_TOPIC_LOG);
    memcpy(p + n, note, note_len);
    n += note_len;
    memcpy(p + n, g_log_buf, send);
    n += send;
    if (0 == mqtt_write(MQTT_PUBLISH, p, n))
    {
        g_log_lost = 0u;
        g_log_tokens -= (uint32_t)send;
    }
    else
    {
        g_log_lost += (uint32_t)send;
    }
    memmove(g_log_buf, g_log_buf + send, g_log_len - send);
    g_log_len = (uint16_t)(g_log_len - send);
    g_log_last_ms = now;
}

/** @brief CONNACK accepted: subscribe to the command topic. */
static void on_connack (void)
{
    uint8_t subscribe[MQTT_SUBSCRIBE_CAP];
    size_t len = 0;

    g_connected = 1u;
    g_backoff = 1u;
    subscribe[len++] = (uint8_t)(g_packet_id >> MQTT_BYTE_SHIFT);
    subscribe[len++] = (uint8_t)g_packet_id;
    len += mqtt_string(subscribe + len, B1_TOPIC_COMMAND);
    subscribe[len++] = 0u;
    (void)mqtt_write(MQTT_SUBSCRIBE, subscribe, len);
}

/** @brief SUBACK accepted: publish "online" and mark the link ready. */
static void on_suback (void)
{
    /* Availability is a soft status indicator; no motor authority. */
    uint8_t b[MQTT_SUBSCRIBE_CAP];
    size_t n = mqtt_string(b, B1_TOPIC_STATUS);

    memcpy(b + n, MQTT_ONLINE, MQTT_ONLINE_LEN);
    n += MQTT_ONLINE_LEN;
    (void)mqtt_write(MQTT_PUBLISH_RETAIN, b, n);
    g_subscribed = 1u;
    publish_link_status(1u);
}

/** @brief Queue a received command frame; STOP goes to its own queue. */
static void on_command (uint8_t const * p_frame, uint16_t n)
{
    b1_frame_t decoded;
    b1_command_t command;
    if ((0 == b1_frame_decode(p_frame, n, &decoded)) &&
        (B1_TYPE_COMMAND == decoded.type) &&
        (0 == b1_decode_command(decoded.payload, decoded.payload_len,
                                &command)) &&
        (B1_CMD_STOP == command.command))
    {
        (void)offer(g_stopq, &g_stop_head, &g_stop_tail, p_frame,
                    (uint8_t)n, 0u, B1_STOP_QUEUE_SIZE);
    }
    else
    {
        (void)offer(g_rxq, &g_rx_head, &g_rx_tail, p_frame, (uint8_t)n,
                    0u, B1_QUEUE_SIZE);
    }
}

/** @brief Handle one complete MQTT packet from the broker. */
static void process_packet (uint8_t head, uint8_t const * p, uint16_t n)
{
    uint8_t const type = (uint8_t)(head >> MQTT_TYPE_SHIFT);

    if ((MQTT_TYPE_CONNACK == type) && (MQTT_CONNACK_LEN == n))
    {
        if (0u != p[MQTT_CONNACK_CODE])
        {
            discard();
            return;
        }
        on_connack();
    }
    else if ((MQTT_TYPE_SUBACK == type) && (n >= MQTT_SUBACK_MIN))
    {
        if (0u == p[MQTT_SUBACK_CODE])
        {
            on_suback();
        }
    }
    else if (MQTT_TYPE_PINGRESP == type)
    {
        /* Nothing to do: any packet already refreshed g_last_rx_ms. */
    }
    else if ((MQTT_TYPE_PUBLISH == type) && (n >= MQTT_STRING_HEADER) &&
             (0u == (head & MQTT_PUBLISH_FLAGS_MASK)))
    {
        /* QoS 0, non-retained only. */
        uint16_t const len = (uint16_t)(((uint16_t)p[0] << MQTT_BYTE_SHIFT) |
                                        p[1]);
        char const * const p_topic = B1_TOPIC_COMMAND;
        uint8_t const * const p_body = p + MQTT_STRING_HEADER + len;

        if (((size_t)len == strlen(p_topic)) &&
            (n >= (MQTT_STRING_HEADER + len)) &&
            (0 == memcmp(p + MQTT_STRING_HEADER, p_topic, len)) &&
            ((n - MQTT_STRING_HEADER - len) <= B1_MAX_FRAME))
        {
            on_command(p_body, (uint16_t)(n - MQTT_STRING_HEADER - len));
        }
    }
    else
    {
        /* Other packet types are ignored. */
    }
}

/** @brief lwIP receive callback: reassemble and dispatch MQTT packets. */
static err_t tcp_recv_cb (void * p_arg, struct tcp_pcb * p_pcb,
                          struct pbuf * p_buf, err_t err)
{
    uint16_t pos = 0;

    (void)p_arg;
    if ((ERR_OK != err) || (NULL == p_buf))
    {
        if (NULL != p_buf)
        {
            pbuf_free(p_buf);
        }
        discard();
        return ERR_OK;
    }
    tcp_recved(p_pcb, p_buf->tot_len);
    g_last_rx_ms = sys_now();
    if (((uint32_t)g_rxlen + p_buf->tot_len) > sizeof g_rx)
    {
        pbuf_free(p_buf);
        discard();
        return ERR_OK;
    }
    (void)pbuf_copy_partial(p_buf, g_rx + g_rxlen, p_buf->tot_len, 0u);
    g_rxlen = (uint16_t)(g_rxlen + p_buf->tot_len);
    pbuf_free(p_buf);
    while (g_rxlen >= MQTT_STRING_HEADER)
    {
        uint32_t remain = 0;
        uint32_t mul = 1u;
        uint8_t b;
        uint16_t body = 1u;

        do
        {
            if (body >= g_rxlen)
            {
                return ERR_OK;
            }
            b = g_rx[body++];
            remain += (uint32_t)(b & MQTT_VARLEN_MASK) * mul;
            mul *= MQTT_VARLEN_BASE;
            if ((body > MQTT_VARLEN_END) || (remain > MQTT_RX_BODY_MAX))
            {
                discard();
                return ERR_OK;
            }
        } while (0u != (b & MQTT_VARLEN_MORE));
        if (remain > (uint32_t)(g_rxlen - body))
        {
            return ERR_OK;
        }
        process_packet(g_rx[0], g_rx + body, (uint16_t)remain);
        pos = (uint16_t)(body + remain);
        if (pos < g_rxlen)
        {
            memmove(g_rx, g_rx + pos, g_rxlen - pos);
        }
        g_rxlen = (uint16_t)(g_rxlen - pos);
        if (NULL == gp_pcb)
        {
            return ERR_OK;
        }
    }
    return ERR_OK;
}

/** @brief lwIP connect callback: send CONNECT. */
static err_t tcp_connected_cb (void * p_arg, struct tcp_pcb * p_pcb,
                               err_t err)
{
    (void)p_arg;
    (void)p_pcb;
    if (ERR_OK != err)
    {
        discard();
        return ERR_OK;
    }
    g_tcp_ready = 1u;
    g_last_rx_ms = sys_now();
    g_last_ping_ms = g_last_rx_ms;
    if (0 != connect_mqtt())
    {
        discard();
    }
    return ERR_OK;
}

/** @brief Start a TCP connection to the broker when the retry time has
 *  come. */
static void start_connect (struct netif * p_netif, uint32_t now)
{
    ip4_addr_t ip;
    err_t result;

    if ((int32_t)(now - g_next_connect_ms) < 0)
    {
        return;
    }
    if ((0 == ip4addr_aton(B1_BROKER_IPV4, &ip)) || ip4_addr_isany_val(ip))
    {
        return;
    }
    gp_pcb = tcp_new();
    g_last_rx_ms = now;
    if (NULL == gp_pcb)
    {
        g_next_connect_ms = now + B1_ALLOC_RETRY_MS;
        return;
    }
    tcp_bind_netif(gp_pcb, p_netif);
    tcp_arg(gp_pcb, NULL);
    tcp_err(gp_pcb, tcp_error);
    tcp_recv(gp_pcb, tcp_recv_cb);
    result = tcp_connect(gp_pcb, (ip_addr_t const *)&ip,
                         (u16_t)B1_BROKER_PORT, tcp_connected_cb);
    if (ERR_OK != result)
    {
        discard();
    }
}

/** @brief Run the MQTT client once; see mqtt_utk.h. */
void mqtt_utk_poll (struct netif * p_netif)
{
    uint32_t const now = sys_now();
    b1_queued_t queued;

    if ((NULL == p_netif) || !netif_is_up(p_netif) ||
        !netif_is_link_up(p_netif))
    {
        if (NULL != gp_pcb)
        {
            discard();
        }
        return;
    }
    if (NULL == gp_pcb)
    {
        start_connect(p_netif, now);
        return;
    }
    if ((0u == g_tcp_ready) || (0u == g_connected))
    {
        if ((now - g_last_rx_ms) > B1_CONNECT_TIMEOUT_MS)
        {
            discard();
        }
        return;
    }
    if ((now - g_last_rx_ms) > B1_LINK_SILENCE_MS)
    {
        discard();
        return;
    }
    /* Ping every period even while telemetry is sending: PINGRESP is what
     * keeps g_last_rx_ms fresh when the dashboard sends nothing. */
    if ((now - g_last_ping_ms) > B1_PING_PERIOD_MS)
    {
        if (0 == mqtt_write(MQTT_PINGREQ, (uint8_t const *)"", 0u))
        {
            g_last_ping_ms = now;
        }
    }
    if ((0u != g_subscribed) && (g_ack_head != g_ack_tail))
    {
        queued = g_ackq[g_ack_tail];
        if (0 == mqtt_publish(&queued))
        {
            b1_barrier();
            g_ack_tail = (uint8_t)((g_ack_tail + 1u) % B1_ACK_QUEUE_SIZE);
        }
    }
    else if ((0u != g_subscribed) && (g_tx_head != g_tx_tail))
    {
        /* The tail advances only once tcp_write accepted the packet. */
        queued = g_txq[g_tx_tail];
        if (0 == mqtt_publish(&queued))
        {
            b1_barrier();
            g_tx_tail = (uint8_t)((g_tx_tail + 1u) % B1_QUEUE_SIZE);
        }
    }
    else if (0u != g_subscribed)
    {
        log_poll(now);
    }
    else
    {
        /* Waiting for SUBACK. */
    }
}
#else
/** @brief MQTT disabled in this build: nothing to do. */
void mqtt_utk_poll (struct netif * p_netif)
{
    (void)p_netif;
}
#endif
/*** end of file ***/

