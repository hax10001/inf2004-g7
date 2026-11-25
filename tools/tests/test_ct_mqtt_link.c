/** @file test_ct_mqtt_link.c
 * @brief Host test: the real MQTT client lib/libnet/lwip/mqtt_utk.c over
 *        stub lwIP (FR15 MQTT link loss, FR13 command delivery).
 *
 * mqtt_utk.c is built with B1_ENABLE_MQTT and TM_WIFI_NETIF=1 against the
 * stub headers in tools/tests/stubs/lwip. This file implements those stub
 * functions: tcp_write() output is recorded packet by packet, the test
 * calls the registered connect, receive and error callbacks with broker
 * bytes, and sys_now() returns a clock the test advances. The broker is a
 * documentation-range address; no real credentials are used.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "mqtt_utk.h"
#include "modules/telemetry/buddy1_mqtt.h"

#define CT_ROOT "g7/b1/robot"
#define CT_TOPIC_COMMAND CT_ROOT "/command"
#define CT_TOPIC_TELEM CT_ROOT "/telemetry"
#define CT_TOPIC_ACK CT_ROOT "/ack"
#define CT_TOPIC_STATUS CT_ROOT "/availability"
#define CT_TOPIC_LOG CT_ROOT "/log"
#define CT_CLIENT_ID "g7-b1-car01"
#define CT_BROKER_ADDR (0x0A0200C0U) /* 192.0.2.10 in network order */
#define CT_BROKER_PORT 1883U
/* Timing contract from mqtt_utk.c (FR15). */
#define CT_PING_MS 5000U
#define CT_SILENCE_MS 15000U
#define CT_CONNECT_TIMEOUT_MS 10000U
#define CT_ALLOC_RETRY_MS 2000U
#define CT_STEP_MS 100U
#define CT_RX_CAP 256U
/* Recorded output. */
#define CT_TX_SLOTS 64U
#define CT_PACKET_CAP 400U
/* MQTT packet first bytes. */
#define CT_CONNECT 0x10U
#define CT_CONNACK 0x20U
#define CT_PUBLISH 0x30U
#define CT_PUBLISH_RETAIN 0x31U
#define CT_PUBLISH_QOS1 0x32U
#define CT_SUBSCRIBE 0x82U
#define CT_SUBACK 0x90U
#define CT_PINGREQ 0xC0U
#define CT_PINGRESP 0xD0U
#define CT_SNDBUF_FREE 2920U

/* Stub lwIP state. */
static struct tcp_pcb g_pcb;
static struct netif g_netif;
static u32_t g_now = 1000U;
static uint32_t g_news;
static uint32_t g_closes;
static uint32_t g_connects;
static uint8_t g_fail_new;
static err_t g_connect_result = ERR_OK;
static err_t g_write_result = ERR_OK;
static u16_t g_sndbuf = CT_SNDBUF_FREE;
static ip4_addr_t g_connect_ip;
static u16_t g_connect_port;
static tcp_recv_fn g_recv_cb;
static tcp_connected_fn g_connected_cb;
static tcp_err_fn g_err_cb;
static uint32_t g_fed_bufs;
static uint32_t g_freed_bufs;
static uint32_t g_fed_bytes;
static uint32_t g_recved_bytes;
static uint8_t g_tx[CT_TX_SLOTS][CT_PACKET_CAP];
static uint16_t g_tx_len[CT_TX_SLOTS];
static uint32_t g_tx_count;
/* Console copy offered to the log topic. */
static uint8_t g_log_text[CT_PACKET_CAP];
static uint32_t g_log_len;
static uint32_t g_log_dropped;

/** @brief Stub clock. */
u32_t sys_now (void)
{
    return g_now;
}

/** @brief Stub: dotted quad to network-order address. */
int ip4addr_aton (char const * p_text, ip4_addr_t * p_addr)
{
    uint32_t part = 0U;
    uint32_t parts = 0U;
    uint32_t addr = 0U;

    for (;;)
    {
        if ((*p_text >= '0') && (*p_text <= '9'))
        {
            part = (part * 10U) + (uint32_t)(*p_text - '0');
        }
        else
        {
            if ((part > 255U) || (parts > 3U))
            {
                return 0;
            }
            addr |= part << (8U * parts);
            parts++;
            part = 0U;
            if ('\0' == *p_text)
            {
                break;
            }
        }
        p_text++;
    }
    p_addr->addr = addr;
    return (4U == parts) ? 1 : 0;
}

/** @brief Stub: copy out of the single contiguous pbuf. */
u16_t pbuf_copy_partial (struct pbuf const * p_buf, void * p_out, u16_t len,
                         u16_t offset)
{
    memcpy(p_out, (uint8_t const *)p_buf->payload + offset, len);
    return len;
}

/** @brief Stub: count frees. */
u8_t pbuf_free (struct pbuf * p_buf)
{
    (void)p_buf;
    g_freed_bufs++;
    return 1U;
}

/** @brief Stub: one connection object, or none when g_fail_new. */
struct tcp_pcb * tcp_new (void)
{
    g_news++;
    return (0U != g_fail_new) ? NULL : &g_pcb;
}

/** @brief Stub: argument unused. */
void tcp_arg (struct tcp_pcb * p_pcb, void * p_arg)
{
    assert(&g_pcb == p_pcb);
    (void)p_arg;
}

/** @brief Stub: keep the receive callback. */
void tcp_recv (struct tcp_pcb * p_pcb, tcp_recv_fn recv)
{
    assert(&g_pcb == p_pcb);
    g_recv_cb = recv;
}

/** @brief Stub: keep the error callback. */
void tcp_err (struct tcp_pcb * p_pcb, tcp_err_fn err)
{
    assert(&g_pcb == p_pcb);
    g_err_cb = err;
}

/** @brief Stub: sent callback unused. */
void tcp_sent (struct tcp_pcb * p_pcb, tcp_sent_fn sent)
{
    assert(&g_pcb == p_pcb);
    (void)sent;
}

/** @brief Stub: the client binds to the Wi-Fi interface it was given. */
void tcp_bind_netif (struct tcp_pcb * p_pcb, struct netif const * p_netif)
{
    assert(&g_pcb == p_pcb);
    assert(&g_netif == p_netif);
}

/** @brief Stub: record the target; the test completes it later. */
err_t tcp_connect (struct tcp_pcb * p_pcb, ip_addr_t const * p_ip,
                   u16_t port, tcp_connected_fn connected)
{
    assert(&g_pcb == p_pcb);
    g_connects++;
    g_connect_ip = *p_ip;
    g_connect_port = port;
    g_connected_cb = connected;
    return g_connect_result;
}

/** @brief Stub: record one MQTT packet per call. */
err_t tcp_write (struct tcp_pcb * p_pcb, void const * p_data, u16_t len,
                 u8_t flags)
{
    assert(&g_pcb == p_pcb);
    assert(TCP_WRITE_FLAG_COPY == flags);
    if (ERR_OK != g_write_result)
    {
        return g_write_result;
    }
    assert(g_tx_count < CT_TX_SLOTS);
    assert(len <= CT_PACKET_CAP);
    memcpy(g_tx[g_tx_count], p_data, len);
    g_tx_len[g_tx_count] = len;
    g_tx_count++;
    return ERR_OK;
}

/** @brief Stub: output always succeeds. */
err_t tcp_output (struct tcp_pcb * p_pcb)
{
    assert(&g_pcb == p_pcb);
    return ERR_OK;
}

/** @brief Stub: count graceful closes. */
err_t tcp_close (struct tcp_pcb * p_pcb)
{
    assert(&g_pcb == p_pcb);
    g_closes++;
    return ERR_OK;
}

/** @brief Stub: abort never needed because close succeeds. */
void tcp_abort (struct tcp_pcb * p_pcb)
{
    (void)p_pcb;
    assert(0);
}

/** @brief Stub: count window updates. */
void tcp_recved (struct tcp_pcb * p_pcb, u16_t len)
{
    assert(&g_pcb == p_pcb);
    g_recved_bytes += len;
}

/** @brief Stub: free send buffer chosen by the test. */
u16_t tcp_sndbuf (struct tcp_pcb const * p_pcb)
{
    (void)p_pcb;
    return g_sndbuf;
}

/** @brief Stub: send queue empty. */
u16_t tcp_sndqueuelen (struct tcp_pcb const * p_pcb)
{
    (void)p_pcb;
    return 0U;
}

/** @brief Stub USB console copy: hand over the queued text. */
int tm_netlog_take (unsigned char * p_out, int max, unsigned int * p_dropped)
{
    uint32_t n = g_log_len;

    if (n > (uint32_t)max)
    {
        n = (uint32_t)max;
    }
    memcpy(p_out, g_log_text, n);
    memmove(g_log_text, g_log_text + n, g_log_len - n);
    g_log_len -= n;
    *p_dropped = g_log_dropped;
    g_log_dropped = 0U;
    return (int)n;
}

/** @brief Stub USB console copy: bytes waiting. */
int tm_netlog_pending (void)
{
    return (int)g_log_len;
}

/** @brief Forget recorded packets. */
static void tx_reset (void)
{
    g_tx_count = 0U;
}

/** @brief Poll the client once at the current time. */
static void poll (void)
{
    mqtt_utk_poll(&g_netif);
}

/** @brief Feed one TCP segment through the registered receive callback;
 *  every segment must be freed and acknowledged to the TCP window. */
static void feed (uint8_t const * p_bytes, uint16_t n)
{
    static uint8_t copy[CT_PACKET_CAP];
    struct pbuf segment;

    assert(NULL != g_recv_cb);
    assert(n <= CT_PACKET_CAP);
    memcpy(copy, p_bytes, n);
    segment.next = NULL;
    segment.payload = copy;
    segment.tot_len = n;
    segment.len = n;
    g_fed_bufs++;
    g_fed_bytes += n;
    assert(ERR_OK == g_recv_cb(NULL, &g_pcb, &segment, ERR_OK));
    assert(g_fed_bufs == g_freed_bufs);
    assert(g_fed_bytes == g_recved_bytes);
}

/** @brief Append an MQTT length-prefixed string; returns bytes written. */
static uint16_t put_string (uint8_t * p_out, char const * p_text)
{
    uint16_t const n = (uint16_t)strlen(p_text);

    p_out[0] = (uint8_t)(n >> 8U);
    p_out[1] = (uint8_t)n;
    memcpy(p_out + 2U, p_text, n);
    return (uint16_t)(n + 2U);
}

/** @brief Build a packet: first byte, remaining length, body. */
static uint16_t packet (uint8_t * p_out, uint8_t head, uint8_t const * p_body,
                        uint16_t n)
{
    uint16_t pos = 0U;
    uint16_t rest = n;

    p_out[pos++] = head;
    do
    {
        uint8_t digit = (uint8_t)(rest % 128U);

        rest = (uint16_t)(rest / 128U);
        if (0U != rest)
        {
            digit |= 0x80U;
        }
        p_out[pos++] = digit;
    } while (0U != rest);
    memcpy(p_out + pos, p_body, n);
    return (uint16_t)(pos + n);
}

/** @brief Build an encoded command frame; returns its length. */
static uint16_t command_frame (uint8_t * p_out, uint8_t command,
                               uint16_t transaction)
{
    b1_command_t cmd = {0};
    b1_frame_t frame = {0};
    size_t n;

    cmd.command = command;
    cmd.transaction = transaction;
    frame.type = B1_TYPE_COMMAND;
    frame.session = 1U;
    frame.payload_len = (uint8_t)b1_encode_command(&cmd, frame.payload,
                                                   sizeof frame.payload);
    assert(B1_CMD_LEN == frame.payload_len);
    n = b1_frame_encode(&frame, p_out, B1_MAX_FRAME);
    assert(0U != n);
    return (uint16_t)n;
}

/** @brief Feed a PUBLISH of one command frame on a topic. */
static void feed_command (uint8_t head, char const * p_topic,
                          uint8_t command, uint16_t transaction)
{
    uint8_t body[CT_PACKET_CAP];
    uint8_t bytes[CT_PACKET_CAP];
    uint16_t n = put_string(body, p_topic);

    n = (uint16_t)(n + command_frame(body + n, command, transaction));
    feed(bytes, packet(bytes, head, body, n));
}

/** @brief Take the next received command: 1 and its kind, or 0. */
static int32_t take_command (uint8_t * p_command)
{
    b1_frame_t frame = {0};
    b1_command_t cmd = {0};
    int32_t const result = b1_mqtt_receive_command(&frame);

    if (1 == result)
    {
        assert(B1_TYPE_COMMAND == frame.type);
        assert(0 == b1_decode_command(frame.payload, frame.payload_len,
                                      &cmd));
        *p_command = cmd.command;
    }
    return result;
}

/** @brief Build the expected PUBLISH of one frame on a topic. */
static uint16_t expected_publish (uint8_t * p_out, char const * p_topic,
                                  b1_frame_t const * p_frame)
{
    uint8_t body[CT_PACKET_CAP];
    uint16_t n = put_string(body, p_topic);

    n = (uint16_t)(n + b1_frame_encode(p_frame, body + n, B1_MAX_FRAME));
    return packet(p_out, CT_PUBLISH, body, n);
}

/** @brief 1 when recorded packet index equals the expected bytes. */
static int32_t tx_is (uint32_t index, uint8_t const * p_bytes, uint16_t n)
{
    return (int32_t)((index < g_tx_count) && (n == g_tx_len[index]) &&
                     (0 == memcmp(g_tx[index], p_bytes, n)));
}

/** @brief 1 when recorded packet index is a PINGREQ. */
static int32_t tx_is_ping (uint32_t index)
{
    uint8_t const ping[2] = {CT_PINGREQ, 0x00U};

    return tx_is(index, ping, 2U);
}

/** @brief A telemetry frame with a recognisable payload. */
static b1_frame_t telemetry_frame (uint16_t seq)
{
    b1_frame_t frame = {0};

    frame.type = B1_TYPE_HEARTBEAT;
    frame.session = 7U;
    frame.seq = seq;
    frame.time_ms = g_now;
    frame.payload_len = B1_HEARTBEAT_LEN;
    b1_put32(frame.payload, 0xC0FFEE00U + seq);
    return frame;
}

/** @brief Poll every 100 ms until the client opens a TCP connection. */
static void wait_for_tcp_new (void)
{
    uint32_t const news = g_news;
    uint32_t waited = 0U;

    assert(0U == b1_mqtt_connected());
    g_netif.up = 1U;
    g_netif.link_up = 1U;
    for (;;)
    {
        poll();
        if (news != g_news)
        {
            break;
        }
        g_now += CT_STEP_MS;
        waited += CT_STEP_MS;
        assert(waited <= 40000U);
    }
}

/** @brief Complete TCP connect; CONNECT must be the only packet sent. */
static void tcp_complete (void)
{
    tx_reset();
    assert(NULL != g_connected_cb);
    assert(ERR_OK == g_connected_cb(NULL, &g_pcb, ERR_OK));
    assert(1U == g_tx_count);
    assert(CT_CONNECT == g_tx[0][0]);
}

/** @brief Feed a CONNACK with a return code. */
static void feed_connack (uint8_t code)
{
    uint8_t const connack[4] = {CT_CONNACK, 0x02U, 0x00U, code};

    feed(connack, 4U);
}

/** @brief Feed a SUBACK for packet 1 with a return code. */
static void feed_suback (uint8_t code)
{
    uint8_t const suback[5] = {CT_SUBACK, 0x03U, 0x00U, 0x01U, code};

    feed(suback, 5U);
}

/** @brief Connect, CONNACK, SUBACK: the link is ready (FR15). */
static void handshake (void)
{
    wait_for_tcp_new();
    tcp_complete();
    feed_connack(0U);
    assert((2U == g_tx_count) && (CT_SUBSCRIBE == g_tx[1][0]));
    feed_suback(0U);
    assert((3U == g_tx_count) && (CT_PUBLISH_RETAIN == g_tx[2][0]));
    assert(1U == b1_mqtt_connected());
    tx_reset();
}

/** @brief Test: CONNECT, CONNACK, SUBSCRIBE, SUBACK and "online" (FR15).
 *  The link is not ready until subscribed to the command topic. */
static void test_handshake (void)
{
    uint8_t body[CT_PACKET_CAP];
    uint8_t expect[CT_PACKET_CAP];
    uint16_t n = 0U;
    uint8_t const connack_head[1] = {CT_CONNACK};
    uint8_t const connack_rest[3] = {0x02U, 0x00U, 0x00U};

    assert(0U == b1_mqtt_connected());
    assert(0U == b1_mqtt_can_send());
    g_netif.up = 1U;
    g_netif.link_up = 0U;
    poll();
    assert(0U == g_news);
    wait_for_tcp_new();
    assert((1U == g_connects) && (CT_BROKER_PORT == g_connect_port));
    assert(CT_BROKER_ADDR == g_connect_ip.addr);
    tcp_complete();
    n = put_string(body, "MQTT");
    body[n++] = 4U;     /* protocol level 3.1.1 */
    body[n++] = 0x26U;  /* clean session, retained QoS 0 will */
    body[n++] = 0U;
    body[n++] = 30U;    /* keep-alive seconds */
    n = (uint16_t)(n + put_string(body + n, CT_CLIENT_ID));
    n = (uint16_t)(n + put_string(body + n, CT_TOPIC_STATUS));
    n = (uint16_t)(n + put_string(body + n, "offline"));
    assert(1 == tx_is(0U, expect, packet(expect, CT_CONNECT, body, n)));
    assert(0U == b1_mqtt_connected());

    /* CONNACK split over two segments is reassembled. */
    feed(connack_head, 1U);
    assert(1U == g_tx_count);
    feed(connack_rest, 3U);
    n = 0U;
    body[n++] = 0U;
    body[n++] = 1U;
    n = (uint16_t)(n + put_string(body + n, CT_TOPIC_COMMAND));
    body[n++] = 0U;
    assert(1 == tx_is(1U, expect, packet(expect, CT_SUBSCRIBE, body, n)));
    assert(0U == b1_mqtt_connected());
    poll();
    assert(2U == g_tx_count);

    feed_suback(0U);
    n = put_string(body, CT_TOPIC_STATUS);
    memcpy(body + n, "online", 6U);
    n = (uint16_t)(n + 6U);
    assert(1 == tx_is(2U, expect,
                      packet(expect, CT_PUBLISH_RETAIN, body, n)));
    assert(1U == b1_mqtt_connected());
    assert(1U == b1_mqtt_can_send());
    tx_reset();
    puts("PASS FR15 handshake: CONNECT with retained offline will, "
         "SUBSCRIBE command topic, online after SUBACK only");
}

/** @brief Test: PINGREQ every 5 s and the link drops after more than
 *  15 s with nothing received, even while telemetry is being sent; it is
 *  still up at 14.9 s and at exactly 15 s (FR15). */
static void test_silence_drop (void)
{
    uint32_t const t0 = g_now;
    uint32_t const closes = g_closes;
    uint32_t last_ping = t0;
    uint32_t pings = 0U;
    uint32_t published = 0U;
    uint8_t expect[CT_PACKET_CAP];
    b1_frame_t first = telemetry_frame(1U);

    for (uint32_t t = t0 + CT_STEP_MS; t <= (t0 + CT_SILENCE_MS);
         t += CT_STEP_MS)
    {
        g_now = t;
        if (0U == ((t - t0) % 1000U))
        {
            b1_frame_t const frame = telemetry_frame((uint16_t)(t - t0));

            assert(0 == b1_mqtt_submit(&frame));
            if (1000U == (t - t0))
            {
                first = frame;
            }
        }
        tx_reset();
        poll();
        for (uint32_t i = 0U; i < g_tx_count; i++)
        {
            if (1 == tx_is_ping(i))
            {
                assert((t - last_ping) <= (CT_PING_MS + CT_STEP_MS));
                assert((t - last_ping) > CT_PING_MS);
                last_ping = t;
                pings++;
            }
            else
            {
                assert(CT_PUBLISH == g_tx[i][0]);
                if (0U == published)
                {
                    assert(1 == tx_is(i, expect,
                                      expected_publish(expect, CT_TOPIC_TELEM,
                                                       &first)));
                }
                published++;
            }
        }
        assert(1U == b1_mqtt_connected());
        assert(closes == g_closes);
    }
    assert(2U == pings);
    assert(15U == published);
    g_now = t0 + CT_SILENCE_MS + 1U;
    poll();
    assert(0U == b1_mqtt_connected());
    assert(0U == b1_mqtt_can_send());
    assert((closes + 1U) == g_closes);
    assert(NULL == g_recv_cb);
    puts("PASS FR15 link silence: PINGREQ every 5 s, up at 14.9 s and 15 s, "
         "dropped at 15.001 s although telemetry was still sent");
}

/** @brief Test: a broker answering each PINGREQ keeps the link up for a
 *  minute with no other traffic (FR15). */
static void test_pingresp_keeps_alive (void)
{
    uint8_t const pingresp[2] = {CT_PINGRESP, 0x00U};
    uint32_t const t0 = g_now;
    uint32_t due = 0U;
    uint32_t pings = 0U;

    handshake();
    for (uint32_t t = g_now + CT_STEP_MS; t <= (t0 + 60000U);
         t += CT_STEP_MS)
    {
        g_now = t;
        if (t == due)
        {
            feed(pingresp, 2U);
        }
        tx_reset();
        poll();
        if ((1U == g_tx_count) && (1 == tx_is_ping(0U)))
        {
            pings++;
            due = t + 200U;
        }
        assert(1U == b1_mqtt_connected());
    }
    assert(pings >= 11U);
    puts("PASS FR15 PINGRESP within 200 ms keeps the link up for 60 s");
}

/** @brief Test: Wi-Fi link down drops the MQTT link in the same poll; the
 *  reconnect waits 1 s, then 2 s after a TCP error (FR15). */
static void test_netif_down_and_backoff (void)
{
    uint32_t const closes = g_closes;
    uint32_t news;
    uint32_t td;

    assert(1U == b1_mqtt_connected());
    g_netif.link_up = 0U;
    poll();
    assert(0U == b1_mqtt_connected());
    assert((closes + 1U) == g_closes);
    td = g_now;
    news = g_news;
    g_now = td + 500U;
    poll();
    assert(news == g_news);

    g_netif.link_up = 1U;
    g_now = td + 999U;
    poll();
    assert(news == g_news);
    g_now = td + 1000U;
    poll();
    assert((news + 1U) == g_news);

    /* lwIP reports a reset: the PCB is already freed, never closed. */
    assert(NULL != g_err_cb);
    g_err_cb(NULL, ERR_RST);
    assert((closes + 1U) == g_closes);
    td = g_now;
    g_now = td + 1999U;
    poll();
    assert((news + 1U) == g_news);
    g_now = td + 2000U;
    poll();
    assert((news + 2U) == g_news);
    tcp_complete();
    feed_connack(0U);
    feed_suback(0U);
    assert(1U == b1_mqtt_connected());
    tx_reset();
    puts("PASS FR15 Wi-Fi link down drops at once; reconnect backoff "
         "1 s then 2 s; TCP error never closes a freed PCB");
}

/** @brief Test: STOP has its own queue: taken first even when the
 *  ordinary command queue is full and a START was dropped (FR13/FR15). */
static void test_stop_priority (void)
{
    uint8_t command = 0U;
    uint32_t snapshots = 0U;

    for (uint16_t i = 1U; i <= 7U; i++)
    {
        feed_command(CT_PUBLISH, CT_TOPIC_COMMAND, B1_CMD_SNAPSHOT, i);
    }
    feed_command(CT_PUBLISH, CT_TOPIC_COMMAND, B1_CMD_START, 8U);
    feed_command(CT_PUBLISH, CT_TOPIC_COMMAND, B1_CMD_STOP, 9U);
    assert(1 == take_command(&command));
    assert(B1_CMD_STOP == command);
    while (1 == take_command(&command))
    {
        assert(B1_CMD_SNAPSHOT == command);
        snapshots++;
    }
    assert(7U == snapshots);
    assert(0 == take_command(&command));
    assert(1U == b1_mqtt_connected());
    puts("PASS FR13/FR15 STOP delivered first with the command queue full "
         "(7 queued, START dropped)");
}

/** @brief Test: commands queued before a link loss are discarded, but
 *  telemetry queued offline is sent after reconnecting (FR15). */
static void test_reconnect_discards_commands (void)
{
    uint8_t command = 0U;
    uint8_t expect[CT_PACKET_CAP];
    b1_frame_t const offline = telemetry_frame(77U);

    feed_command(CT_PUBLISH, CT_TOPIC_COMMAND, B1_CMD_START, 20U);
    feed_command(CT_PUBLISH, CT_TOPIC_COMMAND, B1_CMD_STOP, 21U);
    g_err_cb(NULL, ERR_RST);
    assert(0U == b1_mqtt_connected());
    assert(0 == b1_mqtt_submit(&offline));
    handshake();
    assert(0 == take_command(&command));
    poll();
    assert(1 == tx_is(0U, expect,
                      expected_publish(expect, CT_TOPIC_TELEM, &offline)));
    tx_reset();
    puts("PASS FR15 reconnect never replays a queued START or STOP; "
         "offline telemetry is still published");
}

/** @brief Test: malformed or oversized input drops the link instead of
 *  being parsed, and a fragmented command is reassembled (FR15). */
static void test_malformed_input (void)
{
    uint8_t const five_byte_length[6] = {
        CT_PUBLISH, 0x80U, 0x80U, 0x80U, 0x80U, 0x01U
    };
    uint8_t const body_201[3] = {CT_PUBLISH, 0xC9U, 0x01U};
    uint8_t big[CT_RX_CAP + 1U];
    uint8_t bytes[CT_PACKET_CAP];
    uint8_t body[CT_PACKET_CAP];
    uint8_t command = 0U;
    uint16_t n;
    uint32_t closes = g_closes;

    feed(five_byte_length, 6U);
    assert(0U == b1_mqtt_connected());
    assert((closes + 1U) == g_closes);
    handshake();
    closes = g_closes;
    feed(body_201, 3U);
    assert(0U == b1_mqtt_connected());
    assert((closes + 1U) == g_closes);
    handshake();
    closes = g_closes;
    memset(big, 0, sizeof big);
    big[0] = CT_PINGRESP;
    feed(big, (uint16_t)sizeof big);
    assert(0U == b1_mqtt_connected());
    assert((closes + 1U) == g_closes);
    handshake();

    /* One PUBLISH in three segments: header byte, length, rest. */
    n = put_string(body, CT_TOPIC_COMMAND);
    n = (uint16_t)(n + command_frame(body + n, B1_CMD_START, 30U));
    n = packet(bytes, CT_PUBLISH, body, n);
    feed(bytes, 1U);
    feed(bytes + 1U, 1U);
    assert(0 == take_command(&command));
    feed(bytes + 2U, (uint16_t)(n - 2U));
    assert(1 == take_command(&command));
    assert(B1_CMD_START == command);
    assert(1U == b1_mqtt_connected());
    puts("PASS FR15 malformed: 5-byte length, 201-byte body and 257-byte "
         "segment drop the link; fragmented PUBLISH reassembled");
}

/** @brief Test: only QoS 0, non-retained PUBLISH on the exact command
 *  topic is queued; a corrupt frame is reported as bad (FR13). */
static void test_ignored_publishes (void)
{
    uint8_t bytes[CT_PACKET_CAP];
    uint8_t body[CT_PACKET_CAP];
    b1_frame_t frame = {0};
    uint8_t command = 0U;
    uint16_t n;

    feed_command(CT_PUBLISH_RETAIN, CT_TOPIC_COMMAND, B1_CMD_START, 40U);
    feed_command(CT_PUBLISH_QOS1, CT_TOPIC_COMMAND, B1_CMD_START, 41U);
    feed_command(CT_PUBLISH, CT_TOPIC_COMMAND "x", B1_CMD_START, 42U);
    feed_command(CT_PUBLISH, CT_TOPIC_TELEM, B1_CMD_START, 43U);
    assert(0 == take_command(&command));

    /* Longer than any frame: ignored. */
    n = put_string(body, CT_TOPIC_COMMAND);
    memset(body + n, 0xB1, B1_MAX_FRAME + 1U);
    n = (uint16_t)(n + B1_MAX_FRAME + 1U);
    feed(bytes, packet(bytes, CT_PUBLISH, body, n));
    assert(0 == take_command(&command));

    /* A frame with a bad CRC is queued and reported as bad, not run. */
    n = put_string(body, CT_TOPIC_COMMAND);
    n = (uint16_t)(n + command_frame(body + n, B1_CMD_START, 44U));
    body[n - 1U] ^= 0x01U;
    feed(bytes, packet(bytes, CT_PUBLISH, body, n));
    assert(-1 == b1_mqtt_receive_command(&frame));
    assert(0 == b1_mqtt_receive_command(&frame));
    assert(1U == b1_mqtt_connected());
    puts("PASS FR13 retained, QoS 1, wrong-topic and oversize PUBLISH "
         "ignored; bad-CRC command returned as -1");
}

/** @brief Test: ACK goes before older telemetry; a refused tcp_write
 *  loses nothing; the telemetry queue is bounded (FR15). */
static void test_send_order_and_retry (void)
{
    uint8_t expect[CT_PACKET_CAP];
    b1_frame_t const t1 = telemetry_frame(101U);
    b1_frame_t const t2 = telemetry_frame(102U);
    b1_frame_t ack = telemetry_frame(103U);
    uint32_t queued = 0U;
    uint16_t n;

    ack.type = B1_TYPE_ACK;
    assert(0 == b1_mqtt_submit(&t1));
    assert(0 == b1_mqtt_submit(&ack));
    tx_reset();
    poll();
    n = expected_publish(expect, CT_TOPIC_ACK, &ack);
    assert((1U == g_tx_count) && (1 == tx_is(0U, expect, n)));
    tx_reset();
    poll();
    n = expected_publish(expect, CT_TOPIC_TELEM, &t1);
    assert((1U == g_tx_count) && (1 == tx_is(0U, expect, n)));

    assert(0 == b1_mqtt_submit(&t2));
    g_write_result = ERR_MEM;
    tx_reset();
    poll();
    assert(0U == g_tx_count);
    g_write_result = ERR_OK;
    poll();
    n = expected_publish(expect, CT_TOPIC_TELEM, &t2);
    assert((1U == g_tx_count) && (1 == tx_is(0U, expect, n)));

    while (1U == b1_mqtt_can_send())
    {
        assert(0 == b1_mqtt_submit(&t1));
        queued++;
    }
    assert(7U == queued);
    assert(-1 == b1_mqtt_submit(&t1));
    for (uint32_t i = 0U; i < queued; i++)
    {
        poll();
    }
    assert(1U == b1_mqtt_can_send());
    tx_reset();
    puts("PASS FR15 ACK before older telemetry, refused write retried, "
         "telemetry queue bounded at 7 (submit -1 when full)");
}

/** @brief Test: CONNACK refusal, CONNECT timeout and allocation failure
 *  each close and retry; a refused SUBACK never reports ready (FR15). */
static void test_connect_failures (void)
{
    uint32_t closes;
    uint32_t news;
    uint32_t t0;

    g_netif.link_up = 0U;
    poll();
    g_netif.link_up = 1U;

    /* Broker refuses the session (5 = not authorised). */
    wait_for_tcp_new();
    tcp_complete();
    closes = g_closes;
    feed_connack(5U);
    assert(1U == g_tx_count);
    assert((closes + 1U) == g_closes);
    assert(0U == b1_mqtt_connected());

    /* After a refused session the client keeps retrying with backoff:
     * the next connection is closed again on its CONNACK, and the one
     * after that completes, so the link recovers on its own. */
    wait_for_tcp_new();
    tcp_complete();
    closes = g_closes;
    feed_connack(0U);
    assert((closes + 1U) == g_closes);
    assert(0U == b1_mqtt_connected());
    handshake();
    assert(1U == b1_mqtt_connected());
    puts("PASS FR15 refused CONNACK: the client retries with backoff and "
         "the link recovers");

    /* TCP connected but no CONNACK: retry after 10 s, not before. */
    g_netif.link_up = 0U;
    poll();
    g_netif.link_up = 1U;
    wait_for_tcp_new();
    tcp_complete();
    t0 = g_now;
    closes = g_closes;
    g_now = t0 + CT_CONNECT_TIMEOUT_MS;
    poll();
    assert(closes == g_closes);
    g_now = t0 + CT_CONNECT_TIMEOUT_MS + 1U;
    poll();
    assert((closes + 1U) == g_closes);

    /* No PCB memory: try again after 2 s. */
    g_fail_new = 1U;
    wait_for_tcp_new();
    news = g_news;
    t0 = g_now;
    g_fail_new = 0U;
    g_now = t0 + CT_ALLOC_RETRY_MS - 1U;
    poll();
    assert(news == g_news);
    g_now = t0 + CT_ALLOC_RETRY_MS;
    poll();
    assert((news + 1U) == g_news);

    /* SUBACK failure (0x80): stays not ready, START cannot arrive. */
    tcp_complete();
    feed_connack(0U);
    feed_suback(0x80U);
    assert(2U == g_tx_count);
    assert(0U == b1_mqtt_connected());
    assert(0U == b1_mqtt_can_send());
    g_netif.link_up = 0U;
    poll();
    handshake();
    puts("PASS FR15 refused CONNACK closes, no CONNACK in 10 s closes, "
         "tcp_new failure retried after 2 s, SUBACK 0x80 never ready");
}

/** @brief Test: console text goes out on the log topic as whole lines,
 *  lost bytes are reported in-band, and logs never use the send buffer
 *  reserved for ACKs and telemetry (FR15 remote diagnostics). */
static void test_console_log (void)
{
    uint8_t body[CT_PACKET_CAP];
    uint8_t expect[CT_PACKET_CAP];
    uint16_t n;
    char const * p_note = "[netlog] lost 5 bytes\n";

    /* An idle poll starts the 100 ms partial-line wait. */
    tx_reset();
    g_now += CT_STEP_MS;
    poll();
    memcpy(g_log_text, "abc\ndef", 7U);
    g_log_len = 7U;
    g_now += CT_STEP_MS / 2U;
    poll();
    assert(0U == g_tx_count);
    g_now += CT_STEP_MS / 2U;
    poll();
    n = put_string(body, CT_TOPIC_LOG);
    memcpy(body + n, "abc\n", 4U);
    n = (uint16_t)(n + 4U);
    assert((1U == g_tx_count) &&
           (1 == tx_is(0U, expect, packet(expect, CT_PUBLISH, body, n))));

    g_log_dropped = 5U;
    g_now += CT_STEP_MS;
    tx_reset();
    poll();
    n = put_string(body, CT_TOPIC_LOG);
    memcpy(body + n, p_note, strlen(p_note));
    n = (uint16_t)(n + strlen(p_note));
    memcpy(body + n, "def", 3U);
    n = (uint16_t)(n + 3U);
    assert((1U == g_tx_count) &&
           (1 == tx_is(0U, expect, packet(expect, CT_PUBLISH, body, n))));

    memcpy(g_log_text, "x\n", 2U);
    g_log_len = 2U;
    g_sndbuf = 1000U;
    tx_reset();
    for (uint32_t i = 0U; i < 5U; i++)
    {
        g_now += CT_STEP_MS;
        poll();
    }
    assert(0U == g_tx_count);
    g_sndbuf = CT_SNDBUF_FREE;
    g_now += CT_STEP_MS;
    poll();
    assert((1U == g_tx_count) && (CT_PUBLISH == g_tx[0][0]));

    /* A full 240-byte chunk without a newline goes at once, whole, with a
     * two-byte remaining length (2 + 15 + 240 = 257). */
    memset(g_log_text, 'a', 300U);
    g_log_len = 300U;
    g_now += CT_STEP_MS;
    tx_reset();
    poll();
    assert((1U == g_tx_count) && (260U == g_tx_len[0]));
    assert((CT_PUBLISH == g_tx[0][0]) && (0x81U == g_tx[0][1]) &&
           (0x02U == g_tx[0][2]) && ('a' == g_tx[0][259]));

    /* A refused write loses the 60 remaining bytes: reported next. */
    g_write_result = ERR_MEM;
    g_now += CT_STEP_MS;
    poll();
    g_write_result = ERR_OK;
    tx_reset();
    g_now += CT_STEP_MS;
    poll();
    p_note = "[netlog] lost 60 bytes\n";
    n = put_string(body, CT_TOPIC_LOG);
    memcpy(body + n, p_note, strlen(p_note));
    n = (uint16_t)(n + strlen(p_note));
    assert((1U == g_tx_count) &&
           (1 == tx_is(0U, expect, packet(expect, CT_PUBLISH, body, n))));
    tx_reset();
    puts("PASS FR15 console log: whole lines, in-band lost-byte note, held "
         "back while the TCP send buffer is low, full 240-byte chunk sent "
         "at once, refused write counted as lost");
}

/** @brief Test: the broker closing the TCP connection, or tcp_connect()
 *  failing, drops the link at once and retries (FR15). */
static void test_remote_close (void)
{
    uint32_t const closes = g_closes;
    uint32_t news;

    assert(1U == b1_mqtt_connected());
    assert(NULL != g_recv_cb);
    assert(ERR_OK == g_recv_cb(NULL, &g_pcb, NULL, ERR_OK));
    assert(0U == b1_mqtt_connected());
    assert((closes + 1U) == g_closes);

    g_connect_result = ERR_MEM;
    wait_for_tcp_new();
    assert((closes + 2U) == g_closes);
    g_connect_result = ERR_OK;
    news = g_news;
    handshake();
    assert(g_news > news);
    puts("PASS FR15 broker FIN (NULL pbuf) drops the link at once; "
         "tcp_connect error closes and retries");
}

/** @brief Run every test case in order (each builds on a live link). */
int main (void)
{
    test_handshake();
    test_silence_drop();
    test_pingresp_keeps_alive();
    test_netif_down_and_backoff();
    test_stop_priority();
    test_reconnect_discards_commands();
    test_malformed_input();
    test_ignored_publishes();
    test_send_order_and_retry();
    test_connect_failures();
    test_console_log();
    test_remote_close();
    puts("PASS MQTT client link-loss and command-delivery checks");
    return 0;
}

/*** end of file ***/

