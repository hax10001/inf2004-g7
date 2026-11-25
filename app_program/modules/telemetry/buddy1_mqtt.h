/** @file buddy1_mqtt.h
 * @brief Application-only API for bounded MQTT message buffers.
 */
#ifndef BUDDY1_MQTT_H
#define BUDDY1_MQTT_H
#include "buddy1_protocol.h"
#include <stdint.h>
/* Nonblocking, fixed-size SPSC frame queues. The network owner encodes and
 * sends; application code never calls MQTT or lwIP directly. */
/** @brief Queue a frame for sending; -1 if the queue is full. */
int32_t b1_mqtt_submit(const b1_frame_t *p_frame);
/** @brief Take a received command: 1 frame, 0 none, -1 bad frame. */
int32_t b1_mqtt_receive_command(b1_frame_t *p_frame);
/** @brief 1 once subscribed to the command topic; 0 when offline. */
uint8_t b1_mqtt_connected(void);
/** @brief 1 when connected and the telemetry queue has a free slot. Take a
 *  frame from the store only then, so offline telemetry stays buffered. */
uint8_t b1_mqtt_can_send(void);
#endif
/*** end of file ***/

