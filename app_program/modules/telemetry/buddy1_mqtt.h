/** @file buddy1_mqtt.h
 * @brief Application-only API for bounded MQTT message buffers.
 */
#ifndef BUDDY1_MQTT_H
#define BUDDY1_MQTT_H
#include "buddy1_protocol.h"
#include <stdint.h>
/* Nonblocking, fixed-size SPSC frame queues. The network owner performs
 * serialization and transport; app code never calls raw MQTT/lwIP APIs. */
int b1_mqtt_submit(const B1_Frame *f);
int b1_mqtt_receive_command(B1_Frame *f);
uint32_t b1_mqtt_queue_drops(void);
/* Broker subscription ready; false when Wi-Fi/MQTT is disconnected. */
uint8_t b1_mqtt_connected(void);
/* Connected and the telemetry queue has a free slot: only then take a
 * frame out of the store, so offline telemetry stays in its rings. */
uint8_t b1_mqtt_can_send(void);
#endif
/*** end of file ***/

