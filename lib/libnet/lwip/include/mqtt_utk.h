/** @file mqtt_utk.h
 * @brief Network-owner MQTT interface, polled by the CYW43 service.
 */
#ifndef MQTT_UTK_H
#define MQTT_UTK_H
struct netif;
/** @brief Run the MQTT client once: connect, receive, send queued frames.
 *  Call only from the CYW43/lwIP owner task (non-blocking). */
void mqtt_utk_poll (struct netif * p_netif);
#endif
/*** end of file ***/

