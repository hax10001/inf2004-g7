/* Network-owner MQTT interface; never call from application tasks. */
#ifndef MQTT_UTK_H
#define MQTT_UTK_H
struct netif;
/* Invoke only from CYW43/lwIP owner task (nonblocking polling). */
void mqtt_utk_poll(struct netif *netif);
#endif
