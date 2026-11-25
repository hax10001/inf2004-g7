/** @file netif.h
 * @brief Host stub of an lwIP network interface whose admin and link
 *        state the test switches (FR15 link loss). Not the real header.
 */
#ifndef CT_LWIP_NETIF_H
#define CT_LWIP_NETIF_H
#include "lwip/err.h"

/** Only the two flags mqtt_utk_poll() reads. */
struct netif
{
    u8_t up;
    u8_t link_up;
};

/** @brief 1 when the interface is administratively up. */
static inline int netif_is_up (struct netif const * p_netif)
{
    return 0U != p_netif->up;
}

/** @brief 1 when the Wi-Fi link is associated. */
static inline int netif_is_link_up (struct netif const * p_netif)
{
    return 0U != p_netif->link_up;
}

#endif /* CT_LWIP_NETIF_H */

/*** end of file ***/

