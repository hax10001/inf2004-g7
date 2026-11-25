/** @file ip4_addr.h
 * @brief Host stub of lwIP IPv4 addresses for mqtt_utk.c tests (FR15).
 *        Not the real lwIP header.
 */
#ifndef CT_LWIP_IP4_ADDR_H
#define CT_LWIP_IP4_ADDR_H
#include "lwip/err.h"

/** IPv4 address, network byte order as in lwIP. */
typedef struct ip4_addr
{
    u32_t addr;
} ip4_addr_t;
typedef ip4_addr_t ip_addr_t;

/** @brief Parse dotted-quad text; 1 on success, 0 on bad text. */
int ip4addr_aton(char const * p_text, ip4_addr_t * p_addr);

/** @brief 1 when the address is 0.0.0.0 (lwIP ip4_addr_isany_val). */
static inline int ip4_addr_isany_val (ip4_addr_t addr)
{
    return 0U == addr.addr;
}

#endif /* CT_LWIP_IP4_ADDR_H */

/*** end of file ***/

