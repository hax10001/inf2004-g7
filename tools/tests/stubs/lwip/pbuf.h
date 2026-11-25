/** @file pbuf.h
 * @brief Host stub of one contiguous lwIP pbuf so a test can feed broker
 *        bytes to mqtt_utk.c (FR15). Not the real lwIP header.
 */
#ifndef CT_LWIP_PBUF_H
#define CT_LWIP_PBUF_H
#include "lwip/err.h"

/** One received segment; the stub never chains pbufs. */
struct pbuf
{
    struct pbuf * next;
    void * payload;
    u16_t tot_len;
    u16_t len;
};

/** @brief Copy len bytes from offset; returns the bytes copied. */
u16_t pbuf_copy_partial(struct pbuf const * p_buf, void * p_out, u16_t len,
                        u16_t offset);
/** @brief Release a pbuf; the test counts calls to detect leaks. */
u8_t pbuf_free(struct pbuf * p_buf);

#endif /* CT_LWIP_PBUF_H */

/*** end of file ***/

