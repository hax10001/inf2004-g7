/** @file err.h
 * @brief Host stub of lwIP base types and error codes for mqtt_utk.c
 *        tests (FR15 MQTT link loss). Not the real lwIP header.
 */
#ifndef CT_LWIP_ERR_H
#define CT_LWIP_ERR_H
#include <stdint.h>

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t err_t;

#define ERR_OK (0)
#define ERR_MEM (-1)
#define ERR_CONN (-11)
#define ERR_RST (-14)

#endif /* CT_LWIP_ERR_H */

/*** end of file ***/

