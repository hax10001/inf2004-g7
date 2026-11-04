/** @file buddy1_protocol.c
 * @brief Portable binary telemetry encoding and decoding.
 */
#include "buddy1_protocol.h"
#include <string.h>

void b1_put16(uint8_t *p, uint16_t x) { p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8); }
uint16_t b1_get16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
void b1_put32(uint8_t *p, uint32_t x) { b1_put16(p, (uint16_t)x);
    b1_put16(p + 2, (uint16_t)(x >> 16)); }
uint32_t b1_get32(const uint8_t *p) {
    return (uint32_t)b1_get16(p) | ((uint32_t)b1_get16(p + 2) << 16); }

static uint8_t checksum(const uint8_t *p, size_t n) {
    uint8_t crc = 0;
    size_t i;
    while(n-- != 0u) {
        uint8_t b = *p++;
        for(i = 0; i < 8u; i++) {
            uint8_t carry = (uint8_t)((crc ^ b) & 0x80u);
            crc = (uint8_t)(crc << 1u);
            if(carry != 0u) crc ^= 0x07u;
            b = (uint8_t)(b << 1u);
        }
    }
    return crc;
}

size_t b1_frame_encode(const B1_Frame *in, uint8_t *out, size_t cap) {
    size_t n;
    if(in == NULL || out == NULL || in->payload_len > B1_MAX_PAYLOAD) return 0;
    n = B1_HEADER + (size_t)in->payload_len + 1u;
    if(n > cap) return 0;
    out[0] = B1_MAGIC; out[1] = B1_VERSION; out[2] = in->type;
    out[3] = in->flags;
    b1_put16(out + 4, in->session); b1_put16(out + 6, in->seq);
    b1_put32(out + 8, in->time_ms); out[12] = in->payload_len;
    memcpy(out + B1_HEADER, in->payload, in->payload_len);
    out[n - 1u] = checksum(out, n - 1u);
    return n;
}
int b1_frame_decode(const uint8_t *bytes, size_t n, B1_Frame *out) {
    if(bytes == NULL || out == NULL || n < B1_HEADER + 1u ||
       n > B1_MAX_FRAME || bytes[0] != B1_MAGIC || bytes[1] != B1_VERSION ||
       bytes[12] > B1_MAX_PAYLOAD || n != B1_HEADER + (size_t)bytes[12] + 1u ||
       checksum(bytes, n - 1u) != bytes[n - 1u]) return -1;
    memset(out, 0, sizeof(*out));
    out->type = bytes[2]; out->flags = bytes[3];
    out->session = b1_get16(bytes + 4); out->seq = b1_get16(bytes + 6);
    out->time_ms = b1_get32(bytes + 8); out->payload_len = bytes[12];
    memcpy(out->payload, bytes + B1_HEADER, out->payload_len);
    return 0;
}

int b1_pack_mapping(const uint8_t x[4], uint8_t dst[3]) {
    uint32_t v;
    unsigned i,j;
    if(x == NULL || dst == NULL) return -1;
    for(i = 0; i < 4u; i++) {
        if(x[i] > 25u) return -1;
        for(j = 0; j < i; j++) if(x[j] == x[i]) return -1;
    }
    v = (uint32_t)x[0] | ((uint32_t)x[1] << 5u) |
        ((uint32_t)x[2] << 10u) | ((uint32_t)x[3] << 15u);
    dst[0] = (uint8_t)v; dst[1] = (uint8_t)(v >> 8);
        dst[2] = (uint8_t)(v >> 16);
    return 0;
}
int b1_unpack_mapping(const uint8_t src[3], uint8_t x[4]) {
    uint32_t v;
    uint8_t mapped[3];
    unsigned i;
    if(src == NULL || x == NULL || (src[2] & 0xf0u) != 0u) return -1;
    v = (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16);
    for(i = 0; i < 4u; i++) x[i] = (uint8_t)((v >> (5u * i)) & 31u);
    return b1_pack_mapping(x, mapped);
}

/* Least-significant-bit first within each octet, independent of C ABI. */
typedef struct { uint8_t *bytes; size_t bits, cap; } Writer;
static void putbits(Writer *w, uint32_t v, unsigned count) {
    unsigned i;
    for(i = 0; i < count; i++) {
        size_t bit = w->bits++;
        if(bit < w->cap * 8u) {
            if((v & (1u << i)) != 0u) w->bytes[bit / 8u] |= (uint8_t)(1u << (
            bit % 8u));
        }
    }
}
typedef struct { const uint8_t *bytes; size_t bits; } Reader;
static uint32_t getbits(Reader *r, unsigned count) {
    uint32_t v = 0; unsigned i;
    for(i = 0; i < count; i++) {
        size_t bit = r->bits++;
        if((r->bytes[bit / 8u] & (uint8_t)(1u << (bit % 8u))) != 0u) v |= (1u
            << i);
    }
    return v;
}
static uint16_t bounded(uint16_t x, uint16_t max) { return x > max ? max : x; }
static uint16_t signed12(int16_t x) {
    if(x < -2047) x = -2047;
    if(x > 2047) x = 2047;
    return (uint16_t)x & 0xfffu;
}
static int16_t unsign12(uint16_t x) {
    return (int16_t)((x & 0x800u) != 0u ? (int16_t)(x | 0xf000u) : (int16_t)x);
}
size_t b1_encode_snapshot(const B1_SnapshotPayload *s, uint8_t dst[13]) {
    Writer w;
    if(s == NULL || dst == NULL || s->barcode > 31u || s->action > 3u ||
       s->mission > 7u || s->line > 7u || s->side > 3u) return 0;
    memset(dst, 0, 13u); w.bytes = dst; w.bits = 0; w.cap = 13u;
    putbits(&w, s->source_mask & 31u, 5u); putbits(&w, s->stale_mask & 31u, 5u);
    putbits(&w, s->mission, 3u); putbits(&w, s->line, 3u);
    putbits(&w, s->ir & 7u, 3u); putbits(&w, s->barcode, 5u);
    putbits(&w, s->action, 2u); putbits(&w, s->action_valid ? 1u : 0u, 1u);
    putbits(&w, bounded(s->hump_mm,1023u),10u);
    putbits(&w, bounded(s->max_hump_mm,1023u),10u);
    putbits(&w, bounded(s->obstacle_cm,1023u),10u);
    putbits(&w, s->side,2u); putbits(&w, s->bypass_count,8u);
    putbits(&w, s->faults,8u);
    putbits(&w, signed12(s->speed_left_mm_s),12u);
    putbits(&w, signed12(s->speed_right_mm_s),12u);
    return (w.bits + 7u) / 8u;
}
int b1_decode_snapshot(const uint8_t *src, size_t size, B1_SnapshotPayload *s) {
    Reader r;
    if(src == NULL || s == NULL || size != 13u || (src[12] & 0xf8u) != 0u)
        return -1;
    r.bytes = src; r.bits = 0;
    memset(s,0,sizeof(*s));
    s->source_mask = (uint8_t)getbits(&r,5u);
        s->stale_mask = (uint8_t)getbits(&r,5u);
    s->mission=(uint8_t)getbits(&r,3u); s->line=(uint8_t)getbits(&r,3u);
    s->ir=(uint8_t)getbits(&r,3u); s->barcode=(uint8_t)getbits(&r,5u);
    s->action=(uint8_t)getbits(&r,2u); s->action_valid=(uint8_t)getbits(&r,1u);
    s->hump_mm=(uint16_t)getbits(&r,10u);
        s->max_hump_mm=(uint16_t)getbits(&r,10u);
    s->obstacle_cm=(uint16_t)getbits(&r,10u);
    s->side=(uint8_t)getbits(&r,2u); s->bypass_count=(uint8_t)getbits(&r,8u);
    s->faults=(uint8_t)getbits(&r,8u);
    s->speed_left_mm_s=unsign12((uint16_t)getbits(&r,12u));
    s->speed_right_mm_s=unsign12((uint16_t)getbits(&r,12u));
    if(s->barcode>25u && s->barcode!=31u)return -1;
    return 0;
}
size_t b1_encode_command(const B1_Command *c, uint8_t *p, size_t cap) {
    if(c == NULL || p == NULL || cap < (c->command
        == B1_CMD_MAPPING ? 6u : 3u)) return 0;
    if(c->command < B1_CMD_MAPPING || c->command > B1_CMD_RECAL_IMU) return 0;
    p[0]=c->command; b1_put16(p+1,c->transaction);
    if(c->command==B1_CMD_MAPPING) {
        if(b1_pack_mapping(c->mapping,p+3)!=0) return 0;
        return 6u;
    }
    return 3u;
}
int b1_decode_command(const uint8_t *p,size_t n,B1_Command *c) {
    if(p==NULL || c==NULL || n < 3u || p[0]<B1_CMD_MAPPING
        || p[0]>B1_CMD_RECAL_IMU ||
       n != (p[0]==B1_CMD_MAPPING ? 6u : 3u)) return -1;
    memset(c,0,sizeof(*c)); c->command=p[0]; c->transaction=b1_get16(p+1);
    if(c->command==B1_CMD_MAPPING && b1_unpack_mapping(p+3,
        c->mapping)!=0) return -1;
    return 0;
}
/*** end of file ***/

