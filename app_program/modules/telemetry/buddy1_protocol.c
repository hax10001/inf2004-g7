/** @file buddy1_protocol.c
 * @brief Portable binary telemetry encoding and decoding.
 */
#include "buddy1_protocol.h"
#include <string.h>

/** @brief Write a 16-bit value little-endian. */
void b1_put16(uint8_t *p_buf, uint16_t x) { p_buf[0] = (uint8_t)x;
    p_buf[1] = (uint8_t)(x >> B1_BYTE_SHIFT);
}
/** @brief Read a 16-bit little-endian value. */
uint16_t b1_get16(const uint8_t *p_buf)
{
    return (uint16_t)((uint16_t)p_buf[0] | ((uint16_t)p_buf[1] <<
        B1_BYTE_SHIFT));
}
/** @brief Write a 32-bit value little-endian. */
void b1_put32(uint8_t *p_buf, uint32_t x) { b1_put16(p_buf, (uint16_t)x);
    b1_put16(p_buf + B1_HALF_BYTES, (uint16_t)(x >> B1_HALF_SHIFT));
}
/** @brief Read a 32-bit little-endian value. */
uint32_t b1_get32(const uint8_t *p_buf)
{
    return (uint32_t)b1_get16(p_buf) |
        ((uint32_t)b1_get16(p_buf + B1_HALF_BYTES) << B1_HALF_SHIFT);
}

/** @brief CRC-8 (polynomial 0x07) over a byte buffer. */
static uint8_t checksum(const uint8_t *p_buf, size_t n)
{
    uint8_t crc = 0;
    size_t i;
    while(n-- != 0u)
    {
        uint8_t b = *p_buf++;
        for(i = 0; i < B1_BITS_PER_BYTE; i++)
        {
            uint8_t carry = (uint8_t)((crc ^ b) & B1_CRC_TOP_BIT);
            crc = (uint8_t)(crc << 1u);
            if(carry != 0u)
            {
                crc ^= B1_CRC_POLY;
            }
            b = (uint8_t)(b << 1u);
        }
    }
    return crc;
}

/** @brief Encode a frame: 13-byte header, payload, CRC-8. */
size_t b1_frame_encode(const b1_frame_t *p_in, uint8_t *p_out, size_t cap)
{
    size_t n;
    if(p_in == NULL || p_out == NULL || p_in->payload_len > B1_MAX_PAYLOAD)
    {
        return 0;
    }
    n = B1_HEADER + (size_t)p_in->payload_len + 1u;
    if(n > cap)
    {
        return 0;
    }
    p_out[0] = B1_MAGIC;
    p_out[1] = B1_VERSION;
    p_out[B1_HDR_OFF_TYPE] = p_in->type;
    p_out[B1_HDR_OFF_FLAGS] = p_in->flags;
    b1_put16(p_out + B1_HDR_OFF_SESSION, p_in->session);
    b1_put16(p_out + B1_HDR_OFF_SEQ, p_in->seq);
    b1_put32(p_out + B1_HDR_OFF_TIME, p_in->time_ms);
    p_out[B1_HDR_OFF_LEN] = p_in->payload_len;
    memcpy(p_out + B1_HEADER, p_in->payload, p_in->payload_len);
    p_out[n - 1u] = checksum(p_out, n - 1u);
    return n;
}
/** @brief Decode and validate a frame (magic, length, CRC). */
int32_t b1_frame_decode(const uint8_t *p_bytes, size_t n, b1_frame_t *p_out)
{
    if(p_bytes == NULL || p_out == NULL || n < B1_HEADER + 1u ||
        n > B1_MAX_FRAME || p_bytes[0] != B1_MAGIC || p_bytes[1] != B1_VERSION
            ||
        p_bytes[B1_HDR_OFF_LEN] > B1_MAX_PAYLOAD || n != B1_HEADER + (
            size_t)p_bytes[B1_HDR_OFF_LEN] + 1u ||
        checksum(p_bytes, n - 1u) != p_bytes[n - 1u])
    {
        return -1;
    }
    memset(p_out, 0, sizeof(*p_out));
    p_out->type = p_bytes[B1_HDR_OFF_TYPE];
    p_out->flags = p_bytes[B1_HDR_OFF_FLAGS];
    p_out->session = b1_get16(p_bytes + B1_HDR_OFF_SESSION);
    p_out->seq = b1_get16(p_bytes + B1_HDR_OFF_SEQ);
    p_out->time_ms = b1_get32(p_bytes + B1_HDR_OFF_TIME);
    p_out->payload_len = p_bytes[B1_HDR_OFF_LEN];
    memcpy(p_out->payload, p_bytes + B1_HEADER, p_out->payload_len);
    return 0;
}

/** @brief Pack four distinct letters A-Z into 20 bits. */
int32_t b1_pack_mapping(const uint8_t x[B1_MAP_LETTERS],
    uint8_t dst[B1_MAP_BYTES])
{
    uint32_t v;
    uint32_t i;
    uint32_t j;
    if(x == NULL || dst == NULL)
    {
        return -1;
    }
    for(i = 0; i < B1_MAP_LETTERS; i++)
    {
        if(x[i] > B1_LETTER_MAX)
        {
            return -1;
        }
        for(j = 0; j < i; j++)
        {
            if(x[j] == x[i])
            {
                return -1;
            }
        }
    }
    v = (uint32_t)x[0] | ((uint32_t)x[1] << B1_LETTER_BITS) |
    ((uint32_t)x[B1_MAP_IDX_L2] << B1_MAP_SHIFT_L2) |
    ((uint32_t)x[B1_MAP_IDX_L3] << B1_MAP_SHIFT_L3);
    dst[0] = (uint8_t)v;
    dst[1] = (uint8_t)(v >> B1_BYTE_SHIFT);
    dst[B1_MAP_BYTE_HI] = (uint8_t)(v >> B1_HALF_SHIFT);
    return 0;
}
/** @brief Unpack and validate a 20-bit route mapping. */
int32_t b1_unpack_mapping(const uint8_t src[B1_MAP_BYTES],
    uint8_t x[B1_MAP_LETTERS])
{
    uint32_t v;
    uint8_t mapped[B1_MAP_BYTES];
    uint32_t i;
    if(src == NULL || x == NULL ||
        (src[B1_MAP_BYTE_HI] & B1_MAP_RESERVED) != 0u)
    {
        return -1;
    }
    v = (uint32_t)src[0] | ((uint32_t)src[1] << B1_BYTE_SHIFT) |
        ((uint32_t)src[B1_MAP_BYTE_HI] << B1_HALF_SHIFT);
    for(i = 0; i < B1_MAP_LETTERS; i++)
    {
        x[i] = (uint8_t)((v >> (B1_LETTER_BITS * i)) & B1_LETTER_MASK);
    }
    return b1_pack_mapping(x, mapped);
}

/* Bit writer and reader: LSB first within each byte. */
typedef struct { uint8_t *bytes;
    size_t bits, cap;
} b1_bit_writer_t;
/** @brief Append the low bits of a value to a bit writer. */
static void putbits(b1_bit_writer_t *p_writer, uint32_t v, uint32_t count)
{
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        size_t bit = p_writer->bits++;
        if(bit < p_writer->cap * B1_BITS_PER_BYTE)
        {
            if((v & (1u << i)) != 0u)
            {
                p_writer->bytes[bit / B1_BITS_PER_BYTE] |=
                    (uint8_t)(1u << (bit % B1_BITS_PER_BYTE));
            }
        }
    }
}
typedef struct { const uint8_t *bytes;
    size_t bits;
} b1_bit_reader_t;
/** @brief Read bits from a bit reader. */
static uint32_t getbits(b1_bit_reader_t *p_reader, uint32_t count)
{
    uint32_t v = 0;
    uint32_t i;
    for(i = 0; i < count; i++)
    {
        size_t bit = p_reader->bits++;
        if((p_reader->bytes[bit / B1_BITS_PER_BYTE] &
            (uint8_t)(1u << (bit % B1_BITS_PER_BYTE))) != 0u)
        {
            v |= (1u << i);
        }
    }
    return v;
}
/** @brief Clamp a value to an unsigned field width. */
static uint16_t bounded(uint16_t x, uint16_t max) { return x > max ? max : x;
}
/** @brief Clamp a speed to +-2047 and encode it as a 12-bit field. */
static uint16_t signed12(int16_t x)
{
    if(x < -B1_SPEED12_MAX)
    {
        x = -B1_SPEED12_MAX;
    }
    if(x > B1_SPEED12_MAX)
    {
        x = B1_SPEED12_MAX;
    }
    return (uint16_t)x & B1_SPEED12_MASK;
}
/** @brief Sign-extend a 12-bit field to int16_t. */
static int16_t unsign12(uint16_t x)
{
    return (int16_t)((x & B1_SPEED12_SIGN) != 0u ?
        (int16_t)(x | B1_SPEED12_EXTEND) : (int16_t)x);
}
/** @brief Serialise a snapshot into its 13-byte payload. */
size_t b1_encode_snapshot(const b1_snapshot_payload_t *p_snap,
    uint8_t dst[B1_SNAPSHOT_LEN])
{
    b1_bit_writer_t w;
    if(p_snap == NULL || dst == NULL || (p_snap->barcode > B1_LETTER_MAX &&
        p_snap->barcode != B1_LETTER_NONE))
    {
        return 0;
    }
    memset(dst, 0, B1_SNAPSHOT_LEN);
    w.bytes = dst;
    w.bits = 0;
    w.cap = B1_SNAPSHOT_LEN;
    putbits(&w, p_snap->source_mask & B1_SOURCES_MASK, B1_SNAP_W_SOURCES);
    putbits(&w, p_snap->stale_mask & B1_SOURCES_MASK, B1_SNAP_W_SOURCES);
    putbits(&w, p_snap->mission, B1_SNAP_W_MISSION);
    putbits(&w, p_snap->line, B1_SNAP_W_LINE);
    putbits(&w, p_snap->ir & B1_IR_MASK, B1_SNAP_W_IR);
    putbits(&w, p_snap->barcode, B1_SNAP_W_BARCODE);
    putbits(&w, p_snap->action, B1_SNAP_W_ACTION);
    putbits(&w, p_snap->action_valid ? 1u : 0u, 1u);
    putbits(&w, bounded(p_snap->hump_mm,B1_RANGE10_NONE),B1_SNAP_W_RANGE);
    putbits(&w, bounded(p_snap->max_hump_mm,B1_RANGE10_NONE),
        B1_SNAP_W_RANGE);
    putbits(&w, bounded(p_snap->obstacle_cm,B1_RANGE10_NONE),
        B1_SNAP_W_RANGE);
    putbits(&w, p_snap->side,B1_SNAP_W_SIDE);
    putbits(&w, p_snap->bypass_count,B1_SNAP_W_BYPASS);
    putbits(&w, p_snap->faults,B1_SNAP_W_FAULTS);
    putbits(&w, signed12(p_snap->speed_left_mm_s),B1_SNAP_W_SPEED);
    putbits(&w, signed12(p_snap->speed_right_mm_s),B1_SNAP_W_SPEED);
    return (w.bits + B1_BITS_ROUND_UP) / B1_BITS_PER_BYTE;
}
/** @brief Parse a 13-byte snapshot payload; reject reserved bits. */
int32_t b1_decode_snapshot(const uint8_t *p_src, size_t size,
    b1_snapshot_payload_t *p_snap)
{
    b1_bit_reader_t r;
    if(p_src == NULL || p_snap == NULL || size != B1_SNAPSHOT_LEN ||
        (p_src[B1_SNAPSHOT_LAST] & B1_SNAPSHOT_PAD) != 0u)
    {
        return -1;
    }
    r.bytes = p_src;
    r.bits = 0;
    memset(p_snap,0,sizeof(*p_snap));
    p_snap->source_mask = (uint8_t)getbits(&r,B1_SNAP_W_SOURCES);
    p_snap->stale_mask = (uint8_t)getbits(&r,B1_SNAP_W_SOURCES);
    p_snap->mission=(uint8_t)getbits(&r,B1_SNAP_W_MISSION);
    p_snap->line=(uint8_t)getbits(&r,B1_SNAP_W_LINE);
    p_snap->ir=(uint8_t)getbits(&r,B1_SNAP_W_IR);
    p_snap->barcode=(uint8_t)getbits(&r,B1_SNAP_W_BARCODE);
    p_snap->action=(uint8_t)getbits(&r,B1_SNAP_W_ACTION);
    p_snap->action_valid=(uint8_t)getbits(&r,1u);
    p_snap->hump_mm=(uint16_t)getbits(&r,B1_SNAP_W_RANGE);
    p_snap->max_hump_mm=(uint16_t)getbits(&r,B1_SNAP_W_RANGE);
    p_snap->obstacle_cm=(uint16_t)getbits(&r,B1_SNAP_W_RANGE);
    p_snap->side=(uint8_t)getbits(&r,B1_SNAP_W_SIDE);
    p_snap->bypass_count=(uint8_t)getbits(&r,B1_SNAP_W_BYPASS);
    p_snap->faults=(uint8_t)getbits(&r,B1_SNAP_W_FAULTS);
    p_snap->speed_left_mm_s=unsign12((uint16_t)getbits(&r,B1_SNAP_W_SPEED));
    p_snap->speed_right_mm_s=unsign12((uint16_t)getbits(&r,
        B1_SNAP_W_SPEED));
    if(p_snap->barcode>B1_LETTER_MAX && p_snap->barcode!=B1_LETTER_NONE)
    {
        return -1;
    }
    return 0;
}
/** @brief Serialise a command payload. */
size_t b1_encode_command(const b1_command_t *p_cmd, uint8_t *p_buf, size_t cap)
{
    if(p_cmd == NULL || p_buf == NULL || cap < (p_cmd->command
        == B1_CMD_MAPPING ? B1_CMD_LEN_MAPPING : B1_CMD_LEN))
    {
        return 0;
    }
    if(p_cmd->command < B1_CMD_MAPPING || p_cmd->command > B1_CMD_LAST)
    {
        return 0;
    }
    p_buf[0]=p_cmd->command;
    b1_put16(p_buf+B1_CMD_OFF_TRANSACTION,p_cmd->transaction);
    if(p_cmd->command==B1_CMD_MAPPING)
    {
        if(b1_pack_mapping(p_cmd->mapping,p_buf+B1_CMD_OFF_MAPPING)!=0)
        {
            return 0;
        }
        return B1_CMD_LEN_MAPPING;
    }
    return B1_CMD_LEN;
}
/** @brief Parse and validate a command payload. */
int32_t b1_decode_command(const uint8_t *p_buf,size_t n,b1_command_t *p_cmd)
{
    if(p_buf==NULL || p_cmd==NULL || n < B1_CMD_LEN ||
        p_buf[0]<B1_CMD_MAPPING
        || p_buf[0]>B1_CMD_LAST ||
        n != (p_buf[0]==B1_CMD_MAPPING ? B1_CMD_LEN_MAPPING : B1_CMD_LEN))
    {
        return -1;
    }
    memset(p_cmd,0,sizeof(*p_cmd));
    p_cmd->command=p_buf[0];
    p_cmd->transaction=b1_get16(p_buf+B1_CMD_OFF_TRANSACTION);
    if(p_cmd->command==B1_CMD_MAPPING && b1_unpack_mapping(
        p_buf+B1_CMD_OFF_MAPPING,p_cmd->mapping)!=0)
    {
        return -1;
    }
    return 0;
}
/*** end of file ***/

