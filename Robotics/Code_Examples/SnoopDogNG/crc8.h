/*  crc8.h  – header-only CRC-8/ATM (poly 0x07)  */
#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>
#include <stddef.h>

static inline uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    return crc;
}

static inline uint8_t crc8_buf(const uint8_t *buf, size_t len)
{
    uint8_t c = 0;
    while (len--) c = crc8_update(c, *buf++);
    return c;
}

#endif /* CRC8_H */
