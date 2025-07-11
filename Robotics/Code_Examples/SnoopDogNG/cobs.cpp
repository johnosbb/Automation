#include "cobs.h"

/* ------------ Encoder ------------ */
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
    const uint8_t *start_out = out;      /* remember for length calc */

    uint8_t  code  = 1;                  /* length of current block   */
    uint8_t *codep = out++;              /* where the length will go  */

    for (size_t i = 0; i < len; ++i)
    {
        if (in[i] == 0) {                /* terminates a block */
            *codep   = code;
            codep    = out++;            /* reserve next length byte  */
            code     = 1;
        } else {
            *out++   = in[i];
            if (++code == 0xFF) {        /* block full */
                *codep = code;
                codep  = out++;
                code   = 1;
            }
        }
    }
    *codep = code;                       /* length for the final block */
    return (size_t)(out - start_out);
}

/* ------------ Decoder ------------ */
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out)
{
    const uint8_t *end = in + len;
    uint8_t       *start_out = out;

    while (in < end)
    {
        uint8_t code = *in++;
        if (code == 0 || in + code - 1 > end)   /* malformed */
            return 0;

        for (uint8_t i = 1; i < code; ++i)      /* copy data */
            *out++ = *in++;

        if (code != 0xFF && in < end)           /* insert zero except after 0xFF */
            *out++ = 0x00;
    }
    return (size_t)(out - start_out);
}
