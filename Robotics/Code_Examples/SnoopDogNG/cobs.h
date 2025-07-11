#ifndef COBS_H
#define COBS_H

#include <stddef.h>
#include <stdint.h>

/*  Encodes exactly `len` bytes from `in` into `out`.
 *  Returns the encoded length (always len+1 .. len+ceil(len/254)+1).
 *  `out` must be at least len+1 bytes long.
 */
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out);

/*  Decodes exactly `len` COBS bytes from `in` into `out`.
 *  Returns the decoded length, or 0 if the data are invalid.
 *  `out` must be at least len-1 bytes long.
 */
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out);

#endif /* COBS_H */
