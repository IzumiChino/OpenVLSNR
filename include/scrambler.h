/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X Physical Layer Scrambler
 *
 * The PL scrambling sequence Rn is a 2-bit value generated per symbol
 * by two 18-bit LFSRs (x and y) combined as a Gold code, exactly as in
 * ETSI EN 302 307-1 clause 5.5.4 / Annex F:
 *
 *   x init = 0x00001, y init = 0x3FFFF
 *   per symbol:
 *     zna = (x & 1) ^ (y & 1)
 *     znb = parity(x & 0x8050) ^ parity(y & 0xFF60)
 *     Rn  = (znb << 1) | zna
 *     advance x: xb = parity(x & 0x0081); x >>= 1; if (xb) x |= 0x20000
 *     advance y: ya = parity(y & 0x04A1); y >>= 1; if (ya) y |= 0x20000
 *
 * The Gold code index n is applied by clocking x n times (xb tap) before
 * generation.  Rn maps to a rotation that is applied to each PL-frame
 * symbol:
 *   QPSK data and ALL pilots (4-phase):
 *     0 -> +1   1 -> +j   2 -> -1   3 -> -j
 *   non-QPSK data (BPSK / BPSK-SF2), real +/-1 only:
 *     sign = (Rn & 1) ? -1 : +1
 *
 * For VL-SNR, the data field begins reading Rn at index
 * VLSNR_HDR_LEN (900): the scrambler is advanced 900 symbols (through
 * the header positions) before the first data-field symbol.
 */

#ifndef DVBS2X_SCRAMBLER_H
#define DVBS2X_SCRAMBLER_H

#include "dvbs2x_types.h"

/* LFSR register length */
#define DVBS2X_SCRAMBLER_REG_LEN	18

/* Maximum scrambling code index */
#define DVBS2X_SCRAMBLER_MAX_IDX	148574

/* Scrambler state */
struct dvbs2x_scrambler {
	uint32_t	x_reg;
	uint32_t	y_reg;
	unsigned int	gold_idx;
};

/*
 * dvbs2x_scrambler_init - Initialize scrambler with Gold code index
 * @scr: scrambler state
 * @gold_idx: Gold sequence index (root code, 0 for the default)
 */
void dvbs2x_scrambler_init(struct dvbs2x_scrambler *scr,
			   unsigned int gold_idx);

/*
 * dvbs2x_scrambler_reset - Reset scrambler to its initial (post-init) state
 * @scr: scrambler state (must have been initialized)
 */
void dvbs2x_scrambler_reset(struct dvbs2x_scrambler *scr);

/*
 * dvbs2x_scrambler_next_rn - Return the next 2-bit Rn and advance one step
 * @scr: scrambler state
 */
unsigned int dvbs2x_scrambler_next_rn(struct dvbs2x_scrambler *scr);

/*
 * dvbs2x_scrambler_seek - Advance the scrambler by @n symbols (discard Rn)
 * @scr: scrambler state
 * @n: number of steps (e.g. VLSNR_HDR_LEN for the VL-SNR data field)
 */
void dvbs2x_scrambler_seek(struct dvbs2x_scrambler *scr, unsigned int n);

/*
 * dvbs2x_scramble_field - Scramble a PL data field (data + pilots) in place
 * @scr: scrambler state (seek to the data-field start index first)
 * @symbols: data-field symbols, modified in place
 * @len: number of symbols
 * @is_pilot: per-symbol map, 1 = pilot, 0 = data (NULL = all data)
 * @is_qpsk: non-zero if the data modulation is QPSK (data uses 4-phase too)
 *
 * Pilots always use the 4-phase rotation; non-QPSK data uses real +/-1.
 */
void dvbs2x_scramble_field(struct dvbs2x_scrambler *scr,
			   struct dvbs2x_complex *symbols,
			   unsigned int len,
			   const uint8_t *is_pilot,
			   int is_qpsk);

/*
 * dvbs2x_descramble_field - Inverse of dvbs2x_scramble_field
 */
void dvbs2x_descramble_field(struct dvbs2x_scrambler *scr,
			     struct dvbs2x_complex *symbols,
			     unsigned int len,
			     const uint8_t *is_pilot,
			     int is_qpsk);

/*
 * dvbs2x_scramble / dvbs2x_descramble - Whole-buffer 4-phase (de)scramble
 * @scr: scrambler state
 * @symbols: symbols, modified in place
 * @len: number of symbols
 *
 * Applies the 4-phase rotation to every symbol (QPSK convention).
 */
void dvbs2x_scramble(struct dvbs2x_scrambler *scr,
		     struct dvbs2x_complex *symbols,
		     unsigned int len);

void dvbs2x_descramble(struct dvbs2x_scrambler *scr,
		       struct dvbs2x_complex *symbols,
		       unsigned int len);

#endif /* DVBS2X_SCRAMBLER_H */
