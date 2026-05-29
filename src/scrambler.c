// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X Physical Layer Scrambler
 *
 * Gold sequence generation using two 18-bit LFSRs:
 *   x sequence: x^18 + x^7 + 1 (polynomial 0x20080)
 *   y sequence: x^18 + x^5 + x^2 + x + 1 (polynomial 0x20027)
 *
 * The Gold code index n initializes the y-register to the state
 * obtained after clocking the all-ones initial state n times.
 *
 * Two consecutive Gold sequence bits (R0, R1) determine the
 * scrambling rotation applied to each symbol:
 *   (0,0) -> exp(j*0)     = +1
 *   (0,1) -> exp(j*pi/2)  = +j
 *   (1,0) -> exp(j*pi)    = -1
 *   (1,1) -> exp(j*3pi/2) = -j
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.4
 */

#include "scrambler.h"

/* LFSR feedback tap masks (bit positions for XOR) */
#define X_POLY_TAPS	((1u << 7) | (1u << 0))   /* x^7 + 1 */
#define Y_POLY_TAPS	((1u << 5) | (1u << 2) | (1u << 1) | (1u << 0))

#define LFSR_MASK	((1u << 18) - 1)

static inline unsigned int lfsr_x_step(uint32_t *reg)
{
	unsigned int out;
	unsigned int fb;

	out = (*reg >> 17) & 1;
	fb = ((*reg >> 17) ^ (*reg >> 6)) & 1;
	*reg = ((*reg << 1) | fb) & LFSR_MASK;
	return out;
}

static inline unsigned int lfsr_y_step(uint32_t *reg)
{
	unsigned int out;
	unsigned int fb;

	out = (*reg >> 17) & 1;
	fb = ((*reg >> 17) ^ (*reg >> 4) ^ (*reg >> 1) ^ (*reg >> 0)) & 1;
	*reg = ((*reg << 1) | fb) & LFSR_MASK;
	return out;
}

void dvbs2x_scrambler_init(struct dvbs2x_scrambler *scr,
			   unsigned int gold_idx)
{
	unsigned int i;

	scr->gold_idx = gold_idx;

	/* x-register always starts at all ones */
	scr->x_reg = LFSR_MASK;

	/* y-register: start at all ones, then advance gold_idx steps */
	scr->y_reg = LFSR_MASK;
	for (i = 0; i < gold_idx; i++)
		lfsr_y_step(&scr->y_reg);
}

void dvbs2x_scrambler_reset(struct dvbs2x_scrambler *scr)
{
	dvbs2x_scrambler_init(scr, scr->gold_idx);
}

/*
 * Generate next scrambling rotation.
 * Returns complex multiplier as (i, q) pair.
 */
static void scrambler_next(struct dvbs2x_scrambler *scr,
			   double *si, double *sq)
{
	unsigned int r0, r1;
	unsigned int x0, x1, y0, y1;

	/* Generate two Gold sequence bits */
	x0 = lfsr_x_step(&scr->x_reg);
	y0 = lfsr_y_step(&scr->y_reg);
	r0 = x0 ^ y0;

	x1 = lfsr_x_step(&scr->x_reg);
	y1 = lfsr_y_step(&scr->y_reg);
	r1 = x1 ^ y1;

	/*
	 * Map (R0, R1) to complex rotation:
	 *   (0,0) -> +1 + 0j
	 *   (0,1) ->  0 + 1j
	 *   (1,0) -> -1 + 0j
	 *   (1,1) ->  0 - 1j
	 */
	switch ((r0 << 1) | r1) {
	case 0:
		*si = 1.0;
		*sq = 0.0;
		break;
	case 1:
		*si = 0.0;
		*sq = 1.0;
		break;
	case 2:
		*si = -1.0;
		*sq = 0.0;
		break;
	case 3:
		*si = 0.0;
		*sq = -1.0;
		break;
	}
}

void dvbs2x_scramble(struct dvbs2x_scrambler *scr,
		     struct dvbs2x_complex *symbols,
		     unsigned int len)
{
	unsigned int n;
	double si, sq;
	double ti, tq;

	for (n = 0; n < len; n++) {
		scrambler_next(scr, &si, &sq);

		/* Complex multiply: symbol * scramble */
		ti = symbols[n].i * si - symbols[n].q * sq;
		tq = symbols[n].i * sq + symbols[n].q * si;
		symbols[n].i = ti;
		symbols[n].q = tq;
	}
}

void dvbs2x_descramble(struct dvbs2x_scrambler *scr,
		       struct dvbs2x_complex *symbols,
		       unsigned int len)
{
	unsigned int n;
	double si, sq;
	double ti, tq;

	for (n = 0; n < len; n++) {
		scrambler_next(scr, &si, &sq);

		/* Complex multiply: symbol * conj(scramble) */
		ti = symbols[n].i * si + symbols[n].q * sq;
		tq = -symbols[n].i * sq + symbols[n].q * si;
		symbols[n].i = ti;
		symbols[n].q = tq;
	}
}
