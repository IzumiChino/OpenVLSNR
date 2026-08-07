// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2 Physical Layer Scrambler
 *
 * Two 18-bit LFSRs (x and y) combined as a Gold code produce a 2-bit
 * value Rn per symbol, used to (de)scramble the PL frame data field.
 * The generator matches ETSI EN 302 307-1 clause 5.5.4 bit-for-bit (the
 * same arithmetic used by GNU Radio gr-dtv build_symbol_scrambler_table).
 *
 * The PLFRAME payload (everything after the PLHEADER, and for VL-SNR
 * after the VL-SNR header) is multiplied by a complex randomisation
 * sequence Cn = exp(j * Rn * pi/2), Rn in {0,1,2,3}.  Rn comes from a
 * Gold code of two 18-bit shift registers: the x register is seeded
 * with the scrambling (root) code n and the y register starts all ones.
 * The register taps and the Rn construction follow the reference
 * implementation so the sequence is bit-exact with on-air signals.
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.4
 */

#include "scrambler.h"

/* Feedback bit (x^18 term) loaded into the top of the 18-bit register. */
#define SCRAMBLER_FEEDBACK	0x20000u

/* Parity (XOR of all set bits) of (a & b). */
static int parity_chk(uint32_t a, uint32_t b)
{
	a &= b;
	a ^= a >> 16;
	a ^= a >> 8;
	a ^= a >> 4;
	a ^= a >> 2;
	a ^= a >> 1;
	return (int)(a & 1);
}

unsigned int dvbs2x_scrambler_next_rn(struct dvbs2x_scrambler *scr)
{
	int xa, xb, xc, ya, yb, yc;

	xa = parity_chk(scr->x_reg, 0x8050);
	xb = parity_chk(scr->x_reg, 0x0081);
	xc = (int)(scr->x_reg & 1);
	scr->x_reg >>= 1;
	if (xb)
		scr->x_reg |= SCRAMBLER_FEEDBACK;

	ya = parity_chk(scr->y_reg, 0x04A1);
	yb = parity_chk(scr->y_reg, 0xFF60);
	yc = (int)(scr->y_reg & 1);
	scr->y_reg >>= 1;
	if (ya)
		scr->y_reg |= SCRAMBLER_FEEDBACK;

	/* Rn = (znb << 1) + zna, with zna = xc ^ yc and znb = xa ^ yb */
	return (unsigned int)(((xa ^ yb) << 1) + (xc ^ yc));
}

void dvbs2x_scrambler_init(struct dvbs2x_scrambler *scr,
			   unsigned int gold_idx)
{
	uint32_t x = 1;
	unsigned int g;

	scr->gold_idx = gold_idx;
	scr->x_reg = 0x00001u;
	scr->y_reg = 0x3FFFFu;

	/*
	 * The x register root state is the standard x sequence (init 1)
	 * advanced by gold_idx steps; the y register starts all ones.
	 * Gold code 0 therefore seeds x with 1, not 0.
	 */
	for (g = 0; g < gold_idx; g++)
		x = (((x ^ (x >> 7)) & 1) << 17) | (x >> 1);
	scr->x_reg = x;
	scr->y_reg = 0x3ffffu;
}

void dvbs2x_scrambler_reset(struct dvbs2x_scrambler *scr)
{
	dvbs2x_scrambler_init(scr, scr->gold_idx);
}

void dvbs2x_scrambler_seek(struct dvbs2x_scrambler *scr, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		dvbs2x_scrambler_next_rn(scr);
}

/* Apply the forward 4-phase rotation for Rn to one symbol. */
static inline void rot_fwd(struct dvbs2x_complex *s, unsigned int rn)
{
	double ti, tq;

	switch (rn) {
	case 1:				/* +90 deg: j * s */
		ti = -s->q;
		tq = s->i;
		break;
	case 2:				/* 180 deg: -s */
		ti = -s->i;
		tq = -s->q;
		break;
	case 3:				/* -90 deg: -j * s */
		ti = s->q;
		tq = -s->i;
		break;
	default:			/* 0 deg */
		return;
	}
	s->i = ti;
	s->q = tq;
}

/* Apply the inverse 4-phase rotation for Rn to one symbol. */
static inline void rot_inv(struct dvbs2x_complex *s, unsigned int rn)
{
	double ti, tq;

	switch (rn) {
	case 1:				/* inverse of +90: -j * s */
		ti = s->q;
		tq = -s->i;
		break;
	case 2:				/* 180 deg: -s */
		ti = -s->i;
		tq = -s->q;
		break;
	case 3:				/* inverse of -90: j * s */
		ti = -s->q;
		tq = s->i;
		break;
	default:
		return;
	}
	s->i = ti;
	s->q = tq;
}

void dvbs2x_scramble_field(struct dvbs2x_scrambler *scr,
			   struct dvbs2x_complex *symbols,
			   unsigned int len,
			   const uint8_t *is_pilot,
			   int is_qpsk)
{
	unsigned int n;

	for (n = 0; n < len; n++) {
		unsigned int rn = dvbs2x_scrambler_next_rn(scr);
		int four_phase = is_qpsk || (is_pilot && is_pilot[n]);

		if (four_phase) {
			rot_fwd(&symbols[n], rn);
		} else if (rn & 1) {
			/* non-QPSK data: real +/-1 only */
			symbols[n].i = -symbols[n].i;
			symbols[n].q = -symbols[n].q;
		}
	}
}

void dvbs2x_descramble_field(struct dvbs2x_scrambler *scr,
			     struct dvbs2x_complex *symbols,
			     unsigned int len,
			     const uint8_t *is_pilot,
			     int is_qpsk)
{
	unsigned int n;

	for (n = 0; n < len; n++) {
		unsigned int rn = dvbs2x_scrambler_next_rn(scr);
		int four_phase = is_qpsk || (is_pilot && is_pilot[n]);

		if (four_phase) {
			rot_inv(&symbols[n], rn);
		} else if (rn & 1) {
			symbols[n].i = -symbols[n].i;
			symbols[n].q = -symbols[n].q;
		}
	}
}

void dvbs2x_scramble(struct dvbs2x_scrambler *scr,
		     struct dvbs2x_complex *symbols,
		     unsigned int len)
{
	unsigned int n;

	for (n = 0; n < len; n++) {
		double si = symbols[n].i;
		double sq = symbols[n].q;

		switch (dvbs2x_scrambler_next_rn(scr)) {
		case 1:				/* x j */
			symbols[n].i = -sq;
			symbols[n].q = si;
			break;
		case 2:				/* x -1 */
			symbols[n].i = -si;
			symbols[n].q = -sq;
			break;
		case 3:				/* x -j */
			symbols[n].i = sq;
			symbols[n].q = -si;
			break;
		default:			/* x 1 */
			break;
		}
	}
}

void dvbs2x_descramble(struct dvbs2x_scrambler *scr,
		       struct dvbs2x_complex *symbols,
		       unsigned int len)
{
	unsigned int n;

	for (n = 0; n < len; n++) {
		double si = symbols[n].i;
		double sq = symbols[n].q;

		/* Apply conj(Cn) = exp(-j * Rn * pi/2). */
		switch (dvbs2x_scrambler_next_rn(scr)) {
		case 1:				/* x -j */
			symbols[n].i = sq;
			symbols[n].q = -si;
			break;
		case 2:				/* x -1 */
			symbols[n].i = -si;
			symbols[n].q = -sq;
			break;
		case 3:				/* x j */
			symbols[n].i = -sq;
			symbols[n].q = si;
			break;
		default:			/* x 1 */
			break;
		}
	}
}
