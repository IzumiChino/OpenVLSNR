// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X Physical Layer Scrambler
 *
 * Two 18-bit LFSRs (x and y) combined as a Gold code produce a 2-bit
 * value Rn per symbol, used to (de)scramble the PL frame data field.
 * The generator matches ETSI EN 302 307-1 clause 5.5.4 bit-for-bit (the
 * same arithmetic used by GNU Radio gr-dtv build_symbol_scrambler_table).
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.4
 */

#include "scrambler.h"

#define LFSR_MSB	0x20000u	/* bit 17 reseed position */

/* Parity (XOR of bits) of the 18-bit value (a & b). */
static inline unsigned int parity_chk(uint32_t a, uint32_t b)
{
	uint32_t v = a & b;

	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1u;
}

void dvbs2x_scrambler_init(struct dvbs2x_scrambler *scr,
			   unsigned int gold_idx)
{
	unsigned int i;

	scr->gold_idx = gold_idx;
	scr->x_reg = 0x00001u;
	scr->y_reg = 0x3FFFFu;

	/* Gold code: clock the x register gold_idx times (xb tap). */
	for (i = 0; i < gold_idx; i++) {
		unsigned int xb = parity_chk(scr->x_reg, 0x0081u);

		scr->x_reg >>= 1;
		if (xb)
			scr->x_reg |= LFSR_MSB;
	}
}

void dvbs2x_scrambler_reset(struct dvbs2x_scrambler *scr)
{
	dvbs2x_scrambler_init(scr, scr->gold_idx);
}

unsigned int dvbs2x_scrambler_next_rn(struct dvbs2x_scrambler *scr)
{
	unsigned int xa, xb, xc, ya, yb, yc, zna, znb;

	xa = parity_chk(scr->x_reg, 0x8050u);
	xb = parity_chk(scr->x_reg, 0x0081u);
	xc = scr->x_reg & 1u;
	scr->x_reg >>= 1;
	if (xb)
		scr->x_reg |= LFSR_MSB;

	ya = parity_chk(scr->y_reg, 0x04A1u);
	yb = parity_chk(scr->y_reg, 0xFF60u);
	yc = scr->y_reg & 1u;
	scr->y_reg >>= 1;
	if (ya)
		scr->y_reg |= LFSR_MSB;

	zna = xc ^ yc;
	znb = xa ^ yb;
	return (znb << 1) | zna;
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
	dvbs2x_scramble_field(scr, symbols, len, NULL, 1);
}

void dvbs2x_descramble(struct dvbs2x_scrambler *scr,
		       struct dvbs2x_complex *symbols,
		       unsigned int len)
{
	dvbs2x_descramble_field(scr, symbols, len, NULL, 1);
}
