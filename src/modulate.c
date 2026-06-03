// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR pi/2-BPSK and QPSK Modulator
 *
 * pi/2-BPSK uses the standard period-2 diagonal constellation (ETSI
 * EN 302 307-2 clause 5.4.1, identical to gr-dtv m_bpsk[i & 1][b]):
 *
 *   even symbol index:  bit 0 -> (+,+)/sqrt2   bit 1 -> (-,-)/sqrt2
 *   odd  symbol index:  bit 0 -> (-,+)/sqrt2   bit 1 -> (+,-)/sqrt2
 *
 * Successive symbols alternate between the two diagonals, giving the
 * pi/2 rotation while keeping a constant envelope.
 *
 * QPSK: Gray-coded mapping per ETSI EN 302 307-1 Table 9.
 */

#include "modulator.h"

#define M_SQRT1_2 0.70710678118654752440

void dvbs2x_mod_pi2bpsk(const uint8_t *bits,
			struct dvbs2x_complex *symbols,
			unsigned int len)
{
	unsigned int n;

	for (n = 0; n < len; n++) {
		unsigned int b = bits[n] & 1;

		if ((n & 1) == 0) {
			/* even: m_bpsk[0][b] */
			symbols[n].i = b ? -M_SQRT1_2 : M_SQRT1_2;
			symbols[n].q = b ? -M_SQRT1_2 : M_SQRT1_2;
		} else {
			/* odd: m_bpsk[1][b] */
			symbols[n].i = b ? M_SQRT1_2 : -M_SQRT1_2;
			symbols[n].q = b ? -M_SQRT1_2 : M_SQRT1_2;
		}
	}
}

void dvbs2x_mod_qpsk(const uint8_t *bits,
		     struct dvbs2x_complex *symbols,
		     unsigned int num_symbols)
{
	unsigned int n;
	uint8_t b0, b1;

	for (n = 0; n < num_symbols; n++) {
		b0 = bits[2 * n];
		b1 = bits[2 * n + 1];

		/*
		 * Gray-coded QPSK (DVB-S2 Table 9):
		 *   b0 b1 -> I, Q
		 *   0  0  -> +1/sqrt(2), +1/sqrt(2)
		 *   0  1  -> +1/sqrt(2), -1/sqrt(2)
		 *   1  0  -> -1/sqrt(2), +1/sqrt(2)
		 *   1  1  -> -1/sqrt(2), -1/sqrt(2)
		 */
		symbols[n].i = b0 ? -M_SQRT1_2 : M_SQRT1_2;
		symbols[n].q = b1 ? -M_SQRT1_2 : M_SQRT1_2;
	}
}

void dvbs2x_mod_spread_bits(const uint8_t *in, uint8_t *out,
			    unsigned int in_len)
{
	unsigned int n;

	/* Spread factor 2: duplicate each coded bit. */
	for (n = 0; n < in_len; n++) {
		out[2 * n] = in[n];
		out[2 * n + 1] = in[n];
	}
}
