// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR pi/2-BPSK and QPSK Modulator
 *
 * pi/2-BPSK: each symbol is rotated by n*pi/2 where n is the
 * symbol index. This creates a constant-envelope signal that
 * avoids zero crossings.
 *
 * QPSK: Gray-coded mapping per ETSI EN 302 307-1 Table 9.
 */

#include "modulator.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M_SQRT1_2 0.70710678118654752440

void dvbs2x_mod_pi2bpsk(const uint8_t *bits,
			struct dvbs2x_complex *symbols,
			unsigned int len)
{
	unsigned int n;
	double val;
	double angle;

	for (n = 0; n < len; n++) {
		/* BPSK mapping: 0 -> +1, 1 -> -1 */
		val = bits[n] ? -1.0 : 1.0;

		/* Rotate by n * pi/2 */
		angle = (double)(n % 4) * M_PI / 2.0;
		symbols[n].i = val * cos(angle);
		symbols[n].q = val * sin(angle);
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

void dvbs2x_mod_spread(const struct dvbs2x_complex *in,
		       struct dvbs2x_complex *out,
		       unsigned int in_len)
{
	unsigned int n;

	for (n = 0; n < in_len; n++) {
		out[2 * n] = in[n];
		out[2 * n + 1] = in[n];
	}
}
