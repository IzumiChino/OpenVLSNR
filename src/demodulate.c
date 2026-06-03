// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Soft Demapper
 *
 * Computes log-likelihood ratios (LLR) for received symbols.  A positive
 * LLR means bit 0 is more likely (matching the LDPC decoder, which takes
 * llr < 0 as bit 1).
 *
 * pi/2-BPSK uses the period-2 diagonal constellation (gr-dtv m_bpsk):
 *   even index: bit 0 at (+,+)/sqrt2 -> decision metric (i + q)/sqrt2
 *   odd  index: bit 0 at (-,+)/sqrt2 -> decision metric (q - i)/sqrt2
 * LLR = 2 * metric / sigma^2 = sqrt(2) * (component sum) / sigma^2.
 *
 * For QPSK the I and Q components carry independent Gray-coded bits.
 */

#include "modulator.h"
#include <math.h>

#define M_SQRT1_2 0.70710678118654752440
#define M_SQRT2_D 1.41421356237309504880

void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
			  double *llr, unsigned int len,
			  double noise_var)
{
	unsigned int n;
	double scale = M_SQRT2_D / noise_var;

	for (n = 0; n < len; n++) {
		if ((n & 1) == 0)
			llr[n] = scale * (symbols[n].i + symbols[n].q);
		else
			llr[n] = scale * (symbols[n].q - symbols[n].i);
	}
}

void dvbs2x_demod_qpsk(const struct dvbs2x_complex *symbols,
			double *llr, unsigned int num_symbols,
			double noise_var)
{
	unsigned int n;
	double scale;

	/* For Gray-coded QPSK, LLR is proportional to I/Q component */
	scale = 2.0 * M_SQRT1_2 / noise_var;

	for (n = 0; n < num_symbols; n++) {
		/* LLR for bit 0 (I component) */
		llr[2 * n] = scale * symbols[n].i;
		/* LLR for bit 1 (Q component) */
		llr[2 * n + 1] = scale * symbols[n].q;
	}
}

void dvbs2x_demod_despread_llr(const double *in, double *out,
			       unsigned int out_len)
{
	unsigned int n;

	/*
	 * Despread factor 2: sum the LLRs from each symbol pair (same
	 * coded bit at even and odd pi/2-BPSK phases).
	 */
	for (n = 0; n < out_len; n++)
		out[n] = in[2 * n] + in[2 * n + 1];
}
