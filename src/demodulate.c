// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Soft Demapper
 *
 * Computes log-likelihood ratios (LLR) for received symbols.
 * Positive LLR indicates bit 0 is more likely.
 *
 * For pi/2-BPSK: LLR = 2 * Re(y * conj(r)) / sigma^2
 * where r is the reference constellation point and y is received.
 *
 * For QPSK: independent LLR for each bit dimension.
 */

#include "modulator.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M_SQRT1_2 0.70710678118654752440

/*
 * dvbs2x_demod_pi2bpsk - Compute LLRs for pi/2-BPSK symbols
 * @symbols: received complex symbols
 * @llr: output LLR values (one per symbol)
 * @len: number of symbols
 * @noise_var: noise variance (sigma^2)
 */
void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
			  double *llr, unsigned int len,
			  double noise_var)
{
	unsigned int n;
	double angle;
	double cos_a, sin_a;
	double re_derot;

	for (n = 0; n < len; n++) {
		/* Remove pi/2 rotation */
		angle = (double)(n % 4) * M_PI / 2.0;
		cos_a = cos(angle);
		sin_a = sin(angle);

		/* De-rotate: multiply by exp(-j*angle) */
		re_derot = symbols[n].i * cos_a + symbols[n].q * sin_a;

		/*
		 * LLR for BPSK: 2 * y_real / sigma^2
		 * Positive means bit 0 (+1) more likely
		 */
		llr[n] = 2.0 * re_derot / noise_var;
	}
}

/*
 * dvbs2x_demod_qpsk - Compute LLRs for QPSK symbols
 * @symbols: received complex symbols
 * @llr: output LLR values (two per symbol)
 * @num_symbols: number of symbols
 * @noise_var: noise variance (sigma^2)
 */
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

/*
 * dvbs2x_demod_despread - Combine spread symbols (factor 2)
 * @in: input symbols (2 * out_len)
 * @out: output combined symbols
 * @out_len: number of output symbols
 *
 * Averages pairs of consecutive symbols.
 */
void dvbs2x_demod_despread(const struct dvbs2x_complex *in,
			   struct dvbs2x_complex *out,
			   unsigned int out_len)
{
	unsigned int n;

	for (n = 0; n < out_len; n++) {
		out[n].i = (in[2 * n].i + in[2 * n + 1].i) * 0.5;
		out[n].q = (in[2 * n].q + in[2 * n + 1].q) * 0.5;
	}
}
