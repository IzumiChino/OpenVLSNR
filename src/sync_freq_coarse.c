// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Coarse Frequency Synchronization
 *
 * Balanced Quadricorrelator Frequency Error Detector (BQ-FED)
 * in a first-order frequency locked loop (FLL).
 *
 * The BQ-FED estimates frequency offset from the rate of phase
 * change between consecutive symbols:
 *   e(n) = Im{y(n) * conj(y(n-1))}
 *
 * This is a decision-directed frequency detector that works
 * for any constant-envelope modulation (BPSK, QPSK).
 *
 * Can track offsets up to 20% of the symbol rate.
 *
 * Reference: ETSI TR 102 376-1, Annex C
 */

#include "sync.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dvbs2x_freq_coarse_init(struct dvbs2x_freq_coarse *fc,
			     double loop_bw)
{
	fc->freq_est = 0.0;
	fc->loop_bw = loop_bw;
	fc->phase_acc = 0.0;
	fc->integrator = 0.0;
}

void dvbs2x_freq_coarse_process(struct dvbs2x_freq_coarse *fc,
				struct dvbs2x_complex *symbols,
				unsigned int len)
{
	unsigned int n;
	double prev_i, prev_q;
	double cur_i, cur_q;
	double err;
	double cos_p, sin_p;

	if (len == 0)
		return;

	prev_i = symbols[0].i;
	prev_q = symbols[0].q;

	for (n = 0; n < len; n++) {
		/* Apply current frequency correction */
		cos_p = cos(fc->phase_acc);
		sin_p = sin(fc->phase_acc);
		cur_i = symbols[n].i * cos_p + symbols[n].q * sin_p;
		cur_q = -symbols[n].i * sin_p + symbols[n].q * cos_p;
		symbols[n].i = cur_i;
		symbols[n].q = cur_q;

		/* BQ-FED: cross-product frequency error */
		if (n > 0) {
			err = cur_i * prev_q - cur_q * prev_i;

			/* First-order loop filter (integrator) */
			fc->integrator += fc->loop_bw * err;
			fc->freq_est = fc->integrator;
		}

		/* Update phase accumulator */
		fc->phase_acc += fc->freq_est;

		/* Keep phase in [-pi, pi] */
		while (fc->phase_acc > M_PI)
			fc->phase_acc -= 2.0 * M_PI;
		while (fc->phase_acc < -M_PI)
			fc->phase_acc += 2.0 * M_PI;

		prev_i = cur_i;
		prev_q = cur_q;
	}
}
