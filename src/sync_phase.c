// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Phase Estimation
 *
 * Pilot-aided phase estimation with optional smoothing filter
 * for very low SNR operation.
 *
 * Algorithm:
 *   1. Compute phase from pilot correlation:
 *      phi = angle(sum(pilot(n) * conj(ref(n))))
 *   2. Apply first-order IIR filter:
 *      phi_filt = alpha * phi + (1 - alpha) * phi_prev
 *   3. Optionally apply moving average smoothing
 *   4. Correct data symbols by rotating by -phi_filt
 *
 * For Es/N0 < -8 dB, set alpha < 0.5 and enable smoothing.
 *
 * Reference: ETSI TR 102 376-1, Section 5.7
 */

#include "sync.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SMOOTH_WINDOW	5

void dvbs2x_phase_est_init(struct dvbs2x_phase_est *pe,
			   double alpha, int smooth)
{
	pe->phase = 0.0;
	pe->alpha = alpha;
	pe->need_smooth = smooth;
}

void dvbs2x_phase_est_process(struct dvbs2x_phase_est *pe,
			      struct dvbs2x_complex *symbols,
			      const struct dvbs2x_complex *pilots,
			      const struct dvbs2x_complex *ref_pilots,
			      unsigned int num_pilots,
			      unsigned int num_symbols)
{
	double corr_i = 0.0;
	double corr_q = 0.0;
	double raw_phase;
	double cos_p, sin_p;
	double ti, tq;
	unsigned int n;

	if (num_pilots == 0)
		return;

	/* Compute phase from pilot correlation */
	for (n = 0; n < num_pilots; n++) {
		corr_i += pilots[n].i * ref_pilots[n].i +
			  pilots[n].q * ref_pilots[n].q;
		corr_q += pilots[n].q * ref_pilots[n].i -
			  pilots[n].i * ref_pilots[n].q;
	}

	raw_phase = atan2(corr_q, corr_i);

	/* Phase unwrapping relative to previous estimate */
	while (raw_phase - pe->phase > M_PI)
		raw_phase -= 2.0 * M_PI;
	while (raw_phase - pe->phase < -M_PI)
		raw_phase += 2.0 * M_PI;

	/* IIR filter */
	pe->phase = pe->alpha * raw_phase + (1.0 - pe->alpha) * pe->phase;

	/* Apply phase correction to all data symbols */
	cos_p = cos(-pe->phase);
	sin_p = sin(-pe->phase);

	for (n = 0; n < num_symbols; n++) {
		ti = symbols[n].i * cos_p - symbols[n].q * sin_p;
		tq = symbols[n].i * sin_p + symbols[n].q * cos_p;
		symbols[n].i = ti;
		symbols[n].q = tq;
	}
}
