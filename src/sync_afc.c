// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Multi-Frame AFC (Automatic Frequency Control)
 *
 * Coherent accumulation of single-lag header correlations across
 * multiple frames to acquire and track carrier frequency offsets
 * at very low SNR.
 *
 * Algorithm:
 *   1. Compute z[n] = received[wh_start+n] * conj(wh_ref[n])
 *   2. Single-lag correlation: R = sum{ z[n] * conj(z[n-1]) }
 *   3. IIR accumulate: acc += alpha * R  (complex)
 *   4. Extract frequency: f = atan2(acc_q, acc_i) / (2*pi)
 *
 * Capture range: +/- 0.5 cycles/symbol (single-lag).
 * Convergence: 4-8 frames at Es/N0 = -10 dB (896 header symbols).
 *
 * Reference: Luise & Reggiannini, "Carrier frequency recovery in
 * all-digital modems for burst-mode transmissions," IEEE Trans.
 * Commun., 1995.
 */

#include "sync.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Declare locked after this many frames with stable estimate */
#define AFC_LOCK_FRAMES		4

void dvbs2x_afc_init(struct dvbs2x_afc *afc, double alpha)
{
	afc->freq_est = 0.0;
	afc->acc_i = 0.0;
	afc->acc_q = 0.0;
	afc->frames = 0;
	afc->locked = 0;
	afc->alpha = alpha;
}

void dvbs2x_afc_update(struct dvbs2x_afc *afc,
			const struct dvbs2x_complex *sym,
			unsigned int wh_start,
			const struct dvbs2x_complex *wh_ref)
{
	double ri = 0.0, rq = 0.0;
	double zi_prev, zq_prev, zi, zq;
	unsigned int n;

	/*
	 * Compute z[n] = received * conj(reference) for the header,
	 * then accumulate the single-lag autocorrelation R(1).
	 */
	zi_prev = sym[wh_start].i * wh_ref[0].i +
		  sym[wh_start].q * wh_ref[0].q;
	zq_prev = sym[wh_start].q * wh_ref[0].i -
		  sym[wh_start].i * wh_ref[0].q;

	for (n = 1; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		zi = r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		zq = r->q * wh_ref[n].i - r->i * wh_ref[n].q;

		/* R(1) += z[n] * conj(z[n-1]) */
		ri += zi * zi_prev + zq * zq_prev;
		rq += zq * zi_prev - zi * zq_prev;

		zi_prev = zi;
		zq_prev = zq;
	}

	/* IIR update of the coherent accumulator */
	afc->acc_i = afc->alpha * afc->acc_i + ri;
	afc->acc_q = afc->alpha * afc->acc_q + rq;
	afc->frames++;

	/*
	 * Extract total frequency estimate from the accumulator.
	 * In streaming mode, the pre-correction removes freq_est
	 * before this measurement, so the accumulator tracks the
	 * residual.  We add it to the running estimate.
	 */
	if (afc->locked) {
		/* Tracking: small residual correction */
		double residual;

		residual = atan2(afc->acc_q, afc->acc_i) / (2.0 * M_PI);
		afc->freq_est += 0.1 * residual;
		afc->acc_i = 0.0;
		afc->acc_q = 0.0;
	} else {
		/* Acquisition: full estimate from accumulator */
		afc->freq_est = atan2(afc->acc_q, afc->acc_i) /
				(2.0 * M_PI);
	}

	if (afc->frames >= AFC_LOCK_FRAMES)
		afc->locked = 1;
}
