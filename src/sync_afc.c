/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Multi-Frame AFC (Automatic Frequency Control)
 *
 * Single-lag header correlation gives a per-frame raw frequency estimate.
 * At the small offsets that occur in practice (~1e-4 cycles/sym) the
 * per-symbol phase step is below the noise floor, so a single frame's raw
 * estimate is noise-dominated and swings by ~1e-3 frame to frame.  Applying
 * such an estimate as a pre-correction corrupts the data field and loses the
 * frame.
 *
 * To avoid that, the AFC only commits to a non-zero estimate when successive
 * raw estimates AGREE within AFC_CONSISTENT_TOL: a real offset repeats frame
 * to frame, noise does not.  Until the estimate is consistent, freq_est stays
 * 0 and the pre-correction is a no-op, leaving the per-frame coarse recovery
 * and residual phase tracker to handle the (small) offset -- which they do.
 *
 * Algorithm per frame:
 *   1. z[n] = received[wh_start+n] * conj(wh_ref[n])
 *   2. Raw single-lag estimate: f_raw = arg(sum z[n] conj(z[n-1])) / 2pi
 *   3. If |f_raw - f_raw_prev| < tol, count it consistent; else reset.
 *   4. Once enough consistent frames, IIR-track freq_est toward f_raw,
 *      clamped so one bad frame cannot rotate beyond recovery.
 *
 * Capture range: +/- 0.5 cycles/symbol (single-lag).
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

/* Consecutive consistent raw estimates required before committing */
#define AFC_LOCK_FRAMES		3
/* Two raw estimates agree if within this many cycles/symbol */
#define AFC_CONSISTENT_TOL	1.5e-4
/* Max change to freq_est per frame, so one bad frame cannot run away */
#define AFC_MAX_STEP		2.0e-4

void dvbs2x_afc_init(struct dvbs2x_afc *afc, double alpha)
{
	afc->freq_est = 0.0;
	afc->acc_i = 0.0;
	afc->acc_q = 0.0;
	afc->frames = 0;
	afc->locked = 0;
	afc->alpha = alpha;
	afc->raw_prev = 0.0;
	afc->consistent = 0;
}

void dvbs2x_afc_update(struct dvbs2x_afc *afc,
			const struct dvbs2x_complex *sym,
			unsigned int wh_start,
			const struct dvbs2x_complex *wh_ref)
{
	double ri = 0.0;
	double rq = 0.0;
	double zi_prev, zq_prev, zi, zq;
	double f_raw;
	double step;
	unsigned int n;

	/*
	 * Raw single-lag autocorrelation R(1) of this frame's header.
	 * The pre-correction (if any) has already removed freq_est, so this
	 * measures the residual offset for the current frame alone.
	 */
	zi_prev = sym[wh_start].i * wh_ref[0].i +
		  sym[wh_start].q * wh_ref[0].q;
	zq_prev = sym[wh_start].q * wh_ref[0].i -
		  sym[wh_start].i * wh_ref[0].q;

	for (n = 1; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		zi = r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		zq = r->q * wh_ref[n].i - r->i * wh_ref[n].q;

		ri += zi * zi_prev + zq * zq_prev;
		rq += zq * zi_prev - zi * zq_prev;

		zi_prev = zi;
		zq_prev = zq;
	}

	f_raw = atan2(rq, ri) / (2.0 * M_PI);
	afc->frames++;

	/*
	 * Consistency gate: a genuine offset gives a repeatable raw estimate;
	 * noise does not.  Only count this frame as consistent if it agrees
	 * with the previous raw estimate.
	 */
	if (afc->frames > 1 &&
	    fabs(f_raw - afc->raw_prev) < AFC_CONSISTENT_TOL) {
		if (afc->consistent < AFC_LOCK_FRAMES)
			afc->consistent++;
	} else {
		afc->consistent = 0;
	}
	afc->raw_prev = f_raw;

	/*
	 * Only commit to a correction once the raw estimate has been
	 * consistent for AFC_LOCK_FRAMES frames.  Track toward the residual
	 * (which the pre-correction has already partly removed), clamped to a
	 * small step so a single bad frame cannot run the estimate away.
	 */
	if (afc->consistent >= AFC_LOCK_FRAMES) {
		afc->locked = 1;
		step = f_raw;
		if (step > AFC_MAX_STEP)
			step = AFC_MAX_STEP;
		else if (step < -AFC_MAX_STEP)
			step = -AFC_MAX_STEP;
		afc->freq_est += step;
	} else {
		/*
		 * Not yet trustworthy: leave freq_est unchanged (0 until the
		 * first lock) so the pre-correction stays a no-op and the
		 * per-frame coarse/residual recovery carries the frame.
		 */
		afc->locked = 0;
	}
}
