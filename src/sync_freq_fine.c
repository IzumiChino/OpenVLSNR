// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Fine Frequency Estimation
 *
 * Pilot-aided fine frequency estimation using correlation
 * across multiple pilot blocks with configurable lag.
 *
 * Algorithm:
 *   For each pilot block, compute correlation between received
 *   pilots and reference pilots at multiple lags. The frequency
 *   offset is estimated from the phase slope:
 *
 *   f_est = angle(sum_of_correlations) / (pi * (NumLags + 1))
 *
 * Can track offsets up to 3.5% of the symbol rate.
 * Uses sliding window across multiple frames for averaging.
 *
 * Reference: ETSI TR 102 376-1, Section 5.6
 */

#include "sync.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dvbs2x_freq_fine_init(struct dvbs2x_freq_fine *ff,
			   unsigned int num_lags)
{
	ff->freq_est = 0.0;
	ff->num_lags = num_lags;
	ff->corr_i = 0.0;
	ff->corr_q = 0.0;
	ff->frame_count = 0;
}

double dvbs2x_freq_fine_estimate(struct dvbs2x_freq_fine *ff,
				 const struct dvbs2x_complex *pilots,
				 const struct dvbs2x_complex *ref_pilots,
				 unsigned int num_pilots)
{
	unsigned int lag;
	double sum_i = 0.0;
	double sum_q = 0.0;
	unsigned int n;

	if (num_pilots < ff->num_lags + 1)
		return ff->freq_est;

	/*
	 * Compute auto-correlation at each lag:
	 *   R(lag) = sum_n { z(n+lag) * conj(z(n)) }
	 * where z(n) = pilots(n) * conj(ref_pilots(n))
	 *
	 * Then sum R(1) through R(num_lags).
	 */
	for (lag = 1; lag <= ff->num_lags; lag++) {
		double lag_i = 0.0;
		double lag_q = 0.0;

		for (n = 0; n + lag < num_pilots; n++) {
			double z0_i, z0_q;
			double z1_i, z1_q;

			/* z(n) = pilot(n) * conj(ref(n)) */
			z0_i = pilots[n].i * ref_pilots[n].i +
			       pilots[n].q * ref_pilots[n].q;
			z0_q = pilots[n].q * ref_pilots[n].i -
			       pilots[n].i * ref_pilots[n].q;

			/* z(n+lag) = pilot(n+lag) * conj(ref(n+lag)) */
			z1_i = pilots[n + lag].i * ref_pilots[n + lag].i +
			       pilots[n + lag].q * ref_pilots[n + lag].q;
			z1_q = pilots[n + lag].q * ref_pilots[n + lag].i -
			       pilots[n + lag].i * ref_pilots[n + lag].q;

			/* R(lag) += z(n+lag) * conj(z(n)) */
			lag_i += z1_i * z0_i + z1_q * z0_q;
			lag_q += z1_q * z0_i - z1_i * z0_q;
		}

		sum_i += lag_i;
		sum_q += lag_q;
	}

	/* Sliding window accumulation */
	ff->corr_i = 0.9 * ff->corr_i + 0.1 * sum_i;
	ff->corr_q = 0.9 * ff->corr_q + 0.1 * sum_q;
	ff->frame_count++;

	/* Frequency estimate from phase of accumulated correlation */
	ff->freq_est = atan2(ff->corr_q, ff->corr_i) /
		       (M_PI * (ff->num_lags + 1));

	return ff->freq_est;
}
