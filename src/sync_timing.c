// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Symbol Timing Recovery
 *
 * Gardner (non-data-aided) timing error detector with a
 * proportional-integral loop filter.
 *
 * The Gardner TED computes:
 *   e(n) = Re{y(n-1/2) * conj(y(n) - y(n-1))}
 *
 * where y(n-1/2) is the midpoint sample between two symbol
 * instants. This works for any PSK/QAM constellation without
 * requiring knowledge of the transmitted symbols.
 *
 * Reference: F.M. Gardner, "A BPSK/QPSK Timing-Error Detector
 *            for Sampled Receivers," IEEE Trans. Comm., 1986.
 */

#include "sync.h"
#include <math.h>
#include <string.h>

void dvbs2x_timing_sync_init(struct dvbs2x_timing_sync *ts,
			     unsigned int sps, double loop_bw)
{
	double damp = 1.0 / sqrt(2.0);
	double bw_norm = loop_bw;
	double denom;

	ts->sps = sps;
	ts->loop_bw = loop_bw;
	ts->mu = 0.0;
	ts->integrator = 0.0;

	/*
	 * PI loop filter gains from loop bandwidth and damping:
	 *   Kp = (4 * damp * bw_norm) / (1 + 2*damp*bw_norm + bw_norm^2)
	 *   Ki = (4 * bw_norm^2) / (1 + 2*damp*bw_norm + bw_norm^2)
	 */
	denom = 1.0 + 2.0 * damp * bw_norm + bw_norm * bw_norm;
	ts->kp = (4.0 * damp * bw_norm) / denom;
	ts->ki = (4.0 * bw_norm * bw_norm) / denom;
}

/*
 * Linear interpolation between two samples.
 */
static struct dvbs2x_complex interp_linear(const struct dvbs2x_complex *s,
					   double mu)
{
	struct dvbs2x_complex result;

	result.i = s[0].i + mu * (s[1].i - s[0].i);
	result.q = s[0].q + mu * (s[1].q - s[0].q);
	return result;
}

void dvbs2x_timing_sync_process(struct dvbs2x_timing_sync *ts,
				const struct dvbs2x_complex *in,
				unsigned int in_len,
				struct dvbs2x_complex *out,
				unsigned int *out_len)
{
	unsigned int idx = 0;
	unsigned int out_idx = 0;
	struct dvbs2x_complex prev_sym = {0.0, 0.0};
	struct dvbs2x_complex mid_sym;
	struct dvbs2x_complex cur_sym;
	double ted_err;
	double loop_out;
	int have_prev = 0;

	while (idx + ts->sps < in_len) {
		unsigned int base;
		int int_mu;

		/* Compute interpolation point */
		int_mu = (int)ts->mu;
		base = idx + int_mu;
		if (base + 1 >= in_len)
			break;

		/* Interpolate at symbol instant */
		cur_sym = interp_linear(&in[base], ts->mu - int_mu);

		/* Interpolate at midpoint (half symbol earlier) */
		if (have_prev && base >= ts->sps / 2) {
			unsigned int mid_base;
			double mid_mu;

			mid_base = base - ts->sps / 2;
			mid_mu = ts->mu - int_mu;
			if (mid_base + 1 < in_len)
				mid_sym = interp_linear(&in[mid_base], mid_mu);
			else
				mid_sym = cur_sym;

			/* Gardner TED */
			ted_err = mid_sym.i * (prev_sym.i - cur_sym.i) +
				  mid_sym.q * (prev_sym.q - cur_sym.q);

			/* PI loop filter */
			ts->integrator += ts->ki * ted_err;
			loop_out = ts->kp * ted_err + ts->integrator;

			/* Update fractional delay */
			ts->mu += ts->sps + loop_out;
		} else {
			ts->mu += ts->sps;
		}

		out[out_idx] = cur_sym;
		out_idx++;
		prev_sym = cur_sym;
		have_prev = 1;

		/* Advance input index */
		idx += (unsigned int)ts->mu;
		ts->mu -= (unsigned int)ts->mu;
	}

	*out_len = out_idx;
}
