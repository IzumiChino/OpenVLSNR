// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X Root Raised Cosine (RRC) Pulse Shaping Filter
 *
 * Designs and applies RRC filters for transmit pulse shaping
 * and receive matched filtering.
 *
 * The RRC impulse response is:
 *   h(t) = [sin(pi*t*(1-r)) + 4*r*t*cos(pi*t*(1+r))] /
 *          [pi*t*(1-(4*r*t)^2)]
 *
 * where r is the roll-off factor and t is normalized to symbol period.
 *
 * Reference: ETSI EN 302 307-1 clause 4.2
 */

#include "filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Compute RRC impulse response sample at time t (normalized)
 */
static double rrc_impulse(double t, double rolloff)
{
	double r = rolloff;
	double num, den;
	double four_r_t;

	if (fabs(t) < 1e-10) {
		/* t = 0: h(0) = 1 - r + 4*r/pi */
		return 1.0 - r + 4.0 * r / M_PI;
	}

	four_r_t = 4.0 * r * t;

	if (fabs(fabs(t) - 1.0 / (4.0 * r)) < 1e-10) {
		/* t = +/- 1/(4r): special case */
		return r / sqrt(2.0) *
		       ((1.0 + 2.0 / M_PI) * sin(M_PI / (4.0 * r)) +
			(1.0 - 2.0 / M_PI) * cos(M_PI / (4.0 * r)));
	}

	num = sin(M_PI * t * (1.0 - r)) +
	      four_r_t * cos(M_PI * t * (1.0 + r));
	den = M_PI * t * (1.0 - four_r_t * four_r_t);

	if (fabs(den) < 1e-15)
		return 0.0;

	return num / den;
}

int dvbs2x_rrc_filter_init(struct dvbs2x_rrc_filter *flt,
			   double rolloff, unsigned int sps,
			   unsigned int span)
{
	unsigned int num_taps;
	unsigned int i;
	double t;
	double energy = 0.0;
	double norm;

	num_taps = 2 * span * sps + 1;
	if (num_taps > DVBS2X_FILTER_MAX_TAPS)
		return -1;

	flt->num_taps = num_taps;
	flt->sps = sps;
	flt->rolloff = rolloff;
	flt->delay_idx = 0;

	/* Design filter coefficients */
	for (i = 0; i < num_taps; i++) {
		t = ((double)i - (double)(num_taps - 1) / 2.0) / (double)sps;
		flt->coeffs[i] = rrc_impulse(t, rolloff);
		energy += flt->coeffs[i] * flt->coeffs[i];
	}

	/* Normalize for unit energy */
	norm = 1.0 / sqrt(energy);
	for (i = 0; i < num_taps; i++)
		flt->coeffs[i] *= norm;

	/* Clear delay lines */
	memset(flt->delay_i, 0, sizeof(flt->delay_i));
	memset(flt->delay_q, 0, sizeof(flt->delay_q));

	return 0;
}

void dvbs2x_rrc_filter_reset(struct dvbs2x_rrc_filter *flt)
{
	memset(flt->delay_i, 0, sizeof(flt->delay_i));
	memset(flt->delay_q, 0, sizeof(flt->delay_q));
	flt->delay_idx = 0;
}

/*
 * Linear convolution helper: compute dot product of the coefficient
 * array with a contiguous segment of the delay line.  The delay line
 * is maintained as a double-length linear buffer (mirror writes) so
 * this inner loop has no branches and auto-vectorizes cleanly.
 */
static inline void filter_one(struct dvbs2x_rrc_filter *flt,
			      double in_i, double in_q,
			      double *out_i, double *out_q)
{
	unsigned int idx = flt->delay_idx;
	unsigned int nt = flt->num_taps;
	const double *c = flt->coeffs;
	const double *di, *dq;
	double si = 0.0, sq = 0.0;
	unsigned int k;

	/* Write to both halves of the double-length buffer */
	flt->delay_i[idx] = in_i;
	flt->delay_i[idx + nt] = in_i;
	flt->delay_q[idx] = in_q;
	flt->delay_q[idx + nt] = in_q;

	/*
	 * The newest sample is at idx; oldest at idx+1 (wrapped).
	 * Reading from idx downward in the double buffer gives a
	 * contiguous block: delay_i[idx+1 .. idx+nt].
	 * Coefficients are stored newest-first, so we read the
	 * delay line starting at idx going backward = idx+nt-k.
	 * Equivalently: di[nt - 1 - k] with di = &delay_i[idx+1].
	 */
	di = &flt->delay_i[idx + 1];
	dq = &flt->delay_q[idx + 1];

	for (k = 0; k < nt; k++) {
		si += di[nt - 1 - k] * c[k];
		sq += dq[nt - 1 - k] * c[k];
	}

	*out_i = si;
	*out_q = sq;

	/* Advance write pointer */
	idx++;
	if (idx >= nt)
		idx = 0;
	flt->delay_idx = idx;
}

void dvbs2x_rrc_upsample(struct dvbs2x_rrc_filter *flt,
			  const struct dvbs2x_complex *in,
			  unsigned int in_len,
			  struct dvbs2x_complex *out,
			  unsigned int *out_len)
{
	unsigned int n, s;
	unsigned int out_idx = 0;

	for (n = 0; n < in_len; n++) {
		for (s = 0; s < flt->sps; s++) {
			double si, sq;

			if (s == 0)
				filter_one(flt, in[n].i, in[n].q,
					   &si, &sq);
			else
				filter_one(flt, 0.0, 0.0, &si, &sq);

			out[out_idx].i = si;
			out[out_idx].q = sq;
			out_idx++;
		}
	}

	*out_len = out_idx;
}

void dvbs2x_rrc_filter_apply(struct dvbs2x_rrc_filter *flt,
			     const struct dvbs2x_complex *in,
			     unsigned int in_len,
			     struct dvbs2x_complex *out,
			     unsigned int *out_len)
{
	unsigned int n;

	for (n = 0; n < in_len; n++)
		filter_one(flt, in[n].i, in[n].q,
			   &out[n].i, &out[n].q);

	*out_len = in_len;
}
