// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Symbol Timing Recovery
 *
 * Feed-forward symbol-timing estimation using the Oerder & Meyr
 * "square-and-DFT" algorithm.  The squared magnitude of an
 * oversampled, pulse-shaped signal contains a spectral line at the
 * symbol rate whose phase gives the optimum sampling instant:
 *
 *   X = sum_n |x[n]|^2 * exp(-j 2 pi n / sps)
 *   tau = -arg(X) / (2 pi)            (fraction of a symbol)
 *
 * The estimate is non-data-aided and, being averaged over the whole
 * burst, is robust well below 0 dB Es/N0.  Because it is feed-forward
 * there is no loop transient, so the (constant-envelope, repeated)
 * spread VL-SNR waveforms are resampled cleanly - essential for the
 * symbol pairing used by despreading.
 *
 * The signal is then resampled at instants (tau + m) * sps using a
 * 4-point cubic interpolator.
 *
 * Reference: M. Oerder and H. Meyr, "Digital filter and square timing
 *            recovery," IEEE Trans. Comm., 1988.
 */

#include "sync.h"
#include <math.h>
#include <string.h>
#ifdef DEBUG
#include <stdio.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TIMING_BLOCK_SYMBOLS	4096
#define TIMING_MAX_DRIFT		0.002

void dvbs2x_timing_sync_init(struct dvbs2x_timing_sync *ts,
			     unsigned int sps, double loop_bw)
{
	ts->sps = sps;
	ts->loop_bw = loop_bw;
	ts->mu = 0.0;
	ts->integrator = 0.0;
	ts->kp = 0.0;
	ts->ki = 0.0;
}

/* 4-point cubic (Catmull-Rom) interpolation at fractional index pos. */
static struct dvbs2x_complex cubic_interp(const struct dvbs2x_complex *x,
					  unsigned int len, double pos)
{
	struct dvbs2x_complex out = {0.0, 0.0};
	long i = (long)floor(pos);
	double f = pos - (double)i;
	double f2 = f * f;
	double f3 = f2 * f;
	double c0, c1, c2, c3;
	long im1 = i - 1, ip1 = i + 1, ip2 = i + 2;

	if (im1 < 0)
		im1 = 0;
	if (i < 0)
		i = 0;
	if (ip1 >= (long)len)
		ip1 = len - 1;
	if (ip2 >= (long)len)
		ip2 = len - 1;

	c0 = -0.5 * f3 + f2 - 0.5 * f;
	c1 = 1.5 * f3 - 2.5 * f2 + 1.0;
	c2 = -1.5 * f3 + 2.0 * f2 + 0.5 * f;
	c3 = 0.5 * f3 - 0.5 * f2;

	out.i = c0 * x[im1].i + c1 * x[i].i + c2 * x[ip1].i + c3 * x[ip2].i;
	out.q = c0 * x[im1].q + c1 * x[i].q + c2 * x[ip1].q + c3 * x[ip2].q;
	return out;
}

void dvbs2x_timing_sync_process(struct dvbs2x_timing_sync *ts,
				const struct dvbs2x_complex *in,
				unsigned int in_len,
				struct dvbs2x_complex *out,
				unsigned int *out_len)
{
	unsigned int sps = ts->sps;
	double cr = 0.0, ci = 0.0;
	double off, drift = 0.0;
	unsigned int n, m;
	double cos_tbl[16], sin_tbl[16];

	if (sps < 2 || in_len < sps) {
		memcpy(out, in, in_len * sizeof(*in));
		*out_len = in_len;
		return;
	}

	/* Precompute cos/sin for the sps phase offsets */
	for (n = 0; n < sps && n < 16; n++) {
		double ang = -2.0 * M_PI * (double)n / (double)sps;

		cos_tbl[n] = cos(ang);
		sin_tbl[n] = sin(ang);
	}

	/* Oerder-Meyr spectral-line timing estimate over the complete burst. */
	for (n = 0; n < in_len; n++) {
		double e = in[n].i * in[n].i + in[n].q * in[n].q;
		unsigned int phase = n % sps;

		cr += e * cos_tbl[phase];
		ci += e * sin_tbl[phase];
	}

	/* Optimum sampling instant within a symbol, in samples [0, sps). */
	off = -atan2(ci, cr) * (double)sps / (2.0 * M_PI);
	while (off < 1.0)
		off += (double)sps;

	/*
	 * Independent SDR clocks turn a fixed timing phase into a slow ramp.
	 * Estimate that ramp from block Oerder-Meyr phases and resample with a
	 * correspondingly adjusted period.  Weighted linear regression rejects
	 * low-energy tail blocks; implausible slopes retain the burst estimate.
	 */
	{
		unsigned int block_len = TIMING_BLOCK_SYMBOLS * sps;
		double sw = 0.0, swx = 0.0, swy = 0.0;
		double swxx = 0.0, swxy = 0.0;
		double previous = 0.0;
		unsigned int blocks = 0;
		unsigned int start;

		for (start = 0; start + block_len <= in_len;
		     start += block_len) {
			double br = 0.0, bi = 0.0, energy = 0.0;
			double phase, weight, centre;
			unsigned int end = start + block_len;

			for (n = start; n < end; n++) {
				double e = in[n].i * in[n].i +
					   in[n].q * in[n].q;
				unsigned int phase_idx = n % sps;

				br += e * cos_tbl[phase_idx];
				bi += e * sin_tbl[phase_idx];
				energy += e;
			}
			if (energy <= 0.0)
				continue;
			phase = -atan2(bi, br) * (double)sps /
				(2.0 * M_PI);
			if (blocks) {
				while (phase - previous > (double)sps / 2.0)
					phase -= (double)sps;
				while (phase - previous < -(double)sps / 2.0)
					phase += (double)sps;
			}
			previous = phase;
			weight = sqrt(br * br + bi * bi) / energy;
			centre = (double)start + (double)(block_len - 1) / 2.0;
			sw += weight;
			swx += weight * centre;
			swy += weight * phase;
			swxx += weight * centre * centre;
			swxy += weight * centre * phase;
			blocks++;
		}
		if (blocks >= 3) {
			double denominator = sw * swxx - swx * swx;

			if (fabs(denominator) > 1e-12) {
				double candidate;

				candidate = (sw * swxy - swx * swy) /
					denominator;
				if (fabs(candidate) <= TIMING_MAX_DRIFT) {
					drift = candidate;
					off = (swy - drift * swx) / sw;
					while (off < 1.0)
						off += (double)sps;
					while (off >= 1.0 + (double)sps)
						off -= (double)sps;
				}
			}
		}
	}
#ifdef DEBUG
	fprintf(stderr, "[dbg] timing off=%.6f drift=%.9f\n", off, drift);
#endif

	/*
	 * Track the residual clock error with a normalized Gardner detector.
	 * The feed-forward slope supplies coarse acquisition; the loop removes
	 * estimator bias and follows clock wander across a long frame.
	 */
	{
		double pos = off / (1.0 - drift);
		double omega = (double)sps / (1.0 - drift);
		double integrator = 0.0;
		struct dvbs2x_complex previous;

		m = 0;
		if (pos + 2.0 < (double)in_len) {
			previous = cubic_interp(in, in_len, pos);
			out[m++] = previous;
		}
		for (;;) {
			struct dvbs2x_complex middle, current;
			double next_pos = pos + omega;
			double error, power;

			if (next_pos + 2.0 >= (double)in_len)
				break;
			middle = cubic_interp(in, in_len, pos + omega / 2.0);
			current = cubic_interp(in, in_len, next_pos);
			power = previous.i * previous.i +
				previous.q * previous.q +
				current.i * current.i + current.q * current.q;
			error = ((previous.i - current.i) * middle.i +
				 (previous.q - current.q) * middle.q) /
				(power + 1e-12);
			integrator += 1e-7 * error;
			if (integrator > TIMING_MAX_DRIFT * (double)sps)
				integrator = TIMING_MAX_DRIFT * (double)sps;
			else if (integrator <
				 -TIMING_MAX_DRIFT * (double)sps)
				integrator = -TIMING_MAX_DRIFT * (double)sps;
			pos = next_pos + 0.005 * error;
			omega = (double)sps / (1.0 - drift) + integrator;
			current = cubic_interp(in, in_len, pos);
			out[m++] = current;
			previous = current;
		}
	}
	*out_len = m;
}
