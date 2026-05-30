// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR end-to-end RF BER test.
 *
 * Drives the full public pipeline:
 *
 *   dvbs2x_modulate() -> [timing + carrier + phase offset + AWGN] ->
 *   dvbs2x_demodulate()
 *
 * i.e. BB frame, BCH, LDPC, interleave, pi/2-BPSK/QPSK, spreading,
 * PL/VL-SNR headers, pilots, scrambling and RRC pulse shaping on the
 * transmit side, and RRC matched filtering, Oerder-Meyr symbol timing,
 * Walsh-Hadamard frame sync, data-aided carrier recovery, pilot phase
 * tracking, descrambling, soft demap, LDPC (sum-product) and BCH on
 * the receive side.
 *
 * Reports bit-error rate and frame-error rate versus Es/N0 so the
 * operating point of each MODCOD can be read off directly.
 *
 * Usage: test_rf_ber [modcod] [frames_per_point]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "dvbs2x_vlsnr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Small, fixed residual RF impairments (as after coarse acquisition) */
#define RF_FREQ_OFF	1.0e-4		/* cycles / symbol */
#define RF_PHASE_OFF	0.7		/* radians */
#define RF_TIMING_OFF	0.3		/* samples */
#define SPS		2

static double randn(void)
{
	double u1, u2;

	do {
		u1 = (double)rand() / RAND_MAX;
	} while (u1 < 1e-10);
	u2 = (double)rand() / RAND_MAX;
	return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void frac_delay(struct dvbs2x_complex *x, unsigned int len, double tau)
{
	struct dvbs2x_complex *t;
	unsigned int n;

	if (tau == 0.0)
		return;
	t = malloc(len * sizeof(*t));
	if (!t)
		return;
	memcpy(t, x, len * sizeof(*t));
	for (n = 0; n < len; n++) {
		double pos = (double)n - tau;
		long i = (long)floor(pos);
		double f = pos - (double)i, f2 = f * f, f3 = f2 * f;
		double c0 = -0.5 * f3 + f2 - 0.5 * f;
		double c1 = 1.5 * f3 - 2.5 * f2 + 1.0;
		double c2 = -1.5 * f3 + 2.0 * f2 + 0.5 * f;
		double c3 = 0.5 * f3 - 0.5 * f2;
		long a = i - 1, b = i, c = i + 1, d = i + 2;

		if (a < 0)
			a = 0;
		if (b < 0)
			b = 0;
		if (c >= (long)len)
			c = len - 1;
		if (d >= (long)len)
			d = len - 1;
		x[n].i = c0 * t[a].i + c1 * t[b].i + c2 * t[c].i + c3 * t[d].i;
		x[n].q = c0 * t[a].q + c1 * t[b].q + c2 * t[c].q + c3 * t[d].q;
	}
	free(t);
}

static void run_point(const struct dvbs2x_modcod *mc, unsigned int idx,
		      double esn0_db, unsigned int frames,
		      double *ber, double *fer)
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_demodulator demod;
	uint8_t *uin, *uout;
	struct dvbs2x_complex *iq;
	unsigned int user_bits = mc->k_bch - 80;
	unsigned int cap, out_len, rec_len, i, fr;
	unsigned long errs = 0, total = 0, frame_err = 0;
	double sigma = sqrt(1.0 / (2.0 * pow(10.0, esn0_db / 10.0)));

	cap = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
	      mc->fec_len * 2 + mc->fec_len / 16 + 512;
	cap *= SPS + 1;
	uin = malloc(user_bits);
	uout = malloc(mc->k_bch);
	iq = malloc((size_t)cap * sizeof(struct dvbs2x_complex));

	dvbs2x_modulator_init(&mod, idx, 0.35, SPS, 0);
	dvbs2x_demodulator_init(&demod, 0.35, SPS, 0);

	for (fr = 0; fr < frames; fr++) {
		unsigned int bad = 0;

		for (i = 0; i < user_bits; i++)
			uin[i] = rand() & 1;

		if (dvbs2x_modulate(&mod, uin, user_bits, iq, &out_len) < 0)
			continue;
		for (i = 0; i < 256 && out_len + i < cap; i++) {
			iq[out_len + i].i = 0.0;
			iq[out_len + i].q = 0.0;
		}
		out_len += 256;

		frac_delay(iq, out_len, RF_TIMING_OFF);
		for (i = 0; i < out_len; i++) {
			double a = 2.0 * M_PI * (RF_FREQ_OFF / SPS) * i +
				   RF_PHASE_OFF;
			double c = cos(a), s = sin(a);
			double ti = iq[i].i * c - iq[i].q * s;
			double tq = iq[i].i * s + iq[i].q * c;

			iq[i].i = ti + sigma * randn();
			iq[i].q = tq + sigma * randn();
		}

		rec_len = 0;
		if (dvbs2x_demodulate(&demod, iq, out_len, uout,
				      &rec_len) < 0) {
			bad = 1;
			errs += user_bits;	/* count a lost frame fully */
			total += user_bits;
		} else {
			for (i = 0; i < user_bits; i++) {
				if (i >= rec_len || uout[i] != uin[i]) {
					errs++;
					bad = 1;
				}
				total++;
			}
		}
		if (bad)
			frame_err++;
	}

	*ber = total ? (double)errs / (double)total : 1.0;
	*fer = frames ? (double)frame_err / (double)frames : 1.0;
	free(uin);
	free(uout);
	free(iq);
}

int main(int argc, char *argv[])
{
	unsigned int idx = (argc >= 2) ? (unsigned int)atoi(argv[1]) : 0;
	unsigned int frames = (argc >= 3) ? (unsigned int)atoi(argv[2]) : 8;
	unsigned int lo, hi, m;
	double esn0;

	srand((unsigned int)time(NULL));

	printf("DVB-S2X VL-SNR end-to-end RF BER\n");
	printf("================================\n");
	printf("Full chain: mod -> RRC -> [timing %.2f, freq %.0e, "
	       "phase %.1f, AWGN] -> demod\n", RF_TIMING_OFF, RF_FREQ_OFF,
	       RF_PHASE_OFF);
	printf("Frames/point: %u\n", frames);

	if (idx >= 1 && idx <= DVBS2X_VLSNR_NUM_MODCODS) {
		lo = idx;
		hi = idx;
	} else {
		lo = 1;
		hi = DVBS2X_VLSNR_NUM_MODCODS;
	}

	for (m = lo; m <= hi; m++) {
		const struct dvbs2x_modcod *mc = dvbs2x_vlsnr_get_modcod(m);

		printf("\nMODCOD %u: %s %u/%u, FEC %u%s\n", m,
		       mc->modulation == DVBS2X_MOD_QPSK ? "QPSK" : "BPSK",
		       mc->code_rate_num, mc->code_rate_den, mc->fec_len,
		       mc->has_spread ? " (spread x2)" : "");
		printf("  Es/N0    BER        FER\n");
		printf("  -----    --------   -----\n");
		for (esn0 = 2.0; esn0 >= -11.0; esn0 -= 1.0) {
			double ber, fer;

			run_point(mc, m, esn0, frames, &ber, &fer);
			printf("  %+5.1f    %.2e   %.2f\n", esn0, ber, fer);
			fflush(stdout);
			if (ber > 0.2)
				break;	/* well past the waterfall */
		}
	}

	printf("\nDone.\n");
	return 0;
}
