// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Public-API loopback test with optional RF impairments.
 *
 *   mode 0: symbol-level path
 *       dvbs2x_modulate_symbols -> [freq/phase + AWGN] ->
 *       dvbs2x_demodulate_symbols
 *   mode 1: full RF path
 *       dvbs2x_modulate -> [timing/freq/phase + AWGN] ->
 *       dvbs2x_demodulate
 *
 * Usage:
 *   test_api_loopback [modcod] [esn0_db] [mode] [seed] \
 *                     [freq_cyc_sym] [phase_rad] [timing_frac]
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

static double randn(void)
{
	double u1, u2;

	do {
		u1 = (double)rand() / RAND_MAX;
	} while (u1 < 1e-10);
	u2 = (double)rand() / RAND_MAX;
	return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* Fractional-delay resample (cubic) by tau samples, in place via copy. */
static void frac_delay(struct dvbs2x_complex *x, unsigned int len, double tau)
{
	struct dvbs2x_complex *tmp;
	unsigned int n;

	if (tau == 0.0)
		return;
	tmp = malloc(len * sizeof(*tmp));
	if (!tmp)
		return;
	memcpy(tmp, x, len * sizeof(*tmp));

	for (n = 0; n < len; n++) {
		double pos = (double)n - tau;
		long i = (long)floor(pos);
		double f = pos - (double)i;
		double f2 = f * f, f3 = f2 * f;
		double c0 = -0.5 * f3 + f2 - 0.5 * f;
		double c1 = 1.5 * f3 - 2.5 * f2 + 1.0;
		double c2 = -1.5 * f3 + 2.0 * f2 + 0.5 * f;
		double c3 = 0.5 * f3 - 0.5 * f2;
		long im1 = i - 1, i0 = i, ip1 = i + 1, ip2 = i + 2;

		if (im1 < 0)
			im1 = 0;
		if (i0 < 0)
			i0 = 0;
		if (ip1 >= (long)len)
			ip1 = len - 1;
		if (ip2 >= (long)len)
			ip2 = len - 1;
		x[n].i = c0 * tmp[im1].i + c1 * tmp[i0].i +
			 c2 * tmp[ip1].i + c3 * tmp[ip2].i;
		x[n].q = c0 * tmp[im1].q + c1 * tmp[i0].q +
			 c2 * tmp[ip1].q + c3 * tmp[ip2].q;
	}
	free(tmp);
}

int main(int argc, char *argv[])
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_demodulator demod;
	unsigned int modcod_idx = (argc >= 2) ? (unsigned int)atoi(argv[1]) : 9;
	double esn0_db = (argc >= 3) ? atof(argv[2]) : 99.0;
	int mode = (argc >= 4) ? atoi(argv[3]) : 0;
	unsigned int seed = (argc >= 5) ? (unsigned)atoi(argv[4]) :
			    (unsigned)time(NULL);
	double f_off = (argc >= 6) ? atof(argv[5]) : 0.0;	/* cyc/sym */
	double ph_off = (argc >= 7) ? atof(argv[6]) : 0.0;	/* rad */
	double t_off = (argc >= 8) ? atof(argv[7]) : 0.0;	/* samples */
	const struct dvbs2x_modcod *mc;
	uint8_t *user_in, *user_out;
	struct dvbs2x_complex *iq;
	unsigned int user_bits, out_len = 0, rec_len = 0;
	unsigned int i, errs = 0, cap;
	unsigned int sps = 2;
	double esn0_lin, noise_var, sigma, fs;
	int ret, ok;

	srand(seed);

	if (dvbs2x_modulator_init(&mod, modcod_idx, 0.35, sps, 0) < 0 ||
	    dvbs2x_demodulator_init(&demod, 0.35, sps, 0) < 0) {
		printf("init failed\n");
		return 1;
	}

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	user_bits = mc->k_bch - 80;
	cap = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
	      mc->fec_len * 2 + mc->fec_len / 16 + 256;
	cap *= 4;

	user_in = malloc(user_bits);
	user_out = malloc(mc->k_bch);
	iq = malloc((size_t)cap * sizeof(struct dvbs2x_complex));
	if (!user_in || !user_out || !iq)
		return 1;

	for (i = 0; i < user_bits; i++)
		user_in[i] = (i * 5 + 1) & 1;

	esn0_lin = pow(10.0, esn0_db / 10.0);
	noise_var = 1.0 / (2.0 * esn0_lin);
	sigma = sqrt(noise_var);

	if (mode == 0) {
		ret = dvbs2x_modulate_symbols(&mod, user_in, user_bits,
					      iq, &out_len);
		fs = 1.0;		/* freq is per symbol */
	} else {
		ret = dvbs2x_modulate(&mod, user_in, user_bits, iq, &out_len);
		for (i = 0; i < 256 && out_len + i < cap; i++) {
			iq[out_len + i].i = 0.0;
			iq[out_len + i].q = 0.0;
		}
		out_len += 256;
		fs = (double)sps;	/* freq per sample = f_off/sps */
	}
	if (ret < 0) {
		printf("modulate failed\n");
		return 1;
	}

	/* Timing offset (RF only) */
	if (mode == 1 && t_off != 0.0)
		frac_delay(iq, out_len, t_off);

	/* Carrier frequency + phase offset */
	if (f_off != 0.0 || ph_off != 0.0) {
		for (i = 0; i < out_len; i++) {
			double a = 2.0 * M_PI * (f_off / fs) * (double)i + ph_off;
			double c = cos(a), s = sin(a);
			double ti = iq[i].i * c - iq[i].q * s;
			double tq = iq[i].i * s + iq[i].q * c;

			iq[i].i = ti;
			iq[i].q = tq;
		}
	}

	/* AWGN */
	if (esn0_db < 50.0)
		for (i = 0; i < out_len; i++) {
			iq[i].i += sigma * randn();
			iq[i].q += sigma * randn();
		}

	if (mode == 0)
		ret = dvbs2x_demodulate_symbols(&demod, iq, out_len,
						noise_var, user_out, &rec_len);
	else
		ret = dvbs2x_demodulate(&demod, iq, out_len,
					user_out, &rec_len);

	for (i = 0; i < user_bits && i < rec_len; i++)
		if (user_out[i] != user_in[i])
			errs++;

	ok = (ret == 0 && errs == 0 && rec_len >= user_bits);
	printf("MODCOD %u Es/N0=%.1f mode=%s f=%.1e ph=%.2f t=%.2f : %s "
	       "(errs=%u/%u)\n",
	       modcod_idx, esn0_db, mode ? "RF" : "sym", f_off, ph_off, t_off,
	       ok ? "PASS" : "FAIL", ret == 0 ? errs : user_bits, user_bits);

	free(user_in);
	free(user_out);
	free(iq);
	return ok ? 0 : 1;
}
