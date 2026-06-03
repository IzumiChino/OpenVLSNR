// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Continuous-Mode Streaming Test
 *
 * Generates a continuous stream of multiple PL frames with AWGN and
 * RF impairments, then feeds them to dvbs2x_demodulate_stream() to
 * verify multi-frame lock acquisition and sustained decoding.
 *
 * Usage: test_stream [modcod] [esn0_db] [num_frames] [freq_off]
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

int main(int argc, char *argv[])
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_demodulator demod;
	unsigned int modcod_idx = 9;
	double esn0_db = 99.0;
	unsigned int num_frames = 5;
	double freq_off = 1e-4;
	const struct dvbs2x_modcod *mc;
	unsigned int sps = 2;
	unsigned int frame_samples, stream_len;
	unsigned int user_bits, i, fr;
	struct dvbs2x_complex *stream;
	uint8_t *tx_data, *rx_data;
	unsigned int out_len, offset;
	unsigned int decoded = 0, failed = 0;
	double sigma, noise_var;

	if (argc >= 2)
		modcod_idx = (unsigned int)atoi(argv[1]);
	if (argc >= 3)
		esn0_db = atof(argv[2]);
	if (argc >= 4)
		num_frames = (unsigned int)atoi(argv[3]);
	if (argc >= 5)
		freq_off = atof(argv[4]);

	if (dvbs2x_modulator_init(&mod, modcod_idx, 0.35, sps, 0) < 0 ||
	    dvbs2x_demodulator_init(&demod, 0.35, sps, 0) < 0) {
		printf("init failed\n");
		return 1;
	}

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	user_bits = mc->k_bch - 80;

	/* Allocate stream buffer for all frames */
	frame_samples = 80000;
	stream_len = num_frames * frame_samples + 512;
	stream = calloc(stream_len, sizeof(*stream));
	tx_data = calloc(user_bits, 1);
	rx_data = calloc(mc->k_bch, 1);
	if (!stream || !tx_data || !rx_data)
		return 1;

	/* Generate test data */
	srand((unsigned int)time(NULL));
	for (i = 0; i < user_bits; i++)
		tx_data[i] = rand() & 1;

	/* Modulate num_frames back-to-back */
	offset = 0;
	for (fr = 0; fr < num_frames; fr++) {
		out_len = 0;
		dvbs2x_modulate(&mod, tx_data, user_bits,
				stream + offset, &out_len);
		offset += out_len;
	}
	stream_len = offset + 256;

	/* Apply frequency offset to entire stream */
	if (freq_off != 0.0) {
		for (i = 0; i < stream_len; i++) {
			double a = 2.0 * M_PI * freq_off / (double)sps *
				   (double)i;
			double c = cos(a), s = sin(a);
			double ti = stream[i].i * c - stream[i].q * s;
			double tq = stream[i].i * s + stream[i].q * c;

			stream[i].i = ti;
			stream[i].q = tq;
		}
	}

	/* Add AWGN */
	if (esn0_db < 50.0) {
		noise_var = 1.0 / (2.0 * pow(10.0, esn0_db / 10.0));
		sigma = sqrt(noise_var);
		for (i = 0; i < stream_len; i++) {
			stream[i].i += sigma * randn();
			stream[i].q += sigma * randn();
		}
	}

	/* Feed to streaming demodulator */
	printf("DVB-S2X VL-SNR Streaming Test\n");
	printf("=============================\n");
	printf("MODCOD %u, Es/N0=%.1f dB, frames=%u, "
	       "freq=%.1e cyc/sym\n\n",
	       modcod_idx, esn0_db, num_frames, freq_off);

	offset = 0;
	while (offset < stream_len) {
		unsigned int avail = stream_len - offset;
		unsigned int cons = 0;
		unsigned int rx_len = 0;
		int ret;

		ret = dvbs2x_demodulate_stream(&demod, stream + offset,
					       avail, rx_data, &rx_len,
					       &cons);
		if (cons == 0)
			break;

		if (ret == 0) {
			unsigned int errs = 0;

			for (i = 0; i < user_bits && i < rx_len; i++)
				if (rx_data[i] != tx_data[i])
					errs++;
			printf("  frame %u: DECODED (errs=%u/%u) "
			       "state=%d\n",
			       decoded + failed + 1, errs, user_bits,
			       demod.state);
			decoded++;
		} else {
			printf("  frame %u: FAILED state=%d\n",
			       decoded + failed + 1, demod.state);
			failed++;
		}
		offset += cons;
	}

	printf("\nResult: %u decoded, %u failed out of %u frames\n",
	       decoded, failed, num_frames);
	printf("Final state: %s\n",
	       demod.state == DVBS2X_DEMOD_TRACK ? "TRACK" :
	       demod.state == DVBS2X_DEMOD_ACQUIRE ? "ACQUIRE" :
	       "SEARCH");

	free(stream);
	free(tx_data);
	free(rx_data);
	dvbs2x_demodulator_destroy(&demod);
	dvbs2x_modulator_destroy(&mod);
	return (decoded >= num_frames - 1) ? 0 : 1;
}
