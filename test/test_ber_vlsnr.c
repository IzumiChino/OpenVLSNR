// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR BER Test with Puncturing/Shortening
 *
 * Tests the complete VL-SNR system including:
 * - Shortening: first Xs info bits are zero, not transmitted
 * - Puncturing: Xp parity bits removed before transmission
 * - Spreading: symbol repetition for MODCODs 5, 6
 *
 * The LDPC code operates on the full frame (fec_len bits).
 * Puncturing/shortening is handled at the system level:
 * - Shortened positions: LLR = +large (known zero)
 * - Punctured positions: LLR = 0 (no information)
 * - Transmitted positions: LLR from channel
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "dvbs2x_types.h"
#include "dvbs2x_modcod.h"
#include "bch.h"
#include "ldpc.h"
#include "modulator.h"
#include "interleaver.h"

extern void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
				 double *llr, unsigned int len,
				 double noise_var);
extern void dvbs2x_demod_despread(const struct dvbs2x_complex *in,
				  struct dvbs2x_complex *out,
				  unsigned int out_len);

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

/*
 * Compute puncturing pattern: which parity positions are punctured.
 * Returns array of 0/1 flags (1 = punctured).
 */
static void compute_punct_pattern(const struct dvbs2x_modcod *mc,
				  uint8_t *pattern, unsigned int m)
{
	unsigned int i;
	unsigned int punctured = 0;

	memset(pattern, 0, m);

	if (mc->xp == 0 || mc->p_period == 0)
		return;

	for (i = 0; i < m && punctured < mc->xp; i++) {
		if (i % mc->p_period == 0) {
			pattern[i] = 1;
			punctured++;
		}
	}
}

/*
 * Run BER test for a MODCOD with full puncturing/shortening.
 */
static double test_ber_vlsnr(unsigned int modcod_idx, double esn0_db,
			     unsigned int num_frames, unsigned int max_iter)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_ldpc_encoder ldpc_enc;
	struct dvbs2x_ldpc_decoder ldpc_dec;
	uint8_t *punct_pattern;
	unsigned int m;
	unsigned int total_bits = 0;
	unsigned int error_bits = 0;
	unsigned int frame;
	double noise_var, esn0_lin, sigma;
	unsigned int num_tx_sym;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		return -1.0;

	m = mc->fec_len - mc->k_ldpc;

	/* Compute puncturing pattern */
	punct_pattern = calloc(m, 1);
	compute_punct_pattern(mc, punct_pattern, m);

	/* Es/N0 per transmitted symbol */
	esn0_lin = pow(10.0, esn0_db / 10.0);
	noise_var = 1.0 / (2.0 * esn0_lin);
	sigma = sqrt(noise_var);

	dvbs2x_ldpc_encoder_init(&ldpc_enc, mc);
	dvbs2x_ldpc_decoder_init(&ldpc_dec, mc, max_iter);

	for (frame = 0; frame < num_frames; frame++) {
		uint8_t *info_bits;
		uint8_t *codeword;
		struct dvbs2x_complex *tx_sym;
		struct dvbs2x_complex *spread_sym;
		struct dvbs2x_complex *rx_sym;
		double *llr;
		uint8_t *decoded;
		unsigned int i, iter_used;
		unsigned int num_data_sym;

		info_bits = calloc(mc->k_ldpc, 1);
		codeword = calloc(mc->fec_len, 1);
		llr = calloc(mc->fec_len, sizeof(double));
		decoded = calloc(mc->k_ldpc, 1);

		/* Generate random info (skip shortened positions) */
		for (i = mc->xs; i < mc->k_ldpc; i++)
			info_bits[i] = rand() & 1;
		/* First xs positions stay zero (shortened) */

		/* LDPC encode (full codeword) */
		dvbs2x_ldpc_encode(&ldpc_enc, info_bits, codeword);

		/*
		 * Simulate channel: only transmit non-punctured,
		 * non-shortened bits.
		 *
		 * Transmitted bits:
		 * - Info: positions xs to k_ldpc-1 (skip shortened)
		 * - Parity: positions where punct_pattern[i] == 0
		 */
		num_data_sym = (mc->k_ldpc - mc->xs) + (m - mc->xp);
		if (mc->has_spread)
			num_tx_sym = num_data_sym * 2;
		else
			num_tx_sym = num_data_sym;

		tx_sym = calloc(num_data_sym, sizeof(struct dvbs2x_complex));
		spread_sym = NULL;
		rx_sym = NULL;

		/* Modulate transmitted bits as pi/2-BPSK */
		{
			uint8_t *tx_bits = calloc(num_data_sym, 1);
			unsigned int idx = 0;

			/* Info bits (skip shortened) */
			for (i = mc->xs; i < mc->k_ldpc; i++)
				tx_bits[idx++] = codeword[i];
			/* Parity bits (skip punctured) */
			for (i = 0; i < m; i++) {
				if (!punct_pattern[i])
					tx_bits[idx++] = codeword[mc->k_ldpc + i];
			}

			dvbs2x_mod_pi2bpsk(tx_bits, tx_sym, num_data_sym);
			free(tx_bits);
		}

		/* Apply spreading if needed */
		if (mc->has_spread) {
			spread_sym = calloc(num_tx_sym,
					    sizeof(struct dvbs2x_complex));
			dvbs2x_mod_spread(tx_sym, spread_sym, num_data_sym);
			rx_sym = spread_sym;
		} else {
			rx_sym = tx_sym;
		}

		/* Add AWGN */
		for (i = 0; i < num_tx_sym; i++) {
			rx_sym[i].i += sigma * randn();
			rx_sym[i].q += sigma * randn();
		}

		/* Despread if needed */
		if (mc->has_spread) {
			struct dvbs2x_complex *despread;
			double *tx_llr;
			unsigned int src = 0;

			despread = calloc(num_data_sym,
					  sizeof(struct dvbs2x_complex));
			dvbs2x_demod_despread(rx_sym, despread, num_data_sym);
			tx_llr = calloc(num_data_sym, sizeof(double));
			dvbs2x_demod_pi2bpsk(despread, tx_llr, num_data_sym,
					     noise_var / 2.0);
			free(despread);

			/* Map to full codeword LLR */
			for (i = 0; i < mc->xs; i++)
				llr[i] = 50.0;
			for (i = mc->xs; i < mc->k_ldpc; i++)
				llr[i] = tx_llr[src++];
			for (i = 0; i < m; i++) {
				if (punct_pattern[i])
					llr[mc->k_ldpc + i] = 0.0;
				else
					llr[mc->k_ldpc + i] = tx_llr[src++];
			}
			free(tx_llr);
		} else {
			/* Demodulate directly */
			double *tx_llr = calloc(num_data_sym, sizeof(double));
			unsigned int src = 0;

			dvbs2x_demod_pi2bpsk(rx_sym, tx_llr, num_data_sym,
					     noise_var);

			/* Map to full codeword LLR */
			for (i = 0; i < mc->xs; i++)
				llr[i] = 50.0;
			for (i = mc->xs; i < mc->k_ldpc; i++)
				llr[i] = tx_llr[src++];
			for (i = 0; i < m; i++) {
				if (punct_pattern[i])
					llr[mc->k_ldpc + i] = 0.0;
				else
					llr[mc->k_ldpc + i] = tx_llr[src++];
			}
			free(tx_llr);
		}

		/* LDPC decode */
		dvbs2x_ldpc_decode(&ldpc_dec, llr, decoded, &iter_used);

		/* Count errors (only non-shortened info bits) */
		for (i = mc->xs; i < mc->k_ldpc; i++) {
			if (decoded[i] != info_bits[i])
				error_bits++;
		}
		total_bits += mc->k_ldpc - mc->xs;

		free(info_bits);
		free(codeword);
		free(tx_sym);
		free(spread_sym);
		free(llr);
		free(decoded);
	}

	free(punct_pattern);
	dvbs2x_ldpc_decoder_free(&ldpc_dec);

	if (total_bits == 0)
		return -1.0;
	return (double)error_bits / (double)total_bits;
}

int main(int argc, char *argv[])
{
	unsigned int modcod_idx = 5;
	unsigned int num_frames = 30;
	unsigned int max_iter = 200;
	double esn0_db;
	double ber;

	if (argc >= 2)
		modcod_idx = atoi(argv[1]);
	if (argc >= 3)
		num_frames = atoi(argv[2]);
	if (argc >= 4)
		max_iter = atoi(argv[3]);

	srand(time(NULL));

	{
		const struct dvbs2x_modcod *mc;

		mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
		printf("DVB-S2X VL-SNR BER Test (with puncturing)\n");
		printf("=========================================\n");
		printf("MODCOD %u: %s %u/%u, fec=%u, xs=%u, xp=%u, P=%u\n",
		       modcod_idx,
		       mc->modulation == DVBS2X_MOD_BPSK ? "BPSK" : "QPSK",
		       mc->code_rate_num, mc->code_rate_den,
		       mc->fec_len, mc->xs, mc->xp, mc->p_period);
		printf("Spread: %s, Frames: %u, MaxIter: %u\n\n",
		       mc->has_spread ? "yes (x2)" : "no",
		       num_frames, max_iter);
	}

	printf("Es/N0 (dB)    BER\n");
	printf("----------    --------\n");

	for (esn0_db = -4.0; esn0_db >= -12.0; esn0_db -= 1.0) {
		ber = test_ber_vlsnr(modcod_idx, esn0_db,
				     num_frames, max_iter);
		printf("  %+5.1f       %.2e", esn0_db, ber);
		if (ber == 0.0)
			printf("  (error-free)");
		printf("\n");
		fflush(stdout);

		if (ber == 0.0)
			break;
	}

	printf("\nDone.\n");
	return 0;
}
