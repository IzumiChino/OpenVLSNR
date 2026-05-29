// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR BER Test with AWGN Channel
 *
 * Tests the complete encode -> AWGN channel -> decode chain
 * at various Es/N0 points to verify FEC performance.
 *
 * The AWGN channel adds complex Gaussian noise with variance
 * sigma^2 = 1 / (2 * 10^(EsN0_dB/10)) per dimension.
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

/* Declared in demodulate.c */
extern void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
				 double *llr, unsigned int len,
				 double noise_var);
extern void dvbs2x_demod_qpsk(const struct dvbs2x_complex *symbols,
			      double *llr, unsigned int num_symbols,
			      double noise_var);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Box-Muller transform for Gaussian random numbers.
 */
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
 * Add AWGN to complex symbols.
 * noise_var = sigma^2 per dimension = N0/2 = 1/(2*Es/N0_linear)
 */
static void add_awgn(struct dvbs2x_complex *symbols, unsigned int len,
		     double noise_var)
{
	unsigned int i;
	double sigma = sqrt(noise_var);

	for (i = 0; i < len; i++) {
		symbols[i].i += sigma * randn();
		symbols[i].q += sigma * randn();
	}
}

/*
 * Run BER test for a single MODCOD at a given Es/N0.
 * Returns bit error rate, or -1 on failure.
 */
static double test_ber(unsigned int modcod_idx, double esn0_db,
		       unsigned int num_frames)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_bch_encoder bch_enc;
	struct dvbs2x_ldpc_encoder ldpc_enc;
	struct dvbs2x_ldpc_decoder ldpc_dec;
	struct dvbs2x_bch_decoder bch_dec;
	uint8_t *user_bits = NULL;
	uint8_t *bch_out = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *interleaved = NULL;
	struct dvbs2x_complex *symbols = NULL;
	double *llr = NULL;
	double *deinterleaved = NULL;
	uint8_t *ldpc_decoded = NULL;
	uint8_t *bch_decoded = NULL;
	unsigned int num_symbols;
	unsigned int frame;
	unsigned int total_bits = 0;
	unsigned int error_bits = 0;
	double noise_var;
	double esn0_lin;
	unsigned int i;
	int ret;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		return -1.0;

	/* Compute noise variance from Es/N0 */
	esn0_lin = pow(10.0, esn0_db / 10.0);
	noise_var = 1.0 / (2.0 * esn0_lin);

	/* Initialize FEC */
	if (dvbs2x_bch_encoder_init(&bch_enc, mc) < 0)
		return -1.0;
	if (dvbs2x_ldpc_encoder_init(&ldpc_enc, mc) < 0)
		return -1.0;
	if (dvbs2x_ldpc_decoder_init(&ldpc_dec, mc, 50) < 0)
		return -1.0;
	if (dvbs2x_bch_decoder_init(&bch_dec, mc) < 0)
		return -1.0;

	/* Allocate buffers */
	user_bits = malloc(mc->k_bch);
	bch_out = malloc(mc->n_bch);
	ldpc_out = malloc(mc->fec_len);
	interleaved = malloc(mc->fec_len);

	if (mc->modulation == DVBS2X_MOD_QPSK)
		num_symbols = mc->fec_len / 2;
	else
		num_symbols = mc->fec_len;

	symbols = malloc(num_symbols * sizeof(struct dvbs2x_complex));
	llr = malloc(mc->fec_len * sizeof(double));
	deinterleaved = malloc(mc->fec_len * sizeof(double));
	ldpc_decoded = malloc(mc->k_ldpc);
	bch_decoded = malloc(mc->n_bch);

	if (!user_bits || !bch_out || !ldpc_out || !interleaved ||
	    !symbols || !llr || !deinterleaved || !ldpc_decoded ||
	    !bch_decoded) {
		error_bits = 0;
		goto out;
	}

	for (frame = 0; frame < num_frames; frame++) {
		unsigned int iter_used;

		/* Generate random data */
		for (i = 0; i < mc->k_bch; i++)
			user_bits[i] = rand() & 1;

		/* Encode */
		dvbs2x_bch_encode(&bch_enc, user_bits, bch_out);
		dvbs2x_ldpc_encode(&ldpc_enc, bch_out, ldpc_out);
		dvbs2x_interleave(mc, ldpc_out, interleaved);

		/* Modulate */
		if (mc->modulation == DVBS2X_MOD_QPSK)
			dvbs2x_mod_qpsk(interleaved, symbols, num_symbols);
		else
			dvbs2x_mod_pi2bpsk(interleaved, symbols, num_symbols);

		/* Add AWGN */
		add_awgn(symbols, num_symbols, noise_var);

		/* Demodulate */
		if (mc->modulation == DVBS2X_MOD_QPSK)
			dvbs2x_demod_qpsk(symbols, llr, num_symbols,
					  noise_var);
		else
			dvbs2x_demod_pi2bpsk(symbols, llr, num_symbols,
					     noise_var);

		/* Deinterleave */
		dvbs2x_deinterleave(mc, llr, deinterleaved);

		/* LDPC decode */
		ret = dvbs2x_ldpc_decode(&ldpc_dec, deinterleaved,
					 ldpc_decoded, &iter_used);

		/* BCH decode */
		memcpy(bch_decoded, ldpc_decoded, mc->n_bch);
		dvbs2x_bch_decode(&bch_dec, bch_decoded);

		/* Count bit errors */
		for (i = 0; i < mc->k_bch; i++) {
			if (bch_decoded[i] != user_bits[i])
				error_bits++;
		}
		total_bits += mc->k_bch;
	}

out:
	free(user_bits);
	free(bch_out);
	free(ldpc_out);
	free(interleaved);
	free(symbols);
	free(llr);
	free(deinterleaved);
	free(ldpc_decoded);
	free(bch_decoded);

	if (total_bits == 0)
		return -1.0;
	return (double)error_bits / (double)total_bits;
}

int main(int argc, char *argv[])
{
	unsigned int modcod_idx;
	double esn0_db;
	unsigned int num_frames = 10;
	double ber;

	/* Default: test MODCOD 9 at various Es/N0 */
	if (argc >= 2)
		modcod_idx = atoi(argv[1]);
	else
		modcod_idx = 9;

	if (argc >= 3)
		num_frames = atoi(argv[2]);

	srand(time(NULL));

	printf("DVB-S2X VL-SNR BER Test\n");
	printf("=======================\n");
	printf("MODCOD: %u, Frames per point: %u\n\n", modcod_idx, num_frames);
	printf("Es/N0 (dB)    BER\n");
	printf("----------    --------\n");

	/* Test at several Es/N0 points */
	for (esn0_db = 4.0; esn0_db >= -4.0; esn0_db -= 1.0) {
		ber = test_ber(modcod_idx, esn0_db, num_frames);
		printf("  %+5.1f       %.2e", esn0_db, ber);
		if (ber == 0.0)
			printf("  (error-free)");
		printf("\n");
		fflush(stdout);

		/* Stop if error-free for 2 consecutive points */
		if (ber == 0.0 && esn0_db < 3.0)
			break;
	}

	printf("\nDone.\n");
	return 0;
}
