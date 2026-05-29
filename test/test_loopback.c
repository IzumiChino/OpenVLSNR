// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Loopback Test
 *
 * Tests the complete encode -> decode chain without noise.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "dvbs2x_types.h"
#include "dvbs2x_modcod.h"
#include "bch.h"
#include "ldpc.h"
#include "modulator.h"
#include "interleaver.h"

extern void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
				 double *llr, unsigned int len,
				 double noise_var);
extern void dvbs2x_demod_qpsk(const struct dvbs2x_complex *symbols,
			      double *llr, unsigned int num_symbols,
			      double noise_var);

static int test_loopback_modcod(unsigned int modcod_idx)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_bch_encoder bch_enc;
	struct dvbs2x_bch_decoder bch_dec;
	struct dvbs2x_ldpc_encoder ldpc_enc;
	struct dvbs2x_ldpc_decoder ldpc_dec;
	uint8_t *user_bits, *bch_out, *ldpc_out, *interleaved;
	uint8_t *ldpc_decoded, *bch_decoded;
	struct dvbs2x_complex *symbols;
	double *llr, *deinterleaved;
	unsigned int num_symbols, i, iter_used;
	int ret;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		return -1;

	printf("  MODCOD %u (%s %u/%u, %u bits)...\n",
	       modcod_idx,
	       mc->modulation == DVBS2X_MOD_BPSK ? "BPSK" : "QPSK",
	       mc->code_rate_num, mc->code_rate_den, mc->fec_len);

	user_bits = calloc(mc->k_bch, 1);
	bch_out = calloc(mc->n_bch, 1);
	ldpc_out = calloc(mc->fec_len, 1);
	interleaved = calloc(mc->fec_len, 1);
	num_symbols = (mc->modulation == DVBS2X_MOD_QPSK) ?
		      mc->fec_len / 2 : mc->fec_len;
	symbols = calloc(num_symbols, sizeof(struct dvbs2x_complex));
	llr = calloc(mc->fec_len, sizeof(double));
	deinterleaved = calloc(mc->fec_len, sizeof(double));
	ldpc_decoded = calloc(mc->k_ldpc, 1);
	bch_decoded = calloc(mc->n_bch, 1);

	/* Generate test data */
	for (i = 0; i < mc->k_bch; i++)
		user_bits[i] = (i * 7 + 3) & 1;

	/* Encode */
	dvbs2x_bch_encoder_init(&bch_enc, mc);
	dvbs2x_bch_encode(&bch_enc, user_bits, bch_out);
	dvbs2x_ldpc_encoder_init(&ldpc_enc, mc);
	dvbs2x_ldpc_encode(&ldpc_enc, bch_out, ldpc_out);
	dvbs2x_interleave(mc, ldpc_out, interleaved);

	/* Modulate */
	if (mc->modulation == DVBS2X_MOD_QPSK)
		dvbs2x_mod_qpsk(interleaved, symbols, num_symbols);
	else
		dvbs2x_mod_pi2bpsk(interleaved, symbols, num_symbols);

	/* Demodulate */
	if (mc->modulation == DVBS2X_MOD_QPSK)
		dvbs2x_demod_qpsk(symbols, llr, num_symbols, 0.001);
	else
		dvbs2x_demod_pi2bpsk(symbols, llr, num_symbols, 0.001);

	/* Deinterleave */
	dvbs2x_deinterleave(mc, llr, deinterleaved);

	/* LDPC decode */
	dvbs2x_ldpc_decoder_init(&ldpc_dec, mc, DVBS2X_LDPC_MAX_ITER);
	ret = dvbs2x_ldpc_decode(&ldpc_dec, deinterleaved,
				 ldpc_decoded, &iter_used);

	if (ret != 0) {
		printf("    LDPC failed (iter=%u)\n", iter_used);
		/* Hard decision fallback */
		for (i = 0; i < mc->k_ldpc; i++)
			ldpc_decoded[i] = (deinterleaved[i] < 0) ? 1 : 0;
	}

	/* BCH decode */
	dvbs2x_bch_decoder_init(&bch_dec, mc);
	memcpy(bch_decoded, ldpc_decoded, mc->n_bch);
	ret = dvbs2x_bch_decode(&bch_dec, bch_decoded);

	/* Verify */
	if (memcmp(bch_decoded, user_bits, mc->k_bch) == 0)
		printf("    PASS (iter=%u)\n", iter_used);
	else
		printf("    FAIL\n");

	free(user_bits); free(bch_out); free(ldpc_out); free(interleaved);
	free(symbols); free(llr); free(deinterleaved);
	free(ldpc_decoded); free(bch_decoded);
	return 0;
}

int main(void)
{
	unsigned int i;

	printf("DVB-S2X VL-SNR Loopback Tests\n");
	printf("=============================\n");

	for (i = 1; i <= DVBS2X_VLSNR_NUM_MODCODS; i++)
		test_loopback_modcod(i);

	printf("\nAll loopback tests completed.\n");
	return 0;
}
