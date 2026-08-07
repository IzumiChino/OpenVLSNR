/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR FEC Loopback Test
 *
 * Exercise shortening, puncturing, interleaving and SF2 spreading for
 * every VL-SNR MODCOD without channel impairments.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dvbs2x_types.h"
#include "dvbs2x_modcod.h"
#include "bch.h"
#include "ldpc.h"
#include "modulator.h"
#include "interleaver.h"

static int test_loopback_modcod(unsigned int modcod_idx)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_bch_encoder bch_enc;
	struct dvbs2x_bch_decoder bch_dec;
	struct dvbs2x_ldpc_encoder ldpc_enc;
	struct dvbs2x_ldpc_decoder ldpc_dec = { 0 };
	struct dvbs2x_complex *symbols = NULL;
	uint8_t *user_bits = NULL;
	uint8_t *bch_out = NULL;
	uint8_t *ldpc_info = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *tx_bits = NULL;
	uint8_t *interleaved = NULL;
	uint8_t *spread_bits = NULL;
	uint8_t *ldpc_decoded = NULL;
	uint8_t *bch_decoded = NULL;
	double *symbol_llr = NULL;
	double *llr = NULL;
	double *deinterleaved = NULL;
	double *full_llr = NULL;
	unsigned int tx_coded;
	unsigned int num_symbols;
	unsigned int iter_used = 0;
	unsigned int i, p_idx, t_idx, d_idx;
	int ret = -1;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		goto out;

	printf("  MODCOD %u (%s %u/%u, %u bits)...\n",
	       modcod_idx,
	       mc->modulation == DVBS2X_MOD_BPSK ? "BPSK" : "QPSK",
	       mc->code_rate_num, mc->code_rate_den, mc->fec_len);

	tx_coded = dvbs2x_tx_coded_bits(mc);
	num_symbols = mc->modulation == DVBS2X_MOD_QPSK ?
		      tx_coded / 2 : tx_coded;
	if (mc->has_spread)
		num_symbols *= 2;

	user_bits = calloc(mc->k_bch, 1);
	bch_out = calloc(mc->n_bch, 1);
	ldpc_info = calloc(mc->k_ldpc, 1);
	ldpc_out = calloc(mc->fec_len, 1);
	tx_bits = calloc(tx_coded, 1);
	interleaved = calloc(tx_coded, 1);
	if (mc->has_spread)
		spread_bits = calloc(num_symbols, 1);
	symbols = calloc(num_symbols, sizeof(*symbols));
	symbol_llr = calloc(num_symbols * 2, sizeof(*symbol_llr));
	llr = calloc(tx_coded, sizeof(*llr));
	deinterleaved = calloc(tx_coded, sizeof(*deinterleaved));
	full_llr = calloc(mc->fec_len, sizeof(*full_llr));
	ldpc_decoded = calloc(mc->k_ldpc, 1);
	bch_decoded = calloc(mc->n_bch, 1);
	if (!user_bits || !bch_out || !ldpc_info || !ldpc_out ||
	    !tx_bits || !interleaved || (mc->has_spread && !spread_bits) ||
	    !symbols || !symbol_llr || !llr || !deinterleaved ||
	    !full_llr || !ldpc_decoded || !bch_decoded) {
		printf("    allocation failed\n");
		goto out;
	}

	for (i = 0; i < mc->k_bch; i++)
		user_bits[i] = (i * 7 + 3) & 1;

	if (dvbs2x_bch_encoder_init(&bch_enc, mc) < 0 ||
	    dvbs2x_ldpc_encoder_init(&ldpc_enc, mc) < 0 ||
	    dvbs2x_ldpc_decoder_init(&ldpc_dec, mc,
				       DVBS2X_LDPC_MAX_ITER) < 0 ||
	    dvbs2x_bch_decoder_init(&bch_dec, mc) < 0) {
		printf("    initialization failed\n");
		goto out;
	}

	if (dvbs2x_bch_encode(&bch_enc, user_bits, bch_out) < 0)
		goto encode_failed;
	memcpy(ldpc_info + mc->xs, bch_out, mc->n_bch);
	if (dvbs2x_ldpc_encode(&ldpc_enc, ldpc_info, ldpc_out) < 0)
		goto encode_failed;

	/* Remove shortened information bits and punctured parity bits. */
	t_idx = 0;
	for (i = mc->xs; i < mc->k_ldpc; i++)
		tx_bits[t_idx++] = ldpc_out[i];
	p_idx = 0;
	for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
		if (mc->xp && p_idx < mc->xp &&
		    i == p_idx * mc->p_period) {
			p_idx++;
			continue;
		}
		tx_bits[t_idx++] = ldpc_out[mc->k_ldpc + i];
	}
	if (t_idx != tx_coded)
		goto encode_failed;

	dvbs2x_interleave(mc, tx_bits, interleaved);
	if (mc->modulation == DVBS2X_MOD_QPSK) {
		dvbs2x_mod_qpsk(interleaved, symbols, num_symbols);
		dvbs2x_demod_qpsk(symbols, symbol_llr, num_symbols, 0.001);
		memcpy(llr, symbol_llr, tx_coded * sizeof(*llr));
	} else if (mc->has_spread) {
		dvbs2x_mod_spread_bits(interleaved, spread_bits, tx_coded);
		dvbs2x_mod_pi2bpsk(spread_bits, symbols, num_symbols);
		dvbs2x_demod_pi2bpsk(symbols, symbol_llr, num_symbols, 0.001);
		dvbs2x_demod_despread_llr(symbol_llr, llr, tx_coded);
	} else {
		dvbs2x_mod_pi2bpsk(interleaved, symbols, num_symbols);
		dvbs2x_demod_pi2bpsk(symbols, llr, num_symbols, 0.001);
	}
	dvbs2x_deinterleave(mc, llr, deinterleaved);

	/* Restore known shortened bits and erasures for punctured parity. */
	for (i = 0; i < mc->xs; i++)
		full_llr[i] = 40.0;
	for (i = mc->xs; i < mc->k_ldpc; i++)
		full_llr[i] = deinterleaved[i - mc->xs];
	d_idx = mc->k_ldpc - mc->xs;
	p_idx = 0;
	for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
		if (mc->xp && p_idx < mc->xp &&
		    i == p_idx * mc->p_period) {
			full_llr[mc->k_ldpc + i] = 0.0;
			p_idx++;
		} else {
			full_llr[mc->k_ldpc + i] = deinterleaved[d_idx++];
		}
	}

	if (dvbs2x_ldpc_decode(&ldpc_dec, full_llr, ldpc_decoded,
			       &iter_used) < 0) {
		printf("    LDPC failed (iter=%u)\n", iter_used);
		goto out;
	}
	memcpy(bch_decoded, ldpc_decoded + mc->xs, mc->n_bch);
	if (dvbs2x_bch_decode(&bch_dec, bch_decoded) < 0) {
		printf("    BCH failed\n");
		goto out;
	}
	if (memcmp(bch_decoded, user_bits, mc->k_bch) != 0) {
		printf("    bit comparison failed\n");
		goto out;
	}

	printf("    PASS (iter=%u)\n", iter_used);
	ret = 0;
	goto out;

encode_failed:
	printf("    encode failed\n");
out:
	free(user_bits);
	free(bch_out);
	free(ldpc_info);
	free(ldpc_out);
	free(tx_bits);
	free(interleaved);
	free(spread_bits);
	free(symbols);
	free(symbol_llr);
	free(llr);
	free(deinterleaved);
	free(full_llr);
	free(ldpc_decoded);
	free(bch_decoded);
	dvbs2x_ldpc_decoder_free(&ldpc_dec);
	return ret;
}

int main(void)
{
	unsigned int i;
	int failed = 0;

	printf("DVB-S2X VL-SNR Loopback Tests\n");
	printf("=============================\n");

	for (i = 1; i <= DVBS2X_VLSNR_NUM_MODCODS; i++) {
		if (test_loopback_modcod(i) < 0)
			failed = 1;
	}

	if (failed) {
		printf("\nLoopback tests failed.\n");
		return 1;
	}

	printf("\nAll loopback tests passed.\n");
	return 0;
}
