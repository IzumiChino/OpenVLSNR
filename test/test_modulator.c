// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Basic modulator chain test
 *
 * Tests the encoding and modulation pipeline:
 * BB frame -> BCH encode -> LDPC encode -> interleave ->
 * pi/2-BPSK modulate -> scramble -> verify
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
#include "scrambler.h"
#include "bb_frame.h"
#include "plheader.h"
#include "vlsnr_header.h"
#include "filter.h"

static void test_modcod_table(void)
{
	const struct dvbs2x_modcod *mc;
	const char *name;
	unsigned int i;

	printf("  MODCOD table...\n");

	for (i = 1; i <= DVBS2X_VLSNR_NUM_MODCODS; i++) {
		mc = dvbs2x_vlsnr_get_modcod(i);
		assert(mc != NULL);
		assert(mc->index == i);
		assert(mc->fec_len == 64800 || mc->fec_len == 32400 ||
		       mc->fec_len == 16200);
		assert(mc->k_bch < mc->n_bch);
		assert(mc->k_ldpc == mc->xs + mc->n_bch);
		assert(mc->k_ldpc < mc->fec_len);
		name = dvbs2x_vlsnr_get_modcod_name(i);
		assert(name != NULL);
		assert(dvbs2x_vlsnr_get_modcod_by_name(name) == mc);
	}

	/* Invalid index */
	assert(dvbs2x_vlsnr_get_modcod(0) == NULL);
	assert(dvbs2x_vlsnr_get_modcod(10) == NULL);
	assert(dvbs2x_vlsnr_get_modcod_name(0) == NULL);
	assert(dvbs2x_vlsnr_get_modcod_by_name(NULL) == NULL);
	assert(dvbs2x_vlsnr_get_modcod_by_name("BPSK 1/5") == NULL);
	assert(dvbs2x_vlsnr_get_modcod_by_name("BPSK 1/3") == NULL);

	printf("    PASS\n");
}

static void test_bch_encode(void)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_bch_encoder enc;
	uint8_t *input;
	uint8_t *output;
	int ret;

	printf("  BCH encode (MODCOD 9, short frame)...\n");

	mc = dvbs2x_vlsnr_get_modcod(9);
	assert(mc != NULL);

	ret = dvbs2x_bch_encoder_init(&enc, mc);
	assert(ret == 0);
	assert(enc.k == mc->k_bch);
	assert(enc.n == mc->n_bch);

	input = calloc(mc->k_bch, 1);
	output = calloc(mc->n_bch, 1);
	assert(input && output);

	/* Set some test data */
	input[0] = 1;
	input[1] = 0;
	input[2] = 1;
	input[3] = 1;

	ret = dvbs2x_bch_encode(&enc, input, output);
	assert(ret == 0);

	/* Verify systematic: first k bits should match input */
	assert(memcmp(output, input, mc->k_bch) == 0);

	/* Verify parity is not all zeros (for non-zero input) */
	{
		unsigned int i;
		int has_parity = 0;

		for (i = mc->k_bch; i < mc->n_bch; i++) {
			if (output[i]) {
				has_parity = 1;
				break;
			}
		}
		assert(has_parity);
	}

	free(input);
	free(output);
	printf("    PASS\n");
}

static void test_ldpc_encode(void)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_ldpc_encoder enc;
	uint8_t *info;
	uint8_t *codeword;
	int ret;

	printf("  LDPC encode (MODCOD 9, short frame)...\n");

	mc = dvbs2x_vlsnr_get_modcod(9);
	assert(mc != NULL);

	ret = dvbs2x_ldpc_encoder_init(&enc, mc);
	assert(ret == 0);

	info = calloc(mc->k_ldpc, 1);
	codeword = calloc(mc->fec_len, 1);
	assert(info && codeword);

	/* Set test data */
	info[0] = 1;
	info[10] = 1;
	info[100] = 1;

	ret = dvbs2x_ldpc_encode(&enc, info, codeword);
	assert(ret == 0);

	/* Verify systematic */
	assert(memcmp(codeword, info, mc->k_ldpc) == 0);

	/* Verify parity is not all zeros */
	{
		unsigned int i;
		int has_parity = 0;

		for (i = mc->k_ldpc; i < mc->fec_len; i++) {
			if (codeword[i]) {
				has_parity = 1;
				break;
			}
		}
		assert(has_parity);
	}

	free(info);
	free(codeword);
	printf("    PASS\n");
}

static void test_pi2bpsk_modulation(void)
{
	uint8_t bits[8] = {0, 1, 0, 1, 0, 0, 1, 1};
	struct dvbs2x_complex symbols[8];
	unsigned int i;

	printf("  pi/2-BPSK modulation...\n");

	dvbs2x_mod_pi2bpsk(bits, symbols, 8);

	/* Verify unit power for all symbols */
	for (i = 0; i < 8; i++) {
		double power = symbols[i].i * symbols[i].i +
			       symbols[i].q * symbols[i].q;
		assert(power > 0.99 && power < 1.01);
	}

	printf("    PASS\n");
}

static void test_scrambler(void)
{
	struct dvbs2x_scrambler scr;
	struct dvbs2x_complex symbols[100];
	struct dvbs2x_complex original[100];
	unsigned int i;

	printf("  PL scrambler...\n");

	/* Generate test symbols */
	for (i = 0; i < 100; i++) {
		symbols[i].i = 1.0;
		symbols[i].q = 0.0;
		original[i] = symbols[i];
	}

	/* Scramble */
	dvbs2x_scrambler_init(&scr, 0);
	dvbs2x_scramble(&scr, symbols, 100);

	/* Verify symbols changed */
	{
		int changed = 0;

		for (i = 0; i < 100; i++) {
			if (symbols[i].i != original[i].i ||
			    symbols[i].q != original[i].q) {
				changed = 1;
				break;
			}
		}
		assert(changed);
	}

	/* Descramble and verify recovery */
	dvbs2x_scrambler_init(&scr, 0);
	dvbs2x_descramble(&scr, symbols, 100);

	for (i = 0; i < 100; i++) {
		double err_i = symbols[i].i - original[i].i;
		double err_q = symbols[i].q - original[i].q;

		assert(err_i * err_i + err_q * err_q < 1e-20);
	}

	printf("    PASS\n");
}

static void test_plheader(void)
{
	struct dvbs2x_complex symbols[DVBS2X_PLHEADER_LEN];
	unsigned int recovered_pls;
	int ret;

	printf("  PL header generate/recover...\n");

	/* Generate header for VL-SNR Set 1 (PLS 129 = 0x80 | 1) */
	dvbs2x_plheader_generate(DVBS2X_PLS_VLSNR_SET1, symbols);

	/* Recover (no noise) - returns lower 7 bits */
	ret = dvbs2x_plheader_recover(symbols, &recovered_pls);
	assert(ret == 0);
	assert(recovered_pls == (DVBS2X_PLS_VLSNR_SET1 & 0x7F));

	/* Test Set 2 (PLS 131 = 0x80 | 3) */
	dvbs2x_plheader_generate(DVBS2X_PLS_VLSNR_SET2, symbols);
	ret = dvbs2x_plheader_recover(symbols, &recovered_pls);
	assert(ret == 0);
	assert(recovered_pls == (DVBS2X_PLS_VLSNR_SET2 & 0x7F));

	printf("    PASS\n");
}

static void test_rrc_filter(void)
{
	struct dvbs2x_rrc_filter flt;
	struct dvbs2x_complex in[32];
	struct dvbs2x_complex out[64];
	unsigned int out_len;
	int ret;
	unsigned int i;

	printf("  RRC filter...\n");

	ret = dvbs2x_rrc_filter_init(&flt, 0.35, 2, 8);
	assert(ret == 0);
	assert(flt.num_taps == 33);

	/* Generate impulse at position 0 */
	memset(in, 0, sizeof(in));
	in[0].i = 1.0;

	dvbs2x_rrc_upsample(&flt, in, 32, out, &out_len);
	assert(out_len == 64);

	/* Verify output has energy (impulse response) */
	{
		double energy = 0.0;

		for (i = 0; i < out_len; i++)
			energy += out[i].i * out[i].i + out[i].q * out[i].q;
		assert(energy > 0.01);
	}

	printf("    PASS\n");
}

int main(void)
{
	printf("DVB-S2X VL-SNR Modulator Tests\n");
	printf("==============================\n");

	test_modcod_table();
	test_bch_encode();
	test_ldpc_encode();
	test_pi2bpsk_modulation();
	test_scrambler();
	test_plheader();
	test_rrc_filter();

	printf("\nAll tests passed.\n");
	return 0;
}
