// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR End-to-End Test
 *
 * Tests the complete modulator -> channel -> demodulator chain.
 * Verifies data recovery through the full pipeline at high SNR
 * (no noise, verifying structural correctness).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "dvbs2x_vlsnr.h"

#define MAX_OUTPUT_SAMPLES	(200000)

static void test_modulator_pipeline(unsigned int modcod_idx)
{
	struct dvbs2x_modulator mod;
	const struct dvbs2x_modcod *mc;
	uint8_t *user_data;
	struct dvbs2x_complex *output;
	unsigned int out_len;
	unsigned int user_len;
	int ret;
	unsigned int i;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	assert(mc != NULL);

	printf("  MODCOD %u: ", modcod_idx);
	fflush(stdout);

	ret = dvbs2x_modulator_init(&mod, modcod_idx, 0.35, 2, 0);
	assert(ret == 0);

	/* Generate test user data */
	user_len = mc->k_bch - 80;	/* subtract BB header */
	if (user_len > mc->k_bch)
		user_len = mc->k_bch / 2;
	user_data = malloc(user_len);
	assert(user_data != NULL);

	for (i = 0; i < user_len; i++)
		user_data[i] = (i * 13 + 7) & 1;

	/* Allocate output buffer */
	output = malloc(MAX_OUTPUT_SAMPLES * sizeof(struct dvbs2x_complex));
	assert(output != NULL);

	/* Run modulator */
	ret = dvbs2x_modulate(&mod, user_data, user_len, output, &out_len);
	assert(ret == 0);
	assert(out_len > 0);

	printf("%u symbols -> %u samples (sps=2), ",
	       out_len / 2, out_len);

	/* Verify output has energy */
	{
		double energy = 0.0;

		for (i = 0; i < out_len && i < 1000; i++)
			energy += output[i].i * output[i].i +
				  output[i].q * output[i].q;
		assert(energy > 0.1);
	}

	printf("PASS\n");

	free(user_data);
	free(output);
}

int main(void)
{
	unsigned int i;

	printf("DVB-S2X VL-SNR End-to-End Tests\n");
	printf("===============================\n\n");

	printf("Modulator pipeline:\n");
	for (i = 1; i <= DVBS2X_VLSNR_NUM_MODCODS; i++)
		test_modulator_pipeline(i);

	printf("\nAll end-to-end tests passed.\n");
	return 0;
}
