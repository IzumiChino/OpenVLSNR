/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Noiseless Symbol-Chain Test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "dvbs2x_vlsnr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void add_carrier_offset(struct dvbs2x_complex *symbols,
			       unsigned int len, double freq, double phase)
{
	unsigned int i;

	for (i = 0; i < len; i++) {
		double angle = 2.0 * M_PI * freq * (double)i + phase;
		double c = cos(angle), s = sin(angle);
		double si = symbols[i].i, sq = symbols[i].q;

		symbols[i].i = si * c - sq * s;
		symbols[i].q = si * s + sq * c;
	}
}

static int test_modcod(unsigned int index)
{
	struct dvbs2x_modulator mod = { 0 };
	struct dvbs2x_demodulator demod = { 0 };
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *symbols = NULL;
	uint8_t *input = NULL, *output = NULL;
	unsigned int payload_len, symbol_len = 0, output_len = 0, i;
	int ret = -1;

	mc = dvbs2x_vlsnr_get_modcod(index);
	if (!mc || dvbs2x_modulator_init(&mod, index, 0.35, 2, 0) < 0)
		return -1;
	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		goto out;
	payload_len = mc->k_bch - 80;
	input = malloc(payload_len);
	output = malloc(payload_len);
	symbols = malloc(DVBS2X_VLSNR_FRAME_LONG * sizeof(*symbols));
	if (!input || !output || !symbols)
		goto out;
	for (i = 0; i < payload_len; i++)
		input[i] = (i * 11 + index * 3) & 1;
	if (dvbs2x_modulate_symbols_ex(&mod, input, payload_len, symbols,
				       DVBS2X_VLSNR_FRAME_LONG,
				       &symbol_len) < 0)
		goto out;
	add_carrier_offset(symbols, symbol_len, 0.0002, 0.31);
	if (dvbs2x_demodulate_symbols_ex(&demod, symbols, symbol_len, 0.001,
					 output, payload_len, &output_len) < 0 ||
	    output_len != payload_len ||
	    memcmp(input, output, payload_len) != 0)
		goto out;
	ret = 0;
out:
	free(input);
	free(output);
	free(symbols);
	dvbs2x_demodulator_destroy(&demod);
	dvbs2x_modulator_destroy(&mod);
	return ret;
}

int main(void)
{
	unsigned int index;

	printf("DVB-S2X VL-SNR Noiseless Full-Chain Tests\n");
	printf("=========================================\n");
	for (index = 1; index <= DVBS2X_VLSNR_NUM_MODCODS; index++) {
		printf("  MODCOD %u... ", index);
		if (test_modcod(index) < 0) {
			printf("FAIL\n");
			return 1;
		}
		printf("PASS\n");
	}
	return 0;
}
