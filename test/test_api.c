/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Public API Validation Test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dvbs2x_vlsnr.h"

static int test_init_parameters(void)
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_demodulator demod;

	if (dvbs2x_modulator_init(NULL, 1, 0.35, 2, 0) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_modulator_init(&mod, 0, 0.35, 2, 0) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_modulator_init(&mod, 1, 0.35, 1, 0) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_modulator_init(&mod, 1, 0.0, 2, 0) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_modulator_init(&mod, 1, 0.35, 2, 262142) !=
	    DVBS2X_ERR_PARAM)
		return -1;

	memset(&demod, 0xa5, sizeof(demod));
	if (dvbs2x_demodulator_init(NULL, 0.35, 2, 0) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_demodulator_init(&demod, 0.35, 1, 0) !=
	    DVBS2X_ERR_PARAM)
		return -1;
	dvbs2x_demodulator_destroy(&demod);
	dvbs2x_demodulator_destroy(&demod);
	dvbs2x_demodulator_destroy(NULL);
	dvbs2x_modulator_destroy(NULL);
	return 0;
}

static int test_modulator_parameters(void)
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_complex output[1];
	uint8_t data[1] = { 0 };
	unsigned int out_len;
	unsigned int required;

	if (dvbs2x_modulator_init(&mod, 9, 0.35, 2, 0) < 0)
		return -1;
	out_len = 123;
	if (dvbs2x_modulate(NULL, data, 1, output, &out_len) !=
	    DVBS2X_ERR_PARAM || out_len != 0)
		return -1;
	out_len = 123;
	if (dvbs2x_modulate(&mod, NULL, 1, output, &out_len) !=
	    DVBS2X_ERR_PARAM || out_len != 0)
		return -1;
	out_len = 123;
	if (dvbs2x_modulate(&mod, data, mod.bb_ctx.dfl + 1, output,
			   &out_len) != DVBS2X_ERR_PARAM || out_len != 0)
		return -1;
	if (dvbs2x_modulate(&mod, data, 1, output, NULL) !=
	    DVBS2X_ERR_PARAM)
		return -1;
	required = 0;
	if (dvbs2x_modulate_ex(&mod, data, 1, output, 0, &required) !=
	    DVBS2X_ERR_SHORT || required == 0)
		return -1;
	out_len = 0;
	if (dvbs2x_modulate_symbols_ex(&mod, data, 1, output, 0,
				       &out_len) != DVBS2X_ERR_SHORT ||
	    out_len == 0)
		return -1;
	dvbs2x_modulator_destroy(&mod);
	return 0;
}

static int test_bbframe_parameters(void)
{
	const struct dvbs2x_modcod *mc = dvbs2x_vlsnr_get_modcod(9);
	struct dvbs2x_bb_frame_ctx ctx;
	uint8_t *frame;
	uint8_t data[1] = { 0 };
	unsigned int out_len = 123;
	int ret = -1;

	if (!mc)
		return -1;
	dvbs2x_bb_frame_init(&ctx, mc, DVBS2X_STREAM_GS);
	frame = calloc(mc->k_bch, 1);
	if (!frame)
		return -1;
	if (dvbs2x_bb_frame_build(NULL, data, 1, frame) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_bb_frame_build(&ctx, NULL, 1, frame) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_bb_frame_build(&ctx, data, ctx.dfl + 1, frame) !=
	    DVBS2X_ERR_PARAM ||
	    dvbs2x_bb_frame_parse(&ctx, frame, NULL, &out_len) !=
	    DVBS2X_ERR_PARAM || out_len != 0)
		goto out;
	ret = 0;
out:
	free(frame);
	return ret;
}

static int test_bbframe_padding(void)
{
	const struct dvbs2x_modcod *mc = dvbs2x_vlsnr_get_modcod(9);
	struct dvbs2x_bb_frame_ctx ctx;
	uint8_t input[37], output[37];
	uint8_t *frame;
	unsigned int output_len = 0, i;
	int ret = -1;

	if (!mc)
		return -1;
	for (i = 0; i < sizeof(input); i++)
		input[i] = (i * 5 + 1) & 1;
	frame = calloc(mc->k_bch, 1);
	if (!frame)
		return -1;
	dvbs2x_bb_frame_init(&ctx, mc, DVBS2X_STREAM_GS);
	if (dvbs2x_bb_frame_build(&ctx, input, sizeof(input), frame) < 0 ||
	    dvbs2x_bb_frame_parse_ex(&ctx, frame, output, sizeof(output),
				     &output_len) < 0 ||
	    output_len != sizeof(input) ||
	    memcmp(input, output, sizeof(input)) != 0)
		goto out;
	dvbs2x_bb_frame_init(&ctx, mc, DVBS2X_STREAM_TS);
	if (dvbs2x_bb_frame_build(&ctx, input, sizeof(input), frame) !=
	    DVBS2X_ERR_PARAM)
		goto out;
	ret = 0;
out:
	free(frame);
	return ret;
}

static int test_demodulator_capacity(void)
{
	struct dvbs2x_modulator mod;
	struct dvbs2x_demodulator demod;
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *symbols = NULL;
	uint8_t *payload = NULL;
	uint8_t output[1];
	unsigned int payload_len, symbol_len = 0, required = 0;
	int ret = -1;

	mc = dvbs2x_vlsnr_get_modcod(9);
	if (!mc || dvbs2x_modulator_init(&mod, 9, 0.35, 2, 0) < 0 ||
	    dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	payload_len = mc->k_bch - 80;
	payload = calloc(payload_len, 1);
	symbols = calloc(DVBS2X_VLSNR_FRAME_SHORT, sizeof(*symbols));
	if (!payload || !symbols)
		goto out;
	if (dvbs2x_modulate_symbols_ex(&mod, payload, payload_len, symbols,
				       DVBS2X_VLSNR_FRAME_SHORT,
				       &symbol_len) < 0)
		goto out;
	if (dvbs2x_demodulate_symbols_ex(&demod, symbols, symbol_len, 0.001,
					 output, 0, &required) !=
	    DVBS2X_ERR_SHORT || required != payload_len)
		goto out;
	ret = 0;
out:
	free(payload);
	free(symbols);
	dvbs2x_demodulator_destroy(&demod);
	dvbs2x_modulator_destroy(&mod);
	return ret;
}

int main(void)
{
	printf("DVB-S2X VL-SNR API Validation Tests\n");
	printf("===================================\n");
	if (test_init_parameters() < 0 ||
	    test_modulator_parameters() < 0 ||
	    test_bbframe_parameters() < 0 ||
	    test_bbframe_padding() < 0 ||
	    test_demodulator_capacity() < 0) {
		printf("FAIL\n");
		return 1;
	}
	printf("PASS\n");
	return 0;
}
