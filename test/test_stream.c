/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Streaming Ownership Test
 *
 * Verify that arbitrary input chunks are consumed exactly once while
 * incomplete frames and read-ahead remain owned by the demodulator.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dvbs2x_vlsnr.h"

#define NUM_FRAMES	4

struct stream_fixture {
	struct dvbs2x_modulator mod;
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *samples;
	uint8_t *payloads;
	unsigned int sample_len;
	unsigned int payload_len;
};

static void fixture_destroy(struct stream_fixture *fix)
{
	free(fix->samples);
	free(fix->payloads);
	dvbs2x_modulator_destroy(&fix->mod);
}

static int fixture_init(struct stream_fixture *fix, unsigned int modcod_idx)
{
	unsigned int max_frame_samples;
	unsigned int offset = 0;
	unsigned int frame, i;

	memset(fix, 0, sizeof(*fix));
	fix->mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!fix->mc ||
	    dvbs2x_modulator_init(&fix->mod, modcod_idx, 0.35, 2, 0) < 0)
		return -1;

	fix->payload_len = fix->mc->k_bch - 80;
	max_frame_samples = DVBS2X_VLSNR_FRAME_LONG * 2;
	fix->samples = calloc(NUM_FRAMES * max_frame_samples,
			      sizeof(*fix->samples));
	fix->payloads = calloc(NUM_FRAMES * fix->payload_len, 1);
	if (!fix->samples || !fix->payloads)
		goto fail;

	for (frame = 0; frame < NUM_FRAMES; frame++) {
		uint8_t *payload = fix->payloads + frame * fix->payload_len;
		unsigned int out_len = 0;

		for (i = 0; i < fix->payload_len; i++)
			payload[i] = ((i * 13) + frame * 7 + 1) & 1;
		if (dvbs2x_modulate(&fix->mod, payload, fix->payload_len,
				     fix->samples + offset, &out_len) < 0)
			goto fail;
		offset += out_len;
	}
	fix->sample_len = offset;
	return 0;

fail:
	fixture_destroy(fix);
	return -1;
}

static int check_frame(const struct stream_fixture *fix,
		       const uint8_t *received, unsigned int received_len,
		       unsigned int frame)
{
	const uint8_t *expected = fix->payloads + frame * fix->payload_len;

	if (received_len != fix->payload_len)
		return -1;
	return memcmp(received, expected, fix->payload_len);
}

static int test_one_frame(const struct stream_fixture *fix)
{
	struct dvbs2x_demodulator demod;
	uint8_t *received = NULL;
	unsigned int frame_samples = fix->sample_len / NUM_FRAMES;
	unsigned int received_len = 0;
	unsigned int consumed = 0;
	int ret = -1;

	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	received = calloc(fix->mc->k_bch, 1);
	if (!received)
		goto out;
	if (dvbs2x_demodulate_stream(&demod, fix->samples, frame_samples,
				     received, &received_len, &consumed) < 0)
		goto out;
	if (consumed != frame_samples ||
	    check_frame(fix, received, received_len, 0) != 0)
		goto out;
	ret = 0;
out:
	free(received);
	dvbs2x_demodulator_destroy(&demod);
	return ret;
}

static int test_bbframe(const struct stream_fixture *fix)
{
	struct dvbs2x_bb_frame_ctx bb;
	struct dvbs2x_demodulator demod;
	struct dvbs2x_demod_stats stats;
	uint8_t *expected = NULL;
	uint8_t *received = NULL;
	unsigned int frame_samples = fix->sample_len / NUM_FRAMES;
	unsigned int received_len = 0, consumed = 0;
	int ret = -1;

	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	expected = calloc(fix->mc->k_bch, 1);
	received = calloc(fix->mc->k_bch, 1);
	if (!expected || !received)
		goto out;
	dvbs2x_bb_frame_init(&bb, fix->mc, DVBS2X_STREAM_GS);
	if (dvbs2x_bb_frame_build(&bb, fix->payloads, fix->payload_len,
				  expected) < 0)
		goto out;
	if (dvbs2x_demodulate_bbframe_stream_ex(
		    &demod, fix->samples, frame_samples, received,
		    fix->mc->k_bch, &received_len, &consumed) < 0)
		goto out;
	if (consumed != frame_samples || received_len != fix->mc->k_bch ||
	    memcmp(received, expected, received_len) != 0)
		goto out;
	if (dvbs2x_demodulator_get_stats(&demod, &stats) < 0 ||
	    stats.result != 0 || stats.modcod != fix->mc->index ||
	    stats.sync_confidence <= 0.0 || !stats.ldpc_iterations)
		goto out;
	ret = 0;
out:
	free(received);
	free(expected);
	dvbs2x_demodulator_destroy(&demod);
	return ret;
}

static int test_large_buffer(const struct stream_fixture *fix)
{
	struct dvbs2x_demodulator demod;
	uint8_t *received = NULL;
	unsigned int received_len, consumed;
	unsigned int frame;
	int ret = -1;

	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	received = calloc(fix->mc->k_bch, 1);
	if (!received)
		goto out;

	for (frame = 0; frame < NUM_FRAMES; frame++) {
		const struct dvbs2x_complex *input = NULL;
		unsigned int input_len = 0;

		if (frame == 0) {
			input = fix->samples;
			input_len = fix->sample_len;
		}
		received_len = 0;
		consumed = 0;
		if (dvbs2x_demodulate_stream(&demod, input, input_len,
					     received, &received_len,
					     &consumed) < 0)
			goto out;
		if (consumed != input_len ||
		    check_frame(fix, received, received_len, frame) != 0)
			goto out;
	}
	ret = 0;
out:
	free(received);
	dvbs2x_demodulator_destroy(&demod);
	return ret;
}

static int test_arbitrary_chunks(const struct stream_fixture *fix)
{
	static const unsigned int chunks[] = {
		1, 17, 511, 4093, 37, 8191, 3, 1021,
	};
	struct dvbs2x_demodulator demod;
	uint8_t *received = NULL;
	unsigned int offset = 0, frame = 0, chunk_idx = 0;
	int ret = -1;

	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	received = calloc(fix->mc->k_bch, 1);
	if (!received)
		goto out;

	while (offset < fix->sample_len) {
		unsigned int input_len = chunks[chunk_idx++ %
			(sizeof(chunks) / sizeof(chunks[0]))];
		unsigned int received_len = 0, consumed = 0;
		int dret;

		if (input_len > fix->sample_len - offset)
			input_len = fix->sample_len - offset;
		dret = dvbs2x_demodulate_stream(&demod, fix->samples + offset,
					       input_len, received,
					       &received_len, &consumed);
		if (consumed != input_len)
			goto out;
		offset += consumed;
		if (dret == DVBS2X_ERR_SHORT)
			continue;
		if (dret < 0 || frame >= NUM_FRAMES ||
		    check_frame(fix, received, received_len, frame) != 0)
			goto out;
		frame++;
	}

	while (frame < NUM_FRAMES) {
		unsigned int received_len = 0, consumed = 1;

		if (dvbs2x_demodulate_stream(&demod, NULL, 0, received,
					     &received_len, &consumed) < 0 ||
		    consumed != 0 ||
		    check_frame(fix, received, received_len, frame) != 0)
			goto out;
		frame++;
	}
	ret = 0;
out:
	free(received);
	dvbs2x_demodulator_destroy(&demod);
	return ret;
}

static int test_modcod(unsigned int modcod_idx)
{
	struct stream_fixture fix;
	int ret = -1;

	printf("  MODCOD %u...\n", modcod_idx);
	if (fixture_init(&fix, modcod_idx) < 0)
		return -1;
	if (test_one_frame(&fix) < 0 || test_bbframe(&fix) < 0 ||
	    test_large_buffer(&fix) < 0 ||
	    test_arbitrary_chunks(&fix) < 0)
		goto out;
	printf("    PASS\n");
	ret = 0;
out:
	fixture_destroy(&fix);
	return ret;
}

static int test_search_progress(void)
{
	struct dvbs2x_demodulator demod;
	struct dvbs2x_complex *samples = NULL;
	uint8_t *received = NULL;
	unsigned int sample_len = DVBS2X_VLSNR_FRAME_SHORT * 2;
	unsigned int received_len = 0;
	unsigned int consumed = 0;
	int ret = -1;

	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return -1;
	samples = calloc(sample_len, sizeof(*samples));
	received = calloc(DVBS2X_LDPC_NORMAL, 1);
	if (!samples || !received)
		goto out;
	if (dvbs2x_demodulate_stream(&demod, samples, sample_len,
				     received, &received_len, &consumed) !=
	    DVBS2X_ERR_NOSYNC || consumed != sample_len)
		goto out;
	if (dvbs2x_demodulate_stream(&demod, NULL, 0, received,
				     &received_len, &consumed) !=
	    DVBS2X_ERR_SHORT)
		goto out;
	ret = 0;
out:
	free(samples);
	free(received);
	dvbs2x_demodulator_destroy(&demod);
	return ret;
}

int main(void)
{
	static const unsigned int modcods[] = { 1, 2, 9 };
	unsigned int i;

	printf("DVB-S2X VL-SNR Streaming Tests\n");
	printf("===============================\n");
	if (test_search_progress() < 0) {
		printf("  SEARCH progress: FAIL\n");
		return 1;
	}
	for (i = 0; i < sizeof(modcods) / sizeof(modcods[0]); i++) {
		if (test_modcod(modcods[i]) < 0) {
			printf("    FAIL\n");
			return 1;
		}
	}
	printf("\nAll streaming tests passed.\n");
	return 0;
}
