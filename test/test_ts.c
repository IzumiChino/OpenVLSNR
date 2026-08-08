// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR MPEG-TS Mode Adaptation Tests
 *
 * Golden hashes were generated with GNU Radio 3.10.12.0 gr-dtv using
 * dvb_bbheader_bb followed by dvb_bbscrambler_bb.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dvbs2x_vlsnr.h"

#define TEST_FRAMES	3
#define TEST_PACKETS	128

struct ts_vector {
	unsigned int modcod;
	uint32_t hash[TEST_FRAMES];
};

static const struct ts_vector vectors[] = {
	{ 1, { 0xdc853528, 0xa9681463, 0x2732cb60 } },
	{ 2, { 0x9a3b969a, 0x4c4d4d3f, 0xa6f1912e } },
	{ 9, { 0x3718d97e, 0x8675425f, 0x4787dfb0 } },
};

static void make_packet(uint8_t *packet, unsigned int index)
{
	unsigned int i;

	packet[0] = 0x47;
	packet[1] = 0x40 | ((index >> 8) & 0x1f);
	packet[2] = index & 0xff;
	packet[3] = 0x10 | (index & 0x0f);
	for (i = 4; i < DVBS2X_TS_PACKET_SIZE; i++)
		packet[i] = (index * 29 + i * 17 + 3) & 0xff;
}

static uint32_t hash_bits(const uint8_t *bits, unsigned int len)
{
	uint32_t hash = 2166136261U;
	unsigned int i;

	for (i = 0; i < len; i++) {
		hash ^= bits[i];
		hash *= 16777619U;
	}
	return hash;
}

static int test_vector(const struct ts_vector *vector)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_ts_tx tx = { 0 };
	struct dvbs2x_ts_rx rx;
	uint8_t source[TEST_PACKETS][DVBS2X_TS_PACKET_SIZE];
	uint8_t output[16 * DVBS2X_TS_PACKET_SIZE];
	uint8_t *frame = NULL;
	unsigned int frame_len, frame_count = 0, output_count = 0;
	unsigned int recovered = 0, packet_index;
	int ret = -1;

	mc = dvbs2x_vlsnr_get_modcod(vector->modcod);
	if (!mc)
		return -1;
	frame = malloc(mc->k_bch);
	if (!frame)
		return -1;
	for (packet_index = 0; packet_index < TEST_PACKETS;
	     packet_index++)
		make_packet(source[packet_index], packet_index);
	if (dvbs2x_ts_tx_init(&tx, mc, DVBS2X_RO_0_35) < 0 ||
	    dvbs2x_ts_rx_init(&rx, mc) < 0)
		goto out;

	for (packet_index = 0; packet_index < TEST_PACKETS &&
	     frame_count < TEST_FRAMES; packet_index++) {
		if (dvbs2x_ts_tx_push(&tx, source[packet_index], frame,
				      mc->k_bch, &frame_len) < 0)
			goto out;
		if (!frame_len)
			continue;
		if (hash_bits(frame, frame_len) !=
		    vector->hash[frame_count])
			goto out;
		if (dvbs2x_ts_rx_push(&rx, frame, output, 16,
				      &output_count) < 0)
			goto out;
		if (memcmp(output, source[recovered],
			   output_count * DVBS2X_TS_PACKET_SIZE) != 0)
			goto out;
		recovered += output_count;
		frame_count++;
	}
	if (frame_count != TEST_FRAMES || recovered == 0)
		goto out;
	ret = 0;
out:
	dvbs2x_ts_tx_destroy(&tx);
	free(frame);
	return ret;
}

static int test_parameters(void)
{
	const struct dvbs2x_modcod *mc = dvbs2x_vlsnr_get_modcod(9);
	struct dvbs2x_ts_tx tx = { 0 };
	struct dvbs2x_ts_rx rx;
	uint8_t packet[DVBS2X_TS_PACKET_SIZE];
	uint8_t output[DVBS2X_TS_PACKET_SIZE];
	uint8_t *frame;
	unsigned int len = 1;
	int ret = -1;

	if (!mc)
		return -1;
	frame = malloc(mc->k_bch);
	if (!frame)
		return -1;
	make_packet(packet, 0);
	if (dvbs2x_ts_tx_init(&tx, mc, DVBS2X_RO_0_35) < 0 ||
	    dvbs2x_ts_rx_init(&rx, mc) < 0)
		goto out;
	if (dvbs2x_ts_tx_push(&tx, packet, frame, 0, &len) !=
	    DVBS2X_ERR_SHORT || len != mc->k_bch)
		goto out;
	packet[0] = 0;
	if (dvbs2x_ts_tx_push(&tx, packet, frame, mc->k_bch, &len) !=
	    DVBS2X_ERR_PARAM || len != 0)
		goto out;
	if (dvbs2x_ts_rx_push(NULL, frame, output, 1, &len) !=
	    DVBS2X_ERR_PARAM || len != 0)
		goto out;
	ret = 0;
out:
	dvbs2x_ts_tx_destroy(&tx);
	free(frame);
	return ret;
}

static int test_up_crc(void)
{
	const struct dvbs2x_modcod *mc = dvbs2x_vlsnr_get_modcod(9);
	struct dvbs2x_ts_tx tx = { 0 };
	struct dvbs2x_ts_rx rx;
	uint8_t packet[DVBS2X_TS_PACKET_SIZE];
	uint8_t output[16 * DVBS2X_TS_PACKET_SIZE];
	uint8_t *frame;
	unsigned int frame_len = 0, packet_count, index;
	int ret = -1;

	if (!mc)
		return -1;
	frame = malloc(mc->k_bch);
	if (!frame)
		return -1;
	if (dvbs2x_ts_tx_init(&tx, mc, DVBS2X_RO_0_35) < 0 ||
	    dvbs2x_ts_rx_init(&rx, mc) < 0)
		goto out;
	for (index = 0; !frame_len; index++) {
		make_packet(packet, index);
		if (dvbs2x_ts_tx_push(&tx, packet, frame, mc->k_bch,
				      &frame_len) < 0)
			goto out;
	}
	frame[80 + 16] ^= 1;
	if (dvbs2x_ts_rx_push(&rx, frame, output, 16, &packet_count) !=
	    DVBS2X_ERR_CRC || packet_count != 0)
		goto out;
	ret = 0;
out:
	dvbs2x_ts_tx_destroy(&tx);
	free(frame);
	return ret;
}

int main(void)
{
	unsigned int i;

	printf("DVB-S2X VL-SNR MPEG-TS Tests\n");
	printf("=============================\n");
	for (i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
		printf("  MODCOD %u... ", vectors[i].modcod);
		if (test_vector(&vectors[i]) < 0) {
			printf("FAIL\n");
			return 1;
		}
		printf("PASS\n");
	}
	if (test_parameters() < 0) {
		printf("  parameters... FAIL\n");
		return 1;
	}
	printf("  parameters... PASS\n");
	if (test_up_crc() < 0) {
		printf("  UP CRC-8... FAIL\n");
		return 1;
	}
	printf("  UP CRC-8... PASS\n");
	return 0;
}
