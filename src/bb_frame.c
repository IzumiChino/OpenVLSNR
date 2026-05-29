// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Baseband Frame Assembly
 *
 * BB frame structure (ETSI EN 302 307-1 clause 5.1):
 *   [BB Header (80 bits)] [Data Field (DFL bits)] [Padding]
 *
 * BB Header (10 bytes):
 *   MATYPE-1 (8 bits) | MATYPE-2 (8 bits) | UPL (16 bits) |
 *   DFL (16 bits) | SYNC (8 bits) | SYNCD (16 bits) | CRC-8 (8 bits)
 *
 * After assembly, the entire frame is scrambled with PRBS
 * (x^15 + x^14 + 1, init = 0x4A80).
 */

#include "bb_frame.h"
#include <string.h>

#define BB_HEADER_BITS	80
#define BB_PRBS_INIT	0x4A80
#define BB_CRC8_POLY	0xD5

void dvbs2x_bb_frame_init(struct dvbs2x_bb_frame_ctx *ctx,
			  const struct dvbs2x_modcod *modcod,
			  unsigned int stream_type)
{
	ctx->k_bch = modcod->k_bch;
	ctx->stream_type = stream_type;

	if (stream_type == DVBS2X_STREAM_TS) {
		ctx->upl = 188 * 8;	/* TS packet = 188 bytes */
		ctx->dfl = ctx->k_bch - BB_HEADER_BITS;
	} else {
		ctx->upl = 0;
		ctx->dfl = ctx->k_bch - BB_HEADER_BITS;
	}
}

uint8_t dvbs2x_bb_crc8(const uint8_t *data, unsigned int len)
{
	uint8_t crc = 0;
	unsigned int i, j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ BB_CRC8_POLY;
			else
				crc <<= 1;
		}
	}
	return crc;
}

/*
 * Apply BB scrambling (PRBS x^15 + x^14 + 1)
 * Scrambles bits starting after the BB header CRC.
 */
static void bb_scramble(uint8_t *frame, unsigned int len)
{
	uint16_t reg = BB_PRBS_INIT;
	unsigned int i;
	uint8_t prbs_bit;

	for (i = 0; i < len; i++) {
		prbs_bit = ((reg >> 14) ^ (reg >> 13)) & 1;
		frame[i] ^= prbs_bit;
		reg = (reg << 1) | prbs_bit;
		reg &= 0x7FFF;
	}
}

int dvbs2x_bb_frame_build(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *user_data,
			  unsigned int user_len,
			  uint8_t *bbframe)
{
	uint8_t header_bytes[DVBS2X_BB_HEADER_LEN];
	unsigned int dfl;
	unsigned int i, bit_idx;
	uint8_t matype1;

	dfl = ctx->dfl;
	if (user_len > dfl)
		user_len = dfl;

	/* Build MATYPE-1 */
	if (ctx->stream_type == DVBS2X_STREAM_TS)
		matype1 = 0xF0;	/* TS, SIS, CCM, ISSYI=0, NPD=0 */
	else
		matype1 = 0x70;	/* GS, SIS, CCM */

	/* Assemble header bytes */
	header_bytes[0] = matype1;
	header_bytes[1] = 0x00;		/* MATYPE-2: input stream id */
	header_bytes[2] = (ctx->upl >> 8) & 0xFF;
	header_bytes[3] = ctx->upl & 0xFF;
	header_bytes[4] = (dfl >> 8) & 0xFF;
	header_bytes[5] = dfl & 0xFF;
	header_bytes[6] = 0x47;		/* SYNC byte for TS */
	header_bytes[7] = 0x00;		/* SYNCD high */
	header_bytes[8] = 0x00;		/* SYNCD low */
	header_bytes[9] = dvbs2x_bb_crc8(header_bytes, 9);

	/* Convert header to bits */
	bit_idx = 0;
	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		for (b = 0; b < 8; b++) {
			bbframe[bit_idx++] = (header_bytes[i] >> (7 - b)) & 1;
		}
	}

	/* Copy user data */
	for (i = 0; i < user_len; i++)
		bbframe[bit_idx + i] = user_data[i];

	/* Zero padding */
	for (i = user_len; i < dfl; i++)
		bbframe[bit_idx + i] = 0;

	/* Apply BB scrambling to entire frame */
	bb_scramble(bbframe, ctx->k_bch);

	return 0;
}

int dvbs2x_bb_frame_parse(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *bbframe,
			  uint8_t *user_data,
			  unsigned int *user_len)
{
	uint8_t descrambled[DVBS2X_LDPC_NORMAL];
	uint8_t header_bytes[DVBS2X_BB_HEADER_LEN];
	unsigned int i, bit_idx;
	uint8_t crc;
	uint16_t dfl;

	/* Copy and descramble */
	for (i = 0; i < ctx->k_bch; i++)
		descrambled[i] = bbframe[i];
	bb_scramble(descrambled, ctx->k_bch);

	/* Extract header bytes from bits */
	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		header_bytes[i] = 0;
		for (b = 0; b < 8; b++)
			header_bytes[i] |= descrambled[i * 8 + b] << (7 - b);
	}

	/* Verify CRC-8 */
	crc = dvbs2x_bb_crc8(header_bytes, 9);
	if (crc != header_bytes[9])
		return -1;

	/* Extract DFL */
	dfl = ((uint16_t)header_bytes[4] << 8) | header_bytes[5];
	if (dfl > ctx->k_bch - BB_HEADER_BITS)
		dfl = ctx->k_bch - BB_HEADER_BITS;

	/* Extract user data */
	bit_idx = BB_HEADER_BITS;
	for (i = 0; i < dfl; i++)
		user_data[i] = descrambled[bit_idx + i];

	*user_len = dfl;
	return 0;
}
