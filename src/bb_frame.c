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
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define BB_HEADER_BITS	80
#define BB_PRBS_INIT	0x4A80
#define BB_CRC8_POLY	0xD5

#define TS_MAX_PACKETS_PER_FRAME \
	(DVBS2X_LDPC_NORMAL / DVBS2X_TS_PACKET_BITS + 1)

uint8_t dvbs2x_ro_from_rolloff(double rolloff)
{
	if (rolloff <= 0.225)
		return DVBS2X_RO_0_20;
	if (rolloff <= 0.30)
		return DVBS2X_RO_0_25;
	return DVBS2X_RO_0_35;
}

void dvbs2x_bb_frame_init(struct dvbs2x_bb_frame_ctx *ctx,
			  const struct dvbs2x_modcod *modcod,
			  unsigned int stream_type)
{
	ctx->k_bch = modcod->k_bch;
	ctx->stream_type = stream_type;
	ctx->ro = DVBS2X_RO_0_35;	/* default; TX overrides per rolloff */

	ctx->upl = 0;
	ctx->dfl = ctx->k_bch - BB_HEADER_BITS;
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
 * Apply BB scrambling (PRBS x^15 + x^14 + 1), matching the gr-dtv
 * dvb_bbscrambler init_bb_randomiser() generator bit-for-bit:
 *   sr = 0x4A80; b = (sr ^ (sr >> 1)) & 1; sr >>= 1; if (b) sr |= 0x4000;
 */
static void bb_scramble(uint8_t *frame, unsigned int len)
{
	uint16_t sr = BB_PRBS_INIT;
	unsigned int i;
	uint8_t b;

	for (i = 0; i < len; i++) {
		b = (sr ^ (sr >> 1)) & 1;
		frame[i] ^= b;
		sr >>= 1;
		if (b)
			sr |= 0x4000;
	}
}

static void bb_header_write(const struct dvbs2x_bb_frame_ctx *ctx,
			    unsigned int dfl, uint16_t upl, uint8_t sync,
			    uint16_t syncd, uint8_t matype1,
			    uint8_t *bbframe)
{
	uint8_t header[DVBS2X_BB_HEADER_LEN];
	unsigned int i, bit_idx = 0;

	header[0] = matype1 | (ctx->ro & 0x03);
	header[1] = 0;
	header[2] = (upl >> 8) & 0xff;
	header[3] = upl & 0xff;
	header[4] = (dfl >> 8) & 0xff;
	header[5] = dfl & 0xff;
	header[6] = sync;
	header[7] = (syncd >> 8) & 0xff;
	header[8] = syncd & 0xff;
	header[9] = dvbs2x_bb_crc8(header, 9);

	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		for (b = 0; b < 8; b++)
			bbframe[bit_idx++] = (header[i] >> (7 - b)) & 1;
	}
}

static int bb_header_read(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *bbframe, uint8_t *descrambled,
			  struct dvbs2x_bb_header *header)
{
	uint8_t bytes[DVBS2X_BB_HEADER_LEN];
	unsigned int i;

	if (ctx->k_bch > DVBS2X_LDPC_NORMAL)
		return DVBS2X_ERR_PARAM;
	memcpy(descrambled, bbframe, ctx->k_bch);
	bb_scramble(descrambled, ctx->k_bch);

	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		bytes[i] = 0;
		for (b = 0; b < 8; b++)
			bytes[i] |= descrambled[i * 8 + b] << (7 - b);
	}
	if (dvbs2x_bb_crc8(bytes, 9) != bytes[9])
		return DVBS2X_ERR_CRC;

	header->matype1 = bytes[0];
	header->matype2 = bytes[1];
	header->upl = ((uint16_t)bytes[2] << 8) | bytes[3];
	header->dfl = ((uint16_t)bytes[4] << 8) | bytes[5];
	header->sync = bytes[6];
	header->syncd = ((uint16_t)bytes[7] << 8) | bytes[8];
	header->crc8 = bytes[9];
	return 0;
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

	if (!ctx || !user_data || !bbframe)
		return DVBS2X_ERR_PARAM;
	if (ctx->stream_type != DVBS2X_STREAM_GS || user_len > ctx->dfl)
		return DVBS2X_ERR_PARAM;
	dfl = user_len;

	/*
	 * Build MATYPE-1: [TS/GS][SIS/MIS][CCM/ACM][ISSYI][NPD][RO:2].
	 * TS,SIS,CCM,ISSYI=0,NPD=0 -> 0xF0; GS -> 0x70; low 2 bits = RO.
	 */
	matype1 = 0x70;
	matype1 |= ctx->ro & 0x03;

	/* Assemble header bytes */
	header_bytes[0] = matype1;
	header_bytes[1] = 0x00;		/* MATYPE-2: input stream id */
	header_bytes[2] = (ctx->upl >> 8) & 0xFF;
	header_bytes[3] = ctx->upl & 0xFF;
	header_bytes[4] = (dfl >> 8) & 0xFF;
	header_bytes[5] = dfl & 0xFF;
	header_bytes[6] = 0x00;		/* not applicable to continuous GS */
	header_bytes[7] = 0x00;		/* SYNCD high */
	header_bytes[8] = 0x00;		/* SYNCD low */
	header_bytes[9] = dvbs2x_bb_crc8(header_bytes, 9);

	/* Convert header to bits at the start of the BB frame */
	bit_idx = 0;
	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		for (b = 0; b < 8; b++)
			bbframe[bit_idx++] = (header_bytes[i] >> (7 - b)) & 1;
	}

	/* Copy user data */
	for (i = 0; i < user_len; i++)
		bbframe[bit_idx + i] = user_data[i];

	/* Zero padding */
	for (i = user_len; i < ctx->dfl; i++)
		bbframe[bit_idx + i] = 0;

	/* Scramble the whole BB frame (header + data field) */
	bb_scramble(bbframe, ctx->k_bch);

	return 0;
}

int dvbs2x_ts_tx_init(struct dvbs2x_ts_tx *tx,
		      const struct dvbs2x_modcod *modcod, uint8_t ro)
{
	if (!tx || !modcod || ro > DVBS2X_RO_0_20)
		return DVBS2X_ERR_PARAM;
	memset(tx, 0, sizeof(*tx));
	dvbs2x_bb_frame_init(&tx->bb, modcod, DVBS2X_STREAM_TS);
	tx->bb.upl = DVBS2X_TS_PACKET_BITS;
	tx->bb.ro = ro;
	tx->data_bits = malloc(tx->bb.dfl);
	if (!tx->data_bits)
		return DVBS2X_ERR_NOMEM;
	return 0;
}

void dvbs2x_ts_tx_destroy(struct dvbs2x_ts_tx *tx)
{
	if (!tx)
		return;
	free(tx->data_bits);
	tx->data_bits = NULL;
	tx->data_len = 0;
}

int dvbs2x_ts_tx_push(struct dvbs2x_ts_tx *tx, const uint8_t *packet,
		      uint8_t *bbframe, unsigned int frame_capacity,
		      unsigned int *frame_len)
{
	unsigned int i;

	if (!frame_len)
		return DVBS2X_ERR_PARAM;
	*frame_len = 0;
	if (!tx || !tx->data_bits || !packet || !bbframe ||
	    packet[0] != 0x47)
		return DVBS2X_ERR_PARAM;
	if (frame_capacity < tx->bb.k_bch) {
		*frame_len = tx->bb.k_bch;
		return DVBS2X_ERR_SHORT;
	}

	for (i = 0; i < DVBS2X_TS_PACKET_BITS; i++) {
		uint8_t byte;

		byte = i < 8 ? tx->previous_crc : packet[i / 8];
		tx->data_bits[tx->data_len++] = (byte >> (7 - i % 8)) & 1;
		if (tx->data_len != tx->bb.dfl)
			continue;

		bb_header_write(&tx->bb, tx->bb.dfl,
				DVBS2X_TS_PACKET_BITS, 0x47, tx->syncd,
				0xf0, bbframe);
		memcpy(bbframe + BB_HEADER_BITS, tx->data_bits,
		       tx->bb.dfl);
		bb_scramble(bbframe, tx->bb.k_bch);
		*frame_len = tx->bb.k_bch;
		tx->data_len = 0;
		tx->syncd = i + 1 == DVBS2X_TS_PACKET_BITS ? 0 :
			DVBS2X_TS_PACKET_BITS - i - 1;
	}
	tx->previous_crc = dvbs2x_bb_crc8(packet + 1,
					  DVBS2X_TS_PACKET_SIZE - 1);
	return 0;
}

int dvbs2x_ts_rx_init(struct dvbs2x_ts_rx *rx,
		      const struct dvbs2x_modcod *modcod)
{
	if (!rx || !modcod)
		return DVBS2X_ERR_PARAM;
	memset(rx, 0, sizeof(*rx));
	dvbs2x_bb_frame_init(&rx->bb, modcod, DVBS2X_STREAM_TS);
	rx->bb.upl = DVBS2X_TS_PACKET_BITS;
	return 0;
}

void dvbs2x_ts_rx_reset(struct dvbs2x_ts_rx *rx)
{
	if (!rx)
		return;
	rx->packet_bits = 0;
	rx->have_previous = 0;
}

int dvbs2x_ts_rx_push(struct dvbs2x_ts_rx *rx, const uint8_t *bbframe,
		      uint8_t *packets, unsigned int packet_capacity,
		      unsigned int *packet_count)
{
	uint8_t descrambled[DVBS2X_LDPC_NORMAL];
	uint8_t output[TS_MAX_PACKETS_PER_FRAME * DVBS2X_TS_PACKET_SIZE];
	uint8_t packet[DVBS2X_TS_PACKET_SIZE];
	uint8_t previous[DVBS2X_TS_PACKET_SIZE];
	struct dvbs2x_bb_header header;
	unsigned int packet_bits, complete, needed, produced = 0;
	unsigned int start = 0;
	unsigned int i;
	int have_previous;
	int ret;

	if (!packet_count)
		return DVBS2X_ERR_PARAM;
	*packet_count = 0;
	if (!rx || !bbframe || !packets)
		return DVBS2X_ERR_PARAM;
	ret = bb_header_read(&rx->bb, bbframe, descrambled, &header);
	if (ret < 0)
		return ret;
	if ((header.matype1 & 0xc0) != 0xc0 || header.matype2 != 0 ||
	    header.upl != DVBS2X_TS_PACKET_BITS || header.sync != 0x47 ||
	    header.dfl > rx->bb.dfl)
		return DVBS2X_ERR_PARAM;
	if (!rx->packet_bits && !rx->have_previous) {
		if (header.syncd > header.dfl)
			return DVBS2X_ERR_NOSYNC;
		start = header.syncd;
	} else if (header.syncd != (rx->packet_bits ?
		   DVBS2X_TS_PACKET_BITS - rx->packet_bits : 0)) {
		return DVBS2X_ERR_NOSYNC;
	}

	complete = (rx->packet_bits + header.dfl - start) /
		DVBS2X_TS_PACKET_BITS;
	needed = complete;
	if (!rx->have_previous && needed)
		needed--;
	if (packet_capacity < needed) {
		*packet_count = needed;
		return DVBS2X_ERR_SHORT;
	}

	memcpy(packet, rx->packet, sizeof(packet));
	memcpy(previous, rx->previous, sizeof(previous));
	packet_bits = rx->packet_bits;
	have_previous = rx->have_previous;
	for (i = start; i < header.dfl; i++) {
		unsigned int byte = packet_bits / 8;
		unsigned int bit = packet_bits % 8;

		if (!bit)
			packet[byte] = 0;
		packet[byte] |= descrambled[BB_HEADER_BITS + i] << (7 - bit);
		packet_bits++;
		if (packet_bits != DVBS2X_TS_PACKET_BITS)
			continue;

		{
			uint8_t marker = packet[0];

			packet[0] = 0x47;
			if (have_previous) {
				uint8_t crc;

				crc = dvbs2x_bb_crc8(previous + 1,
						     DVBS2X_TS_PACKET_SIZE - 1);
				if (crc != marker)
					return DVBS2X_ERR_CRC;
				memcpy(output + produced * DVBS2X_TS_PACKET_SIZE,
				       previous, DVBS2X_TS_PACKET_SIZE);
				produced++;
			}
			memcpy(previous, packet, sizeof(previous));
			have_previous = 1;
			packet_bits = 0;
		}
	}

	memcpy(rx->packet, packet, sizeof(packet));
	memcpy(rx->previous, previous, sizeof(previous));
	rx->packet_bits = packet_bits;
	rx->have_previous = have_previous;
	memcpy(packets, output, produced * DVBS2X_TS_PACKET_SIZE);
	*packet_count = produced;
	return 0;
}

int dvbs2x_ts_rx_finalize_unchecked(struct dvbs2x_ts_rx *rx,
				    uint8_t *packet,
				    unsigned int packet_capacity,
				    unsigned int *packet_count)
{
	if (!packet_count)
		return DVBS2X_ERR_PARAM;
	*packet_count = 0;
	if (!rx || !packet)
		return DVBS2X_ERR_PARAM;
	if (rx->have_previous && !packet_capacity) {
		*packet_count = 1;
		return DVBS2X_ERR_SHORT;
	}
	if (rx->have_previous) {
		memcpy(packet, rx->previous, DVBS2X_TS_PACKET_SIZE);
		*packet_count = 1;
	}
	dvbs2x_ts_rx_reset(rx);
	return 0;
}

int dvbs2x_bb_frame_parse_ex(const struct dvbs2x_bb_frame_ctx *ctx,
			     const uint8_t *bbframe,
			     uint8_t *user_data,
			     unsigned int user_capacity,
			     unsigned int *user_len)
{
	uint8_t descrambled[DVBS2X_LDPC_NORMAL];
	uint8_t header_bytes[DVBS2X_BB_HEADER_LEN];
	unsigned int i, bit_idx;
	uint8_t crc;
	uint16_t dfl;

	if (!user_len)
		return DVBS2X_ERR_PARAM;
	*user_len = 0;
	if (!ctx || !bbframe || !user_data ||
	    ctx->k_bch > DVBS2X_LDPC_NORMAL)
		return DVBS2X_ERR_PARAM;

	/* Copy and descramble the BB frame */
	for (i = 0; i < ctx->k_bch; i++)
		descrambled[i] = bbframe[i];
	bb_scramble(descrambled, ctx->k_bch);

	/* Extract header bytes from the start of the frame */
	bit_idx = 0;
	for (i = 0; i < DVBS2X_BB_HEADER_LEN; i++) {
		unsigned int b;

		header_bytes[i] = 0;
		for (b = 0; b < 8; b++)
			header_bytes[i] |=
				descrambled[bit_idx + i * 8 + b] << (7 - b);
	}

	/* Verify CRC-8 */
	crc = dvbs2x_bb_crc8(header_bytes, 9);
	if (crc != header_bytes[9])
		return -1;
	if ((header_bytes[0] & 0xc0) != 0x40 ||
	    header_bytes[2] != 0 || header_bytes[3] != 0)
		return DVBS2X_ERR_PARAM;

	/* Extract DFL */
	dfl = ((uint16_t)header_bytes[4] << 8) | header_bytes[5];
	if (dfl > ctx->k_bch - BB_HEADER_BITS)
		dfl = ctx->k_bch - BB_HEADER_BITS;
	*user_len = dfl;
	if (user_capacity < dfl)
		return DVBS2X_ERR_SHORT;

	/* Extract user data */
	bit_idx = BB_HEADER_BITS;
	for (i = 0; i < dfl; i++)
		user_data[i] = descrambled[bit_idx + i];

	return 0;
}

int dvbs2x_bb_frame_parse(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *bbframe,
			  uint8_t *user_data,
			  unsigned int *user_len)
{
	return dvbs2x_bb_frame_parse_ex(ctx, bbframe, user_data, UINT_MAX,
					user_len);
}
