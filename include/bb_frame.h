/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Baseband Frame
 *
 * BB frame assembly and disassembly including:
 * - BB header (MATYPE, UPL, DFL, SYNC, SYNCD, CRC-8)
 * - Data field with optional padding
 * - BB scrambling (PRBS: x^15 + x^14 + 1)
 *
 * Reference: ETSI EN 302 307-1 clause 5.1, 5.2
 */

#ifndef DVBS2X_BB_FRAME_H
#define DVBS2X_BB_FRAME_H

#include "dvbs2x_types.h"

/* BB scrambler PRBS polynomial: x^15 + x^14 + 1 */
#define DVBS2X_BB_PRBS_POLY	0x6000

/* Stream types */
#define DVBS2X_STREAM_TS	0	/* MPEG transport stream */
#define DVBS2X_STREAM_GS	1	/* Generic continuous bit stream */

#define DVBS2X_TS_PACKET_SIZE	188
#define DVBS2X_TS_PACKET_BITS	(DVBS2X_TS_PACKET_SIZE * 8)

/* MATYPE-1 roll-off field (bits 1:0) per ETSI EN 302 307-1 Table 2 */
#define DVBS2X_RO_0_35	0
#define DVBS2X_RO_0_25	1
#define DVBS2X_RO_0_20	2

/* BB frame context */
struct dvbs2x_bb_frame_ctx {
	unsigned int	k_bch;		/* BCH input capacity = BB frame bits */
	unsigned int	stream_type;
	uint16_t	upl;		/* user packet length */
	uint16_t	dfl;		/* data field length */
	uint8_t		ro;		/* MATYPE-1 roll-off code (TX) */
};

/* Stateful MPEG-TS mode-adaptation transmitter. */
struct dvbs2x_ts_tx {
	struct dvbs2x_bb_frame_ctx bb;
	uint8_t		*data_bits;
	unsigned int	data_len;
	uint16_t	syncd;
	uint8_t		previous_crc;
};

/* Stateful MPEG-TS mode-adaptation receiver. */
struct dvbs2x_ts_rx {
	struct dvbs2x_bb_frame_ctx bb;
	uint8_t		packet[DVBS2X_TS_PACKET_SIZE];
	uint8_t		previous[DVBS2X_TS_PACKET_SIZE];
	unsigned int	packet_bits;
	int		have_previous;
};

/*
 * dvbs2x_ro_from_rolloff - Map a roll-off factor to its MATYPE-1 RO code
 */
uint8_t dvbs2x_ro_from_rolloff(double rolloff);

/*
 * dvbs2x_bb_frame_init - Initialize BB frame context
 * @ctx: BB frame context
 * @modcod: MODCOD parameters
 * @stream_type: stream type recorded in the context
 *
 * Use dvbs2x_ts_tx_init() for raw MPEG-TS packet mode adaptation.
 */
void dvbs2x_bb_frame_init(struct dvbs2x_bb_frame_ctx *ctx,
			  const struct dvbs2x_modcod *modcod,
			  unsigned int stream_type);

/*
 * dvbs2x_bb_frame_build - Assemble a BB frame
 * @ctx: BB frame context
 * @user_data: input user data bits
 * @user_len: length of user data in bits
 * @bbframe: output BB frame (k_bch bits)
 *
 * The input is an already adapted generic continuous bit stream.  DFL is
 * the number of input bits; unused BBFRAME capacity is padding.  This API
 * does not accept raw 188-byte MPEG-TS packets.
 */
int dvbs2x_bb_frame_build(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *user_data,
			  unsigned int user_len,
			  uint8_t *bbframe);

/*
 * dvbs2x_bb_frame_parse - Parse a received BB frame
 * @ctx: BB frame context
 * @bbframe: received BB frame (k_bch bits, after FEC decode)
 * @user_data: output user data bits
 * @user_len: output user data length in bits
 *
 * Removes BB scrambling, verifies CRC-8, extracts user data.
 * Returns 0 on success, -1 on CRC failure.
 */
int dvbs2x_bb_frame_parse(const struct dvbs2x_bb_frame_ctx *ctx,
			  const uint8_t *bbframe,
			  uint8_t *user_data,
			  unsigned int *user_len);

/* Capacity-checked variant of dvbs2x_bb_frame_parse(). */
int dvbs2x_bb_frame_parse_ex(const struct dvbs2x_bb_frame_ctx *ctx,
			     const uint8_t *bbframe,
			     uint8_t *user_data,
			     unsigned int user_capacity,
			     unsigned int *user_len);

/*
 * dvbs2x_bb_crc8 - Compute CRC-8 for BB header
 * @data: header bytes (first 9 bytes)
 * @len: number of bytes
 *
 * Polynomial: x^8 + x^7 + x^6 + x^4 + x^2 + 1 (0xD5)
 */
uint8_t dvbs2x_bb_crc8(const uint8_t *data, unsigned int len);

/* Initialize and release a continuous MPEG-TS transmitter. */
int dvbs2x_ts_tx_init(struct dvbs2x_ts_tx *tx,
		      const struct dvbs2x_modcod *modcod, uint8_t ro);
void dvbs2x_ts_tx_destroy(struct dvbs2x_ts_tx *tx);

/*
 * Consume one complete TS packet and emit at most one BBFRAME.  frame_len
 * is zero until enough packet bits have filled a data field.
 */
int dvbs2x_ts_tx_push(struct dvbs2x_ts_tx *tx, const uint8_t *packet,
		      uint8_t *bbframe, unsigned int frame_capacity,
		      unsigned int *frame_len);

/* Initialize or reset a continuous MPEG-TS receiver. */
int dvbs2x_ts_rx_init(struct dvbs2x_ts_rx *rx,
		      const struct dvbs2x_modcod *modcod);
void dvbs2x_ts_rx_reset(struct dvbs2x_ts_rx *rx);

/*
 * Consume one BBFRAME and emit CRC-verified TS packets.  packet_capacity
 * and packet_count are measured in complete 188-byte packets.
 */
int dvbs2x_ts_rx_push(struct dvbs2x_ts_rx *rx, const uint8_t *bbframe,
		      uint8_t *packets, unsigned int packet_capacity,
		      unsigned int *packet_count);

/*
 * End a finite TS stream and return its last complete packet.  The UP CRC
 * for a packet is carried by the following packet, so this explicit path
 * returns the held packet without CRC verification.
 */
int dvbs2x_ts_rx_finalize_unchecked(struct dvbs2x_ts_rx *rx,
				    uint8_t *packet,
				    unsigned int packet_capacity,
				    unsigned int *packet_count);

#endif /* DVBS2X_BB_FRAME_H */
