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
#define DVBS2X_STREAM_TS	0	/* Transport Stream */
#define DVBS2X_STREAM_GS	1	/* Generic Stream */

/* BB frame context */
struct dvbs2x_bb_frame_ctx {
	unsigned int	k_bch;		/* total BCH input capacity (bits) */
	unsigned int	xs;		/* shortening zero-prefix length */
	unsigned int	stream_type;
	uint16_t	upl;		/* user packet length */
	uint16_t	dfl;		/* data field length */
};

/*
 * dvbs2x_bb_frame_init - Initialize BB frame context
 * @ctx: BB frame context
 * @modcod: MODCOD parameters
 * @stream_type: DVBS2X_STREAM_TS or DVBS2X_STREAM_GS
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
 * Adds BB header, user data, padding, and applies BB scrambling.
 * Returns 0 on success, -1 on error.
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

/*
 * dvbs2x_bb_crc8 - Compute CRC-8 for BB header
 * @data: header bytes (first 9 bytes)
 * @len: number of bytes
 *
 * Polynomial: x^8 + x^7 + x^6 + x^4 + x^2 + 1 (0xD5)
 */
uint8_t dvbs2x_bb_crc8(const uint8_t *data, unsigned int len);

#endif /* DVBS2X_BB_FRAME_H */
