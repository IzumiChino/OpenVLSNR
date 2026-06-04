// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Protocol Layer
 *
 * Frame format for network streaming:
 *  - Fixed 16-byte header with magic, sequence, payload length
 *  - CRC-32 (IEEE 802.3) for integrity checking
 *  - Support for fragmentation (large frames)
 *  - Network byte order (big-endian)
 */

#ifndef DVBS2X_PROTOCOL_H
#define DVBS2X_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/*
 * Protocol constants
 */
#define DVBS2X_FRAME_MAGIC		0x53325856U  /* "XVS2" big-endian */
#define DVBS2X_FRAME_HEADER_SIZE	16
#define DVBS2X_FRAME_CRC_SIZE		4
#define DVBS2X_FRAME_MAX_PAYLOAD	65520

/*
 * Frame types
 */
enum dvbs2x_frame_type {
	DVBS2X_FRAME_IQ_SAMPLES		= 0,
	DVBS2X_FRAME_TS_PACKETS		= 1,
	DVBS2X_FRAME_BBFRAME		= 2,
};

/*
 * Frame flags
 */
#define DVBS2X_FRAME_MORE_FRAGS		(1 << 0)

/*
 * Wire frame header (16 bytes, network byte order)
 */
struct dvbs2x_frame_header {
	uint32_t	magic;		/* 0x53325856 */
	uint32_t	sequence;	/* Sequence number */
	uint16_t	payload_len;	/* Payload length */
	uint8_t		frame_type;	/* Frame type */
	uint8_t		flags;		/* Flags + frag_offset[21:16] */
	uint16_t	frag_offset;	/* Fragment offset (8-byte units) */
	uint16_t	stream_id;	/* Stream multiplex ID */
} __attribute__((packed));

/*
 * dvbs2x_crc32 - Compute CRC-32 (IEEE 802.3)
 * @data: data buffer
 * @len: buffer length
 *
 * Returns CRC-32 value.
 */
uint32_t dvbs2x_crc32(const void *data, size_t len);

/*
 * dvbs2x_frame_encode - Encode frame to wire format
 * @buf: output buffer (must hold header + payload + CRC)
 * @buf_size: buffer size
 * @payload: payload data
 * @payload_len: payload length
 * @frame_type: frame type
 * @sequence: sequence number
 * @stream_id: stream ID
 *
 * Returns total frame size (header + payload + CRC), or negative errno.
 */
int dvbs2x_frame_encode(void *buf, size_t buf_size,
			const void *payload, size_t payload_len,
			enum dvbs2x_frame_type frame_type,
			uint32_t sequence, uint16_t stream_id);

/*
 * dvbs2x_frame_decode - Decode frame from wire format
 * @buf: input buffer
 * @buf_len: buffer length
 * @payload: output payload buffer (may be NULL to check only)
 * @payload_size: payload buffer size
 * @frame_type: frame type (output)
 * @sequence: sequence number (output)
 * @stream_id: stream ID (output)
 *
 * Returns payload length on success, or negative errno.
 * -EINVAL: invalid header
 * -EBADMSG: CRC mismatch
 * -ENOSPC: payload buffer too small
 */
int dvbs2x_frame_decode(const void *buf, size_t buf_len,
			void *payload, size_t payload_size,
			enum dvbs2x_frame_type *frame_type,
			uint32_t *sequence, uint16_t *stream_id);

/*
 * dvbs2x_seq_compare - Compare sequence numbers with wrap handling
 * @a: first sequence number
 * @b: second sequence number
 *
 * Returns:
 *  > 0 if a is after b (accounting for wrap)
 *  < 0 if a is before b (accounting for wrap)
 *  = 0 if a equals b
 *
 * This function correctly handles 32-bit sequence number wrap-around
 * (0xFFFFFFFF -> 0x00000000). It assumes sequence numbers are within
 * 2^31 of each other (no more than ~2 billion frames apart).
 */
static inline int dvbs2x_seq_compare(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b);
}

/*
 * dvbs2x_seq_after - Test if sequence a is after b
 * @a: first sequence number
 * @b: second sequence number
 *
 * Returns true if a is after b (accounting for wrap).
 */
static inline int dvbs2x_seq_after(uint32_t a, uint32_t b)
{
	return dvbs2x_seq_compare(a, b) > 0;
}

/*
 * dvbs2x_seq_before - Test if sequence a is before b
 * @a: first sequence number
 * @b: second sequence number
 *
 * Returns true if a is before b (accounting for wrap).
 */
static inline int dvbs2x_seq_before(uint32_t a, uint32_t b)
{
	return dvbs2x_seq_compare(a, b) < 0;
}

#ifdef __cplusplus
}
#endif

#endif /* DVBS2X_PROTOCOL_H */
