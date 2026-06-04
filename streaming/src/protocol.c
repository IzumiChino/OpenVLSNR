// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Protocol Implementation
 *
 * Frame encoding/decoding with CRC-32 integrity checking.
 */

#include "dvbs2x_protocol.h"
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>

int dvbs2x_frame_encode(void *buf, size_t buf_size,
			const void *payload, size_t payload_len,
			enum dvbs2x_frame_type frame_type,
			uint32_t sequence, uint16_t stream_id)
{
	struct dvbs2x_frame_header *hdr;
	uint8_t *p;
	uint32_t crc;
	size_t total_size;

	/* Validate parameters */
	if (!buf || !payload)
		return -EINVAL;
	if (payload_len > DVBS2X_FRAME_MAX_PAYLOAD)
		return -EINVAL;
	if (frame_type > DVBS2X_FRAME_BBFRAME)
		return -EINVAL;

	total_size = DVBS2X_FRAME_HEADER_SIZE + payload_len +
		     DVBS2X_FRAME_CRC_SIZE;
	if (buf_size < total_size)
		return -ENOSPC;

	/* Build header */
	hdr = buf;
	hdr->magic = htonl(DVBS2X_FRAME_MAGIC);
	hdr->sequence = htonl(sequence);
	hdr->payload_len = htons((uint16_t)payload_len);
	hdr->frame_type = frame_type;
	hdr->flags = 0;
	hdr->frag_offset = 0;
	hdr->stream_id = htons(stream_id);

	/* Copy payload */
	p = (uint8_t *)buf + DVBS2X_FRAME_HEADER_SIZE;
	memcpy(p, payload, payload_len);

	/* Compute CRC over header[4:15] + payload */
	crc = dvbs2x_crc32((uint8_t *)buf + 4,
			   DVBS2X_FRAME_HEADER_SIZE - 4 + payload_len);

	/* Append CRC */
	p = (uint8_t *)buf + DVBS2X_FRAME_HEADER_SIZE + payload_len;
	*(uint32_t *)p = htonl(crc);

	return (int)total_size;
}

int dvbs2x_frame_decode(const void *buf, size_t buf_len,
			void *payload, size_t payload_size,
			enum dvbs2x_frame_type *frame_type,
			uint32_t *sequence, uint16_t *stream_id)
{
	const struct dvbs2x_frame_header *hdr;
	const uint8_t *p;
	uint32_t magic, seq, crc_expected, crc_actual;
	uint16_t plen, sid;
	size_t total_size;

	/* Minimum frame size check */
	if (!buf || buf_len < DVBS2X_FRAME_HEADER_SIZE + DVBS2X_FRAME_CRC_SIZE)
		return -EINVAL;

	hdr = buf;

	/* Verify magic */
	magic = ntohl(hdr->magic);
	if (magic != DVBS2X_FRAME_MAGIC)
		return -EINVAL;

	/* Extract header fields */
	seq = ntohl(hdr->sequence);
	plen = ntohs(hdr->payload_len);
	sid = ntohs(hdr->stream_id);

	/* Validate payload length */
	if (plen > DVBS2X_FRAME_MAX_PAYLOAD)
		return -EINVAL;

	total_size = DVBS2X_FRAME_HEADER_SIZE + plen + DVBS2X_FRAME_CRC_SIZE;
	if (buf_len < total_size)
		return -EINVAL;

	/* Verify CRC */
	crc_expected = dvbs2x_crc32((const uint8_t *)buf + 4,
				    DVBS2X_FRAME_HEADER_SIZE - 4 + plen);
	p = (const uint8_t *)buf + DVBS2X_FRAME_HEADER_SIZE + plen;
	crc_actual = ntohl(*(const uint32_t *)p);

	if (crc_expected != crc_actual)
		return -EBADMSG;

	/* Extract payload if buffer provided */
	if (payload) {
		if (payload_size < plen)
			return -ENOSPC;
		p = (const uint8_t *)buf + DVBS2X_FRAME_HEADER_SIZE;
		memcpy(payload, p, plen);
	}

	/* Return metadata */
	if (frame_type)
		*frame_type = hdr->frame_type;
	if (sequence)
		*sequence = seq;
	if (stream_id)
		*stream_id = sid;

	return (int)plen;
}
