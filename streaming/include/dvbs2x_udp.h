// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR UDP Transport
 *
 * Connectionless unreliable transport for low-latency real-time streaming.
 * Suitable for satellite links where application-layer FEC (DVB-S2X LDPC)
 * already provides error correction.
 */

#ifndef DVBS2X_UDP_H
#define DVBS2X_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dvbs2x_platform.h"
#include <stdint.h>
#include <stddef.h>

/*
 * UDP context
 *
 * Thread safety: Each context must be accessed by a single thread only.
 * Concurrent access to the same context from multiple threads will result
 * in undefined behavior. Statistics counters (tx_packets, rx_packets,
 * tx_bytes, rx_bytes) are NOT atomic and will be corrupted under
 * concurrent access.
 *
 * For multi-threaded use, create separate contexts per thread or provide
 * external synchronization.
 */
struct dvbs2x_udp_ctx {
	dvbs2x_socket_t		sock;
	struct sockaddr_in	peer_addr;
	int			peer_set;
	uint64_t		tx_packets;
	uint64_t		rx_packets;
	uint64_t		tx_bytes;
	uint64_t		rx_bytes;
};

/*
 * dvbs2x_udp_init - Initialize UDP socket
 * @ctx: UDP context
 * @port: local port to bind (0 = any available port)
 *
 * Returns 0 on success, negative errno on failure.
 */
int dvbs2x_udp_init(struct dvbs2x_udp_ctx *ctx, uint16_t port);

/*
 * dvbs2x_udp_set_peer - Set default peer address
 * @ctx: UDP context
 * @host: peer hostname or IP address
 * @port: peer port
 *
 * Returns 0 on success, negative errno on failure.
 */
int dvbs2x_udp_set_peer(struct dvbs2x_udp_ctx *ctx,
			const char *host, uint16_t port);

/*
 * dvbs2x_udp_send - Send datagram
 * @ctx: UDP context
 * @buf: data buffer
 * @len: buffer length
 *
 * Sends to previously set peer address.
 * Returns number of bytes sent, or negative errno.
 */
int dvbs2x_udp_send(struct dvbs2x_udp_ctx *ctx, const void *buf, size_t len);

/*
 * dvbs2x_udp_recv - Receive datagram
 * @ctx: UDP context
 * @buf: receive buffer
 * @len: buffer size
 * @timeout_ms: timeout in milliseconds (0 = block forever)
 *
 * Returns number of bytes received, or negative errno.
 */
int dvbs2x_udp_recv(struct dvbs2x_udp_ctx *ctx, void *buf, size_t len,
		    int timeout_ms);

/*
 * dvbs2x_udp_close - Close UDP socket
 * @ctx: UDP context
 */
void dvbs2x_udp_close(struct dvbs2x_udp_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DVBS2X_UDP_H */
