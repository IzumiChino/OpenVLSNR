// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR TCP Transport
 *
 * Connection-oriented reliable transport for IQ samples and decoded streams.
 * Provides server (listen/accept) and client (connect) modes with timeout
 * support and statistics tracking.
 */

#ifndef DVBS2X_TCP_H
#define DVBS2X_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dvbs2x_platform.h"
#include <stdint.h>
#include <stddef.h>

/*
 * TCP connection context
 *
 * Thread safety: Each context must be accessed by a single thread only.
 * Concurrent access to the same context from multiple threads will result
 * in undefined behavior. Statistics counters (tx_bytes, rx_bytes) are NOT
 * atomic and will be corrupted under concurrent access.
 *
 * For multi-threaded use, create separate contexts per thread or provide
 * external synchronization.
 */
struct dvbs2x_tcp_ctx {
	dvbs2x_socket_t		sock;
	int			connected;
	uint64_t		tx_bytes;
	uint64_t		rx_bytes;
};

/**
 * dvbs2x_tcp_server_init() - Initialize TCP server
 * @ctx: TCP context pointer
 * @port: Port to bind (0 = any available port)
 *
 * Initializes a TCP server socket and binds to the specified port.
 * Sets SO_REUSEADDR and TCP_NODELAY for low latency.
 *
 * Context: Process context. May sleep.
 * Return: 0 on success, negative errno on failure.
 */
int dvbs2x_tcp_server_init(struct dvbs2x_tcp_ctx *ctx, uint16_t port);

/**
 * dvbs2x_tcp_server_accept() - Accept incoming connection
 * @server: Server context pointer
 * @client: Client context pointer (output)
 * @timeout_ms: Timeout in milliseconds (0 = block forever)
 *
 * Accepts an incoming TCP connection with optional timeout.
 * The accepted connection inherits TCP_NODELAY from the server.
 *
 * Context: Process context. May sleep.
 * Return: 0 on success, -ETIMEDOUT on timeout, negative errno on error.
 */
int dvbs2x_tcp_server_accept(struct dvbs2x_tcp_ctx *server,
			      struct dvbs2x_tcp_ctx *client,
			      int timeout_ms);

/**
 * dvbs2x_tcp_client_connect() - Connect to TCP server
 * @ctx: TCP context pointer
 * @host: Hostname or IP address
 * @port: Port number
 * @timeout_ms: Connection timeout in milliseconds
 *
 * Establishes a TCP connection to the specified server with timeout.
 * Automatically retries on EINTR and sets TCP_NODELAY for low latency.
 *
 * Context: Process context. May sleep.
 * Return: 0 on success, negative errno on failure.
 */
int dvbs2x_tcp_client_connect(struct dvbs2x_tcp_ctx *ctx,
			       const char *host, uint16_t port,
			       int timeout_ms);

/**
 * dvbs2x_tcp_send() - Send data over TCP
 * @ctx: TCP context pointer
 * @buf: Data buffer to send
 * @len: Length of data in bytes
 *
 * Sends data over TCP connection. Loops until all bytes are sent or
 * an error occurs. Handles partial sends and EINTR correctly.
 *
 * Context: Process context. May sleep.
 * Return: Number of bytes sent (always @len) on success, negative errno
 *         on error, -EPIPE if connection closed.
 */
int dvbs2x_tcp_send(struct dvbs2x_tcp_ctx *ctx, const void *buf, size_t len);

/**
 * dvbs2x_tcp_recv() - Receive data from TCP
 * @ctx: TCP context pointer
 * @buf: Receive buffer
 * @len: Buffer size in bytes
 * @timeout_ms: Receive timeout in milliseconds (0 = block forever)
 *
 * Receives data from TCP connection. Loops until exactly @len bytes are
 * received or connection closes. Handles partial receives and EINTR.
 *
 * Context: Process context. May sleep.
 * Return: Number of bytes received (@len on complete), 0 on connection close,
 *         negative errno on error, -ETIMEDOUT on timeout.
 */
int dvbs2x_tcp_recv(struct dvbs2x_tcp_ctx *ctx, void *buf, size_t len,
		    int timeout_ms);

/**
 * dvbs2x_tcp_close() - Close TCP connection
 * @ctx: TCP context pointer
 *
 * Performs proper shutdown sequence (shutdown + close) to avoid data loss.
 * Safe to call multiple times.
 *
 * Context: Process context.
 * Return: None.
 */
void dvbs2x_tcp_close(struct dvbs2x_tcp_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DVBS2X_TCP_H */
