// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR TCP Transport Implementation
 *
 * Fixed version with proper handling of:
 * - Partial send/recv (critical for real networks)
 * - EINTR signal interruption
 * - TCP_NODELAY for low latency
 * - Proper shutdown sequence
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "dvbs2x_tcp.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

int dvbs2x_tcp_server_init(struct dvbs2x_tcp_ctx *ctx, uint16_t port)
{
	struct sockaddr_in addr;
	int opt = 1;

	if (unlikely(!ctx))
		return -EINVAL;

	memset(ctx, 0, sizeof(*ctx));

	/* Create socket */
	ctx->sock = socket(AF_INET, SOCK_STREAM, 0);
	if (unlikely(ctx->sock == DVBS2X_INVALID_SOCKET))
		return -dvbs2x_socket_error();

	/* Set socket options */
	if (unlikely(setsockopt(ctx->sock, SOL_SOCKET, SO_REUSEADDR,
				&opt, sizeof(opt)) != 0))
		goto err_close;

	/* Disable Nagle's algorithm for low latency */
	if (unlikely(setsockopt(ctx->sock, IPPROTO_TCP, TCP_NODELAY,
				&opt, sizeof(opt)) != 0))
		goto err_close;

	/* Bind to port */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	if (unlikely(bind(ctx->sock, (struct sockaddr *)&addr,
			  sizeof(addr)) != 0))
		goto err_close;

	/* Listen */
	if (unlikely(listen(ctx->sock, 5) != 0))
		goto err_close;

	ctx->connected = 0;
	return 0;

err_close:
	dvbs2x_socket_close(ctx->sock);
	ctx->sock = DVBS2X_INVALID_SOCKET;
	return -dvbs2x_socket_error();
}

int dvbs2x_tcp_server_accept(struct dvbs2x_tcp_ctx *server,
			      struct dvbs2x_tcp_ctx *client,
			      int timeout_ms)
{
	struct sockaddr_in addr;
	dvbs2x_socklen_t addr_len = sizeof(addr);
	dvbs2x_socket_t sock;
	fd_set readfds;
	struct timeval tv;
	int ret, opt = 1;

	if (!server || !client)
		return -EINVAL;
	if (server->sock == DVBS2X_INVALID_SOCKET)
		return -EINVAL;

	/* Wait for connection with timeout */
	if (timeout_ms > 0) {
		FD_ZERO(&readfds);
		FD_SET(server->sock, &readfds);
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;

		ret = select(server->sock + 1, &readfds, NULL, NULL, &tv);
		if (ret == 0)
			return -ETIMEDOUT;
		if (ret < 0)
			return -errno;
	}

	/* Accept connection (retry on EINTR) */
	do {
		sock = accept(server->sock, (struct sockaddr *)&addr,
			      &addr_len);
	} while (sock == DVBS2X_INVALID_SOCKET && errno == EINTR);

	if (sock == DVBS2X_INVALID_SOCKET)
		return -dvbs2x_socket_error();

	/* Set TCP_NODELAY on accepted socket */
	setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

	/* Initialize client context */
	memset(client, 0, sizeof(*client));
	client->sock = sock;
	client->connected = 1;

	return 0;
}

int dvbs2x_tcp_client_connect(struct dvbs2x_tcp_ctx *ctx,
			       const char *host, uint16_t port,
			       int timeout_ms)
{
	struct addrinfo hints, *result, *rp;
	char port_str[16];
	int ret, opt = 1;

	if (!ctx || !host)
		return -EINVAL;

	memset(ctx, 0, sizeof(*ctx));

	/* Resolve hostname */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;

	snprintf(port_str, sizeof(port_str), "%u", port);
	ret = getaddrinfo(host, port_str, &hints, &result);
	if (ret != 0)
		return -EHOSTUNREACH;

	/* Try each address */
	for (rp = result; rp; rp = rp->ai_next) {
		ctx->sock = socket(rp->ai_family, rp->ai_socktype,
				   rp->ai_protocol);
		if (ctx->sock == DVBS2X_INVALID_SOCKET)
			continue;

		/* Set TCP_NODELAY */
		setsockopt(ctx->sock, IPPROTO_TCP, TCP_NODELAY,
			   &opt, sizeof(opt));

		/* Set timeout */
		if (timeout_ms > 0)
			dvbs2x_socket_set_timeout(ctx->sock, timeout_ms,
						  timeout_ms);

		/* Connect (retry on EINTR) */
		do {
			ret = connect(ctx->sock, rp->ai_addr, rp->ai_addrlen);
		} while (ret != 0 && errno == EINTR);

		if (ret == 0) {
			ctx->connected = 1;
			freeaddrinfo(result);
			return 0;
		}

		dvbs2x_socket_close(ctx->sock);
		ctx->sock = DVBS2X_INVALID_SOCKET;
	}

	freeaddrinfo(result);
	return -ECONNREFUSED;
}

/*
 * dvbs2x_tcp_send_all - Send all bytes (loop until complete)
 *
 * Critical fix: TCP send() may send partial data, especially on
 * real networks (WAN, congested links). This function loops until
 * all bytes are sent or an error occurs.
 */
int dvbs2x_tcp_send(struct dvbs2x_tcp_ctx *ctx, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	size_t remaining = len;
	ssize_t n;

	if (unlikely(!ctx || !buf))
		return -EINVAL;
	if (unlikely(ctx->sock == DVBS2X_INVALID_SOCKET))
		return -ENOTCONN;

	/* Loop until all bytes sent */
	while (remaining > 0) {
		do {
			n = send(ctx->sock, p, remaining, 0);
		} while (unlikely(n < 0 && errno == EINTR));

		if (unlikely(n < 0))
			return -errno;
		if (unlikely(n == 0))
			return -EPIPE;  /* Connection closed */

		p += n;
		remaining -= n;
		ctx->tx_bytes += n;
	}

	return (int)len;
}

/*
 * dvbs2x_tcp_recv_all - Receive exactly len bytes (loop until complete)
 *
 * Critical fix: TCP recv() may return partial data. This function
 * loops until exactly len bytes are received, or connection closes,
 * or timeout occurs.
 */
int dvbs2x_tcp_recv(struct dvbs2x_tcp_ctx *ctx, void *buf, size_t len,
		    int timeout_ms)
{
	uint8_t *p = buf;
	size_t remaining = len;
	ssize_t n;

	if (unlikely(!ctx || !buf))
		return -EINVAL;
	if (unlikely(ctx->sock == DVBS2X_INVALID_SOCKET))
		return -ENOTCONN;

	/* Set timeout */
	if (timeout_ms > 0)
		dvbs2x_socket_set_timeout(ctx->sock, 0, timeout_ms);

	/* Loop until all bytes received */
	while (remaining > 0) {
		do {
			n = recv(ctx->sock, p, remaining, 0);
		} while (unlikely(n < 0 && errno == EINTR));

		if (unlikely(n < 0))
			return -errno;
		if (unlikely(n == 0))
			return (int)(len - remaining);  /* Connection closed */

		p += n;
		remaining -= n;
		ctx->rx_bytes += n;
	}

	return (int)len;
}

void dvbs2x_tcp_close(struct dvbs2x_tcp_ctx *ctx)
{
	if (unlikely(!ctx))
		return;

	if (likely(ctx->sock != DVBS2X_INVALID_SOCKET)) {
		/* Proper shutdown sequence */
		shutdown(ctx->sock, SHUT_RDWR);
		dvbs2x_socket_close(ctx->sock);
		ctx->sock = DVBS2X_INVALID_SOCKET;
	}

	ctx->connected = 0;
}
