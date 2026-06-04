// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR UDP Transport Implementation
 *
 * Fixed version with:
 * - EINTR handling
 * - MTU awareness
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "dvbs2x_udp.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>

int dvbs2x_udp_init(struct dvbs2x_udp_ctx *ctx, uint16_t port)
{
	struct sockaddr_in addr;

	if (!ctx)
		return -EINVAL;

	memset(ctx, 0, sizeof(*ctx));

	/* Create socket */
	ctx->sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (ctx->sock == DVBS2X_INVALID_SOCKET)
		return -dvbs2x_socket_error();

	/* Bind to port */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	if (bind(ctx->sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		int err = dvbs2x_socket_error();
		dvbs2x_socket_close(ctx->sock);
		ctx->sock = DVBS2X_INVALID_SOCKET;
		return -err;
	}

	ctx->peer_set = 0;
	return 0;
}

int dvbs2x_udp_set_peer(struct dvbs2x_udp_ctx *ctx,
			const char *host, uint16_t port)
{
	struct addrinfo hints, *result;
	char port_str[16];
	int ret;

	if (!ctx || !host)
		return -EINVAL;
	if (ctx->sock == DVBS2X_INVALID_SOCKET)
		return -EINVAL;

	/* Resolve hostname */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	snprintf(port_str, sizeof(port_str), "%u", port);
	ret = getaddrinfo(host, port_str, &hints, &result);
	if (ret != 0)
		return -EHOSTUNREACH;

	/* Use first result */
	if (result) {
		memcpy(&ctx->peer_addr, result->ai_addr,
		       sizeof(ctx->peer_addr));
		ctx->peer_set = 1;
		freeaddrinfo(result);
		return 0;
	}

	freeaddrinfo(result);
	return -EHOSTUNREACH;
}

int dvbs2x_udp_send(struct dvbs2x_udp_ctx *ctx, const void *buf, size_t len)
{
	ssize_t n;

	if (!ctx || !buf)
		return -EINVAL;
	if (ctx->sock == DVBS2X_INVALID_SOCKET)
		return -EINVAL;
	if (!ctx->peer_set)
		return -EDESTADDRREQ;

	/* Send with EINTR retry */
	do {
		n = sendto(ctx->sock, buf, len, 0,
			   (struct sockaddr *)&ctx->peer_addr,
			   sizeof(ctx->peer_addr));
	} while (n < 0 && errno == EINTR);

	if (n < 0) {
		/* EMSGSIZE indicates packet too large for MTU */
		if (errno == EMSGSIZE)
			return -EMSGSIZE;
		return -errno;
	}

	ctx->tx_packets++;
	ctx->tx_bytes += n;
	return (int)n;
}

int dvbs2x_udp_recv(struct dvbs2x_udp_ctx *ctx, void *buf, size_t len,
		    int timeout_ms)
{
	fd_set readfds;
	struct timeval tv;
	ssize_t n;
	int ret;

	if (!ctx || !buf)
		return -EINVAL;
	if (ctx->sock == DVBS2X_INVALID_SOCKET)
		return -EINVAL;

	/* Wait for data with timeout */
	if (timeout_ms > 0) {
		FD_ZERO(&readfds);
		FD_SET(ctx->sock, &readfds);
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;

		ret = select(ctx->sock + 1, &readfds, NULL, NULL, &tv);
		if (ret == 0)
			return -ETIMEDOUT;
		if (ret < 0 && errno != EINTR)
			return -errno;
	}

	/* Receive datagram with EINTR retry */
	do {
		n = recvfrom(ctx->sock, buf, len, 0, NULL, NULL);
	} while (n < 0 && errno == EINTR);

	if (n < 0)
		return -errno;

	ctx->rx_packets++;
	ctx->rx_bytes += n;
	return (int)n;
}

void dvbs2x_udp_close(struct dvbs2x_udp_ctx *ctx)
{
	if (!ctx)
		return;

	if (ctx->sock != DVBS2X_INVALID_SOCKET) {
		dvbs2x_socket_close(ctx->sock);
		ctx->sock = DVBS2X_INVALID_SOCKET;
	}

	ctx->peer_set = 0;
}
