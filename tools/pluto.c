// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "pluto.h"

static int write_longlong(struct iio_channel *channel, const char *name,
			  long long value)
{
	if (iio_channel_attr_write_longlong(channel, name, value) < 0) {
		fprintf(stderr, "cannot set %s\n", name);
		return -1;
	}
	return 0;
}

static int configure_phy(struct iio_context *ctx,
			 const struct pluto_config *cfg, int tx)
{
	struct iio_device *phy;
	struct iio_channel *channel;
	struct iio_channel *lo;
	const char *port;

	phy = iio_context_find_device(ctx, "ad9361-phy");
	if (!phy) {
		fprintf(stderr, "cannot find ad9361-phy\n");
		return -1;
	}
	channel = iio_device_find_channel(phy, "voltage0", tx);
	lo = iio_device_find_channel(phy, tx ? "altvoltage1" : "altvoltage0",
				     true);
	if (!channel || !lo) {
		fprintf(stderr, "cannot find AD9361 configuration channels\n");
		return -1;
	}
	port = tx ? "A" : "A_BALANCED";
	if (iio_channel_attr_write(channel, "rf_port_select", port) < 0 ||
	    write_longlong(channel, "rf_bandwidth", cfg->bandwidth) < 0 ||
	    write_longlong(channel, "sampling_frequency", cfg->sample_rate) < 0 ||
	    write_longlong(lo, "frequency", cfg->frequency) < 0)
		return -1;
	if (tx) {
		if (iio_channel_attr_write_double(channel, "hardwaregain",
						  cfg->gain) < 0)
			return -1;
	} else {
		if (iio_channel_attr_write(channel, "gain_control_mode",
					   "manual") < 0 ||
		    iio_channel_attr_write_double(channel, "hardwaregain",
						  cfg->gain) < 0)
			return -1;
	}
	return 0;
}

static int stream_open(struct pluto_stream *stream,
		       const struct pluto_config *cfg,
		       unsigned int capacity, int tx)
{
	const char *device_name;

	if (!stream || !cfg || !cfg->uri || !capacity)
		return -1;
	memset(stream, 0, sizeof(*stream));
	stream->ctx = iio_create_context_from_uri(cfg->uri);
	if (!stream->ctx) {
		fprintf(stderr, "cannot open IIO context %s\n", cfg->uri);
		goto fail;
	}
	if (iio_context_set_timeout(stream->ctx, 1000) < 0) {
		fprintf(stderr, "cannot set IIO timeout\n");
		goto fail;
	}
	if (configure_phy(stream->ctx, cfg, tx) < 0)
		goto fail;
	device_name = tx ? "cf-ad9361-dds-core-lpc" : "cf-ad9361-lpc";
	stream->dev = iio_context_find_device(stream->ctx, device_name);
	if (!stream->dev) {
		fprintf(stderr, "cannot find %s\n", device_name);
		goto fail;
	}
	stream->i = iio_device_find_channel(stream->dev, "voltage0", tx);
	stream->q = iio_device_find_channel(stream->dev, "voltage1", tx);
	if (!stream->i || !stream->q) {
		fprintf(stderr, "cannot find Pluto IQ stream channels\n");
		goto fail;
	}
	iio_channel_enable(stream->i);
	iio_channel_enable(stream->q);
	stream->buf = iio_device_create_buffer(stream->dev, capacity, false);
	if (!stream->buf) {
		fprintf(stderr, "cannot create Pluto stream buffer\n");
		goto fail;
	}
	stream->capacity = capacity;
	return 0;
fail:
	pluto_stream_close(stream);
	return -1;
}

int pluto_tx_open(struct pluto_stream *stream,
		  const struct pluto_config *cfg, unsigned int capacity)
{
	return stream_open(stream, cfg, capacity, 1);
}

int pluto_rx_open(struct pluto_stream *stream,
		  const struct pluto_config *cfg, unsigned int capacity)
{
	return stream_open(stream, cfg, capacity, 0);
}

void pluto_stream_close(struct pluto_stream *stream)
{
	if (!stream)
		return;
	if (stream->buf)
		iio_buffer_destroy(stream->buf);
	if (stream->i)
		iio_channel_disable(stream->i);
	if (stream->q)
		iio_channel_disable(stream->q);
	if (stream->ctx)
		iio_context_destroy(stream->ctx);
	memset(stream, 0, sizeof(*stream));
}

void pluto_stream_cancel(struct pluto_stream *stream)
{
	if (stream && stream->buf)
		iio_buffer_cancel(stream->buf);
}

static int16_t scale_sample(double sample, double scale)
{
	double value = sample * scale * INT16_MAX;

	if (value > INT16_MAX)
		value = INT16_MAX;
	else if (value < INT16_MIN)
		value = INT16_MIN;
	return (int16_t)value;
}

int pluto_tx_write(struct pluto_stream *stream,
		   const struct dvbs2x_complex *samples,
		   unsigned int sample_count, double scale)
{
	ptrdiff_t step;
	unsigned int offset = 0;

	if (!stream || !stream->buf || !samples || scale <= 0.0 || scale > 1.0)
		return -1;
	step = iio_buffer_step(stream->buf);
	while (offset < sample_count) {
		unsigned int count = sample_count - offset;
		char *pi;
		char *pq;
		unsigned int i;
		ssize_t ret;

		if (count > stream->capacity)
			count = stream->capacity;
		pi = iio_buffer_first(stream->buf, stream->i);
		pq = iio_buffer_first(stream->buf, stream->q);
		for (i = 0; i < count; i++) {
			int16_t si = scale_sample(samples[offset + i].i, scale);
			int16_t sq = scale_sample(samples[offset + i].q, scale);

			iio_channel_convert_inverse(stream->i, pi, &si);
			iio_channel_convert_inverse(stream->q, pq, &sq);
			pi += step;
			pq += step;
		}
		ret = iio_buffer_push_partial(stream->buf, count);
		if (ret < 0) {
			fprintf(stderr, "Pluto TX push failed: %s\n",
				strerror((int)-ret));
			return -1;
		}
		offset += count;
	}
	return 0;
}

int pluto_rx_read(struct pluto_stream *stream,
		  struct dvbs2x_complex *samples,
		  unsigned int sample_capacity, unsigned int *sample_count)
{
	ptrdiff_t step;
	char *pi;
	char *pq;
	char *end;
	unsigned int count = 0;
	ssize_t ret;

	if (!sample_count)
		return -1;
	*sample_count = 0;
	if (!stream || !stream->buf || !samples)
		return -1;
	ret = iio_buffer_refill(stream->buf);
	if (ret < 0) {
		if (ret == -EAGAIN || ret == -ETIMEDOUT) {
#ifdef _WIN32
			Sleep(10);
#else
			const struct timespec delay = { 0, 10000000 };

			nanosleep(&delay, NULL);
#endif
			return 1;
		}
		fprintf(stderr, "Pluto RX refill failed: %s\n",
			strerror((int)-ret));
		return -1;
	}
	step = iio_buffer_step(stream->buf);
	pi = iio_buffer_first(stream->buf, stream->i);
	pq = iio_buffer_first(stream->buf, stream->q);
	end = iio_buffer_end(stream->buf);
	while (pi < end && count < sample_capacity) {
		int16_t si;
		int16_t sq;

		iio_channel_convert(stream->i, &si, pi);
		iio_channel_convert(stream->q, &sq, pq);
		samples[count].i = (double)si / INT16_MAX;
		samples[count].q = (double)sq / INT16_MAX;
		pi += step;
		pq += step;
		count++;
	}
	*sample_count = count;
	return 0;
}
