/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef OPENVLSNR_PLUTO_H
#define OPENVLSNR_PLUTO_H

#include <iio.h>

#include "dvbs2x_vlsnr.h"

struct pluto_stream {
	struct iio_context	*ctx;
	struct iio_device	*dev;
	struct iio_channel	*i;
	struct iio_channel	*q;
	struct iio_buffer	*buf;
	unsigned int		capacity;
};

struct pluto_config {
	const char	*uri;
	long long	frequency;
	long long	sample_rate;
	long long	bandwidth;
	double		gain;
};

int pluto_tx_open(struct pluto_stream *stream,
		  const struct pluto_config *cfg, unsigned int capacity);
int pluto_rx_open(struct pluto_stream *stream,
		  const struct pluto_config *cfg, unsigned int capacity);
void pluto_stream_close(struct pluto_stream *stream);
int pluto_tx_write(struct pluto_stream *stream,
		   const struct dvbs2x_complex *samples,
		   unsigned int sample_count, double scale);
int pluto_rx_read(struct pluto_stream *stream,
		  struct dvbs2x_complex *samples,
		  unsigned int sample_capacity, unsigned int *sample_count);

#endif /* OPENVLSNR_PLUTO_H */
