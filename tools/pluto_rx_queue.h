/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef OPENVLSNR_PLUTO_RX_QUEUE_H
#define OPENVLSNR_PLUTO_RX_QUEUE_H

#include <stdio.h>
#include <time.h>

#include "pluto.h"

struct pluto_rx_queue;

int pluto_rx_queue_start(struct pluto_rx_queue **queue,
			 struct pluto_stream *stream,
			 unsigned int block_samples,
			 unsigned int num_blocks,
			 FILE *capture,
			 time_t deadline);
int pluto_rx_queue_acquire(struct pluto_rx_queue *queue,
			   const struct dvbs2x_complex **samples,
			   unsigned int *sample_count);
void pluto_rx_queue_release(struct pluto_rx_queue *queue);
void pluto_rx_queue_get_stats(const struct pluto_rx_queue *queue,
			      unsigned long long *received_samples,
			      unsigned int *queued_blocks,
			      unsigned int *high_water,
			      unsigned int *backpressure);
void pluto_rx_queue_stop(struct pluto_rx_queue *queue);

#endif /* OPENVLSNR_PLUTO_RX_QUEUE_H */
