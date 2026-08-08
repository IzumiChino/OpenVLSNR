// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

#include "pluto_rx_queue.h"

struct rx_block {
	struct dvbs2x_complex	*samples;
	unsigned int		len;
};

struct pluto_rx_queue {
	struct pluto_stream		*stream;
	struct rx_block			*blocks;
	struct dvbs2x_complex		*storage;
	float				*capture_buffer;
	FILE				*capture;
	time_t				deadline;
	unsigned int			block_samples;
	unsigned int			num_blocks;
	atomic_uint			read_pos;
	atomic_uint			write_pos;
	atomic_int			stop;
	atomic_int			done;
	atomic_int			error;
	atomic_ullong			received_samples;
	atomic_uint			high_water;
	atomic_uint			backpressure;
#ifdef _WIN32
	HANDLE				thread;
#else
	pthread_t			thread;
#endif
};

static void queue_yield(int idle)
{
#ifdef _WIN32
	Sleep(idle ? 1 : 0);
#else
	if (idle) {
		const struct timespec delay = { 0, 1000000 };

		nanosleep(&delay, NULL);
	} else {
		sched_yield();
	}
#endif
}

static int capture_block(struct pluto_rx_queue *queue,
			 const struct rx_block *block)
{
	unsigned int i;

	if (!queue->capture)
		return 0;
	for (i = 0; i < block->len; i++) {
		queue->capture_buffer[2 * i] = (float)block->samples[i].i;
		queue->capture_buffer[2 * i + 1] = (float)block->samples[i].q;
	}
	if (fwrite(queue->capture_buffer, 2 * sizeof(float), block->len,
		   queue->capture) != block->len)
		return -1;
	return 0;
}

static void receive_loop(struct pluto_rx_queue *queue)
{
	int queue_full = 0;

	while (!atomic_load_explicit(&queue->stop, memory_order_relaxed)) {
		unsigned int write_pos, next_pos;
		struct rx_block *block;
		int ret;

		if (queue->deadline && time(NULL) >= queue->deadline)
			break;
		write_pos = atomic_load_explicit(&queue->write_pos,
						 memory_order_relaxed);
		next_pos = (write_pos + 1) % queue->num_blocks;
		if (next_pos == atomic_load_explicit(&queue->read_pos,
						     memory_order_acquire)) {
			if (!queue_full)
				atomic_fetch_add(&queue->backpressure, 1);
			queue_full = 1;
			queue_yield(0);
			continue;
		}
		queue_full = 0;
		block = &queue->blocks[write_pos];
		ret = pluto_rx_read(queue->stream, block->samples,
				    queue->block_samples, &block->len);
		if (ret < 0) {
			if (!atomic_load_explicit(&queue->stop,
						  memory_order_relaxed))
				atomic_store(&queue->error, 1);
			break;
		}
		if (ret > 0)
			continue;
		if (capture_block(queue, block) < 0) {
			atomic_store(&queue->error, 1);
			break;
		}
		atomic_store_explicit(&queue->write_pos, next_pos,
				      memory_order_release);
		atomic_fetch_add(&queue->received_samples, block->len);
		{
			unsigned int read_pos, depth, high_water;

			read_pos = atomic_load_explicit(&queue->read_pos,
							memory_order_acquire);
			depth = (next_pos + queue->num_blocks - read_pos) %
				queue->num_blocks;
			high_water = atomic_load(&queue->high_water);
			while (depth > high_water &&
			       !atomic_compare_exchange_weak(&queue->high_water,
							     &high_water, depth))
				;
		}
	}
	atomic_store_explicit(&queue->done, 1, memory_order_release);
}

#ifdef _WIN32
static DWORD WINAPI receive_thread(LPVOID opaque)
{
	receive_loop(opaque);
	return 0;
}
#else
static void *receive_thread(void *opaque)
{
	receive_loop(opaque);
	return NULL;
}
#endif

int pluto_rx_queue_start(struct pluto_rx_queue **result,
			 struct pluto_stream *stream,
			 unsigned int block_samples,
			 unsigned int num_blocks,
			 FILE *capture,
			 time_t deadline)
{
	struct pluto_rx_queue *queue;
	unsigned int i;

	if (!result || !stream || !block_samples || num_blocks < 2 ||
	    num_blocks > SIZE_MAX / block_samples / sizeof(*queue->storage))
		return -1;
	queue = calloc(1, sizeof(*queue));
	if (!queue)
		return -1;
	queue->blocks = calloc(num_blocks, sizeof(*queue->blocks));
	queue->storage = malloc((size_t)num_blocks * block_samples *
				sizeof(*queue->storage));
	if (capture)
		queue->capture_buffer = malloc(2 * block_samples *
					       sizeof(*queue->capture_buffer));
	if (!queue->blocks || !queue->storage ||
	    (capture && !queue->capture_buffer))
		goto fail;
	queue->stream = stream;
	queue->block_samples = block_samples;
	queue->num_blocks = num_blocks;
	queue->capture = capture;
	queue->deadline = deadline;
	for (i = 0; i < num_blocks; i++)
		queue->blocks[i].samples = queue->storage + i * block_samples;
#ifdef _WIN32
	queue->thread = CreateThread(NULL, 0, receive_thread, queue, 0, NULL);
	if (!queue->thread)
		goto fail;
#else
	if (pthread_create(&queue->thread, NULL, receive_thread, queue))
		goto fail;
#endif
	*result = queue;
	return 0;

fail:
	free(queue->capture_buffer);
	free(queue->storage);
	free(queue->blocks);
	free(queue);
	return -1;
}

int pluto_rx_queue_acquire(struct pluto_rx_queue *queue,
			   const struct dvbs2x_complex **samples,
			   unsigned int *sample_count)
{
	unsigned int read_pos;

	if (!queue || !samples || !sample_count)
		return -1;
	for (;;) {
		read_pos = atomic_load_explicit(&queue->read_pos,
						memory_order_relaxed);
		if (read_pos != atomic_load_explicit(&queue->write_pos,
						     memory_order_acquire)) {
			*samples = queue->blocks[read_pos].samples;
			*sample_count = queue->blocks[read_pos].len;
			return 0;
		}
		if (atomic_load_explicit(&queue->done, memory_order_acquire))
			return atomic_load(&queue->error) ? -1 : 1;
		queue_yield(1);
	}
}

void pluto_rx_queue_release(struct pluto_rx_queue *queue)
{
	unsigned int read_pos;

	if (!queue)
		return;
	read_pos = atomic_load_explicit(&queue->read_pos,
					memory_order_relaxed);
	atomic_store_explicit(&queue->read_pos,
			      (read_pos + 1) % queue->num_blocks,
			      memory_order_release);
}

void pluto_rx_queue_get_stats(const struct pluto_rx_queue *queue,
			      unsigned long long *received_samples,
			      unsigned int *queued_blocks,
			      unsigned int *high_water,
			      unsigned int *backpressure)
{
	unsigned int read_pos, write_pos;

	if (!queue)
		return;
	read_pos = atomic_load_explicit(&queue->read_pos,
					memory_order_acquire);
	write_pos = atomic_load_explicit(&queue->write_pos,
					 memory_order_acquire);
	if (received_samples)
		*received_samples = atomic_load(&queue->received_samples);
	if (queued_blocks)
		*queued_blocks = (write_pos + queue->num_blocks - read_pos) %
			queue->num_blocks;
	if (high_water)
		*high_water = atomic_load(&queue->high_water);
	if (backpressure)
		*backpressure = atomic_load(&queue->backpressure);
}

void pluto_rx_queue_stop(struct pluto_rx_queue *queue)
{
	if (!queue)
		return;
	atomic_store(&queue->stop, 1);
	pluto_stream_cancel(queue->stream);
#ifdef _WIN32
	WaitForSingleObject(queue->thread, INFINITE);
	CloseHandle(queue->thread);
#else
	pthread_join(queue->thread, NULL);
#endif
	free(queue->capture_buffer);
	free(queue->storage);
	free(queue->blocks);
	free(queue);
}
