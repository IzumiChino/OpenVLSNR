// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Lock-free Ring Buffer (SPSC - Single Producer Single Consumer)
 *
 * Implementation follows Linux kernel kfifo design:
 * - Power-of-2 capacity for efficient modulo (bitwise AND)
 * - Memory barriers for correctness without locks
 * - Cache line alignment prevents false sharing
 * - Always-incrementing indices (wrap at 2^32)
 *
 * Design principles for satellite communication:
 * - Deterministic latency (no locks, no syscalls in hot path)
 * - Zero-copy capable via peek operations
 * - Batch operations amortize atomic overhead
 * - Statistics for monitoring and diagnostics
 */

#ifndef DVBS2X_RINGBUF_H
#define DVBS2X_RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define DVBS2X_CACHE_LINE	64
#define DVBS2X_CACHE_ALIGNED	__attribute__((aligned(DVBS2X_CACHE_LINE)))

/*
 * Ring buffer structure
 *
 * Producer writes to head, consumer reads from tail.
 * head and tail on separate cache lines to prevent false sharing.
 * Always-incrementing indices wrap at 2^32 (unsigned arithmetic).
 */
struct dvbs2x_ringbuf {
	/* Producer cache line */
	uint32_t		head DVBS2X_CACHE_ALIGNED;
	char			_pad1[DVBS2X_CACHE_LINE - sizeof(uint32_t)];

	/* Consumer cache line */
	uint32_t		tail DVBS2X_CACHE_ALIGNED;
	char			_pad2[DVBS2X_CACHE_LINE - sizeof(uint32_t)];

	/* Read-only after init (shared cache line is safe) */
	uint32_t		size;
	uint32_t		mask;
	size_t			elem_size;
	void			*buffer;

	/* Statistics (relaxed atomics, diagnostic only) */
	uint64_t		stats_overruns;
	uint64_t		stats_underruns;
	uint64_t		stats_total_written;
	uint64_t		stats_total_read;
};

/*
 * Ring buffer statistics
 */
struct dvbs2x_ringbuf_stats {
	uint64_t		overruns;
	uint64_t		underruns;
	uint64_t		total_written;
	uint64_t		total_read;
	uint32_t		current_usage;
	uint32_t		capacity;
};

/*
 * dvbs2x_ringbuf_init - Initialize ring buffer
 * @rb: ring buffer structure
 * @buffer: pre-allocated buffer (must be aligned if needed)
 * @size: buffer size in elements (MUST be power of 2)
 * @elem_size: size of each element in bytes
 *
 * Returns 0 on success, -1 if size is not power of 2.
 */
static inline int dvbs2x_ringbuf_init(struct dvbs2x_ringbuf *rb,
				      void *buffer, uint32_t size,
				      size_t elem_size)
{
	/* Verify power of 2 */
	if ((size & (size - 1)) != 0)
		return -1;

	rb->head = 0;
	rb->tail = 0;
	rb->size = size;
	rb->mask = size - 1;
	rb->elem_size = elem_size;
	rb->buffer = buffer;
	rb->stats_overruns = 0;
	rb->stats_underruns = 0;
	rb->stats_total_written = 0;
	rb->stats_total_read = 0;

	return 0;
}

/*
 * dvbs2x_ringbuf_avail - Number of available elements for reading
 */
static inline uint32_t dvbs2x_ringbuf_avail(const struct dvbs2x_ringbuf *rb)
{
	uint32_t head = __atomic_load_n(&rb->head, __ATOMIC_ACQUIRE);

	return head - rb->tail;
}

/*
 * dvbs2x_ringbuf_free - Number of free slots for writing
 */
static inline uint32_t dvbs2x_ringbuf_free(const struct dvbs2x_ringbuf *rb)
{
	uint32_t tail = __atomic_load_n(&rb->tail, __ATOMIC_ACQUIRE);

	return rb->size - (rb->head - tail);
}

/*
 * dvbs2x_ringbuf_is_empty - Check if buffer is empty
 */
static inline bool dvbs2x_ringbuf_is_empty(const struct dvbs2x_ringbuf *rb)
{
	return dvbs2x_ringbuf_avail(rb) == 0;
}

/*
 * dvbs2x_ringbuf_is_full - Check if buffer is full
 */
static inline bool dvbs2x_ringbuf_is_full(const struct dvbs2x_ringbuf *rb)
{
	return dvbs2x_ringbuf_free(rb) == 0;
}

/*
 * dvbs2x_ringbuf_put - Write single element
 * @rb: ring buffer
 * @elem: pointer to element data
 *
 * Returns true on success, false if full.
 */
static inline bool dvbs2x_ringbuf_put(struct dvbs2x_ringbuf *rb,
				      const void *elem)
{
	if (dvbs2x_ringbuf_free(rb) == 0) {
		__atomic_fetch_add(&rb->stats_overruns, 1, __ATOMIC_RELAXED);
		return false;
	}

	uint32_t pos = rb->head & rb->mask;

	memcpy((char *)rb->buffer + pos * rb->elem_size, elem, rb->elem_size);
	__atomic_store_n(&rb->head, rb->head + 1, __ATOMIC_RELEASE);
	__atomic_fetch_add(&rb->stats_total_written, 1, __ATOMIC_RELAXED);

	return true;
}

/*
 * dvbs2x_ringbuf_get - Read single element
 * @rb: ring buffer
 * @elem: pointer to destination buffer
 *
 * Returns true on success, false if empty.
 */
static inline bool dvbs2x_ringbuf_get(struct dvbs2x_ringbuf *rb, void *elem)
{
	if (dvbs2x_ringbuf_avail(rb) == 0) {
		__atomic_fetch_add(&rb->stats_underruns, 1, __ATOMIC_RELAXED);
		return false;
	}

	uint32_t pos = rb->tail & rb->mask;

	memcpy(elem, (char *)rb->buffer + pos * rb->elem_size, rb->elem_size);
	__atomic_store_n(&rb->tail, rb->tail + 1, __ATOMIC_RELEASE);
	__atomic_fetch_add(&rb->stats_total_read, 1, __ATOMIC_RELAXED);

	return true;
}

/*
 * dvbs2x_ringbuf_put_batch - Write multiple elements
 * @rb: ring buffer
 * @elems: pointer to source array
 * @count: number of elements to write
 *
 * Returns number of elements actually written (may be less than count).
 */
static inline uint32_t dvbs2x_ringbuf_put_batch(struct dvbs2x_ringbuf *rb,
						const void *elems,
						uint32_t count)
{
	uint32_t free = dvbs2x_ringbuf_free(rb);
	uint32_t n = (count < free) ? count : free;

	if (n == 0) {
		__atomic_fetch_add(&rb->stats_overruns, 1, __ATOMIC_RELAXED);
		return 0;
	}

	uint32_t head = rb->head;
	uint32_t pos = head & rb->mask;
	uint32_t contig = rb->size - pos;

	if (n <= contig) {
		memcpy((char *)rb->buffer + pos * rb->elem_size,
		       elems, n * rb->elem_size);
	} else {
		memcpy((char *)rb->buffer + pos * rb->elem_size,
		       elems, contig * rb->elem_size);
		memcpy(rb->buffer,
		       (char *)elems + contig * rb->elem_size,
		       (n - contig) * rb->elem_size);
	}

	__atomic_store_n(&rb->head, head + n, __ATOMIC_RELEASE);
	__atomic_fetch_add(&rb->stats_total_written, n, __ATOMIC_RELAXED);

	return n;
}

/*
 * dvbs2x_ringbuf_get_batch - Read multiple elements
 * @rb: ring buffer
 * @elems: pointer to destination array
 * @count: maximum number of elements to read
 *
 * Returns number of elements actually read (may be less than count).
 */
static inline uint32_t dvbs2x_ringbuf_get_batch(struct dvbs2x_ringbuf *rb,
						void *elems, uint32_t count)
{
	uint32_t avail = dvbs2x_ringbuf_avail(rb);
	uint32_t n = (count < avail) ? count : avail;

	if (n == 0) {
		__atomic_fetch_add(&rb->stats_underruns, 1, __ATOMIC_RELAXED);
		return 0;
	}

	uint32_t tail = rb->tail;
	uint32_t pos = tail & rb->mask;
	uint32_t contig = rb->size - pos;

	if (n <= contig) {
		memcpy(elems, (char *)rb->buffer + pos * rb->elem_size,
		       n * rb->elem_size);
	} else {
		memcpy(elems, (char *)rb->buffer + pos * rb->elem_size,
		       contig * rb->elem_size);
		memcpy((char *)elems + contig * rb->elem_size,
		       rb->buffer,
		       (n - contig) * rb->elem_size);
	}

	__atomic_store_n(&rb->tail, tail + n, __ATOMIC_RELEASE);
	__atomic_fetch_add(&rb->stats_total_read, n, __ATOMIC_RELAXED);

	return n;
}

/*
 * dvbs2x_ringbuf_peek - Look ahead without consuming
 * @rb: ring buffer
 * @elem: pointer to destination buffer
 * @offset: offset from current tail position
 *
 * Returns true on success, false if offset exceeds available elements.
 */
static inline bool dvbs2x_ringbuf_peek(const struct dvbs2x_ringbuf *rb,
				       void *elem, uint32_t offset)
{
	if (offset >= dvbs2x_ringbuf_avail(rb))
		return false;

	uint32_t pos = (rb->tail + offset) & rb->mask;

	memcpy(elem, (char *)rb->buffer + pos * rb->elem_size, rb->elem_size);
	return true;
}

/*
 * dvbs2x_ringbuf_reset - Clear buffer
 *
 * NOT thread-safe. Caller must ensure no concurrent access.
 */
static inline void dvbs2x_ringbuf_reset(struct dvbs2x_ringbuf *rb)
{
	rb->head = 0;
	rb->tail = 0;
	rb->stats_overruns = 0;
	rb->stats_underruns = 0;
	rb->stats_total_written = 0;
	rb->stats_total_read = 0;
}

/*
 * dvbs2x_ringbuf_get_stats - Get buffer statistics
 */
static inline void dvbs2x_ringbuf_get_stats(const struct dvbs2x_ringbuf *rb,
					    struct dvbs2x_ringbuf_stats *stats)
{
	stats->overruns = __atomic_load_n(&rb->stats_overruns,
					  __ATOMIC_RELAXED);
	stats->underruns = __atomic_load_n(&rb->stats_underruns,
					   __ATOMIC_RELAXED);
	stats->total_written = __atomic_load_n(&rb->stats_total_written,
					       __ATOMIC_RELAXED);
	stats->total_read = __atomic_load_n(&rb->stats_total_read,
					    __ATOMIC_RELAXED);
	stats->current_usage = dvbs2x_ringbuf_avail(rb);
	stats->capacity = rb->size;
}

#ifdef __cplusplus
}
#endif

#endif /* DVBS2X_RINGBUF_H */
