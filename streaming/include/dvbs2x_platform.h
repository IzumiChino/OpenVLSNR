// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Platform Abstraction Layer
 *
 * Cross-platform types, socket operations, threading primitives, and timing.
 * Zero runtime overhead: all platform selection via compile-time macros.
 *
 * Supported platforms:
 *  - Linux (primary target)
 *  - Windows (optional, via Winsock2)
 */

#ifndef DVBS2X_PLATFORM_H
#define DVBS2X_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * Platform detection
 */
#if defined(_WIN32) || defined(_WIN64)
#define DVBS2X_PLATFORM_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")
#else
#define DVBS2X_PLATFORM_POSIX
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>

/*
 * Branch prediction hints (netdev style)
 * These help the compiler optimize hot paths by indicating expected
 * branch outcomes. Use likely() for fast paths, unlikely() for errors.
 */
#ifndef likely
#define likely(x)	__builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x)	__builtin_expect(!!(x), 0)
#endif
#include <time.h>
#ifdef __linux__
#include <sys/mman.h>
#include <sched.h>
#endif
#endif

/*
 * Socket types
 */
#ifdef DVBS2X_PLATFORM_WINDOWS
typedef SOCKET			dvbs2x_socket_t;
typedef int			dvbs2x_socklen_t;
#define DVBS2X_INVALID_SOCKET	INVALID_SOCKET
#define DVBS2X_SOCKET_ERROR	SOCKET_ERROR
#else
typedef int			dvbs2x_socket_t;
typedef socklen_t		dvbs2x_socklen_t;
#define DVBS2X_INVALID_SOCKET	(-1)
#define DVBS2X_SOCKET_ERROR	(-1)
#endif

/*
 * Thread types
 */
struct dvbs2x_thread {
#ifdef DVBS2X_PLATFORM_WINDOWS
	HANDLE			handle;
	DWORD			tid;
#else
	pthread_t		thread;
#endif
};

struct dvbs2x_mutex {
#ifdef DVBS2X_PLATFORM_WINDOWS
	CRITICAL_SECTION	cs;
#else
	pthread_mutex_t		mutex;
#endif
};

struct dvbs2x_cond {
#ifdef DVBS2X_PLATFORM_WINDOWS
	CONDITION_VARIABLE	cond;
#else
	pthread_cond_t		cond;
#endif
};

/*
 * Platform initialization and cleanup
 */
int dvbs2x_platform_init(void);
void dvbs2x_platform_cleanup(void);

/*
 * Socket operations (inline for zero overhead)
 */
static inline int dvbs2x_socket_close(dvbs2x_socket_t s)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return closesocket(s);
#else
	return close(s);
#endif
}

static inline int dvbs2x_socket_error(void)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return WSAGetLastError();
#else
	return errno;
#endif
}

static inline int dvbs2x_socket_set_nonblocking(dvbs2x_socket_t s, int enable)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	u_long mode = enable ? 1 : 0;
	return ioctlsocket(s, FIONBIO, &mode);
#else
	int flags = fcntl(s, F_GETFL, 0);

	if (flags == -1)
		return -1;
	flags = enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
	return fcntl(s, F_SETFL, flags);
#endif
}

static inline int dvbs2x_socket_set_timeout(dvbs2x_socket_t s, int send_ms,
					    int recv_ms)
{
	int ret;

#ifdef DVBS2X_PLATFORM_WINDOWS
	DWORD send_timeout = send_ms;
	DWORD recv_timeout = recv_ms;

	ret = setsockopt(s, SOL_SOCKET, SO_SNDTIMEO,
			 (char *)&send_timeout, sizeof(send_timeout));
	if (ret != 0)
		return ret;
	return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
			  (char *)&recv_timeout, sizeof(recv_timeout));
#else
	struct timeval send_tv = {
		.tv_sec = send_ms / 1000,
		.tv_usec = (send_ms % 1000) * 1000
	};
	struct timeval recv_tv = {
		.tv_sec = recv_ms / 1000,
		.tv_usec = (recv_ms % 1000) * 1000
	};

	ret = setsockopt(s, SOL_SOCKET, SO_SNDTIMEO,
			 &send_tv, sizeof(send_tv));
	if (ret != 0)
		return ret;
	return setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
			  &recv_tv, sizeof(recv_tv));
#endif
}

static inline int dvbs2x_socket_set_nodelay(dvbs2x_socket_t s, int enable)
{
	int flag = enable ? 1 : 0;

	return setsockopt(s, IPPROTO_TCP, TCP_NODELAY,
			  (char *)&flag, sizeof(flag));
}

static inline int dvbs2x_socket_set_reuseaddr(dvbs2x_socket_t s, int enable)
{
	int flag = enable ? 1 : 0;

	return setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
			  (char *)&flag, sizeof(flag));
}

/*
 * Thread operations
 */
typedef void *(*dvbs2x_thread_func_t)(void *);

int dvbs2x_thread_create(struct dvbs2x_thread *thread,
			 dvbs2x_thread_func_t func, void *arg);
int dvbs2x_thread_join(struct dvbs2x_thread *thread, void **retval);
int dvbs2x_thread_detach(struct dvbs2x_thread *thread);
int dvbs2x_thread_set_priority(struct dvbs2x_thread *thread, int priority);
int dvbs2x_thread_set_affinity(struct dvbs2x_thread *thread, int cpu);

int dvbs2x_mutex_init(struct dvbs2x_mutex *mtx);
void dvbs2x_mutex_destroy(struct dvbs2x_mutex *mtx);
int dvbs2x_mutex_lock(struct dvbs2x_mutex *mtx);
int dvbs2x_mutex_unlock(struct dvbs2x_mutex *mtx);

int dvbs2x_cond_init(struct dvbs2x_cond *cond);
void dvbs2x_cond_destroy(struct dvbs2x_cond *cond);
int dvbs2x_cond_wait(struct dvbs2x_cond *cond, struct dvbs2x_mutex *mtx);
int dvbs2x_cond_timedwait(struct dvbs2x_cond *cond, struct dvbs2x_mutex *mtx,
			  int timeout_ms);
int dvbs2x_cond_signal(struct dvbs2x_cond *cond);
int dvbs2x_cond_broadcast(struct dvbs2x_cond *cond);

/*
 * High-resolution timing
 */
static inline uint64_t dvbs2x_get_time_ns(void)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	static LARGE_INTEGER freq = {0};
	LARGE_INTEGER counter;

	if (freq.QuadPart == 0)
		QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&counter);
	return (counter.QuadPart * 1000000000ULL) / freq.QuadPart;
#else
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
#endif
}

/*
 * Memory locking (real-time systems)
 */
int dvbs2x_lock_memory(void *addr, size_t len);
int dvbs2x_unlock_memory(void *addr, size_t len);
int dvbs2x_lock_all_memory(void);

#ifdef __cplusplus
}
#endif

#endif /* DVBS2X_PLATFORM_H */
