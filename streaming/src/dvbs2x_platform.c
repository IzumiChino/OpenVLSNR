// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Platform Abstraction Layer Implementation
 *
 * Non-inline functions for thread management, platform initialization,
 * and memory locking.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "dvbs2x_platform.h"
#include <stdlib.h>

/*
 * Platform initialization
 */
int dvbs2x_platform_init(void)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	WSADATA wsa_data;

	if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
		return -1;
	return 0;
#else
	/* Ignore SIGPIPE for socket writes */
	signal(SIGPIPE, SIG_IGN);
	return 0;
#endif
}

void dvbs2x_platform_cleanup(void)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	WSACleanup();
#endif
}

/*
 * Thread operations
 */
#ifdef DVBS2X_PLATFORM_WINDOWS
static DWORD WINAPI thread_wrapper(LPVOID arg)
{
	struct {
		dvbs2x_thread_func_t func;
		void *arg;
	} *params = arg;
	dvbs2x_thread_func_t func = params->func;
	void *thread_arg = params->arg;

	free(params);
	func(thread_arg);
	return 0;
}
#endif

int dvbs2x_thread_create(struct dvbs2x_thread *thread,
			 dvbs2x_thread_func_t func, void *arg)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	struct {
		dvbs2x_thread_func_t func;
		void *arg;
	} *params;

	params = malloc(sizeof(*params));
	if (!params)
		return -1;
	params->func = func;
	params->arg = arg;

	thread->handle = CreateThread(NULL, 0, thread_wrapper,
				       params, 0, &thread->tid);
	if (!thread->handle) {
		free(params);
		return -1;
	}
	return 0;
#else
	return pthread_create(&thread->thread, NULL, func, arg);
#endif
}

int dvbs2x_thread_join(struct dvbs2x_thread *thread, void **retval)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	DWORD ret;

	ret = WaitForSingleObject(thread->handle, INFINITE);
	if (ret != WAIT_OBJECT_0)
		return -1;
	CloseHandle(thread->handle);
	if (retval)
		*retval = NULL;
	return 0;
#else
	return pthread_join(thread->thread, retval);
#endif
}

int dvbs2x_thread_detach(struct dvbs2x_thread *thread)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	CloseHandle(thread->handle);
	return 0;
#else
	return pthread_detach(thread->thread);
#endif
}

int dvbs2x_thread_set_priority(struct dvbs2x_thread *thread, int priority)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	int win_priority;

	if (priority >= 80)
		win_priority = THREAD_PRIORITY_TIME_CRITICAL;
	else if (priority >= 60)
		win_priority = THREAD_PRIORITY_HIGHEST;
	else if (priority >= 40)
		win_priority = THREAD_PRIORITY_ABOVE_NORMAL;
	else
		win_priority = THREAD_PRIORITY_NORMAL;

	return SetThreadPriority(thread->handle, win_priority) ? 0 : -1;
#else
#ifdef __linux__
	struct sched_param sp = { .sched_priority = priority };

	return pthread_setschedparam(thread->thread, SCHED_FIFO, &sp);
#else
	(void)thread;
	(void)priority;
	return -1;
#endif
#endif
}

int dvbs2x_thread_set_affinity(struct dvbs2x_thread *thread, int cpu)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	DWORD_PTR mask = 1ULL << cpu;

	return SetThreadAffinityMask(thread->handle, mask) ? 0 : -1;
#else
#ifdef __linux__
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(cpu, &cpuset);
	return pthread_setaffinity_np(thread->thread, sizeof(cpuset), &cpuset);
#else
	(void)thread;
	(void)cpu;
	return -1;
#endif
#endif
}

/*
 * Mutex operations
 */
int dvbs2x_mutex_init(struct dvbs2x_mutex *mtx)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	InitializeCriticalSection(&mtx->cs);
	return 0;
#else
#ifdef __linux__
	pthread_mutexattr_t attr;
	int ret;

	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ADAPTIVE_NP);
	ret = pthread_mutex_init(&mtx->mutex, &attr);
	pthread_mutexattr_destroy(&attr);
	return ret;
#else
	return pthread_mutex_init(&mtx->mutex, NULL);
#endif
#endif
}

void dvbs2x_mutex_destroy(struct dvbs2x_mutex *mtx)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	DeleteCriticalSection(&mtx->cs);
#else
	pthread_mutex_destroy(&mtx->mutex);
#endif
}

int dvbs2x_mutex_lock(struct dvbs2x_mutex *mtx)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	EnterCriticalSection(&mtx->cs);
	return 0;
#else
	return pthread_mutex_lock(&mtx->mutex);
#endif
}

int dvbs2x_mutex_unlock(struct dvbs2x_mutex *mtx)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	LeaveCriticalSection(&mtx->cs);
	return 0;
#else
	return pthread_mutex_unlock(&mtx->mutex);
#endif
}

/*
 * Condition variable operations
 */
int dvbs2x_cond_init(struct dvbs2x_cond *cond)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	InitializeConditionVariable(&cond->cond);
	return 0;
#else
	return pthread_cond_init(&cond->cond, NULL);
#endif
}

void dvbs2x_cond_destroy(struct dvbs2x_cond *cond)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	/* No cleanup needed for CONDITION_VARIABLE */
	(void)cond;
#else
	pthread_cond_destroy(&cond->cond);
#endif
}

int dvbs2x_cond_wait(struct dvbs2x_cond *cond, struct dvbs2x_mutex *mtx)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return SleepConditionVariableCS(&cond->cond, &mtx->cs, INFINITE) ? 0 : -1;
#else
	return pthread_cond_wait(&cond->cond, &mtx->mutex);
#endif
}

int dvbs2x_cond_timedwait(struct dvbs2x_cond *cond, struct dvbs2x_mutex *mtx,
			  int timeout_ms)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return SleepConditionVariableCS(&cond->cond, &mtx->cs, timeout_ms) ? 0 : -1;
#else
	struct timespec ts;
	struct timeval now;
	long nsec;

	gettimeofday(&now, NULL);
	nsec = now.tv_usec * 1000 + (timeout_ms % 1000) * 1000000;
	ts.tv_sec = now.tv_sec + timeout_ms / 1000 + nsec / 1000000000;
	ts.tv_nsec = nsec % 1000000000;

	return pthread_cond_timedwait(&cond->cond, &mtx->mutex, &ts);
#endif
}

int dvbs2x_cond_signal(struct dvbs2x_cond *cond)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	WakeConditionVariable(&cond->cond);
	return 0;
#else
	return pthread_cond_signal(&cond->cond);
#endif
}

int dvbs2x_cond_broadcast(struct dvbs2x_cond *cond)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	WakeAllConditionVariable(&cond->cond);
	return 0;
#else
	return pthread_cond_broadcast(&cond->cond);
#endif
}

/*
 * Memory locking
 */
int dvbs2x_lock_memory(void *addr, size_t len)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return VirtualLock(addr, len) ? 0 : -1;
#else
#ifdef __linux__
	return mlock(addr, len);
#else
	(void)addr;
	(void)len;
	return -1;
#endif
#endif
}

int dvbs2x_unlock_memory(void *addr, size_t len)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return VirtualUnlock(addr, len) ? 0 : -1;
#else
#ifdef __linux__
	return munlock(addr, len);
#else
	(void)addr;
	(void)len;
	return -1;
#endif
#endif
}

int dvbs2x_lock_all_memory(void)
{
#ifdef DVBS2X_PLATFORM_WINDOWS
	return -1; /* Not supported on Windows */
#else
#ifdef __linux__
	return mlockall(MCL_CURRENT | MCL_FUTURE);
#else
	return -1;
#endif
#endif
}
