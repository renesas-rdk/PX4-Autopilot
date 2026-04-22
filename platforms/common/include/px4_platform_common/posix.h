/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#if defined(__PX4_FREERTOS)
#include <errno.h>
#include <time.h>
#include <termios.h>

#ifndef PTHREAD_STACK_MIN
#define PTHREAD_STACK_MIN 1024
#endif

#ifndef F_OK
#define F_OK 0
#endif

#ifndef R_OK
#define R_OK 4
#endif

#ifndef W_OK
#define W_OK 2
#endif

#ifndef ENOTSUP
#define ENOTSUP 95
#endif

#ifndef TCIOFLUSH
#define TCIOFLUSH 0
#endif

#ifndef CSTOPB
#define CSTOPB 0
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif /* __PX4_FREERTOS */

#include "sem.h"

#define  PX4_F_RDONLY 1
#define  PX4_F_WRONLY 2

#ifdef __PX4_NUTTX

#include <poll.h>

typedef struct pollfd px4_pollfd_struct_t;
typedef pollevent_t px4_pollevent_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define px4_open 	_GLOBAL open
#define px4_close 	_GLOBAL close
#define px4_ioctl 	_GLOBAL ioctl
#define px4_write 	_GLOBAL write
#define px4_read 	_GLOBAL read
#define px4_poll 	_GLOBAL poll
#define px4_access 	_GLOBAL access
#define px4_getpid 	_GLOBAL getpid

#define  PX4_STACK_OVERHEAD	0

#elif defined(__PX4_POSIX)

#define	 PX4_STACK_OVERHEAD	(1024 * 24)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__BEGIN_DECLS

typedef short px4_pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	px4_pollevent_t 	events;   /* The input event flags */
	px4_pollevent_t 	revents;  /* The output event flags */

	/* Required for PX4 compatibility */
	px4_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} px4_pollfd_struct_t;

#ifndef POLLIN
#define POLLIN       (0x01)
#endif

#if defined(__PX4_QURT)
// Qurt has no fsync implementation so need to declare one here
// and then define a fake one in the Qurt platform code.
void fsync(int fd);
// Qurt doesn't have a way to set the scheduler policy. It is always, essentially,
// SCHED_FIFO. So have to add a fake function for the code that tries to set it.
#include <pthread.h>
__EXPORT int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
// Qurt POSIX implementation doesn't define the SIGCONT signal so we just map it
// to a reasonable alternative
#define SIGCONT SIGALRM
#endif

__EXPORT int 		px4_open(const char *path, int flags, ...);
__EXPORT int 		px4_close(int fd);
__EXPORT ssize_t	px4_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	px4_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		px4_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		px4_poll(px4_pollfd_struct_t *fds, unsigned int nfds, int timeout);
__EXPORT int		px4_access(const char *pathname, int mode);
#if defined(__PX4_FREERTOS)
__EXPORT off_t		px4_lseek(int fd, off_t offset, int whence);
__EXPORT int		px4_fsync(int fd);
__EXPORT int		px4_unlink(const char *pathname);
#endif /* __PX4_FREERTOS */
__EXPORT px4_task_t	px4_getpid(void);

__END_DECLS
#if defined(__PX4_FREERTOS)
static inline int px4_file_access(const char *pathname, int mode)
{
	return px4_access(pathname, mode);
}
static inline int px4_file_open(const char *path, int flags, mode_t mode)
{
	return px4_open(path, flags, mode);
}
static inline int px4_file_close(int fd)
{
	return px4_close(fd);
}
static inline ssize_t px4_file_read(int fd, void *buffer, size_t buflen)
{
	return px4_read(fd, buffer, buflen);
}
static inline ssize_t px4_file_write(int fd, const void *buffer, size_t buflen)
{
	return px4_write(fd, buffer, buflen);
}
static inline off_t px4_file_lseek(int fd, off_t offset, int whence)
{
	return px4_lseek(fd, offset, whence);
}
static inline int px4_file_fsync(int fd)
{
	return px4_fsync(fd);
}
static inline int px4_file_unlink(const char *pathname)
{
	return px4_unlink(pathname);
}
__BEGIN_DECLS
int px4_pthread_setname_np(pthread_t thread, const char *name);
int pthread_setname_np(pthread_t thread, const char *name);
__END_DECLS
#ifndef __PX4_PTHREAD_SETNAME_SOURCE
static inline int px4_pthread_setname_np_current(const char *name)
{
	return px4_pthread_setname_np(pthread_self(), name);
}
#define PX4_PTHREAD_SETNAME_DISPATCH(_1, _2, _FN, ...) _FN
#undef pthread_setname_np
#define pthread_setname_np(...) \
	PX4_PTHREAD_SETNAME_DISPATCH(__VA_ARGS__, px4_pthread_setname_np, px4_pthread_setname_np_current)(__VA_ARGS__)
#endif
#ifndef HAVE_PTHREAD_ATTR_SETINHERITSCHED
int pthread_attr_setinheritsched(pthread_attr_t *attr, int policy);
#endif
#ifndef HAVE_PTHREAD_ATTR_SETCHEDPOLICY
int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
#endif
int pthread_cancel(pthread_t thread);
int pthread_setcancelstate(int state, int *oldstate);
int pthread_setcanceltype(int type, int *oldtype);
int pthread_kill(pthread_t thread, int sig);
long sysconf(int name);
__BEGIN_DECLS
int open(const char *path, int flags, ...);
int close(int fd);
int fsync(int fd);
int unlink(const char *pathname);
int access(const char *pathname, int mode);
off_t lseek(int fd, off_t offset, int whence);
ssize_t read(int fd, void *buffer, size_t buflen);
ssize_t write(int fd, const void *buffer, size_t buflen);
int truncate(const char *path, off_t length);
int rmdir(const char *path);
int mkdir(const char *path, mode_t mode);
int cfsetspeed(struct termios *termios_p, speed_t speed);
int tcflush(int fd, int queue_selector);
struct tm *gmtime_r(const time_t *timer, struct tm *result);
struct tm *localtime_r(const time_t *timer, struct tm *result);
__END_DECLS
#ifndef _SC_PAGESIZE
#define _SC_PAGESIZE 30
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#endif /* __PX4_FREERTOS */
#else
#error "No TARGET OS Provided"
#endif

// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define PX4_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + PX4_STACK_OVERHEAD)

__BEGIN_DECLS

__EXPORT void		px4_show_files(void);

__END_DECLS
