#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>
#include <unistd.h>  // For errno in FreeRTOS+POSIX
#include <errno.h>

// Declare errno if not already defined by FreeRTOS+POSIX
#ifndef errno
extern int FreeRTOS_errno;
#define errno FreeRTOS_errno
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct pollfd {
	int fd;
	short events;
	short revents;
};

#ifndef POLLIN
#define POLLIN 0x0001
#endif
#ifndef POLLOUT
#define POLLOUT 0x0004
#endif
#ifndef POLLERR
#define POLLERR 0x0008
#endif

static inline int poll(struct pollfd *fds, unsigned long nfds, int timeout)
{
	(void)fds;
	(void)nfds;
	(void)timeout;
	errno = ENOSYS;
	return -1;
}

#ifdef __cplusplus
}

#endif

#else
#include_next <poll.h>
#endif /* __PX4_FREERTOS */
