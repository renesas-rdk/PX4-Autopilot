#pragma once

#if defined(__PX4_FREERTOS)

#include <stdint.h>

typedef int64_t fsblkcnt_t;
typedef int64_t fsfilcnt_t;

struct statfs
{
	fsblkcnt_t f_blocks;
	fsblkcnt_t f_bfree;
	fsblkcnt_t f_bavail;
	fsfilcnt_t f_files;
	fsfilcnt_t f_ffree;
	unsigned long f_bsize;
};

/* Set errno=ENOSYS so logger util::check_free_space() takes the
 * "filesystem does not expose statfs" path (returns PX4_OK) instead
 * of treating the error as a real failure and aborting logging. */
#define statfs(path, buf) ((void)(path), (void)(buf), (errno = ENOSYS), -1)

#else
#include_next <sys/statfs.h>
#endif /* __PX4_FREERTOS */
