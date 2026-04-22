#pragma once

#if defined(__PX4_FREERTOS)


/* Directory entry structure for bare-metal / FreeRTOS+RPC builds.
 * opendir/readdir/closedir are implemented in posix_dir_stubs.c and route
 * /fs/ paths through the RZV OpenAMP RPC layer to CA55/Linux. */
struct dirent
{
	char d_name[256];
	unsigned char d_type;
};

#ifndef DT_UNKNOWN
#define DT_UNKNOWN 0
#endif

#ifndef DT_DIR
#define DT_DIR 4
#endif

#ifndef DT_REG
#define DT_REG 8
#endif

/* Opaque directory handle — full definition in posix_dir_stubs.c */
struct DIR;
typedef struct DIR DIR;

#ifdef __cplusplus
extern "C" {
#endif

DIR           *opendir(const char *name);
int            closedir(DIR *dir);
struct dirent *readdir(DIR *dir);
void           rewinddir(DIR *dir);

#ifdef __cplusplus
}

#endif

#else
#include_next <dirent.h>
#endif /* __PX4_FREERTOS */
