/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file posix_dir_stubs.c
 * @brief POSIX directory operations for Renesas RZ/V2H (FreeRTOS)
 *
 * opendir/readdir/closedir route /fs/ paths through the OpenAMP RPC layer
 * to CA55/Linux.  Used by mavlink_log_handler (log listing) and
 * mavlink_ftp (directory listing).
 *
 * DIR handle table: RZV_MAX_OPEN_DIRS concurrent directory scans.
 * Each slot holds the CA55 dir handle returned by rzv_remote_opendir().
 */


#if defined(__PX4_FREERTOS)

#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/* Remote directory RPC functions declared in remote_storage_rpc.h / .c */
extern int rzv_remote_opendir(const char *path);
extern int rzv_remote_readdir(int dir_handle, char *d_name, size_t name_len, unsigned char *d_type);
extern int rzv_remote_closedir(int dir_handle);

/* ---------------------------------------------------------------------------
 * DIR handle table
 * --------------------------------------------------------------------------- */
#define RZV_MAX_OPEN_DIRS 8

struct DIR {
    int           rpc_handle;  /* CA55 dir handle, -1 if slot free */
    struct dirent entry;       /* storage for current entry (readdir returns pointer) */
};

static struct DIR s_dirs[RZV_MAX_OPEN_DIRS];
static pthread_mutex_t s_dirs_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool s_dirs_init = false;

static void dirs_ensure_init(void)
{
    if (!s_dirs_init) {
        for (int i = 0; i < RZV_MAX_OPEN_DIRS; ++i) {
            s_dirs[i].rpc_handle = -1;
        }
        s_dirs_init = true;
    }
}

static struct DIR *dirs_alloc(int rpc_handle)
{
    pthread_mutex_lock(&s_dirs_mutex);
    dirs_ensure_init();

    for (int i = 0; i < RZV_MAX_OPEN_DIRS; ++i) {
        if (s_dirs[i].rpc_handle < 0) {
            s_dirs[i].rpc_handle = rpc_handle;
            pthread_mutex_unlock(&s_dirs_mutex);
            return &s_dirs[i];
        }
    }

    pthread_mutex_unlock(&s_dirs_mutex);
    return NULL;
}

static void dirs_free(struct DIR *dir)
{
    if (!dir) { return; }
    pthread_mutex_lock(&s_dirs_mutex);
    dir->rpc_handle = -1;
    pthread_mutex_unlock(&s_dirs_mutex);
}

/* ---------------------------------------------------------------------------
 * POSIX API
 * --------------------------------------------------------------------------- */

DIR *opendir(const char *path)
{
    if (!path) {
        errno = EINVAL;
        return NULL;
    }

    /* Only route remote filesystem paths through RPC */
    if (strncmp(path, "/fs/", 4) != 0) {
        errno = ENOENT;
        return NULL;
    }

    int handle = rzv_remote_opendir(path);

    if (handle < 0) {
        /* errno already set by rzv_remote_opendir */
        return NULL;
    }

    struct DIR *dir = dirs_alloc(handle);

    if (!dir) {
        rzv_remote_closedir(handle);
        errno = EMFILE;
        return NULL;
    }

    return dir;
}

struct dirent *readdir(DIR *dir)
{
    if (!dir || dir->rpc_handle < 0) {
        errno = EBADF;
        return NULL;
    }

    unsigned char d_type = DT_UNKNOWN;
    int ret = rzv_remote_readdir(dir->rpc_handle,
                                  dir->entry.d_name,
                                  sizeof(dir->entry.d_name),
                                  &d_type);

    if (ret == 1) {
        /* EOF — readdir returns NULL without setting errno */
        return NULL;
    }

    if (ret < 0) {
        /* errno already set */
        return NULL;
    }

    dir->entry.d_type = d_type;
    return &dir->entry;
}

int closedir(DIR *dir)
{
    if (!dir || dir->rpc_handle < 0) {
        errno = EBADF;
        return -1;
    }

    int handle = dir->rpc_handle;
    dirs_free(dir);
    return rzv_remote_closedir(handle);
}

#endif /* __PX4_FREERTOS */
