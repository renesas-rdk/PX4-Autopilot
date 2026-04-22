/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file embedded_metadata.cpp
 * @brief QGC metadata files embedded in CR8 firmware ROM.
 *
 * Embeds component_general.json.xz, actuators.json.xz and parameters.json.xz
 * into the firmware binary so that MAVLink FTP can serve them to QGC before
 * CA55/Linux is available.  When CA55 comes up, normal RPC-based FTP takes
 * over transparently.
 *
 * FD allocation:
 *   kEmbeddedFdBase ... kEmbeddedFdBase + kEmbeddedFdSlots-1
 * These are distinct from kRemoteFdBase (4000) used by remote_storage.cpp.
 */


#if defined(__PX4_FREERTOS)

#include "embedded_metadata.h"

#include <cerrno>
#include <fcntl.h>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <pthread.h>

#include <px4_platform_common/log.h>

// ---------------------------------------------------------------------------
// Embedded binary data
// The CMake targets generate these files into ${PX4_BINARY_DIR}/:
//   component_general.json.xz, actuators.json.xz, parameters.json.xz
// We reference them via the absolute path baked in by the compile command
// (see CMakeLists.txt: EMBEDDED_METADATA_BUILD_DIR).
//
// rc.board_defaults.cmds is already embedded by px4_startup_shim.cpp via its
// own .incbin block (_binary_rc_board_defaults_cmds_start/end). We reuse those
// symbols here to avoid a second copy in the firmware binary.
// ---------------------------------------------------------------------------

#ifndef EMBEDDED_METADATA_BUILD_DIR
#error "EMBEDDED_METADATA_BUILD_DIR must be defined by CMake (absolute path to px4/build/rzv)"
#endif

// Stringify macro helper
#define _EMB_STR(x) #x
#define EMB_STR(x) _EMB_STR(x)

// component_general.json.xz
__asm__(
    ".section .rodata\n"
    ".balign 4\n"
    ".global _emb_component_general_start\n"
    "_emb_component_general_start:\n"
    ".incbin \"" EMB_STR(EMBEDDED_METADATA_BUILD_DIR) "/component_general.json.xz\"\n"
    ".global _emb_component_general_end\n"
    "_emb_component_general_end:\n"
    ".balign 4\n"
);

// actuators.json.xz
__asm__(
    ".section .rodata\n"
    ".balign 4\n"
    ".global _emb_actuators_start\n"
    "_emb_actuators_start:\n"
    ".incbin \"" EMB_STR(EMBEDDED_METADATA_BUILD_DIR) "/actuators.json.xz\"\n"
    ".global _emb_actuators_end\n"
    "_emb_actuators_end:\n"
    ".balign 4\n"
);

// parameters.json.xz
__asm__(
    ".section .rodata\n"
    ".balign 4\n"
    ".global _emb_parameters_start\n"
    "_emb_parameters_start:\n"
    ".incbin \"" EMB_STR(EMBEDDED_METADATA_BUILD_DIR) "/parameters.json.xz\"\n"
    ".global _emb_parameters_end\n"
    "_emb_parameters_end:\n"
    ".balign 4\n"
);

// events/all_events.json.xz  (QGC event log decoding)
__asm__(
    ".section .rodata\n"
    ".balign 4\n"
    ".global _emb_all_events_start\n"
    "_emb_all_events_start:\n"
    ".incbin \"" EMB_STR(EMBEDDED_METADATA_BUILD_DIR) "/events/all_events.json.xz\"\n"
    ".global _emb_all_events_end\n"
    "_emb_all_events_end:\n"
    ".balign 4\n"
);

extern "C" {
    extern const uint8_t _emb_component_general_start[];
    extern const uint8_t _emb_component_general_end[];
    extern const uint8_t _emb_actuators_start[];
    extern const uint8_t _emb_actuators_end[];
    extern const uint8_t _emb_parameters_start[];
    extern const uint8_t _emb_parameters_end[];
    extern const uint8_t _emb_all_events_start[];
    extern const uint8_t _emb_all_events_end[];
    // Reuse symbols defined by px4_startup_shim.cpp — no second copy in flash.
    extern const uint8_t _binary_rc_board_defaults_cmds_start[];
    extern const uint8_t _binary_rc_board_defaults_cmds_end[];
}

// ---------------------------------------------------------------------------
// File table
// ---------------------------------------------------------------------------

struct EmbeddedFile {
    const char     *virtual_path;   // path as seen by PX4 (/fs/microsd/...)
    const uint8_t  *data_start;
    const uint8_t  *data_end;
};

static constexpr EmbeddedFile kEmbeddedFiles[] = {
    {
        "/fs/microsd/etc/extras/component_general.json.xz",
        _emb_component_general_start,
        _emb_component_general_end
    },
    {
        "/fs/microsd/etc/extras/actuators.json.xz",
        _emb_actuators_start,
        _emb_actuators_end
    },
    {
        "/fs/microsd/etc/extras/parameters.json.xz",
        _emb_parameters_start,
        _emb_parameters_end
    },
    {
        "/fs/microsd/etc/extras/all_events.json.xz",
        _emb_all_events_start,
        _emb_all_events_end
    },
    {
        // Startup shim intercepts /etc/init.d/rc.board_defaults directly via
        // strcmp before calling px4_open(), so that entry is not needed here.
        // We only register the /fs/microsd/ path used by the Tier-1 RPC fallback.
        "/fs/microsd/rc.board_defaults.cmds",
        _binary_rc_board_defaults_cmds_start,
        _binary_rc_board_defaults_cmds_end
    },
};
static constexpr int kNumEmbeddedFiles = (int)(sizeof(kEmbeddedFiles) / sizeof(kEmbeddedFiles[0]));

// FD namespace: 5000..5015
static constexpr int kEmbeddedFdBase  = 5000;
static constexpr int kEmbeddedFdSlots = 16;

struct EmbeddedHandle {
    int     file_idx;   // index into kEmbeddedFiles, or -1 if free
    off_t   offset;
};

static EmbeddedHandle g_emb_handles[kEmbeddedFdSlots];
static pthread_mutex_t g_emb_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_emb_init_done = false;

static void emb_ensure_init()
{
    if (!g_emb_init_done) {
        for (int i = 0; i < kEmbeddedFdSlots; ++i) {
            g_emb_handles[i].file_idx = -1;
            g_emb_handles[i].offset   = 0;
        }
        g_emb_init_done = true;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool rzv_embedded_metadata_is_handled(const char *path)
{
    if (!path) { return false; }
    for (int i = 0; i < kNumEmbeddedFiles; ++i) {
        if (strcmp(path, kEmbeddedFiles[i].virtual_path) == 0) {
            return true;
        }
    }
    return false;
}

// Look up file size by path without allocating an fd slot (used by stat()).
off_t rzv_embedded_metadata_size_by_path(const char *path)
{
    if (!path) { return (off_t)-1; }
    for (int i = 0; i < kNumEmbeddedFiles; ++i) {
        if (strcmp(path, kEmbeddedFiles[i].virtual_path) == 0) {
            return (off_t)(kEmbeddedFiles[i].data_end - kEmbeddedFiles[i].data_start);
        }
    }
    return (off_t)-1;
}

bool rzv_embedded_metadata_is_fd(int fd)
{
    return fd >= kEmbeddedFdBase && fd < (kEmbeddedFdBase + kEmbeddedFdSlots);
}

int rzv_embedded_metadata_open(const char *path, int flags)
{
    if (!path) {
        errno = EINVAL;
        return -1;
    }

    // Only support read-only access to embedded files.
    // Check write bits explicitly to be robust against both flag conventions:
    //   Newlib arm-none-eabi: O_WRONLY=0x1, O_RDWR=0x2 (low bits)
    //   FreeRTOS-POSIX:       O_WRONLY=0x8000, O_RDWR=0xA000 (high bits)
    // O_CREAT/O_TRUNC also imply write intent.
    const bool is_write = (flags & 0x0003) != 0   /* newlib O_WRONLY/O_RDWR */
                       || (flags & 0x8000) != 0   /* FreeRTOS O_WRONLY/O_RDWR */
                       || (flags & 0x0002) != 0   /* FreeRTOS O_CREAT */
                       || (flags & 0x0200) != 0;  /* Newlib O_CREAT */
    if (is_write) {
        errno = EROFS;
        return -1;
    }

    int file_idx = -1;
    for (int i = 0; i < kNumEmbeddedFiles; ++i) {
        if (strcmp(path, kEmbeddedFiles[i].virtual_path) == 0) {
            file_idx = i;
            break;
        }
    }

    if (file_idx < 0) {
        errno = ENOENT;
        return -1;
    }

    pthread_mutex_lock(&g_emb_mutex);
    emb_ensure_init();

    int slot = -1;
    for (int i = 0; i < kEmbeddedFdSlots; ++i) {
        if (g_emb_handles[i].file_idx < 0) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        pthread_mutex_unlock(&g_emb_mutex);
        errno = EMFILE;
        return -1;
    }

    g_emb_handles[slot].file_idx = file_idx;
    g_emb_handles[slot].offset   = 0;
    pthread_mutex_unlock(&g_emb_mutex);

    PX4_INFO("embedded_metadata: open %s → fd %d (size=%d B)", path, kEmbeddedFdBase + slot,
             (int)(kEmbeddedFiles[file_idx].data_end - kEmbeddedFiles[file_idx].data_start));
    return kEmbeddedFdBase + slot;
}

ssize_t rzv_embedded_metadata_read(int fd, void *buf, size_t count)
{
    if (!buf || count == 0) {
        errno = EINVAL;
        return -1;
    }

    const int slot = fd - kEmbeddedFdBase;
    if (slot < 0 || slot >= kEmbeddedFdSlots) {
        errno = EBADF;
        return -1;
    }

    pthread_mutex_lock(&g_emb_mutex);
    emb_ensure_init();

    const int file_idx = g_emb_handles[slot].file_idx;
    if (file_idx < 0) {
        pthread_mutex_unlock(&g_emb_mutex);
        errno = EBADF;
        return -1;
    }

    const EmbeddedFile &f = kEmbeddedFiles[file_idx];
    const off_t total_size = (off_t)(f.data_end - f.data_start);
    const off_t pos = g_emb_handles[slot].offset;

    if (pos >= total_size) {
        pthread_mutex_unlock(&g_emb_mutex);
        return 0;   // EOF
    }

    const off_t available = total_size - pos;
    const ssize_t to_copy = (ssize_t)((available < (off_t)count) ? (size_t)available : count);

    memcpy(buf, f.data_start + pos, (size_t)to_copy);
    g_emb_handles[slot].offset += to_copy;

    pthread_mutex_unlock(&g_emb_mutex);
    return to_copy;
}

off_t rzv_embedded_metadata_lseek(int fd, off_t offset, int whence)
{
    const int slot = fd - kEmbeddedFdBase;
    if (slot < 0 || slot >= kEmbeddedFdSlots) {
        errno = EBADF;
        return (off_t)-1;
    }

    pthread_mutex_lock(&g_emb_mutex);
    emb_ensure_init();

    const int file_idx = g_emb_handles[slot].file_idx;
    if (file_idx < 0) {
        pthread_mutex_unlock(&g_emb_mutex);
        errno = EBADF;
        return (off_t)-1;
    }

    const EmbeddedFile &f = kEmbeddedFiles[file_idx];
    const off_t total_size = (off_t)(f.data_end - f.data_start);
    off_t new_pos = g_emb_handles[slot].offset;

    switch (whence) {
    case SEEK_SET: new_pos = offset; break;
    case SEEK_CUR: new_pos = g_emb_handles[slot].offset + offset; break;
    case SEEK_END: new_pos = total_size + offset; break;
    default:
        pthread_mutex_unlock(&g_emb_mutex);
        errno = EINVAL;
        return (off_t)-1;
    }

    if (new_pos < 0) { new_pos = 0; }
    if (new_pos > total_size) { new_pos = total_size; }

    g_emb_handles[slot].offset = new_pos;
    pthread_mutex_unlock(&g_emb_mutex);
    return new_pos;
}

off_t rzv_embedded_metadata_size(int fd)
{
    const int slot = fd - kEmbeddedFdBase;
    if (slot < 0 || slot >= kEmbeddedFdSlots) {
        return (off_t)-1;
    }

    pthread_mutex_lock(&g_emb_mutex);
    emb_ensure_init();
    const int file_idx = g_emb_handles[slot].file_idx;
    pthread_mutex_unlock(&g_emb_mutex);

    if (file_idx < 0) {
        return (off_t)-1;
    }

    return (off_t)(kEmbeddedFiles[file_idx].data_end - kEmbeddedFiles[file_idx].data_start);
}

int rzv_embedded_metadata_close(int fd)
{
    const int slot = fd - kEmbeddedFdBase;
    if (slot < 0 || slot >= kEmbeddedFdSlots) {
        errno = EBADF;
        return -1;
    }

    pthread_mutex_lock(&g_emb_mutex);
    emb_ensure_init();

    if (g_emb_handles[slot].file_idx < 0) {
        pthread_mutex_unlock(&g_emb_mutex);
        errno = EBADF;
        return -1;
    }

    g_emb_handles[slot].file_idx = -1;
    g_emb_handles[slot].offset   = 0;
    pthread_mutex_unlock(&g_emb_mutex);
    return 0;
}

#endif /* __PX4_FREERTOS */
