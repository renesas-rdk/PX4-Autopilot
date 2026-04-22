/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_daemon_shim.cpp
 * @brief PX4 Daemon shim implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include <cstdio>
#include <cctype>
#include <cerrno>
#include <string>
#include <vector>

#include <FreeRTOS.h>
#include <task.h>

#ifndef __PX4_SQ_ENTRY_DEFINED
struct sq_entry_s {
    struct sq_entry_s *flink;
};
#define __PX4_SQ_ENTRY_DEFINED 1
#endif

#include <platforms/posix/apps.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <px4/platforms/posix/src/px4/common/px4_daemon/pxh.h>
#include <px4_daemon/server_io.h>

#ifndef MODULE_NAME
#define MODULE_NAME "pxh_shim"
#endif

namespace px4_daemon
{

apps_map_type Pxh::_apps{};
Pxh *Pxh::_instance{nullptr};

int Pxh::process_line(const std::string &line, bool silently_fail)
{
    if (line.empty()) {
        return PX4_OK;
    }

    if (_apps.empty()) {
        init_app_map(_apps);
    }

    // Avoid std::istringstream here to prevent locale caching that pulls in
    // heavy C++ runtime state and locks, which caused the FreeRTOS task to stall
    // when commands arrive from ISR contexts on this target.
    std::vector<std::string> words;
    words.reserve(8);

    const char *data = line.c_str();
    const std::size_t length = line.length();
    std::size_t idx = 0;

    while (idx < length) {
        while ((idx < length) && std::isspace(static_cast<unsigned char>(data[idx]))) {
            ++idx;
        }

        if (idx >= length) {
            break;
        }

        const std::size_t start = idx;

        while ((idx < length) && !std::isspace(static_cast<unsigned char>(data[idx]))) {
            ++idx;
        }

        words.emplace_back(line.substr(start, idx - start));
    }

    if (words.empty()) {
        return PX4_OK;
    }

    const std::string &command = words.front();
    auto app = _apps.find(command);

    if (app != _apps.end()) {
        std::vector<char *> argv(words.size() + 1, nullptr);

        for (std::size_t i = 0; i < words.size(); ++i) {
            argv[i] = const_cast<char *>(words[i].c_str());
        }

        const int ret = app->second(static_cast<int>(words.size()), argv.data());

        if ((ret != PX4_OK) && !silently_fail) {
            PX4_WARN("Command '%s' returned %d", command.c_str(), ret);
        }

        return ret;
    }

    if (command == "help") {
        list_builtins(_apps);
        return PX4_OK;
    }

    if (!silently_fail) {
        PX4_WARN("Unknown command: %s", command.c_str());
    }

    return PX4_ERROR;
}

void Pxh::run_pxh()
{
    PX4_WARN("pxh shell is not available on this FreeRTOS target");
}

void Pxh::run_remote_pxh(int, int)
{
    PX4_WARN("Remote pxh shell is not available on this FreeRTOS target");
}

void Pxh::stop()
{
    // Nothing to stop in the lightweight shim implementation.
}

} // namespace px4_daemon

extern "C" FILE *get_stdout(bool *isatty_)
{
    if (isatty_ != nullptr) {
        *isatty_ = false;
    }

    return stdout;
}

#endif /* __PX4_FREERTOS */
