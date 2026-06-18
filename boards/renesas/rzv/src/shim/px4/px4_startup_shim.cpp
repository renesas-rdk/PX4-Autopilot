/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file px4_startup_shim.cpp
 * @brief PX4 Startup shim implementation for Renesas RZ/V2H
 */

#if defined(__PX4_FREERTOS)

#include "px4_startup_shim.h"

#include <FreeRTOS.h>
#include <task.h>
#define MODULE_NAME "px4_bootstrap"

#ifndef __PX4_SQ_ENTRY_DEFINED
struct sq_entry_s
{
    struct sq_entry_s *flink;
};
#define __PX4_SQ_ENTRY_DEFINED 1
#endif
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <lib/parameters/param.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include "openamp_rpc_client.h"
#include "rzv_sdram_layout.h"

#ifndef RC_BOARD_DEFAULTS_CMDS_PATH
#error "RC_BOARD_DEFAULTS_CMDS_PATH must be defined to embed rc.board_defaults.cmds"
#endif

#ifndef RC_BOARD_RCS_PATH
#error "RC_BOARD_RCS_PATH must be defined"
#endif

extern "C"
{
    extern const std::uint8_t _binary_rc_board_defaults_cmds_start[];
    extern const std::uint8_t _binary_rc_board_defaults_cmds_end[];
    extern const std::uint8_t _binary_config_txt_start[];
    extern const std::uint8_t _binary_config_txt_end[];
    extern const std::uint8_t _binary_rcS_start[];
    extern const std::uint8_t _binary_rcS_end[];
}

#include <px4/platforms/posix/src/px4/common/px4_daemon/pxh.h>

#include "rzv_fsp/device_registry.h"

extern "C" void px4_rzv_register_default_uart_devices_once(void);

namespace px4
{
    void init_once();
    void init(int argc, char *argv[], const char *process_name);
}

// Forward declarations for file-scope functions used by anon namespace
extern "C" int rzv_px4_load_sdram_params(void);

namespace
{

    constexpr const char *kDefaultAppName = "px4";

    // warn_on_fail: when true (user-managed files: config.txt, extras.txt), every failed
    // command emits PX4_WARN and continues — user does not need to add '?' prefix.
    // '?' prefix retains its meaning (optional) regardless of warn_on_fail.
    static int run_command_list(const char *begin, size_t total_len, int silently_fail,
                                bool warn_on_fail = false)
    {
        if (!begin || total_len == 0)
        {
            return -PX4_ERROR;
        }

        size_t pos = 0;
        int last_result = PX4_OK;

        while (pos < total_len)
        {
            size_t line_end = pos;

            while (line_end < total_len &&
                   begin[line_end] != '\n' &&
                   begin[line_end] != '\r')
            {
                ++line_end;
            }

            std::string line(begin + pos, line_end - pos);

            while (line_end < total_len &&
                   (begin[line_end] == '\n' || begin[line_end] == '\r'))
            {
                ++line_end;
            }

            pos = line_end;

            const auto first = line.find_first_not_of(" \t");

            if (first == std::string::npos)
            {
                continue;
            }

            const auto last = line.find_last_not_of(" \t");
            line = line.substr(first, last - first + 1);

            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            bool optional = warn_on_fail; // user files: all commands are implicitly optional

            if (!line.empty() && line.front() == '?')
            {
                optional = true;
                line.erase(0, 1);

                const auto opt_first = line.find_first_not_of(" \t");

                if (opt_first == std::string::npos)
                {
                    continue;
                }

                const auto opt_last = line.find_last_not_of(" \t");
                line = line.substr(opt_first, opt_last - opt_first + 1);

                if (line.empty())
                {
                    continue;
                }
            }

            int command_silent = optional ? 1 : silently_fail;
            int result = rzv_px4_run_command(line.c_str(), command_silent);

            if (result != PX4_OK && optional)
            {
                PX4_WARN("user script: command failed (ignored): %s (%d)", line.c_str(), result);
                continue;
            }

            if ((result != PX4_OK) && (silently_fail == 0))
            {
                PX4_ERR("script: command failed: %s", line.c_str());
                return result;
            }

            if (result != PX4_OK)
            {
                last_result = result;
            }
        }

        return last_result;
    }

} // namespace

static int run_embedded_rc_board_defaults(int /*silently_fail*/)
{
    // Upstream-like boot sequence (mirrors NuttX rcS):
    //   1. rc.board_defaults.cmds  → factory params (embedded, always runs)
    //   2. BSON load               → QGC/autosave params override factory
    //   3. config.txt              → user param overrides (SDRAM/U-Boot, optional — WINS)
    //   4. rcS                     → mandatory module starts (embedded, like rc.board_extras)
    //   5. extras.txt              → user extra modules (SDRAM/U-Boot, optional)
    //
    // config.txt runs AFTER BSON (matches upstream) — user-set values always win.
    // All steps use silently_fail=1: a failed command never aborts boot.
    int result = PX4_OK;

    // Step 1: Factory param defaults from rc.board_defaults.cmds (embedded, always runs)
    PX4_INFO("boot: running rc.board_defaults.cmds (factory params)");
    {
        const size_t len = static_cast<size_t>(_binary_rc_board_defaults_cmds_end
                                               - _binary_rc_board_defaults_cmds_start);
        int r = run_command_list(
            reinterpret_cast<const char *>(_binary_rc_board_defaults_cmds_start), len, 1);
        if (r != PX4_OK) { result = r; }
    }

    // Step 2: Load BSON params from SDRAM (QGC/autosave values override factory defaults)
    PX4_INFO("boot: loading BSON params from SDRAM");
    rzv_px4_load_sdram_params();

    // Step 3: User param overrides from config.txt (SDRAM/U-Boot, optional — wins over BSON)
    // Matches upstream: config.txt runs after param import so user values always apply.
    // Absent (SDRAM size=0) → silently skip, BSON+factory params apply unchanged.
    {
        const uint32_t sz = *reinterpret_cast<volatile const uint32_t *>(RZV_SDRAM_CONFIG_SIZE_ADDR);
        if (sz > 0U && sz <= RZV_SDRAM_CONFIG_MAX_PAYLOAD) {
            PX4_INFO("boot: running config.txt (user param overrides, %u bytes)", (unsigned)sz);
            const char *data = reinterpret_cast<const char *>(RZV_SDRAM_CONFIG_DATA_ADDR);
            int r = run_command_list(data, static_cast<size_t>(sz), 1, true /*warn_on_fail*/);
            if (r != PX4_OK) { result = r; }
        }
    }

    // Step 4: Mandatory module starts from rcS (embedded, always runs after params settled)
    PX4_INFO("boot: running rcS (mandatory modules)");
    {
        const size_t len = static_cast<size_t>(_binary_rcS_end - _binary_rcS_start);
        if (len > 0) {
            int r = run_command_list(
                reinterpret_cast<const char *>(_binary_rcS_start), len, 1);
            if (r != PX4_OK) { result = r; }
        }
    }

    // Step 5: User extra modules from extras.txt (SDRAM/U-Boot, optional — skip if absent)
    {
        const uint32_t sz = *reinterpret_cast<volatile const uint32_t *>(RZV_SDRAM_EXTRAS_SIZE_ADDR);
        if (sz > 0U && sz <= RZV_SDRAM_EXTRAS_MAX_PAYLOAD) {
            PX4_INFO("boot: running extras.txt (user extra modules, %u bytes)", (unsigned)sz);
            const char *data = reinterpret_cast<const char *>(RZV_SDRAM_EXTRAS_DATA_ADDR);
            int r = run_command_list(data, static_cast<size_t>(sz), 1, true /*warn_on_fail*/);
            if (r != PX4_OK) { result = r; }
        }
        // else: no extras.txt on SD card — silently skip (all mandatory modules started by rcS)
    }

    return result;
}

__asm__(".section .rodata\n"
        ".global _binary_rc_board_defaults_cmds_start\n"
        "_binary_rc_board_defaults_cmds_start:\n"
        ".incbin \"" RC_BOARD_DEFAULTS_CMDS_PATH "\"\n"
        ".global _binary_rc_board_defaults_cmds_end\n"
        "_binary_rc_board_defaults_cmds_end:\n"

        ".global _binary_rcS_start\n"
        "_binary_rcS_start:\n"
        ".incbin \"" RC_BOARD_RCS_PATH "\"\n"
        ".global _binary_rcS_end\n"
        "_binary_rcS_end:\n");

extern "C" int rzv_px4_bootstrap(void)
{
    if (rzv_register_fsp_devices() != 0)
    {
        PX4_ERR("Failed to register RZ/V FSP devices");
        return -PX4_ERROR;
    }

    px4_rzv_register_default_uart_devices_once();

    px4::init_once();

    char app_name[sizeof(kDefaultAppName)] = {};
    std::snprintf(app_name, sizeof(app_name), "%s", kDefaultAppName);
    char *argv[] = {app_name, nullptr};
    px4::init(1, argv, kDefaultAppName);

    // CA55 readiness is handled asynchronously by deferred_param_sync_task.
    // CR8 modules start immediately; param load retries gracefully when CA55 comes up.

    const char *param_file = "/fs/microsd/params";
    const char *param_backup_file = "/fs/microsd/parameters_backup.bson";

    if (param_file && param_file[0] != '\0')
    {
        if (param_set_default_file(param_file) != 0)
        {
            PX4_ERR("param_set_default_file(%s) failed", param_file);
            return -PX4_ERROR;
        }
    }

    if (param_backup_file && param_backup_file[0] != '\0')
    {
        if (param_set_backup_file(param_backup_file) != 0)
        {
            PX4_WARN("param_set_backup_file(%s) failed", param_backup_file);
        }
    }

    // Enable autosave unconditionally when a param file path is configured.
    // param_load_default() is intentionally NOT called here; it is deferred to after
    // the rc.board_defaults startup script runs (see PX4_ONLINE_COMMANDS in
    // px4_offline_defaults.h). This matches upstream PX4/NuttX rcS behaviour:
    //   1. rc.board_defaults sets factory/fallback defaults via "param set"
    //   2. "param load /fs/microsd/params" runs last → saved user values WIN
    // If the param file does not exist yet (first boot), param load fails silently
    // and the factory defaults from the script remain in effect.
    const char *default_file = param_get_default_file();

    if (default_file && default_file[0] != '\0')
    {
        param_control_autosave(true);
        PX4_INFO("Parameter autosave enabled for %s (load deferred)", default_file);
    }
    else
    {
        PX4_INFO("No parameter default file configured, using compiled parameter defaults");
        param_control_autosave(false);
    }

    return PX4_OK;
}

// Set to true once SDRAM params were loaded successfully.
// Used by rzv_px4_run_command() to skip redundant "param import /fs/microsd/params"
// and by deferred_param_sync_task to skip redundant param_load_default().
static bool s_sdram_params_loaded = false;

// Set to true ONLY after deferred_param_sync_task has completed all its operations
// (session_reset + param_load + param_save).  CA55ReadinessCheck gates arming on this
// flag so that arming is never allowed during the brief window when the RPC endpoint
// has just bound but param sync is still doing heavy RPC work.
static volatile bool s_ca55_storage_ready = false;

extern "C" bool rzv_ca55_storage_ready(void)
{
    return s_ca55_storage_ready;
}

extern "C" int rzv_px4_run_command(const char *command, int silently_fail)
{
    if (!command || command[0] == '\0')
    {
        return -PX4_ERROR;
    }

    // Handle built-in shim commands that are not PX4 shell applications.
    if (strcmp(command, "rzv_sdram_params_load") == 0)
    {
        return rzv_px4_load_sdram_params();
    }

    // Skip "param import/load /fs/microsd/params" when SDRAM params are already
    // loaded — the RPC path is not available yet (CA55 not ready) and would print
    // a spurious "ERROR [param] open '/fs/microsd/params' failed (22)".
    // deferred_param_sync handles the real load once CA55 comes up.
    if (s_sdram_params_loaded &&
        (strstr(command, "param import") != nullptr || strstr(command, "param load") != nullptr) &&
        strstr(command, "/fs/microsd/params") != nullptr)
    {
        PX4_INFO("param load: SDRAM params already loaded — skipping early RPC load");
        return PX4_OK;
    }

    return px4_daemon::Pxh::process_line(command, silently_fail != 0);
}

extern "C" int rzv_px4_run_script(const char *script_path, int silently_fail)
{
    if (!script_path || script_path[0] == '\0')
    {
        return -PX4_ERROR;
    }

    PX4_DEBUG("running script %s", script_path);

    int fd = px4_open(script_path, O_RDONLY);

    if (fd < 0)
    {
        if (strcmp(script_path, "/etc/init.d/rc.board_defaults") == 0)
        {
            return run_embedded_rc_board_defaults(silently_fail);
        }

        PX4_ERR("px4_open(%s) failed: %d", script_path, errno);
        return -PX4_ERROR;
    }

    constexpr size_t kReadChunk = 256;
    std::string contents;
    contents.reserve(1024);

    char buffer[kReadChunk];
    ssize_t bytes_read = 0;

    while ((bytes_read = px4_read(fd, buffer, sizeof(buffer))) > 0)
    {
        contents.append(buffer, static_cast<size_t>(bytes_read));
    }

    if (bytes_read < 0)
    {
        int err = errno;
        PX4_ERR("px4_read(%s) failed: %d", script_path, err);
        px4_close(fd);
        return -PX4_ERROR;
    }

    px4_close(fd);

    int last_result = PX4_OK;

    size_t pos = 0;

    while (pos < contents.size())
    {
        size_t end = contents.find('\n', pos);

        if (end == std::string::npos)
        {
            end = contents.size();
        }

        std::string line = contents.substr(pos, end - pos);
        pos = end + 1;

        // trim leading whitespace
        const auto first = line.find_first_not_of(" \t\r");

        if (first == std::string::npos)
        {
            continue;
        }

        const auto last = line.find_last_not_of(" \t\r");
        line = line.substr(first, last - first + 1);

        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        last_result = px4_daemon::Pxh::process_line(line, silently_fail != 0);

        if (last_result != PX4_OK && silently_fail == 0)
        {
            PX4_ERR("script %s failed on line: %s", script_path, line.c_str());
            break;
        }
    }

    return last_result;
}

// ---------------------------------------------------------------------------
// SDRAM params load — Tier 0 (no CA55 needed)
// ---------------------------------------------------------------------------
// U-Boot pre-loads /boot/cr8_data/params (BSON) into SDRAM before releasing CR8:
//   mw.l 0x41710000 0x00000000
//   if ext4load mmc 0:2 0x41710008 boot/cr8_data/params; then mw.l 0x41710000 ${filesize}; fi
// Layout: [uint32_t size @ 0x41710000][BSON data @ 0x41710008]
// Called after rc.board_defaults.cmds (factory defaults) to override with user-saved values.

static constexpr uint32_t UBOOT_PARAMS_SIZE_ADDR = RZV_SDRAM_PARAMS_SIZE_ADDR;
static constexpr uint32_t UBOOT_PARAMS_DATA_ADDR = RZV_SDRAM_PARAMS_DATA_ADDR;
static constexpr uint32_t UBOOT_PARAMS_MAX_SIZE = RZV_SDRAM_PARAMS_MAX_PAYLOAD;

extern "C" int rzv_px4_load_sdram_params(void)
{
    // Invalidate D-cache for the params SDRAM region before reading.
    {
        const uintptr_t flush_start = static_cast<uintptr_t>(UBOOT_PARAMS_SIZE_ADDR);
        const uintptr_t flush_end = static_cast<uintptr_t>(UBOOT_PARAMS_DATA_ADDR) + UBOOT_PARAMS_MAX_SIZE;
        constexpr uintptr_t kCacheLineSize = 32U;
        for (uintptr_t addr = flush_start; addr < flush_end; addr += kCacheLineSize)
        {
            __asm__ volatile("mcr p15, 0, %0, c7, c6, 1" : : "r"(addr) : "memory");
        }
        __asm__ volatile("dsb" : : : "memory");
    }

    const uint32_t bson_size =
        *reinterpret_cast<volatile const uint32_t *>(UBOOT_PARAMS_SIZE_ADDR);

    if (bson_size == 0U)
    {
        PX4_INFO("sdram_params: no SDRAM params (first boot or file missing), using factory defaults");
        return PX4_OK; // not an error — first boot
    }

    if (bson_size > UBOOT_PARAMS_MAX_SIZE)
    {
        PX4_WARN("sdram_params: invalid size %" PRIu32 " (max %" PRIu32 "), skipping", bson_size, (uint32_t)UBOOT_PARAMS_MAX_SIZE);
        return PX4_OK;
    }

    const void *bson_data = reinterpret_cast<const void *>(UBOOT_PARAMS_DATA_ADDR);
    int ret = param_load_buf(bson_data, static_cast<size_t>(bson_size));

    if (ret == 0)
    {
        PX4_INFO("sdram_params: loaded %" PRIu32 " bytes from SDRAM (latest saved params)", bson_size);
        s_sdram_params_loaded = true;
        return PX4_OK;
    }

    PX4_WARN("sdram_params: load failed (%d), using factory defaults", ret);
    return PX4_OK; // silently_fail-friendly: return OK so step loop continues
}

extern "C" void rzv_px4_enable_param_autosave(int enable)
{
    param_control_autosave(enable != 0);
}

extern "C" int rzv_px4_apply_autoconfig(void)
{
    // Mirrors upstream PX4 rcS behaviour:
    //   if param greater SYS_AUTOCONFIG 0
    //   then
    //       param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* ...
    //       # (airframe script was already applied before param load — re-apply now)
    //       param set SYS_AUTOCONFIG 0
    //       param save
    //   fi
    //
    // Must be called AFTER "param load /fs/microsd/params" so we read the user-saved
    // SYS_AUTOCONFIG value, not the compiled default.

    param_t handle = param_find("SYS_AUTOCONFIG");

    if (handle == PARAM_INVALID)
    {
        return PX4_OK;
    }

    int32_t autoconfig = 0;

    if (param_get(handle, &autoconfig) != 0 || autoconfig <= 0)
    {
        return PX4_OK; // Normal boot — nothing to do
    }

    PX4_INFO("SYS_AUTOCONFIG=%d detected: resetting non-calibration params and re-applying factory defaults",
             (int)autoconfig);

    // Reset all params EXCEPT calibration, RC calibration, flight modes, and flight history.
    // This preserves gyro/accel/mag calibration so the user does not need to re-calibrate.
    rzv_px4_run_command(
        "param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* LND_FLIGHT* TC_*", 0);

    // Re-apply factory param defaults from rc.board_defaults.cmds (modules already running).
    {
        const size_t len = static_cast<size_t>(_binary_rc_board_defaults_cmds_end
                                               - _binary_rc_board_defaults_cmds_start);
        run_command_list(reinterpret_cast<const char *>(_binary_rc_board_defaults_cmds_start),
                         len, 1 /*silently_fail*/);
    }

    // Clear the flag so the next boot is a normal boot.
    rzv_px4_run_command("param set SYS_AUTOCONFIG 0", 0);

    // Persist immediately so the reset survives a power-cycle.
    rzv_px4_run_command("param save", 0);

    PX4_INFO("SYS_AUTOCONFIG: factory defaults applied and saved");
    return PX4_OK;
}

// ---------------------------------------------------------------------------
// Deferred param sync — async CA55 readiness polling
// ---------------------------------------------------------------------------
//
// Problem: CR8 boots before CA55/Linux is up. The "param load /fs/microsd/params"
// startup step may fail (RPC not ready yet). Factory defaults remain. Once CA55
// boots and the RPC endpoint becomes ready (~30–60 s), we want to:
//   1. Load the saved binary params from SD card (user tuning + calibration).
//   2. Run apply_autoconfig in case the user set SYS_AUTOCONFIG=1 via QGC.
//
// Until CA55 is ready, QGC param changes go to PX4 in-memory (RAM). PX4 autosave
// will keep retrying the write every ~10 s. Once RPC is ready the first retry
// succeeds. So user edits are NOT lost between CA55 boot.
//
// CPU cost: vTaskDelay(500ms) per iteration — 0% CPU between iterations.
// Stack: 1 kB (only local vars + PX4 logging).

#define DEFERRED_SYNC_STACK_WORDS (8192U / sizeof(StackType_t))
#define DEFERRED_SYNC_PRIORITY (tskIDLE_PRIORITY + 1U) // Just above idle, lowest possible
#define DEFERRED_SYNC_POLL_MS 500U
#define DEFERRED_SYNC_TIMEOUT_MS 180000U // 3 min: covers slow Linux boot + userspace init

static void deferred_param_sync_task(void *pvParameters)
{
    (void)pvParameters;

    uint32_t elapsed_ms = 0U;

    PX4_DEBUG("deferred_param_sync: started, polling for CA55 RPC endpoint...");

    while (elapsed_ms < DEFERRED_SYNC_TIMEOUT_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(DEFERRED_SYNC_POLL_MS));
        elapsed_ms += DEFERRED_SYNC_POLL_MS;

        if (!px4_openamp_rpc_endpoint_ready())
        {
            continue;
        }

        // CA55 is alive — give the RPC server an extra second to finish its own
        // initialisation before issuing file I/O.
        vTaskDelay(pdMS_TO_TICKS(1000U));

        PX4_INFO("deferred_param_sync: CA55 RPC ready after %u ms", (unsigned)(elapsed_ms + 1000U));

        // Send SESSION_RESET to CA55:
        //   1. Bumps g_rpc_next_request_id far above the previous session's range to
        //      prevent stale-response req_id collisions (EBADF on write attempts).
        //   2. Clears stale PARAM staging/file state from the previous CR8 session
        //      without touching unrelated non-param fds (for example an active logger fd).
        // This must run BEFORE any param file I/O.
        {
            int sr_ret = px4_openamp_rpc_session_reset();
            if (sr_ret != 0)
            {
                PX4_WARN("deferred_param_sync: session reset failed (%d), proceeding anyway", sr_ret);
            }
            // Brief pause to let any remaining stale RPMsg responses drain — they will
            // appear as "no pending slot" in the log and be discarded harmlessly.
            vTaskDelay(pdMS_TO_TICKS(100U));
        }

        if (s_sdram_params_loaded)
        {
            // SDRAM params were already loaded at boot (U-Boot pre-loaded them).
            // No need to re-issue param_load_default() — would wipe the already-loaded values.
            PX4_INFO("deferred_param_sync: SDRAM params already loaded, skipping param_load_default");
        }
        else
        {
            // No SDRAM params (first boot): load via RPC now that CA55 is ready.
            int ret = param_load_default();

            if (ret < 0)
            {
                PX4_WARN("deferred_param_sync: param_load_default failed (%d); "
                         "using factory defaults until next save",
                         ret);
            }
            else
            {
                PX4_INFO("deferred_param_sync: saved params loaded successfully");
            }
        }

        // Handle SYS_AUTOCONFIG if the user requested a factory reset via QGC.
        rzv_px4_apply_autoconfig();

        // By saving here (after all startup is done and CA55 is ready), we guarantee
        // /boot/cr8_data/params always contains the full correct state for U-Boot SDRAM pre-load.
        // This also handles the "deleted param file" case: factory defaults are written back
        // so the file exists on next boot.
        {
            int save_ret = param_save_default(true); // blocking: waits for in-flight async save
            if (save_ret == 0)
            {
                PX4_INFO("deferred_param_sync: post-boot param save completed");
            }
            else
            {
                PX4_WARN("deferred_param_sync: post-boot param save failed (%d)", save_ret);
            }
        }

        // Only now is it safe to arm: session_reset + param_save are complete.
        // CA55ReadinessCheck gates on this flag instead of px4_openamp_rpc_endpoint_ready()
        // to prevent the arming-during-deferred-sync race condition (CR8 crash).
        s_ca55_storage_ready = true;
        PX4_INFO("deferred_param_sync: CA55 storage ready; arming now allowed");

        break;
    }

    if (elapsed_ms >= DEFERRED_SYNC_TIMEOUT_MS)
    {
        PX4_WARN("deferred_param_sync: CA55 not ready after %u ms; "
                 "operating on factory defaults permanently",
                 DEFERRED_SYNC_TIMEOUT_MS);
    }

    vTaskDelete(NULL);
}

extern "C" void rzv_px4_start_deferred_param_sync(void)
{
    static StaticTask_t s_task_buf;
    static StackType_t s_task_stack[DEFERRED_SYNC_STACK_WORDS];

    TaskHandle_t h = xTaskCreateStatic(
        deferred_param_sync_task,
        "param_sync",
        DEFERRED_SYNC_STACK_WORDS,
        NULL,
        DEFERRED_SYNC_PRIORITY,
        s_task_stack,
        &s_task_buf);

    if (h == NULL)
    {
        PX4_ERR("deferred_param_sync: xTaskCreateStatic failed");
    }
    else
    {
        PX4_DEBUG("deferred_param_sync: task started");
    }
}

#endif /* __PX4_FREERTOS */
