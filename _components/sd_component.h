#ifndef ESP32_CSI_SD_COMPONENT_H
#define ESP32_CSI_SD_COMPONENT_H

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

#include "sdkconfig.h"

/*
 * SD card sink (v6 SDMMC API)
 *
 * Hernandez 2020 used the legacy v4.3 SDSPI_SLOT_CONFIG_DEFAULT() form which
 * was removed in ESP-IDF v5.0+. VeriCSI-ESP32 uses the v6 driver flow:
 *
 *     spi_bus_initialize(host_id, &bus_cfg, ...)
 *     sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT()
 *     esp_vfs_fat_sdspi_mount("/sdcard", &host, &dev_cfg, &mount_cfg, &card)
 *
 * Pin map matches the original (TTGO T8 V1.7 default). Override via Kconfig
 * if your board wires the SD slot differently.
 *
 * outprintf() is preserved as the v1 helper symbol so that any user code that
 * may have called it directly continues to compile. New code should prefer
 * sink_write() in sink_component.h, which fans out to UART / USB-JTAG / SD
 * uniformly.
 */

#ifndef CONFIG_ESP32_CSI_TOOL_SD_MISO
#define CONFIG_ESP32_CSI_TOOL_SD_MISO  2
#endif
#ifndef CONFIG_ESP32_CSI_TOOL_SD_MOSI
#define CONFIG_ESP32_CSI_TOOL_SD_MOSI  15
#endif
#ifndef CONFIG_ESP32_CSI_TOOL_SD_CLK
#define CONFIG_ESP32_CSI_TOOL_SD_CLK   14
#endif
#ifndef CONFIG_ESP32_CSI_TOOL_SD_CS
#define CONFIG_ESP32_CSI_TOOL_SD_CS    13
#endif

#if defined(CONFIG_ESP32_CSI_TOOL_SINK_SD) || defined(CONFIG_SEND_CSI_TO_SD)

static const char *TAG_SD = "sd";

static FILE         *s_sd_file        = NULL;
static char          s_sd_filename[32] = {0};
static sdmmc_card_t *s_sd_card        = NULL;

static void _sd_pick_next_file(void) {
    int i = -1;
    struct stat st;
    while (true) {
        i++;
        printf("Checking %i.csv\n", i);
        snprintf(s_sd_filename, sizeof(s_sd_filename), "/sdcard/%i.csv", i);

        if (stat(s_sd_filename, &st) != 0) {
            break;
        }

        printf("File size: %li\n", st.st_size);
    }
}

static inline esp_err_t sd_init(void) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // gcc 15 -Werror=missing-field-initializers fires on designated-init even
    // when the missing fields would default-init to 0. Use C++ value-init
    // (`= {}`) followed by individual assignments — same fix pattern as
    // Same shape as gptimer_broadcast.h / udp_component.h. This shape
    // is robust against future IDF struct extensions (any new field will
    // default-init to 0 without source changes).
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num     = CONFIG_ESP32_CSI_TOOL_SD_MOSI;
    bus_cfg.miso_io_num     = CONFIG_ESP32_CSI_TOOL_SD_MISO;
    bus_cfg.sclk_io_num     = CONFIG_ESP32_CSI_TOOL_SD_CLK;
    bus_cfg.quadwp_io_num   = -1;
    bus_cfg.quadhd_io_num   = -1;
    bus_cfg.max_transfer_sz = 4000;
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev_cfg.gpio_cs = (gpio_num_t)CONFIG_ESP32_CSI_TOOL_SD_CS;
    dev_cfg.host_id = (spi_host_device_t)host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {};
    mount_cfg.format_if_mount_failed = false;
    mount_cfg.max_files              = 1;
    mount_cfg.allocation_unit_size   = 16 * 1024;

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &dev_cfg, &mount_cfg, &s_sd_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SD,
                     "Failed to mount filesystem. "
                     "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG_SD,
                     "Failed to initialize the card (%s). "
                     "If you do not have an SD card attached, please ignore this message. "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ret;
    }

    sdmmc_card_print_info(stdout, s_sd_card);

    _sd_pick_next_file();
    s_sd_file = fopen(s_sd_filename, "a");
    if (s_sd_file == NULL) {
        ESP_LOGE(TAG_SD, "fopen(%s) failed", s_sd_filename);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/*
 * Periodic durability sync. A bare fwrite() leaves everything in libc/FATFS
 * buffers and - critically - the FAT directory entry (file size) is only
 * committed on fsync/fclose. Found the hard way (2026-08-02, validating the SD
 * bench point): a 5-minute capture powered off without shutdown left a
 * present-but-0-byte file on the card. sink_flush() only runs on the
 * shutdown path, which a pull-the-plug field workflow never takes. So
 * sd_write self-syncs: every SD_SYNC_EVERY_LINES writes or SD_SYNC_MIN_US
 * microseconds, whichever first, fflush + fsync commits data AND metadata.
 * fsync() is the part that updates the FAT chain + directory entry.
 */
#define SD_SYNC_EVERY_LINES 64
#define SD_SYNC_MIN_US      2000000

static int     s_sd_lines_since_sync = 0;
static int64_t s_sd_last_sync_us     = 0;

/* Returns 0 when the full line was written, -1 when the file is closed or
 * the write came up short (sink failure contract, audit item I11). */
static inline int sd_write(const char *data, size_t len) {
    if (s_sd_file == NULL) {
        return -1;
    }
    int rc = fwrite(data, 1, len, s_sd_file) == len ? 0 : -1;
    s_sd_lines_since_sync++;
    int64_t now = esp_timer_get_time();
    if (s_sd_lines_since_sync >= SD_SYNC_EVERY_LINES
            || now - s_sd_last_sync_us >= SD_SYNC_MIN_US) {
        fflush(s_sd_file);
        fsync(fileno(s_sd_file));
        s_sd_lines_since_sync = 0;
        s_sd_last_sync_us = now;
    }
    return rc;
}

static inline void sd_flush(void) {
    if (s_sd_file != NULL) {
        fflush(s_sd_file);
        fclose(s_sd_file);
        s_sd_file = fopen(s_sd_filename, "a");
    }
}

/*
 * Legacy v1 helper. Preserved because some downstream forks call it directly
 * (e.g., custom logging from outside csi_component.h). New code in VeriCSI-ESP32 uses
 * sink_write() in sink_component.h instead.
 */
static inline void outprintf(const char *format, ...) {
    va_list args;
    va_start(args, format);

#if defined(CONFIG_ESP32_CSI_TOOL_SINK_UART) || defined(CONFIG_SEND_CSI_TO_SERIAL)
    vprintf(format, args);
#endif

    if (s_sd_file != NULL) {
        va_list args_sd;
        va_copy(args_sd, args);
        vfprintf(s_sd_file, format, args_sd);
        va_end(args_sd);
    }

    va_end(args);
}

#else // SD sink not enabled

static inline esp_err_t sd_init(void) { return ESP_OK; }
static inline void      sd_write(const char *data, size_t len) { (void)data; (void)len; }
static inline void      sd_flush(void) {}

/*
 * outprintf() must still link when SD is disabled because legacy code paths
 * may still call it. It falls back to vprintf which routes through the IDF
 * console (UART or USB-Serial-JTAG).
 */
static inline void outprintf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

#endif // CONFIG_ESP32_CSI_TOOL_SINK_SD || CONFIG_SEND_CSI_TO_SD

#endif // ESP32_CSI_SD_COMPONENT_H
