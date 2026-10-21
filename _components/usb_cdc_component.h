#ifndef ESP32_CSI_USB_CDC_COMPONENT_H
#define ESP32_CSI_USB_CDC_COMPONENT_H

#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/*
 * TinyUSB CDC sink (ESP32-S2)
 *
 * The S2 has USB-OTG (full-speed USB 1.1) but no native USB-Serial-JTAG
 * controller, so it cannot route stdout over USB without a software stack.
 * This sink runs the TinyUSB CDC ACM device class on top of the USB OTG
 * peripheral, exposing the chip as a /dev/ttyACM* (Linux), COMx (Windows),
 * /dev/cu.usbmodem* (macOS) device. Once enumerated, the host receives CSV
 * lines at full USB-FS speed (~12 Mbps) instead of the UART baud-rate ceiling.
 *
 * Reference: Embedded_WiFi_Sensing (Armenta-Garcia 2025, Sensors 25, 6220)
 * uses the same TinyUSB CDC pattern on the S3-DevKitC-VIE to achieve 180
 * packets/s on standard baud rates; the S2 implementation here mirrors that
 * design, with one CDC interface per device.
 *
 * Dependency: espressif/esp_tinyusb (managed component) ^2.1.1
 *   Pulled in conditionally for esp32s2 builds via main/idf_component.yml.
 *   The manifest previously pinned "~1.4.0", which no 2.x release satisfies,
 *   so the S2 build failed to resolve the component and stopped at
 *   `fatal error: tinyusb.h: No such file or directory`.
 *
 * Lifecycle:
 *   usb_cdc_sink_init()  - install the TinyUSB driver and CDC ACM interface
 *   usb_cdc_sink_write() - non-blocking enqueue + best-effort flush
 *   usb_cdc_sink_flush() - blocking flush (used at shutdown / SD-style files)
 */

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC

/*
 * esp_tinyusb >= 2.0 renamed and restructured this API:
 *   - "tusb_cdc_acm.h" is a deprecated shim that #pragma-warns; the current
 *     header is "tinyusb_cdc_acm.h".
 *   - tinyusb_config_t was restructured (port / phy / task / descriptor /
 *     event_cb). TINYUSB_DEFAULT_CONFIG() from "tinyusb_default_config.h"
 *     fills it, picking full-speed on S2/S3 and high-speed on P4.
 *   - tinyusb_config_cdcacm_t lost its `usb_dev` member (the port now lives in
 *     the driver config) and its `rx_unread_buf_sz` member.
 *   - the old tinyusb_cdcacm_write_queue_flush() is now
 *     tinyusb_cdcacm_write_flush().
 */
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"

static const char *TAG_USB_CDC = "usb_cdc";

/* TinyUSB has its own write queue (default 64 byte). For CSV burst writes
 * we queue chunks of at most this size and flush after each chunk. The
 * 256-byte chunk below is empirically a good compromise between syscall
 * overhead and queue saturation; tune via menuconfig if needed.
 */
#define USB_CDC_CHUNK_SIZE        256

/* Flush timeout: how long write_queue_flush blocks waiting for the host to
 * drain. 50 ms is long enough to absorb host scheduler hiccups, short enough
 * that a disconnected host does not stall the consumer task indefinitely.
 */
#define USB_CDC_FLUSH_TIMEOUT_MS  50

static bool s_usb_cdc_ready = false;

static inline void usb_cdc_sink_init(void) {
    /*
     * Install the TinyUSB driver. TINYUSB_DEFAULT_CONFIG() selects the
     * full-speed port on S2/S3 with the on-chip PHY and NULL descriptors, so
     * the device enumerates with Espressif's defaults (VID 0x303A) - the same
     * outcome the pre-2.0 code got by zeroing device/string/config descriptors
     * and setting external_phy = false.
     */
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_USB_CDC, "tinyusb_driver_install failed: %s; disabling sink",
                 esp_err_to_name(err));
        return;
    }

    /* CDC ACM interface 0 (the only one we use). rx callbacks are unused
     * because the tool only emits CSV; the VeriCSI-ESP32 SETTIME flow stays on UART
     * via input_component.h.
     */
    // gcc 15 -Werror=missing-field-initializers — value-init then assign
    // (same idiom as sd_component.h / gptimer_broadcast.h / udp_component.h).
    tinyusb_config_cdcacm_t acm_cfg = {};
    acm_cfg.cdc_port                     = TINYUSB_CDC_ACM_0;
    acm_cfg.callback_rx                  = NULL;
    acm_cfg.callback_rx_wanted_char      = NULL;
    acm_cfg.callback_line_state_changed  = NULL;
    acm_cfg.callback_line_coding_changed = NULL;
    err = tinyusb_cdcacm_init(&acm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_USB_CDC, "tinyusb_cdcacm_init failed: %s; disabling sink",
                 esp_err_to_name(err));
        return;
    }

    s_usb_cdc_ready = true;
    ESP_LOGI(TAG_USB_CDC,
             "TinyUSB CDC ACM ready on USB OTG (chunk %d byte, flush timeout %d ms)",
             USB_CDC_CHUNK_SIZE, USB_CDC_FLUSH_TIMEOUT_MS);
}

/* True once the host has opened the CDC port (DTR asserted). */
static inline bool usb_cdc_host_connected(void) {
    return s_usb_cdc_ready && tud_cdc_n_connected(0);
}

/* Returns 0 when the full line was enqueued, -1 when the sink is not ready
 * or the tail of the line was dropped (sink failure contract, audit item
 * I11). */
static inline int usb_cdc_sink_write(const char *data, size_t len) {
    if (!s_usb_cdc_ready || data == NULL || len == 0) {
        return -1;
    }

    size_t written = 0;
    while (written < len) {
        size_t want = len - written;
        if (want > USB_CDC_CHUNK_SIZE) {
            want = USB_CDC_CHUNK_SIZE;
        }
        size_t enqueued = tinyusb_cdcacm_write_queue(
            TINYUSB_CDC_ACM_0,
            (const uint8_t *)data + written,
            want);
        written += enqueued;

        /* If the queue is full or partially full, flush so the host can drain. */
        if (enqueued == 0) {
            (void)tinyusb_cdcacm_write_flush(
                TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(USB_CDC_FLUSH_TIMEOUT_MS));
            /* If the flush also failed (host disconnected, queue still full),
             * drop the rest of this CSV line rather than blocking the CSI
             * consumer task indefinitely. The drop lands in the sink_fail
             * counter through the return code.
             */
            size_t retry = tinyusb_cdcacm_write_queue(
                TINYUSB_CDC_ACM_0,
                (const uint8_t *)data + written,
                len - written);
            if (retry == 0) {
                static uint32_t s_drop_count = 0;
                s_drop_count++;
                if ((s_drop_count & 0x3F) == 0) {
                    ESP_LOGW(TAG_USB_CDC, "CDC queue full; dropping line (cumulative=%lu)",
                             (unsigned long)s_drop_count);
                }
                return -1;
            }
            /* Count the retry's progress; re-sending it would duplicate
             * bytes on the wire (defect found 2026-09-15). */
            written += retry;
        }
    }

    /* Best-effort flush after each line. The CSI consumer task calls this
     * once per CSV row, so flushing per line keeps host-side latency under
     * one packet period at 100/200 Hz target rates.
     */
    (void)tinyusb_cdcacm_write_flush(
        TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(USB_CDC_FLUSH_TIMEOUT_MS));
    return 0;
}

static inline void usb_cdc_sink_flush(void) {
    if (!s_usb_cdc_ready) {
        return;
    }
    (void)tinyusb_cdcacm_write_flush(
        TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(USB_CDC_FLUSH_TIMEOUT_MS));
}

#else  /* CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC not set */

static inline void usb_cdc_sink_init(void)                      {}
static inline void usb_cdc_sink_write(const char *d, size_t l)  { (void)d; (void)l; }
static inline void usb_cdc_sink_flush(void)                     {}
static inline bool usb_cdc_host_connected(void)                 { return true; }

#endif /* CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC */

#endif /* ESP32_CSI_USB_CDC_COMPONENT_H */
