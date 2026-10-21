#ifndef ESP32_CSI_SINK_COMPONENT_H
#define ESP32_CSI_SINK_COMPONENT_H

#include <stddef.h>
#include "esp_log.h"
#include "sdkconfig.h"

#include "uart_component.h"
#include "usb_jtag_component.h"
#include "usb_cdc_component.h"
#include "udp_component.h"
#include "sd_component.h"

// __attribute__((unused)): suppress -Wunused-variable in TUs where
// sink_compat_check() is called but the relevant Kconfig branches are all
// inactive (so the inlined ESP_LOGI/W path is dead-code-eliminated).
static const char *TAG_SINK __attribute__((unused)) = "sink";

/*
 * Compile-time sink x chip compatibility check (O-2).
 *
 * Surfaces obvious misconfigurations as ESP_LOGW lines at sink_init()
 * rather than letting the user discover them as silent no-ops at run time.
 *
 * The matrix is intentionally permissive: we warn but never refuse, so a
 * researcher who knows what they are doing (e.g. wiring an external SD
 * adapter to a non-TTGO board) can still set the Kconfig and proceed.
 */
static inline void sink_compat_check(void) {
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_JTAG
#  if !(defined(CONFIG_IDF_TARGET_ESP32C3)  || defined(CONFIG_IDF_TARGET_ESP32S3) || \
        defined(CONFIG_IDF_TARGET_ESP32C5)  || defined(CONFIG_IDF_TARGET_ESP32C6) || \
        defined(CONFIG_IDF_TARGET_ESP32C61))
    ESP_LOGW(TAG_SINK,
             "USB-Serial-JTAG sink enabled on a chip without a native USB-Serial-JTAG controller (%s); "
             "output will go nowhere. Use UART or USB-CDC instead.",
             CONFIG_IDF_TARGET);
#  endif
#endif

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC
#  if !defined(CONFIG_IDF_TARGET_ESP32S2)
    ESP_LOGW(TAG_SINK,
             "TinyUSB CDC sink enabled on %s; this sink is implemented for ESP32-S2 only. "
             "On C3 / S3 / C5 / C6 / C61 use the native USB-Serial-JTAG sink instead.",
             CONFIG_IDF_TARGET);
#  endif
#endif

#if defined(CONFIG_ESP32_CSI_TOOL_SINK_SD) || defined(CONFIG_SEND_CSI_TO_SD)
#  if !defined(CONFIG_IDF_TARGET_ESP32)
    ESP_LOGW(TAG_SINK,
             "SD sink enabled on %s; the default pin map (GPIO 13/14/15/2) targets the "
             "TTGO T8 V1.7 (ESP32 classic) hardware. Override CONFIG_ESP32_CSI_TOOL_SD_* "
             "if your board wires the SD slot differently, or disable the SD sink.",
             CONFIG_IDF_TARGET);
#  endif
#endif

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_UDP
    ESP_LOGI(TAG_SINK,
             "UDP sink enabled. Note: passive mode does not associate with a network; "
             "UDP datagrams will silently fail until a Wi-Fi link is up.");
#endif
}

/*
 * Sink abstraction
 *
 * VeriCSI-ESP32 supports five output paths in any combination:
 *   - UART                (default on ESP32 / S2)
 *   - USB-Serial-JTAG     (default on C3 / S3 / C5 / C6 / C61)
 *   - TinyUSB CDC         (S2 only, not hardware-verified)
 *   - UDP                 (any chip with active Wi-Fi link)
 *   - SD card             (TTGO T8 / boards with on-board SD)
 *
 * Selection is via Kconfig (`CONFIG_ESP32_CSI_TOOL_SINK_*`); each enabled sink
 * receives every CSV line. The legacy v1 Kconfig flags
 * `CONFIG_SEND_CSI_TO_SERIAL` and `CONFIG_SEND_CSI_TO_SD` are honoured during
 * the Kconfig migration period as deprecated aliases.
 *
 * Convention: sink_write is called once per CSV line by the CSI consumer task
 * (see csi_component.h). UDP relies on this to map one datagram per CSI frame.
 */

static inline void sink_init(void) {
    sink_compat_check();
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_UART) || defined(CONFIG_SEND_CSI_TO_SERIAL)
    uart_sink_init();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_JTAG
    usb_jtag_sink_init();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC
    usb_cdc_sink_init();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_UDP
    udp_sink_init();
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_SD) || defined(CONFIG_SEND_CSI_TO_SD)
    sd_init();
#endif
}

/* Returns 0 when every configured sink accepted the line, -1 when at least
 * one configured sink reported a detectable write failure (sink failure
 * contract, audit item I11). Each sink states its own detection boundary at
 * its writer; mirrored sinks are not summed, the return is a single
 * any-sink-failed flag, and per-sink counters remain future work. */
static inline int sink_write(const char *data, size_t len) {
    int rc = 0;
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_UART) || defined(CONFIG_SEND_CSI_TO_SERIAL)
    if (uart_sink_write(data, len) != 0) rc = -1;
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_JTAG
    if (usb_jtag_sink_write(data, len) != 0) rc = -1;
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC
    if (usb_cdc_sink_write(data, len) != 0) rc = -1;
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_UDP
    if (udp_sink_write(data, len) != 0) rc = -1;
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_SD) || defined(CONFIG_SEND_CSI_TO_SD)
    if (sd_write(data, len) != 0) rc = -1;
#endif
    return rc;
}

static inline void sink_flush(void) {
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_UART) || defined(CONFIG_SEND_CSI_TO_SERIAL)
    uart_sink_flush();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_JTAG
    usb_jtag_sink_flush();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC
    usb_cdc_sink_flush();
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_UDP
    udp_sink_flush();
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_SINK_SD) || defined(CONFIG_SEND_CSI_TO_SD)
    sd_flush();
#endif
}

#endif // ESP32_CSI_SINK_COMPONENT_H
