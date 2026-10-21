#ifndef ESP32_CSI_USB_JTAG_COMPONENT_H
#define ESP32_CSI_USB_JTAG_COMPONENT_H

#include <stdio.h>
#include <stddef.h>
#include "sdkconfig.h"

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#include "freertos/FreeRTOS.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif

/*
 * USB-Serial-JTAG sink (ESP32-C3 / S3 / C5 / C6 / C61 native peripheral)
 *
 * When CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y (set automatically by
 * sdkconfig.defaults.<target> for the relevant chips), stdout is routed to
 * the on-chip USB-Serial-JTAG controller, and the serial sink of
 * uart_component.h writes through the functions below.
 *
 * Native USB 1.1 full-speed (12 Mbps) bypasses the UART baud-rate ceiling
 * and is the recommended sink for high CSI rates (measured on hardware).
 *
 * Note: ESP32-S2 has USB-OTG but NOT native USB-Serial-JTAG; the equivalent
 * "USB at full speed" experience on S2 is provided by TinyUSB CDC ACM in
 * `usb_cdc_component.h`.
 */

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
/*
 * Without the driver, the console VFS writes straight to the 64-byte TX FIFO
 * and, once the FIFO has stayed full for 50 ms, discards bytes while fwrite()
 * still reports success (ESP-IDF v6.0.1, usb_serial_jtag_vfs.c,
 * TX_FLUSH_TIMEOUT_US). A row lost there reaches no counter: the E28 bench
 * lost 12 rows in 140,058 that way (experiments/e28_c_validation). With the
 * driver installed, a row goes into a TX ring buffer that absorbs a host
 * stall, and usb_serial_jtag_write_bytes() either queues the whole row or
 * reports 0, so a row that does not fit within the wait lands in sink_fail
 * and never reaches the wire in part.
 */
#define CSI_USJ_TX_BUFFER_BYTES 16384
#define CSI_USJ_TX_WAIT_MS      1000

static inline void usj_driver_init(void) {
    if (usb_serial_jtag_is_driver_installed()) {
        return;
    }
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = CSI_USJ_TX_BUFFER_BYTES;
    if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
        /* Console text (logs, banners) then goes through the same ring
         * buffer, so it cannot interleave with a row inside the FIFO. */
        fflush(stdout);
        usb_serial_jtag_vfs_use_driver();
    }
}

static inline int usj_driver_write(const char *data, size_t len) {
    if (!usb_serial_jtag_is_driver_installed()) {
        return fwrite(data, 1, len, stdout) == len ? 0 : -1;
    }
    int n = usb_serial_jtag_write_bytes(data, len,
                                        pdMS_TO_TICKS(CSI_USJ_TX_WAIT_MS));
    return n == (int)len ? 0 : -1;
}

static inline void usj_driver_flush(void) {
    fflush(stdout);
    if (usb_serial_jtag_is_driver_installed()) {
        (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(CSI_USJ_TX_WAIT_MS));
    }
}
#endif

static inline void usb_jtag_sink_init(void) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    usj_driver_init();
#endif
}

/* Returns 0 when the full line was accepted by the stream, -1 on a short
 * write (sink failure contract, audit item I11). */
static inline int usb_jtag_sink_write(const char *data, size_t len) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    return usj_driver_write(data, len);
#else
    return fwrite(data, 1, len, stdout) == len ? 0 : -1;
#endif
}

static inline void usb_jtag_sink_flush(void) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    usj_driver_flush();
#else
    fflush(stdout);
#endif
}

#endif // ESP32_CSI_USB_JTAG_COMPONENT_H
