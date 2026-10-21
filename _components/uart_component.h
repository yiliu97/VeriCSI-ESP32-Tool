#ifndef ESP32_CSI_UART_COMPONENT_H
#define ESP32_CSI_UART_COMPONENT_H

#include <stdio.h>
#include <stddef.h>
#include "sdkconfig.h"

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#include "usb_jtag_component.h"
#endif

/*
 * Serial (console) sink
 *
 * The simplest of the VeriCSI-ESP32 sinks. CSV data goes to the console
 * stream so that the existing `idf.py monitor | grep CSI_DATA` workflow from
 * the Hernandez 2020 README continues to work unchanged.
 *
 * On a UART console the rows are written with fwrite(stdout); the baud rate
 * is configured via sdkconfig (CONFIG_ESP_CONSOLE_UART_BAUDRATE), default
 * 921600 in VeriCSI-ESP32 defaults. On a USB-Serial-JTAG console the rows go
 * through the USB-Serial-JTAG driver (usb_jtag_component.h), whose write
 * reports a row the link could not take instead of discarding it.
 */

static inline void uart_sink_init(void) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    usj_driver_init();
#endif
    // On a UART console, stdout is set up by the IDF console component
    // before app_main runs.
}

/* Returns 0 when the full line was accepted by the stream, -1 on a short
 * write. Part of the sink failure contract (audit item I11): a detectable
 * write failure must land in a counter, not vanish. */
static inline int uart_sink_write(const char *data, size_t len) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    return usj_driver_write(data, len);
#else
    return fwrite(data, 1, len, stdout) == len ? 0 : -1;
#endif
}

static inline void uart_sink_flush(void) {
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    usj_driver_flush();
#else
    fflush(stdout);
#endif
}

#endif // ESP32_CSI_UART_COMPONENT_H
