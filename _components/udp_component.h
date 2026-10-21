#ifndef ESP32_CSI_UDP_COMPONENT_H
#define ESP32_CSI_UDP_COMPONENT_H

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

/*
 * UDP sink
 *
 * Streams CSV lines as UDP datagrams to a configurable host:port. Two
 * robustness features over a plain sendto() loop:
 *
 *   O-4 self-heal: track consecutive sendto() failures; when the threshold is
 *   crossed, close + recreate the socket. This recovers from situations where
 *   the kernel-side socket has gone stale (e.g. after a Wi-Fi reassociation
 *   on the device side, or a host-side application restart).
 *
 *   O-7 datagram batching: optionally accumulate N CSV lines into one
 *   sendto() call. Reduces syscall overhead at high packet rates from "one
 *   per CSI frame" to "one per N CSI frames". Default N=1 preserves the
 *   default behaviour.
 *
 * Each CSV line is appended verbatim to the batch buffer (the firmware
 * already terminates lines with \n, which on the host side is enough for
 * "split datagram into lines" recovery).
 */

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_UDP

#ifndef CONFIG_ESP32_CSI_TOOL_UDP_BATCH_LINES
#  define CONFIG_ESP32_CSI_TOOL_UDP_BATCH_LINES 1
#endif

/*
 * Self-heal threshold: after this many consecutive sendto failures, close
 * the socket and recreate it. 32 is a balance between "rare failures should
 * not trigger reconnection" and "host actually went away should reconnect
 * within a few seconds at 100 Hz".
 */
#define UDP_SELFHEAL_THRESHOLD 32

/*
 * Datagram buffer size. 4 KB easily holds 8-16 typical v2 CSV lines
 * (~500 byte each) without exceeding the IPv4 MTU and without forcing
 * fragmentation.
 */
#define UDP_BATCH_BUF_SIZE 4096

static const char *TAG_UDP = "udp_sink";

static int                 s_udp_socket          = -1;
static struct sockaddr_in  s_udp_dest            = {};   // C++ value-init: avoid -Werror=missing-field-initializers
static uint32_t            s_udp_consec_failures = 0;
static uint64_t            s_udp_total_failures  = 0;
static uint64_t            s_udp_total_recreates = 0;

static char    s_udp_batch_buf[UDP_BATCH_BUF_SIZE];
static size_t  s_udp_batch_used  = 0;
static int     s_udp_batch_lines = 0;

/* Forward declarations so flush can call out and back. */
static inline void _udp_flush_batch(void);
static inline int  _udp_open_socket(void);
static inline void _udp_close_socket(void);
static inline void _udp_recreate_socket(void);

static inline int _udp_open_socket(void) {
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (fd < 0) {
        ESP_LOGE(TAG_UDP, "socket() failed: errno %d", errno);
        return -1;
    }

    s_udp_dest.sin_family = AF_INET;
#  ifdef CONFIG_ESP32_CSI_TOOL_UDP_HOST
    s_udp_dest.sin_addr.s_addr = inet_addr(CONFIG_ESP32_CSI_TOOL_UDP_HOST);
#  else
    s_udp_dest.sin_addr.s_addr = inet_addr("192.168.4.2");
#  endif
#  ifdef CONFIG_ESP32_CSI_TOOL_UDP_PORT
    s_udp_dest.sin_port = htons(CONFIG_ESP32_CSI_TOOL_UDP_PORT);
#  else
    s_udp_dest.sin_port = htons(5005);
#  endif

    return fd;
}

static inline void _udp_close_socket(void) {
    if (s_udp_socket >= 0) {
        close(s_udp_socket);
        s_udp_socket = -1;
    }
}

static inline void _udp_recreate_socket(void) {
    _udp_close_socket();
    s_udp_socket = _udp_open_socket();
    s_udp_total_recreates++;
    s_udp_consec_failures = 0;
    if (s_udp_socket >= 0) {
        ESP_LOGW(TAG_UDP, "self-healed socket (recreate #%llu) -> %s:%d",
                 (unsigned long long)s_udp_total_recreates,
                 inet_ntoa(s_udp_dest.sin_addr),
                 ntohs(s_udp_dest.sin_port));
    }
}

static inline void udp_sink_init(void) {
    s_udp_socket          = _udp_open_socket();
    s_udp_consec_failures = 0;
    s_udp_total_failures  = 0;
    s_udp_total_recreates = 0;
    s_udp_batch_used      = 0;
    s_udp_batch_lines     = 0;
    if (s_udp_socket >= 0) {
        ESP_LOGI(TAG_UDP, "UDP sink targeting %s:%d, batch=%d lines/datagram",
                 inet_ntoa(s_udp_dest.sin_addr),
                 ntohs(s_udp_dest.sin_port),
                 CONFIG_ESP32_CSI_TOOL_UDP_BATCH_LINES);
    }
}

static inline void _udp_flush_batch(void) {
    if (s_udp_batch_used == 0 || s_udp_socket < 0) {
        s_udp_batch_used  = 0;
        s_udp_batch_lines = 0;
        return;
    }

    int sent = sendto(s_udp_socket, s_udp_batch_buf, s_udp_batch_used, 0,
                      (struct sockaddr *)&s_udp_dest, sizeof(s_udp_dest));
    if (sent < 0) {
        s_udp_total_failures++;
        s_udp_consec_failures++;
        if ((s_udp_total_failures & 0xFFu) == 0) {
            ESP_LOGW(TAG_UDP, "sendto failed: errno %d (cumulative=%llu, consec=%lu)",
                     errno,
                     (unsigned long long)s_udp_total_failures,
                     (unsigned long)s_udp_consec_failures);
        }
        if (s_udp_consec_failures >= UDP_SELFHEAL_THRESHOLD) {
            _udp_recreate_socket();
        }
    } else {
        s_udp_consec_failures = 0;
    }

    s_udp_batch_used  = 0;
    s_udp_batch_lines = 0;
}

/* Returns 0 when the line was accepted into the batch, -1 when the socket
 * is down or the line was dropped as oversize (sink failure contract, audit
 * item I11). A datagram lost after sendto() is outside this detection
 * boundary and belongs to the receiving side's sequence audit. */
static inline int udp_sink_write(const char *data, size_t len) {
    if (s_udp_socket < 0 || data == NULL || len == 0) {
        return -1;
    }

    /* If this line on its own would not fit, flush whatever is buffered then
     * skip the line (it cannot be sent anyway and oversized payloads only
     * hurt the receiver). */
    if (len > UDP_BATCH_BUF_SIZE) {
        _udp_flush_batch();
        ESP_LOGW(TAG_UDP, "dropping oversize line (%u byte > batch buf %d)",
                 (unsigned)len, UDP_BATCH_BUF_SIZE);
        return -1;
    }

    /* Flush before append if the buffer would overflow. */
    if (s_udp_batch_used + len > UDP_BATCH_BUF_SIZE) {
        _udp_flush_batch();
    }

    memcpy(s_udp_batch_buf + s_udp_batch_used, data, len);
    s_udp_batch_used  += len;
    s_udp_batch_lines += 1;

    if (s_udp_batch_lines >= CONFIG_ESP32_CSI_TOOL_UDP_BATCH_LINES) {
        _udp_flush_batch();
    }
    return 0;
}

static inline void udp_sink_flush(void) {
    _udp_flush_batch();
}

#else  /* CONFIG_ESP32_CSI_TOOL_SINK_UDP */

static inline void udp_sink_init(void) {}
static inline int udp_sink_write(const char *data, size_t len) { (void)data; (void)len; return 0; }
static inline void udp_sink_flush(void) {}

#endif /* CONFIG_ESP32_CSI_TOOL_SINK_UDP */

#endif /* ESP32_CSI_UDP_COMPONENT_H */
