#ifndef ESP32_CSI_CSI_COMPONENT_H
#define ESP32_CSI_CSI_COMPONENT_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

#include "time_component.h"
#include "chip_aware_decoder.h"
#include "sink_component.h"
#include "board_capabilities.h"
#include "rxctrl_compat.h"
#include "die_temp.h"

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_BINARY
#include "binary_sink.h"
#endif

/* gptimer broadcast counters (active_ap only); declared weakly so other apps
 * link without pulling in the broadcast module. */
#ifdef CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
extern uint64_t gptimer_broadcast_tx_count(void);
extern uint64_t gptimer_broadcast_tx_failures(void);
extern uint32_t gptimer_broadcast_seq(void);
#endif

/*
 * CSI component (VeriCSI-ESP32 rewrite)
 *
 * Differences vs Hernandez 2020 (v1):
 *   - On HE-capable chips (C5/C6/C61) the v6.0.1 wifi_csi_config_t is a
 *     typedef alias of wifi_csi_acquire_config_t with rich Wi-Fi 6 HE-LTF
 *     acquisition flags (acquire_csi_su/mu/dcm/beamformed/he_stbc_mode) plus
 *     the C61/C5 workaround flag acquire_csi_force_lltf. On non-HE chips
 *     (ESP32/S2/S3/C3) wifi_csi_config_t keeps the v4.x layout
 *     (lltf_en/htltf_en/stbc_htltf2_en/ltf_merge_en/channel_filter_en/
 *     manu_scale/shift/dump_ack_en). csi_init() configures both via #if
 *     CONFIG_SOC_WIFI_HE_SUPPORT branches.
 *   - The rx callback no longer formats CSV in line: it copies the buffer into
 *     a queue item and returns immediately. A consumer task formats and writes
 *     the CSV. This keeps the Wi-Fi callback fast on single-core RISC-V chips
 *     (C3 / C5 / C6 / C61) and avoids dropped frames at high sampling rates.
 *   - CSV line format is the v2 super-set documented in docs/csv_format_v2.md
 *     (v1 columns preserved; 8 v2 columns inserted between `len` and `CSI_DATA`).
 *   - Output goes through sink_write() instead of printf+outprintf, so any
 *     enabled sink (UART / USB-Serial-JTAG / SD / UDP) receives the data.
 *
 * Public API (preserved from v1):
 *   csi_init(role)           - install rx callback, spawn consumer task,
 *                              configure CSI acquisition flags from Kconfig.
 *
 * VeriCSI-ESP32 additions:
 *   csi_set_broadcast_seq(seq)
 *                            - called by gptimer_broadcast.h on the AP side
 *                              to thread the broadcast sequence number through
 *                              into emitted CSV rows. STA / passive ignore it.
 */

#define CSI_RAW       1
#define CSI_AMPLITUDE 0
#define CSI_PHASE     0
#define CSI_TYPE      CSI_RAW

/*
 * CSI queue depth.
 *
 * Default 64 items (~32 KB internal SRAM at sizeof(csi_queue_item_t) ~ 500 B)
 * holds about 0.6 s of headroom at 100 Hz CSI rate, enough to ride out short
 * sink-write stalls on all targets.
 *
 * On chips with PSRAM enabled (CONFIG_SPIRAM=y - typically ESP32-S3 with
 * Octal SPI 8/16 MB) the user can opt into a much larger queue allocated
 * from external RAM via CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER. With
 * CONFIG_ESP32_CSI_TOOL_PSRAM_QUEUE_DEPTH=4096 (default for the PSRAM path)
 * the queue holds ~40 s at 100 Hz, surviving multi-second sink stalls and
 * USB host disconnects without dropped frames.
 */
#if defined(CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER) && \
    defined(CONFIG_ESP32_CSI_TOOL_PSRAM_QUEUE_DEPTH)
#  define CSI_QUEUE_DEPTH     CONFIG_ESP32_CSI_TOOL_PSRAM_QUEUE_DEPTH
#else
#  define CSI_QUEUE_DEPTH     64
#endif
#define CSI_MAX_BUF_LEN       384   // covers HE-LTF max length seen on C5 / C6 / C61
#define CSI_LINE_BUF_LEN      2048  // worst-case CSV line: ~ 25 metadata + 8 v2 + 384*5 byte CSI
#define CSI_CONSUMER_STACK    8192
#define CSI_CONSUMER_PRIORITY 5

/*
 * Consumer task core affinity (O-10 in the optimisation review).
 * Default: same core as the Wi-Fi rx task on dual-core chips so that the
 * inter-task queue stays cache-local. Override via Kconfig if Wi-Fi back-
 * pressure is observed under sustained high CSI rates.
 */
#if CONFIG_FREERTOS_UNICORE
#  define CSI_CONSUMER_CORE   tskNO_AFFINITY
#elif defined(CONFIG_ESP32_CSI_TOOL_CONSUMER_CORE)
#  define CSI_CONSUMER_CORE   CONFIG_ESP32_CSI_TOOL_CONSUMER_CORE
#else
#  define CSI_CONSUMER_CORE   0
#endif

/*
 * Diagnostic frame emission cadence (O-1). Default 10 s; 0 disables.
 */
#ifndef CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S
#  define CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S 10
#endif

/*
 * Consumer task yield cadence (O-6). Default yield once per 32 frames; 0
 * disables yielding. Yielding gives equal-priority tasks a chance to run
 * under sustained-high-rate captures and prevents the consumer from
 * monopolising the core.
 */
#ifndef CONFIG_ESP32_CSI_TOOL_CONSUMER_YIELD_EVERY
#  define CONFIG_ESP32_CSI_TOOL_CONSUMER_YIELD_EVERY 32
#endif

#ifndef CONFIG_ESP32_CSI_TOOL_VAL_SCALE
#define CONFIG_ESP32_CSI_TOOL_VAL_SCALE 0
#endif

static const char *TAG_CSI = "csi";

/*
 * Receive-gain state carried with every record (schema v2.1).
 *
 * ESP-IDF exposes no public API to read the automatic gain control setting.
 * The vendor component esp_csi_gain_ctrl does, and schema v2.2 uses it when
 * ESP32_CSI_TOOL_RX_GAIN_CTRL is set: agc_gain and fft_gain are read per
 * record from the receive-control structure, and the force routine locks the
 * gain when a value is configured. Without the component the two columns read
 * CSI_FIELD_NA and the gain runs free, which is how every capture in this
 * repository before 2026-08-18 was taken.
 *
 * The component ships as a precompiled static library per IDF version and
 * target, Apache-licensed but not source-available, so a capture taken with
 * it depends on a binary this project cannot rebuild. That is stated in the
 * record itself through the two columns, which read CSI_FIELD_NA when the
 * component is absent.
 */
#if defined(CONFIG_ESP32_CSI_TOOL_RX_GAIN_CTRL) && CONFIG_ESP32_CSI_TOOL_RX_GAIN_CTRL
#  define CSI_GAIN_CTRL 1
#  include "esp_csi_gain_ctrl.h"
#else
#  define CSI_GAIN_CTRL 0
#endif

#if defined(CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE) && defined(CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE_VALUE)
static inline int csi_gain_force_value(void)
{
    return CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE_VALUE;
}
#else
static inline int csi_gain_force_value(void) { return CSI_FIELD_NA; }
#endif

/*
 * Apply the lock. Until 2026-08-18 RX_GAIN_FORCE only wrote its own value into
 * the gain_force_val column and locked nothing, so a capture could be labelled
 * "gain locked at N" while the hardware ran free. That is worse than not
 * offering the option, because the label is wrong rather than absent. The call
 * below is what makes the label true, and it needs the component: with
 * RX_GAIN_CTRL off the function logs and does nothing, leaving the column at
 * CSI_FIELD_NA.
 *
 * Called from csi_init(). The result is logged either way, because a lock that
 * the driver refused has to be visible before a session is taken under it.
 */
static inline void csi_gain_apply_lock(void)
{
#if defined(CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE) && CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE
#  if CSI_GAIN_CTRL
    uint8_t agc = (uint8_t)CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE_VALUE;
    int8_t fft = (int8_t)CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE_FFT;
    esp_err_t r = esp_csi_gain_ctrl_set_rx_force_gain(agc, fft);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_CSI, "esp_csi_gain_ctrl_set_rx_force_gain(agc=%u, "
                 "fft=%d) failed: %s; the gain is NOT locked and the "
                 "gain_force_val column would misdescribe this capture",
                 (unsigned)agc, (int)fft, esp_err_to_name(r));
    } else {
        ESP_LOGI(TAG_CSI, "receive gain locked at agc=%u fft=%d",
                 (unsigned)agc, (int)fft);
    }
#  else
    ESP_LOGE(TAG_CSI, "RX_GAIN_FORCE is set but RX_GAIN_CTRL is not, so no "
             "lock is applied; enable ESP32_CSI_TOOL_RX_GAIN_CTRL");
#  endif
#endif
}

/* Per-record receive gain, schema v2.2. Both return CSI_FIELD_NA when the
 * component is not linked, so a v2.2 capture taken without it stays
 * distinguishable from one taken with it and reading a genuine zero. */
static inline int csi_rx_agc_gain(const void *rx_ctrl)
{
#if CSI_GAIN_CTRL
    uint8_t agc = 0;
    int8_t fft = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc, &fft);
    return (int)agc;
#else
    (void)rx_ctrl;
    return CSI_FIELD_NA;
#endif
}

static inline int csi_rx_fft_gain(const void *rx_ctrl)
{
#if CSI_GAIN_CTRL
    uint8_t agc = 0;
    int8_t fft = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc, &fft);
    return (int)fft;
#else
    (void)rx_ctrl;
    return CSI_FIELD_NA;
#endif
}

char *project_type;

/*
 * Bytes of the 802.11 MAC header deep-copied out of the driver buffer.
 * 24 B covers Frame Control (2) + Duration (2) + Addr1/2/3 (18) + Sequence
 * Control (2) for a standard data/management frame. Frame-type decoding only
 * needs byte 0, but keeping Addr2 lets downstream tooling attribute a frame to
 * a specific transmitter (e.g. tell two co-channel APs' beacons apart).
 */
#define CSI_HDR_COPY_LEN 24

/*
 * Frame container kept on the queue. We copy data->buf into a fixed-size
 * scratch space because the wifi driver reuses its buffer right after the
 * callback returns; the pointer carried in info.buf points inside the queue
 * item itself so format_csi_csv_line() can read both metadata and payload
 * from one struct.
 *
 * The same reuse hazard applies to info.hdr / info.payload: memcpy of
 * wifi_csi_info_t is a SHALLOW copy, so those pointers would dangle by the
 * time the consumer task runs. We therefore deep-copy the MAC header bytes we
 * actually use into hdr_copy and never dereference info.hdr downstream.
 */
typedef struct {
    int64_t          timestamp_us;                 /* esp_timer, us since boot, stamped in cb */
    double           real_ts;                      /* steady clock seconds, stamped in cb */
    uint32_t         broadcast_seq;
    wifi_csi_info_t  info;
    uint16_t         rx_seq;                       /* 802.11 Sequence Control, RX side */
    uint8_t          hdr_valid;                    /* 1 if hdr_copy holds real bytes */
    uint8_t          hdr_copy[CSI_HDR_COPY_LEN];
    uint16_t         buf_len;
    int8_t           first_word_invalid;           /* schema v2.1 */
    int8_t           rx_estimate_vld;              /* schema v2.1, NA on non-HE */
    int8_t           buf_data[CSI_MAX_BUF_LEN];
} csi_queue_item_t;

static QueueHandle_t s_csi_queue          = NULL;
static TaskHandle_t  s_csi_consumer_task  = NULL;
static uint64_t      s_csi_rx_count       = 0;     // CSI callback invocations
static uint64_t      s_csi_cb_reject      = 0;     // invocations rejected before enqueue
static uint64_t      s_csi_drop_count     = 0;
static uint64_t      s_csi_layout_drop    = 0;     // frames discarded by chip_aware_should_drop()
static uint32_t      s_broadcast_seq      = 0;     // updated by gptimer_broadcast.h on AP

/* Acquisition-contract additions (2026-09-15; audit items I11/I17).
 * format_fail: format_csi_csv_line returned an error, so a dequeued frame
 * produced no CSV row. sink_fail: at least one configured sink reported a
 * detectable write failure for a row. Both increment in the consumer task.
 * session_id: one value per boot, printed in the banner and every DIAG
 * frame so windows never straddle a reset silently. The stop flags carry
 * the termination protocol: csi_capture_stop() disables the driver
 * callback and raises `stopping`; the consumer drains the queue, emits a
 * terminal DIAG_END frame with a drained marker, flushes the sinks, and
 * raises `stopped`. */
static uint64_t      s_csi_format_fail    = 0;
static uint64_t      s_csi_sink_fail      = 0;
static int64_t       s_session_id         = 0;
static volatile bool s_capture_stopping   = false;
static volatile bool s_capture_stopped    = false;

#if defined(CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER)
/*
 * Static-queue backing storage for the PSRAM ring buffer path. The queue
 * descriptor (StaticQueue_t, ~80 B) lives in internal RAM so the kernel can
 * read it without crossing the PSRAM bus on every queue op; only the bulk
 * item array is placed in PSRAM. This matches the IDF guidance for
 * "small control struct + large data buffer" patterns.
 */
static StaticQueue_t s_csi_queue_static;
static uint8_t      *s_csi_queue_storage  = NULL;  /* heap_caps_malloc(MALLOC_CAP_SPIRAM) */
#endif

void csi_set_broadcast_seq(uint32_t seq) {
    s_broadcast_seq = seq;
}

uint64_t csi_get_drop_count(void) {
    return s_csi_drop_count;
}

uint64_t csi_get_layout_drop_count(void) {
    return s_csi_layout_drop;
}

/*
 * Decode the 802.11 frame type from the first byte of the MAC header.
 *
 * Frame Control byte 0 layout (IEEE 802.11-2020 §9.2.4.1):
 *   bits [1:0] = Protocol Version (always 00 in our captures)
 *   bits [3:2] = Type (00=Mgmt, 01=Ctrl, 10=Data, 11=Reserved)
 *   bits [7:4] = Subtype (ACK = subtype 13 within Ctrl class)
 *
 * Returns a short uppercase string suitable for direct CSV emission. ACK is
 * separated from other Ctrl frames so users can subset captures by
 * `frame_type == 'ACK'` after enabling `acquire_csi_dump_ack_en`.
 */
static inline const char *_frame_type_name(const uint8_t *hdr) {
    if (hdr == NULL) return "?";
    uint8_t fc0 = hdr[0];
    uint8_t type = (fc0 >> 2) & 0x3;
    uint8_t subtype = (fc0 >> 4) & 0xF;
    switch (type) {
        case 0: return "MGMT";
        case 1: return (subtype == 13) ? "ACK" : "CTRL";
        case 2: return "DATA";
        default: return "RSVD";
    }
}

/*
 * Format one CSV row in v2 super-set schema.
 * Returns the number of bytes written (excluding NUL), or -1 on overflow.
 */
static int format_csi_csv_line(char *out, size_t out_len, const csi_queue_item_t *item) {
    const wifi_csi_info_t *d = &item->info;
    char mac[20] = {0};
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             d->mac[0], d->mac[1], d->mac[2], d->mac[3], d->mac[4], d->mac[5]);

    csi_layout_id_t layout = chip_aware_infer_layout(d);
    const char *layout_name = chip_aware_layout_name(layout);

#if defined(CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE)
    int he_stbc_mode = CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE;
#else
    int he_stbc_mode = 2;
#endif

#if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
    int acquire_force_lltf = 1;
#else
    int acquire_force_lltf = 0;
#endif

    // Session-level flag: replicated per row. ESP-IDF v6.0.1 does not expose
    // a per-frame "this packet was beamformed" bit, so a row value of 0
    // means "no beamformed frames present in this capture" (driver-level
    // filter); a value of 1 means "capture may contain a mix" - downstream
    // tooling cannot per-frame-distinguish but can decide at file scope.
    // See docs/known_issues.md for the sensing-task feature-corruption table.
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED
    int acquire_beamformed = 1;
#else
    int acquire_beamformed = 0;
#endif

    int he_ltf_present = (layout == CSI_LAYOUT_HE_LTF_SU || layout == CSI_LAYOUT_HE_LTF_MU) ? 1 : 0;

    /* Read the deep-copied header, never item->info.hdr (nulled in the callback
     * because the driver reuses that buffer immediately after it returns). */
    const char *frame_type_name =
        item->hdr_valid ? _frame_type_name(item->hdr_copy) : "?";

    // ----------------------------------------------------------------------
    // v2 schema layout: 25 v1 columns, then CSI_DATA at column 25, then 8 v2
    // columns appended at the end. This preserves all v1 Python tooling that
    // splits by comma and indexes column 25 (e.g. serial_plot_csi_live.py)
    // bit-for-bit. v1 regex tools (parse_csi.py) keep working transparently.
    // See docs/csv_format_v2.md.
    // ----------------------------------------------------------------------

    // ... 25 v1 metadata columns (preserved from Hernandez 2020) ...
    int n = snprintf(out, out_len,
        "CSI_DATA,%s,%s,"
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%lu,%d,%d,%d,"
        "%d,%.6f,%d,",
        project_type, mac,
        /* All PHY metadata goes through rxctrl_compat.h: wifi_pkt_rx_ctrl_t is
         * a different struct on HE-capable SoCs (C5/C6/C61) and most of these
         * fields simply do not exist there. Accessors return -1 (CSI_FIELD_NA)
         * for a field the SoC cannot report, so "unsupported" stays
         * distinguishable from a genuine measurement of 0. */
        csi_rx_rssi(&d->rx_ctrl), csi_rx_rate(&d->rx_ctrl), csi_rx_sig_mode(&d->rx_ctrl),
        csi_rx_mcs(&d->rx_ctrl), csi_rx_bandwidth(&d->rx_ctrl), csi_rx_smoothing(&d->rx_ctrl),
        csi_rx_not_sounding(&d->rx_ctrl), csi_rx_aggregation(&d->rx_ctrl), csi_rx_stbc(&d->rx_ctrl),
        csi_rx_fec_coding(&d->rx_ctrl), csi_rx_sgi(&d->rx_ctrl),
        csi_rx_noise_floor(&d->rx_ctrl), csi_rx_ampdu_cnt(&d->rx_ctrl),
        csi_rx_channel(&d->rx_ctrl), csi_rx_secondary_channel(&d->rx_ctrl),
        csi_rx_timestamp(&d->rx_ctrl), csi_rx_ant(&d->rx_ctrl),
        csi_rx_sig_len(&d->rx_ctrl), csi_rx_rx_state(&d->rx_ctrl),
        real_time_set, item->real_ts, d->len);
    if (n < 0 || (size_t)n >= out_len) return -1;

    // ... CSI_DATA payload at column 25 (v1 position) ...
    int data_len = item->buf_len;
#if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
    if (data_len > 128) data_len = 128;   // LLTF only: 64 subcarriers * 2
#endif

    out[n++] = '[';
    if ((size_t)n >= out_len) return -1;

    int m;
#if CSI_RAW
    for (int i = 0; i < data_len; i++) {
        m = snprintf(out + n, out_len - n, "%d ", (int)item->buf_data[i]);
        if (m < 0 || (size_t)(n + m) >= out_len) return -1;
        n += m;
    }
#elif CSI_AMPLITUDE
    for (int i = 0; i < data_len / 2; i++) {
        int amp = (int)sqrt((double)(item->buf_data[i*2] * item->buf_data[i*2] +
                                     item->buf_data[i*2+1] * item->buf_data[i*2+1]));
        m = snprintf(out + n, out_len - n, "%d ", amp);
        if (m < 0 || (size_t)(n + m) >= out_len) return -1;
        n += m;
    }
#elif CSI_PHASE
    for (int i = 0; i < data_len / 2; i++) {
        int phase = (int)atan2((double)item->buf_data[i*2], (double)item->buf_data[i*2+1]);
        m = snprintf(out + n, out_len - n, "%d ", phase);
        if (m < 0 || (size_t)(n + m) >= out_len) return -1;
        n += m;
    }
#endif

    // ... close-bracket (NO trailing newline yet; v2 columns follow) ...
    m = snprintf(out + n, out_len - n, "]");
    if (m < 0 || (size_t)(n + m) >= out_len) return -1;
    n += m;

    // ... 12 v2 columns appended at the end (column index 26-37) ...
    // Trailing-column extension: timestamp_us (the 64-bit esp_timer stamp
    // taken in the CSI callback) comes after rx_seq, so every parser
    // written against the earlier 36-column layout keeps working
    // unchanged. Column 18 stays the v1-compat 32-bit rx_ctrl counter
    // (wraps at 2^32 us); column 23 stays the steady-clock double; this
    // column is the integer-microsecond form of the same 64-bit clock.
    // Schema v2.1 appends four more columns after timestamp_us, again at the
    // end, so both the v1 parser and any consumer written against the
    // 38-column v2.0 layout keep working: first_word_invalid and
    // rx_estimate_vld are the per-record validity flags the vendor documents
    // for the MAC v3 parts, gain_force_val is the receive-gain state this
    // build applied (CSI_FIELD_NA when the gain runs free, which is the
    // default), and die_temp_c10 is the on-die temperature in tenths of a
    // degree, sampled on a slow timer rather than per record.
    m = snprintf(out + n, out_len - n,
        ",%s,%s,%d,%s,%lu,%d,%d,%d,%d,%s,%u,%lld,%d,%d,%d,%d,%d,%d\n",
        /* Fixed tool name, no suffix: ESP32 is the hardware family. Schema
         * generation is carried by the column count (and CSI_FORMAT_VERSION
         * in the banner), never by this string. Firmware built before
         * 2026-08-19 wrote a ".N" schema minor after the name. */
        CONFIG_IDF_TARGET, "VeriCSI-ESP32",
        item->buf_len, layout_name,
        (unsigned long)item->broadcast_seq,
        he_ltf_present, he_stbc_mode, acquire_force_lltf,
        acquire_beamformed, frame_type_name,
        (unsigned)item->rx_seq,
        (long long)item->timestamp_us,
        item->first_word_invalid, item->rx_estimate_vld,
        csi_gain_force_value(), die_temp_read_c10(),
        csi_rx_agc_gain(&d->rx_ctrl), csi_rx_fft_gain(&d->rx_ctrl));
    if (m < 0 || (size_t)(n + m) >= out_len) return -1;
    n += m;

    return n;
}

/*
 * Wi-Fi rx callback. Per the v6 Wi-Fi driver guide this is invoked from a
 * deferred task context (NOT ISR), so xQueueSend with 0 ticks is correct.
 * We deliberately use a non-blocking send so that a stalled consumer never
 * back-pressures the Wi-Fi driver; dropped frames are counted instead.
 */
static void _wifi_csi_cb(void *ctx, wifi_csi_info_t *data) {
    /* Count every invocation BEFORE any early return. This is the only figure
     * that distinguishes "the driver never delivered a frame" from "frames
     * arrived but were dropped downstream", which is why it is incremented
     * ahead of the null checks rather than alongside the enqueue. */
    s_csi_rx_count++;

#if CONFIG_ESP32_CSI_TOOL_FI_REJECT_EVERY_N > 0
    /* Fault injection (test builds only): treat every Nth invocation as a
     * pre-enqueue reject, at the same classification site as a real one. */
    if (s_csi_rx_count % CONFIG_ESP32_CSI_TOOL_FI_REJECT_EVERY_N == 0) {
        s_csi_cb_reject++;
        return;
    }
#endif

    if (s_csi_queue == NULL || data == NULL || data->buf == NULL) {
        s_csi_cb_reject++;
        return;
    }

    csi_queue_item_t item;
    item.timestamp_us  = esp_timer_get_time();
    item.real_ts       = get_steady_clock_timestamp();
    item.broadcast_seq = s_broadcast_seq;
    memcpy(&item.info, data, sizeof(wifi_csi_info_t));
    item.buf_len = data->len > CSI_MAX_BUF_LEN ? CSI_MAX_BUF_LEN : data->len;
    item.first_word_invalid = (int8_t)csi_first_word_invalid(data);
    item.rx_estimate_vld    = (int8_t)csi_rx_estimate_valid(&data->rx_ctrl);
    memcpy(item.buf_data, data->buf, item.buf_len);
    item.info.buf = item.buf_data;

    /*
     * RX-side sequence number straight out of the 802.11 MAC header. Unlike
     * broadcast_seq (TX side, AP only) this is present on every received frame
     * on every SoC, so it is the only field that lets a receiver detect its own
     * frame loss. Field is unconditional in wifi_csi_info_t on all targets.
     */
    item.rx_seq = data->rx_seq;

    /*
     * Deep-copy the MAC header before the driver reuses its buffer. Without
     * this the consumer task would dereference a stale data->hdr pointer.
     *
     * The copy is bounded by sig_len because short frames are shorter than a
     * full 24 B header (an ACK is 10 B + FCS), and we must not read past the
     * frame. Frame-type decoding only needs byte 0, so a short copy still
     * yields a correct frame_type; the extra bytes are a bonus for transmitter
     * attribution on frames that actually carry them.
     */
    memset(item.hdr_copy, 0, CSI_HDR_COPY_LEN);
    item.hdr_valid = 0;
    if (data->hdr != NULL) {
        int sig_len = csi_rx_sig_len(&data->rx_ctrl);
        size_t hdr_n = (sig_len > 0 && sig_len < CSI_HDR_COPY_LEN)
                           ? (size_t)sig_len : (size_t)CSI_HDR_COPY_LEN;
        memcpy(item.hdr_copy, data->hdr, hdr_n);
        item.hdr_valid = 1;
    }

    /* info.hdr / info.payload point into driver memory that is about to be
     * reused. Null them so no downstream code can dereference them by mistake. */
    item.info.hdr     = NULL;
    item.info.payload = NULL;

#if CONFIG_ESP32_CSI_TOOL_FI_QUEUE_DROP_EVERY_N > 0
    /* Fault injection (test builds only): count every Nth frame as a queue
     * drop without enqueueing it, at the same classification site. */
    if (s_csi_rx_count % CONFIG_ESP32_CSI_TOOL_FI_QUEUE_DROP_EVERY_N == 0) {
        s_csi_drop_count++;
        return;
    }
#endif

    if (xQueueSend(s_csi_queue, &item, 0) != pdTRUE) {
        s_csi_drop_count++;
    }
#if CONFIG_ESP32_CSI_TOOL_FI_DUP_EVERY_N > 0
    /* Fault injection (test builds only): enqueue every Nth frame twice.
     * The duplicate is deliberately NOT counted anywhere: it is the anomaly
     * the sequence-level audit must catch despite count cancellation. */
    else if (s_csi_rx_count % CONFIG_ESP32_CSI_TOOL_FI_DUP_EVERY_N == 0) {
        (void)xQueueSend(s_csi_queue, &item, 0);
    }
#endif
}

/*
 * Diagnostic frame (O-1). Emitted as a `#`-prefixed comment line so all v1
 * and v2 parsers transparently skip it. Carries cumulative drop counts,
 * heap stats and (on AP) broadcast TX counters.
 */
/*
 * CPU load over the last diagnostic interval, in tenths of a percent.
 *
 * The vendor datasheets quote receive current with the CPU idle and the
 * peripherals disabled, which is not the condition a CSI capture runs in, so
 * that figure bounds a capture from below and says nothing about how far
 * below. This is the missing term. It is not energy: these parts expose no
 * current or power sensor, so nothing on the device can measure consumption,
 * and an energy number needs an external meter.
 *
 * FreeRTOS reports idle time cumulatively since boot, which flattens as a
 * capture runs. Differencing the idle counter over one diagnostic interval
 * gives the load for that interval instead, which is what a per-window audit
 * needs. Enabled by ESP32_CSI_TOOL_DIAG_CPU_LOAD, which selects the FreeRTOS
 * run-time stats it depends on; without it the field reports NA rather than
 * a fabricated value.
 */
#if defined(CONFIG_ESP32_CSI_TOOL_DIAG_CPU_LOAD) && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
static uint32_t s_prev_idle_ticks;
static int64_t  s_prev_load_us;

/* ulTaskGetIdleRunTimeCounter() reports the calling core's idle task alone:
 * ESP-IDF forwards it to ulTaskGetIdleRunTimeCounterForCore(portGET_CORE_ID())
 * (freertos_tasks_c_additions.h). Dividing one core's idle time by a span
 * scaled to the whole core count put every dual-core reading about 50 points
 * too high, which is what an ESP32-S3 measured at 62.6% against 24 to 25% on
 * the single-core RISC-V parts exposed on 2026-08-17. The sum over cores is
 * taken explicitly here. Where the per-core accessor is unavailable the field
 * reports not-available on multi-core parts. */
#if !defined(CONFIG_FREERTOS_SMP) && (INCLUDE_xTaskGetIdleTaskHandle == 1)
#define CSI_IDLE_PER_CORE 1
#else
#define CSI_IDLE_PER_CORE 0
#endif

static int csi_cpu_load_permille(void)
{
#if !CSI_IDLE_PER_CORE && CONFIG_FREERTOS_NUMBER_OF_CORES > 1
    return CSI_FIELD_NA;
#else
    int64_t now_us = esp_timer_get_time();
    uint32_t idle = 0;
#if CSI_IDLE_PER_CORE
    for (int c = 0; c < CONFIG_FREERTOS_NUMBER_OF_CORES; c++) {
        idle += (uint32_t)ulTaskGetIdleRunTimeCounterForCore((BaseType_t)c);
    }
#else
    idle = (uint32_t)ulTaskGetIdleRunTimeCounter();
#endif
    int64_t d_us = now_us - s_prev_load_us;
    uint32_t d_idle = idle - s_prev_idle_ticks;
    s_prev_load_us = now_us;
    s_prev_idle_ticks = idle;
    if (d_us <= 0) return CSI_FIELD_NA;
    /* The run-time counter ticks at the esp_timer rate, so the idle delta and
     * the wall-clock delta share units. The span covers every core, matching
     * the summed idle delta above. */
    int64_t span = d_us * CONFIG_FREERTOS_NUMBER_OF_CORES;
    if (span <= 0) return CSI_FIELD_NA;
    int64_t busy = span - (int64_t)d_idle;
    if (busy < 0) busy = 0;
    if (busy > span) busy = span;
    return (int)((busy * 1000) / span);
#endif
}
#else
static int csi_cpu_load_permille(void) { return CSI_FIELD_NA; }
#endif

/* `terminal` distinguishes the periodic heartbeat ("# DIAG") from the
 * end-of-capture summary ("# DIAG_END"), which additionally states whether
 * the queue was drained when it was written. Only a DIAG_END with
 * drained=1 certifies completeness of the tail; an ordinary trailing DIAG
 * does not (audit item I17). */
static void emit_diag_frame(bool terminal) {
#if CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S > 0
    char buf[416];
    /* Radio channel and total frames delivered to the callback. Without these
     * a capture that received nothing is indistinguishable from a capture that
     * was never tuned to the right channel - the ambiguity that stalled the
     * 2026-07-30 broadcast-visibility investigation. */
    uint8_t diag_ch = 0;
    wifi_second_chan_t diag_sec = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&diag_ch, &diag_sec) != ESP_OK) {
        diag_ch = 0;
    }
    unsigned qwait = s_csi_queue != NULL
        ? (unsigned)uxQueueMessagesWaiting(s_csi_queue) : 0;
    int n = snprintf(buf, sizeof(buf),
        "# %s,timestamp_us=%lld,csi_rx=%llu,cb_reject=%llu,csi_drop=%llu,layout_drop=%llu,"
        "channel=%u,heap_free=%u,heap_min=%u"
        ",queue_wait=%u,format_fail=%llu,sink_fail=%llu,session=%lld",
        terminal ? "DIAG_END" : "DIAG",
        (long long)esp_timer_get_time(),
        (unsigned long long)s_csi_rx_count,
        (unsigned long long)s_csi_cb_reject,
        (unsigned long long)s_csi_drop_count,
        (unsigned long long)s_csi_layout_drop,
        (unsigned)diag_ch,
        (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        qwait,
        (unsigned long long)s_csi_format_fail,
        (unsigned long long)s_csi_sink_fail,
        (long long)s_session_id);
    if (n < 0 || (size_t)n >= sizeof(buf)) return;
    if (terminal) {
        int mt = snprintf(buf + n, sizeof(buf) - n, ",drained=%d",
                          qwait == 0 ? 1 : 0);
        if (mt < 0 || (size_t)(n + mt) >= sizeof(buf)) return;
        n += mt;
    }
#  ifdef CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
    int m = snprintf(buf + n, sizeof(buf) - n,
        ",gptimer_tx=%llu,gptimer_fail=%llu,gptimer_seq=%lu",
        (unsigned long long)gptimer_broadcast_tx_count(),
        (unsigned long long)gptimer_broadcast_tx_failures(),
        (unsigned long)gptimer_broadcast_seq());
    if (m < 0 || (size_t)(n + m) >= sizeof(buf)) return;
    n += m;
#  endif
    /* Appended last on purpose: compile_ledger.py matches the DIAG line by
     * prefix, so a field inserted in the middle would break the frozen ledger. */
    int mc = snprintf(buf + n, sizeof(buf) - n, ",cpu_load_permille=%d",
                      csi_cpu_load_permille());
    if (mc < 0 || (size_t)(n + mc) >= sizeof(buf)) return;
    n += mc;
    int m2 = snprintf(buf + n, sizeof(buf) - n, "\n");
    if (m2 < 0 || (size_t)(n + m2) >= sizeof(buf)) return;
    n += m2;
    sink_write(buf, (size_t)n);
#endif
}

static void csi_consumer_task(void *arg) {
    csi_queue_item_t item;
    static char line_buf[CSI_LINE_BUF_LEN];
    int drain_empty_passes = 0;
#if CONFIG_ESP32_CSI_TOOL_CAPTURE_DURATION_S > 0
    const int64_t task_start_us = esp_timer_get_time();
#endif

    /*
     * Diagnostic timer state. xQueueReceive uses a small timeout (100 ms) so
     * the task wakes regularly enough to emit diagnostic frames even when
     * CSI traffic is silent.
     */
    int64_t next_diag_us = esp_timer_get_time();
#if CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S > 0
    const int64_t diag_period_us =
        (int64_t)CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S * 1000000LL;
#endif

    uint32_t frames_since_yield = 0;

    for (;;) {
        BaseType_t got = xQueueReceive(s_csi_queue, &item, pdMS_TO_TICKS(100));

        if (got == pdTRUE) {
            /* Drop frames flagged as broken by chip_aware_decoder (e.g. C5 5 GHz #18493).
             * Counted separately from queue-full drops so that loss accounting
             * closes: csi_rx = cb_reject + csi_drop + layout_drop
             *                  + format_fail + sink_fail + delivered rows. */
            csi_layout_id_t layout = chip_aware_infer_layout(&item.info);
#if CONFIG_ESP32_CSI_TOOL_FI_LAYOUT_DROP_EVERY_N > 0
            /* Fault injection (test builds only): classify every Nth
             * dequeued frame as a layout drop, at the same site. */
            static uint64_t s_fi_layout_ctr = 0;
            if (++s_fi_layout_ctr % CONFIG_ESP32_CSI_TOOL_FI_LAYOUT_DROP_EVERY_N == 0) {
                s_csi_layout_drop++;
            } else
#endif
            if (chip_aware_should_drop(layout)) {
                s_csi_layout_drop++;
            } else {
                int n = format_csi_csv_line(line_buf, sizeof(line_buf), &item);
                if (n > 0) {
#if CONFIG_ESP32_CSI_TOOL_FI_SINK_FAIL_EVERY_N > 0
                    /* Fault injection (test builds only): drop every Nth
                     * formatted row before the sink and count it, producing
                     * a host-visible gap matched by the counter. */
                    static uint64_t s_fi_sink_ctr = 0;
                    if (++s_fi_sink_ctr % CONFIG_ESP32_CSI_TOOL_FI_SINK_FAIL_EVERY_N == 0) {
                        s_csi_sink_fail++;
                    } else
#endif
                    if (sink_write(line_buf, (size_t)n) != 0) {
                        s_csi_sink_fail++;
                    }
                } else {
                    /* A dequeued frame that produced no CSV row was invisible
                     * to every counter before 2026-09-15 (audit item I11). */
                    s_csi_format_fail++;
                }
#ifdef CONFIG_ESP32_CSI_TOOL_SINK_BINARY
                /* Emit binary frame in addition to CSV. The two coexist on the
                 * same stdout stream because grep CSI_DATA still matches only
                 * CSV lines; v2 binary parsers detect frames by CSIF magic. */
                binary_sink_write_frame(&item);
#endif
            }

            /* O-6: cooperative yield under sustained load so that other tasks
             * (Wi-Fi housekeeping, OTA, user input) get scheduling time. With
             * the queue never empty the receive above never blocks, and
             * taskYIELD() passes the core only to tasks of equal or higher
             * priority. The idle task then starved, the task watchdog fired
             * every 5 s, and its interrupt-context message cut a row in the
             * UART console stream (B1, 2026-10-01). A one-tick delay lets
             * every lower-priority task run. */
#if CONFIG_ESP32_CSI_TOOL_CONSUMER_YIELD_EVERY > 0
            if (++frames_since_yield >= CONFIG_ESP32_CSI_TOOL_CONSUMER_YIELD_EVERY) {
                frames_since_yield = 0;
                vTaskDelay(1);
            }
#endif
        }

        /* O-1: time-based diagnostic frame emission. Independent of CSI
         * traffic: even when no CSI is being received we still want a
         * heart-beat line every diag period. */
#if CONFIG_ESP32_CSI_TOOL_DIAG_FRAME_PERIOD_S > 0
        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_diag_us) {
            die_temp_sample();          /* slow sensor, sampled off the CSI path */
            emit_diag_frame(false);
            next_diag_us = now_us + diag_period_us;
        }
#endif

#if CONFIG_ESP32_CSI_TOOL_CAPTURE_DURATION_S > 0
        /* Finite capture window: after the configured duration, stop
         * admitting callbacks and enter the termination protocol below.
         * Used by the accounting-validation bench for windows whose tail
         * is provably drained. */
        if (!s_capture_stopping && esp_timer_get_time() - task_start_us >=
                (int64_t)CONFIG_ESP32_CSI_TOOL_CAPTURE_DURATION_S * 1000000LL) {
            (void)esp_wifi_set_csi(false);
            s_capture_stopping = true;
        }
#endif

        /* Termination protocol: once stopping is requested and the callback
         * is disabled, the queue can only shrink. Requiring the queue to
         * read empty on two consecutive passes (the receive timeout is
         * 100 ms) outlasts any callback that was already executing when the
         * disable landed, so the terminal DIAG_END is ordered after the
         * last data row in every sink. Flush, mark stopped, park the task. */
        if (s_capture_stopping && !s_capture_stopped) {
            if (s_csi_queue == NULL
                    || uxQueueMessagesWaiting(s_csi_queue) == 0) {
                if (++drain_empty_passes >= 2) {
                    emit_diag_frame(true);
                    sink_flush();
                    s_capture_stopped = true;
                    vTaskSuspend(NULL);
                }
            } else {
                drain_empty_passes = 0;
            }
        }
    }
}

void _print_csi_csv_header() {
    char hdr[640];
    int n = snprintf(hdr, sizeof(hdr),
        "# CSI_TOOL_VERSION=2.0\n"
        "# CSI_FORMAT_VERSION=2\n"
        "# CHIP=%s\n"
        "# IDF_VERSION=%s\n"
        "# RECORDING_SESSION_ID=startup-%lld\n"
#if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
        "# CSI_FORCE_LLTF=1\n"
#else
        "# CSI_FORCE_LLTF=0\n"
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU
        "# CSI_ACQUIRE_HE_SU=1\n"
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU
        "# CSI_ACQUIRE_HE_MU=1\n"
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED
        "# CSI_ACQUIRE_BEAMFORMED=1\n"
#else
        "# CSI_ACQUIRE_BEAMFORMED=0\n"
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_DCM) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_DCM
        "# CSI_ACQUIRE_DCM=1\n"
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES) && CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES
        "# CSI_DUMP_ACK_FRAMES=1\n"
#endif
        ,
        CONFIG_IDF_TARGET, IDF_VER,
        (long long)(s_session_id != 0
                        ? s_session_id
                        : (s_session_id = esp_timer_get_time())));
    if (n > 0 && (size_t)n < sizeof(hdr)) {
        sink_write(hdr, (size_t)n);
    }

    // When active_ap is built with the application-layer
    // gptimer broadcast enabled, surface the configured target rate as a
    // self-describing boot comment so post-hoc analyzers (e.g.
    // experiments/e5_packet_rate/packet_rate_analyzer.py) can read the
    // intended rate directly from the capture without an external metadata
    // file. Only emitted on active_ap with gptimer broadcast on; absent on
    // active_sta / passive (the Kconfig itself is only declared there).
#if defined(CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST) && CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
    {
        /* Emit the PERIOD as the authoritative value: whole-Hz cannot express
         * rates near the beacon interval (102400 us = 9.765625 Hz), so the
         * period is what the timer was actually armed with. The derived rate is
         * written alongside it for readability and v1 tooling compatibility. */
#  if defined(CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US) && CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US > 0
        const long bcast_period_us = (long)CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US;
#  else
        const long bcast_period_us =
            (long)(1000000L / (long)CONFIG_ESP32_CSI_TOOL_BROADCAST_RATE_HZ);
#  endif
        char br_hdr[128];
        int br_n = snprintf(br_hdr, sizeof(br_hdr),
                            "# CSI_BROADCAST_PERIOD_US=%ld\n"
                            "# CSI_BROADCAST_RATE_HZ=%.6f\n",
                            bcast_period_us,
                            1000000.0 / (double)bcast_period_us);
        if (br_n > 0 && (size_t)br_n < sizeof(br_hdr)) {
            sink_write(br_hdr, (size_t)br_n);
        }
    }
#endif

    /* SoftAP beacon interval. Only declared on active_ap, and it is the
     * dominant strictly-periodic MAC event in a quiet channel, so record it
     * with the capture rather than relying on external notes. */
#if defined(CONFIG_ESP32_CSI_TOOL_BEACON_INTERVAL_TU)
    {
        char bi_hdr[96];
        int bi_n = snprintf(bi_hdr, sizeof(bi_hdr),
                            "# CSI_BEACON_INTERVAL_TU=%d\n"
                            "# CSI_BEACON_RATE_HZ=%.6f\n",
                            CONFIG_ESP32_CSI_TOOL_BEACON_INTERVAL_TU,
                            1000000.0 /
                                ((double)CONFIG_ESP32_CSI_TOOL_BEACON_INTERVAL_TU * 1024.0));
        if (bi_n > 0 && (size_t)bi_n < sizeof(bi_hdr)) {
            sink_write(bi_hdr, (size_t)bi_n);
        }
    }
#endif

    // v2 super-set column header.
    // CSI_DATA stays at column index 25 (zero-indexed) to keep existing v1
    // Python tools that split-by-comma and index column 25 working unchanged
    // (e.g. serial_plot_csi_live.py). The 8 v2 columns are appended after.
    static const char col_hdr[] =
        "type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,"
        "aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,"
        "local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,"
        "CSI_DATA,"
        "chip_id,firmware_version,csi_buf_len,subcarrier_layout_id,gptimer_seq,"
        "he_ltf_present,he_stbc_mode,acquire_force_lltf,acquire_beamformed,frame_type,"
        "rx_seq,timestamp_us,"
        /* schema v2.1. These four were emitted in every record from the day
         * the schema shipped, but this header string was not extended with
         * them, so a capture announced 38 columns and delivered 42. Fixed
         * 2026-08-16 after a positional parser read the wrong fields. */
        "first_word_invalid,rx_estimate_vld,gain_force_val,die_temp_c10,"
        /* schema v2.2, appended at the end like every generation before it.
         * agc_gain and fft_gain are the per-record receive gain read through
         * the esp_csi_gain_ctrl component; both read -1 when the component is
         * not linked, which is what separates a free-gain capture from a
         * locked one without the consumer having to guess. */
        "agc_gain,fft_gain\n";
    sink_write(col_hdr, sizeof(col_hdr) - 1);
}

/*
 * Termination protocol entry (2026-09-15, acquisition contract). Disables
 * the driver CSI callback and asks the consumer to drain and finalize: the
 * consumer empties the queue, emits a terminal DIAG_END frame whose
 * drained marker certifies the tail, flushes every sink, and parks itself.
 * Returns true once the terminal frame has been written; false on a 5 s
 * timeout, in which case the capture must be treated as Incomplete. The
 * duration-based variant (CONFIG_ESP32_CSI_TOOL_CAPTURE_DURATION_S) runs
 * the same protocol from the consumer itself.
 */
static inline bool csi_capture_stop(void) {
    (void)esp_wifi_set_csi(false);
    s_capture_stopping = true;
    for (int i = 0; i < 50; i++) {
        if (s_capture_stopped) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return s_capture_stopped;
}

void csi_init(char *type) {
    project_type = type;

    /* The boot-session identity must exist before the consumer task can emit
     * its first DIAG frame. It used to be assigned lazily by the CSV header;
     * on one B1 boot (ESP32-C61, 2026-10-02) the consumer's first receive
     * timeout fired 0.5 ms before the header was printed, the opening DIAG
     * frame carried session=0, and the strict verdict reported a reset inside
     * the window. The header prints this same value. */
    if (s_session_id == 0) {
        s_session_id = esp_timer_get_time();
    }

    /* Apply the receive-gain lock before the first record is produced, so no
     * record carries a gain_force_val the hardware was not yet honouring. */
    csi_gain_apply_lock();

    /*
     * Bring the output sinks up before anything can be written to them.
     * This call used to be missing entirely: sink_write() was reached from
     * four sites but sink_init() from none. Harmless for the UART sink
     * (printf goes to an already-initialised console) and for SD (sd_init()
     * has its own call site), but the UDP sink's socket stayed at its -1
     * sentinel forever, and udp_sink_write() silently returns on
     * socket < 0 - so the UDP sink had never delivered a single datagram.
     * Found on hardware 2026-08-02 while validating the UDP sink: CSI flowing at
     * 100 fps, association fine, zero device-side errors, zero datagrams at
     * the host. sink_init() is idempotent per boot and cheap.
     */
    sink_init();
    die_temp_init();   /* schema v2.1: on-die sensor, NA on parts without one */

#ifdef CONFIG_SHOULD_COLLECT_CSI

    // Allocate queue and spawn the consumer task BEFORE installing the rx
    // callback so that no frame can arrive while the queue is still NULL.
#if defined(CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER)
#  if !CSI_HAS_PSRAM_ENABLED
#    error "CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER=y but CONFIG_SPIRAM=n; enable PSRAM in sdkconfig.defaults.<target> first"
#  endif
    /* Allocate the bulk item-storage array in PSRAM. Total size:
     *   CSI_QUEUE_DEPTH * sizeof(csi_queue_item_t)
     * e.g. 4096 * ~500 B = ~2 MB on ESP32-S3 with Octal PSRAM. Fits well
     * inside the 8 MB / 16 MB Octal modules. */
    const size_t psram_storage_bytes = (size_t)CSI_QUEUE_DEPTH * sizeof(csi_queue_item_t);
    s_csi_queue_storage = (uint8_t *)heap_caps_malloc(psram_storage_bytes, MALLOC_CAP_SPIRAM);
    if (s_csi_queue_storage == NULL) {
        ESP_LOGE(TAG_CSI,
                 "failed to allocate PSRAM ring buffer (%u byte = %d items x %d B); "
                 "falling back to internal-RAM queue with default depth",
                 (unsigned)psram_storage_bytes,
                 CSI_QUEUE_DEPTH, (int)sizeof(csi_queue_item_t));
        s_csi_queue = xQueueCreate(64, sizeof(csi_queue_item_t));
    } else {
        s_csi_queue = xQueueCreateStatic(CSI_QUEUE_DEPTH,
                                         sizeof(csi_queue_item_t),
                                         s_csi_queue_storage,
                                         &s_csi_queue_static);
        if (s_csi_queue != NULL) {
            ESP_LOGI(TAG_CSI,
                     "PSRAM ring buffer: %d items x %d B = %u byte allocated from MALLOC_CAP_SPIRAM",
                     CSI_QUEUE_DEPTH, (int)sizeof(csi_queue_item_t),
                     (unsigned)psram_storage_bytes);
        }
    }
#else
    s_csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_queue_item_t));
#endif
    if (s_csi_queue == NULL) {
        ESP_LOGE(TAG_CSI, "failed to allocate CSI queue (%d * %d byte)",
                 CSI_QUEUE_DEPTH, (int)sizeof(csi_queue_item_t));
        return;
    }

    BaseType_t t = xTaskCreatePinnedToCore(
        csi_consumer_task, "csi_consumer", CSI_CONSUMER_STACK,
        NULL, CSI_CONSUMER_PRIORITY, &s_csi_consumer_task, CSI_CONSUMER_CORE);
    if (t != pdPASS) {
        ESP_LOGE(TAG_CSI, "failed to spawn csi_consumer task");
        return;
    }

    // Configure CSI acquisition. v6.0.1 splits CSI config into two structures
    // depending on CONFIG_SOC_WIFI_HE_SUPPORT (see
    // C:/esp/v6.0.1/esp-idf/components/esp_wifi/include/local/esp_wifi_types_native.h):
    //
    //   - HE chips (C5/C6/C61):
    //     wifi_csi_config_t is a typedef alias of wifi_csi_acquire_config_t
    //     with rich Wi-Fi 6 fields: enable, acquire_csi_legacy,
    //     acquire_csi_force_lltf, acquire_csi_ht20/ht40/vht/su/mu/dcm/
    //     beamformed/he_stbc_mode, val_scale_cfg, dump_ack_en.
    //
    //   - Non-HE chips (ESP32/S2/S3/C3):
    //     wifi_csi_config_t is the legacy v4.x structure with
    //     lltf_en/htltf_en/stbc_htltf2_en/ltf_merge_en/channel_filter_en/
    //     manu_scale/shift/dump_ack_en (NO acquire_csi_* fields).
    //
    // The public API esp_wifi_set_csi_config() takes the chip-appropriate type
    // (it is declared once in esp_wifi.h:1274 with wifi_csi_config_t).
    wifi_csi_config_t cfg = {};

#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)
    // ===== HE-capable chips (C5/C6/C61): wifi_csi_acquire_config_t alias =====
    //
    // The two flags whose #else fallback is 1 (acquire_csi_legacy and
    // acquire_csi_ht20) are declared as Kconfig int, not bool. A bool set to n
    // leaves its macro undefined, which selects the #else branch and turns the
    // slot back ON, so an int with range 0 1 is what makes "off" expressible.
    // The flags whose fallback is 0 or false can stay bool.
    cfg.enable                   = true;
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_LEGACY)
    cfg.acquire_csi_legacy       = CONFIG_ESP32_CSI_TOOL_ACQUIRE_LEGACY;
#  else
    cfg.acquire_csi_legacy       = 1;
#  endif
    /*
     * acquire_csi_force_lltf / acquire_csi_vht / lltf_bit_mode exist only on
     * the MAC v3 variant of wifi_csi_acquire_config_t
     * (esp_wifi_he_types.h:44-65, guarded by CONFIG_SOC_WIFI_MAC_VERSION_NUM
     * == 3). Per ESP-IDF v6.0.1 soc caps: C5 and C61 are MAC v3, C6 is MAC v2.
     * Referencing them unconditionally is what broke the C6 build.
     */
#  if CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3
#    if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
    cfg.acquire_csi_force_lltf   = 1;
#    else
    cfg.acquire_csi_force_lltf   = 0;
#    endif
    /* esp-idf issue #18982 maintainer reply (2026-08-26): this bit selects
     * the L-LTF I/Q width. 0 = 12-bit (the driver default: four bytes per
     * complex value, 26 tones at every second subcarrier, the 106-byte
     * record), 1 = 8-bit (the layout the Wi-Fi driver guide documents).
     * Written explicitly so every capture runs a known, stated mode. */
    cfg.lltf_bit_mode            = 0;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT20)
    cfg.acquire_csi_ht20         = CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT20;
#  else
    cfg.acquire_csi_ht20         = 1;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT40)
    cfg.acquire_csi_ht40         = CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT40;
#  else
    cfg.acquire_csi_ht40         = 0;
#  endif
#  if CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3
#    if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_VHT)
    cfg.acquire_csi_vht          = CONFIG_ESP32_CSI_TOOL_ACQUIRE_VHT;
#    else
    cfg.acquire_csi_vht          = false;
#    endif
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU)
    cfg.acquire_csi_su           = CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU;
#  else
    cfg.acquire_csi_su           = false;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU)
    cfg.acquire_csi_mu           = CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU;
#  else
    cfg.acquire_csi_mu           = false;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_DCM)
    cfg.acquire_csi_dcm          = CONFIG_ESP32_CSI_TOOL_ACQUIRE_DCM;
#  else
    cfg.acquire_csi_dcm          = false;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED)
    cfg.acquire_csi_beamformed   = CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED;
#  else
    cfg.acquire_csi_beamformed   = false;
#  endif
    /* Same field, two spellings: `acquire_csi_he_stbc_mode` on MAC v3,
     * `acquire_csi_he_stbc` on MAC v2. Both are 2-bit and take the same
     * values (0 = HE-LTF1, 1 = HE-LTF2, 2 = sample evenly). */
#  if defined(CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE)
#    define ESP32_CSI_TOOL_HE_STBC_VALUE CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE
#  else
#    define ESP32_CSI_TOOL_HE_STBC_VALUE 2
#  endif
#  if CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3
    cfg.acquire_csi_he_stbc_mode = ESP32_CSI_TOOL_HE_STBC_VALUE;
#  else
    cfg.acquire_csi_he_stbc      = ESP32_CSI_TOOL_HE_STBC_VALUE;
#  endif
    cfg.val_scale_cfg            = CONFIG_ESP32_CSI_TOOL_VAL_SCALE;
#  if defined(CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES)
    cfg.dump_ack_en              = CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES;
#  else
    cfg.dump_ack_en              = false;
#  endif

#else // !CONFIG_SOC_WIFI_HE_SUPPORT
    // ===== Non-HE chips (ESP32/S2/S3/C3): legacy v4.x wifi_csi_config_t =====
    //
    // IMPORTANT: this branch uses Hernandez 2020 known-good values verbatim.
    // On the legacy chip family the driver requires all 4 _en flags = true to
    // deliver CSI on regular reception; turning htltf_en/stbc_htltf2_en off
    // to "force LLTF" at driver level was a mis-interpretation
    // and produced ZERO callbacks in field testing on TTGO T8 V1.7.
    //
    // The "force LLTF" Kconfig (CONFIG_ESP32_CSI_TOOL_FORCE_LLTF /
    // CONFIG_SHOULD_COLLECT_ONLY_LLTF) is honoured at OUTPUT time instead,
    // see format_csi_csv_line() which truncates data_len to 128 bytes when
    // the flag is set. This matches Hernandez 2020 v1 behaviour.
    //
    // channel_filter_en = false: Hernandez 2020 default. Setting this true
    // appears to make the driver discard frames whose channel estimation has
    // not converged enough, which in noisy environments (phone hotspots, etc.)
    // results in zero CSI callbacks.
    cfg.lltf_en              = true;
    cfg.htltf_en             = true;
    cfg.stbc_htltf2_en       = true;
    cfg.ltf_merge_en         = true;
    cfg.channel_filter_en    = false;
    cfg.manu_scale           = false;       // automatic scaling
    cfg.shift                = 0;
#  if defined(CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES)
    cfg.dump_ack_en          = CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES;
#  else
    cfg.dump_ack_en          = false;
#  endif
#endif // CONFIG_SOC_WIFI_HE_SUPPORT

#if defined(CONFIG_ESP32_CSI_TOOL_SINK_USB_CDC) && CONFIG_ESP32_CSI_TOOL_CDC_WAIT_HOST_S > 0
    /* The TinyUSB port carries nothing until the host opens it, so a record
     * produced earlier would be counted but never reach the capture file.
     * Start CSI once the host is attached (or the wait expires), so the
     * file holds the boot session from its header on. */
    for (int i = 0; i < CONFIG_ESP32_CSI_TOOL_CDC_WAIT_HOST_S * 10
                    && !usb_cdc_host_connected(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG_CSI, "USB-CDC host %s; starting CSI",
             usb_cdc_host_connected() ? "attached" : "absent after wait");
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&_wifi_csi_cb, NULL));

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_BINARY
    binary_sink_init();
#endif

    _print_csi_csv_header();

    ESP_LOGI(TAG_CSI, "CSI acquisition started (chip=%s, queue=%d, role=%s)",
             CONFIG_IDF_TARGET, CSI_QUEUE_DEPTH, type);
#endif // CONFIG_SHOULD_COLLECT_CSI
}

#endif //ESP32_CSI_CSI_COMPONENT_H
