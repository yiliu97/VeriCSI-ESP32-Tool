#ifndef ESP32_CSI_BINARY_SINK_H
#define ESP32_CSI_BINARY_SINK_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_rom_crc.h"
#include "sdkconfig.h"
#include "rxctrl_compat.h"

/*
 * Binary frame sink.
 *
 * Emits each captured CSI frame as a packed binary record on stdout, *in
 * addition* to the CSV row produced by csi_component.h. The two formats
 * coexist on the same stream:
 *
 *   ASCII CSV : starts with the literal token "CSI_DATA," and ends with \n.
 *               Captured by   idf.py monitor | grep CSI_DATA > file.csv
 *   Binary    : starts with the magic "CSIF" (0x46495343 LE) and is
 *               self-delimiting via embedded length fields and a CRC32.
 *               Captured by   idf.py monitor | python csi_bin_to_csv.py
 *
 * v1 / v2 CSV parsers ignore the binary stream because none of its bytes
 * spell out the token CSI_DATA in ASCII. v2 binary parsers re-sync on the
 * CSIF magic, so a partial / corrupt binary frame just gets skipped.
 *
 * VeriCSI-ESP32 implementation note: this is a deliberately simplified version of
 * the format described in docs/binary_format.md. The metadata block uses
 * fixed-size string fields instead of length-prefixed strings (faster to
 * parse, ~50 byte larger per frame; v2.1 may switch). The stream-level
 * JSON metadata header documented in binary_format.md is also deferred to
 * v2.1; VeriCSI-ESP32 emits a small fixed `CSI6` header at startup instead.
 */

/* Frame magic chosen so that the first byte is not a valid printable ASCII
 * (so a console viewer reading the mixed stream as text shows them as
 * non-character glyphs rather than misinterpreting them as CSV). */
#define BINARY_FRAME_MAGIC      "CSIF"
#define BINARY_FRAME_MAGIC_LEN  4
#define BINARY_STREAM_MAGIC     "CSI6"
#define BINARY_STREAM_MAGIC_LEN 4
#define BINARY_FORMAT_VERSION   0x01

/* Fixed-size string field lengths. Long enough for current and likely-future
 * values; short enough that the metadata block stays compact. */
#define BINARY_CHIP_ID_LEN          16
#define BINARY_FW_VERSION_LEN       16
#define BINARY_LAYOUT_ID_LEN        24

/*
 * Packed metadata block laid out for direct memcpy from a csi_queue_item_t.
 * The on-wire byte order is little-endian throughout (matches every modern
 * ESP32 chip; matches typical x86/ARM host).
 */
typedef struct __attribute__((packed)) {
    /* v1 metadata (matches Hernandez 2020 CSV columns 0-24) */
    uint8_t  type_code;          /* 0 = CSI_DATA */
    uint8_t  role_code;          /* 0=STA, 1=AP, 2=PASSIVE */
    uint8_t  mac[6];
    int8_t   rssi;
    uint8_t  rate;
    uint8_t  sig_mode;
    uint8_t  mcs;
    uint8_t  bandwidth;
    uint8_t  smoothing;
    uint8_t  not_sounding;
    uint8_t  aggregation;
    uint8_t  stbc;
    uint8_t  fec_coding;
    uint8_t  sgi;
    int8_t   noise_floor;
    uint8_t  ampdu_cnt;
    uint8_t  channel;
    uint8_t  secondary_channel;
    uint32_t local_timestamp;    /* IDF rx_ctrl, microseconds */
    uint8_t  ant;
    uint16_t sig_len;
    uint8_t  rx_state;
    uint8_t  real_time_set;
    uint64_t real_timestamp_ns;  /* steady clock at v2 emit, nanoseconds */
    uint16_t len;                /* matches CSV `len` column (data->len) */
    /* v2 fields (CSV columns 26-33) */
    char     chip_id[BINARY_CHIP_ID_LEN];               /* nul-terminated */
    char     firmware_version[BINARY_FW_VERSION_LEN];   /* nul-terminated */
    uint16_t csi_buf_len;
    char     subcarrier_layout_id[BINARY_LAYOUT_ID_LEN]; /* nul-terminated */
    uint32_t gptimer_seq;
    uint8_t  he_ltf_present;
    uint8_t  he_stbc_mode;
    uint8_t  acquire_force_lltf;
    uint8_t  acquire_beamformed; /* was `reserved` earlier; same offset */
} binary_meta_t;

/* Forward declarations from csi_component / chip_aware_decoder so this header
 * can be included in any order. */
struct _csi_queue_item;
typedef struct _csi_queue_item csi_queue_item_t_fwd;

#ifdef CONFIG_ESP32_CSI_TOOL_SINK_BINARY

static const char *TAG_BIN = "binary_sink";
static uint64_t   s_bin_frames_written = 0;

/* Helper: copy a C string into a fixed-size buffer, NUL-terminate, pad with
 * zeros. Truncates silently if too long (the strings we copy are all under
 * 24 chars; truncation is not expected). */
static inline void _bin_copy_str(char *dst, size_t dst_len, const char *src) {
    if (src == NULL) {
        memset(dst, 0, dst_len);
        return;
    }
    size_t n = strnlen(src, dst_len - 1);
    memcpy(dst, src, n);
    memset(dst + n, 0, dst_len - n);
}

static inline void binary_sink_init(void) {
    /* Stream header: 16 bytes. Emitted once at startup so a host-side parser
     * connected to a pipe or socket can identify the stream as binary
     * before the first frame arrives. */
    uint8_t hdr[16] = {0};
    memcpy(hdr, BINARY_STREAM_MAGIC, BINARY_STREAM_MAGIC_LEN);
    hdr[4] = BINARY_FORMAT_VERSION;
    hdr[5] = 0;  /* flags reserved */
    /* hdr[6..7] reserved */
    /* hdr[8..15] reserved (host should accept any value here in VeriCSI-ESP32) */
    fwrite(hdr, 1, sizeof(hdr), stdout);
    fflush(stdout);
    s_bin_frames_written = 0;
    ESP_LOGI(TAG_BIN, "binary stream header written (CSI6 v%d)", BINARY_FORMAT_VERSION);
}

/*
 * Serialise one csi_queue_item_t into a binary frame and write it to stdout.
 *
 * The csi_queue_item_t type is forward-declared because including
 * csi_component.h here would create a circular dependency. The cast to a
 * matching struct shape is safe because the layout is fully under our
 * control (defined in csi_component.h and re-described locally below).
 */
static inline void binary_sink_write_frame(const void *queue_item) {
    /* Matching shape of csi_queue_item_t from csi_component.h. Keep this
     * struct in sync if the queue_item layout changes. */
    struct {
        int64_t          timestamp_us;
        uint32_t         broadcast_seq;
        wifi_csi_info_t  info;
        uint16_t         buf_len;
        int8_t           buf_data[384];
    } *item = (typeof(item))queue_item;

    binary_meta_t meta;
    memset(&meta, 0, sizeof(meta));

    /* role_code is encoded into project_type (a global char* set by
     * csi_init). 'STA' / 'AP' / 'PASSIVE' map to 0 / 1 / 2. */
    extern char *project_type;
    if (project_type) {
        if (project_type[0] == 'A') meta.role_code = 1;          /* "AP" */
        else if (project_type[0] == 'P') meta.role_code = 2;     /* "PASSIVE" */
        else meta.role_code = 0;                                  /* "STA" or other */
    }

    meta.type_code = 0;     /* CSI_DATA */
    memcpy(meta.mac, item->info.mac, 6);
    /* Via rxctrl_compat.h: wifi_pkt_rx_ctrl_t has a different layout on
     * HE-capable SoCs and most of these fields are absent there. Accessors
     * yield -1 (CSI_FIELD_NA) when the SoC cannot report a field. The binary
     * meta fields are narrow integers, so -1 lands as an all-ones value in the
     * corresponding width - documented in docs/binary_format.md. */
    meta.rssi             = csi_rx_rssi(&item->info.rx_ctrl);
    meta.rate             = csi_rx_rate(&item->info.rx_ctrl);
    meta.sig_mode         = csi_rx_sig_mode(&item->info.rx_ctrl);
    meta.mcs              = csi_rx_mcs(&item->info.rx_ctrl);
    meta.bandwidth        = csi_rx_bandwidth(&item->info.rx_ctrl);
    meta.smoothing        = csi_rx_smoothing(&item->info.rx_ctrl);
    meta.not_sounding     = csi_rx_not_sounding(&item->info.rx_ctrl);
    meta.aggregation      = csi_rx_aggregation(&item->info.rx_ctrl);
    meta.stbc             = csi_rx_stbc(&item->info.rx_ctrl);
    meta.fec_coding       = csi_rx_fec_coding(&item->info.rx_ctrl);
    meta.sgi              = csi_rx_sgi(&item->info.rx_ctrl);
    meta.noise_floor      = csi_rx_noise_floor(&item->info.rx_ctrl);
    meta.ampdu_cnt        = csi_rx_ampdu_cnt(&item->info.rx_ctrl);
    meta.channel          = csi_rx_channel(&item->info.rx_ctrl);
    meta.secondary_channel= csi_rx_secondary_channel(&item->info.rx_ctrl);
    meta.local_timestamp  = csi_rx_timestamp(&item->info.rx_ctrl);
    meta.ant              = csi_rx_ant(&item->info.rx_ctrl);
    meta.sig_len          = csi_rx_sig_len(&item->info.rx_ctrl);
    meta.rx_state         = csi_rx_rx_state(&item->info.rx_ctrl);
    meta.real_time_set    = (uint8_t)real_time_set;
    /* Steady clock as nanoseconds since v2 boot: more bits than the v1 ms
     * counter and avoids the wall-clock-dependence of real_timestamp. */
    meta.real_timestamp_ns = (uint64_t)item->timestamp_us * 1000ULL;
    meta.len              = item->info.len;

    _bin_copy_str(meta.chip_id, BINARY_CHIP_ID_LEN, CONFIG_IDF_TARGET);
    /* Fixed tool name, no suffix (see csi_component.h). */
    _bin_copy_str(meta.firmware_version, BINARY_FW_VERSION_LEN, "VeriCSI-ESP32");
    meta.csi_buf_len = item->buf_len;
    csi_layout_id_t layout = chip_aware_infer_layout(&item->info);
    _bin_copy_str(meta.subcarrier_layout_id, BINARY_LAYOUT_ID_LEN,
                  chip_aware_layout_name(layout));
    meta.gptimer_seq        = item->broadcast_seq;
    meta.he_ltf_present     = (layout == CSI_LAYOUT_HE_LTF_SU ||
                               layout == CSI_LAYOUT_HE_LTF_MU) ? 1 : 0;
#  if defined(CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE)
    meta.he_stbc_mode       = CONFIG_ESP32_CSI_TOOL_HE_STBC_MODE;
#  else
    meta.he_stbc_mode       = 2;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
    meta.acquire_force_lltf = 1;
#  else
    meta.acquire_force_lltf = 0;
#  endif
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED
    meta.acquire_beamformed = 1;
#  else
    meta.acquire_beamformed = 0;
#  endif

    /* Frame layout:
     *
     *  +-----------------+----------+--------+-----------+--------+
     *  |  CSIF magic (4) | meta_len |  meta  |  csi_len  |  csi   | + crc32 (4)
     *  +-----------------+----------+--------+-----------+--------+
     *                       (2)       (HL)      (2)        (CL)
     *
     * CRC32 is over the bytes from 'CSIF' through the last CSI byte (NOT
     * including the CRC itself); ESP-ROM CRC32 is hardware-accelerated.
     */
    const uint16_t meta_len = (uint16_t)sizeof(meta);
    const uint16_t csi_len  = item->buf_len;
    const size_t   total_pre_crc =
        BINARY_FRAME_MAGIC_LEN + 2 + meta_len + 2 + csi_len;

    /* Stream out in segments to avoid a stack-resident assembly buffer the
     * size of the whole frame. fwrite's userland buffering coalesces the
     * pieces. */
    fwrite(BINARY_FRAME_MAGIC, 1, BINARY_FRAME_MAGIC_LEN, stdout);
    fwrite(&meta_len, 1, 2, stdout);
    fwrite(&meta, 1, meta_len, stdout);
    fwrite(&csi_len, 1, 2, stdout);
    fwrite(item->buf_data, 1, csi_len, stdout);

    /* CRC32 across the whole pre-CRC payload. Compute incrementally so we
     * do not need an assembly buffer. */
    uint32_t crc = 0;
    crc = ~esp_rom_crc32_le(~crc, (const uint8_t *)BINARY_FRAME_MAGIC, BINARY_FRAME_MAGIC_LEN);
    crc = ~esp_rom_crc32_le(~crc, (const uint8_t *)&meta_len, 2);
    crc = ~esp_rom_crc32_le(~crc, (const uint8_t *)&meta, meta_len);
    crc = ~esp_rom_crc32_le(~crc, (const uint8_t *)&csi_len, 2);
    crc = ~esp_rom_crc32_le(~crc, (const uint8_t *)item->buf_data, csi_len);
    /* Note: esp_rom_crc32_le applies bitwise NOT internally, so the standard
     * "init=0xFFFFFFFF, finalize ^=0xFFFFFFFF" trick is achieved by
     * complementing the seed and the result. The host-side parser uses the
     * same convention; see python_utils/csi_bin_to_csv.py. */
    fwrite(&crc, 1, 4, stdout);

    /* Force the bytes onto the wire so a downstream parser does not get a
     * partial frame stuck in stdout's userland buffer. */
    fflush(stdout);

    s_bin_frames_written++;
    (void)total_pre_crc;
}

#else /* binary sink not enabled */

static inline void binary_sink_init(void) {}
static inline void binary_sink_write_frame(const void *queue_item) { (void)queue_item; }

#endif /* CONFIG_ESP32_CSI_TOOL_SINK_BINARY */

#endif /* ESP32_CSI_BINARY_SINK_H */
