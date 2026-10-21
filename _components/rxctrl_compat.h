#ifndef ESP32_CSI_RXCTRL_COMPAT_H
#define ESP32_CSI_RXCTRL_COMPAT_H

/*
 * Per-frame RX metadata accessors that work on every supported SoC.
 *
 * WHY THIS EXISTS
 * ---------------
 * `wifi_pkt_rx_ctrl_t` is not one struct. ESP-IDF v6.0.1 typedefs it
 * differently depending on whether the target has 802.11ax support
 * (components/esp_wifi/include/local/esp_wifi_types_native.h:33-87):
 *
 *     #if CONFIG_SOC_WIFI_HE_SUPPORT
 *     typedef esp_wifi_rxctrl_t wifi_pkt_rx_ctrl_t;    // C5 / C6 / C61
 *     #else
 *     typedef struct { ... } wifi_pkt_rx_ctrl_t;       // ESP32 / S2 / S3 / C3
 *     #endif
 *
 * The two layouts share only a handful of fields. Everything the Hernandez
 * 2020 CSV schema reports about the PHY - sig_mode, mcs, cwb, smoothing,
 * not_sounding, aggregation, stbc, fec_coding, sgi, ampdu_cnt, ant - exists
 * ONLY on the non-HE layout. `secondary_channel` exists on both but is spelled
 * `second` on the HE layout. Referencing the non-HE names on an HE target is a
 * hard compile error, which is why the C5/C6/C61 builds never compiled.
 *
 * SEMANTICS
 * ---------
 * Every accessor returns `int`. A field the current SoC genuinely cannot
 * report returns **-1**, never 0: writing 0 would be indistinguishable from a
 * real measurement of zero, and downstream analysis must be able to tell
 * "this chip does not expose it" from "this chip measured it as 0".
 *
 * `sig_mode` is the one field we can reconstruct rather than drop. The HE
 * layout carries `cur_bb_format` (wifi_rx_bb_format_t, esp_wifi_he_types.h:
 * 161-172), which is strictly more informative, so we map it back onto the
 * legacy sig_mode encoding and extend it:
 *
 *     legacy sig_mode : 0 = non-HT (11b/g/a), 1 = HT (11n), 3 = VHT (11ac)
 *     extension       : 4 = HE (11ax)   <- new, HE-capable chips only
 *
 * That keeps every v1 consumer correct on legacy formats while making the
 * 11ax case explicit instead of silently reporting it as non-HT.
 *
 * CHIP MATRIX (verified against ESP-IDF v6.0.1 soc caps)
 *
 *     SoC     HE_SUPPORT  MAC_VERSION_NUM  rx_ctrl layout
 *     ESP32       no            -          legacy
 *     ESP32-S2    no            -          legacy
 *     ESP32-S3    no            -          legacy
 *     ESP32-C3    no            -          legacy
 *     ESP32-C5    yes           3          esp_wifi_rxctrl_t (MAC v3)
 *     ESP32-C6    yes           2          esp_wifi_rxctrl_t (MAC v2)
 *     ESP32-C61   yes           3          esp_wifi_rxctrl_t (MAC v3)
 *
 * The two esp_wifi_rxctrl_t variants differ in reserved-field layout but agree
 * on every field this header touches, so one HE branch covers C5/C6/C61.
 */

#include "esp_wifi.h"
#include "esp_wifi_types.h"

/* Value written for a field the current SoC does not expose. */
#define CSI_FIELD_NA (-1)

/* Extension to the legacy sig_mode encoding for 802.11ax PPDUs. */
#define CSI_SIG_MODE_HE 4

#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)

static inline int csi_rx_sig_mode(const wifi_pkt_rx_ctrl_t *rx)
{
    switch (rx->cur_bb_format) {
        case RX_BB_FORMAT_11B:
        case RX_BB_FORMAT_11G:      /* == RX_BB_FORMAT_11A */
            return 0;               /* non-HT */
        case RX_BB_FORMAT_HT:
            return 1;               /* HT (11n) */
        case RX_BB_FORMAT_VHT:
        case RX_BB_FORMAT_VHT_MU:
            return 3;               /* VHT (11ac) */
        case RX_BB_FORMAT_HE_SU:
        case RX_BB_FORMAT_HE_MU:
        case RX_BB_FORMAT_HE_ERSU:
        case RX_BB_FORMAT_HE_TB:
            return CSI_SIG_MODE_HE; /* HE (11ax) */
        default:
            return CSI_FIELD_NA;
    }
}

/* Fields with no counterpart in esp_wifi_rxctrl_t. The PHY parameters they
 * describe are encoded in he_siga1/he_siga2 on HE targets, but decoding those
 * is format-dependent and out of scope for a metadata column. */
static inline int csi_rx_mcs(const wifi_pkt_rx_ctrl_t *rx)          { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_bandwidth(const wifi_pkt_rx_ctrl_t *rx)    { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_smoothing(const wifi_pkt_rx_ctrl_t *rx)    { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_not_sounding(const wifi_pkt_rx_ctrl_t *rx) { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_aggregation(const wifi_pkt_rx_ctrl_t *rx)  { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_stbc(const wifi_pkt_rx_ctrl_t *rx)         { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_fec_coding(const wifi_pkt_rx_ctrl_t *rx)   { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_sgi(const wifi_pkt_rx_ctrl_t *rx)          { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_ampdu_cnt(const wifi_pkt_rx_ctrl_t *rx)    { (void)rx; return CSI_FIELD_NA; }
static inline int csi_rx_ant(const wifi_pkt_rx_ctrl_t *rx)          { (void)rx; return CSI_FIELD_NA; }

static inline int csi_rx_secondary_channel(const wifi_pkt_rx_ctrl_t *rx)
{
    return (int)rx->second;         /* spelled `second` on the HE layout */
}

#else  /* !CONFIG_SOC_WIFI_HE_SUPPORT - ESP32 / S2 / S3 / C3 */

static inline int csi_rx_sig_mode(const wifi_pkt_rx_ctrl_t *rx)     { return (int)rx->sig_mode; }
static inline int csi_rx_mcs(const wifi_pkt_rx_ctrl_t *rx)          { return (int)rx->mcs; }
static inline int csi_rx_bandwidth(const wifi_pkt_rx_ctrl_t *rx)    { return (int)rx->cwb; }
static inline int csi_rx_smoothing(const wifi_pkt_rx_ctrl_t *rx)    { return (int)rx->smoothing; }
static inline int csi_rx_not_sounding(const wifi_pkt_rx_ctrl_t *rx) { return (int)rx->not_sounding; }
static inline int csi_rx_aggregation(const wifi_pkt_rx_ctrl_t *rx)  { return (int)rx->aggregation; }
static inline int csi_rx_stbc(const wifi_pkt_rx_ctrl_t *rx)         { return (int)rx->stbc; }
static inline int csi_rx_fec_coding(const wifi_pkt_rx_ctrl_t *rx)   { return (int)rx->fec_coding; }
static inline int csi_rx_sgi(const wifi_pkt_rx_ctrl_t *rx)          { return (int)rx->sgi; }
static inline int csi_rx_ampdu_cnt(const wifi_pkt_rx_ctrl_t *rx)    { return (int)rx->ampdu_cnt; }
static inline int csi_rx_ant(const wifi_pkt_rx_ctrl_t *rx)          { return (int)rx->ant; }

static inline int csi_rx_secondary_channel(const wifi_pkt_rx_ctrl_t *rx)
{
    return (int)rx->secondary_channel;
}

#endif /* CONFIG_SOC_WIFI_HE_SUPPORT */

/* Fields present under the same name on both layouts. Wrapped anyway so that
 * every call site goes through one accessor set and a future layout change has
 * exactly one place to be handled. */
static inline int csi_rx_rssi(const wifi_pkt_rx_ctrl_t *rx)         { return (int)rx->rssi; }
static inline int csi_rx_rate(const wifi_pkt_rx_ctrl_t *rx)         { return (int)rx->rate; }
static inline int csi_rx_noise_floor(const wifi_pkt_rx_ctrl_t *rx)  { return (int)rx->noise_floor; }
static inline int csi_rx_channel(const wifi_pkt_rx_ctrl_t *rx)      { return (int)rx->channel; }
static inline int csi_rx_sig_len(const wifi_pkt_rx_ctrl_t *rx)      { return (int)rx->sig_len; }
static inline int csi_rx_rx_state(const wifi_pkt_rx_ctrl_t *rx)     { return (int)rx->rx_state; }

static inline unsigned long csi_rx_timestamp(const wifi_pkt_rx_ctrl_t *rx)
{
    /* 32-bit microsecond counter; wraps every 2^32 us = 71.58 min. Consumers
     * must unwrap it (see experiments/e5_packet_rate/packet_rate_analyzer.py).
     * Precise only when modem sleep and light sleep are both disabled. */
    return (unsigned long)rx->timestamp;
}

/* ------------------------------------------------------------------------
 * Schema v2.1 additions: receive-chain state that the record has to carry if
 * a later analysis is to invert it. Same NA discipline as above: a field the
 * SoC does not expose returns CSI_FIELD_NA, never 0.
 *
 * Rationale. Amplitude reported by any ESP32 CSI tool is a channel response
 * multiplied by whatever the automatic gain control was doing at that instant,
 * and none of the tools in this lineage record that state, so a capture cannot
 * be reduced to a gain-independent quantity after the fact. The validity flags
 * matter for the same reason: the MAC v3 parts document a per-record condition
 * under which part of the buffer is invalid, and without the flag a consumer
 * cannot tell which records it applies to.
 * ------------------------------------------------------------------------ */

/* Per-record CSI validity. Present on the HE layout only
 * (esp_wifi_he_types.h: rx_channel_estimate_info_vld). */
static inline int csi_rx_estimate_valid(const wifi_pkt_rx_ctrl_t *rx)
{
#if CONFIG_SOC_WIFI_HE_SUPPORT
    return (int)rx->rx_channel_estimate_info_vld;
#else
    (void)rx;
    return CSI_FIELD_NA;
#endif
}

/* Leading-sample validity. Unconditional in wifi_csi_info_t on every target
 * (esp_wifi_types_native.h:123), so this one never returns NA. */
static inline int csi_first_word_invalid(const wifi_csi_info_t *d)
{
    return d->first_word_invalid ? 1 : 0;
}

#endif /* ESP32_CSI_RXCTRL_COMPAT_H */
