#ifndef ESP32_CSI_GPTIMER_BROADCAST_H
#define ESP32_CSI_GPTIMER_BROADCAST_H

#include <stdint.h>
#include <stdio.h>     /* sscanf, for the optional source-MAC override */
#include <string.h>
#include <limits.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"

#include "sdkconfig.h"
#include "csi_component.h"   // for csi_set_broadcast_seq()

/*
 * Application-layer timer-driven broadcast
 *
 * Hernandez 2020 ESP32-CSI-Tool drives packet generation indirectly through
 * STA-side HTTP polling. The packet rate observed at the AP is therefore at
 * the mercy of TCP round-trips, retransmissions and the STA's own scheduler -
 * the inter-arrival time of the resulting CSI frames is jittered well above
 * what frequency-domain sensing tasks (breathing rate 0.1-0.5 Hz, heart rate
 * 0.8-3 Hz) can tolerate without spectral leakage.
 *
 * VeriCSI-ESP32 adds an optional AP-side path: a FreeRTOS general-purpose timer fires
 * at exactly target_hz, and a dedicated TX task transmits one known-payload
 * 802.11 broadcast frame per fire. The two design rules are:
 *
 *   1. The gptimer ISR MUST NOT call any Wi-Fi API. esp_wifi_80211_tx takes
 *      internal mutexes; calling it from ISR context can deadlock the Wi-Fi
 *      driver. The ISR therefore only does xTaskNotifyFromISR.
 *
 *   2. The TX task runs at higher priority than the CSI consumer (5) so that
 *      a back-pressured serial sink does not starve packet generation.
 *
 * Frame layout (64 byte total = 24 byte 802.11 header + 40 byte payload):
 *
 *     offset  size  field
 *      0       2    Frame Control (Data, FromDS=1)
 *      2       2    Duration
 *      4       6    Address 1: broadcast (FF FF FF FF FF FF)
 *     10       6    Address 2: self MAC (filled at init)
 *     16       6    Address 3: BSSID = self MAC (filled at init)
 *     22       2    Sequence Control (set by Wi-Fi driver, en_sys_seq=true)
 *     24       4    Magic "CSI2"
 *     28       4    broadcast_seq (uint32 LE; patched per TX)
 *     32       4    timestamp_us low 32 bits (uint32 LE; patched per TX)
 *     36      28    Zero padding (signal energy + frame length stability)
 *
 * Note that 24-byte 802.11 header alone may be discarded by some Wi-Fi PHYs
 * before reaching the CSI capture path; the 40-byte payload above ensures
 * the frame is large enough to be classified as a Data frame and trigger
 * the receiver's CSI rx callback reliably.
 */

#define GPTIMER_BCAST_FRAME_LEN     64
#define GPTIMER_BCAST_TASK_STACK    4096
#define GPTIMER_BCAST_TASK_PRIO     6      /* > csi_consumer_task (5) */
#define GPTIMER_BCAST_HZ_MIN        1
#define GPTIMER_BCAST_HZ_MAX        2000

/*
 * The alarm period is configured in MICROSECONDS, not in whole Hz.
 *
 * Rate-in-integer-Hz cannot express the rates that matter. The 802.11 beacon
 * interval is 100 TU = 102400 us = 9.765625 Hz, and any experiment that sweeps
 * the sounding rate across that value needs sub-Hz resolution: 9.4 / 9.6 /
 * 9.765625 / 9.9 / 10.1 / 10.3 Hz all collapse onto 9 or 10 Hz once rounded.
 * Expressed as a period they are exact at the timer's 1 us tick:
 * 106383 / 104167 / 102400 / 101010 / 99010 / 97087 us.
 */
#define GPTIMER_BCAST_PERIOD_US_MIN 500        /* 2000 Hz */
#define GPTIMER_BCAST_PERIOD_US_MAX 1000000    /* 1 Hz    */

#if CONFIG_FREERTOS_UNICORE
#  define GPTIMER_BCAST_CORE        tskNO_AFFINITY
#else
#  define GPTIMER_BCAST_CORE        0      /* same core as Wi-Fi task on dual-core */
#endif

static const char *TAG_BCAST = "gptimer_broadcast";

typedef struct {
    uint32_t period_us;          /* authoritative: alarm period in microseconds */
    uint32_t broadcast_seq;
    uint64_t timer_start_us;
    uint64_t total_tx_count;
    uint64_t total_tx_failures;
    uint64_t total_isr_misses;
} gptimer_broadcast_state_t;

static gptimer_broadcast_state_t s_bcast_state   = {};   // C++ value-init: all fields zero. Avoids -Werror=missing-field-initializers under v6.0.1 + gcc 15.
static gptimer_handle_t          s_bcast_timer   = NULL;
static TaskHandle_t              s_bcast_tx_task = NULL;

/*
 * The frame template lives in normal RAM (not IRAM); only the ISR callback
 * needs IRAM. The TX task runs in regular task context and can access it
 * directly without cache concerns.
 */
/* HT broadcast formats transmit through ESP-NOW with a per-peer rate
 * config. esp_wifi_80211_tx cannot inject at HT rates on this platform:
 * verified on hardware 2026-08-08 (ESP32-S3), a frame submitted at
 * HT20 MCS0 is accepted (tx counter increments, no error) but never
 * appears on the air, for plain Data and QoS-Data headers alike. ESP-NOW
 * action frames honor esp_now_set_peer_rate_config phymode/rate and are
 * regular OFDM frames, so promiscuous receivers produce CSI for them. */
/* HE20 and VHT20 ride the same ESP-NOW path. wifi_phy_mode_t
 * (esp_wifi_types_generic.h:667-676) carries WIFI_PHY_MODE_HE20 and
 * WIFI_PHY_MODE_VHT20 alongside the HT modes, and the rate table in the same
 * header (lines 1006-1032) gives HE20 and VHT20 columns for the shared
 * WIFI_PHY_RATE_MCSn_LGI encodings, so only the phymode field changes. */
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HT20_MCS0 \
 || CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HT40_MCS0 \
 || CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HE20_MCS0 \
 || CONFIG_ESP32_CSI_TOOL_BCAST_PHY_VHT20_MCS0
#define BCAST_HT 1
#include "esp_now.h"
/* Not const: BCAST_DST_MAC rewrites it before esp_now_add_peer(). */
static uint8_t s_bcast_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_ALT_PATTERN
/* The rate config built at init, kept so the per-frame format switch can
 * reuse its MCS field and change only phymode. */
static esp_now_rate_config_t s_bcast_rate_cfg;
/* Which format the peer is currently set to, 1 for HT40. Starts at a value
 * neither branch produces so the first frame always issues the call. */
static uint8_t s_bcast_alt_cur = 0xFF;
static uint64_t s_bcast_alt_failures;
#endif
#else
#define BCAST_HT 0
#endif
#define BCAST_OFF_SEQCTL 22   /* 802.11 Sequence Control, little-endian */
#define BCAST_OFF_MAGIC 24
#define BCAST_OFF_SEQ   28
#define BCAST_OFF_TS    32

static uint8_t s_bcast_frame[GPTIMER_BCAST_FRAME_LEN] = {
    /*  0..1  */ 0x08, 0x02,                           /* FC: Data, FromDS=1 */
    /*  2..3  */ 0x00, 0x00,                           /* Duration */
    /*  4..9  */ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* Addr1: broadcast */
    /* 10..15 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   /* Addr2: self MAC */
    /* 16..21 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   /* Addr3: BSSID = self */
    /* 22..23 */ 0x00, 0x00,                           /* Sequence Control */
    /* 24..27 */ 'C',  'S',  'I',  '2',                /* Magic */
    /* 28..31 */ 0x00, 0x00, 0x00, 0x00,               /* broadcast_seq */
    /* 32..35 */ 0x00, 0x00, 0x00, 0x00,               /* timestamp_us low */
    /* 36..63 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00,
};

/*
 * Forward declaration of the IDF raw 802.11 transmit helper.
 * Hernandez 2020 declared this in active_sta/main/main.cc; VeriCSI-ESP32 keeps the
 * declaration alongside the only call site in the AP-side broadcast path.
 */
extern "C" esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer,
                                       int len, bool en_sys_seq);

/*
 * Timer ISR. Runs in interrupt context: only allowed to xTaskNotifyFromISR.
 * Returns true to request a context switch if a higher-priority task was
 * woken (per gptimer_alarm_cb_t contract).
 */
static bool IRAM_ATTR _gptimer_alarm_cb(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_ctx) {
    if (s_bcast_tx_task == NULL) {
        return false;
    }
    BaseType_t hp = pdFALSE;
    /*
     * eIncrement: each missed alarm increments the notify counter; the task
     * drains the counter when it finally runs. This preserves "we owe N
     * frames" without growing memory like a queue would.
     */
    BaseType_t r = xTaskNotifyFromISR(s_bcast_tx_task, 0, eIncrement, &hp);
    if (r != pdPASS) {
        /* Should not happen with eIncrement; counted defensively. */
        s_bcast_state.total_isr_misses++;
    }
    return hp == pdTRUE;
}

/*
 * TX task. Blocks on notify, then transmits one frame per pending notify.
 * The notify counter is cleared on receipt; sending one frame per logical
 * alarm keeps the served sequence number monotonic and gap-free, while the
 * MAC layer's own back-pressure (CSMA/CCA) shapes the actual airtime.
 */
static void _gptimer_broadcast_tx_task(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        for (uint32_t i = 0; i < notif; i++) {
            uint32_t seq   = ++s_bcast_state.broadcast_seq;
            uint64_t ts_us = (uint64_t)esp_timer_get_time();
            uint32_t ts_lo = (uint32_t)(ts_us & 0xFFFFFFFFu);

            memcpy(&s_bcast_frame[BCAST_OFF_SEQ], &seq,   sizeof(seq));
            memcpy(&s_bcast_frame[BCAST_OFF_TS],  &ts_lo, sizeof(ts_lo));

#if CONFIG_ESP32_CSI_TOOL_BCAST_APP_SEQ
            /*
             * Write the 802.11 Sequence Control field from the same counter
             * and transmit with en_sys_seq clear, so the number the receiver
             * reports in rx_seq is a number the sender recorded. Without this
             * the MAC assigns it and the transmit side has no record of what
             * went out, which is why the loss accounting of Section 6.6 can
             * only close a receiver against its own counters.
             *
             * Sequence Control is 16 bits little-endian: fragment number in
             * bits 0 to 3, sequence number in bits 4 to 15. The counter is
             * therefore masked to 12 bits and shifted left by four, matching
             * the field the receiver decodes into rx_seq.
             */
            {
                uint16_t sc = (uint16_t)((seq & 0x0FFFu) << 4);
                s_bcast_frame[BCAST_OFF_SEQCTL]     = (uint8_t)(sc & 0xFFu);
                s_bcast_frame[BCAST_OFF_SEQCTL + 1] = (uint8_t)(sc >> 8);
            }
#endif

            /*
             * Forward the sequence to csi_component so that frames captured
             * by our own STA pair (or any STA on the same SSID) can carry
             * gptimer_seq in their CSV row.
             */
            csi_set_broadcast_seq(seq);

#if CONFIG_ESP32_CSI_TOOL_BCAST_TX_POWER_LADDER
            /*
             * Step the transmit power so the receiver's automatic gain control
             * has something to respond to. A bench whose sender, receiver and
             * room are all still holds the receive level constant, the gain
             * settles on one or two values, and a locked-versus-free
             * comparison then contrasts two conditions that behave
             * identically. Measured 2026-08-18 at a fixed 10 dBm over 1 m:
             * agc_gain took two values across 2991 records.
             *
             * The call is issued once per step, not once per frame. It is not
             * checked for failure here because the following capture reads the
             * level the receiver actually saw, which is the quantity the
             * analysis uses.
             */
            {
                static const int8_t ladder[4] = {80, 56, 32, 16};   /* qdBm */
                static uint8_t cur_step = 0xFF;
                uint8_t step = (uint8_t)(((seq - 1u)
                        / CONFIG_ESP32_CSI_TOOL_BCAST_TX_LADDER_FRAMES) & 3u);
                if (step != cur_step) {
                    esp_wifi_set_max_tx_power(ladder[step]);
                    cur_step = step;
                }
            }
#endif

#if BCAST_HT
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_ALT_PATTERN
            /*
             * Per-frame format switch on the repeating pattern
             *   HT20, HT40, HT20, HT20
             * so that consecutive frames from this one sender give both
             * cross-format pairs (positions 0-1 and 1-2) and same-format
             * pairs (positions 2-3 and 3-0), at the same time separation.
             *
             * The call is issued only when the format actually changes, which
             * is twice per period rather than once per frame, and its failure
             * is counted rather than logged: at 100 Hz a log line per failure
             * starves the DIAG heartbeat, which is how the rejected-rate case
             * below was made hard to see in the first place.
             */
            {
                static const uint8_t alt_ht40[4] = {0, 1, 0, 0};
                uint8_t want40 = alt_ht40[(seq - 1u) & 3u];
                if (want40 != s_bcast_alt_cur) {
                    esp_now_rate_config_t rc = s_bcast_rate_cfg;
                    rc.phymode = want40 ? WIFI_PHY_MODE_HT40
                                        : WIFI_PHY_MODE_HT20;
                    esp_err_t ar = esp_now_set_peer_rate_config(s_bcast_addr,
                                                                &rc);
                    if (ar == ESP_OK) {
                        s_bcast_alt_cur = want40;
                    } else {
                        s_bcast_alt_failures++;
                        if ((s_bcast_alt_failures & 0xFFu) == 0) {
                            ESP_LOGW(TAG_BCAST,
                                     "per-frame phymode switch to %s failed: "
                                     "%s (cumulative=%llu); frames continue at "
                                     "the previous format, so the pattern the "
                                     "receiver sees is not the one configured",
                                     want40 ? "HT40" : "HT20",
                                     esp_err_to_name(ar),
                                     (unsigned long long)s_bcast_alt_failures);
                        }
                    }
                }
            }
#endif
            /* payload only: magic + seq + timestamp; ESP-NOW adds its own
             * 802.11 action-frame header */
            esp_err_t r = esp_now_send(s_bcast_addr,
                                       &s_bcast_frame[BCAST_OFF_MAGIC],
                                       GPTIMER_BCAST_FRAME_LEN - BCAST_OFF_MAGIC);
#else
#if CONFIG_ESP32_CSI_TOOL_BCAST_APP_SEQ
            /* en_sys_seq clear: the field written above is what goes on the air.
             *
             * Measured on hardware 2026-08-16, ESP-IDF v6.0.1, ESP32 as AP: the
             * driver refuses this. Every call logs
             *   E wifi:en_sys_seq should be true to avoid side-effect to WiFi connection
             * at the full transmit rate, which floods the console and starves the
             * DIAG heartbeat, and the frame goes out with the MAC-assigned
             * sequence number regardless. The option is kept for the day the
             * driver allows it and is off by default. See
             * experiments/e21_c_group/C1_ROUTE_A_RESULT.md. */
            esp_err_t r = esp_wifi_80211_tx(WIFI_IF_AP, s_bcast_frame,
                                            GPTIMER_BCAST_FRAME_LEN, false);
#else
            esp_err_t r = esp_wifi_80211_tx(WIFI_IF_AP, s_bcast_frame,
                                            GPTIMER_BCAST_FRAME_LEN, true);
#endif
#endif
            if (r == ESP_OK) {
                s_bcast_state.total_tx_count++;
            } else {
                s_bcast_state.total_tx_failures++;
                /* rate-limited log: one line per 256 failures */
                if ((s_bcast_state.total_tx_failures & 0xFFu) == 0) {
                    ESP_LOGW(TAG_BCAST, "esp_wifi_80211_tx failed: %s (cumulative=%llu)",
                             esp_err_to_name(r),
                             (unsigned long long)s_bcast_state.total_tx_failures);
                }
            }
        }
    }
}

/*
 * Public API.
 */

/*
 * Arm the broadcast timer with an explicit alarm period in microseconds.
 * This is the primary entry point; gptimer_broadcast_init_hz() below is a
 * convenience wrapper for whole-Hz callers.
 */
void gptimer_broadcast_init(uint32_t period_us) {
    if (period_us < GPTIMER_BCAST_PERIOD_US_MIN ||
        period_us > GPTIMER_BCAST_PERIOD_US_MAX) {
        ESP_LOGE(TAG_BCAST, "invalid period_us=%lu (range %d..%d)",
                 (unsigned long)period_us,
                 GPTIMER_BCAST_PERIOD_US_MIN, GPTIMER_BCAST_PERIOD_US_MAX);
        return;
    }
    if (s_bcast_timer != NULL) {
        ESP_LOGW(TAG_BCAST, "gptimer_broadcast_init called twice; ignored");
        return;
    }

    s_bcast_state.period_us       = period_us;
    s_bcast_state.broadcast_seq   = 0;
    s_bcast_state.total_tx_count  = 0;
    s_bcast_state.total_tx_failures = 0;
    s_bcast_state.total_isr_misses  = 0;

    /*
     * Force an OFDM PHY rate for the injected frames. THIS IS LOAD-BEARING.
     *
     * esp_wifi_80211_tx defaults to the lowest basic rate, 1 Mbps 802.11b
     * DSSS. DSSS/CCK frames carry NO OFDM training field - no L-LTF, no
     * HT-LTF, no HE-LTF - so no receiver can ever produce CSI from them,
     * regardless of its acquisition config. Verified empirically 2026-07-30:
     * with the default rate, gptimer_tx incremented perfectly (frames on air,
     * SSID visible to a phone) while ESP32 and C6 receivers in both passive
     * and associated-STA roles produced zero CSI for this AP; every frame any
     * receiver DID produce CSI for was OFDM (neighbour beacons at 12M OFDM,
     * our own unicast at 6M OFDM). The esp-csi reference does the same thing
     * for the same reason (esp_wifi_config_80211_tx_rate(..., WIFI_PHY_RATE_6M)).
     *
     * 6 Mbps legacy OFDM: decodable by every 802.11a/g+ receiver, produces
     * L-LTF-based CSI on every supported SoC, and is the slowest OFDM rate
     * (longest range, most robust). esp_wifi_config_80211_tx (the struct API)
     * is used because the older esp_wifi_config_80211_tx_rate is documented
     * as unusable when the interface protocol mask includes 11AX, which is
     * the default on C5/C6/C61.
     *
     * The phymode must match the CURRENT BAND. 11G is a 2.4 GHz-only phymode:
     * requesting it while the radio is on a 5 GHz channel (ESP32-C5 5G_ONLY)
     * is rejected by the driver with "invalid phymode, need to change phy
     * mode" + ESP_FAIL (observed on hardware, 2026-08-01). The 5 GHz legacy
     * OFDM equivalent is 11A at the same 6 Mbps. Decide by the channel the
     * radio actually sits on (set by wifi_band_apply() before this runs):
     * channels above 14 are 5 GHz.
     */
    uint8_t cur_ch = 0;
    wifi_second_chan_t cur_sec = WIFI_SECOND_CHAN_NONE;
    bool is_5g = (esp_wifi_get_channel(&cur_ch, &cur_sec) == ESP_OK
                  && cur_ch > 14);

    /*
     * Destination override, applied before anything reads the address.
     * The ESP-NOW peer is registered a few lines below and the frame template
     * is patched further down, so this has to come first or the peer keeps
     * the broadcast address while the template carries the new one.
     *
     * The timed frames go to ff:ff:ff:ff:ff:ff by default, which is what makes
     * them unacknowledged and therefore not retried, and every frozen capture
     * was taken that way.
     *
     * An external witness may filter on the destination rather than the
     * source. The Intel 5300 does: in monitor mode it produces CSI only for
     * frames addressed to a fixed 00:16:ea:12:34:56, and an ESP32 access
     * point's broadcast yields zero records in either of its modes, measured
     * 2026-08-10. Directing the timed stream at that address is what lets the
     * 5300 witness the same frames the device under test receives.
     *
     * A unicast destination is acknowledged at the MAC, so a witness that
     * never acknowledges may draw retries. Retries carry the same sequence
     * number, so gptimer_tx still counts originated frames rather than
     * transmissions, but the path is no longer the unacknowledged one the
     * ledger's transmit-count anchor was measured on. Whether it happens is
     * visible in gptimer_fail and in the achieved rate; read both before
     * trusting a run with this option set.
     */
#ifdef CONFIG_ESP32_CSI_TOOL_BCAST_DST_MAC
    if (sizeof(CONFIG_ESP32_CSI_TOOL_BCAST_DST_MAC) > 1) {
        unsigned d[6];
        if (sscanf(CONFIG_ESP32_CSI_TOOL_BCAST_DST_MAC,
                   "%x:%x:%x:%x:%x:%x",
                   &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]) == 6) {
            for (int i = 0; i < 6; i++) {
                s_bcast_frame[4 + i] = (uint8_t)d[i];   /* Addr1 */
#if BCAST_HT
                s_bcast_addr[i] = (uint8_t)d[i];        /* ESP-NOW peer */
#endif
            }
            ESP_LOGW(TAG_BCAST,
                     "broadcast destination overridden to %s; a unicast "
                     "destination is acknowledged and may be retried, so "
                     "check gptimer_fail and the achieved rate",
                     CONFIG_ESP32_CSI_TOOL_BCAST_DST_MAC);
        } else {
            ESP_LOGE(TAG_BCAST, "BCAST_DST_MAC %s is not aa:bb:cc:dd:ee:ff",
                     CONFIG_ESP32_CSI_TOOL_BCAST_DST_MAC);
        }
    }
#endif

#if BCAST_HT
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HT40_MCS0 \
 || CONFIG_ESP32_CSI_TOOL_BCAST_PHY_ALT_PATTERN
    /* Pin the 40 MHz secondary direction. Chip generations differ in the
     * softAP default (C6 picks secondary below on channel 6, S3 picks
     * above; observed 2026-08-08), and a receiver can only decode HT40
     * frames whose primary/secondary pair matches its sniffer setting.
     * BELOW matches the passive receivers' HT40 sniff configuration. */
    esp_err_t chr = esp_wifi_set_channel(cur_ch, WIFI_SECOND_CHAN_BELOW);
    ESP_LOGI(TAG_BCAST, "HT40 secondary pinned BELOW (primary ch %u): %s",
             (unsigned)cur_ch, esp_err_to_name(chr));
#endif
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, s_bcast_addr, 6);
    peer.ifidx   = WIFI_IF_AP;
    peer.channel = 0;                       /* current channel */
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    esp_now_rate_config_t rate_cfg = {};
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HT40_MCS0
    rate_cfg.phymode = WIFI_PHY_MODE_HT40;
    const char *fmt_name = "HT40 (ESP-NOW)";
#elif CONFIG_ESP32_CSI_TOOL_BCAST_PHY_HE20_MCS0
    rate_cfg.phymode = WIFI_PHY_MODE_HE20;
    const char *fmt_name = "HE20 (ESP-NOW)";
#elif CONFIG_ESP32_CSI_TOOL_BCAST_PHY_VHT20_MCS0
    rate_cfg.phymode = WIFI_PHY_MODE_VHT20;
    const char *fmt_name = "VHT20 (ESP-NOW)";
#else
    rate_cfg.phymode = WIFI_PHY_MODE_HT20;
    const char *fmt_name = "HT20 (ESP-NOW)";
#endif
    /* MCS index. The enum runs MCS0_LGI at 0x10 through MCS9_LGI at 0x19
     * before the SGI block starts, so the index adds directly
     * (esp_wifi_types_generic.h:1052-1062). Default 0 keeps every earlier
     * capture's rate unchanged.
     *
     * A rate axis needs this. Comparing a broadcast leg against a unicast leg
     * that the driver rate-adapted to MCS6 leaves addressing and modulation
     * order changing together, which is exactly the confound the E26 control
     * ran into. */
#ifdef CONFIG_ESP32_CSI_TOOL_BCAST_MCS
    rate_cfg.rate = (wifi_phy_rate_t)(WIFI_PHY_RATE_MCS0_LGI
                                      + CONFIG_ESP32_CSI_TOOL_BCAST_MCS);
#else
    rate_cfg.rate = WIFI_PHY_RATE_MCS0_LGI;
#endif
#ifdef CONFIG_ESP32_CSI_TOOL_BCAST_MCS
    ESP_LOGI(TAG_BCAST, "broadcast MCS index %d",
             CONFIG_ESP32_CSI_TOOL_BCAST_MCS);
#endif
#if CONFIG_ESP32_CSI_TOOL_BCAST_PHY_ALT_PATTERN
    /* Keep the MCS the TX task will reuse. Only phymode varies per frame. */
    s_bcast_rate_cfg = rate_cfg;
    ESP_LOGI(TAG_BCAST, "alternating format pattern HT20,HT40,HT20,HT20 "
             "(base format above is overridden per frame)");
#endif
    esp_err_t rr = esp_now_set_peer_rate_config(s_bcast_addr, &rate_cfg);
    if (rr != ESP_OK) {
        /* The driver does not stop sending on a rejected rate config: it
         * falls back to the ESP-NOW default, which on 2.4 GHz is 1 Mbps DSSS.
         * A DSSS frame carries no L-LTF, HT-LTF or HE-LTF, so every receiver
         * reports zero CSI and the run looks like an acquire-slot failure
         * instead of a transmit-side one. Any bench that sweeps PHY formats
         * has to read this line before trusting a zero. */
        ESP_LOGE(TAG_BCAST, "esp_now_set_peer_rate_config(%s) failed: %s - "
                 "broadcast will go out at the ESP-NOW default rate, which "
                 "carries no training field and yields zero CSI",
                 fmt_name, esp_err_to_name(rr));
    } else {
        ESP_LOGI(TAG_BCAST, "broadcast PHY rate: %s (ch %u)",
                 fmt_name, (unsigned)cur_ch);
    }
#else
    wifi_tx_rate_config_t rate_cfg = {};
    rate_cfg.phymode = is_5g ? WIFI_PHY_MODE_11A : WIFI_PHY_MODE_11G;
    rate_cfg.rate    = WIFI_PHY_RATE_6M;
    esp_err_t rr = esp_wifi_config_80211_tx(WIFI_IF_AP, &rate_cfg);
    if (rr != ESP_OK) {
        ESP_LOGE(TAG_BCAST,
                 "esp_wifi_config_80211_tx(6M OFDM, %s) failed: %s - injected "
                 "frames will go out at the driver default rate; if that is "
                 "1 Mbps DSSS (2.4 GHz) NO receiver will produce CSI for them",
                 is_5g ? "11a" : "11g", esp_err_to_name(rr));
    } else {
        ESP_LOGI(TAG_BCAST, "broadcast PHY rate: 6 Mbps legacy OFDM (%s, ch %u)",
                 is_5g ? "11a" : "11g", (unsigned)cur_ch);
    }
#endif

    /*
     * Optional source-address override. A third-party receiver may filter CSI
     * on the transmitter address: esp-csi's csi_recv drops every frame whose
     * info->mac differs from a fixed 1a:00:00:00:00:00, so a head-to-head on
     * one shared transmitter needs this board to send from that address. The
     * override is off by default and the default path is unchanged, so no
     * frozen capture is affected. Setting it here, before the frame template
     * is patched below, keeps addr2 and addr3 consistent with the interface.
     */
#ifdef CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC
    if (sizeof(CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC) > 1) {
        unsigned b[6];
        if (sscanf(CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC,
                   "%x:%x:%x:%x:%x:%x",
                   &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
            uint8_t want[6];
            for (int i = 0; i < 6; i++) want[i] = (uint8_t)b[i];
            if (want[0] & 0x01) {
                ESP_LOGE(TAG_BCAST,
                         "BCAST_SRC_MAC %s is a group address; ignored",
                         CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC);
            } else {
                esp_err_t sret = esp_wifi_set_mac(WIFI_IF_AP, want);
                ESP_LOGI(TAG_BCAST, "broadcast source MAC set to %s (%s)",
                         CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC,
                         esp_err_to_name(sret));
            }
        } else {
            ESP_LOGE(TAG_BCAST, "BCAST_SRC_MAC %s is not aa:bb:cc:dd:ee:ff",
                     CONFIG_ESP32_CSI_TOOL_BCAST_SRC_MAC);
        }
    }
#endif

    /* Patch the broadcast frame's source address with our own SoftAP MAC. */
    uint8_t mac[6] = {0};
    esp_err_t mret = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (mret != ESP_OK) {
        ESP_LOGW(TAG_BCAST, "esp_wifi_get_mac(AP) failed (%s); addr2/addr3 left zero",
                 esp_err_to_name(mret));
    } else {
        memcpy(&s_bcast_frame[10], mac, 6);
        memcpy(&s_bcast_frame[16], mac, 6);
    }

    /* Spawn the TX task BEFORE arming the timer so the first ISR has a target. */
    BaseType_t t = xTaskCreatePinnedToCore(
        _gptimer_broadcast_tx_task, "csi_bcast_tx", GPTIMER_BCAST_TASK_STACK,
        NULL, GPTIMER_BCAST_TASK_PRIO, &s_bcast_tx_task, GPTIMER_BCAST_CORE);
    if (t != pdPASS) {
        ESP_LOGE(TAG_BCAST, "failed to spawn TX task");
        return;
    }

    /* 1 MHz timer => 1 us per tick; alarm period used verbatim. */
    gptimer_config_t cfg = {};
    cfg.clk_src         = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction       = GPTIMER_COUNT_UP;
    cfg.resolution_hz   = 1000000;
    cfg.intr_priority   = 0;   /* let IDF pick a default level */
    esp_err_t r = gptimer_new_timer(&cfg, &s_bcast_timer);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_BCAST, "gptimer_new_timer failed: %s", esp_err_to_name(r));
        return;
    }

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count                  = (uint64_t)period_us;
    alarm.reload_count                 = 0;
    alarm.flags.auto_reload_on_alarm   = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_bcast_timer, &alarm));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = _gptimer_alarm_cb;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_bcast_timer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(s_bcast_timer));
    s_bcast_state.timer_start_us = (uint64_t)esp_timer_get_time();
    ESP_ERROR_CHECK(gptimer_start(s_bcast_timer));

    ESP_LOGI(TAG_BCAST,
             "broadcast started: period %lu us (%.6f Hz), frame %d byte",
             (unsigned long)period_us,
             1000000.0 / (double)period_us,
             GPTIMER_BCAST_FRAME_LEN);
}

/* Convenience wrapper for whole-Hz callers. Note the truncation: any rate that
 * does not divide 1000000 exactly is rounded down to the nearest microsecond,
 * so use gptimer_broadcast_init() directly when the exact rate matters. */
static inline void gptimer_broadcast_init_hz(uint32_t target_hz) {
    if (target_hz < GPTIMER_BCAST_HZ_MIN || target_hz > GPTIMER_BCAST_HZ_MAX) {
        ESP_LOGE(TAG_BCAST, "invalid target_hz=%lu (range %d..%d)",
                 (unsigned long)target_hz, GPTIMER_BCAST_HZ_MIN, GPTIMER_BCAST_HZ_MAX);
        return;
    }
    gptimer_broadcast_init((uint32_t)(1000000UL / target_hz));
}

void gptimer_broadcast_stop(void) {
    if (s_bcast_timer != NULL) {
        gptimer_stop(s_bcast_timer);
        gptimer_disable(s_bcast_timer);
        gptimer_del_timer(s_bcast_timer);
        s_bcast_timer = NULL;
    }
    if (s_bcast_tx_task != NULL) {
        vTaskDelete(s_bcast_tx_task);
        s_bcast_tx_task = NULL;
    }
}

uint64_t gptimer_broadcast_tx_count(void)     { return s_bcast_state.total_tx_count; }
uint64_t gptimer_broadcast_tx_failures(void)  { return s_bcast_state.total_tx_failures; }
uint64_t gptimer_broadcast_isr_misses(void)   { return s_bcast_state.total_isr_misses; }
uint32_t gptimer_broadcast_seq(void)          { return s_bcast_state.broadcast_seq; }

#endif /* ESP32_CSI_GPTIMER_BROADCAST_H */
