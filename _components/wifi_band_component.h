#ifndef ESP32_CSI_WIFI_BAND_COMPONENT_H
#define ESP32_CSI_WIFI_BAND_COMPONENT_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

/*
 * Wi-Fi band / protocol / bandwidth configuration helper.
 *
 * In ESP-IDF v5.4+ / v6.0+, the chips that can or could optionally support
 * 5 GHz (ESP32-C5) and the chips that always sit on 2.4 GHz but expose the
 * v5.4 dual-band API surface (ESP32-C6, ESP32-C61) all use the trio:
 *
 *     esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY / 5G_ONLY / AUTO)
 *     esp_wifi_set_protocols(iface, &wifi_protocols_t{.ghz_2g, .ghz_5g})
 *     esp_wifi_set_bandwidths(iface, &wifi_bandwidths_t{.ghz_2g, .ghz_5g})
 *
 * On older targets (ESP32 / S2 / S3 / C3) only `esp_wifi_set_bandwidth` is
 * available and there is no 5 GHz radio anyway, so we leave them on the
 * defaults their drivers picked. Reference: esp-csi `csi_recv` and
 * `esp-crab/master_recv` examples.
 *
 * The function is forgiving: if a particular configuration step is not
 * applicable for the active mode (e.g. promiscuous mode where the STA
 * interface is not yet bound), it logs a warning and continues. We never
 * ESP_ERROR_CHECK these calls because the VeriCSI-ESP32 default deployment does not
 * actually need 5 GHz; users who opt in via Kconfig accept the experimental
 * status documented in `docs/known_issues.md` (#18493).
 */

// __attribute__((unused)): suppress -Wunused-variable in TUs that include
// this header but only call wifi_band_apply() in a configuration where the
// inlined ESP_LOGI/W path is dead-code-eliminated (e.g. active_ap on
// non-HE chips never reaches the wifi_band_check_ap_acquire_match path).
static const char *TAG_BAND __attribute__((unused)) = "wifi_band";

/*
 * Resolve the Kconfig choice block ESP32_CSI_TOOL_C5_BAND_MODE into a
 * single integer suitable for `esp_wifi_set_band_mode`. Default = 2G_ONLY
 * (matches the upstream `csi_recv` default for C5).
 */
#if defined(CONFIG_IDF_TARGET_ESP32C5)
#  if defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY)
#    define ESP32_CSI_TOOL_C5_BAND_MODE_VALUE  WIFI_BAND_MODE_5G_ONLY
#  elif defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_AUTO)
#    define ESP32_CSI_TOOL_C5_BAND_MODE_VALUE  WIFI_BAND_MODE_AUTO
#  else
#    define ESP32_CSI_TOOL_C5_BAND_MODE_VALUE  WIFI_BAND_MODE_2G_ONLY
#  endif
#endif

/*
 * ESP-IDF v6 renamed the bandwidth enumerators. wifi_bandwidth_t is now
 * WIFI_BW20 / WIFI_BW40 / WIFI_BW80 / WIFI_BW160 / WIFI_BW80_BW80
 * (esp_wifi_types_generic.h:285-289); the v4.x-era WIFI_BW_HT20 / WIFI_BW_HT40
 * spellings no longer exist and were a hard compile error on every target that
 * reached this file.
 */
#if defined(CONFIG_IDF_TARGET_ESP32C5) && defined(CONFIG_ESP32_CSI_TOOL_C5_5G_BANDWIDTH_HT40)
#  define ESP32_CSI_TOOL_C5_5G_BW_VALUE   WIFI_BW40
#  define ESP32_CSI_TOOL_C5_5G_BW_NAME    "HT40"
#else
#  define ESP32_CSI_TOOL_C5_5G_BW_VALUE   WIFI_BW20
#  define ESP32_CSI_TOOL_C5_5G_BW_NAME    "HT20"
#endif

#ifndef CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL
#  define CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL  40
#endif

/*
 * Wi-Fi 6 (802.11ax) protocol bitmasks.
 *
 * IDF v6.0 default protocol set when CONFIG_SOC_WIFI_HE_SUPPORT is on and
 * band mode is 2.4 GHz: WIFI_PROTOCOL_11B | 11G | 11N | 11AX. By calling
 * esp_wifi_set_protocols() at all we override that default; if we omit
 * WIFI_PROTOCOL_11AX from the bitmask we silently disable Wi-Fi 6 at the
 * MAC layer. The HE-LTF acquisition flags in wifi_csi_acquire_config_t
 * (acquire_csi_su / mu / he_stbc_mode) only deliver useful CSI when the
 * MAC has actually negotiated 11ax with a peer, so this matters.
 *
 * Reference: docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/network/esp_wifi.html
 *            #esp_wifi_set_protocols
 *            (and the SOC_WIFI_HE_SUPPORT default-protocol description).
 *
 * Users can disable 11AX explicitly via CONFIG_ESP32_CSI_TOOL_ENABLE_11AX=n
 * in menuconfig (useful for ablation: "is HE-LTF necessary, or does 11n
 * give us the same downstream sensing accuracy?"). On chips without HE
 * support (ESP32 / S2 / S3 / C3) the macro reduces to the legacy bitmask.
 */
/* HT40 at 2.4 GHz follows the same driver rule as at 5 GHz: a 40 MHz
 * bandwidth request is rejected (ESP_ERR_INVALID_ARG) while 11ax sits in
 * the protocol mask (observed on the C6, 2026-08-08, matching the 5 GHz
 * behavior below). HT40 is an 11n feature: drop 11AX when it is requested. */
#if defined(CONFIG_SOC_WIFI_HE_SUPPORT) && defined(CONFIG_ESP32_CSI_TOOL_ENABLE_11AX) \
    && !defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40)
#  define ESP32_CSI_TOOL_2G_PROTOCOLS \
      (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX)
#  define ESP32_CSI_TOOL_2G_PROTOCOLS_NAME  "11b/g/n/ax"
#else
#  define ESP32_CSI_TOOL_2G_PROTOCOLS \
      (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N)
#  define ESP32_CSI_TOOL_2G_PROTOCOLS_NAME  "11b/g/n"
#endif

/*
 * 5 GHz protocols (only meaningful on ESP32-C5; other chips' .ghz_5g is
 * ignored by the driver). Default IDF v6.0 set when both SOC_WIFI_5G and
 * SOC_WIFI_HE are enabled is 11A | 11N | 11AC | 11AX.
 */
#if defined(CONFIG_IDF_TARGET_ESP32C5)
#  if defined(CONFIG_ESP32_CSI_TOOL_C5_5G_BANDWIDTH_HT40)
     /* HT40 at 5 GHz: the driver rejects a 40 MHz bandwidth while VHT
      * (11ac) or HE (11ax) sit in the protocol mask; both are 20 MHz-only
      * on this chip (same rule observed on the C6 with 11ax at 2.4 GHz).
      * HT40 is an 11n feature: run 11a/n. */
#    define ESP32_CSI_TOOL_5G_PROTOCOLS \
        (WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N)
#    define ESP32_CSI_TOOL_5G_PROTOCOLS_NAME  "11a/n"
#  elif defined(CONFIG_ESP32_CSI_TOOL_ENABLE_11AX)
#    define ESP32_CSI_TOOL_5G_PROTOCOLS \
        (WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX)
#    define ESP32_CSI_TOOL_5G_PROTOCOLS_NAME  "11a/n/ac/ax"
#  else
#    define ESP32_CSI_TOOL_5G_PROTOCOLS \
        (WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AC)
#    define ESP32_CSI_TOOL_5G_PROTOCOLS_NAME  "11a/n/ac"
#  endif
#else
#  define ESP32_CSI_TOOL_5G_PROTOCOLS       0
#  define ESP32_CSI_TOOL_5G_PROTOCOLS_NAME  "(unused)"
#endif

/*
 * Apply band / protocol / bandwidth configuration. MUST be called after
 * `esp_wifi_start()`: esp_wifi_set_band_mode() and esp_wifi_set_channel()
 * both return ESP_ERR_WIFI_NOT_STARTED before that point. All three callers
 * (passive / active_ap / active_sta) satisfy this.
 *
 * In WIFI_MODE_NULL (passive promiscuous capture) the per-interface calls
 * are skipped - there is no interface - but the GLOBAL calls (band mode,
 * channel) still run, because that mode is precisely where nobody else sets
 * the channel.
 *
 * On ESP32-C5 with 5 GHz selected, this function ALSO sets the channel from
 * CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL (the WIFI_CHANNEL Kconfig is a 2.4 GHz
 * number kept for back-compat). Callers should NOT separately call
 * `esp_wifi_set_channel(WIFI_CHANNEL, ...)` after this when running in 5 GHz
 * mode. The secondary channel is chosen by the driver in the 5 GHz band, not
 * by this tool, and is logged as read back.
 *
 * Returns ESP_OK if the chip family does not need band-mode handling
 * (older targets; nothing to do) or every applicable step succeeded;
 * returns the first non-OK error code otherwise.
 */
static inline esp_err_t wifi_band_apply(wifi_interface_t iface) {
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C61)

#  if defined(CONFIG_IDF_TARGET_ESP32C5)
    wifi_band_mode_t mode = ESP32_CSI_TOOL_C5_BAND_MODE_VALUE;
#  else
    /* C6 / C61: only 2.4 GHz radio. Pin band mode so the driver does not
     * accidentally probe for 5 GHz scan results. */
    wifi_band_mode_t mode = WIFI_BAND_MODE_2G_ONLY;
#  endif

    esp_err_t r = esp_wifi_set_band_mode(mode);
    if (r != ESP_OK) {
        ESP_LOGW(TAG_BAND,
                 "esp_wifi_set_band_mode(%d) failed: %s; continuing on driver default",
                 (int)mode, esp_err_to_name(r));
    } else {
        ESP_LOGI(TAG_BAND, "band mode = %s",
                 mode == WIFI_BAND_MODE_5G_ONLY ? "5G_ONLY" :
                 mode == WIFI_BAND_MODE_AUTO    ? "AUTO"     : "2G_ONLY");
    }

    /*
     * Per-band protocols. fix: include WIFI_PROTOCOL_11AX in
     * the bitmask on chips with HE support so the MAC actually negotiates
     * Wi-Fi 6 with peers. Without this the wifi_csi_acquire_config_t
     * acquire_csi_su / mu flags would set up HE-LTF capture but the chip
     * would never receive HE preambles, since it would not negotiate 11ax.
     * See `_components/wifi_band_component.h` macros above.
     */
    wifi_protocols_t protocols = {
        .ghz_2g = ESP32_CSI_TOOL_2G_PROTOCOLS,
        .ghz_5g = ESP32_CSI_TOOL_5G_PROTOCOLS,
    };
#  if defined(CONFIG_IDF_TARGET_ESP32C61)
    /* C61 issue #18118 workaround: CSI broken for 802.11g packets. Mask
     * 11g out of the protocol bitmask. The chip can still associate over
     * 11b / 11n / 11ax for both data plane and control plane. */
    protocols.ghz_2g &= (uint32_t)~WIFI_PROTOCOL_11G;
#  endif
    /*
     * PER-INTERFACE calls only. These take a wifi_interface_t and are only
     * meaningful when that interface exists. `passive` runs in WIFI_MODE_NULL
     * with promiscuous capture, so there is no STA or AP interface and
     * esp_wifi_set_protocols() returns ESP_ERR_INVALID_ARG:
     *     W wifi_band: esp_wifi_set_protocols(iface=0) failed: ESP_ERR_INVALID_ARG
     *
     * NOTE ON STRUCTURE. This used to be an early `return ESP_OK`, which was a
     * regression: it also skipped the C5 5 GHz channel block below.
     * esp_wifi_set_channel() takes NO interface argument - it is a global radio
     * setting that works fine in WIFI_MODE_NULL (passive/main.cc relies on
     * exactly that for its 2.4 GHz channel). Skipping it broke the contract
     * documented at the top of this function, and because passive/main.cc
     * deliberately does NOT set its own channel on C5-in-5 GHz - it trusts this
     * function to have done it - the receiver would have been left on the
     * driver's fallback channel instead of the configured one. Per
     * esp_wifi_set_band_mode() attention 6, a band change with no channel yet
     * set for the new band silently assigns a default: channel 1 for 2.4 GHz,
     * channel 36 for 5 GHz. So the symptom was not "no channel" but the quieter
     * "channel 36 while the transmitter is on 40" - zero frames, no error, same
     * failure class that cost hours on 2026-07-30.
     * Guard the per-interface calls only; let the global ones always run.
     */
    wifi_mode_t cur_mode = WIFI_MODE_NULL;
    const bool have_iface =
        !(esp_wifi_get_mode(&cur_mode) == ESP_OK && cur_mode == WIFI_MODE_NULL);

    if (!have_iface) {
        ESP_LOGI(TAG_BAND,
                 "WIFI_MODE_NULL (promiscuous capture): band mode applied, "
                 "per-interface protocol/bandwidth skipped (no interface)");
    } else {
        r = esp_wifi_set_protocols(iface, &protocols);
        if (r != ESP_OK) {
            ESP_LOGW(TAG_BAND,
                     "esp_wifi_set_protocols(iface=%d) failed: %s",
                     (int)iface, esp_err_to_name(r));
        } else {
            ESP_LOGI(TAG_BAND, "protocols: 2g=%s, 5g=%s",
                     ESP32_CSI_TOOL_2G_PROTOCOLS_NAME,
                     ESP32_CSI_TOOL_5G_PROTOCOLS_NAME);
        }

        /*
         * Per-band bandwidths.
         *
         * Only ESP32-C5 has a 5 GHz radio. Populating .ghz_5g on a 2.4 GHz-only
         * part (C6 / C61) makes the driver reject the WHOLE call:
         *     E wifi: 5G is not supported, can not set wifi bandwidth of 5G band
         *     W wifi_band: esp_wifi_set_bandwidths(iface=0) failed: ESP_ERR_NOT_SUPPORTED
         * so the 2.4 GHz bandwidth silently never got applied either. Set only
         * the bands the chip actually has.
         */
        wifi_bandwidths_t bandwidth = {};
#  if defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40) && CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40
        bandwidth.ghz_2g = WIFI_BW40;
#  else
        bandwidth.ghz_2g = WIFI_BW20;
#  endif
#  if defined(CONFIG_IDF_TARGET_ESP32C5)
        bandwidth.ghz_5g = ESP32_CSI_TOOL_C5_5G_BW_VALUE;
#  endif
        r = esp_wifi_set_bandwidths(iface, &bandwidth);
        if (r != ESP_OK) {
            ESP_LOGW(TAG_BAND,
                     "esp_wifi_set_bandwidths(iface=%d) failed: %s",
                     (int)iface, esp_err_to_name(r));
        } else {
            /* Read the applied value back. A set call whose result is never
             * verified is how an AP ends up silently beaconing HT20 while
             * the bench notes say HT40. */
            wifi_bandwidths_t bw_chk = {};
            esp_err_t gr = esp_wifi_get_bandwidths(iface, &bw_chk);
            ESP_LOGI(TAG_BAND,
                     "bandwidth readback (iface=%d): 2g=%d 5g=%d (%s)",
                     (int)iface, (int)bw_chk.ghz_2g, (int)bw_chk.ghz_5g,
                     esp_err_to_name(gr));
        }
    }

#  if defined(CONFIG_IDF_TARGET_ESP32C5)
    /* On C5 in 5 GHz mode the band switch above leaves the radio on the
     * driver's 5 GHz default (channel 36, per esp_wifi_set_band_mode()
     * attention 6), not on the user's choice. Override it explicitly.
     *
     * The secondary channel is deliberately passed as WIFI_SECOND_CHAN_NONE
     * even for HT40: esp_wifi_set_channel() attention 6 states that in the
     * 5 GHz band the secondary channel is determined by the primary channel
     * per 802.11 and "any manually configured second channel will be
     * ignored". Passing WIFI_SECOND_CHAN_BELOW here (as this code used to)
     * was a no-op that made the log claim a secondary channel the tool had
     * not actually chosen; read it back from the driver instead. */
    if (mode == WIFI_BAND_MODE_5G_ONLY || mode == WIFI_BAND_MODE_AUTO) {
        /* Empirical correction (2026-08-02): the esp_wifi_set_channel()
         * attention-6 claim that a manually configured second channel "will
         * be ignored" in the 5 GHz band did NOT hold for the softAP BSS
         * width on this chip: passing WIFI_SECOND_CHAN_NONE left the BSS at
         * 20 MHz (readback secondary=NONE) even with the 5 GHz bandwidth
         * accepted as BW40. Pass the 802.11-correct pairing direction when
         * HT40 is configured: the lower member of a 36/40-style pair takes
         * its secondary ABOVE, the upper member BELOW. */
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
#  if defined(CONFIG_ESP32_CSI_TOOL_C5_5G_BANDWIDTH_HT40)
        second = ((CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL / 4) % 2 == 0)
                     ? WIFI_SECOND_CHAN_BELOW
                     : WIFI_SECOND_CHAN_ABOVE;
#  endif
        r = esp_wifi_set_channel((uint8_t)CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL,
                                 second);
        if (r != ESP_OK) {
            ESP_LOGW(TAG_BAND,
                     "esp_wifi_set_channel(5g %d, %s) failed: %s",
                     (int)CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL,
                     ESP32_CSI_TOOL_C5_5G_BW_NAME,
                     esp_err_to_name(r));
        } else {
            /* Read the channel back. A set call whose result is never verified
             * is how a receiver ends up silently listening on the wrong
             * channel; the 2.4 GHz path in passive/main.cc gained the same
             * readback for the same reason. */
            uint8_t got_ch = 0;
            wifi_second_chan_t got_sec = WIFI_SECOND_CHAN_NONE;
            esp_err_t gr = esp_wifi_get_channel(&got_ch, &got_sec);
            const char *sec_name = got_sec == WIFI_SECOND_CHAN_ABOVE ? "ABOVE"
                                 : got_sec == WIFI_SECOND_CHAN_BELOW ? "BELOW"
                                 : "NONE";
            ESP_LOGI(TAG_BAND,
                     "C5 5 GHz channel %d, bandwidth %s "
                     "(readback: channel=%d secondary=%s, %s)",
                     (int)CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL,
                     ESP32_CSI_TOOL_C5_5G_BW_NAME,
                     (int)got_ch, sec_name, esp_err_to_name(gr));
            if (gr == ESP_OK &&
                got_ch != (uint8_t)CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL) {
                ESP_LOGE(TAG_BAND,
                         "CHANNEL MISMATCH: asked for 5 GHz ch %d, radio is on %d",
                         (int)CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL, (int)got_ch);
            }
            ESP_LOGW(TAG_BAND,
                     "Reminder: ESP32-C5 5 GHz CSI is subject to upstream "
                     "issue espressif/esp-idf#18493. Verify empirically that "
                     "the CSI buffer changes between frames before relying "
                     "on the data for analysis.");
        }
    }
#  endif

    return ESP_OK;
#else  /* older targets without the v5.4 dual-band API */
#  if defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40) && CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40
    /* Pre-HE chips: classic per-interface bandwidth API. In WIFI_MODE_NULL
     * (passive promiscuous) there is no interface, the call fails, and the
     * driver default stays; the log line makes that visible. */
    {
        esp_err_t rbw = esp_wifi_set_bandwidth(iface, WIFI_BW40);
        if (rbw != ESP_OK) {
            ESP_LOGW(TAG_BAND,
                     "esp_wifi_set_bandwidth(iface=%d, HT40) failed: %s",
                     (int)iface, esp_err_to_name(rbw));
        } else {
            ESP_LOGI(TAG_BAND, "2.4 GHz bandwidth HT40 (iface=%d)", (int)iface);
        }
    }
    return ESP_OK;
#  else
    (void)iface;
    return ESP_OK;
#  endif
#endif
}

/*
 * After STA association, compare what the AP advertises against which acquire
 * slots this build opened, and warn on either mismatch.
 *
 * The acquire slot is selected by the PPDU format of the received frame and
 * does not fall back to a neighbouring slot, so a format the configuration
 * does not name produces no record at all. Nothing reports it: the station
 * associates, the link carries traffic, the closure residual reads 0.0000%
 * because the frames never reach the callback, and the record count is zero.
 * Both directions of the mismatch have now been measured:
 *
 *   HE  acquire_csi_su=1 against an AP with no 802.11ax gives no HE-LTF,
 *       because there are no HE preambles in the air.
 *   VHT acquire_csi_vht=0 against an 802.11ac AP loses every data frame,
 *       since 11ac data runs at VHT rates. On one 5 GHz link driven at
 *       100 datagrams/s this held the delivered rate at 1.21 frames/s, and
 *       setting the flag on the same link took it to 100.56.
 *
 * Only the VHT half is new; it is the case the DFS bring-up spent a session
 * on before the cause was found.
 *
 * No-op on non-HE chips, which carry no acquire_csi_* fields at all, and on
 * any configuration where neither mismatch can arise. Safe to call from any
 * STA-mode app after the STA_GOT_IP event.
 */
static inline esp_err_t wifi_band_check_ap_acquire_match(void) {
#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)
    wifi_ap_record_t ap_info = {};
    esp_err_t r = esp_wifi_sta_get_ap_info(&ap_info);
    if (r != ESP_OK) {
        ESP_LOGW(TAG_BAND,
                 "esp_wifi_sta_get_ap_info() failed: %s; cannot verify the "
                 "AP capabilities against the acquire configuration",
                 esp_err_to_name(r));
        return r;
    }
#  if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU
    if (!ap_info.phy_11ax) {
        ESP_LOGW(TAG_BAND,
                 "AP \"%s\" does NOT advertise 802.11ax. "
                 "CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU=y is set but no HE-LTF "
                 "frames will be received. Either connect to a Wi-Fi 6 AP, "
                 "or set ACQUIRE_HE_SU=n in menuconfig for an 11n-only run.",
                 (const char *)ap_info.ssid);
    } else {
        ESP_LOGI(TAG_BAND,
                 "AP \"%s\" advertises 802.11ax; HE-LTF acquisition path active",
                 (const char *)ap_info.ssid);
    }
#  endif
    /* acquire_csi_vht exists on the MAC v3 parts only, the same condition
     * csi_component.h guards the field assignment with. Referencing it
     * unconditionally is what broke the C6 build once already. */
#  if CONFIG_SOC_WIFI_MAC_VERSION_NUM == 3
#    if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_VHT) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_VHT
    if (ap_info.phy_11ac) {
        ESP_LOGI(TAG_BAND,
                 "AP \"%s\" advertises 802.11ac; VHT acquisition path active",
                 (const char *)ap_info.ssid);
    }
#    else
    if (ap_info.phy_11ac) {
        ESP_LOGW(TAG_BAND,
                 "AP \"%s\" advertises 802.11ac but ACQUIRE_VHT is off, so its "
                 "data frames produce no CSI and only beacons and any "
                 "legacy-rate frames will be recorded. Measured on one such "
                 "link: 1.21 frames/s with the slot closed against 100.56 "
                 "with it open. Set ESP32_CSI_TOOL_ACQUIRE_VHT=y in "
                 "menuconfig, which also requires 20 MHz on this chip.",
                 (const char *)ap_info.ssid);
    }
#    endif
#  endif
#endif
    return ESP_OK;
}

#endif /* ESP32_CSI_WIFI_BAND_COMPONENT_H */
