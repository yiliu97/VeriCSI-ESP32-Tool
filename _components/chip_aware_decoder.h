#ifndef ESP32_CSI_CHIP_AWARE_DECODER_H
#define ESP32_CSI_CHIP_AWARE_DECODER_H

#include "esp_wifi.h"
#include "sdkconfig.h"

/*
 * Chip-aware decoder
 *
 * Recent ESP32 SoCs report CSI buffers of different lengths and subcarrier
 * orderings depending on the underlying PHY layer behaviour. Three issues are
 * tracked in docs/known_issues.md:
 *
 *   - espressif/esp-idf#14271 (ESP32-C6, missing L-LTF + HT-LTF subcarrier reorder)
 *   - espressif/esp-idf#18118 (ESP32-C61, 802.11g CSI broken)
 *   - espressif/esp-idf#18493 (ESP32-C5, static IQ buffer on 5 GHz)
 *
 * Rather than hide these in #ifdefs scattered across the firmware, we tag every
 * captured frame with a layout id and emit that id into the v2 CSV schema
 * (column "subcarrier_layout_id"). Host-side parsers (python_utils/chip_aware_decoder.py)
 * then know how to reorder / drop / interpret the buffer correctly.
 */

typedef enum {
    CSI_LAYOUT_STANDARD_HT20    = 0,    // ESP32 / S2 / S3 / C3 default HT20 (LLTF + HT-LTF)
    CSI_LAYOUT_C6_HT20_NO_LLTF  = 1,    // C6 in HE acquisition mode, issue #14271
    CSI_LAYOUT_C61_FORCE_LLTF   = 2,    // C61 with acquire_csi_force_lltf=1 (workaround for #18118)
    CSI_LAYOUT_C5_5GHZ          = 3,    // C5 on a 5 GHz channel (opt-in via Kconfig; see docs/known_issues.md #18493)
    CSI_LAYOUT_HE_LTF_SU        = 4,    // C5 / C6 / C61 with acquire_csi_su=1
    CSI_LAYOUT_HE_LTF_MU        = 5,    // C5 / C6 / C61 with acquire_csi_mu=1
    CSI_LAYOUT_HT40             = 6,    // any chip with acquire_csi_ht40=1
    CSI_LAYOUT_UNKNOWN          = 99
} csi_layout_id_t;

/*
 * Map an enum value to the string written to the CSV column.
 * Keep these strings stable - python_utils/chip_aware_decoder.py keys on them.
 */
static inline const char *chip_aware_layout_name(csi_layout_id_t id) {
    switch (id) {
        case CSI_LAYOUT_STANDARD_HT20:   return "STANDARD_HT20";
        case CSI_LAYOUT_C6_HT20_NO_LLTF: return "C6_HT20_NO_LLTF";
        case CSI_LAYOUT_C61_FORCE_LLTF:  return "C61_FORCE_LLTF";
        case CSI_LAYOUT_C5_5GHZ:         return "C5_5GHZ";
        case CSI_LAYOUT_HE_LTF_SU:       return "HE_LTF_SU";
        case CSI_LAYOUT_HE_LTF_MU:       return "HE_LTF_MU";
        case CSI_LAYOUT_HT40:            return "HT40";
        case CSI_LAYOUT_UNKNOWN:
        default:                         return "UNKNOWN";
    }
}

/*
 * Infer the layout id from chip target, configuration and runtime metadata.
 * Called once per frame on the consumer-task side (not in the rx callback).
 *
 * The classification logic is intentionally conservative: it follows the
 * empirical mapping documented in docs/known_issues.md and only declares a
 * non-default layout when both the chip target AND a runtime symptom match.
 */
static inline csi_layout_id_t chip_aware_infer_layout(const wifi_csi_info_t *info) {
    if (info == NULL) {
        return CSI_LAYOUT_UNKNOWN;
    }

#if CONFIG_IDF_TARGET_ESP32C5
    // Issue #18493: 5 GHz channels (36-165) deliver static IQ patterns.
    if (info->rx_ctrl.channel >= 36) {
        return CSI_LAYOUT_C5_5GHZ;
    }
#endif

#if CONFIG_IDF_TARGET_ESP32C61
    // Issue #18118: when force_lltf is enabled we are running the workaround
    // and only LLTF data is reliable.
#  if defined(CONFIG_ESP32_CSI_TOOL_FORCE_LLTF) || defined(CONFIG_SHOULD_COLLECT_ONLY_LLTF)
    return CSI_LAYOUT_C61_FORCE_LLTF;
#  endif
#endif

#if CONFIG_IDF_TARGET_ESP32C6
    // Issue #14271: in HE acquisition mode the buffer comes back at 128 bytes
    // (HT-LTF only, no L-LTF) instead of the expected 256.
    if (info->len == 128) {
        return CSI_LAYOUT_C6_HT20_NO_LLTF;
    }
#endif

    // HT40 on HE-era chips (acquire_csi_ht40=1): hardware-measured lengths
    // are 256 B on the C6 (40 MHz HT-LTF, no L-LTF) and 234 B on the C5.
    // Must run BEFORE the SU/MU blanket returns below, which are
    // length-blind and would claim these records.
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT40) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT40
    if (info->len == 256 || info->len == 234) {
        return CSI_LAYOUT_HT40;
    }
#endif

    // HE-LTF acquisition (any RISC-V Wi-Fi 6 chip with acquire_csi_su / mu)
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_MU
    return CSI_LAYOUT_HE_LTF_MU;
#endif
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU) && CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU
    return CSI_LAYOUT_HE_LTF_SU;
#endif

#if !defined(CONFIG_SOC_WIFI_HE_SUPPORT)
    // Pre-HE 40 MHz HT frame: L-LTF (128 B) plus 40 MHz HT-LTF (256 B).
    // No 20 MHz configuration produces a buffer above 256 B on these chips
    // (hardware-measured: 384 B on the ESP32-C3 in a 40 MHz BSS).
    if (info->len > 256) {
        return CSI_LAYOUT_HT40;
    }
#endif

    return CSI_LAYOUT_STANDARD_HT20;
}

/*
 * Reports whether the consumer task should drop a frame with this layout.
 *
 * change: ESP32-C5 5 GHz frames are NO LONGER dropped here.
 * They are still flagged as `C5_5GHZ` in the v2 CSV `subcarrier_layout_id`
 * column so downstream tooling knows the band, but the user opting into
 * 5 GHz via `CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY` accepts the
 * upstream issue #18493 risk and we deliver whatever the firmware
 * actually produces. See docs/known_issues.md for the verification recipe.
 *
 * The function still flags `UNKNOWN` as drop-worthy because that means
 * `chip_aware_infer_layout` could not classify the frame at all, which
 * implies a parsing bug rather than a user choice.
 */
static inline bool chip_aware_should_drop(csi_layout_id_t id) {
    return id == CSI_LAYOUT_UNKNOWN;
}

#endif // ESP32_CSI_CHIP_AWARE_DECODER_H
