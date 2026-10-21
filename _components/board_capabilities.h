#ifndef ESP32_CSI_BOARD_CAPABILITIES_H
#define ESP32_CSI_BOARD_CAPABILITIES_H

/*
 * board_capabilities.h
 * --------------------
 *
 * Compile-time capability matrix for the seven supported targets:
 *   ESP32 (LX6 dual-core)
 *   ESP32-S2 (LX7 single-core, USB-OTG, no BT)
 *   ESP32-S3 (LX7 dual-core, USB-OTG, PIE/SIMD, Octal PSRAM)
 *   ESP32-C3 (RISC-V single-core)
 *   ESP32-C5 (RISC-V single-core, dual-band Wi-Fi 6)
 *   ESP32-C6 (RISC-V single-core, Wi-Fi 6 + 802.15.4)
 *   ESP32-C61 (RISC-V single-core, cost-reduced C6)
 *
 * The CSI_HAS_* macros exposed here are *tool-level* abstractions over IDF's
 * own CONFIG_SOC_* / CONFIG_IDF_TARGET_* family. Where IDF already exposes a
 * SOC capability flag we reuse it directly so the tool never disagrees with
 * the IDF SOC headers; where IDF doesn't expose a flag we derive from the
 * concrete target macro.
 *
 * Modules under `_components/` (sd_component, usb_jtag_component,
 * binary_sink, ...) can guard their compilation with `#if CSI_HAS_*` for
 * documentation-grade readability, while still falling back to the
 * underlying CONFIG_* macros for IDF-internal compatibility.
 *
 * Reference: ESP32-CSI-Tool-v6-NewFeatures.md section 4.
 */

#include "sdkconfig.h"

/* ------------------------------------------------------------------------- */
/*                            Wi-Fi capabilities                             */
/* ------------------------------------------------------------------------- */

/* HE (Wi-Fi 6 / 802.11ax) radio + MAC support. C5 / C6 / C61. */
#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)
#  define CSI_HAS_WIFI_HE             1
#else
#  define CSI_HAS_WIFI_HE             0
#endif

/* 5 GHz radio. Only ESP32-C5 in the VeriCSI-ESP32 target set. */
#if defined(CONFIG_IDF_TARGET_ESP32C5)
#  define CSI_HAS_5GHZ_RADIO          1
#else
#  define CSI_HAS_5GHZ_RADIO          0
#endif

/* ------------------------------------------------------------------------- */
/*                                  USB                                       */
/* ------------------------------------------------------------------------- */

/* Native USB-Serial-JTAG controller (USB-CDC console at 12 Mbps). All chips
 * after the ESP32 classic and S2 have it. */
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32C61)
#  define CSI_HAS_USB_SERIAL_JTAG     1
#else
#  define CSI_HAS_USB_SERIAL_JTAG     0
#endif

/* USB-OTG controller (TinyUSB CDC / MSC capable). Only S2 / S3. */
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#  define CSI_HAS_USB_OTG             1
#else
#  define CSI_HAS_USB_OTG             0
#endif

/* ------------------------------------------------------------------------- */
/*                                Bluetooth                                   */
/* ------------------------------------------------------------------------- */

/* BT Classic (BR/EDR). Only the original ESP32. */
#if defined(CONFIG_IDF_TARGET_ESP32)
#  define CSI_HAS_BT_CLASSIC          1
#else
#  define CSI_HAS_BT_CLASSIC          0
#endif

/* BLE (any version). All chips except S2 (no Bluetooth at all). */
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#  define CSI_HAS_BLE                 0
#else
#  define CSI_HAS_BLE                 1
#endif

/* ------------------------------------------------------------------------- */
/*                                   CPU                                      */
/* ------------------------------------------------------------------------- */

/* Dual-core capable. ESP32 (LX6 dual) and ESP32-S3 (LX7 dual). The actual
 * runtime config may still be unicore via CONFIG_FREERTOS_UNICORE; this flag
 * captures hardware capability. */
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#  define CSI_HAS_DUAL_CORE_HW        1
#else
#  define CSI_HAS_DUAL_CORE_HW        0
#endif

/* Effective dual-core (hardware capable AND not forced to unicore in
 * sdkconfig). Modules pinning tasks to a specific core should test this. */
#if CSI_HAS_DUAL_CORE_HW && !defined(CONFIG_FREERTOS_UNICORE)
#  define CSI_HAS_DUAL_CORE           1
#else
#  define CSI_HAS_DUAL_CORE           0
#endif

/* PIE / SIMD vector instructions (ESP-DL hardware-accelerated path). Only S3. */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#  define CSI_HAS_AI_SIMD             1
#else
#  define CSI_HAS_AI_SIMD             0
#endif

/* ------------------------------------------------------------------------- */
/*                                 Memory                                     */
/* ------------------------------------------------------------------------- */

/* Hardware capable of Octal SPI PSRAM. Only S3. The actual sdkconfig may
 * still leave PSRAM disabled - see CSI_HAS_PSRAM_ENABLED for that. */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#  define CSI_HAS_PSRAM_OCTAL         1
#else
#  define CSI_HAS_PSRAM_OCTAL         0
#endif

/* Hardware capable of Quad SPI PSRAM (any flavour). Excludes the chips
 * Espressif explicitly does not support: C3, C6. */
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C61)
#  define CSI_HAS_PSRAM_QUAD          1
#else
#  define CSI_HAS_PSRAM_QUAD          0
#endif

/* PSRAM actually enabled in this build (sdkconfig CONFIG_SPIRAM=y). Modules
 * that allocate from MALLOC_CAP_SPIRAM should guard their alloc call with
 * this flag, not the hardware-capability flags above. */
#if defined(CONFIG_SPIRAM)
#  define CSI_HAS_PSRAM_ENABLED       1
#else
#  define CSI_HAS_PSRAM_ENABLED       0
#endif

/* ------------------------------------------------------------------------- */
/*                              Coprocessors                                  */
/* ------------------------------------------------------------------------- */

/* RISC-V ULP coprocessor (ulp-riscv component). S2 and S3 only. */
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#  define CSI_HAS_ULP_RISCV           1
#else
#  define CSI_HAS_ULP_RISCV           0
#endif

/* LP Core (newer-generation low-power RISC-V core). C5 / C6 carry a 20 / 40
 * MHz LP RISC-V; the original ESP32-S2 / S3 have only the older ULP-RISC-V. */
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6)
#  define CSI_HAS_LP_CORE             1
#else
#  define CSI_HAS_LP_CORE             0
#endif

/* ------------------------------------------------------------------------- */
/*                                Storage                                     */
/* ------------------------------------------------------------------------- */

/* Has the TTGO T8 V1.7 default SD card pin map (GPIO 13/14/15/2). Only the
 * ESP32 classic; other targets need an explicit Kconfig override of the SD
 * pin assignments to use an external adapter. */
#if defined(CONFIG_IDF_TARGET_ESP32)
#  define CSI_HAS_SD_CARD_DEFAULT     1
#else
#  define CSI_HAS_SD_CARD_DEFAULT     0
#endif

/* ------------------------------------------------------------------------- */
/*                       Compile-time consistency checks                      */
/* ------------------------------------------------------------------------- */

/* CSI_HAS_PSRAM_ENABLED implies the build is on a chip with at least Quad
 * PSRAM. Trip a static_assert if someone configures CONFIG_SPIRAM=y on a
 * chip Espressif does not support (C3 / C6); the IDF should also reject this
 * at the SOC layer, but a tool-level error is friendlier. */
#if CSI_HAS_PSRAM_ENABLED && !CSI_HAS_PSRAM_QUAD
_Static_assert(0, "CONFIG_SPIRAM=y is set on a target without PSRAM support "
                  "(check sdkconfig.defaults.<target>)");
#endif

#endif /* ESP32_CSI_BOARD_CAPABILITIES_H */
