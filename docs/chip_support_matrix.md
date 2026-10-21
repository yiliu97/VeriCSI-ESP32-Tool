# Chip Support Matrix

VeriCSI-ESP32 covers seven Espressif Wi-Fi SoCs. Targets are classified into three tiers based on the level of validation that has been performed and on outstanding upstream issues.

## Tier definitions

- **Tier 1 - Full support.** All three applications build for this target and a self-pair bench test passes.
- **Tier 2 - Experimental support.** The CI build succeeds but at least one upstream IDF issue limits a specific feature. The chip is shipped with a workaround that may reduce CSI quality or restrict the operating mode.
- **Tier 3 - Community support.** API-level compatibility exists but no main-line testing is performed. Users may build and flash at their own risk.
- **Tier 4 - Not supported.** The chip lacks a Wi-Fi radio.

## Matrix

| Target | Tier | Verified board | Wi-Fi standard | Sink (default) | HE-LTF | Notes |
|---|---|---|---|---|---|---|
| `esp32` | 1 | TTGO T8 V1.7, NodeMCU ESP32-WROOM-32U | 802.11 b/g/n | UART + optional SD | no | Original Hernandez 2020 hardware. Dual-core Xtensa LX6. |
| `esp32s2` | 1 | LOLIN ESP32-S2 mini | 802.11 b/g/n | UART + TinyUSB CDC | no | Single-core LX7, no native USB-Serial-JTAG. TinyUSB CDC ACM sink default-on; managed-component dep `espressif/esp_tinyusb` only pulled for S2 builds. |
| `esp32s3` | 1 | ESP32-S3-DevKitC-1 | 802.11 b/g/n | USB-Serial-JTAG | no | Headline dual-core LX7 board. Used as the K5 packet-rate-precision benchmark host. |
| `esp32c3` | 1 | ESP32-C3-DevKitM-1 | 802.11 b/g/n | USB-Serial-JTAG | no | Single-core RISC-V. ISR-side queue is mandatory. |
| `esp32c5` | 2 | DFRobot FireBeetle 2 ESP32-C5 (DFR1222) | 802.11 b/g/n/ax (dual band) | USB-Serial-JTAG | yes | 2.4 GHz default; **5 GHz available as opt-in** via `ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY`. The #18493 frozen-buffer symptom did not reproduce on v6.0.1 (fifteen 5 GHz sessions, frozen fraction 0.0000); the 106-byte record layout remains an open question, see `docs/known_issues.md`. |
| `esp32c6` | 1 | Diymore ESP32-C6-WROOM1 | 802.11 b/g/n/ax (2.4 GHz) | USB-Serial-JTAG | yes | Headline RISC-V Wi-Fi 6 chip. `chip_aware_decoder` handles #14271. |
| `esp32c61` | 2 | ESP32-C61-DEVKITC-1-N8R2 | 802.11 b/n/ax (2.4 GHz, no 11g) | USB-Serial-JTAG | yes | 11g disabled per #18118. `acquire_csi_force_lltf=1` by default. |
| `esp32c2` | 3 | n/a | 802.11 b/g/n | n/a | no | Not in `esp-csi` "full series support" list. Build at user's risk. |
| `esp32h2` / `esp32h4` / `esp32h21` / `esp32p4` | 4 | n/a | n/a | n/a | n/a | No Wi-Fi radio. |

## Wi-Fi 6 (802.11ax) status

The chips with `CONFIG_SOC_WIFI_HE_SUPPORT` (ESP32-C5 / C6 / C61) get full Wi-Fi 6 capability: the MAC negotiates 11ax via `WIFI_PROTOCOL_11AX` in the protocol bitmask, AND the CSI driver captures HE-LTF preambles via the `acquire_csi_su` / `acquire_csi_mu` flags in `wifi_csi_acquire_config_t`. Both layers are required.

| Target | MAC 11ax | HE-LTF capture | Notes |
|---|---|---|---|
| `esp32`     | no  | no  | Chip predates Wi-Fi 6 |
| `esp32s2`   | no  | no  | 802.11n only |
| `esp32s3`   | no  | no  | 802.11n only |
| `esp32c3`   | no  | no  | 802.11n only |
| `esp32c5`   | yes | yes (2.4 GHz + 5 GHz) | Dual-band Wi-Fi 6 |
| `esp32c6`   | yes | yes (2.4 GHz)         | #14271 affects HE-LTF subcarrier ordering |
| `esp32c61`  | yes | yes (2.4 GHz, 11g off) | #18118 forces 11g off |

Toggle 11ax via `VeriCSI-ESP32 Config -> Wi-Fi 6 HE-LTF acquisition (C5 / C6 / C61 only) -> Enable 802.11ax`. Default y on HE-capable chips.

## Per-record receive gain (schema v2.2)

The `agc_gain` / `fft_gain` columns and the receive-gain lock
(`CONFIG_ESP32_CSI_TOOL_RX_GAIN_CTRL` / `RX_GAIN_FORCE`) depend on the
vendor component `esp_csi_gain_ctrl`, which ships as prebuilt archives.
The archives for two targets contain no code, so the feature is
target-gated below the Kconfig level (`docs/known_issues.md`).

| Target | Component code present | Gain columns | Lock | Hardware-verified |
|---|---|---|---|---|
| `esp32`, `esp32s2` | no (empty archive, IDF 4.4 to 6.0) | `-1` (option must stay `n`) | unavailable | n/a |
| `esp32c6` | yes | per-record values | works | yes: 13 distinct free values; forced 40 reads back 40 in 1230/1230 records |
| `esp32c3`, `esp32c5`, `esp32c61`, `esp32s3` | yes | per-record values | expected | build-verified only |

## PSRAM ring buffer

The CSI consumer task uses a FreeRTOS queue to decouple the Wi-Fi rx callback from the (potentially blocking) sink writer. The default queue holds 64 items in internal SRAM (~32 KB), enough for ~0.6 s of headroom at 100 Hz. Targets with PSRAM can opt into a much larger PSRAM-backed queue via `CONFIG_ESP32_CSI_TOOL_PSRAM_RING_BUFFER` for long-recording sessions that may experience multi-second sink stalls.

| Target | PSRAM hardware | sdkconfig.defaults | Tool support |
|---|---|---|---|
| `esp32`     | Quad SPI, up to 8 MB (board-dependent) | not set | manually set `CONFIG_SPIRAM=y` if your board has it |
| `esp32s2`   | Quad SPI, up to 8 MB                   | not set | as above |
| `esp32s3`   | **Octal SPI, up to 16 MB**             | `CONFIG_SPIRAM=y, MODE_OCT=y, SPEED_80M=y` | recommended host for the PSRAM ring buffer |
| `esp32c3`   | none                                   | n/a     | not supported |
| `esp32c5`   | Quad SPI, up to 8 MB                   | not set | as `esp32` |
| `esp32c6`   | none                                   | n/a     | not supported |
| `esp32c61`  | Quad SPI, up to 8 MB                   | not set | as `esp32` |

Use the `CSI_HAS_PSRAM_OCTAL` / `CSI_HAS_PSRAM_QUAD` / `CSI_HAS_PSRAM_ENABLED` macros from `_components/board_capabilities.h` to gate downstream code that depends on external memory.

## Adding a new target

To extend the support matrix to a future ESP32 SoC:

1. Add `sdkconfig.defaults.<new_target>` with the target-specific Kconfig overrides (band restriction, console route, force-LLTF if applicable).
2. Build all three applications for the new target and confirm they compile.
3. Update this matrix.
4. If the target is Tier 2, also document the limiting upstream issue in `docs/known_issues.md`.
