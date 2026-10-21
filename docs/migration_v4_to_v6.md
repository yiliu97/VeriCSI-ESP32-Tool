# Migration Log: ESP-IDF v4.3 → v6.0.1

This document records the breaking changes between ESP-IDF v4.3 (the version
targeted by the original ESP32-CSI-Tool, Hernandez 2020) and v6.0.1 (the
version targeted by VeriCSI-ESP32), along with the fix in each case. It
serves both as a change log and as a guide for downstream users who wish to
port other v4.3-era CSI tools to a modern IDF.

## Affected sub-systems

### Wi-Fi driver

- `[done]` `WIFI_BW_HT20`, `WIFI_BW_HT40` enums removed in v6.0. Replaced by
  `WIFI_BW20`, `WIFI_BW40`. Any call site of `esp_wifi_set_bandwidth()`
  updated accordingly.
- `[done]` `wifi_csi_config_t` extended into `wifi_csi_acquire_config_t`
  (introduced in v5.0). The new struct exposes `acquire_csi_legacy / ht20 /
  ht40 / vht / su / mu / dcm / beamformed / he_stbc_mode / val_scale_cfg /
  dump_ack_en / acquire_csi_force_lltf`. `_components/csi_component.h`
  always uses the new struct.
- `[done]` `esp_wifi_set_csi_config()` renamed to
  `esp_wifi_set_csi_acquire_config()` (v5.x).
- `[done]` `esp_wifi_init()` returns `ESP_ERR_INVALID_STATE` if already
  initialised. The original v4.3 code wrapped the call in `ESP_ERROR_CHECK`
  and would `abort()` on the second call. VeriCSI-ESP32 inspects the return
  value and proceeds gracefully.
- `[done]` Antenna helpers `esp_wifi_set_ant_gpio` / `esp_wifi_get_ant_gpio`
  / `esp_wifi_set_ant` / `esp_wifi_get_ant` moved to the `esp_phy` component
  as `esp_phy_set_ant_gpio` etc. The original tool did not use these APIs,
  but if a downstream fork did, the symbol must be renamed.

### Drivers (legacy peripheral drivers removed in v6.0)

- `[done]` `driver/timer.h` (legacy Timer Group) removed. VeriCSI-ESP32 uses
  `driver/gptimer.h` for the application-layer timer in
  `gptimer_broadcast.h`.
- `[done]` `driver/sdspi_host.h` API reshaped. The v4.3 idiom
  `SDSPI_SLOT_CONFIG_DEFAULT()` is gone; `_components/sd_component.h` is
  written against the `sdspi_device_config_t` + `spi_bus_initialize` +
  `esp_vfs_fat_sdspi_mount` flow.
- `[done]` `esp_log_buffer_hex` symbol replaced by the macro
  `ESP_LOG_BUFFER_HEX`.

### Includes

- `[done]` `esp_spi_flash.h` removed. The original tool included it in
  `active_sta/main/main.cc` and `active_ap/main/main.cc` but did not use any
  symbol from it. The include line is dropped.

### Build system

- `[done]` Default C standard upgraded to `gnu23`, C++ to `gnu++26`.
  Compiler warnings are treated as errors by default; the format-string and
  implicit-declaration warnings that surfaced are fixed.
- `[done]` Default libc switched from Newlib to Picolibc. Formatting used by
  the firmware (`%d`, `%s`, `%f`) behaves identically.
- `[done]` Python ≥ 3.10 and CMake ≥ 3.22 required.
  `python_utils/requirements.txt` declares the minimum Python version.

### Components moved out of core IDF

- `[done]` `wifi_provisioning` renamed to `network_provisioning` and moved
  to managed components. Not used by this tool.
- `[done]` `cJSON` moved to a managed component. Not used by this tool.
- `[done]` MbedTLS upgraded to 4.0 with PSA Crypto API. Only used indirectly
  through `esp_http_client` for the SETTIME flow; the upgrade is
  transparent.
