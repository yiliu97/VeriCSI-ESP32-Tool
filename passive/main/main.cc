#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "../../_components/nvs_component.h"
#include "../../_components/sd_component.h"
#include "../../_components/csi_component.h"
#include "../../_components/time_component.h"
#include "../../_components/input_component.h"
#include "../../_components/wifi_band_component.h"

/*
 * Passive sniffer (VeriCSI-ESP32, ESP-IDF v6.0.1)
 *
 * Differences from the ESP32-CSI-Tool lineage (Hernandez 2020):
 *   - esp_spi_flash.h removed (gone in IDF v5+).
 *   - esp_http_server.h removed (was unused in passive).
 *   - esp_netif_init / esp_event_loop_create_default added: required in v5+
 *     even for promiscuous-only mode.
 *   - esp_wifi_init() return value is now tolerated rather than asserted.
 */

#ifdef CONFIG_WIFI_CHANNEL
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL
#else
#define WIFI_CHANNEL 6
#endif

#ifdef CONFIG_SHOULD_COLLECT_CSI
#define SHOULD_COLLECT_CSI 1
#else
#define SHOULD_COLLECT_CSI 0
#endif

#ifdef CONFIG_SHOULD_COLLECT_ONLY_LLTF
#define SHOULD_COLLECT_ONLY_LLTF 1
#else
#define SHOULD_COLLECT_ONLY_LLTF 0
#endif

#ifdef CONFIG_SEND_CSI_TO_SERIAL
#define SEND_CSI_TO_SERIAL 1
#else
#define SEND_CSI_TO_SERIAL 0
#endif

#ifdef CONFIG_SEND_CSI_TO_SD
#define SEND_CSI_TO_SD 1
#else
#define SEND_CSI_TO_SD 0
#endif

static const char *TAG = "passive";

void config_print() {
    printf("\n\n\n\n\n\n\n\n");
    printf("-----------------------\n");
    printf("VeriCSI-ESP32 Settings\n");
    printf("-----------------------\n");
    printf("PROJECT_NAME: %s\n", "PASSIVE");
    printf("CONFIG_ESPTOOLPY_MONITOR_BAUD: %d\n", CONFIG_ESPTOOLPY_MONITOR_BAUD);
    // See active_sta/main/main.cc for explanation of this guard.
#if defined(CONFIG_ESP_CONSOLE_UART_BAUDRATE)
    printf("CONFIG_ESP_CONSOLE_UART_BAUDRATE: %d\n", CONFIG_ESP_CONSOLE_UART_BAUDRATE);
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    printf("CONFIG_ESP_CONSOLE_UART_BAUDRATE: (USB-Serial-JTAG, baud N/A)\n");
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    printf("CONFIG_ESP_CONSOLE_UART_BAUDRATE: (USB-CDC, baud N/A)\n");
#else
    printf("CONFIG_ESP_CONSOLE_UART_BAUDRATE: (unknown console)\n");
#endif
    printf("IDF_TARGET: %s\n", CONFIG_IDF_TARGET);
    printf("IDF_VER: %s\n", IDF_VER);
    printf("-----------------------\n");
    printf("WIFI_CHANNEL: %d\n", WIFI_CHANNEL);
    printf("SHOULD_COLLECT_CSI: %d\n", SHOULD_COLLECT_CSI);
    printf("SHOULD_COLLECT_ONLY_LLTF: %d\n", SHOULD_COLLECT_ONLY_LLTF);
    printf("SEND_CSI_TO_SERIAL: %d\n", SEND_CSI_TO_SERIAL);
    printf("SEND_CSI_TO_SD: %d\n", SEND_CSI_TO_SD);
    printf("-----------------------\n");
    printf("\n\n\n\n\n\n\n\n");
}

void passive_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means esp_wifi was already initialised - not fatal.
        ESP_LOGW(TAG, "esp_wifi_init returned %s; continuing", esp_err_to_name(ret));
    }
#if defined(CONFIG_IDF_TARGET_ESP32C5)
    /* ESP32-C5: promiscuous CSI delivers zero callbacks in WIFI_MODE_NULL
     * (observed 2026-08-03: sniffer enabled, filter and channel readback
     * correct, csi_rx stayed 0; the identical configuration produces CSI on
     * the C6 and C61). Run the sniffer on an idle STA-mode radio instead;
     * no association is attempted. The vendor esp-csi C5 example likewise
     * uses STA mode and avoids promiscuous capture. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());

    /*
     * Disable Wi-Fi power save. ESP-IDF documents rx_ctrl.timestamp as
     * "precise only if modem sleep or light sleep is not enabled", and
     * local_timestamp is the only trustworthy per-packet clock this tool has.
     * active_ap and active_sta already did this (main.cc:148 / :178); passive
     * did not, which silently degraded timestamp precision in exactly the role
     * recommended as the clean receiver for timing measurements.
     */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /*
     * Apply ESP32-C5 band selection (2 GHz / 5 GHz / auto) and
     * the v5.4 dual-band protocol/bandwidth API on C5 / C6 / C61. No-op on
     * older targets. On C5 with 5 GHz selected, wifi_band_apply also sets
     * the 5 GHz channel from CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL; the
     * legacy 2.4 GHz channel call below is then redundant but harmless.
     * Note: passive promiscuous mode in 5 GHz on C5 may need WIFI_MODE_STA
     * rather than WIFI_MODE_NULL if the radio is not enumerated.
     */
    wifi_band_apply(WIFI_IF_STA);

    /*
     * Promiscuous frame filter (Kconfig). The ESP32-CSI-Tool lineage used
     * MASK_DATA only because the design assumes a paired controlled traffic
     * source (active_sta or constant ping); MASK_DATA-only delivers cleanest
     * data-frame CSI with zero beacon noise.
     *
     * For ambient sniffing without a controlled source, MASK_DATA-only
     * produces sparse / bursty captures - beacons are the only reliable
     * baseline and they get filtered out. The default is therefore
     * MASK_MGMT | MASK_DATA which gives ~10 fps beacon baseline plus any
     * data traffic that happens to be on the channel. Users can revert to
     * the lineage behaviour or widen further via Kconfig.
     */
#if defined(CONFIG_ESP32_CSI_TOOL_PASSIVE_FILTER_DATA_ONLY)
    const uint32_t promis_mask = WIFI_PROMIS_FILTER_MASK_DATA;
#elif defined(CONFIG_ESP32_CSI_TOOL_PASSIVE_FILTER_MGMT_CTRL_DATA)
    const uint32_t promis_mask = WIFI_PROMIS_FILTER_MASK_MGMT
                               | WIFI_PROMIS_FILTER_MASK_CTRL
                               | WIFI_PROMIS_FILTER_MASK_DATA;
#elif defined(CONFIG_ESP32_CSI_TOOL_PASSIVE_FILTER_ALL)
    const uint32_t promis_mask = WIFI_PROMIS_FILTER_MASK_ALL;
#else
    /* Default = MGMT + DATA (Kconfig ESP32_CSI_TOOL_PASSIVE_FILTER_MGMT_DATA). */
    const uint32_t promis_mask = WIFI_PROMIS_FILTER_MASK_MGMT
                               | WIFI_PROMIS_FILTER_MASK_DATA;
#endif

    const wifi_promiscuous_filter_t filt = { .filter_mask = promis_mask };

    esp_err_t pr = esp_wifi_set_promiscuous(true);
    esp_err_t pf = esp_wifi_set_promiscuous_filter(&filt);
    ESP_LOGI(TAG, "promiscuous: enable=%s filter=%s mask=0x%08x",
             esp_err_to_name(pr), esp_err_to_name(pf), (unsigned)promis_mask);

    /*
     * 2.4 GHz default channel from CONFIG_WIFI_CHANNEL. On C5 with 5 GHz
     * mode active, wifi_band_apply already set a 5 GHz channel above and
     * this call would re-configure to a 2.4 GHz channel; gate it.
     */
#if !(defined(CONFIG_IDF_TARGET_ESP32C5) && \
      (defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY) || \
       defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_AUTO)))
    int curChannel = WIFI_CHANNEL;
    /* The secondary-channel argument sets the sniffer bandwidth: NONE keeps
     * the promiscuous path at 20 MHz, which cannot capture the 40 MHz
     * HT-LTF of HT40 frames. With ACQUIRE_HT40 (or a pre-HE build that
     * wants HT40 records) the receiver must sniff at 40 MHz on the same
     * primary/secondary pair as the sender. The AP driver on channel 6
     * picks secondary BELOW (verified 2026-08-08: an above-sniffing
     * receiver captured nothing; below captures the full stream). */
#if defined(CONFIG_ESP32_CSI_TOOL_ACQUIRE_HT40) || defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40)
    esp_err_t cr = esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_BELOW);
#else
    esp_err_t cr = esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE);
#endif
    /*
     * Read the channel back. The set call used to be fire-and-forget, so a
     * capture that received nothing gave no way to tell "the channel is wrong"
     * from "the channel is right and nothing is on air" - which is exactly the
     * ambiguity hit on 2026-07-30 when a channel-1 capture came back empty.
     */
    uint8_t got_ch = 0;
    wifi_second_chan_t got_sec = WIFI_SECOND_CHAN_NONE;
    esp_err_t gr = esp_wifi_get_channel(&got_ch, &got_sec);
    ESP_LOGI(TAG, "channel: requested=%d set=%s readback=%d (%s)",
             curChannel, esp_err_to_name(cr), (int)got_ch, esp_err_to_name(gr));
    if (gr == ESP_OK && got_ch != curChannel) {
        ESP_LOGE(TAG, "CHANNEL MISMATCH: asked for %d, radio is on %d",
                 curChannel, (int)got_ch);
    }
#endif
}

extern "C" void app_main(void) {
    config_print();
    nvs_init();
    sd_init();
    passive_init();
    csi_init((char *) "PASSIVE");
    input_loop();
}
