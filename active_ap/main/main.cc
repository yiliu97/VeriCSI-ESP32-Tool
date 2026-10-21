#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"   // MACSTR / MAC2STR moved here in IDF v5+
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "../../_components/nvs_component.h"
#include "../../_components/sd_component.h"
#include "../../_components/csi_component.h"
#include "../../_components/time_component.h"
#include "../../_components/input_component.h"
#include "../../_components/sockets_component.h"
#include "../../_components/wifi_band_component.h"

#ifdef CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
#include "../../_components/gptimer_broadcast.h"
#endif

/*
 * Active AP - SoftAP that other Active STA nodes (or any client) connect to.
 *
 * Differences from the ESP32-CSI-Tool lineage (Hernandez 2020):
 *   - esp_spi_flash.h removed (gone in IDF v5+).
 *   - esp_netif_init() now called explicitly.
 *   - esp_wifi_init() return value tolerated rather than asserted.
 *   - Optional application-layer gptimer-driven broadcast: when
 *     CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST is enabled, the AP
 *     emits a known-payload broadcast frame at exactly 100 / 200 / 500 /
 *     1000 Hz (CONFIG_ESP32_CSI_TOOL_BROADCAST_RATE_HZ). Implementation is
 *     _components/gptimer_broadcast.h.
 */

/*
 * The examples use WiFi configuration that you can set via 'idf.py menuconfig'.
 *
 * If you'd rather not, just change the below entries to strings with
 * the config you want - ie #define ESP_WIFI_SSID "mywifissid"
 */
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define MAX_STA_CONN       16

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

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "Active CSI collection (AP)";

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void softap_init() {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_wifi_init returned %s; continuing", esp_err_to_name(ret));
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_ap_config_t wifi_ap_config = {};
    /*
     * The AP channel must be valid for the band the driver comes up in, and
     * the driver PERSISTS the band mode in NVS: after one boot in 5G_ONLY,
     * every later boot initialises in 5G_only BEFORE wifi_band_apply() runs.
     * Passing the 2.4 GHz WIFI_CHANNEL here then aborts esp_wifi_set_config
     * with "Channel number 6 (2G) is not supported in current band mode
     * (5G_only)" - observed on ESP32-C5 hardware 2026-08-01 as a boot loop
     * that only appeared from the SECOND boot after switching to 5 GHz (the
     * first boot still had the 2.4 GHz default band mode at set_config time).
     * So on C5 in 5 GHz mode the config channel is the 5 GHz channel from the
     * start; wifi_band_apply() still runs afterwards and re-asserts it.
     */
#if defined(CONFIG_IDF_TARGET_ESP32C5) && \
    (defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY) || \
     defined(CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_AUTO))
    wifi_ap_config.channel = CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL;
#else
    wifi_ap_config.channel = WIFI_CHANNEL;
#endif
    wifi_ap_config.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_ap_config.max_connection = MAX_STA_CONN;
    /*
     * Beacon interval. Left zero-initialised the driver applies its 100 TU
     * default; exposing it lets an experiment vary the dominant periodic MAC
     * event in the capture. ESP-IDF constrains this to multiples of 100 TU in
     * the range 100..60000 (esp_wifi_types_generic.h:533), where 1 TU = 1024 us,
     * so 100 TU = 102.4 ms = 9.765625 Hz.
     *
     * Note the granularity: because the harmonics of a 200/300/... TU beacon
     * still contain the 9.765625 Hz component, changing this value on an ESP32
     * SoftAP shifts the AMPLITUDE of a beacon-locked spectral line, not its
     * FREQUENCY. Use it as a supporting check; sweep the sounding period for
     * the frequency-domain test.
     */
    wifi_ap_config.beacon_interval = CONFIG_ESP32_CSI_TOOL_BEACON_INTERVAL_TU;

    wifi_config_t wifi_config = {
            .ap = wifi_ap_config,
    };

    strlcpy((char *) wifi_config.ap.ssid, ESP_WIFI_SSID, sizeof(ESP_WIFI_SSID));
    strlcpy((char *) wifi_config.ap.password, ESP_WIFI_PASS, sizeof(ESP_WIFI_PASS));

    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

#if (defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40) || \
     defined(CONFIG_ESP32_CSI_TOOL_C5_5G_BANDWIDTH_HT40)) && \
    (defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
     defined(CONFIG_IDF_TARGET_ESP32C61))
    /* The softAP BSS bandwidth is fixed when the AP starts; a set_bandwidths
     * call after esp_wifi_start() does not retroactively widen the running
     * BSS (observed: STA-side captures showed secondary_channel=0 on every
     * frame). Request HT40 BEFORE start; wifi_band_apply() re-asserts and
     * reads it back afterwards. */
    {
        /* Protocols first: with 802.11ax in the bitmask the driver rejects
         * 40 MHz on these chips (their HE mode is 20 MHz-only), returning
         * ESP_ERR_INVALID_ARG. HT40 is an 11n feature; set the
         * CONFIG-derived protocol mask (disable ENABLE_11AX in menuconfig
         * for HT40 sessions) before requesting the bandwidth. */
        wifi_protocols_t pre_proto = {
            .ghz_2g = ESP32_CSI_TOOL_2G_PROTOCOLS,
            .ghz_5g = ESP32_CSI_TOOL_5G_PROTOCOLS,
        };
        esp_err_t pre_p = esp_wifi_set_protocols(WIFI_IF_AP, &pre_proto);
        wifi_bandwidths_t pre_bw = {};
#  if defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT40)
        pre_bw.ghz_2g = WIFI_BW40;
#  else
        pre_bw.ghz_2g = WIFI_BW20;
#  endif
#  if defined(CONFIG_IDF_TARGET_ESP32C5)
        /* 5 GHz bandwidth from the C5 option; on C6/C61 ghz_5g must stay 0
         * (the driver rejects the whole call otherwise). */
        pre_bw.ghz_5g = ESP32_CSI_TOOL_C5_5G_BW_VALUE;
#  endif
        esp_err_t pre_r = esp_wifi_set_bandwidths(WIFI_IF_AP, &pre_bw);
        ESP_LOGI(TAG,
                 "pre-start AP protocols(2g=%s): %s; bandwidth request "
                 "(2g=%d 5g=%d): %s",
                 ESP32_CSI_TOOL_2G_PROTOCOLS_NAME, esp_err_to_name(pre_p),
                 (int)pre_bw.ghz_2g, (int)pre_bw.ghz_5g,
                 esp_err_to_name(pre_r));
    }
#endif

#if defined(CONFIG_ESP32_CSI_TOOL_2G_BANDWIDTH_HT20) && \
    (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
     defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3))
    /* Pre-HE softAP defaults to HT40; force a 20 MHz BSS so the ESP-NOW
     * HT20 broadcast rate produces genuine 20 MHz frames (an HT20 rate on a
     * 40 MHz BSS was observed to still deliver HT40-layout records). Classic
     * esp_wifi_set_bandwidth API. */
    {
        esp_err_t bw20 = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
        ESP_LOGI(TAG, "pre-start AP HT20 bandwidth request: %s",
                 esp_err_to_name(bw20));
    }
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

    /*
     * Apply ESP32-C5 band selection (2 GHz / 5 GHz / auto) and
     * the v5.4 dual-band protocol/bandwidth API on C5 / C6 / C61. No-op on
     * older targets. See _components/wifi_band_component.h.
     */
    wifi_band_apply(WIFI_IF_AP);

    esp_wifi_set_ps(WIFI_PS_NONE);

#if defined(CONFIG_ESP32_CSI_TOOL_AP_MAX_TX_POWER_QDBM) && \
    CONFIG_ESP32_CSI_TOOL_AP_MAX_TX_POWER_QDBM > 0
    /* Cap TX power for the conducted X1 leg (AP feeds a splitter over cable;
     * full power overloads the cabled receivers). 0.25 dBm units. */
    {
        esp_err_t tpr = esp_wifi_set_max_tx_power(
            CONFIG_ESP32_CSI_TOOL_AP_MAX_TX_POWER_QDBM);
        int8_t got = 0;
        esp_wifi_get_max_tx_power(&got);
        ESP_LOGI(TAG, "max TX power set to %d qdBm (%.2f dBm): %s; readback %d qdBm",
                 CONFIG_ESP32_CSI_TOOL_AP_MAX_TX_POWER_QDBM,
                 CONFIG_ESP32_CSI_TOOL_AP_MAX_TX_POWER_QDBM / 4.0,
                 esp_err_to_name(tpr), (int)got);
    }
#endif

    ESP_LOGI(TAG, "softap_init finished. SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

void config_print() {
    printf("\n\n\n\n\n\n\n\n");
    printf("-----------------------\n");
    printf("VeriCSI-ESP32 Settings\n");
    printf("-----------------------\n");
    printf("PROJECT_NAME: %s\n", "ACTIVE_AP");
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
    printf("ESP_WIFI_SSID: %s\n", ESP_WIFI_SSID);
    printf("ESP_WIFI_PASSWORD: %s\n", ESP_WIFI_PASS);
    printf("SHOULD_COLLECT_CSI: %d\n", SHOULD_COLLECT_CSI);
    printf("SHOULD_COLLECT_ONLY_LLTF: %d\n", SHOULD_COLLECT_ONLY_LLTF);
    printf("SEND_CSI_TO_SERIAL: %d\n", SEND_CSI_TO_SERIAL);
    printf("SEND_CSI_TO_SD: %d\n", SEND_CSI_TO_SD);
#ifdef CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
    printf("AP_MODE: GPTIMER_BROADCAST @ %d Hz\n", CONFIG_ESP32_CSI_TOOL_BROADCAST_RATE_HZ);
#else
    printf("AP_MODE: LEGACY (passive SoftAP, packet rate driven by STA polling)\n");
#endif
    printf("-----------------------\n");
    printf("\n\n\n\n\n\n\n\n");
}

extern "C" void app_main() {
    config_print();
    nvs_init();
    sd_init();
    softap_init();

#if !(SHOULD_COLLECT_CSI)
    printf("CSI will not be collected. Check `idf.py menuconfig  # > VeriCSI-ESP32 Config` to enable CSI");
#endif

    csi_init((char *) "AP");

#ifdef CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST
    /*
     * Application-layer timer-driven broadcast: deterministic packet rate
     * for frequency-domain WiFi sensing. See docs/packet_rate_guide.md.
     *
     * The period is configured in microseconds because whole-Hz resolution is
     * too coarse for the experiments this feature exists for: the beacon
     * interval is 102400 us = 9.765625 Hz, and sweeping the sounding rate
     * across it needs sub-Hz steps. Setting BROADCAST_PERIOD_US to 0 keeps the
     * legacy behaviour of deriving the period from BROADCAST_RATE_HZ.
     */
#  if CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US > 0
    gptimer_broadcast_init(CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US);
#  else
    gptimer_broadcast_init_hz(CONFIG_ESP32_CSI_TOOL_BROADCAST_RATE_HZ);
#  endif
#endif
}
