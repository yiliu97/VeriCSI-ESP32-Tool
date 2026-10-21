#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_log.h"
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

/*
 * Active STA - connects to an Active AP (or any access point) and collects
 * CSI from the data frames it receives.
 *
 * Differences from the ESP32-CSI-Tool lineage (Hernandez 2020):
 *   - esp_spi_flash.h removed (gone in IDF v5+).
 *   - esp_netif_init() now called explicitly (was implicit in v4.3).
 *   - esp_wifi_init() return value tolerated rather than asserted.
 */

/*
 * The examples use WiFi configuration that you can set via 'idf.py menuconfig'.
 *
 * If you'd rather not, just change the below entries to strings with
 * the config you want - ie #define ESP_WIFI_SSID "mywifissid"
 */
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD

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

/*
 * Core affinity for the uplink transmitter task.
 *
 * xTaskCreatePinnedToCore asserts unless 0 <= xCoreID < configNUM_CORES, so a
 * literal core number is only valid on the dual-core Xtensa parts. Every
 * unicore target (ESP32-S2 / C3 / C5 / C6 / C61) needs tskNO_AFFINITY. Mirrors
 * CSI_CONSUMER_CORE in _components/csi_component.h and GPTIMER_BCAST_CORE in
 * _components/gptimer_broadcast.h.
 */
#if CONFIG_FREERTOS_UNICORE
#  define STA_TX_TASK_CORE  tskNO_AFFINITY
#else
#  define STA_TX_TASK_CORE  1
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

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "Active CSI collection (Station)";

esp_err_t _http_event_handle(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (!real_time_set) {
                    char *data = (char *) malloc(evt->data_len + 1);
                    strncpy(data, (char *) evt->data, evt->data_len);
                    data[evt->data_len + 1] = '\0';
                    time_set(data);
                    free(data);
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

//// en_sys_seq: see https://github.com/espressif/esp-idf/blob/master/docs/api-guides/wifi.rst#wi-fi-80211-packet-send for details
extern "C" esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retry connecting to the AP");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        /*
         * Warn if the user enabled HE-LTF SU acquisition
         * but the AP we just associated with does not advertise 11ax.
         * No-op on non-HE chips / when HE-SU is disabled.
         */
        wifi_band_check_ap_acquire_match();
    }
}

bool is_wifi_connected() {
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT);
}

void station_init() {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_wifi_init returned %s; continuing", esp_err_to_name(ret));
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_sta_config_t wifi_sta_config = {};
    wifi_sta_config.channel = WIFI_CHANNEL;
    wifi_config_t wifi_config = {
            .sta = wifi_sta_config,
    };

    strlcpy((char *) wifi_config.sta.ssid, ESP_WIFI_SSID, sizeof(ESP_WIFI_SSID));
    strlcpy((char *) wifi_config.sta.password, ESP_WIFI_PASS, sizeof(ESP_WIFI_PASS));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /*
     * Apply ESP32-C5 band selection (2 GHz / 5 GHz / auto) and
     * the v5.4 dual-band protocol/bandwidth API on C5 / C6 / C61. No-op on
     * older targets. See _components/wifi_band_component.h.
     */
    wifi_band_apply(WIFI_IF_STA);

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

TaskHandle_t xHandle = NULL;

void vTask_socket_transmitter_sta_loop(void *pvParameters) {
    for (;;) {
        socket_transmitter_sta_loop(&is_wifi_connected);
    }
}

void config_print() {
    printf("\n\n\n\n\n\n\n\n");
    printf("-----------------------\n");
    printf("VeriCSI-ESP32 Settings\n");
    printf("-----------------------\n");
    printf("PROJECT_NAME: %s\n", "ACTIVE_STA");
    printf("CONFIG_ESPTOOLPY_MONITOR_BAUD: %d\n", CONFIG_ESPTOOLPY_MONITOR_BAUD);
    // CONFIG_ESP_CONSOLE_UART_BAUDRATE only exists when the console is routed
    // through UART (CONFIG_ESP_CONSOLE_UART_DEFAULT or _CUSTOM). On chips
    // where the console is USB-Serial-JTAG (S3 / C3 / C5 / C6 / C61 default)
    // or USB-CDC, the symbol is undefined and referencing it fails to compile.
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
    /* PACKET_RATE only exists when the uplink task is enabled: its Kconfig
     * entry `depends on ESP32_CSI_TOOL_STA_UPLINK`, so the symbol disappears
     * entirely in pure-receiver builds. */
#ifdef CONFIG_ESP32_CSI_TOOL_STA_UPLINK
    printf("STA_UPLINK: 1\n");
    printf("PACKET_RATE: %i\n", CONFIG_PACKET_RATE);
#else
    printf("STA_UPLINK: 0  (pure receiver; no uplink task)\n");
#endif
    printf("SHOULD_COLLECT_CSI: %d\n", SHOULD_COLLECT_CSI);
    printf("SHOULD_COLLECT_ONLY_LLTF: %d\n", SHOULD_COLLECT_ONLY_LLTF);
    printf("SEND_CSI_TO_SERIAL: %d\n", SEND_CSI_TO_SERIAL);
    printf("SEND_CSI_TO_SD: %d\n", SEND_CSI_TO_SD);
    printf("-----------------------\n");
    printf("\n\n\n\n\n\n\n\n");
}

extern "C" void app_main() {
    config_print();
    nvs_init();
    sd_init();
    station_init();
    csi_init((char *) "STA");

#if !(SHOULD_COLLECT_CSI)
    printf("CSI will not be collected. Check `idf.py menuconfig  # > VeriCSI-ESP32 Config` to enable CSI");
#endif

    // Priority 5 (matches CSI_CONSUMER_PRIORITY in csi_component.h). Hernandez 2020
    // wrote 100 but FreeRTOS configMAX_PRIORITIES = 25 in v6 (range 0-24); v4.3
    // silently clamped, v6 asserts at task creation -> reboot loop. Keep this
    // task moderate so it does not compete with the Wi-Fi task (priority 23) or
    // starve the CSI consumer.
    //
    // Core affinity: this used to be a hardcoded 1, which asserts at boot on
    // every unicore target (C3 / C5 / C6 / C61 / S2) because
    // xTaskCreatePinnedToCore requires 0 <= xCoreID < configNUM_CORES:
    //   assert failed: xTaskCreatePinnedToCore freertos_tasks_c_additions.h:161
    // csi_component.h and gptimer_broadcast.h already guarded this; this call
    // site was missed, so the fault only appeared once a non-ESP32 board was
    // actually powered on.
#ifdef CONFIG_ESP32_CSI_TOOL_STA_UPLINK
    xTaskCreatePinnedToCore(&vTask_socket_transmitter_sta_loop, "socket_transmitter_sta_loop",
                            10000, (void *) &is_wifi_connected, 5, &xHandle,
                            STA_TX_TASK_CORE);
#else
    ESP_LOGI("Active CSI collection (Station)",
             "STA uplink disabled: pure receiver. CSI depends entirely on what "
             "the AP transmits (e.g. the gptimer broadcast).");
#endif
}
