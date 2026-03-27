/**
 * @file main.c
 * @brief RuView CSI Node — Universal Multi-Board Firmware
 *
 * Supported boards (select via sdkconfig.defaults.<board>):
 *   - ESP32-WROOM-32 / 32D / 32E    (4MB, no PSRAM)
 *   - ESP32-WROVER / WROVER-B        (4MB/8MB + 4MB/8MB PSRAM)
 *   - ESP32-DevKitC v4               (4MB, no PSRAM)
 *   - NodeMCU-32S / AZ-Delivery      (4MB, no PSRAM)
 *   - LOLIN D1 Mini ESP32            (4MB, no PSRAM)
 *   - ESP32-SOLO-1                   (4MB, no PSRAM, single core)
 *   - ESP32-PICO-D4 / PICO-KIT       (4MB embedded flash, no PSRAM)
 *   - ESP32-S3-DevKitC               (8MB + PSRAM optional, original target)
 *
 * All boards share the same core CSI sensing pipeline.
 * Board-specific differences handled via sdkconfig.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "csi_collector.h"
#include "stream_sender.h"
#include "nvs_config.h"
#include "edge_processing.h"
#include "ota_update.h"
#include "power_mgmt.h"
#include "wasm_runtime.h"
#include "wasm_upload.h"
#include "mmwave_sensor.h"

/* Display: only on boards that physically have it (S3 AMOLED boards) */
#ifdef CONFIG_DISPLAY_ENABLE
#include "display_task.h"
#endif

#ifdef CONFIG_CSI_MOCK_ENABLED
#include "mock_csi.h"
#endif

#include "esp_timer.h"

static const char *TAG = "main";

static esp_timer_handle_t s_wasm_timer;
nvs_config_t g_nvs_config;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define MAX_RETRY 10

/* ---- Board identification at boot ---- */
static void log_board_info(void)
{
    ESP_LOGI(TAG, "========================================");
#if defined(CONFIG_BOARD_ESP32_WROOM)
    ESP_LOGI(TAG, " Board : ESP32-WROOM-32");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_ESP32_WROVER)
    ESP_LOGI(TAG, " Board : ESP32-WROVER");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: 4MB/8MB");
#elif defined(CONFIG_BOARD_ESP32_DEVKITC)
    ESP_LOGI(TAG, " Board : ESP32-DevKitC v4");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_NODEMCU32S)
    ESP_LOGI(TAG, " Board : NodeMCU-32S");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_D1MINI_ESP32)
    ESP_LOGI(TAG, " Board : LOLIN D1 Mini ESP32");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_ESP32_SOLO)
    ESP_LOGI(TAG, " Board : ESP32-SOLO-1 (single core)");
    ESP_LOGI(TAG, " Flash : 4MB | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_ESP32_PICO)
    ESP_LOGI(TAG, " Board : ESP32-PICO-D4 / PICO-KIT");
    ESP_LOGI(TAG, " Flash : 4MB embedded | RAM: 520KB | PSRAM: None");
#elif defined(CONFIG_BOARD_ESP32S3_DEVKITC)
    ESP_LOGI(TAG, " Board : ESP32-S3-DevKitC (original target)");
    ESP_LOGI(TAG, " Flash : 8MB | RAM: 512KB + 8MB PSRAM");
#else
    ESP_LOGI(TAG, " Board : Generic ESP32 (auto-detect)");
#endif
    ESP_LOGI(TAG, " Firmware: RuView CSI Node v0.5.0-multiboard");
    ESP_LOGI(TAG, "========================================");
}

/* ---- WiFi ---- */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "WiFi retry %d/%d", s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    strncpy((char *)wifi_config.sta.ssid, g_nvs_config.wifi_ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_nvs_config.wifi_password,
            sizeof(wifi_config.sta.password) - 1);
    if (strlen((char *)wifi_config.sta.password) == 0)
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", g_nvs_config.wifi_ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
        ESP_LOGI(TAG, "WiFi connected");
    else
        ESP_LOGE(TAG, "WiFi connection failed after %d retries", MAX_RETRY);
}

/* ---- Main ---- */
void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_config_load(&g_nvs_config);
    log_board_info();
    ESP_LOGI(TAG, "Node ID: %d", g_nvs_config.node_id);

    /* WiFi */
#ifndef CONFIG_CSI_MOCK_SKIP_WIFI_CONNECT
    wifi_init_sta();
#else
    ESP_LOGI(TAG, "Mock mode: skipping WiFi");
#endif

    /* UDP sender */
#ifndef CONFIG_CSI_MOCK_SKIP_WIFI_CONNECT
    if (stream_sender_init_with(g_nvs_config.target_ip, g_nvs_config.target_port) != 0) {
        ESP_LOGE(TAG, "UDP sender init failed");
        return;
    }
#endif

    /* CSI collection */
#ifdef CONFIG_CSI_MOCK_ENABLED
    esp_err_t mock_ret = mock_csi_init(CONFIG_CSI_MOCK_SCENARIO);
    if (mock_ret == ESP_OK)
        ESP_LOGI(TAG, "Mock CSI active (scenario=%d)", CONFIG_CSI_MOCK_SCENARIO);
#else
    csi_collector_init();
#endif

    /* Edge processing */
    edge_config_t edge_cfg = {
        .tier              = g_nvs_config.edge_tier,
        .presence_thresh   = g_nvs_config.presence_thresh,
        .fall_thresh       = g_nvs_config.fall_thresh,
        .vital_window      = g_nvs_config.vital_window,
        .vital_interval_ms = g_nvs_config.vital_interval_ms,
        .top_k_count       = g_nvs_config.top_k_count,
        .power_duty        = g_nvs_config.power_duty,
    };
    esp_err_t edge_ret = edge_processing_init(&edge_cfg);
    if (edge_ret != ESP_OK)
        ESP_LOGW(TAG, "Edge init: %s (continuing)", esp_err_to_name(edge_ret));

    /* OTA */
    httpd_handle_t ota_server = NULL;
#ifndef CONFIG_CSI_MOCK_SKIP_WIFI_CONNECT
    esp_err_t ota_ret = ota_update_init_ex(&ota_server);
    if (ota_ret != ESP_OK)
        ESP_LOGW(TAG, "OTA: %s", esp_err_to_name(ota_ret));
#else
    esp_err_t ota_ret = ESP_ERR_NOT_SUPPORTED;
#endif

    /* WASM runtime (PSRAM if available, else internal heap — see wasm_runtime.c) */
    esp_err_t wasm_ret = wasm_runtime_init();
    if (wasm_ret != ESP_OK) {
        ESP_LOGW(TAG, "WASM: %s", esp_err_to_name(wasm_ret));
    } else {
        if (ota_server != NULL)
            wasm_upload_register(ota_server);
        esp_timer_create_args_t ta = {
            .callback = (void (*)(void *))wasm_runtime_on_timer,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wasm_timer",
        };
        if (esp_timer_create(&ta, &s_wasm_timer) == ESP_OK) {
#ifdef CONFIG_WASM_TIMER_INTERVAL_MS
            uint64_t us = (uint64_t)CONFIG_WASM_TIMER_INTERVAL_MS * 1000ULL;
#else
            uint64_t us = 1000000ULL;
#endif
            esp_timer_start_periodic(s_wasm_timer, us);
        }
    }

    /* mmWave sensor — board-specific GPIO pins via sdkconfig */
#ifdef CONFIG_MMWAVE_TX_PIN
    esp_err_t mmwave_ret = mmwave_sensor_init(CONFIG_MMWAVE_TX_PIN, CONFIG_MMWAVE_RX_PIN);
#else
    esp_err_t mmwave_ret = mmwave_sensor_init(-1, -1);  /* -1 = use defaults */
#endif
    if (mmwave_ret == ESP_OK) {
        mmwave_state_t mw;
        if (mmwave_sensor_get_state(&mw))
            ESP_LOGI(TAG, "mmWave: %s (caps=0x%04x)", mmwave_type_name(mw.type), mw.capabilities);
    } else {
        ESP_LOGI(TAG, "No mmWave sensor detected");
    }

    /* Power management */
    power_mgmt_init(g_nvs_config.power_duty);

    /* Display (only on boards with AMOLED hardware) */
#ifdef CONFIG_DISPLAY_ENABLE
    esp_err_t disp_ret = display_task_start();
    if (disp_ret != ESP_OK)
        ESP_LOGW(TAG, "Display: %s", esp_err_to_name(disp_ret));
#endif

    ESP_LOGI(TAG, "CSI streaming → %s:%d | edge=%u OTA=%s WASM=%s mmWave=%s",
             g_nvs_config.target_ip, g_nvs_config.target_port,
             g_nvs_config.edge_tier,
             (ota_ret    == ESP_OK) ? "on" : "off",
             (wasm_ret   == ESP_OK) ? "on" : "off",
             (mmwave_ret == ESP_OK) ? "on" : "off");

    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}
