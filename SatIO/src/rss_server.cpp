#include "rss_server.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include <nvs_flash.h>
#include <string.h>
#include <esp_hosted.h>

#define TAG "RSS_SERVER"
#define MAX_RSS_SIZE 4096

// Replace with your Wi-Fi credentials
#define WIFI_SSID "foo"
#define WIFI_PASS "bar"

static char rss_buffer[MAX_RSS_SIZE];
static size_t rss_length = 0;

static esp_err_t rss_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/rss+xml");
    httpd_resp_send(req, rss_buffer, rss_length);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 4;  // Minimal

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri = {
            .uri      = "/rss.xml",
            .method   = HTTP_GET,
            .handler  = rss_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri);
        ESP_LOGI(TAG, "HTTP server started");
    }
    return server;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retry Wi-Fi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

esp_err_t rss_server_init(void) {
    // Default RSS (same as before)
    rss_server_update("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                      "<rss version=\"2.0\">\n"
                      "  <channel>\n"
                      "    <title>ESP32-P4-NANO RSS Feed</title>\n"
                      "    <description>Hosted via onboard ESP32-C6</description>\n"
                      "    <item><title>Hello</title><description>From Waveshare P4-NANO!</description></item>\n"
                      "  </channel>\n"
                      "</rss>");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_hosted_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    esp_netif_create_default_wifi_sta();

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete (via C6 coprocessor)");
    return ESP_OK;
}

esp_err_t rss_server_update(const char* new_content) {
    // size_t len = strlen(new_content);
    // if (len >= MAX_RSS_SIZE) {
    //     ESP_LOGE(TAG, "RSS content too large");
    //     return ESP_FAIL;
    // }
    // memcpy(rss_buffer, new_content, len + 1);
    // rss_length = len;
    // ESP_LOGI(TAG, "RSS updated (%zu bytes)", rss_length);
    return ESP_OK;
}