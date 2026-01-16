#ifndef RSS_SERVER_H
#define RSS_SERVER_H

#include <esp_err.h>

/**
 * Initialize WiFi and start the RSS HTTP server.
 * Call this in app_main() after nvs_flash_init().
 */
esp_err_t rss_server_init(void);

/**
 * Update the RSS content with new XML string.
 * @param new_content Null-terminated string with full RSS XML.
 * @return ESP_OK on success, ESP_FAIL if too large.
 */
esp_err_t rss_server_update(const char* new_content);

#endif // RSS_SERVER_H