/*
    ESP32 Helper Library. Written by Benjamin Jack Cullen.

    Provides some ESP32 specific functionality.
*/

#include "esp32_helper.h"
#include <Arduino.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <inttypes.h>
#include <stdint.h>

// ----------------------------------------------------------------------------------------
// Get Partition Type Name.
// ----------------------------------------------------------------------------------------
const char* getPartitionTypeName(int type) {
    switch (type) {
        case ESP_PARTITION_TYPE_APP:             return "ESP_PARTITION_TYPE_APP";
        case ESP_PARTITION_TYPE_DATA:            return "ESP_PARTITION_TYPE_DATA";
        case ESP_PARTITION_TYPE_BOOTLOADER:      return "ESP_PARTITION_TYPE_BOOTLOADER";
        case ESP_PARTITION_TYPE_PARTITION_TABLE: return "ESP_PARTITION_SUBTYPE_PARTITION_TABLE";
        case ESP_PARTITION_TYPE_ANY:             return "ESP_PARTITION_TYPE_ANY";
        default:                                 return "undefined";
    }
}

// ----------------------------------------------------------------------------------------
// Get Partition Subtype Name.
// ----------------------------------------------------------------------------------------
const char* getPartitionSubTypeName(int type, int subtype) {
    if (type == ESP_PARTITION_TYPE_APP) {
        switch (subtype) {
            case ESP_PARTITION_SUBTYPE_APP_FACTORY: return "ESP_PARTITION_SUBTYPE_APP_FACTORY";
            case ESP_PARTITION_SUBTYPE_APP_OTA_MIN: return "ESP_PARTITION_SUBTYPE_APP_OTA_MIN";
            case ESP_PARTITION_SUBTYPE_APP_OTA_1:   return "ESP_PARTITION_SUBTYPE_APP_OTA_1";
            case ESP_PARTITION_SUBTYPE_APP_OTA_2:   return "ESP_PARTITION_SUBTYPE_APP_OTA_2";
            case ESP_PARTITION_SUBTYPE_APP_OTA_3:   return "ESP_PARTITION_SUBTYPE_APP_OTA_3";
            case ESP_PARTITION_SUBTYPE_APP_OTA_4:   return "ESP_PARTITION_SUBTYPE_APP_OTA_4";
            case ESP_PARTITION_SUBTYPE_APP_OTA_5:   return "ESP_PARTITION_SUBTYPE_APP_OTA_5";
            case ESP_PARTITION_SUBTYPE_APP_OTA_6:   return "ESP_PARTITION_SUBTYPE_APP_OTA_6";
            case ESP_PARTITION_SUBTYPE_APP_OTA_7:   return "ESP_PARTITION_SUBTYPE_APP_OTA_7";
            case ESP_PARTITION_SUBTYPE_APP_OTA_8:   return "ESP_PARTITION_SUBTYPE_APP_OTA_8";
            case ESP_PARTITION_SUBTYPE_APP_OTA_9:   return "ESP_PARTITION_SUBTYPE_APP_OTA_9";
            case ESP_PARTITION_SUBTYPE_APP_OTA_10:  return "ESP_PARTITION_SUBTYPE_APP_OTA_10";
            case ESP_PARTITION_SUBTYPE_APP_OTA_11:  return "ESP_PARTITION_SUBTYPE_APP_OTA_11";
            case ESP_PARTITION_SUBTYPE_APP_OTA_12:  return "ESP_PARTITION_SUBTYPE_APP_OTA_12";
            case ESP_PARTITION_SUBTYPE_APP_OTA_13:  return "ESP_PARTITION_SUBTYPE_APP_OTA_13";
            case ESP_PARTITION_SUBTYPE_APP_OTA_14:  return "ESP_PARTITION_SUBTYPE_APP_OTA_14";
            case ESP_PARTITION_SUBTYPE_APP_OTA_15:  return "ESP_PARTITION_SUBTYPE_APP_OTA_15";
            case ESP_PARTITION_SUBTYPE_APP_OTA_MAX: return "ESP_PARTITION_SUBTYPE_APP_OTA_MAX";
            case ESP_PARTITION_SUBTYPE_APP_TEE_MIN: return "ESP_PARTITION_SUBTYPE_APP_TEE_MIN";
            case ESP_PARTITION_SUBTYPE_APP_TEE_MAX: return "ESP_PARTITION_SUBTYPE_APP_TEE_MAX";
            default:                                return "undefined";
        }
    } else if (type == ESP_PARTITION_TYPE_DATA) {
        switch (subtype) {
            case ESP_PARTITION_SUBTYPE_DATA_OTA:       return "ESP_PARTITION_SUBTYPE_DATA_OTA";
            case ESP_PARTITION_SUBTYPE_DATA_PHY:       return "ESP_PARTITION_SUBTYPE_DATA_PHY";
            case ESP_PARTITION_SUBTYPE_DATA_NVS:       return "ESP_PARTITION_SUBTYPE_DATA_NVS";
            case ESP_PARTITION_SUBTYPE_DATA_COREDUMP:  return "ESP_PARTITION_SUBTYPE_DATA_COREDUMP";
            case ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS:  return "ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS";
            case ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM:  return "ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM";
            case ESP_PARTITION_SUBTYPE_DATA_UNDEFINED: return "ESP_PARTITION_SUBTYPE_DATA_UNDEFINED";
            case ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD:  return "ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD";
            case ESP_PARTITION_SUBTYPE_DATA_FAT:       return "ESP_PARTITION_SUBTYPE_DATA_FAT";
            case ESP_PARTITION_SUBTYPE_DATA_SPIFFS:    return "ESP_PARTITION_SUBTYPE_DATA_SPIFFS";
            case ESP_PARTITION_SUBTYPE_DATA_LITTLEFS:  return "ESP_PARTITION_SUBTYPE_DATA_LITTLEFS";
            case ESP_PARTITION_SUBTYPE_DATA_TEE_OTA:   return "ESP_PARTITION_SUBTYPE_DATA_TEE_OTA";
            case ESP_PARTITION_SUBTYPE_ANY:            return "ESP_PARTITION_SUBTYPE_ANY";
            default:                                   return "undefined";
        }
    } else if (type == ESP_PARTITION_TYPE_BOOTLOADER) {
        switch (subtype) {
            case ESP_PARTITION_SUBTYPE_BOOTLOADER_PRIMARY:  return "ESP_PARTITION_SUBTYPE_BOOTLOADER_PRIMARY";
            case ESP_PARTITION_SUBTYPE_BOOTLOADER_OTA:      return "ESP_PARTITION_SUBTYPE_BOOTLOADER_OTA";
            case ESP_PARTITION_SUBTYPE_BOOTLOADER_RECOVERY: return "ESP_PARTITION_SUBTYPE_BOOTLOADER_RECOVERY";
            default:                                        return "undefined";
        }
    } else if (type == ESP_PARTITION_TYPE_PARTITION_TABLE) {
        switch (subtype) {
            case ESP_PARTITION_SUBTYPE_PARTITION_TABLE_PRIMARY: return "ESP_PARTITION_SUBTYPE_PARTITION_TABLE_PRIMARY";
            case ESP_PARTITION_SUBTYPE_PARTITION_TABLE_OTA:     return "ESP_PARTITION_SUBTYPE_PARTITION_TABLE_OTA";
            default:                                            return "undefined";
        }
    } else if (type == ESP_PARTITION_TYPE_ANY) {
        return "ESP_PARTITION_SUBTYPE_ANY";
    }
    return "undefined";
}

// ----------------------------------------------------------------------------------------
// Print Partition Table.
// ----------------------------------------------------------------------------------------
void print_partition_table(void) {
    char type_buf[100];
    char subtype_buf[100];
    esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (iterator != NULL) {
        const esp_partition_t *partition = esp_partition_get(iterator);
        sprintf(type_buf, "%d (%s)", partition->type, getPartitionTypeName(partition->type));
        sprintf(subtype_buf, "%d (%s)", partition->subtype, getPartitionSubTypeName(partition->type, partition->subtype));
        printf("Partition label: '%-10s', type: %-19s, subtype: %-19s, address: 0x%08" PRIx32 ", size: 0x%08" PRIx32 " (%u bytes), encrypted: %-19d\n",
               partition->label ? partition->label : "unnamed",
               type_buf,
               subtype_buf,
               partition->address,
               partition->size,
               partition->size,
               partition->encrypted);
        iterator = esp_partition_next(iterator);
    }
    esp_partition_iterator_release(iterator);
}

// ----------------------------------------------------------------------------------------
// Print RAM Information.
// ----------------------------------------------------------------------------------------
void print_ram_info(void) {
    Serial.println("RAM Information:");
    printf("Free heap size: %u bytes\n", esp_get_free_heap_size());
    printf("Minimum free heap size ever: %u bytes\n", esp_get_minimum_free_heap_size());
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);
    printf("DRAM - Total: %u, Free: %u, Largest block: %u\n",
           heap_info.total_allocated_bytes + heap_info.total_free_bytes,
           heap_info.total_free_bytes,
           heap_info.largest_free_block);
}