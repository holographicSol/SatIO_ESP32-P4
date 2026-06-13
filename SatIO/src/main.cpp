// /** -------------------------------------------------------------------------------------
//  * This is an LVGL learning project.
//  * Bespoke LVGL objects are also being bootstrapped.
//  */

#include "bsp/esp32_p4_wifi6_touch_lcd_xc.h"
#include <limits.h>
#include "esp_log.h"
#include "lvgl.h"
// #include "./strval.h"
#include <stdlib.h>       // malloc/free
#include "ff.h"           // FatFs core
#include "diskio.h"       // Disk I/O
#include "diskio_impl.h"  // ESP32 disk impl
#include "esp_vfs_fat.h"  // VFS integration
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include <stdio.h>
#include <dirent.h>
#include "bsp/esp-bsp.h"

#include <stdio.h>
#include <limits.h>
#include <string.h>
// #include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <Arduino.h>
// #include <Wire.h>

#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdcard_helper.h"
#include "./task_handler.h"
// #include "./multi_display_controller.h"
#include "./i2c_helper.h"
#include "./wit_c_sdk.h"
#include "./satio_lvgl.h"
#include "./astroclock.h"

#include <sys/time.h>
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_uart.h"

#define UART0_NUM      UART_NUM_0
#define UART0_BUF_SIZE         (512)
#define UART0_RD_BUF_SIZE      (UART0_BUF_SIZE)

static const char *UART0_TAG = "UART0_EVENTS";
static QueueHandle_t uart0_queue;

// Line buffer for accumulating partial commands
static char uart0_line_buffer[UART0_BUF_SIZE];
static int uart0_line_pos = 0;

static void uart0_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *) malloc(UART0_RD_BUF_SIZE);
    int len = 0;
    uart_flush_input(UART0_NUM);
    
    for (;;) {
        // Rx
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY)) {
            bzero(dtmp, UART0_RD_BUF_SIZE);
            
            switch (event.type) {
                case UART_DATA: {
                    len = uart_read_bytes(UART0_NUM, dtmp, event.size, 1000 / portTICK_PERIOD_MS);
                    
                    // Accumulate data into line buffer, process on newline
                    for (int i = 0; i < len; i++) {
                        char c = (char)dtmp[i];
                        
                        // Check for line terminator
                        if (c == '\n' || c == '\r') {
                            if (uart0_line_pos > 0) {
                                // Complete line received - process it
                                uart0_line_buffer[uart0_line_pos] = '\0';
                                
                                // Copy to serial0Data.BUFFER for CmdProcess
                                memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
                                strncpy(serial0Data.BUFFER, uart0_line_buffer, sizeof(serial0Data.BUFFER) - 1);
                                
                                printf("Received data: %s\n", serial0Data.BUFFER);
                                CmdProcess();
                                
                                // Reset line buffer
                                uart0_line_pos = 0;
                            }
                            // Skip empty lines (consecutive \r\n)
                        } else {
                            // Accumulate character (with overflow protection)
                            if (uart0_line_pos < UART0_BUF_SIZE - 1) {
                                uart0_line_buffer[uart0_line_pos++] = c;
                            } else {
                                // Buffer overflow - discard and reset
                                ESP_LOGW(UART0_TAG, "Line buffer overflow, discarding");
                                uart0_line_pos = 0;
                            }
                        }
                    }
                    break;
                }
                case UART_FIFO_OVF: {
                    ESP_LOGW(UART0_TAG, "UART FIFO overflow");
                    uart_flush_input(UART0_NUM);
                    xQueueReset(uart0_queue);
                    uart0_line_pos = 0;  // Reset line buffer
                    break;
                }
                case UART_BUFFER_FULL: {
                    ESP_LOGW(UART0_TAG, "UART buffer full");
                    uart_flush_input(UART0_NUM);
                    xQueueReset(uart0_queue);
                    uart0_line_pos = 0;  // Reset line buffer
                    break;
                }
                case UART_BREAK:
                    ESP_LOGI(UART0_TAG, "UART RX break");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(UART0_TAG, "UART parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(UART0_TAG, "UART frame error");
                    break;
                default:
                    ESP_LOGI(UART0_TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

void setup() {

}

// /** ----------------------------------------------------------------------------------------
//  * @brief Interval breach (System counters).
//  */
// int64_t prev_tv_sec;

// void intervalBreach1Second(void) {
//   // if (systemData.interval_breach_1_second) {
//     // store system time
//     storeLocalTime();
//     // store rtc time
//     storeRTCTime();
//     // set loop counter
//     systemData.total_loops_a_second = systemData.loops_a_second;
//     systemData.loops_a_second = 0;
//     // set gps counters
//     systemData.total_gps = systemData.i_count_read_gps;
//     systemData.i_count_read_gps = 0;
//     // set ins counters
//     systemData.total_ins = systemData.i_count_read_ins;
//     systemData.i_count_read_ins = 0;
//     // set gyro counters
//     systemData.total_gyro_0 = systemData.i_count_read_gyro_0;
//     systemData.i_count_read_gyro_0 = 0;
//     // set mplex counters
//     systemData.total_mplex_0 = systemData.i_count_read_mplex_0;
//     systemData.i_count_read_mplex_0 = 0;
//     // set mplex counters
//     systemData.total_matrix = systemData.i_count_matrix;
//     systemData.i_count_matrix = 0;
//     // set mplex counters
//     systemData.total_portcontroller_output = systemData.i_count_port_controller_output;
//     systemData.i_count_port_controller_output = 0;
//     // set mplex counters
//     systemData.total_universe = systemData.i_count_track_planets;
//     systemData.i_count_track_planets = 0;
//     // set mplex counters
//     systemData.total_infocmd = systemData.i_count_read_serial_commands;
//     systemData.i_count_read_serial_commands = 0;
//     // set portcontroller input counters
//     systemData.total_portcontroller_input = systemData.i_count_portcontroller_input;
//     systemData.i_count_portcontroller_input = 0;
//     // set display counters
//     systemData.total_display = systemData.i_count_display;
//     systemData.i_count_display = 0;
//     // set second flags
//     systemData.interval_breach_track_planets = 1;
//     // set uptime
//     systemData.uptime_seconds++;
//     if (systemData.uptime_seconds >= LONG_MAX - 2)
//       {systemData.uptime_seconds = 0;
//         printf("[reset uptime_seconds] %ld\n", systemData.uptime_seconds);
//       }
//   // }
// }

/** -----------------------------------------------------------------------------------------------
 * @brief Main application entry point.
 */
extern "C" void app_main(void)
{
    // --------------------------------------------------------------
    // Warmup delay: some devices require at least one second start.
    // --------------------------------------------------------------
    delay(1000);

    // --------------------------------------------------------------
    // WDT Configuration
    // --------------------------------------------------------------
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5 * 1000,        // 5 seconds
        .idle_core_mask = 0,            // Don't watch idle tasks
        .trigger_panic = false,          // Trigger panic on timeout
    };
    esp_task_wdt_reconfigure(&wdt_config);
    
    // esp_task_wdt_deinit();

    // --------------------------------------------------------------
    // LVGL Initialization
    // --------------------------------------------------------------
    initSatIOUI();

    // --------------------------------------------------------------
    // Initialize Serial 0.
    // --------------------------------------------------------------
    // .baud_rate = 921600,
    uart_config_t uart0_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(UART0_NUM, &uart0_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART0_NUM, UART0_BUF_SIZE * 2, UART0_BUF_SIZE, 20, &uart0_queue, 0));
    // Start event task
    xTaskCreate(uart0_event_task, "uart0_event_task", 3072, NULL, 12, NULL);
    printf("UART0 ready - send data to GPIO1 (RX0)\n");
    
    // --------------------------------------------------------------
    // Initialize I2C BUS 0: RTC
    // --------------------------------------------------------------
    printf("[IIC] intitializing RTC");
    iic_0.setPins(IIC_BUS0_SDA, IIC_BUS0_SCL);
    iic_0.setBufferSize(MAX_IIC_BUFFER_SIZE);
    iic_0.setTimeOut(I2C_TIMEOUT_MS_BUS0);
    iic_0.begin(IIC_BUS0_SDA, IIC_BUS0_SCL, I2C_CLOCK_Hz_BUS0);
    rtc.begin(&iic_0);
    delay(200);
    initSystemTime();

    // -------------------------------------------------------------
    // Initialize I2C BUS 1: Output port controller
    // --------------------------------------------------------------
    printf("[IIC] intitializing output port controller");
    iic_2.setPins(IIC_BUS2_SDA, IIC_BUS2_SCL);
    iic_2.setBufferSize(MAX_IIC_BUFFER_SIZE);
    iic_2.setTimeOut(I2C_TIMEOUT_MS_BUS2);
    iic_2.begin(IIC_BUS2_SDA, IIC_BUS2_SCL, I2C_CLOCK_Hz_BUS2);
    iic_2.setClock(I2C_CLOCK_Hz_BUS2);
    writeOutputPortControllerClear(iic_2, I2C_ADDR_OUTPUT_PORTCONTROLLER);

    // --------------------------------------------------------------
    // Initialize Serial 1 (for GPS).
    // --------------------------------------------------------------
    printf("[Serial1] (GPS)\n");
    int8_t serial_1_rxd = 34;
    int8_t serial_1_txd = -1;
    Serial1.setPins(serial_1_rxd, serial_1_txd, -1, -1);
    Serial1.setRxBufferSize(2000);
    Serial1.setTimeout(10);
    Serial1.begin(115200); while(!Serial1);
    Serial1.flush();
    printf("[Serial1] Baud rate: 115200\n");
    printf("[Serial1] (hardware serial remap: Rx=%d Tx=%d)\n", serial_1_rxd, serial_1_txd);
    
    // --------------------------------------------------------------
    // Create Tasks.
    // --------------------------------------------------------------

    // Storage
    sdcardFlagData.load_system=true; // Set load system flag ready for sdcard flag handler
    printf("creating storage task\n");
    createTaskStorage();             // (target: 2/ps)    SD card
    printf("creating logging task\n");
    createTaskLogging();             // (target: n/ps)    Log to sdcard
    
    // GPS
    printf("creating GPS task\n");
    createTaskGPS();                 // (target: 10/ps)   Time & location
    
    // Gyro
    initWT901();
    printf("creating gyro task\n");
    createTaskGyro();                // (target: 200/ps)  Attitude

    // Auxillary Fast Input
    // Force non-lazy ADC init on signal pin
    analogRead(PIN_ANALOG_DIGITAL_MULTIPLEXER_0_SIG);
    analogSetAttenuation(ADC_11db);  // Full ~0-3.3V range; call once globally
    initADMultiplexer(ad_mux_0);
    createTaskMultiplexers();        // (target: 200/ps)  Fast general input
    
    // Auxillary Large Input (Slow if utilizing all (+-50) input pins)
    // createTaskPortControllerInput(); // (target: 1/ps)    Slow general input

    // Auxillary Output
    printf("creating auxillary output task\n");
    createTaskSwitches();            // (target: approx. max 1000/ps) Fast general output

    // Universe
    printf("creating universe task\n");
    myAstroBegin();
    createTaskUniverse();            // (target: 1/ps)    Star tracking

    // Info/Command
    printf("creating info command task\n");
    createTaskSerialInfoCMD();

    // Attempt to approximately syncronize tasks
    printf("attempting to syncronize tasks\n");
    syncTasks();

    printf("waiting a moment...\n");
    delay(5000); // current

    // --------------------------------------------------------------
    // Display Home screen
    // --------------------------------------------------------------
    printf("starting SatIO UI\n");
    // initSatIOUI(); // already initialized for loading screen
    flag_display_home_screen=true;
    satio_ui_begin();

    // --------------------------------------------------------------
    // Main Loop
    // --------------------------------------------------------------
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}
