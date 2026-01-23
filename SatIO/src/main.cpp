/*

  SatIO - Written by Benjamin Jack Cullen.

-----

  A general purpose programmable I/O platform for automation, throughput and LLM's.

  A huge matrix switch in a small package, supporting stacks of logic across
  70 output pins and 100 mapping slots.

-----

  Inference in Bayesian Reasoning? Moon tracking for example can be used to track the
  moon, it can also be used for one example; to track the tides, if the system is aware of
  moon/planetary positioning and datetime then marine life values may also be inferred
  relative to the inferred tide values and known datetime. There is a lot of data that
  can be used in many ways, with a kind of network effect.

-----

  Wiring For Keystudio ESP32 PLUS Development Board:

          ESP32: 1st ATMEGA2560 with shield as Output Port Controller (not on multiplexer):
          ESP32: IIC 1 SDA as io4 -> ATMEGA2560: I2C SDA
          ESP32: IIC 1 SCL as io5 -> ATMEGA2560: I2C SCL

          ESP32: 2nd ATMEGA2560 with shield as Input Port Controller (not on multiplexer):
          ESP32: IIC 2 SDA as io7 -> ATMEGA2560: I2C SDA
          ESP32: IIC 2 SCL as io8 -> ATMEGA2560: I2C SCL

          Other ESP32 I2C DS3231 (not on multiplexer) (5v):
          ESP32: IIC 0 SDA as io2 -> DS3231 (RTC): SDA
          ESP32: IIC 0 SCL as io3 -> DS3231 (RTC): SCL

          ESP32 i2C: I2C Multiplexing (3.3v):
          ESP32: IIC 0 SDA as io2 -> TCA9548A: SDA
          ESP32: IIC 0 SCL as io3 -> TCA9548A: SCL

          ESP32: Analog/Digital Multiplexing (3.3v):
          ESP32: io23   -> CD74HC4067: S0
          ESP32: io22   -> CD74HC4067: S1
          ESP32: io21   -> CD74HC4067: S2
          ESP32: io20   -> CD74HC4067: S3
          ESP32: io53   -> CD74HC4067: SIG

          ESP32: WTGPS300P (5v):
          ESP32: io36 RXD -> WTGPS300P: TXD
          ESP32: null TXD -> WTGPS300P: RXD

          ESP32: WT901 9-Axis Gyro:
          ESP32: io33 RXD -> WT901 TXD
          ESP32: io32 TXD -> WT901 RXD

-----

    To Do: AI I2C modules returning int's as classifiers.
    To Do: SRTM data. Use NASA shuttle radar topographical mission data.
    To Do: Ability to add custom IIC sensor modules after flashing.
    To Do: PCB fabrication.

-----

  There are some required custom libs included in complete project files:
  https://drive.google.com/drive/folders/13yynSxkKL-zxb7iLSkg0v0VXkSLgmtW-?usp=sharing

-----
*/

#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>

#include <Arduino.h>
#include <Wire.h>
               
#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include "./esp32_helper.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdmmc_helper.h"
#include "./task_handler.h"
#include "./multi_display_controller.h"
#include "i2c_helper.h"

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
#include "driver/uart.h"
#include "SPIFFS.h"

// #include "lcdgfx.h"             // https://github.com/lexus2k/lcdgfx
// #include "lcdgfx_gui.h"         // https://github.com/lexus2k/lcdgfx

// DisplaySSD1306_128x64_I2C ssd1306_display_0(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});
// NanoCanvas<128,64,1> canvas_128x64_0;

/**
 * @brief Setup
 */
void setup() {
  // --------------------------------------------------------------
  // Required for operations in taks that may take longer than 5s..
  // --------------------------------------------------------------
  esp_task_wdt_config_t config = {
    .timeout_ms = 60* 1000, // 1 minute
    .trigger_panic = true,  // Trigger panic if watchdog timer not reset
  };
  esp_task_wdt_reconfigure(&config);
  enableLoopWDT();
  // --------------------------------------------------------------
  // Warmup delay: some devices require at least one second start.
  // --------------------------------------------------------------
  delay(1000);
  // --------------------------------------------------------------
  // Initialize Serial 0.
  // --------------------------------------------------------------
  Serial.setRxBufferSize(2000); // ensure this is set before begin()
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(921600); while(!Serial);
  Serial.flush();
  Serial.println();
  Serial.println();
  Serial.println("[SERIAL] (Commands and general output)");
  Serial.println("[SERIAL] Baud rate: 921600");
  // --------------------------------------------------------------
  // System Information.
  // --------------------------------------------------------------
  Serial.println("[xPortGetCoreID] " +
    String(xPortGetCoreID()));
  Serial.println("[ESP_PM_CPU_FREQ_MAX] " +
    String(ESP_PM_CPU_FREQ_MAX));
  Serial.println("[ESP_PM_APB_FREQ_MAX] " +
    String(ESP_PM_APB_FREQ_MAX));
  Serial.println("[ESP_PM_NO_LIGHT_SLEEP] " +
    String(ESP_PM_NO_LIGHT_SLEEP));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHFREQ] " +
    String(CONFIG_ESPTOOLPY_FLASHFREQ));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHMODE] " +
    String(CONFIG_ESPTOOLPY_FLASHMODE));
  Serial.println("[CONFIG_LOG_DEFAULT_LEVEL] " +
    String(CONFIG_LOG_DEFAULT_LEVEL));
  Serial.println("[CONFIG_BOOTLOADER_LOG_LEVEL] " +
    String(CONFIG_BOOTLOADER_LOG_LEVEL));
  Serial.println("[CONFIG_ESP_CONSOLE_UART_BAUDRATE] " +
    String(CONFIG_ESP_CONSOLE_UART_BAUDRATE));
  Serial.println("[CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL] " +
    String(CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL));
  Serial.println("[getCpuFrequencyMhz] " +
    String(getCpuFrequencyMhz()));
  Serial.println("[APB_CLK_FREQ] " +
    String(getApbFrequency()));
  
  // --------------------------------------------------------------
  // 7 Segment Display(s).
  // --------------------------------------------------------------
  // setupTM(); // initialize 7-segment display(s)

  // --------------------------------------------------------------
  // Initialize NVS.
  // --------------------------------------------------------------
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ret = nvs_flash_init();}
  if (ret != ESP_OK) {printf("NVS init failed: %s\n", esp_err_to_name(ret));}
  // --------------------------------------------------------------
  // Initialize SPIFFS.
  // --------------------------------------------------------------
  Serial.println("[SPIFFS] attempting to mount");
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED, "/INTERNAL")){
      Serial.println("[SPIFFS] Mount Failed");
   }
  else {Serial.println("[SPIFFS] mounted successfully");}
  print_partition_table();
  print_ram_info();
  // --------------------------------------------------------------
  // Initialize SDCard and attempt to load files.
  // --------------------------------------------------------------
  sdcardBegin();
  sdmmcFlagData.load_system=true;
  sdcardFlagHandler();
  if (matrixData.load_matrix_on_startup) {
    sdmmcFlagData.load_mapping=true; sdcardFlagHandler();
    sdmmcFlagData.load_matrix=true; sdcardFlagHandler();}
  
  i2c_bus0_mutex = xSemaphoreCreateBinary();
  xSemaphoreGive(i2c_bus0_mutex);  // Make available

  // --------------------------------------------------------------
  // Initialize I2C BUS 0: RTC
  // --------------------------------------------------------------
  Serial.println("[IIC] intitializing RTC");
  iic_0.setPins(IIC_BUS0_SDA, IIC_BUS0_SCL);
  iic_0.setBufferSize(256);
  iic_0.setTimeOut(1000);
  iic_0.begin(IIC_BUS0_SDA, IIC_BUS0_SCL, 200000UL);
  rtc.begin(&iic_0);
  delay(200);

  // --------------------------------------------------------------
  // Initialize I2C BUS 0: Display
  // --------------------------------------------------------------
  // Serial.println("[IIC] intitializing multiplexer");
  // for (int i=0; i<3; i++) {
  //   setI2CMultiplexChannel(Wire, i2c_mux_0, i);
  //   delay(200);
  //   Serial.println("[IIC] intitializing display(s)");
  //   ssd1306_display_0.begin();
  //   ssd1306_display_0.clear();
  //   // test canvas
  //   canvas_128x64_0.setFixedFont(ssd1306xled_font6x8);
  //   canvas_128x64_0.clear();
  //   canvas_128x64_0.printFixed(1, 1, "SatIO", STYLE_BOLD);
  //   ssd1306_display_0.drawCanvas(0, 0, canvas_128x64_0);
  // }

  // --------------------------------------------------------------
  // Initialize I2C BUS 1: Output port controller
  // --------------------------------------------------------------
  Serial.println("[IIC] intitializing output port controller");
  iic_1.setPins(IIC_BUS1_SDA, IIC_BUS1_SCL);
  iic_1.setBufferSize(256);
  iic_1.setTimeOut(1000);
  iic_1.begin(IIC_BUS1_SDA, IIC_BUS1_SCL);
  // iic_1.setClock(400000); // ATMEGA2560 no resistors     good
  // iic_1.setClock(200000); // ESP32 no resistors          good
  // iic_1.setClock(200000); // ESP32 2.2k resistor         good
  // iic_1.setClock(250000); // ATMEGA2560 2.2k resistor    good
  iic_1.setClock(800000); // ATMEGA2560 2.2k resistor       good (+-4ns rise time)
  writeOutputPortControllerClear(iic_1, I2C_ADDR_OUTPUT_PORTCONTROLLER);

  // --------------------------------------------------------------
  // Initialize I2C BUS 2: Input port controller
  // --------------------------------------------------------------
  Serial.println("[IIC] intitializing input port controller");
  iic_2.setPins(IIC_BUS2_SDA, IIC_BUS2_SCL);
  iic_2.setBufferSize(256);
  iic_2.setTimeOut(1000);
  iic_2.begin(IIC_BUS2_SDA, IIC_BUS2_SCL);
  iic_2.setClock(400000);

  // --------------------------------------------------------------
  // Initialize Multiplexer(s).
  // --------------------------------------------------------------
  initADMultiplexer(ad_mux_0);

  // --------------------------------------------------------------
  // Initialize System Time (for local time).
  // --------------------------------------------------------------
  initSystemTime();
  // --------------------------------------------------------------
  // Initialize Serial 1 (for GPS).
  // --------------------------------------------------------------
  Serial.println("[SERIAL1] (GPS)");
  Serial1.setPins(36, -1, -1, -1); // ensure this is set before begin()
  Serial1.setRxBufferSize(2000); // ensure this is set before begin()
  Serial1.setTimeout(10); // ensure this is set before begin()
  Serial1.begin(115200); while(!Serial1);
  Serial1.flush();
  Serial.println("[SERIAL1] Baud rate: 115200");
  Serial.println("[SERIAL1] (hardware serial remap: Rx=36 Tx=-1)");
  
  // --------------------------------------------------------------
  // Initialize Serial 2 (for 9-Axis Gyro).
  // --------------------------------------------------------------
  Serial.println("[SERIAL2] (9-Axis Gyro)");
  Serial2.setPins(33, 32, -1, -1); // ensure this is set before begin()
  Serial2.setRxBufferSize(2000); // ensure this is set before begin()
  Serial2.setTimeout(1000); // ensure this is set before begin()
  Serial2.flush();
  Serial.println("[SERIAL2] (hardware serial remap: Rx=33 Tx=32)");
	Serial.println("[SERIAL2] initializing Gyro0");
  initWT901();
  delay(1000);

  // ------------------------------------------------------------
  // Interrupts
  // ------------------------------------------------------------
  // requires HIGH/LOW inversion from interrupting device
  // pinMode(ISR_PIN_MULTIDISPLAY_CONTROLLER_0, INPUT_PULLDOWN);
  // attachInterruptArg(digitalPinToInterrupt(ISR_PIN_MULTIDISPLAY_CONTROLLER_0),
  //                                          ISR_MultiDisplayController_0,
  //                                          NULL,
  //                                          FALLING);

  // --------------------------------------------------------------
  // Create Tasks.
  // --------------------------------------------------------------
  // Targets are determined for ESP32-P4, adjust as required.

  createTaskMultiDisplay();        // (target: n/ps)    General displays/indicators
  
  createTaskGPS();                 // (target: 10/ps)   Time & location
  createTaskSerialInfoCMD();       // (target: cmds/ps) Serial I/O

  createTaskGyro();                // (target: 200/ps)  Attitude
  createTaskMultiplexers();        // (target: 200/ps)  Fast general input
  createTaskPortControllerInput(); // (target: 1/ps)    Slow general input
  createTaskSwitches();            // (target: approx. max 1000/ps) Fast general output

  myAstroBegin();
  createTaskUniverse();            // (target: 1/ps)    Star tracking

  createTaskStorage();             // (target: 2/ps)    SD card
  createTaskLogging();             // (target: n/ps)    Log to sdcard

  syncTasks();
}

/**
 * @brief Loop
 */
int64_t prev_tv_sec;

void intervalBreach1Second(void) {
  // if (systemData.interval_breach_1_second) {
    // store system time
    storeLocalTime();
    // store rtc time
    storeRTCTime();
    // set loop counter
    systemData.total_loops_a_second = systemData.loops_a_second;
    systemData.loops_a_second = 0;
    // set gps counters
    systemData.total_gps = systemData.i_count_read_gps;
    systemData.i_count_read_gps = 0;
    // set ins counters
    systemData.total_ins = systemData.i_count_read_ins;
    systemData.i_count_read_ins = 0;
    // set gyro counters
    systemData.total_gyro_0 = systemData.i_count_read_gyro_0;
    systemData.i_count_read_gyro_0 = 0;
    // set mplex counters
    systemData.total_mplex_0 = systemData.i_count_read_mplex_0;
    systemData.i_count_read_mplex_0 = 0;
    // set mplex counters
    systemData.total_matrix = systemData.i_count_matrix;
    systemData.i_count_matrix = 0;
    // set mplex counters
    systemData.total_portcontroller_output = systemData.i_count_port_controller_output;
    systemData.i_count_port_controller_output = 0;
    // set mplex counters
    systemData.total_universe = systemData.i_count_track_planets;
    systemData.i_count_track_planets = 0;
    // set mplex counters
    systemData.total_infocmd = systemData.i_count_read_serial_commands;
    systemData.i_count_read_serial_commands = 0;
    // set portcontroller input counters
    systemData.total_portcontroller_input = systemData.i_count_portcontroller_input;
    systemData.i_count_portcontroller_input = 0;
    // set display counters
    systemData.total_display = systemData.i_count_display;
    systemData.i_count_display = 0;
    // set second flags
    systemData.interval_breach_track_planets = 1;
    // set uptime
    systemData.uptime_seconds++;
    if (systemData.uptime_seconds >= LONG_MAX - 2)
      {systemData.uptime_seconds = 0;
        Serial.println("[reset uptime_seconds] " + String(systemData.uptime_seconds));
      }
  // }
}

void loop() {

  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
  satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

  if (tv_now.tv_sec != prev_tv_sec) {
    prev_tv_sec = tv_now.tv_sec;
    systemData.interval_breach_1_second=true;
    systemData.interval_breach_1_second_output=true;
    intervalBreach1Second();
    Serial.println("[ " + String(satioData.local_unixtime_uS) + " ]" + 
                   " gps=" + String(gnrmcData.utc_time) +
                   " rtc=" + String(satioData.padded_rtc_time_HHMMSS) +
                   " lcl=" + String(satioData.padded_local_time_HHMMSS) +
                   " syn=" + String(satioData.padded_rtc_sync_time_HHMMSS) +
                   "  t_loop=" + String(systemData.total_loops_a_second) +
                   " t_gps=" + String(systemData.total_gps) +
                   " t_ins=" + String(systemData.total_ins) +
                   " t_gyr=" + String(systemData.total_gyro_0) +
                   " t_mlx=" + String(systemData.total_mplex_0) +
                   " t_uni=" + String(systemData.total_universe) +
                   " t_pci=" + String(systemData.total_portcontroller_input) +
                   " t_mtx=" + String(systemData.total_matrix) +
                   " t_pco=" + String(systemData.total_portcontroller_output) +
                   " t_dsp=" + String(systemData.total_display) +
                   "  lat=" + String(satioData.degrees_latitude, 7) +
                   " lon=" + String(satioData.degrees_longitude, 7) +
                   " alt=" + String(satioData.altitude_converted) + "_" + String(satioData.char_altitude_unit_mode[satioData.altitude_unit_mode]) +
                   " ghd=" + String(satioData.ground_heading) +
                   " spd=" + String(satioData.speed_converted) + "_" + String(satioData.char_speed_unit_mode[satioData.speed_unit_mode]) +
                   "  ang_x=" + String(gyroData.gyro_0_ang_x) +
                   " ang_y=" + String(gyroData.gyro_0_ang_y) +
                   " ang_z=" + String(gyroData.gyro_0_ang_z) +
                   " gyr_x=" + String(gyroData.gyro_0_gyr_x) +
                   " gyr_y=" + String(gyroData.gyro_0_gyr_y) +
                   " gyr_z=" + String(gyroData.gyro_0_gyr_z) +
                   " acc_x=" + String(gyroData.gyro_0_acc_x) +
                   " acc_y=" + String(gyroData.gyro_0_acc_y) +
                   " acc_z=" + String(gyroData.gyro_0_acc_z) +
                   " mag_x=" + String(gyroData.gyro_0_mag_x) +
                   " mag_y=" + String(gyroData.gyro_0_mag_y) +
                   " mag_z=" + String(gyroData.gyro_0_mag_z)
    );
  }
  
  systemData.loops_a_second++;
}