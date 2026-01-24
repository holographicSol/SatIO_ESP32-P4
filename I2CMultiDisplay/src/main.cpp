/*
Written by Benjamin Jack Cullen.

SatIO I2C Multi-Display Controller.
Lcdgfx Library SSD1306 is currently used for the SSD1309 panels for larger more visible displays (in green).
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
// #include <TM1637TinyDisplay.h>  // Include 4-Digit Display lib https://github.com/jasonacox/TM1637TinyDisplay
// #include <TM1637TinyDisplay6.h> // Include 6-Digit Display lib https://github.com/jasonacox/TM1637TinyDisplay
#include <FastLED.h>            // https://github.com/FastLED
#include "lcdgfx.h"             // https://github.com/lexus2k/lcdgfx
#include "lcdgfx_gui.h"         // https://github.com/lexus2k/lcdgfx
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "./multiplexers.h"
#include "./strval.h"
#include "./i2c_helper.h"

#include "freertos/semphr.h"
SemaphoreHandle_t xSevenSegMutex = NULL;

/** ----------------------------------------------------------------------------
 * Task Display.
 */
#define TASK_DISPLAY_PRIORITY               5
#define TASK_DISPLAY_CORE                   0
#define TASK_DISPLAY_STACK_SIZE             20480
#define TICK_DELAY_TASK_DISPLAY             true
#define DELAY_TASK_DISPLAY                  1
TaskHandle_t TaskDisplay;

/** ----------------------------------------------------------------------------
 * SSD1306 Displays - Mixed 128×32 and 128×64.
 */
#define MAX_SSD1306_128X32   0 // 0: specify number of atached displays
#define MAX_SSD1306_128X64   4 // 0: specify number of atached displays
#define MAX_SSD1306_DISPLAYS (MAX_SSD1306_128X32 + MAX_SSD1306_128X64)
#define MAX_SSD1306_DISPLAY_VALUES 3 // number of values per display

// Display type
enum Ssd1306Type {
    SSD_128X32 = 0,
    SSD_128X64 = 1,
    /* 1: add type as required */
};

// Display data (properties in ssd1306_displays must be updated if modified)
struct Ssd1306Display {
    DisplaySSD1306_128x32_I2C* display32; // NULL if 64-high
    DisplaySSD1306_128x64_I2C* display64; // NULL if 32-high
    NanoCanvas<128,32,1>*      canvas32;  // NULL if 64-high
    NanoCanvas<128,64,1>*      canvas64;  // NULL if 32-high
    Ssd1306Type                type;      // display type
    const uint8_t*             font;
    int                        dx[MAX_SSD1306_DISPLAY_VALUES]; // x position to print on canvas
    int                        dy[MAX_SSD1306_DISPLAY_VALUES]; // y position to print on canvas
    char                       value[MAX_SSD1306_DISPLAY_VALUES][MAX_IIC_BUFFER_SIZE]; // values to be drawn on canvas
    char                       prev_value[MAX_SSD1306_DISPLAY_VALUES][MAX_IIC_BUFFER_SIZE]; // values to be drawn on canvas
    bool                       draw; // draws canvas on display once if true
};

// Display instances (each atached display must have its own instance)
DisplaySSD1306_128x32_I2C ssd1306_display_0(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});
DisplaySSD1306_128x64_I2C ssd1306_display_1(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});
DisplaySSD1306_128x64_I2C ssd1306_display_2(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});
DisplaySSD1306_128x64_I2C ssd1306_display_3(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});
DisplaySSD1306_128x64_I2C ssd1306_display_4(-1, {0, 0x3C, IIC_BUS0_SCL, IIC_BUS0_SDA});

// Canvas instances (each atached display must have its own canvas)
NanoCanvas<128,32,1> canvas_128x32_0;
NanoCanvas<128,64,1> canvas_128x64_1;
NanoCanvas<128,64,1> canvas_128x64_2;
NanoCanvas<128,64,1> canvas_128x64_3;
NanoCanvas<128,64,1> canvas_128x64_4;

NanoCanvas<60,8,1> canvas_60x8_0;

NanoCanvas<7*8,8,1> canvas_8charsx8; // 7px x 8 chars
NanoCanvas<7*7,8,1> canvas_7charsx8; // 7px x 7 chars
NanoCanvas<7*6,8,1> canvas_6charsx8; // 7px x 6 chars

NanoCanvas<30,30,1> canvas_30_30_0;
NanoCanvas<30,30,1> canvas_30_30_1;

NanoCanvas<41,41,1> canvas_41_41_0;
NanoCanvas<41,41,1> canvas_41_41_1;

NanoCanvas<10,56,1> canvas_pitch;
NanoCanvas<50,16,1> canvas_yaw;

/* 3: add canvas */

// Unified array (instance index directly correlates with multiplexer channel)
Ssd1306Display ssd1306_displays[MAX_SSD1306_DISPLAYS] = {
    // { &ssd1306_display_0, nullptr, &canvas_128x32_0, nullptr, SSD_128X32, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    { nullptr, &ssd1306_display_1, nullptr, &canvas_128x64_1, SSD_128X64, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    { nullptr, &ssd1306_display_2, nullptr, &canvas_128x64_2, SSD_128X64, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    { nullptr, &ssd1306_display_3, nullptr, &canvas_128x64_3, SSD_128X64, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    { nullptr, &ssd1306_display_4, nullptr, &canvas_128x64_4, SSD_128X64, ssd1306xled_font6x8, 0, 0, {}, {}, false}
    /* 4: add display to ssd1306_displays and configure display properties */
};

/** ----------------------------------------------------------------------------
 * Display Brightness.
 */
#define MAX_BRIGHTNESS_STAGE 6
uint8_t brightness_stage = 5;
const int LED_BRIGHTNESS_LEVELS[MAX_BRIGHTNESS_STAGE]  = {0, 1, 50, 100, 200, 255}; // adjust as required (0-255)
const int SEG_BRIGHTNESS_LEVELS[MAX_BRIGHTNESS_STAGE]  = {0, 1, 2, 3, 4, 7}; // adjust as required (0-7)

/**----------------------------------------------------------------------------
 * Interrupts.
 * High/Low inversion for receiving device pin mode INPUT_PULLDOWN.
 * INPUT_PULLDOWN may be required on receiving device pin to avoid 'floating'.
 */
#define MASTER_INTERRUPT_PIN 13

/* Interrupt master (this is not an ISR) */
void interruptMaster() {
  // Serial.println("[interruptMaster] Interrupting master");
  digitalWrite(MASTER_INTERRUPT_PIN, LOW);
  digitalWrite(MASTER_INTERRUPT_PIN, HIGH);
}

/* ISR brightness button */
#define ISR_PIN_BRIGHTNESS 36
bool iter_brightness = false;
void IRAM_ATTR ISR_brightness_button(void * arg) {iter_brightness = true;}

/**----------------------------------------------------------------------------
 * Addressable LEDs.
 */
#define MAX_INDICATORS 20 // adjust as required
#define INDICATOR_DIO 17
#define MAX_LED_COLOR_VALUES        3
#define INDEX_LED_COLOR_VALUE_RED   0
#define INDEX_LED_COLOR_VALUE_GREEN 1
#define INDEX_LED_COLOR_VALUE_BLUE  2
CRGB leds[MAX_INDICATORS];

void UpdateAllIndicators(int start, int end, int r, int g, int b) {
  for (int i=start; i<=end; i++) {leds[i] = CRGB(r,g,b);}
}

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 1.
 * @warning Uncomment and customize to use locally (backup first) or copy into project!
*/
void requestEventBus1Bin() {
  // Serial.println("[requestEventBus1Bin] id: " + String(I2CLinkBus1.REQUEST_ID));
  switch (I2CLinkBus1.REQUEST_ID) {

    // Request ID: 1 - Brightness Stage
    case 0x01: {
        // Serial.println("[requestEventBus1Bin] preparing to send requested data (brightness_stage): " + String(brightness_stage));
        clearI2CLinkOutputPacket(I2CLinkBus1);
        write_uint8_ToPacket(I2CLinkBus1.OUTPUT_PACKET, 0, brightness_stage);
        writeI2CToMasterBin(Wire1, I2CLinkBus1, 1, 0);
        break;
    }
    default: {
        // Serial.println("[requestEventBus1Bin] event id is not defined: " + String(I2CLinkBus1.REQUEST_ID));
        break;
    }
  }
  I2CLinkBus1.REQUEST_ID = 0;
}

/** ----------------------------------------------------------------------------
 * @brief Receive event handler for Bus 1.
 * @warning Uncomment and customize to use locally (backup first) or copy into project!
*/
size_t str_len;
uint8_t cmd;
uint8_t display_index;
uint8_t value_idx;
uint8_t dx;
uint8_t dy;
uint8_t colorR;
uint8_t colorG;
uint8_t colorB;
bool display_decimal = false;
bool display_colon = false;
char    value_char[32];

int   satellite_count=0;
float gps_precision=0;

char local_time_str[32]="";
char local_date_str[32]="";

float gyro_roll=0;
float gyro_pitch=0;
float gyro_yaw=0;

float gyro_accel_x=0;
float gyro_accel_y=0;
float gyro_accel_z=0;

long gyro_mag_x=0;
long gyro_mag_y=0;
long gyro_mag_z=0;

void receiveEventBus1Bin(size_t n_bytes_received) {
  if (n_bytes_received < 1) return;
  cmd = Wire1.read(); // expects uint8 command byte (up to 255 unique commands can be accepted). 
  // Serial.println("[receiveEventBus1Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
  switch (cmd) {

    /* Set Request ID: 1 */
    case 0x01: {
      // Serial.println("[receiveEventBus1Bin] preparing to process command: " + String(cmd));
      I2CLinkBus1.REQUEST_ID=0x01;
      drainBus(Wire1);
      break;
    }

    // /* Indicators */
    // case 0x0A: {
    //   if (n_bytes_received!=5) {return;}
      
    //   // display index
    //   read_uint8_FromWire(Wire1, display_index);
    //   if (display_index >= MAX_INDICATORS) {
    //     Serial.printf("[ERROR] LED display_index out of bounds: %d\n", display_index);
    //     drainBus(Wire1);
    //     break;
    //   }

    //   // color value red
    //   read_uint8_FromWire(Wire1, colorR);
    //   if (colorR > 255 || colorR < 0) {
    //     Serial.printf("[ERROR] LED colorR value out of bounds: %d\n", colorR);
    //     drainBus(Wire1);
    //     break;
    //   }

    //   // color value green
    //   read_uint8_FromWire(Wire1, colorG);
    //   if (colorG > 255 || colorG < 0) {
    //     Serial.printf("[ERROR] LED colorG value out of bounds: %d\n", colorG);
    //     drainBus(Wire1);
    //     break;
    //   }

    //   // color value blue
    //   read_uint8_FromWire(Wire1, colorB);
    //   if (colorB > 255 || colorB < 0) {
    //     Serial.printf("[ERROR] LED colorB value out of bounds: %d\n", colorB);
    //     drainBus(Wire1);
    //     break;
    //   }

    //   // update LED color
    //   leds[display_index] = CRGB(colorR,colorG,colorB);
    //   // Serial.printf("[RX] led %d: r=%d g=%d b=%d\n",
    //   //               display_index,
    //   //               colorR,
    //   //               colorG,
    //   //               colorB
    //   //             );
    //   drainBus(Wire1);
    //   break;
    // }

    /* Satellite count */
    case 0x0B: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      satellite_count = atoi(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* GPS precision */
    case 0x0C: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gps_precision = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Local time string */
    case 0x0D: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      memset(local_time_str, 0, sizeof(local_time_str));
      strncpy(local_time_str, value_char, strlen(value_char));
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Local date string */
    case 0x0E: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      memset(local_date_str, 0, sizeof(local_date_str));
      strncpy(local_date_str, value_char, strlen(value_char));
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Roll */
    case 0x15: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_roll = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Pitch */
    case 0x16: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_pitch = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Yaw */
    case 0x17: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_yaw = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Accel X */
    case 0x1F: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_accel_x = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Accel Y */
    case 0x20: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_accel_y = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Accel Z */
    case 0x21: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_accel_z = atof(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Mag X */
    case 0x29: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_mag_x = atol(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Mag Y */
    case 0x2A: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_mag_y = atol(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Mag Z */
    case 0x2B: {
      // value
      str_len = n_bytes_received - 1; // deduct command byte
      memset(value_char, 0, sizeof(value_char));
      read_nchars_FromWire(Wire1, value_char, str_len);
      value_char[str_len] = '\0'; // null terminate
      // update display value
      gyro_mag_z = atol(value_char);
      // Serial.printf("[RX] SSD1306: value=%s\n", value_char);
      drainBus(Wire1);
      break;
    }

    /* Update SSD1306 Canvas Bool */
    case 0x0A: {
      // display index
      read_uint8_FromWire(Wire1, display_index);
      if (display_index >= MAX_SSD1306_DISPLAYS) {
        Serial.printf("[ERROR] SSD1306 draw display_index out of bounds: %d\n", display_index);
        drainBus(Wire1);
        break;
      }
      // set draw flag
      auto& dispSSD1306 = ssd1306_displays[display_index];
      dispSSD1306.draw=true;
      // Serial.printf("[RX] SSD1306 %d: draw canvas.\n",
      //               display_index
      //             );
      drainBus(Wire1);
      break;
    }
    default: {
        // Serial.println("[receiveEventBus1Bin] command is not defined: " + String(cmd));
        drainBus(Wire1);
        break;
    }
  }
}

/** ----------------------------------------------------------------------------
 * taskDisplay.
 * 
 * @brief Update Display(s) & Any Indicators.
 */
unsigned long current_time;
static unsigned long display_loop_counter = 0;
static unsigned long display_loop_last_time = 0;

long pitch_pos=0;
long yaw_pos=0;




int fooang=0;

void taskDisplay(void * pvParameters) {
  // while (global_task_sync==false) {vTaskDelay(1);}
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    
    // -----------------------------------------------------
    // Adjust Brightness Level
    // -----------------------------------------------------
    if (iter_brightness==true) {

      // -----------------------
      // Set brightness stage
      // -----------------------
      iter_brightness=false;
      brightness_stage = brightness_stage + 1;
      if (brightness_stage>=MAX_BRIGHTNESS_STAGE) {brightness_stage=0;}
      // Serial.printf("[Brightness level] %d\n", brightness_stage);

      // -----------------------------------------------------
      // LEDs: Adjust Brightness
      // -----------------------------------------------------
      // clear
      // -----------------------
      FastLED.setBrightness(LED_BRIGHTNESS_LEVELS[brightness_stage]);
      FastLED.show();

      // -----------------------------------------------------
      // SSD1306: Adjust Brightness
      // -----------------------------------------------------
      // clear
      // -----------------------
      if (brightness_stage==0) {
        for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
          // Serial.printf("[DEBUG] setI2CMultiplexChannel for SSD1306 display %d\n", i_display);
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
          auto& dispSSD1306 = ssd1306_displays[i_display];
          
          if (dispSSD1306.type==SSD_128X32) {
            dispSSD1306.canvas32->clear();
            dispSSD1306.display32->drawCanvas(0, 0, *dispSSD1306.canvas32);
          }
          else if (dispSSD1306.type==SSD_128X64) {
            dispSSD1306.canvas64->clear();
            dispSSD1306.display64->drawCanvas(0, 0, *dispSSD1306.canvas64);
          }
        }
      }
      // -----------------------
      // set brightness
      // -----------------------
      else {
        for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
          auto& dispSSD1306 = ssd1306_displays[i_display];
          // Serial.printf("[DEBUG] setI2CMultiplexChannel for SSD1306 display %d\n", i_display);
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
          if (dispSSD1306.type==SSD_128X32) {
            // send brightness/contrast command if supported
          }
          else if (dispSSD1306.type==SSD_128X64) {
            // send brightness/contrast command if supported
          }
        }
      }
      interruptMaster();
    }
    // -----------------------------------------------------
    // Update Displays
    // -----------------------------------------------------
    // 6 chars: -nn.nn 

    if (!brightness_stage==0) {

      for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
        auto& dispSSD1306 = ssd1306_displays[i_display];
        if (dispSSD1306.draw==true) {
          dispSSD1306.draw=false;
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);

          /** ---------------------------------------------
           * Basics
           */
          if (i_display==0) {
            // satellite count
            canvas_6charsx8.clear();
            canvas_6charsx8.printFixed(21 - (int(strlen(String(satellite_count).c_str()) * 7) /2), 0, String(satellite_count).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 0, canvas_6charsx8);
            // gps precision
            canvas_6charsx8.clear();
            canvas_6charsx8.printFixed(21 - (int(strlen(String(gps_precision).c_str()) * 7) /2), 0, String(gps_precision).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(128-42, 0, canvas_6charsx8);
            // local time
            canvas_8charsx8.clear();
            canvas_8charsx8.printFixed(28 - (int(strlen(String(local_time_str).c_str()) * 7) /2), 0, String(local_time_str).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(64-28, 10, canvas_8charsx8);
            // local date
            canvas_8charsx8.clear();
            canvas_8charsx8.printFixed(28 - (int(strlen(String(local_date_str).c_str()) * 7) /2), 0, String(local_date_str).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(64-28, 20, canvas_8charsx8);
            // position and style
          }

          /** ---------------------------------------------
           * Attitude
           */
          else if (i_display==1) {
            // -------------------------------------------
            // Value roll
            // -------------------------------------------
            canvas_7charsx8.clear();
            canvas_7charsx8.printFixed(0, 0, String(gyro_roll).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 0, canvas_7charsx8);
            // -------------------------------------------
            // Value pitch
            // -------------------------------------------
            canvas_7charsx8.clear();
            canvas_7charsx8.printFixed(0, 0, String(gyro_pitch).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 10, canvas_7charsx8);
            // -------------------------------------------
            // Value yaw
            // -------------------------------------------
            canvas_7charsx8.clear();
            canvas_7charsx8.printFixed(0, 0, String(gyro_yaw).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 20, canvas_7charsx8);
            // -------------------------------------------
            // Pitch scale
            // -------------------------------------------
            canvas_pitch.clear();
            canvas_pitch.drawVLine(5, 0, 49); // axis
            canvas_pitch.drawHLine(6, 0, 8);  // end
            canvas_pitch.drawHLine(6, 1, 9);  // end
            canvas_pitch.drawHLine(6, 5, 6);  // q4
            canvas_pitch.drawHLine(6, 6, 7);  // q4
            canvas_pitch.drawHLine(6, 7, 6);  // q4
            canvas_pitch.drawHLine(6, 12, 7); // top center
            canvas_pitch.drawHLine(6, 17, 6);  // q3
            canvas_pitch.drawHLine(6, 18, 7);  // q3
            canvas_pitch.drawHLine(6, 19, 6);  // q3
            canvas_pitch.drawHLine(4, 23, 5); // center
            canvas_pitch.drawHLine(3, 24, 9); // center
            canvas_pitch.drawHLine(4, 25, 5); // center
            canvas_pitch.drawHLine(6, 29, 6);  // q2
            canvas_pitch.drawHLine(6, 30, 7);  // q2
            canvas_pitch.drawHLine(6, 31, 6);  // q2
            canvas_pitch.drawHLine(6, 36, 7); // bottom center
            canvas_pitch.drawHLine(6, 42, 6);  // q1
            canvas_pitch.drawHLine(6, 43, 7);  // q1
            canvas_pitch.drawHLine(6, 44, 6);  // q1
            canvas_pitch.drawHLine(6, 48, 9); // begin
            canvas_pitch.drawHLine(6, 49, 8); // begin
            // -------------------------------------------
            // Pitch mapped
            // -------------------------------------------
            pitch_pos = map(gyro_pitch, -90, 90, 0, 47);
            if (isnan(pitch_pos) || pitch_pos < 0) pitch_pos = 0;
            if (pitch_pos > 47) pitch_pos = 47;
            // -------------------------------------------
            // Pitch slider
            // -------------------------------------------
            canvas_pitch.fillRect(0, (int)pitch_pos, 1, (int)pitch_pos+1); // filled rectangle 2x2 pitch
            dispSSD1306.display64->drawCanvas(118, 0, canvas_pitch);
            // -------------------------------------------
            // Yaw scale
            // -------------------------------------------
            canvas_yaw.clear();
            canvas_yaw.drawHLine(0, 5, 49); // axis
            canvas_yaw.drawVLine(0, 6, 8);  // end
            canvas_yaw.drawVLine(1, 6, 9);  // end
            canvas_yaw.drawVLine(5, 6, 6);  // q4
            canvas_yaw.drawVLine(6, 6, 7);  // q4
            canvas_yaw.drawVLine(7, 6, 6);  // q4
            canvas_yaw.drawVLine(12, 6, 7); // left center
            canvas_yaw.drawVLine(17, 6, 6); // q3
            canvas_yaw.drawVLine(18, 6, 7); // q3
            canvas_yaw.drawVLine(19, 6, 6); // q3
            canvas_yaw.drawVLine(23, 4, 5); // center
            canvas_yaw.drawVLine(24, 3, 9); // center
            canvas_yaw.drawVLine(25, 4, 5); // center
            canvas_yaw.drawVLine(29, 6, 6); // q2
            canvas_yaw.drawVLine(30, 6, 7); // q2
            canvas_yaw.drawVLine(31, 6, 6); // q2
            canvas_yaw.drawVLine(36, 6, 7); // right center
            canvas_yaw.drawVLine(42, 6, 6); // q1
            canvas_yaw.drawVLine(43, 6, 7); // q1
            canvas_yaw.drawVLine(44, 6, 6); // q1
            canvas_yaw.drawVLine(48, 6, 9); // begin
            canvas_yaw.drawVLine(49, 6, 8); // begin
            // -------------------------------------------
            // Yaw mapped
            // -------------------------------------------
            yaw_pos = map(gyro_yaw, -180, 180, 0, 47);
            if (isnan(yaw_pos) || yaw_pos < 0) yaw_pos = 0;
            if (yaw_pos > 47) yaw_pos = 47;
            // -------------------------------------------
            // Yaw slider
            // -------------------------------------------
            canvas_yaw.fillRect((int)yaw_pos, 0, (int)yaw_pos+1, 1); // filled rectangle 2x2 yaw
            dispSSD1306.display64->drawCanvas(127-49-10, 48, canvas_yaw);
            // -------------------------------------------
            // Roll rotator
            // -------------------------------------------
            canvas_30_30_0.clear();
            canvas_30_30_0.fillRect(8,  14, 20, 14); // base
            canvas_30_30_0.fillRect(8,  11,  8, 14); // lref
            canvas_30_30_0.fillRect(20, 11, 20, 14); // rref
            canvas_30_30_0.rotate(canvas_30_30_1, (int)fooang); // test rotation
            // canvas_30_30_0.rotate(canvas_30_30_1, (int)gyro_roll); // gyro roll
            fooang++;
            if (fooang>360) {fooang=0;}
            dispSSD1306.display64->drawCanvas(68+10, 15, canvas_30_30_1);
          }

          /** ---------------------------------------------
           * Angle Velocity
           */
          else if (i_display==2) {
            // gyro accel x
            canvas_6charsx8.clear();
            canvas_6charsx8.printFixed(0, 0, String(gyro_accel_x).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 0, canvas_6charsx8);
            // gyro accel y
            canvas_6charsx8.printFixed(0, 0, String(gyro_accel_y).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 10, canvas_6charsx8);
            // gyro accel z
            canvas_6charsx8.clear();
            canvas_6charsx8.printFixed(0, 0, String(gyro_accel_z).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 20, canvas_6charsx8);
            // position values and draw axial acceleration graphic
          }

          /** ---------------------------------------------
           * Magnetic Field
           */
          else if (i_display==3) {
            // gyro mag x
            canvas_60x8_0.clear();
            canvas_60x8_0.printFixed(0, 0, String(gyro_mag_x).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 0, canvas_60x8_0);
            // gyro mag y
            canvas_60x8_0.clear();
            canvas_60x8_0.printFixed(0, 0, String(gyro_mag_y).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 10, canvas_60x8_0);
            // gyro mag z
            canvas_60x8_0.clear();
            canvas_60x8_0.printFixed(0, 0, String(gyro_mag_z).c_str(), STYLE_BOLD);
            dispSSD1306.display64->drawCanvas(0, 20, canvas_60x8_0);
            // position values and draw axial magnetic field graphic
          }
        }
      }
    }
  
    // -----------------------------------------------------
    // Track and print loops per second
    // -----------------------------------------------------
    display_loop_counter++;
    current_time = millis();
    if (current_time - display_loop_last_time >= 1000) {
      Serial.printf("Display Task: %lu loops/sec\n", display_loop_counter);
      display_loop_counter = 0;
      display_loop_last_time = current_time;
    }
    // -----------------------------------------------------
    // Delay Task
    // -----------------------------------------------------
    if (TICK_DELAY_TASK_DISPLAY==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_DISPLAY / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_DISPLAY);}
  }
}

/** ----------------------------------------------------------------------------
 * Create the display task.
 */
void createTaskDisplay() {
    xTaskCreatePinnedToCore(
    taskDisplay,             /* Function to implement the task */
    "TaskDisplay",           /* Name of the task */
    TASK_DISPLAY_STACK_SIZE, /* Stack size in words */
    NULL,                    /* Task input parameter */
    2,   /* Priority of the task */
    &TaskDisplay,            /* Task handle. */
    TASK_DISPLAY_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Setup.
 */
void setup() {
  // --------------------------------------------------------------
  // Required for operations in taks that may take longer than 5s..
  // --------------------------------------------------------------
  // esp_task_wdt_config_t config = {
  //   .timeout_ms = 60* 1000, // 1 minute
  //   .trigger_panic = true,  // Trigger panic if watchdog timer not reset
  // };
  // esp_task_wdt_reconfigure(&config);
  // enableLoopWDT();
  // --------------------------------------------------------------
  // Warmup delay: some devices require at least one second start.
  // --------------------------------------------------------------
  delay(1000);
  
  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // ------------------------------------------------------------
  // Indicators
  // ------------------------------------------------------------
  FastLED.addLeds<NEOPIXEL, INDICATOR_DIO>(leds, MAX_INDICATORS);
  FastLED.setBrightness(255); // 0–255, adjust as needed
  
  for (int i=0; i<10; i++) {
    UpdateAllIndicators(0, MAX_INDICATORS, 0,0,0);
    FastLED.show();
    delay(50);
    UpdateAllIndicators(0, MAX_INDICATORS, 255,0,0);
    FastLED.show();
    delay(50);
  }
  UpdateAllIndicators(0, MAX_INDICATORS, 0,0,0);
  FastLED.show();

  // ------------------------------------------------------------
  // I2C Master Initialization
  // ------------------------------------------------------------
  Wire.setPins(IIC_BUS0_SDA, IIC_BUS0_SCL);
  
  if (!Wire.begin(IIC_BUS0_SDA, IIC_BUS0_SCL, 800000))
    {Serial.printf("[I2C] master failed to start (SDA=%d SCL=%d)\n",
                   IIC_BUS0_SDA,
                   IIC_BUS0_SCL
                  );}
  else
    {Serial.printf("[I2C] master started sucessfully (SDA=%d SCL=%d)\n",
                   IIC_BUS0_SDA,
                   IIC_BUS0_SCL
                  );}
  Wire.setTimeOut(1000);
  
  // ------------------------------------------------------------
  // I2C Slave Initialization
  // ------------------------------------------------------------
  Wire1.setPins(IIC_BUS1_SDA, IIC_BUS1_SCL);
  if (!Wire1.begin(SLAVE_ADDR_BUS1))
    {Serial.printf("[I2C] slave failed to start on address 0x%02X (SDA=%d SCL=%d)\n",
                   SLAVE_ADDR_BUS1,
                   IIC_BUS1_SDA,
                   IIC_BUS1_SCL
                  );}
  else
    {Serial.printf("[I2C] slave started sucessfully on address 0x%02X (SDA=%d SCL=%d)\n",
                   SLAVE_ADDR_BUS1,
                   IIC_BUS1_SDA,
                   IIC_BUS1_SCL
                  );}
  Wire1.setTimeOut(1000);

  // ------------------------------------------------------------
  // Initialize I2C Multiplexers
  // ------------------------------------------------------------
  setI2CMultiplexChannel(Wire, i2c_mux_0, 0);

  // ------------------------------------------------------------
  // Initialize Analog/Digital Multiplexers
  // ------------------------------------------------------------
  initADMultiplexer(ad_mux_0);
  initADMultiplexer(ad_mux_1);
  // --------------------------------------------------------------------
  // Set analog/digital multiplexer sig as OUTPUT LOW (ready for write).
  // --------------------------------------------------------------------
    Serial.printf("[DEBUG] pinMode(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]=%d, OUTPUT)\n", ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
    pinMode(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT); // DIO pins
    Serial.printf("[DEBUG] pinMode(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]=%d, OUTPUT)\n", ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
    pinMode(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT); // CLK pins
    Serial.printf("[DEBUG] digitalWrite(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]=%d, LOW)\n", ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
    digitalWrite(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], LOW);
    Serial.printf("[DEBUG] digitalWrite(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]=%d, LOW)\n", ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
    digitalWrite(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], LOW);

  // ------------------------------------------------------------
  // Initialize SSD1306 Display(s)
  // ------------------------------------------------------------
  Serial.println("[Initializing I2C Multiplexer Displays]");
  for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
    setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
    auto& disp = ssd1306_displays[i_display];

    Serial.printf("  Display %d: type=%d display32=%p display64=%p canvas32=%p canvas64=%p\n",
                  i_display, disp.type,
                  disp.display32, disp.display64,
                  disp.canvas32,  disp.canvas64
                );
    if (disp.type == SSD_128X32 && disp.display32 == nullptr)
      {Serial.println("!!! FATAL: display32 is NULL pointer !!!");}
    if (disp.type == SSD_128X64 && disp.display64 == nullptr)
      {Serial.println("!!! FATAL: display64 is NULL pointer !!!");}

    if (disp.type==SSD_128X32) {
      disp.display32->begin();
      disp.display32->clear();
      // test canvas
      disp.canvas32->setFixedFont(disp.font);
      disp.canvas32->clear();
      disp.canvas32->printFixed(1, 1, "SatIO", STYLE_BOLD);
      disp.display32->drawCanvas(0, 0, *disp.canvas32);
    }
    else if (disp.type==SSD_128X64) {
      disp.display64->begin();
      disp.display64->clear();
      // test canvas
      disp.canvas64->setFixedFont(disp.font);
      disp.canvas64->clear();
      disp.canvas64->printFixed(1, 1, "SatIO", STYLE_BOLD);
      disp.display64->drawCanvas(0, 0, *disp.canvas64);
    }
  }
  delay(1000);
  for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
    setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
    auto& disp = ssd1306_displays[i_display];
    if (disp.type==SSD_128X32) {
      disp.display32->begin();
      disp.display32->clear();
    }
    else if (disp.type==SSD_128X64) {
      disp.display64->begin();
      disp.display64->clear();
    }
  }

  canvas_60x8_0.setFixedFont(ssd1306xled_font6x8);
  canvas_6charsx8.setFixedFont(ssd1306xled_font6x8);
  canvas_7charsx8.setFixedFont(ssd1306xled_font6x8);
  canvas_8charsx8.setFixedFont(ssd1306xled_font6x8);
  
  canvas_41_41_0.setFixedFont(ssd1306xled_font6x8);
  canvas_41_41_1.setFixedFont(ssd1306xled_font6x8);

  xSevenSegMutex = xSemaphoreCreateMutex();

  // ------------------------------------------------------------
  delay(1000);
  Serial.println("Initialization complete..");
  // ------------------------------------------------------------
  // Create Tasks
  // ------------------------------------------------------------
  createTaskDisplay();
  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  Wire1.onRequest(requestEventBus1Bin);
  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire1.onReceive(receiveEventBus1Bin);
  // ------------------------------------------------------------
  // Interrupts
  // ------------------------------------------------------------
  // Send interrupt to master to indicate data is ready
  pinMode(MASTER_INTERRUPT_PIN, OUTPUT);
  digitalWrite(MASTER_INTERRUPT_PIN, HIGH);
  // Button to iterate brightness level
  pinMode(ISR_PIN_BRIGHTNESS, INPUT);
  attachInterruptArg(digitalPinToInterrupt(ISR_PIN_BRIGHTNESS), ISR_brightness_button, NULL, RISING);
}

/** ----------------------------------------------------------------------------
 * Loop.
 * 
 * @brief Update Display(s) & Any Indicators.
 */

void loop() {
}