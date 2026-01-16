/*
Written by Benjamin Jack Cullen.

DisplayController - Simple I2C display module for I2C master requiring 1+ indicators/displays.
                    Receives data over I2C and writes received data to display(s).
                    Customize as required (+/- displays, triggered animations, etc).

Support:
        * Adressable LEDs (see FastLED compatibility)
        * SSD1306 OLEDs
        * 7 segment 4 Digit Displays (TM1637)
        * 7 segment 6 Digit Displays (TM1637)
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
#include <TM1637TinyDisplay.h>  // Include 4-Digit Display lib https://github.com/jasonacox/TM1637TinyDisplay
#include <TM1637TinyDisplay6.h> // Include 6-Digit Display lib https://github.com/jasonacox/TM1637TinyDisplay
#include <FastLED.h>            // https://github.com/FastLED
#include "lcdgfx.h"             // https://github.com/lexus2k/lcdgfx
#include "lcdgfx_gui.h"         // https://github.com/lexus2k/lcdgfx
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "./multiplexers.h"
#include "./strval.h"

#define SLAVE_ADDR 12 // set address as required (specify an I2C address for this device).

#define MASTER_SDA 21 // (Wire)
#define MASTER_SCL 22 // (Wire)
#define SLAVE_SDA  18 // (Wire1)
#define SLAVE_SCL  19 // (Wire1)

#define MAX_IIC_BUFER_SIZE 32

#define TASK_DISPLAY_PRIORITY               4
#define TASK_DISPLAY_CORE                   0
#define TASK_DISPLAY_STACK_SIZE             16384
#define TICK_DELAY_TASK_DISPLAY             true
#define DELAY_TASK_DISPLAY                  1

TaskHandle_t TaskDisplay;

/** ----------------------------------------------------------------------------
 * SSD1306 Displays - Mixed 128×32 and 128×64.
 */
#define MAX_SSD1306_128X32   1 // 0: specify number of atached displays
#define MAX_SSD1306_128X64   1 // 0: specify number of atached displays
#define MAX_SSD1306_DISPLAYS (MAX_SSD1306_128X32 + MAX_SSD1306_128X64)
#define MAX_SSD1306_DISPLAY_VALUES 2 // number of values per display

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
    char                       value[MAX_SSD1306_DISPLAY_VALUES][MAX_IIC_BUFER_SIZE]; // values to be drawn on canvas
    char                       prev_value[MAX_SSD1306_DISPLAY_VALUES][MAX_IIC_BUFER_SIZE]; // values to be drawn on canvas
    bool                       draw; // draws canvas on display once if true
};

// Display instances (each atached display must have its own instance)
DisplaySSD1306_128x32_I2C ssd1306_display_0(-1, {0, 0x3C, MASTER_SCL, MASTER_SDA});
DisplaySSD1306_128x64_I2C ssd1306_display_1(-1, {0, 0x3C, MASTER_SCL, MASTER_SDA});
/* 2: add display */

// Canvas instances (each atached display must have its own canvas)
NanoCanvas<128,32,1> canvas_128x32_0;
NanoCanvas<128,64,1> canvas_128x64_1;
/* 3: add canvas */

// Unified array (instance index directly correlates with multiplexer channel)
Ssd1306Display ssd1306_displays[MAX_SSD1306_DISPLAYS] = {
    { &ssd1306_display_0, nullptr, &canvas_128x32_0, nullptr, SSD_128X32, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    { nullptr, &ssd1306_display_1, nullptr, &canvas_128x64_1, SSD_128X64, ssd1306xled_font6x8, 0, 0, {}, {}, false},
    /* 4: add display to ssd1306_displays and configure display properties */
};

/** ----------------------------------------------------------------------------
 * 7 Segment Displays.
 */
#define MAX_7SEG_4DIGIT_DISPLAYS 2 // 0: specify number of atached displays
#define MAX_7SEG_6DIGIT_DISPLAYS 2 // 0: specify number of atached displays
#define MAX_7SEG_DISPLAYS  (MAX_7SEG_4DIGIT_DISPLAYS + MAX_7SEG_6DIGIT_DISPLAYS)
#define SEVEN_SEGMENT_DIO_PIN ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG] // 7Seg DIO -> multiplexer 0 SIG
#define SEVEN_SEGMENT_CLK_PIN ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG] // 7Seg CLK -> multiplexer 1 SIG

// Display type
enum SevenSegType {
  SEG_1DIGIT = 0,
  SEG_2DIGIT = 1,
  SEG_4DIGIT = 2,
  SEG_6DIGIT = 3,
  SEG_8DIGIT = 4,
  /* 1: add type as required */
};

// Display data (properties in seven_seg_displays must be updated if modified)
struct SevenSegDisplay {
    TM1637TinyDisplay*   display4;  // NULL if 6-digit
    TM1637TinyDisplay6*  display6;  // NULL if 4-digit
    SevenSegType         type;      // display type
    char value[MAX_IIC_BUFER_SIZE]; // value to be written to display
    char prev_value[MAX_IIC_BUFER_SIZE];
};

// Display instances
TM1637TinyDisplay  tm0(SEVEN_SEGMENT_CLK_PIN, SEVEN_SEGMENT_DIO_PIN);
TM1637TinyDisplay  tm1(SEVEN_SEGMENT_CLK_PIN, SEVEN_SEGMENT_DIO_PIN);
TM1637TinyDisplay6 tm2(SEVEN_SEGMENT_CLK_PIN, SEVEN_SEGMENT_DIO_PIN);
TM1637TinyDisplay6 tm3(SEVEN_SEGMENT_CLK_PIN, SEVEN_SEGMENT_DIO_PIN);
/* 2: add display */

// Unified array (instance index directly correlates with multiplexer channel)
SevenSegDisplay seven_seg_displays[MAX_7SEG_DISPLAYS] = {
    { &tm0, nullptr, SEG_4DIGIT, {""}, {""} },
    { &tm1, nullptr, SEG_4DIGIT, {""}, {""} },
    { nullptr, &tm2, SEG_6DIGIT, {""}, {""} },
    { nullptr, &tm3, SEG_6DIGIT, {""}, {""} }
    /* 4: add display to seven_seg_displays and configure display properties */
};

/** ----------------------------------------------------------------------------
 * Dot/Colon selection arrays for 7-Segment 4 Digit Displays.
 */
uint8_t dot_colon_select_7seg_4digit[32] = {
  0b00000000, // none
  0b10000000, // 1: 0.000
  0b01000000, // 2: 00.00
  0b00100000, // 3: 000.0
  0b11100000, // 4: 0.0.0.0
  0b01000000, // 5: 00:00
  0b11100000, // 6: 0.0:0.0
};

/** ----------------------------------------------------------------------------
 * Dot/Colon selection arrays for 7-Segment 6 Digit Displays.
 */
uint8_t dot_colon_select_7seg_6digit[32] = {
  0b00000000, // none
  0b10000000, // 1: 0.00000
  0b01000000, // 2: 00.0000
  0b00100000, // 3: 000.000
  0b00010000, // 4: 0000.00
  0b00001000, // 5: 00000.0
  0b00000100, // 6: 000000.
  0b01010000, // 7: 00.00.00
  0b01010000, // 8: 00:00:00
};

/** ----------------------------------------------------------------------------
 * Display Brightness.
 */
#define MAX_BRIGHTNESS_STAGE 6
int brightness_stage = 5;
const int LED_BRIGHTNESS_LEVELS[MAX_BRIGHTNESS_STAGE]  = {0, 1, 50, 100, 200, 255}; // adjust as required (0-255)
const int SEG_BRIGHTNESS_LEVELS[MAX_BRIGHTNESS_STAGE]  = {0, 1, 2, 3, 4, 7}; // adjust as required (0-7)

/** ----------------------------------------------------------------------------
 * I2CLinkStruct.
 */
struct I2CLinkStruct {
  volatile int i_token;
  char         * token;
  byte          OUTPUT_BUFFER[MAX_IIC_BUFER_SIZE];
  char          INPUT_BUFFER[MAX_IIC_BUFER_SIZE];
  char          TMP_BUFFER[MAX_IIC_BUFER_SIZE];
};
I2CLinkStruct I2CLink;

/**----------------------------------------------------------------------------
 * Interrupts.
 * High/Low inversion for receiving device pin mode INPUT_PULLDOWN.
 * INPUT_PULLDOWN may be required on receiving device pin to avoid 'floating'.
 */
#define MASTER_INTERRUPT_PIN 13

void interruptMaster() {
  Serial.println("[interruptMaster] Interrupting master");
  digitalWrite(MASTER_INTERRUPT_PIN, LOW);
  digitalWrite(MASTER_INTERRUPT_PIN, HIGH);
}

#define ISR_PIN_BRIGHTNESS 36
bool iter_brightness = false;
void IRAM_ATTR iter_brightness_ISR(void * arg) {iter_brightness = true;}

/**----------------------------------------------------------------------------
 * Task Display Loop Performance Tracking.
 */
static unsigned long display_loop_counter = 0;
static unsigned long display_loop_last_time = 0;

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
uint8_t led_color_values[MAX_INDICATORS][MAX_LED_COLOR_VALUES] = {}; // {R,G,B}

void UpdateIndicator(int index_led, CRGB color) {
  leds[index_led] = color; FastLED.show();
}

void UpdateAllIndicators(int start, int end, CRGB color) {
  for (int i=start; i<=end; i++) {color; FastLED.show();}
}

/** ----------------------------------------------------------------------------
 * requestEvent.
 */
volatile int request_event_id=0;

void requestEvent() {
  // --------------------------
  // brightness level
  // --------------------------
  if (request_event_id==301) {
    memset(I2CLink.TMP_BUFFER, 0, sizeof(I2CLink.TMP_BUFFER));
    strcpy(I2CLink.TMP_BUFFER, "$DATA,");
    strcat(I2CLink.TMP_BUFFER, String(brightness_stage).c_str());
    Serial.printf("[requestEvent] data: %s\n", I2CLink.TMP_BUFFER);
    memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
    for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++)
    {I2CLink.OUTPUT_BUFFER[i]=(byte)I2CLink.TMP_BUFFER[i];}
    Wire1.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
  }
}

/** ----------------------------------------------------------------------------
 * receiveEvent.
 */
int     display_index;
int     value_idx;
char    value_char[32];
int     dtype;
long    dx;
long    dy;
uint8_t colorR;
uint8_t colorG;
uint8_t colorB;

void receiveEvent(int howMany) {
  int len = Wire1.readBytes((char *)I2CLink.INPUT_BUFFER, howMany);
  if (len < 1) return;
  // Serial.printf("[RX] %s\n", I2CLink.INPUT_BUFFER);
  // -----------------------------------------------------
  // Initialize Parser Data
  // -----------------------------------------------------
  I2CLink.i_token = 0;
  I2CLink.token   = strtok(I2CLink.INPUT_BUFFER, ",");
  // -----------------------------------------------------
  // Request Events
  // -----------------------------------------------------
  if (strcmp(I2CLink.INPUT_BUFFER, "301")==0) {request_event_id=301;}

  else {
    dtype = atoi(I2CLink.token);
    // -----------------------------------------------------
    // Parse Command: Draw Canvas
    // -----------------------------------------------------
    if (dtype==101) {
      while (I2CLink.token != NULL) {
        switch (I2CLink.i_token) {
            case 1: ssd1306_displays[atoi(I2CLink.token)].draw=true; break;
        }
        I2CLink.token = strtok(NULL, ",");
        I2CLink.i_token = I2CLink.i_token + 1;
      }
    }
    else {
      // -----------------------------------------------------
      // Parse Command: Addressable LEDs
      // -----------------------------------------------------
      if (dtype==0) {
        int i_san = 0;
        while (I2CLink.token != NULL) {
          switch (I2CLink.i_token) {
              case 1: display_index = atoi(I2CLink.token); i_san++; break;
              case 2: colorR        = atoi(I2CLink.token); i_san++; break;
              case 3: colorG        = atoi(I2CLink.token); i_san++; break;
              case 4: colorB        = atoi(I2CLink.token); i_san++; break;
          }
          I2CLink.token = strtok(NULL, ",");
          I2CLink.i_token = I2CLink.i_token + 1;
        }
        // Serial.printf("[RX] led %d: r=%d g=%d b=%d sanitized=%d/4\n",
        //               display_index,
        //               colorR,
        //               colorG,
        //               colorB,
        //               i_san
        //             );
        if (i_san==4) {
          led_color_values[display_index][INDEX_LED_COLOR_VALUE_RED]   = colorR;
          led_color_values[display_index][INDEX_LED_COLOR_VALUE_GREEN] = colorG;
          led_color_values[display_index][INDEX_LED_COLOR_VALUE_BLUE]  = colorB;
        }
      }
      // -----------------------------------------------------
      // Parse Command: 7 Segment Display Data
      // -----------------------------------------------------
      else if (dtype >= 1 && dtype <=5) {
        int i_san = 0;
        while (I2CLink.token != NULL) {
          switch (I2CLink.i_token) {
              case 1: display_index = atoi(I2CLink.token); i_san++; break;
              case 2: memset(value_char, 0, sizeof(value_char)); strcpy(value_char, I2CLink.token); i_san++; break;
          }
          I2CLink.token = strtok(NULL, ",");
          I2CLink.i_token = I2CLink.i_token + 1;
        }
        // Serial.printf("[RX] 7seg %d: value=%s sanitized=%d/2\n",
        //               display_index,
        //               value_char,
        //               i_san
        //             );
        if (i_san==2) {
          auto& disp = seven_seg_displays[display_index];
          memset(disp.value, 0, sizeof(disp.value));
          strcpy(disp.value, value_char);
        }
      }
      // -----------------------------------------------------
      // Parse Command: SSD1306 Display Data
      // -----------------------------------------------------
      else if (dtype == 6) {
        int i_san = 0;
        while (I2CLink.token != NULL) {
          switch (I2CLink.i_token) {
              case 1: display_index = atoi(I2CLink.token); i_san++; break;
              case 2: value_idx     = atoi(I2CLink.token); i_san++; break;
              case 3: dx            = atoi(I2CLink.token); i_san++; break;
              case 4: dy            = atoi(I2CLink.token); i_san++; break;
              case 5:
                memset(value_char, 0, sizeof(value_char));
                strcpy(value_char, I2CLink.token);
                i_san++;
                break;
          }
          I2CLink.token = strtok(NULL, ",");
          I2CLink.i_token = I2CLink.i_token + 1;
        }
        // Serial.printf("[RX] SSD1306 %d: value_index=%d dx=%d dy%d value=%s sanitized=%d/5\n",
        //               display_index,
        //               value_idx,
        //               dx,
        //               dy,
        //               value_char,
        //               i_san
        //             );
        if (i_san==5) {
          ssd1306_displays[display_index].dx[value_idx] = dx;
          ssd1306_displays[display_index].dy[value_idx] = dy;
          memset(ssd1306_displays[display_index].value[value_idx],
                 0,
                 sizeof(ssd1306_displays[display_index].value[value_idx]));
          strcpy(ssd1306_displays[display_index].value[value_idx], value_char);
        }
      }
    }
  }
}

/** ----------------------------------------------------------------------------
 * taskDisplay.
 * 
 * @brief Update Display(s) & Any Indicators.
 */
void taskDisplay(void * pvParameters) {
  // while (global_task_sync==false) {vTaskDelay(1);}
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
      Serial.printf("[Brightness level] %d\n", brightness_stage);
      // -----------------------------------------------------
      // LEDs: Adjust Brightness
      // -----------------------------------------------------
      // clear
      // -----------------------
      for (int i_display=0; i_display<MAX_INDICATORS; i_display++) {
        FastLED.setBrightness(LED_BRIGHTNESS_LEVELS[brightness_stage]);
        FastLED.show();
      }
      // -----------------------------------------------------
      // 7 Segment Displays: Adjust Brightness
      // -----------------------------------------------------
      // clear
      // -----------------------
      if (brightness_stage==0) {
        for (int i_display=0; i_display<MAX_7SEG_DISPLAYS; i_display++) {
          auto& disp = seven_seg_displays[i_display];
          setADMultiplexerChannel(ad_mux_0, i_display);
          setADMultiplexerChannel(ad_mux_1, i_display);
          if (disp.type == SEG_4DIGIT) {disp.display4->clear();}
          else if (disp.type == SEG_6DIGIT) {disp.display6->clear();}
        }
      }
      // -----------------------
      // set brightness
      // -----------------------
      else {
        for (int i_display=0; i_display<MAX_7SEG_DISPLAYS; i_display++) {
          auto& disp = seven_seg_displays[i_display];
          setADMultiplexerChannel(ad_mux_0, i_display);
          setADMultiplexerChannel(ad_mux_1, i_display);
          if (disp.type == SEG_4DIGIT)
            {disp.display4->setBrightness(SEG_BRIGHTNESS_LEVELS[brightness_stage]);}
          else if (disp.type == SEG_6DIGIT)
            {disp.display6->setBrightness(SEG_BRIGHTNESS_LEVELS[brightness_stage]);}
        }
      }
      // -----------------------------------------------------
      // SSD1306: Adjust Brightness
      // -----------------------------------------------------
      // clear
      // -----------------------
      if (brightness_stage==0) {
        for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
          auto& disp = ssd1306_displays[i_display];
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
          if (disp.type==SSD_128X32) {
            disp.canvas32->clear();
            disp.display32->drawCanvas(0, 0, *disp.canvas32);
          }
          else if (disp.type==SSD_128X64) {
            disp.canvas64->clear();
            disp.display64->drawCanvas(0, 0, *disp.canvas64);
          }
        }
      }
      // -----------------------
      // set brightness
      // -----------------------
      else {
        for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
          auto& disp = ssd1306_displays[i_display];
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
          if (disp.type==SSD_128X32) {
            // send brightness/contrast command if supported
          }
          else if (disp.type==SSD_128X64) {
            // send brightness/contrast command if supported
          }
        }
      }
      interruptMaster();
    }
    // -----------------------------------------------------
    // Update Displays
    // -----------------------------------------------------
    if (!brightness_stage==0) {
      // -----------------------------------------------------
      // Update Indicators
      // -----------------------------------------------------
      for (int i_display=0; i_display<MAX_INDICATORS; i_display++) {
        // Serial.printf("[LED] display_index=%d r=%d g=%d b=%d\n",
        //               i_display,
        //               led_color_values[i_display][INDEX_LED_COLOR_VALUE_RED],
        //               led_color_values[i_display][INDEX_LED_COLOR_VALUE_GREEN],
        //               led_color_values[i_display][INDEX_LED_COLOR_VALUE_BLUE]
        //             );
        UpdateIndicator(i_display,
                        CRGB(led_color_values[i_display][INDEX_LED_COLOR_VALUE_RED],
                            led_color_values[i_display][INDEX_LED_COLOR_VALUE_GREEN],
                            led_color_values[i_display][INDEX_LED_COLOR_VALUE_BLUE]));
      }
      // -----------------------------------------------------
      // Update I2C Display(s)
      // -----------------------------------------------------
      for (int i_display=0; i_display<MAX_SSD1306_DISPLAYS; i_display++) {
        auto& disp = ssd1306_displays[i_display];
        if (disp.draw==true) {
          disp.draw=false;
          setI2CMultiplexChannel(Wire, i2c_mux_0, i_display);
          if (disp.type==SSD_128X32) {
            disp.canvas32->clear();
            for (int i_value=0; i_value<MAX_SSD1306_DISPLAY_VALUES; i_value++) {
              // Serial.printf("[SSD_128X32] display_index=%d type=%d value_index=%d dx=%d dy=%d value=%s\n",
              //               i_display,
              //               disp.type,
              //               i_value,
              //               disp.dx[i_value],
              //               disp.dy[i_value],
              //               disp.value[i_value]
              //             );
              disp.canvas32->printFixed(disp.dx[i_value], disp.dy[i_value], disp.value[i_value], STYLE_BOLD);
            }
            disp.display32->drawCanvas(0, 0, *disp.canvas32);
          }
          else if (disp.type==SSD_128X64) {
            disp.canvas64->clear();
            for (int i_value=0; i_value<MAX_SSD1306_DISPLAY_VALUES; i_value++) {
              // Serial.printf("[SSD_128X64] display_index=%d type=%dvalue_index=%d dx=%d dy=%d value=%s\n",
              //               i_display,
              //               disp.type,
              //               i_value,
              //               disp.dx[i_value],
              //               disp.dy[i_value],
              //               disp.value[i_value]
              //             );
              disp.canvas64->printFixed(disp.dx[i_value], disp.dy[i_value], disp.value[i_value], STYLE_BOLD);
            }
            disp.display64->drawCanvas(0, 0, *disp.canvas64);
          }

        }
      }
      // -----------------------------------------------------
      // Update Analog/Digital Display(s)
      // -----------------------------------------------------
      for (uint8_t i_display = 0; i_display < MAX_7SEG_DISPLAYS; i_display++) {
        auto& disp = seven_seg_displays[i_display];
        // if (disp.value == disp.prev_value) continue;
        setADMultiplexerChannel(ad_mux_0, i_display);
        setADMultiplexerChannel(ad_mux_1, i_display);
        if (disp.type == SEG_4DIGIT) {
          // Serial.printf("[SEG_4DIGIT] display_index=%d type=%d value=%s\n",
          //               i_display,
          //               disp.type,
          //               disp.value
          //             );
          disp.display4->showString(disp.value);
        }
        else if (disp.type == SEG_6DIGIT) {
          // Serial.printf("[SEG_6DIGIT] display_index=%d type=%d value=%s\n",
          //               i_display,
          //               disp.type,
          //               disp.value
          //             );
          disp.display6->showString(disp.value);
        }
        // memset(disp.prev_value, 0, sizeof(disp.prev_value));
        // strcpy(disp.prev_value, disp.value);
      }
    }
  
    // -----------------------------------------------------
    // Track and print loops per second
    // -----------------------------------------------------
    display_loop_counter++;
    unsigned long current_time = millis();
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
    TASK_DISPLAY_PRIORITY,   /* Priority of the task */
    &TaskDisplay,            /* Task handle. */
    TASK_DISPLAY_CORE);      /* Core where the task should run */
    esp_task_wdt_add(TaskDisplay);
}

/** ----------------------------------------------------------------------------
 * Setup.
 */
void setup() {
  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // ------------------------------------------------------------
  // Interrupts
  // ------------------------------------------------------------
  // Send interrupt to master to indicate data is ready
  pinMode(MASTER_INTERRUPT_PIN, OUTPUT);
  digitalWrite(MASTER_INTERRUPT_PIN, HIGH);
  // Button to iterate brightness level
  pinMode(ISR_PIN_BRIGHTNESS, INPUT);
  attachInterruptArg(digitalPinToInterrupt(ISR_PIN_BRIGHTNESS), iter_brightness_ISR, NULL, RISING);

  // ------------------------------------------------------------
  // Indicators
  // ------------------------------------------------------------
  FastLED.addLeds<NEOPIXEL, INDICATOR_DIO>(leds, MAX_INDICATORS);
  FastLED.setBrightness(255); // 0–255, adjust as needed
  UpdateAllIndicators(0, MAX_INDICATORS, CRGB::Black);
  delay(1000);                         // test clear
  UpdateIndicator(0, CRGB::Red);       // test built-in color
  UpdateIndicator(9, CRGB(0,0,255)); // test arbitrary color

  // ------------------------------------------------------------
  // I2C Master Initialization
  // ------------------------------------------------------------
  Wire.setPins(MASTER_SDA, MASTER_SCL);
  if (!Wire.begin())
    {Serial.printf("[I2C] master failed to start (SDA=%d SCL=%d)\n",
                   MASTER_SDA,
                   MASTER_SCL
                  );}
  else
    {Serial.printf("[I2C] master started sucessfully (SDA=%d SCL=%d)\n",
                   MASTER_SDA,
                   MASTER_SCL
                  );}
  Wire.setTimeOut(1000);
  
  // ------------------------------------------------------------
  // I2C Slave Initialization
  // ------------------------------------------------------------
  Wire1.setPins(SLAVE_SDA, SLAVE_SCL);
  if (!Wire1.begin(SLAVE_ADDR))
    {Serial.printf("[I2C] slave failed to start on address 0x%02X (SDA=%d SCL=%d)\n",
                   SLAVE_ADDR,
                   SLAVE_SDA,
                   SLAVE_SCL
                  );}
  else
    {Serial.printf("[I2C] slave started sucessfully on address 0x%02X (SDA=%d SCL=%d)\n",
                   SLAVE_ADDR,
                   SLAVE_SDA,
                   SLAVE_SCL
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
  pinMode(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT); // DIO pins
  pinMode(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT); // CLK pins
  digitalWrite(ad_mux_0.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], LOW);
  digitalWrite(ad_mux_1.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], LOW);

  // ------------------------------------------------------------
  // Initialize 7-Segment Display(s)
  // ------------------------------------------------------------
  Serial.println("[Initializing Analog/Digital Multiplexer Displays]");
  for (int i_display=0; i_display<MAX_7SEG_DISPLAYS; i_display++) {
    auto& disp = seven_seg_displays[i_display];
    setADMultiplexerChannel(ad_mux_0, i_display);
    setADMultiplexerChannel(ad_mux_1, i_display);
    if (disp.type == SEG_4DIGIT) {
      Serial.printf("  Display %d: type=%d\n", i_display, disp.type);
      disp.display4->clear();
      disp.display4->setBrightness(7);
      disp.display4->showString("----");
    }
    else if (disp.type == SEG_6DIGIT) {
      Serial.printf("  Display %d: type=%d\n", i_display, disp.type);
      disp.display6->clear();
      disp.display6->setBrightness(7);
      disp.display6->showString("------");
    }
  }

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
      disp.canvas32->printFixed(1, 1, "starting", STYLE_BOLD);
      disp.display32->drawCanvas(0, 0, *disp.canvas32);
      delay(1000);
    }
    else if (disp.type==SSD_128X64) {
      disp.display64->begin();
      disp.display64->clear();
      // test canvas
      disp.canvas64->setFixedFont(disp.font);
      disp.canvas64->clear();
      disp.canvas64->printFixed(1, 1, "starting", STYLE_BOLD);
      disp.display64->drawCanvas(0, 0, *disp.canvas64);
      // delay(1000);
    }
  }

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
  Wire1.onRequest(requestEvent);
  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire1.onReceive(receiveEvent);
}

/** ----------------------------------------------------------------------------
 * Loop.
 * 
 * @brief Update Display(s) & Any Indicators.
 */

void loop() {
}