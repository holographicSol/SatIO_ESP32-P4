/*
    General Multi Display Controller - Written By Benjamin Jack Cullen.

    Requires IICMultiDisplay module atached.
*/

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "multi_display_controller.h"
#include "satio.h"
#include "wtgps300p.h"
#include "task_handler.h"
#include "freertos/semphr.h"
#include "i2c_helper.h"

bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0    = false;
bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0 = true;

// bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_1    = false; // example second controller.
// bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_1 = true;  // example second controller...

IICLink IICLinkMultiDisplayController; // IIC link data structure for Multi Display Controller

/* Instruction Structure: DisplayType,DisplayIndex,R,G,B */
void updateIndicator(TwoWire &wire, int address, int display_index, int r, int g, int b) {
    clearI2CLinkOutputChars(IICLinkMultiDisplayController);
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_ADDRESSABLE_LEDS).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(r).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(g).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(b).c_str());
    // Serial.printf("[updateIndicator] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2CToSlaveChars(wire, IICLinkMultiDisplayController, address, 4, "updateIndicator");
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment4Digit(TwoWire &wire, int address, int display_index, char* value) {
    clearI2CLinkOutputChars(IICLinkMultiDisplayController);
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_4DIGIT).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment4Digit] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2CToSlaveChars(wire, IICLinkMultiDisplayController, address, 4, "update7Segment4Digit");
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment6Digit(TwoWire &wire, int address, int display_index, char* value) {
    clearI2CLinkOutputChars(IICLinkMultiDisplayController);
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_6DIGIT).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment6Digit] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2CToSlaveChars(wire, IICLinkMultiDisplayController, address, 4, "update7Segment6Digit");
}

/* Instruction Structure: DisplayType,DisplayIndex,ValueIndex,Dx,Dy,Value */
void updateSSD1306(TwoWire &wire, int address, int display_index, int value_index, int dx, int dy, char* value) {
    clearI2CLinkOutputChars(IICLinkMultiDisplayController);
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_SSD1306).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(value_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(dx).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(dy).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(value).c_str());
    // Serial.printf("[updateSSD1306] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2CToSlaveChars(wire, IICLinkMultiDisplayController, address, 4, "updateSSD1306");
}

/* Instruction Structure: EventID,DisplayIndex */
void drawSSD1306Canvas(TwoWire &wire, int address, int display_index) {
  clearI2CLinkOutputChars(IICLinkMultiDisplayController);
  strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, "101,");
  strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
  writeI2CToSlaveChars(wire, IICLinkMultiDisplayController, address, 4, "drawSSD1306Canvas");
}

/* Request data from I2C Multi Display Module if interrupt received */
void requestMultiDisplayControllerData(TwoWire &wire, int address, bool &isr_interrupt_flag) {
 if (isr_interrupt_flag==true) {
   isr_interrupt_flag=false;
   Serial.printf("[ISR FLAG] requestMultiDisplayControllerData\n");
   // Send 301 instruction and read requested response
   requestFromSlaveChars(wire, IICLinkMultiDisplayController, address, 301, 5, 0, "requestMultiDisplayControllerData");
   for (int i = 0; i<I2C_MAX_TOKENS; i++) {
     // Serial.printf("[TKN] %d: %s\n", IICLinkMultiDisplayController.i_token, IICLinkMultiDisplayController.token);
     if      (i==0 && strcmp(IICLinkMultiDisplayController.TOKENS[i], "301")!=0) {Serial.printf("[I2C] Warning: Invalid response ID from Multi Display Controller: %s\n", IICLinkMultiDisplayController.TOKENS[i]); break;}
     else if (i==1 && str_is_bool(IICLinkMultiDisplayController.TOKENS[i])) {ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0=atoi(IICLinkMultiDisplayController.TOKENS[i]); break;}
    }
    // Send other requests as required...
  }
}

/* ISR for request events. Must be attached to pin somewhere, like main setup */
void ISR_MultiDisplayController_0(void * arg) {ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0=true;}
// example second ISR for second controller:
// void IRAM_ATTR ISR_MultiDisplayController_1(void * arg) {ISR_MULTI_DISPLAY_CONTROLLER_1=true;}