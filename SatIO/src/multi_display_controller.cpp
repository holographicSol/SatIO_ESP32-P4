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

bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0    = false;
bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0 = true;

// bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_1    = false; // example second controller.
// bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_1 = true;  // example second controller...

IICLink IICLinkMultiDisplayController; // IIC link data structure for Multi Display Controller

/* Clear output buffer chars */
void clearOutputBufferChars() {
  memset(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, 0, sizeof(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS));
}

/* Clear output buffer bytes */
void clearOutputBufferBytes() {
  memset(IICLinkMultiDisplayController.OUTPUT_BUFFER_BYTES, 0, sizeof(IICLinkMultiDisplayController.OUTPUT_BUFFER_BYTES));
}

/* Clear intput buffer chars */
void clearInputBufferChars() {
  memset(IICLinkMultiDisplayController.INPUT_BUFFER, 0, sizeof(IICLinkMultiDisplayController.INPUT_BUFFER));
}

/* Check output bytes less than MAX_IIC_BUFFER_SIZE */
bool isOuptutBufferBytesLessThanMaxIICBufferSize() {
  if (strlen(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS)<MAX_IIC_BUFFER_SIZE) {return true;}
  return false;
}

/* Converts chars array to bytes array and transmits */
void writeI2C(TwoWire &wire, int address) {
  // Serial.printf("[writeI2C] display\n");
  if (!isOuptutBufferBytesLessThanMaxIICBufferSize()) {
    Serial.printf("[writeI2C] bytes exceed MAX_IIC_BUFFER_SIZE (%d bytes)\n", MAX_IIC_BUFFER_SIZE);
    return;
  }
  clearOutputBufferBytes();
  int len = strlen(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
  for (int i=0;i<len;i++)
  {IICLinkMultiDisplayController.OUTPUT_BUFFER_BYTES[i]=(byte)IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS[i];}
  wire.beginTransmission(address);
  wire.write(IICLinkMultiDisplayController.OUTPUT_BUFFER_BYTES, len);
  wire.endTransmission();
  delay(4); // Allow some time for IIC module to process data.
}

/* Instruction Structure: DisplayType,DisplayIndex,R,G,B */
void updateIndicator(TwoWire &wire, int address, int display_index, int r, int g, int b) {
    clearOutputBufferChars();
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
    writeI2C(wire, address);
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment4Digit(TwoWire &wire, int address, int display_index, char* value) {
    clearOutputBufferChars();
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_4DIGIT).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment4Digit] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2C(wire, address);
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment6Digit(TwoWire &wire, int address, int display_index, char* value) {
    clearOutputBufferChars();
    strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_6DIGIT).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment6Digit] %s\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS);
    writeI2C(wire, address);
}

/* Instruction Structure: DisplayType,DisplayIndex,ValueIndex,Dx,Dy,Value */
void updateSSD1306(TwoWire &wire, int address, int display_index, int value_index, int dx, int dy, char* value) {
    clearOutputBufferChars();
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
    writeI2C(wire, address);
}

/* Instruction Structure: EventID,DisplayIndex, */
void drawSSD1306Canvas(TwoWire &wire, int address, int display_index) {
  clearOutputBufferChars();
  strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, "101,");
  strcat(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
  writeI2C(wire, address);
}

/* Request data from I2C Multi Display Module(s) */
void requestMultiDisplayControllerData(TwoWire &wire, int address, bool &isr_interrupt_flag) {

 if (isr_interrupt_flag==true) {
   isr_interrupt_flag=false;
   Serial.printf("[ISR FLAG] requestMultiDisplayControllerData\n");
   
   // Send 301 instruction
   clearOutputBufferChars();
   strcpy(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, "301");
   if (strlen(IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS)<MAX_IIC_BUFFER_SIZE) {writeI2C(wire, address);}
   
   // Read old value
   clearInputBufferChars();
   if (wire.requestFrom(address, sizeof(IICLinkMultiDisplayController.INPUT_BUFFER)) != sizeof(IICLinkMultiDisplayController.INPUT_BUFFER)) {
     // Serial.printf("[I2C] error requesting data %d\n");
    }
    else {
      uint8_t len = wire.readBytes(IICLinkMultiDisplayController.INPUT_BUFFER, sizeof(IICLinkMultiDisplayController.INPUT_BUFFER));
      // Serial.printf("[MASTER RX] %s (%d bytes)\n", IICLinkMultiDisplayController.OUTPUT_BUFFER_CHARS, len);
    }

    delay(1);

    // Read new value
    clearInputBufferChars();
    if (wire.requestFrom(address, sizeof(IICLinkMultiDisplayController.INPUT_BUFFER)) != sizeof(IICLinkMultiDisplayController.INPUT_BUFFER)) {
      Serial.printf("[I2C] error requesting data %d\n");
    }
    else {
      uint8_t len = wire.readBytes(IICLinkMultiDisplayController.INPUT_BUFFER, sizeof(IICLinkMultiDisplayController.INPUT_BUFFER));
      Serial.printf("[MASTER RX] %s (%d bytes)\n", IICLinkMultiDisplayController.INPUT_BUFFER, len);
      if (len < 1) return;
      IICLinkMultiDisplayController.i_token = 0;
      IICLinkMultiDisplayController.token = strtok(IICLinkMultiDisplayController.INPUT_BUFFER, ",");
      while (IICLinkMultiDisplayController.token != NULL) {
        // Serial.printf("[TKN] %d: %s\n", IICLinkMultiDisplayController.i_token, IICLinkMultiDisplayController.token);
        switch (IICLinkMultiDisplayController.i_token) {
          case 1: if (str_is_bool(IICLinkMultiDisplayController.token)) {ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0=atoi(IICLinkMultiDisplayController.token); break;}
        }
        IICLinkMultiDisplayController.token = strtok(NULL, ",");
        IICLinkMultiDisplayController.i_token = IICLinkMultiDisplayController.i_token + 1;
      }
    }
  }
}

/* ISR for request events. Must be attached to pin somewhere, like main setup */
void ISR_MultiDisplayController_0(void * arg) {ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0=true;}
// example second ISR for second controller:
// void IRAM_ATTR ISR_MultiDisplayController_1(void * arg) {ISR_MULTI_DISPLAY_CONTROLLER_1=true;}