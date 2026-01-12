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

bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0    = false;
bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0 = true;

// bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_1    = false; // example second controller.
// bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_1 = true;  // example second controller.

/* Clear output buffer chars */
void clearOutputBufferChars() {
  memset(IICLink0.OUTPUT_BUFFER_CHARS, 0, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
}

/* Clear output buffer bytes */
void clearOutputBufferBytes() {
  memset(IICLink0.OUTPUT_BUFFER_BYTES, 0, sizeof(IICLink0.OUTPUT_BUFFER_BYTES));
}

/* Check output bytes less than MAX_IIC_BUFFER_SIZE */
bool isOuptutBufferBytesLessThanMaxIICBufferSize(int address) {
  if (strlen(IICLink0.OUTPUT_BUFFER_CHARS)<MAX_IIC_BUFFER_SIZE) {return true;}
  return false;
}

/* Converts chars array to bytes array */
void writeI2C(int address) {
  if (!isOuptutBufferBytesLessThanMaxIICBufferSize(address)) {
    Serial.printf("[writeI2C] bytes exceed MAX_IIC_BUFFER_SIZE (%d bytes)\n", MAX_IIC_BUFFER_SIZE);
    return;
  }
  clearOutputBufferBytes();
  for (byte i=0;i<sizeof(IICLink0.OUTPUT_BUFFER_BYTES);i++)
  {IICLink0.OUTPUT_BUFFER_BYTES[i]=(byte)IICLink0.OUTPUT_BUFFER_CHARS[i];}
  iic_0.beginTransmission(address);
  iic_0.write(IICLink0.OUTPUT_BUFFER_BYTES, sizeof(IICLink0.OUTPUT_BUFFER_BYTES));
  iic_0.endTransmission();
}

/* Instruction Structure: DisplayType,DisplayIndex,R,G,B */
void updateIndicator(int address, int display_index, int r, int g, int b) {
    clearOutputBufferChars();
    strcpy(IICLink0.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_ADDRESSABLE_LEDS).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(r).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(g).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(b).c_str());
    // Serial.printf("[updateIndicator] %s\n", IICLink0.OUTPUT_BUFFER_CHARS);
    writeI2C(address);
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment4Digit(int address, int display_index, char* value) {
    clearOutputBufferChars();
    strcpy(IICLink0.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_4DIGIT).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment4Digit] %s\n", IICLink0.OUTPUT_BUFFER_CHARS);
    writeI2C(address);
}

/* Instruction Structure: DisplayType,DisplayIndex,Value */
void update7Segment6Digit(int address, int display_index, char* value) {
    clearOutputBufferChars();
    strcpy(IICLink0.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_7SEGMENT_6DIGIT).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, value);
    // Serial.printf("[update7Segment6Digit] %s\n", IICLink0.OUTPUT_BUFFER_CHARS);
    writeI2C(address);
}

/* Instruction Structure: DisplayType,DisplayIndex,ValueIndex,Dx,Dy,Value */
void updateSSD1306(int address, int display_index, int value_index, int dx, int dy, char* value) {
    clearOutputBufferChars();
    strcpy(IICLink0.OUTPUT_BUFFER_CHARS, String(DISPLAY_TYPE_SSD1306).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(value_index).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(dx).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(dy).c_str());
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, ",");
    strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(value).c_str());
    // Serial.printf("[updateSSD1306] %s\n", IICLink0.OUTPUT_BUFFER_CHARS);
    writeI2C(address);
}

/* Instruction Structure: EventID,DisplayIndex, */
void drawSSD1306Canvas(int address, int display_index) {
  clearOutputBufferChars();
  strcpy(IICLink0.OUTPUT_BUFFER_CHARS, "101,");
  strcat(IICLink0.OUTPUT_BUFFER_CHARS, String(display_index).c_str());
  writeI2C(address);
}

/* Request data from I2C Multi Display Module(s) */
void requestMultiDisplayControllerData(int address, bool &isr_interrupt_flag) {

 if (isr_interrupt_flag==true) {
   isr_interrupt_flag=false;
   Serial.printf("[ISR] requestMultiDisplayControllerData\n");
   
   // Send 301 instruction
   memset(IICLink0.OUTPUT_BUFFER_CHARS, 0, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
   strcpy(IICLink0.OUTPUT_BUFFER_CHARS, "301");
   if (strlen(IICLink0.OUTPUT_BUFFER_CHARS)<MAX_IIC_BUFFER_SIZE) {writeI2C(address);}
   
   // Read old value
   memset(IICLink0.OUTPUT_BUFFER_CHARS, 0, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
   if (iic_0.requestFrom(address, sizeof(IICLink0.OUTPUT_BUFFER_CHARS)) != sizeof(IICLink0.OUTPUT_BUFFER_CHARS)) {
     // Serial.printf("[I2C] error requesting data %d\n");
    }
    else {
      uint8_t len = iic_0.readBytes(IICLink0.OUTPUT_BUFFER_CHARS, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
      // Serial.printf("[MASTER RX] %s (%d bytes)\n", OUTPUT_BUFFER_CHARS, len);
    }

    delay(1);

    // Read new value
    memset(IICLink0.OUTPUT_BUFFER_CHARS, 0, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
    if (iic_0.requestFrom(address, sizeof(IICLink0.OUTPUT_BUFFER_CHARS)) != sizeof(IICLink0.OUTPUT_BUFFER_CHARS)) {
      Serial.printf("[I2C] error requesting data %d\n");
    }
    else {
      uint8_t len = iic_0.readBytes(IICLink0.OUTPUT_BUFFER_CHARS, sizeof(IICLink0.OUTPUT_BUFFER_CHARS));
      Serial.printf("[MASTER RX] %s (%d bytes)\n", IICLink0.OUTPUT_BUFFER_CHARS, len);
      if (len < 1) return;
      IICLink0.i_token = 0;
      IICLink0.token = strtok(IICLink0.OUTPUT_BUFFER_CHARS, ",");
      while (IICLink0.token != NULL) {
        // Serial.printf("[TKN] %d: %s\n", IICLink0.i_token, IICLink0.token);
        switch (IICLink0.i_token) {
          case 1: if (str_is_bool(IICLink0.token)) {ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0=atoi(IICLink0.token); break;}
        }
        IICLink0.token = strtok(NULL, ",");
        IICLink0.i_token = IICLink0.i_token + 1;
      }
    }
  }
}

/* ISR for request events. Must be attached to pin somewhere, like main setup */
void ISR_MultiDisplayController_0(void * arg) {ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0=true;}
// example second ISR for second controller:
// void IRAM_ATTR ISR_MultiDisplayController_1(void * arg) {ISR_MULTI_DISPLAY_CONTROLLER_1=true;}