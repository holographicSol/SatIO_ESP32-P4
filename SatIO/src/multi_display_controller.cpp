// /*
//     General Multi Display Controller - Written By Benjamin Jack Cullen.

//     Requires IICMultiDisplay module atached.
// */

// #include <Arduino.h>
// #include <Wire.h>
// #include "config.h"
// #include "multi_display_controller.h"
// #include "satio.h"
// #include "wtgps300p.h"
// #include "task_handler.h"
// #include "freertos/semphr.h"
// #include "i2c_helper.h"

// bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0    = false;
// bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0 = true;

// // bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_1    = false; // example second controller.
// // bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_1 = true;  // example second controller...

// // IICLink IICLinkMultiDisplayController; // IIC link data structure for Multi Display Controller

// /* Instruction Structure: DisplayType,DisplayIndex,R,G,B */
// void updateIndicator(TwoWire &wire, int address, int display_index, int r, int g, int b) {
//   // clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 0, 0x0A); // command 10
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 1, display_index);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 2, r);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 3, g);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 4, b);
//   writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 5, 5, "updateIndicator");
// }

// /* Instruction Structure: DisplayType,DisplayIndex,Value */
// void update7Segment4Digit(TwoWire &wire, int address, int display_index, char* value) {
//   clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 0, 0x14); // command 20
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 1, display_index);
//   write_nchars_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 2, value, strlen(value));
//   writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 2+strlen(value), 5, "update7Segment4Digit");
// }

// /* Instruction Structure: DisplayType,DisplayIndex,Value */
// void update7Segment6Digit(TwoWire &wire, int address, int display_index, char* value) {
//   clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 0, 0x1E); // command 30
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 1, display_index);
//   write_nchars_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 2, value, strlen(value));
//   writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 2+strlen(value), 5, "update7Segment6Digit");
// }

// /* Instruction Structure: DisplayType,DisplayIndex,ValueIndex,Dx,Dy,Value */
// void updateSSD1306(TwoWire &wire, int address, int display_index, int value_index, int dx, int dy, char* value) {
//   clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 0, 0x28 ); // command 40
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 1, display_index);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 2, value_index);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 3, dx);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 4, dy);
//   write_nchars_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 5, value, strlen(value));
//   writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 5+strlen(value), 5, "updateSSD1306");
// }

// /* Instruction Structure: EventID,DisplayIndex */
// void drawSSD1306Canvas(TwoWire &wire, int address, int display_index) {
//   clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 0, 0x32 ); // command 50
//   write_uint8_ToPacket(IICLinkMultiDisplayController.OUTPUT_PACKET, 1, display_index);
//   writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 2, 5, "drawSSD1306Canvas");
// }

// /* Request data from I2C Multi Display Module if interrupt received */
// void requestMultiDisplayControllerData(TwoWire &wire, int address, bool &isr_interrupt_flag) {
//  if (isr_interrupt_flag==true) {
//    isr_interrupt_flag=false;
//    Serial.printf("[ISR FLAG] requestMultiDisplayControllerData\n");

//    // Send event ID
//    clearI2CLinkOutputPacket(IICLinkMultiDisplayController);
//    IICLinkMultiDisplayController.OUTPUT_PACKET[0] = 0x01; // 1
//    writeI2CToSlaveBin(wire, IICLinkMultiDisplayController, address, 1, 5, "requestMultiDisplayControllerData");

//    // Request from slave
//    if (requestFromSlaveBinNoID(wire,
//                                IICLinkMultiDisplayController,
//                                address,
//                                1,
//                                0,
//                                "requestMultiDisplayControllerData")) {
//      uint8_t brightness;
//      read_uint8_FromWire(wire, brightness);
//      if (brightness==0) {ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0=false;}
//      else {ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0=true;}
//      Serial.printf("[BIN] %d\n", brightness);
//    }
//   }
// }

// /* ISR for request events. Must be attached to pin somewhere, like main setup */
// void ISR_MultiDisplayController_0(void * arg) {ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0=true;}
// // example second ISR for second controller:
// // void IRAM_ATTR ISR_MultiDisplayController_1(void * arg) {ISR_MULTI_DISPLAY_CONTROLLER_1=true;}
