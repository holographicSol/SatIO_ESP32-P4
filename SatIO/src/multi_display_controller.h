// /*
//     General Multi Display Controller - Written By Benjamin Jack Cullen.

//     Requires IICMultiDisplay module atached.
// */

// #ifndef MULTI_DISPLAY_CONTROLLER_H
// #define MULTI_DISPLAY_CONTROLLER_H

// #include <stdint.h>
// #include <stdbool.h>

// #define ISR_PIN_MULTIDISPLAY_CONTROLLER_0     47 // change as required.
// #define I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0 12 // change and add more as required.

// // #define ISR_PIN_MultiDisplayController_1      46 // example add second controller.
// // #define I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_1 13 // example add second controller.

// extern bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0;
// extern bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0;

// // extern bool ISR_FLAG_MULTI_DISPLAY_CONTROLLER_1;    // example add second controller.
// // extern bool ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_1; // example add second controller.

// #define DISPLAY_TYPE_ADDRESSABLE_LEDS 0
// #define DISPLAY_TYPE_7SEGMENT_1DIGIT  1
// #define DISPLAY_TYPE_7SEGMENT_2DIGIT  2
// #define DISPLAY_TYPE_7SEGMENT_4DIGIT  3
// #define DISPLAY_TYPE_7SEGMENT_6DIGIT  4
// #define DISPLAY_TYPE_7SEGMENT_8DIGIT  5
// #define DISPLAY_TYPE_SSD1306          6

// /**
//  * @brief Control addressable RGB LED(s).
//  * @param wire I2C bus instance.
//  * @param address I2C address of Multi Display Controller.
//  * @param display_index Specify target LED number.
//  * @param r Specify value red between 0-255.
//  * @param g Specify value green between 0-255.
//  * @param b Specify value blue between 0-255.
//  */
// void updateIndicator(TwoWire &wire, int address, int display_index, int r, int g, int b);

// /**
//  * @brief Control 7 segment 4 digit display.
//  * @param address I2C address of Multi Display Controller.
//  * @param display_index Specify display index (0-15).
//  * @param value Specify value to be displayed.
//  */
// void update7Segment4Digit(TwoWire &wire, int address, int display_index, char* value);

// /**
//  * @brief Control 7 segment 6 digit display.
//  * @param wire I2C bus instance.
//  * @param address I2C address of Multi Display Controller.
//  * @param display_index Specify display index (0-15).
//  * @param value Specify value to be displayed.
//  */
// void update7Segment6Digit(TwoWire &wire, int address, int display_index, char* value);

// /**
//  * @brief Writes to a canvas for SSD1306 display.
//  * @param wire I2C bus instance.
//  * @param address I2C address of Multi Display Controller.
//  * @param display_index Specify display index (0-7).
//  * @param value_index Specify value index.
//  * @param dx Specify value x position on canvas.
//  * @param dy Specify value y position on canvas.
//  * @param value Specify value to be displayed.
//  */
// void updateSSD1306(TwoWire &wire, int address, int display_index, int value_index, int dx, int dy, char* value);

// /**
//  * @brief Draws canvas on SSD1306 display.
//  * @param wire I2C bus instance.
//  * @param address I2C address of Multi Display Controller.
//  * @param display_index Specify display index (0-7).
//  */
// void drawSSD1306Canvas(TwoWire &wire, int address, int display_index);

// /**
//  * @brief Draws canvas on SSD1306 display.
//  * @param wire I2C bus instance.
//  * @param address I2C address of Multi Display Controller.
//  * @param isr_interrupt_flag Specify interrupt flag to check.
//  */
// void requestMultiDisplayControllerData(TwoWire &wire, int address, bool &isr_interrupt_flag);

// /**
//  * @brief Interrupt sub-routine to trigger data request.
//  * @details Takes no paramters.
//  * 
//  */
// void IRAM_ATTR ISR_MultiDisplayController_0(void * arg);

// #endif // MULTI_DISPLAY_CONTROLLER_H