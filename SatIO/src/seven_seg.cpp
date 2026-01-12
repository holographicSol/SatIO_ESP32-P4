/*
    SevenSeg - Written By Benjamin Jack Cullen.
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
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
#include "./seven_seg.h"
#include <TM1637TinyDisplay6.h>       // Include 6-Digit Display lib https://github.com/jasonacox/TM1637TinyDisplay

TM1637TinyDisplay6 tm0(S0_SEGMENT_CLK_PIN, S0_SEGMENT_DIO_PIN); // 6-Digit Display Class
TM1637TinyDisplay6 tm1(S1_SEGMENT_CLK_PIN, S1_SEGMENT_DIO_PIN); // 6-Digit Display Class

void setupTM() {
    tm0.clear();
    tm1.clear();

    tm0.setBrightness(7);
    tm1.setBrightness(7);
    
    tm0.showString("------");
    tm1.showString("------");
    // delay(1000);
}

void S0_displayTimeHHMMSS(long value) {
    tm0.showNumberDec(value, (uint8_t)0b01010000, true, 6, (uint8_t)0U);
}

void S1_displayDateDDMMYY(long value) {
    tm1.showNumberDec(value, (uint8_t)0b01010000, true, 6, (uint8_t)0U);
}