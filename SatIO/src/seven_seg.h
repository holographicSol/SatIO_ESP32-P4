/*
    SevenSeg - Written By Benjamin Jack Cullen.
*/

#ifndef SEVEN_SEG_H
#define SEVEN_SEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define S0_SEGMENT_DIO_PIN 46
#define S0_SEGMENT_CLK_PIN 47

#define S1_SEGMENT_DIO_PIN 48
#define S1_SEGMENT_CLK_PIN 53

/** ----------------------------------------------------------------------------
 * Initialize display(s).
 * 
 * @brief Initializes 7-segment display(s).
 */
void setupTM();

/** ----------------------------------------------------------------------------
 * Display value.
 * 
 * @brief Display value on display.
 */
// void S0_displayValue(const char* value);
void S0_displayTimeHHMMSS(long value);
void S1_displayDateDDMMYY(long value);

#ifdef __cplusplus
}
#endif

#endif // SEVEN_SEG_H