/*
    HexToDig Library. Written by Benjamin Jack Cullen.

    A small library for converting hex to digits.
*/

#ifndef HEX_TO_DIG_H
#define HEX_TO_DIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
uint8_t h2d(char hex);
uint8_t h2d2(char h1, char h2);

#ifdef __cplusplus
}
#endif

#endif