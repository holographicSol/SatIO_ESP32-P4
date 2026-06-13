/*******************************************************************************
 * Size: 21 px
 * Bpp: 1
 * Opts: --bpp 1 --size 21 --no-compress --stride 1 --align 1 --font spaceranger.ttf --range 32-127 --format lvgl -o space_ranger_21.c
 ******************************************************************************/

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif



#ifndef SPACE_RANGER_21
#define SPACE_RANGER_21 1
#endif

#if SPACE_RANGER_21

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xff, 0xf8, 0xff, 0x80,

    /* U+0022 "\"" */
    0xef, 0xdf, 0xbe, 0x60,

    /* U+0023 "#" */
    0x38, 0xe1, 0xc7, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0x9c, 0x73, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xc7,
    0xe, 0x38,

    /* U+0024 "$" */
    0x7, 0x0, 0x38, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xf3, 0x83, 0xff, 0xdf, 0xff, 0x7f, 0xf8, 0x39,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0x80, 0x1c,
    0x0,

    /* U+0025 "%" */
    0xfe, 0x70, 0x7f, 0x38, 0x31, 0xb8, 0x1f, 0xdc,
    0xf, 0xfe, 0x0, 0xe, 0x0, 0xf, 0xfe, 0x7,
    0x7f, 0x3, 0xb1, 0x83, 0x9f, 0xc1, 0xcf, 0xe0,

    /* U+0026 "&" */
    0x1f, 0xe0, 0xff, 0x83, 0xfe, 0xe, 0x0, 0x7f,
    0xff, 0xff, 0xff, 0xff, 0xf8, 0xe, 0xff, 0xfb,
    0xff, 0xe7, 0xff, 0x0,

    /* U+0027 "'" */
    0xff, 0xe0,

    /* U+0028 "(" */
    0x7f, 0xff, 0xff, 0xe, 0x1c, 0x38, 0x70, 0xe1,
    0xc3, 0xff, 0xf7, 0xe0,

    /* U+0029 ")" */
    0xfd, 0xff, 0xf8, 0x70, 0xe1, 0xc3, 0x87, 0xe,
    0x1f, 0xff, 0xff, 0xc0,

    /* U+002A "*" */
    0x23, 0x9c, 0xa0,

    /* U+002B "+" */
    0x38, 0x73, 0xff, 0xff, 0xe7, 0xe, 0x0,

    /* U+002C "," */
    0xff, 0xe0,

    /* U+002D "-" */
    0xff, 0xff, 0xf8,

    /* U+002E "." */
    0xff, 0x80,

    /* U+002F "/" */
    0x7, 0x3, 0x83, 0xc1, 0xc1, 0xe0, 0xe0, 0xf0,
    0x70, 0x78, 0x38, 0x1c, 0x0,

    /* U+0030 "0" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc7, 0xfe, 0x3b,
    0xf3, 0x9f, 0xb8, 0xff, 0xc7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0031 "1" */
    0xf7, 0xfe, 0x73, 0x9c, 0xe7, 0x39, 0xce,

    /* U+0032 "2" */
    0xff, 0xf7, 0xff, 0xff, 0xfe, 0x0, 0x77, 0xff,
    0xff, 0xff, 0xff, 0xdc, 0x0, 0xff, 0xff, 0xff,
    0xff, 0xfe,

    /* U+0033 "3" */
    0xff, 0xc7, 0xff, 0x3f, 0xf8, 0x1, 0xcf, 0xff,
    0x7f, 0xff, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0034 "4" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7f, 0xff,
    0xff, 0xfd, 0xff, 0xe0, 0x7, 0x0, 0x38, 0x1,
    0xc0, 0xe,

    /* U+0035 "5" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xf, 0xff,
    0x7f, 0xff, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0036 "6" */
    0x7f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xf, 0xff,
    0x7f, 0xff, 0xff, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfe,

    /* U+0037 "7" */
    0xff, 0xff, 0xff, 0xff, 0xf0, 0x1e, 0x3, 0xc0,
    0x7c, 0x7, 0x80, 0xf0, 0x1e, 0x3, 0xc0, 0x3c,
    0x0,

    /* U+0038 "8" */
    0x3f, 0xe3, 0xff, 0x9f, 0xfc, 0x0, 0xe7, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0039 "9" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7f, 0xff,
    0xff, 0xfd, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+003A ":" */
    0xff, 0x80, 0x3f, 0xe0,

    /* U+003B ";" */
    0xff, 0x80, 0x3f, 0xf8,

    /* U+003C "<" */
    0x7, 0x7, 0x87, 0x87, 0x87, 0x87, 0x81, 0xe0,
    0x78, 0x1e, 0x7, 0x81, 0xc0,

    /* U+003D "=" */
    0xff, 0xff, 0xf8, 0xf, 0xff, 0xff, 0x80,

    /* U+003E ">" */
    0x78, 0x1c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0x3c,
    0x3c, 0x3c, 0x1c, 0x1e, 0x0,

    /* U+003F "?" */
    0xff, 0xf7, 0xff, 0xff, 0xfe, 0x0, 0x73, 0xff,
    0xbf, 0xfd, 0xff, 0xc0, 0x0, 0x70, 0x3, 0x80,
    0x1c, 0x0,

    /* U+0040 "@" */
    0x7f, 0xfd, 0xff, 0xff, 0xff, 0xff, 0x0, 0x7e,
    0xfe, 0xfd, 0xfd, 0xfb, 0x1b, 0xf7, 0xff, 0xef,
    0xfd, 0xc0, 0x3, 0xff, 0xff, 0xff, 0xf7, 0xff,
    0xe0,

    /* U+0041 "A" */
    0x0, 0xf0, 0xf, 0xc0, 0xfe, 0xf, 0x70, 0xf3,
    0x87, 0x1c, 0x7f, 0xe7, 0xff, 0x3f, 0xfb, 0x81,
    0xfc, 0xe,

    /* U+0042 "B" */
    0xff, 0xc7, 0xff, 0x3f, 0xf8, 0x1, 0xcf, 0xff,
    0x7f, 0xff, 0xff, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0043 "C" */
    0x7f, 0xff, 0xff, 0xff, 0xfe, 0x0, 0xe0, 0xe,
    0x0, 0xe0, 0xe, 0x0, 0xff, 0xff, 0xff, 0x7f,
    0xf0,

    /* U+0044 "D" */
    0xff, 0xf7, 0xff, 0xff, 0xfe, 0x0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0045 "E" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0xff,
    0xf7, 0xff, 0xbf, 0xdc, 0x0, 0xff, 0xff, 0xff,
    0xdf, 0xfe,

    /* U+0046 "F" */
    0x7f, 0xff, 0xff, 0xff, 0xfe, 0x0, 0xff, 0xff,
    0xff, 0xff, 0xfe, 0x0, 0xe0, 0xe, 0x0, 0xe0,
    0x0,

    /* U+0047 "G" */
    0x7f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xe, 0x7f,
    0x73, 0xff, 0x9f, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0048 "H" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7f, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0x7, 0xe0, 0x3f, 0x1,
    0xf8, 0xe,

    /* U+0049 "I" */
    0xff, 0x8f, 0xff, 0xff, 0x80,

    /* U+004A "J" */
    0x0, 0x70, 0x7, 0x0, 0x70, 0x7, 0x0, 0x70,
    0x7, 0x0, 0x70, 0x7, 0xff, 0xff, 0xff, 0xff,
    0xe0,

    /* U+004B "K" */
    0xe0, 0x78, 0x70, 0x78, 0x38, 0x78, 0x1c, 0x78,
    0xf, 0xf8, 0x7, 0xfc, 0x3, 0xfe, 0x1, 0xc7,
    0x80, 0xe1, 0xe0, 0x70, 0x78, 0x38, 0x1e, 0x0,
    0x7, 0x80, 0x1, 0xe0, 0x0, 0x70, 0x0, 0x1c,

    /* U+004C "L" */
    0xe0, 0xe, 0x0, 0xe0, 0xe, 0x0, 0xe0, 0xe,
    0x0, 0xe0, 0xe, 0x0, 0xff, 0xff, 0xff, 0x7f,
    0xf0,

    /* U+004D "M" */
    0xf0, 0xf, 0xf8, 0x1f, 0xf8, 0x1f, 0xfc, 0x3f,
    0xfc, 0x3f, 0xfe, 0x7f, 0xef, 0xf7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xe3, 0xc7, 0xe1, 0x87,

    /* U+004E "N" */
    0xe0, 0x3f, 0xc1, 0xff, 0xf, 0xfc, 0x7f, 0xf3,
    0xf7, 0xdf, 0x9f, 0xfc, 0x7f, 0xe1, 0xff, 0x7,
    0xf8, 0xe,

    /* U+004F "O" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0050 "P" */
    0xff, 0xef, 0xff, 0xff, 0xf0, 0x7, 0xff, 0xff,
    0xff, 0xff, 0xee, 0x0, 0xe0, 0xe, 0x0, 0xe0,
    0x0,

    /* U+0051 "Q" */
    0x7f, 0xf3, 0xff, 0xef, 0xff, 0xb8, 0xe, 0xe0,
    0x3b, 0x80, 0xee, 0x7b, 0xb8, 0xee, 0xff, 0xfb,
    0xff, 0xe7, 0xff, 0x0, 0x3c, 0x0, 0x70, 0x0,
    0xe0, 0x3, 0xc0, 0x7,

    /* U+0052 "R" */
    0xff, 0xe1, 0xff, 0xe3, 0xff, 0xc0, 0x3, 0x8f,
    0xff, 0x1f, 0xfe, 0x3f, 0xf8, 0x71, 0xe0, 0xe1,
    0xe1, 0xc1, 0xc3, 0x83, 0xc0, 0x3, 0xc0, 0x3,
    0x80, 0x3, 0x80, 0x7, 0x80, 0x7,

    /* U+0053 "S" */
    0x3f, 0xfd, 0xff, 0xf7, 0xff, 0xdc, 0x0, 0x7f,
    0xf9, 0xff, 0xf3, 0xff, 0xc0, 0x7, 0xff, 0xff,
    0xff, 0xff, 0xff, 0x80,

    /* U+0054 "T" */
    0xff, 0xff, 0xff, 0xff, 0xfe, 0xe, 0x0, 0x70,
    0x3, 0x80, 0x1c, 0x0, 0xe0, 0x7, 0x0, 0x38,
    0x1, 0xc0,

    /* U+0055 "U" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfe,

    /* U+0056 "V" */
    0xf0, 0x3b, 0xc1, 0xce, 0xe, 0x78, 0x71, 0xe3,
    0x87, 0x1c, 0x1c, 0xe0, 0xf7, 0x3, 0xf8, 0xf,
    0xc0, 0x3c,

    /* U+0057 "W" */
    0xf0, 0xe1, 0xee, 0x1c, 0x39, 0xc3, 0xc7, 0x3c,
    0xf8, 0xe3, 0x9f, 0x3c, 0x73, 0xf7, 0xf, 0xee,
    0xe0, 0xfd, 0xfc, 0x1f, 0x9f, 0x3, 0xe3, 0xe0,
    0x3c, 0x38, 0x0,

    /* U+0058 "X" */
    0x70, 0x78, 0x1c, 0x78, 0xf, 0x78, 0x3, 0xf8,
    0x0, 0xf8, 0x0, 0x3c, 0x0, 0x3e, 0x0, 0x3f,
    0x80, 0x3d, 0xe0, 0x1c, 0x78, 0x1c, 0x1e, 0x0,
    0x7, 0x80, 0x1, 0xe0, 0x0, 0x78, 0x0, 0x1c,

    /* U+0059 "Y" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7f, 0xff,
    0xff, 0xfd, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+005A "Z" */
    0xff, 0xff, 0xff, 0xff, 0xe0, 0x3c, 0x7, 0x80,
    0xf0, 0x1e, 0x3, 0xc0, 0x7f, 0xff, 0xff, 0xff,
    0xf0,

    /* U+005B "[" */
    0xff, 0xff, 0xff, 0xe, 0x1c, 0x38, 0x70, 0xe1,
    0xc3, 0xff, 0xff, 0xe0,

    /* U+005C "\\" */
    0x70, 0x38, 0x1e, 0x7, 0x3, 0xc0, 0xe0, 0x78,
    0x1c, 0xf, 0x3, 0x81, 0xc0,

    /* U+005D "]" */
    0xff, 0xff, 0xf8, 0x70, 0xe1, 0xc3, 0x87, 0xe,
    0x1f, 0xff, 0xff, 0xe0,

    /* U+005E "^" */
    0x7f, 0xf7, 0xff, 0xff, 0xfe, 0x0, 0x7f, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0x7, 0xe0, 0x3f, 0x1,
    0xf8, 0xe,

    /* U+005F "_" */
    0xff, 0xff, 0xff, 0xfc,

    /* U+0060 "`" */
    0x67, 0x70,

    /* U+0061 "a" */
    0x7, 0x80, 0x1f, 0x0, 0xfc, 0x3, 0xb8, 0x1e,
    0xe0, 0x71, 0xc3, 0xff, 0xf, 0xfe, 0x7f, 0xf9,
    0xc0, 0xf7, 0x1, 0xc0,

    /* U+0062 "b" */
    0xff, 0xc7, 0xff, 0x3f, 0xf8, 0x1, 0xcf, 0xff,
    0x7f, 0xff, 0xff, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0063 "c" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0x0,
    0x70, 0x3, 0x80, 0x1c, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0064 "d" */
    0xff, 0xf7, 0xff, 0xff, 0xfe, 0x0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0065 "e" */
    0x7f, 0xff, 0xff, 0xff, 0xfe, 0x0, 0xff, 0xff,
    0xff, 0xff, 0xfe, 0x0, 0xff, 0xff, 0xff, 0x7f,
    0xf0,

    /* U+0066 "f" */
    0x7f, 0xff, 0xff, 0xff, 0xfe, 0x0, 0xff, 0xff,
    0xff, 0xff, 0xfe, 0x0, 0xe0, 0xe, 0x0, 0xe0,
    0x0,

    /* U+0067 "g" */
    0x7f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xe, 0x7f,
    0x73, 0xff, 0x9f, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0068 "h" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7f, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0x7, 0xe0, 0x3f, 0x1,
    0xf8, 0xe,

    /* U+0069 "i" */
    0xff, 0xff, 0xff, 0xff, 0x80,

    /* U+006A "j" */
    0x0, 0x38, 0x1, 0xc0, 0xe, 0x0, 0x70, 0x3,
    0x80, 0x1c, 0x0, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+006B "k" */
    0xe0, 0x7f, 0x7, 0xb8, 0x79, 0xc7, 0x8f, 0xf8,
    0x7f, 0xc3, 0xfe, 0x1c, 0x78, 0xe1, 0xe7, 0x7,
    0xb8, 0x1e,

    /* U+006C "l" */
    0xe0, 0xe, 0x0, 0xe0, 0xe, 0x0, 0xe0, 0xe,
    0x0, 0xe0, 0xe, 0x0, 0xff, 0xff, 0xff, 0x7f,
    0xf0,

    /* U+006D "m" */
    0xf0, 0xf, 0xf8, 0x1f, 0xf8, 0x1f, 0xfc, 0x3f,
    0xfc, 0x3f, 0xfe, 0x7f, 0xef, 0xf7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xe3, 0xc7, 0xe1, 0x87,

    /* U+006E "n" */
    0xe0, 0x3f, 0xc1, 0xff, 0xf, 0xfc, 0x7f, 0xf3,
    0xf7, 0xdf, 0x9f, 0xfc, 0x7f, 0xe1, 0xff, 0x7,
    0xf8, 0xe,

    /* U+006F "o" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0070 "p" */
    0xff, 0xef, 0xff, 0xff, 0xf0, 0x7, 0xff, 0xff,
    0xff, 0xff, 0xee, 0x0, 0xe0, 0xe, 0x0, 0xe0,
    0x0,

    /* U+0071 "q" */
    0x7f, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x9e, 0xfc, 0x77, 0xff, 0xff, 0xff,
    0xdf, 0xfc, 0x1, 0xe0, 0x7, 0x0,

    /* U+0072 "r" */
    0xff, 0xef, 0xff, 0xff, 0xf0, 0x7, 0xff, 0xff,
    0xff, 0xff, 0xee, 0x3c, 0xe1, 0xee, 0xe, 0xe0,
    0x70,

    /* U+0073 "s" */
    0x7f, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xf, 0xff,
    0x7f, 0xfd, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+0074 "t" */
    0xff, 0xff, 0xff, 0xff, 0xfe, 0xe, 0x0, 0x70,
    0x3, 0x80, 0x1c, 0x0, 0xe0, 0x7, 0x0, 0x38,
    0x1, 0xc0,

    /* U+0075 "u" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xff, 0xff, 0xff,
    0xdf, 0xfc,

    /* U+0076 "v" */
    0x70, 0x1d, 0xc0, 0xe7, 0x83, 0x8e, 0x1e, 0x3c,
    0x70, 0x73, 0xc1, 0xee, 0x3, 0xf8, 0xf, 0xc0,
    0x1f, 0x0, 0x78, 0x0,

    /* U+0077 "w" */
    0xf0, 0xe1, 0xee, 0x1c, 0x39, 0xc3, 0xc7, 0x3c,
    0xf8, 0xe3, 0x9f, 0x3c, 0x73, 0xf7, 0xf, 0xee,
    0xe0, 0xfd, 0xfc, 0x1f, 0x9f, 0x3, 0xe3, 0xe0,
    0x3c, 0x38, 0x0,

    /* U+0078 "x" */
    0x70, 0x79, 0xc7, 0x8f, 0x78, 0x3f, 0x80, 0xf8,
    0x3, 0xc0, 0x3e, 0x3, 0xf8, 0x3d, 0xe1, 0xc7,
    0x9c, 0x1e,

    /* U+0079 "y" */
    0xe0, 0x3f, 0x1, 0xf8, 0xf, 0xc0, 0x7f, 0xff,
    0xff, 0xfd, 0xff, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xff, 0xfc,

    /* U+007A "z" */
    0xff, 0xff, 0xff, 0xff, 0xe0, 0x3c, 0x7, 0x80,
    0xf0, 0x1e, 0x3, 0xc0, 0x7f, 0xff, 0xff, 0xff,
    0xf0,

    /* U+007B "{" */
    0x78, 0x7, 0xe0, 0x3f, 0x81, 0xde, 0xe, 0x78,
    0x71, 0xc3, 0xff, 0x1f, 0xfc, 0xff, 0xe7, 0x3,
    0xb8, 0x1e,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0,

    /* U+007D "}" */
    0xe0, 0x7f, 0x7, 0xb8, 0x39, 0xc3, 0xce, 0x3c,
    0x71, 0xc3, 0x9c, 0x1d, 0xe0, 0xfe, 0x7, 0xe0,
    0x1e, 0x0,

    /* U+007E "~" */
    0xff, 0xf7, 0xff, 0xff, 0xff, 0xc0, 0x7e, 0x3,
    0xf0, 0x1f, 0x80, 0xfc, 0x7, 0xe0, 0x3f, 0x1,
    0xf8, 0xe,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 106, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 70, .box_w = 3, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 6, .adv_w = 130, .box_w = 7, .box_h = 4, .ofs_x = 1, .ofs_y = 7},
    {.bitmap_index = 10, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 28, .adv_w = 227, .box_w = 13, .box_h = 15, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 53, .adv_w = 291, .box_w = 17, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 77, .adv_w = 244, .box_w = 14, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 97, .adv_w = 70, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = 7},
    {.bitmap_index = 99, .adv_w = 129, .box_w = 7, .box_h = 13, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 111, .adv_w = 129, .box_w = 7, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 123, .adv_w = 80, .box_w = 5, .box_h = 4, .ofs_x = 0, .ofs_y = 8},
    {.bitmap_index = 126, .adv_w = 139, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 133, .adv_w = 70, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 135, .adv_w = 129, .box_w = 7, .box_h = 3, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 138, .adv_w = 70, .box_w = 3, .box_h = 3, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 140, .adv_w = 144, .box_w = 9, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 153, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 171, .adv_w = 104, .box_w = 5, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 178, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 196, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 214, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 232, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 250, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 268, .adv_w = 208, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 285, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 303, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 321, .adv_w = 70, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 325, .adv_w = 70, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 329, .adv_w = 146, .box_w = 9, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 342, .adv_w = 139, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 349, .adv_w = 146, .box_w = 9, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 362, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 380, .adv_w = 260, .box_w = 15, .box_h = 13, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 405, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 423, .adv_w = 228, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 441, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 458, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 476, .adv_w = 229, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 494, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 511, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 529, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 547, .adv_w = 70, .box_w = 3, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 552, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 569, .adv_w = 226, .box_w = 17, .box_h = 15, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 601, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 273, .box_w = 16, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 640, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 658, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 676, .adv_w = 218, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 693, .adv_w = 231, .box_w = 14, .box_h = 16, .ofs_x = 1, .ofs_y = -5},
    {.bitmap_index = 721, .adv_w = 217, .box_w = 15, .box_h = 16, .ofs_x = 1, .ofs_y = -5},
    {.bitmap_index = 751, .adv_w = 246, .box_w = 14, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 771, .adv_w = 224, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 789, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 807, .adv_w = 226, .box_w = 13, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 825, .adv_w = 307, .box_w = 19, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 852, .adv_w = 216, .box_w = 17, .box_h = 15, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 884, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 902, .adv_w = 212, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 919, .adv_w = 126, .box_w = 7, .box_h = 13, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 931, .adv_w = 144, .box_w = 9, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 944, .adv_w = 126, .box_w = 7, .box_h = 13, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 956, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 974, .adv_w = 186, .box_w = 10, .box_h = 3, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 978, .adv_w = 66, .box_w = 4, .box_h = 3, .ofs_x = -1, .ofs_y = 12},
    {.bitmap_index = 980, .adv_w = 235, .box_w = 14, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1000, .adv_w = 228, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1018, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1036, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1054, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1071, .adv_w = 211, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1088, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1106, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1124, .adv_w = 70, .box_w = 3, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1129, .adv_w = 226, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1147, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1165, .adv_w = 210, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1182, .adv_w = 273, .box_w = 16, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1204, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1222, .adv_w = 231, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1240, .adv_w = 218, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1257, .adv_w = 231, .box_w = 13, .box_h = 13, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1279, .adv_w = 217, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1296, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1314, .adv_w = 222, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1332, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1350, .adv_w = 232, .box_w = 14, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1370, .adv_w = 307, .box_w = 19, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1397, .adv_w = 216, .box_w = 13, .box_h = 11, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1415, .adv_w = 225, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1433, .adv_w = 212, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1450, .adv_w = 227, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1468, .adv_w = 70, .box_w = 3, .box_h = 17, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 1475, .adv_w = 226, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1493, .adv_w = 230, .box_w = 13, .box_h = 11, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1511, .adv_w = 66, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 96, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};

/*-----------------
 *    KERNING
 *----------------*/


/*Map glyph_ids to kern left classes*/
static const uint8_t kern_left_class_mapping[] =
{
    0, 0, 0, 1, 2, 3, 4, 5,
    6, 0, 0, 7, 8, 9, 8, 9,
    10, 11, 12, 12, 13, 0, 14, 14,
    15, 16, 17, 18, 18, 19, 2, 20,
    21, 22, 23, 24, 0, 11, 12, 25,
    26, 0, 0, 27, 28, 29, 12, 30,
    11, 31, 11, 32, 14, 33, 0, 3,
    34, 28, 35, 36, 0, 37, 0, 12,
    38, 39, 40, 41, 42, 11, 0, 25,
    26, 0, 0, 14, 28, 43, 12, 35,
    11, 31, 11, 32, 14, 44, 14, 45,
    34, 28, 35, 36, 46, 0, 47, 12,
    0
};

/*Map glyph_ids to kern right classes*/
static const uint8_t kern_right_class_mapping[] =
{
    0, 0, 0, 1, 2, 3, 4, 5,
    1, 0, 0, 6, 7, 8, 7, 8,
    9, 10, 11, 0, 0, 12, 0, 13,
    14, 15, 16, 17, 17, 18, 2, 19,
    20, 0, 21, 0, 13, 0, 10, 22,
    10, 0, 0, 23, 0, 24, 22, 25,
    10, 0, 10, 0, 26, 27, 28, 29,
    30, 31, 0, 32, 0, 33, 0, 22,
    34, 35, 36, 0, 10, 0, 10, 22,
    10, 0, 0, 37, 0, 24, 22, 25,
    10, 0, 10, 0, 22, 27, 24, 38,
    30, 31, 0, 32, 39, 0, 40, 0,
    0
};

/*Kern values between classes*/
static const int8_t kern_class_values[] =
{
    0, 0, 0, 0, -13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -15, 0,
    0, 0, 0, 0, -50, 0, -32, 0,
    0, -22, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -40, -32, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -15, 0, 0, 0,
    0, 0, 0, 0, -15, -5, -12, 0,
    0, 0, 0, -8, 0, -8, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -30, -3, 0, -18, 0, 0,
    0, 0, 0, -10, 0, 0, 0, 0,
    0, 0, -59, 0, -50, -22, -3, 0,
    0, 0, 0, 0, 0, -34, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -18, 0, -3, 0, 0, -3, 0, 0,
    0, 0, -24, -3, -30, 0, -3, 0,
    0, 0, -3, 0, -30, -13, -24, 0,
    -18, -13, 0, -20, 0, -20, 0, 0,
    0, 0, 0, 0, -13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -15, 0,
    0, 0, 0, 0, -50, 0, -32, 0,
    0, -22, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -40, -32, 0, 0, 0,
    0, 0, 0, 0, -24, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -24, 0,
    0, 0, 0, 0, -50, 0, -34, 0,
    0, -22, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -42, -34, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -32, 0, -3, 0,
    0, -8, -3, 0, -32, -13, -24, 0,
    0, 0, 0, -20, 0, -20, 0, 0,
    -34, 0, 0, 0, 0, -30, 0, 0,
    0, 0, -32, -24, 0, -18, 0, 0,
    0, 0, 0, -10, 0, 0, 0, 0,
    0, 0, -40, 0, -54, -18, 0, 0,
    0, 0, 0, 0, 0, -34, 0, 0,
    0, 0, 0, 0, -22, 0, 0, 0,
    0, -3, 0, 0, 0, 0, -24, 0,
    0, 0, 0, 0, -60, -3, -47, 0,
    -5, -22, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -42, -40, 0, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, -8, 0, 0, 0,
    0, 0, 0, 0, -8, -3, -5, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    -7, 0, 0, -7, 0, -24, 0, 0,
    -3, 0, -22, -3, 0, -24, 0, 0,
    0, 0, 0, -8, -8, 0, 0, 0,
    0, 0, -24, 0, -27, -18, -5, 0,
    -22, 0, -24, -3, 0, -22, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, -24, 0, -8, -50,
    -50, 0, 0, 0, 0, 0, -24, 0,
    0, 0, 0, 0, -71, 0, -52, 0,
    0, -24, 0, 0, 0, 0, 0, 0,
    0, -52, 0, -55, -42, 0, 0, 0,
    -7, 0, 0, -7, 0, -24, 0, 0,
    -5, 0, -22, 0, 0, -24, 0, 0,
    0, 0, -5, -8, -8, 0, 0, 0,
    0, 0, -24, 0, -27, -18, -5, 0,
    -24, 0, -24, -3, 0, -22, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, -8, 0, 0, 0,
    0, 0, 0, 0, -8, 0, -5, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -15, -5, 0, 0,
    0, 0, 0, 0, 0, -8, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, -5, 0, 0, -5, 0, -5, 0,
    0, -29, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -12, 0, 0, 0, 0, 0,
    0, 0, -29, -7, -37, 0, -12, 0,
    0, -15, -12, 0, -37, -15, -29, -12,
    0, 0, 0, -20, 0, -20, 0, 0,
    0, 0, 0, 0, -2, 0, 0, -22,
    -30, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -50, 0, -22, 0,
    0, -22, 0, 0, -7, -3, -5, 0,
    -3, -24, 0, -34, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -7, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    -7, 0, 0, -7, 0, -24, 0, 0,
    -5, 0, -22, -3, 0, -24, 0, 0,
    0, 0, -7, -8, -7, 0, 0, 0,
    0, 0, -24, 0, -27, -18, -5, 0,
    -24, 0, -24, -3, 0, -22, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -17, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -30, 0, -3, 0,
    0, -22, 0, 0, 0, 0, 0, 0,
    0, -25, 0, -17, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -2, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, -12, 0, -3, -5, 0, -24, 0,
    0, -5, 0, 0, -5, 0, -5, 0,
    0, -29, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    -34, 0, 0, -59, 0, -34, -3, 0,
    0, 0, -30, -24, 0, -18, 0, 0,
    0, -12, 0, -10, 0, 0, 0, 0,
    0, 0, -67, 0, -67, -30, 0, 0,
    -45, 0, -34, 0, 0, -47, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -5, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -24,
    -30, 0, 0, 0, 0, 0, -2, 0,
    0, 0, -5, 0, -44, 0, -24, 0,
    0, -22, 0, 0, -8, -3, -5, 0,
    0, -29, 0, -30, -2, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -22, 0, -3, -40,
    -44, 0, 0, 0, 0, 0, -24, 0,
    0, -12, 0, 0, -67, 0, -67, 0,
    0, -24, 0, 0, 0, 0, 0, 0,
    0, -34, 0, -47, -67, 0, 0, 0,
    0, -5, -3, 0, -18, 0, -13, -18,
    -32, -3, 0, 0, -3, 0, -18, 0,
    -5, -15, 0, 0, -42, -3, -30, 0,
    -3, -22, 0, 0, 0, 0, 0, 0,
    0, -24, 0, -32, -24, 0, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -5, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    0, -3, -27, -30, -3, -18, -3, 0,
    0, 0, 0, -8, 0, 0, 0, -3,
    0, 0, -44, -3, -60, -32, 0, 0,
    0, 0, 0, 0, -3, -42, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -32, -29, 0, -18, 0, 0,
    0, 0, 0, -10, 0, 0, 0, 0,
    0, 0, -34, 0, -67, -24, 0, 0,
    0, 0, 0, 0, 0, -34, 0, 0,
    0, 0, 0, 0, -24, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -24, 0,
    0, 0, 0, 0, -34, 0, -30, 0,
    0, -24, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -34, -30, 0, 0, 0,
    -32, -8, 0, -34, -3, -42, -20, 0,
    0, -3, -30, -34, -3, -18, -3, 0,
    0, -20, 0, -17, 0, 0, 0, -3,
    0, 0, -47, -3, -60, -32, 0, 0,
    -42, 0, -34, 0, -5, -42, 0, 0,
    -7, 0, 0, -7, 0, -24, 0, 0,
    -5, 0, -22, -3, 0, -24, 0, 0,
    0, 0, -7, -8, -7, 0, 0, 0,
    0, 0, -24, 0, -27, -18, -5, 0,
    -22, 0, -24, -3, 0, -22, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, -3, -5, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    -34, 0, 0, -59, 0, -34, -3, 0,
    0, 0, -30, -24, 0, -18, 0, 0,
    0, -12, 0, -10, 0, 0, 0, 0,
    0, 0, -67, 0, -67, -30, 0, 0,
    -45, 0, -34, 0, 0, -49, 0, 0,
    0, 0, 0, 0, -22, 0, -3, -40,
    -47, 0, 0, 0, 0, 0, -24, 0,
    0, -12, 0, 0, -67, 0, -67, 0,
    0, -24, 0, 0, 0, 0, 0, 0,
    0, -34, 0, -47, -67, 0, 0, 0,
    0, -8, -3, 0, -22, 0, -20, -34,
    -42, -3, 0, 0, -3, 0, -22, -3,
    -8, -20, 0, 0, -60, -3, -47, 0,
    -3, -22, 0, 0, 0, 0, 0, 0,
    0, -34, 0, -42, -35, 0, -3, 0,
    -42, -15, 0, -50, -7, -50, -32, 0,
    0, -8, -35, -50, -8, -18, -8, 0,
    0, -37, 0, -17, 0, 0, 0, -8,
    0, 0, -67, -5, -87, -42, 0, 0,
    -60, 0, -34, 0, -8, -60, 0, -8,
    0, -15, -8, 0, -27, 0, -32, -54,
    -60, -8, 0, 0, -8, 0, -27, -8,
    -15, -37, 0, 0, -87, -8, -67, 0,
    -8, -27, 0, 0, 0, 0, 0, 0,
    0, -67, 0, -60, -66, 0, 0, 0
};


/*Collect the kern class' data in one place*/
static const lv_font_fmt_txt_kern_classes_t kern_classes =
{
    .class_pair_values   = kern_class_values,
    .left_class_mapping  = kern_left_class_mapping,
    .right_class_mapping = kern_right_class_mapping,
    .left_class_cnt      = 47,
    .right_class_cnt     = 40,
};

/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = &kern_classes,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 1,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif

};

extern const lv_font_t LV_FONT_MONTSERRAT_20;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t space_ranger_21 = {
#else
lv_font_t space_ranger_21 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 20,          /*The maximum line height required by the font*/
    .base_line = 5,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -2,
    .underline_thickness = 1,
#endif
    .static_bitmap = 0,
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = &LV_FONT_MONTSERRAT_20,
#endif
    .user_data = NULL,
};



#endif /*#if SPACE_RANGER_21*/
