/*******************************************************************************
 * Size: 19 px
 * Bpp: 1
 * Opts: --bpp 1 --size 19 --no-compress --stride 1 --align 1 --font spaceranger.ttf --range 32-127 --format lvgl -o space_ranger_19.c
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



#ifndef SPACE_RANGER_19
#define SPACE_RANGER_19 1
#endif

#if SPACE_RANGER_19

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xff, 0xc0, 0xfc,

    /* U+0022 "\"" */
    0xff, 0xff, 0xf6,

    /* U+0023 "#" */
    0x39, 0xc3, 0x9c, 0xff, 0xff, 0xff, 0x39, 0xc3,
    0x9c, 0xff, 0xff, 0xff, 0x39, 0xc3, 0x9c,

    /* U+0024 "$" */
    0xe, 0x1, 0xc3, 0xff, 0xff, 0xfe, 0xe1, 0xdc,
    0x3f, 0xff, 0xff, 0xe, 0xe1, 0xdf, 0xff, 0xff,
    0xf0, 0xe0, 0x1c, 0x0,

    /* U+0025 "%" */
    0xfc, 0xe1, 0xf9, 0xc3, 0x37, 0x7, 0xee, 0xf,
    0xf8, 0x0, 0x7f, 0xc1, 0xdf, 0x83, 0xb3, 0xe,
    0x7e, 0x1c, 0xfc,

    /* U+0026 "&" */
    0x3f, 0x83, 0xfc, 0x1c, 0x0, 0xe0, 0xf, 0xff,
    0xff, 0xff, 0x83, 0x9c, 0x1c, 0xff, 0xe7, 0xff,
    0x0,

    /* U+0027 "'" */
    0xff, 0xe0,

    /* U+0028 "(" */
    0xff, 0xfe, 0x38, 0xe3, 0x8e, 0x38, 0xe3, 0xff,
    0xc0,

    /* U+0029 ")" */
    0xff, 0xfc, 0x38, 0x70, 0xe1, 0xc3, 0x87, 0xf,
    0xff, 0xf8,

    /* U+002A "*" */
    0x67, 0x70,

    /* U+002B "+" */
    0x38, 0x73, 0xff, 0xf3, 0x87, 0xe, 0x0,

    /* U+002C "," */
    0xff, 0xe0,

    /* U+002D "-" */
    0xff, 0xf0,

    /* U+002E "." */
    0xfc,

    /* U+002F "/" */
    0x7, 0xe, 0xe, 0x1c, 0x1c, 0x38, 0x38, 0x70,
    0x70, 0xf0,

    /* U+0030 "0" */
    0x7f, 0xef, 0xff, 0xe3, 0xfe, 0x77, 0xe7, 0x7e,
    0xe7, 0xee, 0x7f, 0xc7, 0xff, 0xff, 0xff,

    /* U+0031 "1" */
    0xf7, 0xce, 0x73, 0x9c, 0xe7, 0x39, 0xc0,

    /* U+0032 "2" */
    0xff, 0xff, 0xff, 0x0, 0x70, 0x7, 0xff, 0xff,
    0xff, 0xe0, 0xe, 0x0, 0xff, 0xff, 0xff,

    /* U+0033 "3" */
    0xff, 0xcf, 0xfe, 0x0, 0xe0, 0xe, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xff,

    /* U+0034 "4" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0x0, 0x70, 0x7,

    /* U+0035 "5" */
    0xff, 0xff, 0xff, 0xe0, 0xe, 0x0, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xff,

    /* U+0036 "6" */
    0x7f, 0xff, 0xff, 0xe0, 0xe, 0x0, 0xff, 0xff,
    0xff, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0037 "7" */
    0xff, 0xff, 0xfc, 0xf, 0x1, 0xe0, 0x78, 0x1e,
    0x3, 0x80, 0xf0, 0x3c, 0xf, 0x0,

    /* U+0038 "8" */
    0x3f, 0xc7, 0xfe, 0x0, 0xe0, 0xe, 0x7f, 0xff,
    0xff, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0039 "9" */
    0xff, 0xef, 0xff, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xfe,

    /* U+003A ":" */
    0xfc, 0x1, 0xf8,

    /* U+003B ";" */
    0xfc, 0x1, 0xff, 0xc0,

    /* U+003C "<" */
    0xf, 0x1e, 0x3c, 0x38, 0x70, 0x70, 0x38, 0x3c,
    0x1e, 0xf,

    /* U+003D "=" */
    0xff, 0xfc, 0x0, 0xf, 0xff, 0xc0,

    /* U+003E ">" */
    0x70, 0x78, 0x3c, 0x1e, 0xe, 0xe, 0x1e, 0x3c,
    0x38, 0x70,

    /* U+003F "?" */
    0xff, 0xff, 0xff, 0x0, 0x70, 0x7, 0x7f, 0xf7,
    0xff, 0x0, 0x0, 0x0, 0x70, 0x7, 0x0,

    /* U+0040 "@" */
    0x7f, 0xfb, 0xff, 0xfe, 0x1, 0xf8, 0x7, 0xef,
    0xdf, 0xbf, 0x7e, 0xcd, 0xfb, 0xff, 0xef, 0xfb,
    0x80, 0xf, 0xff, 0xdf, 0xff,

    /* U+0041 "A" */
    0x1, 0xe0, 0x3f, 0x7, 0x70, 0xe7, 0xe, 0x71,
    0xc7, 0x3f, 0xf3, 0xff, 0x70, 0x7e, 0x7,

    /* U+0042 "B" */
    0xff, 0x8f, 0xfc, 0x1, 0xc0, 0x1c, 0xff, 0xef,
    0xff, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0043 "C" */
    0xff, 0xff, 0xff, 0x80, 0x70, 0xe, 0x1, 0xc0,
    0x38, 0x7, 0x0, 0xff, 0xff, 0xfc,

    /* U+0044 "D" */
    0xff, 0xef, 0xff, 0x0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0045 "E" */
    0x7f, 0xff, 0xff, 0xe0, 0x7e, 0x7, 0xef, 0xfe,
    0xff, 0xe0, 0xe, 0x0, 0xff, 0xff, 0xff,

    /* U+0046 "F" */
    0xff, 0xff, 0xff, 0x80, 0x70, 0xf, 0xff, 0xff,
    0xf8, 0x7, 0x0, 0xe0, 0x1c, 0x0,

    /* U+0047 "G" */
    0x7f, 0xff, 0xff, 0xe0, 0xe, 0x0, 0xe7, 0xfe,
    0x7f, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0048 "H" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7,

    /* U+0049 "I" */
    0xfc, 0xf, 0xff, 0xfc,

    /* U+004A "J" */
    0x0, 0xe0, 0x1c, 0x3, 0x80, 0x70, 0xe, 0x1,
    0xc0, 0x38, 0x7, 0xff, 0xff, 0xfc,

    /* U+004B "K" */
    0xe0, 0xf1, 0xc1, 0xc3, 0x87, 0x7, 0x1c, 0xf,
    0xf0, 0x1f, 0xe0, 0x38, 0xe0, 0x70, 0xe0, 0xe0,
    0xe1, 0xc1, 0xe0, 0x1, 0xe0, 0x1, 0xc0, 0x1,
    0xc0, 0x1, 0xc0,

    /* U+004C "L" */
    0xe0, 0x1c, 0x3, 0x80, 0x70, 0xe, 0x1, 0xc0,
    0x38, 0x7, 0x0, 0xff, 0xff, 0xfc,

    /* U+004D "M" */
    0xf0, 0x3f, 0xc0, 0xff, 0x87, 0xfe, 0x1f, 0xfc,
    0xff, 0xb3, 0x7e, 0xfd, 0xf9, 0xe7, 0xe7, 0x9f,
    0x8c, 0x70,

    /* U+004E "N" */
    0xe0, 0x7f, 0x7, 0xf8, 0x7f, 0xc7, 0xee, 0x7e,
    0x77, 0xe3, 0xfe, 0x1f, 0xe0, 0xfe, 0x7,

    /* U+004F "O" */
    0x7f, 0xef, 0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0050 "P" */
    0xff, 0xff, 0xfc, 0x3, 0x80, 0x7f, 0xff, 0xff,
    0xf8, 0x7, 0x0, 0xe0, 0x1c, 0x0,

    /* U+0051 "Q" */
    0x7f, 0xe7, 0xff, 0xb8, 0x1d, 0xc0, 0xee, 0x7,
    0x73, 0xbb, 0x9d, 0xdc, 0x6e, 0xff, 0xf7, 0xff,
    0x0, 0x38, 0x1, 0xc0, 0x7, 0x0, 0x3c,

    /* U+0052 "R" */
    0xff, 0xe3, 0xff, 0x80, 0xe, 0x0, 0x38, 0xff,
    0xe3, 0xff, 0x8e, 0x38, 0x38, 0x70, 0xe1, 0xc3,
    0x83, 0x80, 0x7, 0x0, 0x1c, 0x0, 0x38, 0x0,
    0x70,

    /* U+0053 "S" */
    0x7f, 0xfb, 0xff, 0xdc, 0x0, 0xe0, 0x7, 0xff,
    0xbf, 0xfc, 0x0, 0xe0, 0x7, 0xff, 0xff, 0xff,
    0xc0,

    /* U+0054 "T" */
    0xff, 0xff, 0xfc, 0x38, 0x7, 0x0, 0xe0, 0x1c,
    0x3, 0x80, 0x70, 0xe, 0x1, 0xc0,

    /* U+0055 "U" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0056 "V" */
    0xe0, 0x77, 0x7, 0x38, 0x73, 0x87, 0x1c, 0x70,
    0xe7, 0x6, 0x70, 0x77, 0x3, 0xf0, 0x1e,

    /* U+0057 "W" */
    0xe0, 0xc3, 0xb8, 0xe1, 0xdc, 0x79, 0xce, 0x7c,
    0xe3, 0xbe, 0x71, 0xdf, 0xb0, 0xfd, 0xf8, 0x3e,
    0xfc, 0x1f, 0x3c, 0x7, 0xe, 0x0,

    /* U+0058 "X" */
    0x70, 0xf0, 0xf3, 0xc0, 0xff, 0x0, 0xfc, 0x0,
    0xf0, 0x1, 0xe0, 0x7, 0xe0, 0x1f, 0xe0, 0x79,
    0xe0, 0xe1, 0xe0, 0x1, 0xc0, 0x1, 0xc0, 0x1,
    0xc0, 0x3, 0xc0,

    /* U+0059 "Y" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xfe,

    /* U+005A "Z" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xff, 0xff, 0xfc,

    /* U+005B "[" */
    0xff, 0xfe, 0x38, 0xe3, 0x8e, 0x38, 0xe3, 0xff,
    0xc0,

    /* U+005C "\\" */
    0xf0, 0x70, 0x70, 0x38, 0x38, 0x1c, 0x1c, 0xe,
    0xe, 0x7,

    /* U+005D "]" */
    0xff, 0xf1, 0xc7, 0x1c, 0x71, 0xc7, 0x1f, 0xff,
    0xc0,

    /* U+005E "^" */
    0xff, 0xef, 0xff, 0x0, 0x70, 0x7, 0xff, 0xff,
    0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7,

    /* U+005F "_" */
    0xff, 0xff, 0xc0,

    /* U+0060 "`" */
    0x67,

    /* U+0061 "a" */
    0x7, 0x80, 0x7c, 0x3, 0xf0, 0x3b, 0x81, 0xde,
    0x1c, 0x70, 0xff, 0x8f, 0xfe, 0x70, 0x77, 0x81,
    0xc0,

    /* U+0062 "b" */
    0xff, 0x8f, 0xfc, 0x1, 0xc0, 0x1c, 0xff, 0xef,
    0xff, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0063 "c" */
    0x7f, 0xff, 0xff, 0xe0, 0x7e, 0x0, 0xe0, 0xe,
    0x0, 0xe0, 0xe, 0x7, 0xff, 0xff, 0xff,

    /* U+0064 "d" */
    0xff, 0xef, 0xff, 0x0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0065 "e" */
    0xff, 0xff, 0xff, 0x80, 0x70, 0xf, 0xff, 0xff,
    0xf8, 0x7, 0x0, 0xff, 0xff, 0xfc,

    /* U+0066 "f" */
    0xff, 0xff, 0xff, 0x80, 0x70, 0xf, 0xff, 0xff,
    0xf8, 0x7, 0x0, 0xe0, 0x1c, 0x0,

    /* U+0067 "g" */
    0x7f, 0xff, 0xff, 0xe0, 0xe, 0x0, 0xe7, 0xfe,
    0x7f, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0068 "h" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7,

    /* U+0069 "i" */
    0xff, 0xff, 0xff, 0xfc,

    /* U+006A "j" */
    0x0, 0x70, 0x7, 0x0, 0x70, 0x7, 0x0, 0x70,
    0x7, 0x0, 0x7e, 0x7, 0xff, 0xff, 0xfe,

    /* U+006B "k" */
    0xe0, 0xfe, 0xe, 0xe1, 0xce, 0x38, 0xff, 0xf,
    0xf0, 0xe3, 0x8e, 0x1c, 0xe0, 0xee, 0xf,

    /* U+006C "l" */
    0xe0, 0x1c, 0x3, 0x80, 0x70, 0xe, 0x1, 0xc0,
    0x38, 0x7, 0x0, 0xff, 0xff, 0xfc,

    /* U+006D "m" */
    0xf0, 0x3f, 0xc0, 0xff, 0x87, 0xfe, 0x1f, 0xfc,
    0xff, 0xb3, 0x7e, 0xfd, 0xf9, 0xe7, 0xe7, 0x9f,
    0x8c, 0x70,

    /* U+006E "n" */
    0xe0, 0x7f, 0x7, 0xf8, 0x7f, 0xc7, 0xee, 0x7e,
    0x77, 0xe3, 0xfe, 0x1f, 0xe0, 0xfe, 0x7,

    /* U+006F "o" */
    0x7f, 0xef, 0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0070 "p" */
    0xff, 0xff, 0xfc, 0x3, 0x80, 0x7f, 0xff, 0xff,
    0xf8, 0x7, 0x0, 0xe0, 0x1c, 0x0,

    /* U+0071 "q" */
    0x7f, 0xef, 0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x77, 0xe7, 0x7e, 0x37, 0xff, 0xff, 0xfe, 0x0,
    0xe0, 0xe,

    /* U+0072 "r" */
    0xff, 0xff, 0xfc, 0x3, 0x80, 0x7f, 0xff, 0xff,
    0xf8, 0xe7, 0x1e, 0xe1, 0xdc, 0x1c,

    /* U+0073 "s" */
    0xff, 0xff, 0xff, 0xe0, 0xe, 0x0, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xff,

    /* U+0074 "t" */
    0xff, 0xff, 0xfc, 0x38, 0x7, 0x0, 0xe0, 0x1c,
    0x3, 0x80, 0x70, 0xe, 0x1, 0xc0,

    /* U+0075 "u" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff, 0xff,

    /* U+0076 "v" */
    0xf0, 0x7b, 0x83, 0x9e, 0x1c, 0x71, 0xc3, 0x8e,
    0xe, 0xe0, 0x77, 0x1, 0xf0, 0xf, 0x80, 0x38,
    0x0,

    /* U+0077 "w" */
    0xe0, 0xc3, 0xb8, 0xe1, 0xdc, 0x79, 0xce, 0x7c,
    0xe3, 0xbe, 0x71, 0xdf, 0xb0, 0xfd, 0xf8, 0x3e,
    0xfc, 0x1f, 0x3c, 0x7, 0xe, 0x0,

    /* U+0078 "x" */
    0x70, 0xf7, 0x9e, 0x3f, 0xc1, 0xf8, 0xf, 0x0,
    0xf0, 0x1f, 0x83, 0xfc, 0x79, 0xe7, 0xf,

    /* U+0079 "y" */
    0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7, 0xff, 0xff,
    0xff, 0x0, 0x70, 0x7, 0xff, 0xff, 0xfe,

    /* U+007A "z" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xff, 0xff, 0xfc,

    /* U+007B "{" */
    0x78, 0xf, 0xc0, 0xee, 0xe, 0x70, 0xe7, 0xe,
    0x38, 0xff, 0xcf, 0xfc, 0xe0, 0xee, 0x7,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,

    /* U+007D "}" */
    0xe0, 0x7e, 0xe, 0xe1, 0xce, 0x1c, 0xe3, 0x8e,
    0x70, 0xe6, 0xe, 0xe0, 0xfc, 0x7, 0x80,

    /* U+007E "~" */
    0xff, 0xff, 0xff, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xe0, 0x7e, 0x7, 0xe0, 0x7e, 0x7,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 95, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 63, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 5, .adv_w = 118, .box_w = 6, .box_h = 4, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 8, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 23, .adv_w = 205, .box_w = 11, .box_h = 14, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 43, .adv_w = 264, .box_w = 15, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 62, .adv_w = 221, .box_w = 13, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 79, .adv_w = 63, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 81, .adv_w = 116, .box_w = 6, .box_h = 11, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 90, .adv_w = 116, .box_w = 7, .box_h = 11, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 100, .adv_w = 72, .box_w = 4, .box_h = 4, .ofs_x = 0, .ofs_y = 7},
    {.bitmap_index = 102, .adv_w = 126, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 109, .adv_w = 63, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 111, .adv_w = 117, .box_w = 6, .box_h = 2, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 113, .adv_w = 63, .box_w = 3, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 114, .adv_w = 130, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 124, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 139, .adv_w = 95, .box_w = 5, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 146, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 161, .adv_w = 203, .box_w = 12, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 176, .adv_w = 204, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 191, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 206, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 221, .adv_w = 188, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 235, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 250, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 265, .adv_w = 63, .box_w = 3, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 268, .adv_w = 63, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 272, .adv_w = 132, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 282, .adv_w = 126, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 288, .adv_w = 132, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 298, .adv_w = 204, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 313, .adv_w = 235, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 334, .adv_w = 206, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 349, .adv_w = 207, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 364, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 378, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 393, .adv_w = 207, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 408, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 422, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 437, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 452, .adv_w = 64, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 456, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 470, .adv_w = 204, .box_w = 15, .box_h = 14, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 497, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 511, .adv_w = 247, .box_w = 14, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 529, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 544, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 559, .adv_w = 197, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 573, .adv_w = 209, .box_w = 13, .box_h = 14, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 596, .adv_w = 197, .box_w = 14, .box_h = 14, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 621, .adv_w = 223, .box_w = 13, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 638, .adv_w = 203, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 652, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 667, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 682, .adv_w = 278, .box_w = 17, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 704, .adv_w = 195, .box_w = 15, .box_h = 14, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 731, .adv_w = 204, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 746, .adv_w = 192, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 760, .adv_w = 114, .box_w = 6, .box_h = 11, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 769, .adv_w = 130, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 779, .adv_w = 114, .box_w = 6, .box_h = 11, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 788, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 803, .adv_w = 169, .box_w = 9, .box_h = 2, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 806, .adv_w = 60, .box_w = 4, .box_h = 2, .ofs_x = -1, .ofs_y = 11},
    {.bitmap_index = 807, .adv_w = 212, .box_w = 13, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 824, .adv_w = 207, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 839, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 854, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 869, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 883, .adv_w = 191, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 897, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 912, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 927, .adv_w = 64, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 931, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 946, .adv_w = 204, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 961, .adv_w = 190, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 975, .adv_w = 247, .box_w = 14, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 993, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1008, .adv_w = 209, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1023, .adv_w = 197, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1037, .adv_w = 209, .box_w = 12, .box_h = 12, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1055, .adv_w = 197, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1069, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1084, .adv_w = 201, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1098, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1113, .adv_w = 209, .box_w = 13, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1130, .adv_w = 278, .box_w = 17, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1152, .adv_w = 195, .box_w = 12, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1167, .adv_w = 204, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1182, .adv_w = 192, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1196, .adv_w = 206, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1211, .adv_w = 63, .box_w = 3, .box_h = 15, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 1217, .adv_w = 205, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1232, .adv_w = 208, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 1247, .adv_w = 60, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
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
    0, 0, 0, 0, -12, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -14, 0,
    0, 0, 0, 0, -46, 0, -29, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -36, -29, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -14, 0, 0, 0,
    0, 0, 0, 0, -14, -5, -11, 0,
    0, 0, 0, -8, 0, -8, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -27, -3, 0, -17, 0, 0,
    0, 0, 0, -9, 0, 0, 0, 0,
    0, 0, -53, 0, -46, -20, -3, 0,
    0, 0, 0, 0, 0, -30, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -17, 0, -3, 0, 0, -3, 0, 0,
    0, 0, -21, -3, -27, 0, -3, 0,
    0, 0, -3, 0, -27, -12, -21, 0,
    -17, -12, 0, -18, 0, -18, 0, 0,
    0, 0, 0, 0, -12, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -14, 0,
    0, 0, 0, 0, -46, 0, -29, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -36, -29, 0, 0, 0,
    0, 0, 0, 0, -21, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -21, 0,
    0, 0, 0, 0, -46, 0, -30, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -38, -30, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -29, 0, -3, 0,
    0, -8, -3, 0, -29, -12, -21, 0,
    0, 0, 0, -18, 0, -18, 0, 0,
    -30, 0, 0, 0, 0, -27, 0, 0,
    0, 0, -29, -21, 0, -17, 0, 0,
    0, 0, 0, -9, 0, 0, 0, 0,
    0, 0, -36, 0, -49, -17, 0, 0,
    0, 0, 0, 0, 0, -30, 0, 0,
    0, 0, 0, 0, -20, 0, 0, 0,
    0, -3, 0, 0, 0, 0, -21, 0,
    0, 0, 0, 0, -55, -3, -43, 0,
    -5, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -38, -36, 0, -3, 0,
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
    -6, 0, 0, -6, 0, -21, 0, 0,
    -3, 0, -20, -3, 0, -21, 0, 0,
    0, 0, 0, -8, -8, 0, 0, 0,
    0, 0, -21, 0, -24, -17, -5, 0,
    -20, 0, -21, -3, 0, -20, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, -21, 0, -8, -46,
    -46, 0, 0, 0, 0, 0, -21, 0,
    0, 0, 0, 0, -64, 0, -47, 0,
    0, -21, 0, 0, 0, 0, 0, 0,
    0, -47, 0, -50, -38, 0, 0, 0,
    -6, 0, 0, -6, 0, -21, 0, 0,
    -5, 0, -20, 0, 0, -21, 0, 0,
    0, 0, -5, -8, -8, 0, 0, 0,
    0, 0, -21, 0, -24, -17, -5, 0,
    -21, 0, -21, -3, 0, -20, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, -8, 0, 0, 0,
    0, 0, 0, 0, -8, 0, -5, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -14, -5, 0, 0,
    0, 0, 0, 0, 0, -8, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, -5, 0, 0, -5, 0, -5, 0,
    0, -26, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -11, 0, 0, 0, 0, 0,
    0, 0, -26, -6, -33, 0, -11, 0,
    0, -14, -11, 0, -33, -14, -26, -11,
    0, 0, 0, -18, 0, -18, 0, 0,
    0, 0, 0, 0, -2, 0, 0, -20,
    -27, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -46, 0, -20, 0,
    0, -20, 0, 0, -6, -3, -5, 0,
    -3, -21, 0, -30, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -6, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    -6, 0, 0, -6, 0, -21, 0, 0,
    -5, 0, -20, -3, 0, -21, 0, 0,
    0, 0, -6, -8, -6, 0, 0, 0,
    0, 0, -21, 0, -24, -17, -5, 0,
    -21, 0, -21, -3, 0, -20, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -15, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -27, 0, -3, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, -23, 0, -15, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -2, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, -11, 0, -3, -5, 0, -21, 0,
    0, -5, 0, 0, -5, 0, -5, 0,
    0, -26, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    -30, 0, 0, -53, 0, -30, -3, 0,
    0, 0, -27, -21, 0, -17, 0, 0,
    0, -11, 0, -9, 0, 0, 0, 0,
    0, 0, -61, 0, -61, -27, 0, 0,
    -41, 0, -30, 0, 0, -43, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -5, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -21,
    -27, 0, 0, 0, 0, 0, -2, 0,
    0, 0, -5, 0, -40, 0, -21, 0,
    0, -20, 0, 0, -8, -3, -5, 0,
    0, -26, 0, -27, -2, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -2, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -20, 0, -3, -36,
    -40, 0, 0, 0, 0, 0, -21, 0,
    0, -11, 0, 0, -61, 0, -61, 0,
    0, -21, 0, 0, 0, 0, 0, 0,
    0, -30, 0, -43, -61, 0, 0, 0,
    0, -5, -3, 0, -17, 0, -12, -17,
    -29, -3, 0, 0, -3, 0, -17, 0,
    -5, -14, 0, 0, -38, -3, -27, 0,
    -3, -20, 0, 0, 0, 0, 0, 0,
    0, -21, 0, -29, -21, 0, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -5, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -11, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    0, -3, -24, -27, -3, -17, -3, 0,
    0, 0, 0, -8, 0, 0, 0, -3,
    0, 0, -40, -3, -55, -29, 0, 0,
    0, 0, 0, 0, -3, -38, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -29, -26, 0, -17, 0, 0,
    0, 0, 0, -9, 0, 0, 0, 0,
    0, 0, -30, 0, -61, -21, 0, 0,
    0, 0, 0, 0, 0, -30, 0, 0,
    0, 0, 0, 0, -21, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -21, 0,
    0, 0, 0, 0, -30, 0, -27, 0,
    0, -21, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -30, -27, 0, 0, 0,
    -29, -8, 0, -30, -3, -38, -18, 0,
    0, -3, -27, -30, -3, -17, -3, 0,
    0, -18, 0, -15, 0, 0, 0, -3,
    0, 0, -43, -3, -55, -29, 0, 0,
    -38, 0, -30, 0, -5, -38, 0, 0,
    -6, 0, 0, -6, 0, -21, 0, 0,
    -5, 0, -20, -3, 0, -21, 0, 0,
    0, 0, -6, -8, -6, 0, 0, 0,
    0, 0, -21, 0, -24, -17, -5, 0,
    -20, 0, -21, -3, 0, -20, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -5, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, -3, -5, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    -30, 0, 0, -53, 0, -30, -3, 0,
    0, 0, -27, -21, 0, -17, 0, 0,
    0, -11, 0, -9, 0, 0, 0, 0,
    0, 0, -61, 0, -61, -27, 0, 0,
    -41, 0, -30, 0, 0, -44, 0, 0,
    0, 0, 0, 0, -20, 0, -3, -36,
    -43, 0, 0, 0, 0, 0, -21, 0,
    0, -11, 0, 0, -61, 0, -61, 0,
    0, -21, 0, 0, 0, 0, 0, 0,
    0, -30, 0, -43, -61, 0, 0, 0,
    0, -8, -3, 0, -20, 0, -18, -30,
    -38, -3, 0, 0, -3, 0, -20, -3,
    -8, -18, 0, 0, -55, -3, -43, 0,
    -3, -20, 0, 0, 0, 0, 0, 0,
    0, -30, 0, -38, -32, 0, -3, 0,
    -38, -14, 0, -46, -6, -46, -29, 0,
    0, -8, -32, -46, -8, -17, -8, 0,
    0, -33, 0, -15, 0, 0, 0, -8,
    0, 0, -61, -5, -79, -38, 0, 0,
    -55, 0, -30, 0, -8, -55, 0, -8,
    0, -14, -8, 0, -24, 0, -29, -49,
    -55, -8, 0, 0, -8, 0, -24, -8,
    -14, -33, 0, 0, -79, -8, -61, 0,
    -8, -24, 0, 0, 0, 0, 0, 0,
    0, -61, 0, -55, -59, 0, 0, 0
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

extern const lv_font_t LV_FONT_MONTSERRAT_18;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t space_ranger_19 = {
#else
lv_font_t space_ranger_19 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 17,          /*The maximum line height required by the font*/
    .base_line = 4,             /*Baseline measured from the bottom of the line*/
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
    .fallback = &LV_FONT_MONTSERRAT_18,
#endif
    .user_data = NULL,
};



#endif /*#if SPACE_RANGER_19*/
