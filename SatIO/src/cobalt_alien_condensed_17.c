/*******************************************************************************
 * Size: 17 px
 * Bpp: 1
 * Opts: --bpp 1 --size 17 --no-compress --stride 1 --align 1 --font cobaltaliencond.ttf --range 32-127 --format lvgl -o cobalt_alien_condensed_17.c
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



#ifndef COBALT_ALIEN_CONDENSED_17
#define COBALT_ALIEN_CONDENSED_17 1
#endif

#if COBALT_ALIEN_CONDENSED_17

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
    0x77, 0x77, 0xff, 0xff, 0x77, 0x77, 0xff, 0xff,
    0x77, 0x77,

    /* U+0024 "$" */
    0x28, 0x51, 0xff, 0xfe, 0x9f, 0xf, 0x97, 0xff,
    0xfc, 0xa1, 0x40,

    /* U+0025 "%" */
    0xf8, 0x7, 0xce, 0x36, 0x71, 0xb7, 0xf, 0xb8,
    0x7f, 0x80, 0x1f, 0xe1, 0xdf, 0xe, 0xd8, 0xe6,
    0xc6, 0x3e, 0x1, 0xf0,

    /* U+0026 "&" */
    0xfc, 0xfe, 0xee, 0xe0, 0xff, 0xff, 0xee, 0xee,
    0xfe, 0xfe,

    /* U+0027 "'" */
    0xff, 0xe0,

    /* U+0028 "(" */
    0xff, 0xf9, 0xce, 0x73, 0x9c, 0xe7, 0x3f, 0xf0,

    /* U+0029 ")" */
    0xff, 0xf1, 0xc7, 0x1c, 0x71, 0xc7, 0x1c, 0x7f,
    0xfe,

    /* U+002A "*" */
    0x4d, 0x80,

    /* U+002B "+" */
    0x21, 0x3f, 0xf2, 0x10,

    /* U+002C "," */
    0xff, 0xe0,

    /* U+002D "-" */
    0xfc,

    /* U+002E "." */
    0xfc,

    /* U+002F "/" */
    0x7, 0x7, 0xe, 0xc, 0x1c, 0x18, 0x38, 0x30,
    0x70, 0x60,

    /* U+0030 "0" */
    0xff, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xf7, 0xff,
    0xfc,

    /* U+0031 "1" */
    0xff, 0x77, 0x77, 0x77, 0x77,

    /* U+0032 "2" */
    0xff, 0xfc, 0x38, 0x77, 0xff, 0xb8, 0x70, 0xff,
    0xfc,

    /* U+0033 "3" */
    0xff, 0xfc, 0x38, 0x77, 0xef, 0xc3, 0x87, 0xff,
    0xfc,

    /* U+0034 "4" */
    0x37, 0x77, 0x77, 0x67, 0x67, 0xff, 0xff, 0x7,
    0x7, 0x7,

    /* U+0035 "5" */
    0xff, 0xff, 0x87, 0xf, 0xcf, 0xc3, 0x87, 0xff,
    0xfc,

    /* U+0036 "6" */
    0xff, 0xff, 0x87, 0xf, 0xff, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0037 "7" */
    0xff, 0xff, 0x7, 0x7, 0x3f, 0x7e, 0x70, 0x70,
    0x70, 0x70,

    /* U+0038 "8" */
    0xff, 0xff, 0xbf, 0x7f, 0xff, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0039 "9" */
    0xff, 0xff, 0xbf, 0x7e, 0xff, 0xff, 0x87, 0xff,
    0xfc,

    /* U+003A ":" */
    0xfc, 0x0, 0x3f,

    /* U+003B ";" */
    0xfc, 0x1, 0xff, 0xc0,

    /* U+003C "<" */
    0x39, 0xdc, 0xce, 0x71, 0x8e, 0x39, 0xc0,

    /* U+003D "=" */
    0xff, 0xc1, 0xff, 0x80,

    /* U+003E ">" */
    0xe7, 0x1c, 0x63, 0x9c, 0xce, 0xe7, 0x0,

    /* U+003F "?" */
    0xff, 0xfc, 0x38, 0x73, 0xef, 0xde, 0x0, 0x70,
    0xe0,

    /* U+0040 "@" */
    0xff, 0xff, 0xff, 0xfe, 0x1, 0xff, 0xf7, 0xff,
    0xdf, 0xf7, 0x7f, 0xdd, 0xff, 0xff, 0xff, 0xff,
    0x80, 0xf, 0xff, 0xff, 0xff,

    /* U+0041 "A" */
    0xff, 0xff, 0x87, 0x7e, 0xff, 0xff, 0xf7, 0xef,
    0xdc,

    /* U+0042 "B" */
    0xff, 0xfc, 0x3f, 0x7f, 0xff, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0043 "C" */
    0xff, 0xff, 0x87, 0xe, 0x1c, 0x38, 0x70, 0xff,
    0xfc,

    /* U+0044 "D" */
    0xff, 0xfc, 0x3f, 0x7e, 0xfd, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0045 "E" */
    0xff, 0xff, 0x87, 0xf, 0xdf, 0xb8, 0x70, 0xff,
    0xfc,

    /* U+0046 "F" */
    0xff, 0xff, 0x87, 0xf, 0xdf, 0xb8, 0x70, 0xe1,
    0xc0,

    /* U+0047 "G" */
    0xff, 0xff, 0x87, 0xe, 0xfd, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0048 "H" */
    0xef, 0xdf, 0xbf, 0x7f, 0xff, 0xfb, 0xf7, 0xef,
    0xdc,

    /* U+0049 "I" */
    0xff, 0xff, 0xff, 0xfc,

    /* U+004A "J" */
    0x1c, 0x71, 0xc7, 0x1c, 0x71, 0xc7, 0xff, 0xe0,

    /* U+004B "K" */
    0xe7, 0xef, 0xec, 0xfc, 0xfc, 0xfc, 0xfc, 0xec,
    0xee, 0xe6,

    /* U+004C "L" */
    0xe7, 0x39, 0xce, 0x73, 0x9c, 0xff, 0xc0,

    /* U+004D "M" */
    0xc0, 0x7c, 0x1f, 0xc7, 0xf8, 0xff, 0xbf, 0xff,
    0xfb, 0xbf, 0x77, 0xe4, 0xfc, 0x1c,

    /* U+004E "N" */
    0xc7, 0xe7, 0xf7, 0xf7, 0xff, 0xff, 0xef, 0xef,
    0xe7, 0xe2,

    /* U+004F "O" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0050 "P" */
    0xff, 0xfc, 0x3f, 0x7e, 0xff, 0xff, 0xf0, 0xe1,
    0xc0,

    /* U+0051 "Q" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xfb, 0xf7, 0xff,
    0xfc, 0x38, 0x70,

    /* U+0052 "R" */
    0xff, 0xfc, 0x3f, 0x7e, 0xff, 0xff, 0x76, 0xed,
    0xdc,

    /* U+0053 "S" */
    0xff, 0xff, 0x87, 0xf, 0xcf, 0xc3, 0x87, 0xff,
    0xfc,

    /* U+0054 "T" */
    0xff, 0xfc, 0xe1, 0xc3, 0x87, 0xe, 0x1c, 0x38,
    0x70,

    /* U+0055 "U" */
    0xef, 0xdf, 0xbf, 0x7e, 0xfd, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+0056 "V" */
    0xe7, 0xe7, 0x76, 0x76, 0x36, 0x3e, 0x3e, 0x3c,
    0x3c, 0x1c,

    /* U+0057 "W" */
    0xe7, 0x7f, 0x3b, 0x9d, 0xcc, 0xef, 0x63, 0x7f,
    0x1f, 0xf8, 0xff, 0x87, 0xbc, 0x3d, 0xe0, 0xef,
    0x0,

    /* U+0058 "X" */
    0xe7, 0xff, 0x7e, 0x3c, 0x3c, 0x3c, 0x3c, 0x7e,
    0x7e, 0xe7,

    /* U+0059 "Y" */
    0xf7, 0xf7, 0x3e, 0x3e, 0x1c, 0x1c, 0x1c, 0x1c,
    0x1c, 0x1c,

    /* U+005A "Z" */
    0xff, 0xfc, 0x38, 0x77, 0xff, 0x38, 0x70, 0xff,
    0xfc,

    /* U+005B "[" */
    0xff, 0xf9, 0xce, 0x73, 0x9c, 0xe7, 0x3f, 0xf0,

    /* U+005C "\\" */
    0xe0, 0xe0, 0x70, 0x30, 0x38, 0x18, 0x1c, 0xc,
    0xe, 0x6,

    /* U+005D "]" */
    0xff, 0xce, 0x73, 0x9c, 0xe7, 0x39, 0xff, 0xf0,

    /* U+005E "^" */
    0xff, 0xfc, 0x3f, 0x7e, 0xff, 0xff, 0xf7, 0xef,
    0xdc,

    /* U+005F "_" */
    0xff, 0xc0,

    /* U+0060 "`" */
    0x4c,

    /* U+0061 "a" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xff, 0xff,

    /* U+0062 "b" */
    0xe1, 0xc3, 0xff, 0xfe, 0xfd, 0xfb, 0x87, 0xff,
    0xfc,

    /* U+0063 "c" */
    0xff, 0xff, 0x87, 0xe, 0x1c, 0x3f, 0xff,

    /* U+0064 "d" */
    0xe, 0x1f, 0xff, 0xfe, 0xfd, 0xfb, 0xf0, 0xff,
    0xfc,

    /* U+0065 "e" */
    0xff, 0xff, 0xbf, 0x7e, 0xfc, 0x3f, 0xff,

    /* U+0066 "f" */
    0xff, 0xff, 0x87, 0xf, 0xdf, 0xb8, 0x70, 0xe1,
    0xc0,

    /* U+0067 "g" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xff, 0xff, 0xe,
    0x3c, 0x78,

    /* U+0068 "h" */
    0xe1, 0xc3, 0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef,
    0xdc,

    /* U+0069 "i" */
    0xff, 0xff, 0xff,

    /* U+006A "j" */
    0x1c, 0x71, 0xc7, 0x1c, 0x71, 0xc7, 0x1f, 0xff,
    0x80,

    /* U+006B "k" */
    0xe0, 0xe0, 0xe6, 0xec, 0xfc, 0xf8, 0xfc, 0xec,
    0xef, 0xe7,

    /* U+006C "l" */
    0xff, 0xff, 0xff, 0xfc,

    /* U+006D "m" */
    0xe0, 0xfe, 0x3f, 0xc7, 0xfd, 0xff, 0xff, 0xdd,
    0xf9, 0x3f, 0x7,

    /* U+006E "n" */
    0x67, 0xe7, 0xf7, 0xff, 0xff, 0xef, 0xe7, 0xe6,

    /* U+006F "o" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xff, 0xff,

    /* U+0070 "p" */
    0xff, 0xfc, 0x3f, 0x7e, 0xfd, 0xff, 0xff, 0xe1,
    0xc0,

    /* U+0071 "q" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xff, 0xff, 0xe,
    0x1c,

    /* U+0072 "r" */
    0xff, 0xf9, 0xce, 0x73, 0x9c,

    /* U+0073 "s" */
    0xff, 0xff, 0x87, 0xc3, 0xe1, 0xff, 0xff,

    /* U+0074 "t" */
    0xe3, 0x8f, 0xff, 0xe3, 0x8e, 0x38, 0xfb, 0xe0,

    /* U+0075 "u" */
    0xef, 0xdf, 0xbf, 0x7e, 0xfd, 0xff, 0xff,

    /* U+0076 "v" */
    0xe7, 0xf7, 0x76, 0x36, 0x3e, 0x3e, 0x3c, 0x1c,

    /* U+0077 "w" */
    0xe6, 0x36, 0x77, 0x6f, 0x66, 0xf6, 0x7f, 0x63,
    0xfe, 0x3d, 0xc3, 0x9c,

    /* U+0078 "x" */
    0x66, 0x7e, 0x3c, 0x3c, 0x3c, 0x3c, 0x7e, 0x66,

    /* U+0079 "y" */
    0xef, 0xdf, 0xbf, 0x7e, 0xfd, 0xff, 0xff, 0xe,
    0x3c, 0x78,

    /* U+007A "z" */
    0xff, 0xfc, 0x39, 0xff, 0x9c, 0x3f, 0xff,

    /* U+007B "{" */
    0xff, 0xff, 0xbf, 0x7e, 0xfd, 0xff, 0xff,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xfe,

    /* U+007D "}" */
    0xff, 0xff, 0xbf, 0x7e, 0xfc, 0x3f, 0xff,

    /* U+007E "~" */
    0xff, 0xff, 0x87, 0xf, 0xff, 0xfb, 0xf7, 0xff,
    0xfc,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 62, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 58, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 5, .adv_w = 101, .box_w = 6, .box_h = 4, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 8, .adv_w = 151, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 18, .adv_w = 126, .box_w = 7, .box_h = 12, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 29, .adv_w = 218, .box_w = 13, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 49, .adv_w = 136, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 59, .adv_w = 58, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 61, .adv_w = 95, .box_w = 5, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 69, .adv_w = 95, .box_w = 6, .box_h = 12, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 78, .adv_w = 57, .box_w = 3, .box_h = 3, .ofs_x = 0, .ofs_y = 8},
    {.bitmap_index = 80, .adv_w = 91, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 84, .adv_w = 58, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 86, .adv_w = 70, .box_w = 3, .box_h = 2, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 87, .adv_w = 58, .box_w = 3, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 88, .adv_w = 134, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 98, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 107, .adv_w = 69, .box_w = 4, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 112, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 121, .adv_w = 125, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 130, .adv_w = 133, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 140, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 149, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 158, .adv_w = 126, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 168, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 177, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 186, .adv_w = 58, .box_w = 3, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 189, .adv_w = 58, .box_w = 3, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 193, .adv_w = 96, .box_w = 5, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 200, .adv_w = 90, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 204, .adv_w = 96, .box_w = 5, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 211, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 220, .adv_w = 237, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 241, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 250, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 259, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 268, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 277, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 286, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 295, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 304, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 313, .adv_w = 58, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 317, .adv_w = 95, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 325, .adv_w = 133, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 335, .adv_w = 95, .box_w = 5, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 342, .adv_w = 188, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 356, .adv_w = 144, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 366, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 375, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 384, .adv_w = 133, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 395, .adv_w = 134, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 404, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 413, .adv_w = 125, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 422, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 431, .adv_w = 139, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 441, .adv_w = 215, .box_w = 13, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 458, .adv_w = 137, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 468, .adv_w = 134, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 478, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 487, .adv_w = 93, .box_w = 5, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 495, .adv_w = 134, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 505, .adv_w = 93, .box_w = 5, .box_h = 12, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 513, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 522, .adv_w = 103, .box_w = 5, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 524, .adv_w = 55, .box_w = 3, .box_h = 2, .ofs_x = 0, .ofs_y = 9},
    {.bitmap_index = 525, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 532, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 541, .adv_w = 126, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 548, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 557, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 564, .adv_w = 126, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 573, .adv_w = 133, .box_w = 7, .box_h = 11, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 583, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 592, .adv_w = 58, .box_w = 3, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 595, .adv_w = 58, .box_w = 6, .box_h = 11, .ofs_x = -2, .ofs_y = -3},
    {.bitmap_index = 604, .adv_w = 133, .box_w = 8, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 614, .adv_w = 58, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 187, .box_w = 11, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 629, .adv_w = 144, .box_w = 8, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 637, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 644, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 653, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 662, .adv_w = 94, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 667, .adv_w = 126, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 674, .adv_w = 101, .box_w = 6, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 682, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 689, .adv_w = 140, .box_w = 8, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 697, .adv_w = 198, .box_w = 12, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 709, .adv_w = 127, .box_w = 8, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 717, .adv_w = 133, .box_w = 7, .box_h = 11, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 727, .adv_w = 126, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 734, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 741, .adv_w = 58, .box_w = 3, .box_h = 13, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 746, .adv_w = 133, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 753, .adv_w = 133, .box_w = 7, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 762, .adv_w = 43, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
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


/*Pair left and right glyphs for kerning*/
static const uint8_t kern_pair_glyph_ids[] =
{
    3, 43,
    3, 84,
    7, 18,
    7, 24,
    7, 53,
    7, 55,
    7, 56,
    7, 57,
    7, 58,
    8, 43,
    8, 83,
    8, 84,
    12, 18,
    12, 20,
    12, 24,
    12, 53,
    12, 55,
    12, 56,
    12, 57,
    12, 58,
    12, 87,
    12, 88,
    12, 89,
    13, 18,
    13, 24,
    13, 53,
    13, 55,
    13, 56,
    13, 58,
    13, 87,
    13, 88,
    14, 18,
    14, 20,
    14, 24,
    14, 53,
    14, 55,
    14, 56,
    14, 57,
    14, 58,
    14, 87,
    14, 88,
    14, 89,
    15, 18,
    15, 24,
    15, 53,
    15, 55,
    15, 56,
    15, 58,
    15, 87,
    15, 88,
    16, 16,
    16, 21,
    16, 43,
    24, 13,
    24, 15,
    24, 43,
    29, 29,
    30, 18,
    31, 18,
    31, 31,
    31, 53,
    31, 55,
    31, 56,
    31, 58,
    36, 12,
    36, 14,
    36, 21,
    38, 12,
    38, 14,
    38, 21,
    39, 12,
    39, 13,
    39, 14,
    39, 15,
    39, 16,
    39, 21,
    39, 43,
    39, 64,
    39, 66,
    39, 68,
    39, 69,
    39, 70,
    39, 72,
    39, 78,
    39, 79,
    39, 80,
    39, 82,
    39, 83,
    39, 84,
    44, 12,
    44, 14,
    45, 3,
    45, 8,
    45, 11,
    45, 24,
    45, 32,
    45, 53,
    45, 55,
    45, 56,
    45, 58,
    45, 61,
    45, 87,
    45, 88,
    49, 13,
    49, 15,
    49, 16,
    49, 43,
    49, 64,
    53, 12,
    53, 13,
    53, 14,
    53, 15,
    53, 16,
    53, 21,
    53, 29,
    53, 43,
    53, 64,
    53, 66,
    53, 68,
    53, 70,
    53, 71,
    53, 72,
    53, 78,
    53, 79,
    53, 80,
    53, 82,
    53, 83,
    53, 84,
    53, 92,
    53, 94,
    55, 12,
    55, 13,
    55, 14,
    55, 15,
    55, 16,
    55, 21,
    55, 29,
    55, 34,
    55, 36,
    55, 38,
    55, 39,
    55, 40,
    55, 43,
    55, 48,
    55, 52,
    55, 64,
    55, 66,
    55, 69,
    55, 70,
    55, 71,
    55, 72,
    55, 78,
    55, 79,
    55, 80,
    55, 82,
    55, 83,
    55, 84,
    55, 92,
    55, 94,
    56, 12,
    56, 13,
    56, 14,
    56, 15,
    56, 16,
    56, 21,
    56, 29,
    56, 34,
    56, 36,
    56, 38,
    56, 39,
    56, 40,
    56, 43,
    56, 48,
    56, 50,
    56, 52,
    56, 64,
    56, 66,
    56, 68,
    56, 69,
    56, 70,
    56, 71,
    56, 72,
    56, 78,
    56, 79,
    56, 80,
    56, 82,
    56, 83,
    56, 84,
    56, 92,
    56, 94,
    57, 12,
    57, 14,
    57, 66,
    57, 68,
    57, 69,
    57, 70,
    57, 72,
    57, 80,
    57, 82,
    57, 94,
    58, 12,
    58, 13,
    58, 14,
    58, 15,
    58, 16,
    58, 21,
    58, 29,
    58, 34,
    58, 36,
    58, 38,
    58, 39,
    58, 40,
    58, 43,
    58, 48,
    58, 50,
    58, 52,
    58, 64,
    58, 66,
    58, 68,
    58, 69,
    58, 70,
    58, 71,
    58, 72,
    58, 74,
    58, 75,
    58, 78,
    58, 79,
    58, 80,
    58, 81,
    58, 82,
    58, 83,
    58, 84,
    58, 86,
    58, 87,
    58, 88,
    58, 89,
    58, 90,
    58, 91,
    58, 92,
    58, 94,
    61, 18,
    61, 53,
    61, 55,
    61, 56,
    61, 58,
    61, 61,
    64, 53,
    64, 55,
    64, 56,
    64, 58,
    64, 87,
    64, 88,
    67, 53,
    67, 55,
    67, 56,
    67, 57,
    67, 58,
    67, 88,
    67, 89,
    70, 53,
    70, 55,
    70, 56,
    70, 58,
    70, 88,
    71, 12,
    71, 13,
    71, 14,
    71, 15,
    71, 16,
    71, 43,
    71, 64,
    72, 53,
    72, 55,
    72, 56,
    72, 58,
    72, 88,
    73, 53,
    73, 55,
    73, 56,
    73, 58,
    73, 88,
    76, 12,
    76, 14,
    78, 53,
    78, 55,
    78, 56,
    78, 88,
    80, 53,
    80, 55,
    80, 56,
    80, 57,
    80, 58,
    80, 88,
    80, 89,
    81, 53,
    81, 55,
    81, 56,
    81, 57,
    81, 58,
    81, 88,
    82, 53,
    82, 55,
    82, 56,
    82, 58,
    82, 88,
    83, 12,
    83, 13,
    83, 14,
    83, 15,
    83, 16,
    83, 43,
    83, 64,
    85, 12,
    85, 13,
    85, 14,
    85, 15,
    85, 43,
    85, 64,
    87, 12,
    87, 13,
    87, 14,
    87, 15,
    87, 43,
    87, 64,
    87, 66,
    87, 68,
    87, 69,
    87, 70,
    87, 71,
    87, 72,
    87, 78,
    87, 79,
    87, 80,
    87, 82,
    87, 84,
    88, 12,
    88, 13,
    88, 14,
    88, 15,
    88, 43,
    88, 64,
    88, 66,
    88, 68,
    88, 69,
    88, 70,
    88, 71,
    88, 72,
    88, 78,
    88, 79,
    88, 80,
    88, 82,
    88, 84,
    89, 12,
    89, 14,
    89, 66,
    89, 68,
    89, 69,
    89, 70,
    89, 72,
    89, 80,
    89, 82,
    92, 53,
    92, 55,
    92, 56,
    92, 58,
    94, 53,
    94, 55,
    94, 56,
    94, 58
};

/* Kerning between the respective left and right glyphs
 * 4.4 format which needs to scaled with `kern_scale`*/
static const int8_t kern_pair_values[] =
{
    -30, -4, -6, -7, -9, -10, -10, -6,
    -10, -30, -4, -4, -10, -12, -5, -18,
    -20, -20, -20, -20, -3, -9, -15, -10,
    -5, -28, -30, -30, -35, -33, -16, -10,
    -12, -5, -18, -20, -20, -20, -20, -3,
    -8, -15, -10, -5, -28, -30, -30, -35,
    -33, -16, -56, -15, -33, -27, -27, -30,
    -17, -5, -9, -17, -16, -20, -20, -20,
    -12, -12, -7, -12, -12, -7, -12, -17,
    -12, -17, -34, -7, -30, -23, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -20, -20, -30, -30, -30, -4, -7,
    -30, -35, -35, -35, -33, -33, -21, -5,
    -5, -24, -28, -19, -18, -28, -18, -28,
    -29, -11, -15, -30, -25, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -7, -16, -7, -16, -25, -16,
    -13, -2, -2, -2, -2, -2, -23, -2,
    -2, -18, -7, -7, -7, -7, -7, -7,
    -7, -7, -7, -7, -7, -7, -7, -7,
    -16, -7, -16, -25, -16, -13, -2, -2,
    -2, -2, -2, -23, -2, -2, -2, -18,
    -7, -7, -7, -7, -7, -7, -7, -7,
    -7, -7, -7, -7, -7, -7, -20, -20,
    -5, -5, -5, -5, -5, -5, -5, -5,
    -14, -31, -14, -31, -31, -16, -24, -4,
    -4, -4, -4, -4, -23, -4, -4, -4,
    -27, -14, -14, -14, -14, -14, -14, -4,
    -4, -14, -14, -14, -3, -14, -14, -14,
    -4, -5, -4, -4, -4, -4, -10, -11,
    -10, -30, -42, -42, -35, -56, -25, -35,
    -35, -33, -37, -16, -5, -5, -5, -5,
    -5, -4, -3, -5, -5, -5, -5, -3,
    -2, -13, -2, -13, -24, -20, -14, -5,
    -5, -5, -5, -3, -5, -5, -5, -5,
    -3, -15, -15, -5, -5, -5, -4, -5,
    -5, -5, -5, -5, -2, -3, -5, -5,
    -5, -5, -5, -2, -5, -5, -5, -5,
    -2, -2, -25, -2, -25, -25, -30, -22,
    -4, -7, -4, -7, -6, -4, -8, -17,
    -8, -18, -21, -16, -4, -4, -3, -3,
    -2, -3, -3, -3, -3, -2, -3, -9,
    -16, -8, -16, -21, -16, -4, -4, -3,
    -3, -3, -3, -3, -4, -3, -2, -3,
    -15, -15, -2, -2, -2, -2, -2, -3,
    -2, -5, -5, -5, -5, -5, -5, -5,
    -5
};

/*Collect the kern pair's data in one place*/
static const lv_font_fmt_txt_kern_pair_t kern_pairs =
{
    .glyph_ids = kern_pair_glyph_ids,
    .values = kern_pair_values,
    .pair_cnt = 369,
    .glyph_ids_size = 0
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
    .kern_dsc = &kern_pairs,
    .kern_scale = 16,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif

};

extern const lv_font_t lv_font_montserrat_16;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t cobalt_alien_condensed_17 = {
#else
lv_font_t cobalt_alien_condensed_17 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 14,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
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
    .fallback = &lv_font_montserrat_16,
#endif
    .user_data = NULL,
};



#endif /*#if COBALT_ALIEN_CONDENSED_17*/
