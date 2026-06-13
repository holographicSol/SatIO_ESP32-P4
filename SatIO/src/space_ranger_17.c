/*******************************************************************************
 * Size: 17 px
 * Bpp: 1
 * Opts: --bpp 1 --size 17 --no-compress --stride 1 --align 1 --font spaceranger.ttf --range 32-127 --format lvgl -o space_ranger_17.c
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



#ifndef SPACE_RANGER_17
#define SPACE_RANGER_17 1
#endif

#if SPACE_RANGER_17

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xc3, 0xc0,

    /* U+0022 "\"" */
    0xde, 0xf6,

    /* U+0023 "#" */
    0x31, 0x86, 0x33, 0xff, 0xff, 0xf3, 0x19, 0xff,
    0xff, 0xf9, 0x8c, 0x31, 0x80,

    /* U+0024 "$" */
    0xc, 0x1f, 0xff, 0xff, 0xe6, 0xf, 0xfd, 0xff,
    0x83, 0x38, 0x67, 0xff, 0xff, 0xf8, 0x30, 0x6,
    0x0,

    /* U+0025 "%" */
    0xf9, 0xc3, 0xef, 0xd, 0xb8, 0x3f, 0xe0, 0x7,
    0xfc, 0x1f, 0xf0, 0xec, 0xc3, 0xbf, 0x1c, 0xfc,

    /* U+0026 "&" */
    0x7f, 0x7, 0xf0, 0x60, 0xf, 0xff, 0xff, 0xfc,
    0x1c, 0xc1, 0xcf, 0xfc, 0xff, 0xc0,

    /* U+0027 "'" */
    0xfc,

    /* U+0028 "(" */
    0xff, 0xfc, 0x30, 0xc3, 0xc, 0x30, 0xff, 0xf0,

    /* U+0029 ")" */
    0xff, 0xc6, 0x31, 0x8c, 0x63, 0xff, 0xc0,

    /* U+002A "*" */
    0x6f, 0x60,

    /* U+002B "+" */
    0x30, 0xcf, 0xff, 0x30, 0xc0,

    /* U+002C "," */
    0xfc,

    /* U+002D "-" */
    0xff, 0xc0,

    /* U+002E "." */
    0xf0,

    /* U+002F "/" */
    0xe, 0x18, 0x70, 0xe3, 0x87, 0x1c, 0x38, 0xe0,

    /* U+0030 "0" */
    0xff, 0xff, 0xfc, 0x7f, 0x3b, 0xde, 0xf7, 0x3f,
    0x8f, 0xff, 0xff, 0xc0,

    /* U+0031 "1" */
    0xff, 0x33, 0x33, 0x33, 0x30,

    /* U+0032 "2" */
    0xff, 0xdf, 0xfc, 0x3, 0xff, 0xff, 0xfd, 0x80,
    0x30, 0x7, 0xff, 0xff, 0xe0,

    /* U+0033 "3" */
    0xff, 0x9f, 0xf0, 0x6, 0x7f, 0xef, 0xfc, 0x1,
    0xc0, 0x3f, 0xff, 0xff, 0xc0,

    /* U+0034 "4" */
    0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff, 0xc0, 0x30,
    0xc, 0x3, 0x0, 0xc0,

    /* U+0035 "5" */
    0xff, 0xff, 0xff, 0x0, 0x7f, 0xef, 0xfe, 0x1,
    0xc0, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0036 "6" */
    0xff, 0xff, 0xff, 0x0, 0x7f, 0xef, 0xff, 0x81,
    0xf0, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0037 "7" */
    0xff, 0xff, 0xc1, 0xc0, 0xe0, 0xe0, 0xe0, 0x70,
    0x70, 0x70, 0x0,

    /* U+0038 "8" */
    0x3f, 0x8f, 0xf8, 0x7, 0x7f, 0xef, 0xfd, 0x81,
    0xf0, 0x3f, 0xff, 0xff, 0xc0,

    /* U+0039 "9" */
    0xff, 0xdf, 0xff, 0x3, 0xff, 0xff, 0xfe, 0x1,
    0xc0, 0x3f, 0xff, 0xff, 0xc0,

    /* U+003A ":" */
    0xf0, 0x3c,

    /* U+003B ";" */
    0xf0, 0x3f,

    /* U+003C "<" */
    0xe, 0x38, 0xe3, 0x8e, 0xe, 0xe, 0xe, 0xe,

    /* U+003D "=" */
    0xff, 0xf0, 0x3f, 0xfc,

    /* U+003E ">" */
    0x70, 0xf0, 0xe0, 0xe0, 0xe3, 0x8e, 0x3c, 0x70,

    /* U+003F "?" */
    0xff, 0xdf, 0xfc, 0x3, 0xbf, 0xf7, 0xfc, 0x0,
    0x0, 0x3, 0x0, 0x60, 0x0,

    /* U+0040 "@" */
    0xff, 0xff, 0xff, 0xdf, 0xbd, 0xfb, 0xd9, 0xbd,
    0xff, 0xdf, 0xfc, 0x0, 0xff, 0xff, 0xff,

    /* U+0041 "A" */
    0x3, 0x81, 0xf0, 0xec, 0x73, 0x1c, 0xcf, 0xf7,
    0xfd, 0x83, 0xe0, 0xc0,

    /* U+0042 "B" */
    0xff, 0x9f, 0xf8, 0x7, 0x7f, 0xef, 0xff, 0xc1,
    0xf8, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0043 "C" */
    0xff, 0xff, 0xfc, 0x3, 0x0, 0xc0, 0x30, 0xc,
    0x3, 0xff, 0xff, 0xc0,

    /* U+0044 "D" */
    0xff, 0xff, 0xf0, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0045 "E" */
    0xff, 0xdf, 0xff, 0x3, 0xef, 0xfd, 0xfd, 0x80,
    0x30, 0x7, 0xff, 0xff, 0xe0,

    /* U+0046 "F" */
    0xff, 0xff, 0xfc, 0x3, 0xff, 0xff, 0xf0, 0xc,
    0x3, 0x0, 0xc0, 0x0,

    /* U+0047 "G" */
    0xff, 0xff, 0xff, 0x0, 0x67, 0xec, 0xff, 0x81,
    0xf0, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0048 "H" */
    0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xc0,

    /* U+0049 "I" */
    0xf0, 0xff, 0xc0,

    /* U+004A "J" */
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc, 0x7,
    0xff, 0xff, 0x80,

    /* U+004B "K" */
    0xc1, 0xc6, 0x1c, 0x31, 0xc1, 0xfc, 0xf, 0xe0,
    0x63, 0x83, 0xe, 0x18, 0x70, 0xc1, 0xc0, 0x7,
    0x0, 0x1c, 0x0, 0x70,

    /* U+004C "L" */
    0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0, 0x30, 0xc,
    0x3, 0xff, 0xff, 0xc0,

    /* U+004D "M" */
    0xe0, 0x7f, 0xf, 0xf0, 0xff, 0x9f, 0xf9, 0xfd,
    0xfb, 0xdf, 0xbc, 0xf3, 0xc6, 0x30,

    /* U+004E "N" */
    0xe0, 0xfc, 0x3f, 0x8f, 0xf3, 0xde, 0xf3, 0xfc,
    0x7f, 0xf, 0xc1, 0xc0,

    /* U+004F "O" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0050 "P" */
    0xff, 0xff, 0xf0, 0xc, 0x3, 0xff, 0xff, 0xfc,
    0x3, 0x0, 0xc0, 0x0,

    /* U+0051 "Q" */
    0xff, 0xdf, 0xfb, 0x3, 0x60, 0x6c, 0xd, 0x9d,
    0xb3, 0xb7, 0xfe, 0xff, 0xc0, 0x30, 0x7, 0x0,
    0x60, 0x6,

    /* U+0052 "R" */
    0xff, 0xc7, 0xfe, 0x0, 0x71, 0xff, 0x8f, 0xfc,
    0x63, 0x83, 0xe, 0x18, 0x70, 0xc1, 0xc0, 0x7,
    0x0, 0x18, 0x0, 0xe0, 0x3, 0x80,

    /* U+0053 "S" */
    0x7f, 0xf7, 0xff, 0x60, 0x7, 0xfe, 0x7f, 0xf0,
    0x7, 0x0, 0x7f, 0xfe, 0xff, 0xe0,

    /* U+0054 "T" */
    0xff, 0xff, 0xf0, 0xc0, 0x30, 0xc, 0x3, 0x0,
    0xc0, 0x30, 0xc, 0x0,

    /* U+0055 "U" */
    0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0056 "V" */
    0xe0, 0xdc, 0x37, 0x8c, 0xe3, 0x1c, 0xc3, 0xb0,
    0xfc, 0x1f, 0x3, 0xc0,

    /* U+0057 "W" */
    0xe1, 0x8e, 0xc7, 0x1d, 0xcf, 0x3b, 0x9e, 0x63,
    0x7d, 0xc7, 0xdf, 0x8f, 0xbe, 0xf, 0x3c, 0x1c,
    0x70,

    /* U+0058 "X" */
    0x71, 0xc1, 0xee, 0x3, 0xb8, 0x7, 0xc0, 0xe,
    0x0, 0x7c, 0x3, 0xb8, 0x1e, 0xf0, 0x71, 0xe0,
    0x3, 0x80, 0x7, 0x0, 0xe,

    /* U+0059 "Y" */
    0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff, 0xc0, 0x30,
    0xf, 0xff, 0xff, 0xc0,

    /* U+005A "Z" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xff, 0xff, 0xc0,

    /* U+005B "[" */
    0xff, 0xfc, 0x30, 0xc3, 0xc, 0x30, 0xff, 0xf0,

    /* U+005C "\\" */
    0xe0, 0xe1, 0xc1, 0xc3, 0x83, 0x87, 0x6, 0xe,

    /* U+005D "]" */
    0xff, 0xc6, 0x31, 0x8c, 0x63, 0xff, 0xc0,

    /* U+005E "^" */
    0xff, 0xff, 0xf0, 0xf, 0xff, 0xff, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xc0,

    /* U+005F "_" */
    0xff, 0xff,

    /* U+0060 "`" */
    0x67,

    /* U+0061 "a" */
    0xf, 0x0, 0xf0, 0x1f, 0x81, 0xb8, 0x39, 0xc3,
    0xfc, 0x7f, 0xe6, 0xe, 0xe0, 0x60,

    /* U+0062 "b" */
    0xff, 0x9f, 0xf8, 0x7, 0x7f, 0xef, 0xff, 0xc1,
    0xf8, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0063 "c" */
    0xff, 0xff, 0xfc, 0xf, 0x0, 0xc0, 0x30, 0xc,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0064 "d" */
    0xff, 0xff, 0xf0, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0065 "e" */
    0xff, 0xff, 0xfc, 0x3, 0xff, 0xff, 0xf0, 0xc,
    0x3, 0xff, 0xff, 0xc0,

    /* U+0066 "f" */
    0xff, 0xff, 0xfc, 0x3, 0xff, 0xff, 0xf0, 0xc,
    0x3, 0x0, 0xc0, 0x0,

    /* U+0067 "g" */
    0xff, 0xff, 0xff, 0x0, 0x67, 0xec, 0xff, 0x81,
    0xf0, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0068 "h" */
    0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xc0,

    /* U+0069 "i" */
    0xff, 0xff, 0xc0,

    /* U+006A "j" */
    0x0, 0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+006B "k" */
    0xc1, 0xf0, 0xec, 0x73, 0xf8, 0xfe, 0x31, 0xcc,
    0x73, 0xe, 0xc1, 0xc0,

    /* U+006C "l" */
    0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0, 0x30, 0xc,
    0x3, 0xff, 0xff, 0xc0,

    /* U+006D "m" */
    0xe0, 0x7f, 0xf, 0xf0, 0xff, 0x9f, 0xf9, 0xfd,
    0xfb, 0xdf, 0x3c, 0xf3, 0xc6, 0x30,

    /* U+006E "n" */
    0xe0, 0xfc, 0x3f, 0x8f, 0xf3, 0xde, 0xf3, 0xfc,
    0x7f, 0xf, 0xc1, 0xc0,

    /* U+006F "o" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0070 "p" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf0, 0xc,
    0x3, 0x0, 0xc0, 0x0,

    /* U+0071 "q" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf3, 0xbc,
    0xef, 0xff, 0xff, 0xc0, 0x60, 0x1c,

    /* U+0072 "r" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf1, 0xcc,
    0x3b, 0xe, 0xc1, 0xc0,

    /* U+0073 "s" */
    0xff, 0xff, 0xff, 0x0, 0x7f, 0xef, 0xfe, 0x1,
    0xc0, 0x3f, 0xfe, 0xff, 0xc0,

    /* U+0074 "t" */
    0xff, 0xff, 0xf0, 0xc0, 0x30, 0xc, 0x3, 0x0,
    0xc0, 0x30, 0xc, 0x0,

    /* U+0075 "u" */
    0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0xff, 0xff, 0xc0,

    /* U+0076 "v" */
    0xe0, 0xee, 0x1d, 0xc3, 0x1c, 0xe3, 0x98, 0x3f,
    0x7, 0xe0, 0x78, 0xe, 0x0,

    /* U+0077 "w" */
    0xe1, 0x8e, 0xc7, 0x1d, 0xcf, 0x3b, 0x9e, 0x63,
    0x7d, 0xc7, 0xdf, 0x8f, 0xbe, 0xf, 0x3c, 0x1c,
    0x70,

    /* U+0078 "x" */
    0x71, 0xcf, 0x70, 0xee, 0xf, 0x80, 0xe0, 0x3e,
    0xe, 0xe3, 0xdc, 0x71, 0xc0,

    /* U+0079 "y" */
    0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff, 0xc0, 0x30,
    0xf, 0xff, 0xff, 0xc0,

    /* U+007A "z" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xff, 0xff, 0xc0,

    /* U+007B "{" */
    0x70, 0x3e, 0xd, 0xc3, 0x38, 0xce, 0x3f, 0xcf,
    0xfb, 0x6, 0xc1, 0xc0,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xf0,

    /* U+007D "}" */
    0xc1, 0xf0, 0xec, 0x7b, 0x1c, 0xce, 0x37, 0xf,
    0xc3, 0xe0, 0xf0, 0x0,

    /* U+007E "~" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xc0,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 85, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 57, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 4, .adv_w = 106, .box_w = 5, .box_h = 3, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 6, .adv_w = 187, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 19, .adv_w = 184, .box_w = 11, .box_h = 12, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 36, .adv_w = 236, .box_w = 14, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 52, .adv_w = 197, .box_w = 12, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 66, .adv_w = 57, .box_w = 2, .box_h = 3, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 67, .adv_w = 104, .box_w = 6, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 75, .adv_w = 104, .box_w = 5, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 82, .adv_w = 65, .box_w = 4, .box_h = 3, .ofs_x = 0, .ofs_y = 7},
    {.bitmap_index = 84, .adv_w = 113, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 89, .adv_w = 57, .box_w = 2, .box_h = 3, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 90, .adv_w = 105, .box_w = 5, .box_h = 2, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 92, .adv_w = 57, .box_w = 2, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 93, .adv_w = 117, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 101, .adv_w = 187, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 113, .adv_w = 85, .box_w = 4, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 118, .adv_w = 184, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 131, .adv_w = 182, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 144, .adv_w = 182, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 156, .adv_w = 184, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 169, .adv_w = 184, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 182, .adv_w = 169, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 193, .adv_w = 186, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 206, .adv_w = 184, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 219, .adv_w = 57, .box_w = 2, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 221, .adv_w = 57, .box_w = 2, .box_h = 8, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 223, .adv_w = 118, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 231, .adv_w = 113, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 235, .adv_w = 118, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 243, .adv_w = 182, .box_w = 11, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 256, .adv_w = 211, .box_w = 12, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 271, .adv_w = 184, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 283, .adv_w = 185, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 296, .adv_w = 171, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 308, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 320, .adv_w = 185, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 333, .adv_w = 171, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 345, .adv_w = 187, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 358, .adv_w = 184, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 370, .adv_w = 57, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 373, .adv_w = 171, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 384, .adv_w = 183, .box_w = 13, .box_h = 12, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 404, .adv_w = 171, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 416, .adv_w = 221, .box_w = 12, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 430, .adv_w = 187, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 442, .adv_w = 187, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 454, .adv_w = 176, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 466, .adv_w = 187, .box_w = 11, .box_h = 13, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 484, .adv_w = 176, .box_w = 13, .box_h = 13, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 506, .adv_w = 199, .box_w = 12, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 520, .adv_w = 182, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 532, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 544, .adv_w = 183, .box_w = 10, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 556, .adv_w = 249, .box_w = 15, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 573, .adv_w = 175, .box_w = 14, .box_h = 12, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 594, .adv_w = 182, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 606, .adv_w = 172, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 102, .box_w = 6, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 626, .adv_w = 117, .box_w = 7, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 634, .adv_w = 102, .box_w = 5, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 641, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 653, .adv_w = 151, .box_w = 8, .box_h = 2, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 655, .adv_w = 53, .box_w = 4, .box_h = 2, .ofs_x = -1, .ofs_y = 10},
    {.bitmap_index = 656, .adv_w = 190, .box_w = 12, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 670, .adv_w = 185, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 683, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 695, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 707, .adv_w = 171, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 719, .adv_w = 171, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 731, .adv_w = 186, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 744, .adv_w = 184, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 756, .adv_w = 57, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 759, .adv_w = 183, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 771, .adv_w = 183, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 783, .adv_w = 170, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 795, .adv_w = 221, .box_w = 12, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 809, .adv_w = 187, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 821, .adv_w = 187, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 833, .adv_w = 176, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 845, .adv_w = 187, .box_w = 10, .box_h = 11, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 859, .adv_w = 176, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 871, .adv_w = 184, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 884, .adv_w = 180, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 896, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 908, .adv_w = 187, .box_w = 11, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 921, .adv_w = 249, .box_w = 15, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 938, .adv_w = 175, .box_w = 11, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 951, .adv_w = 182, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 963, .adv_w = 172, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 975, .adv_w = 184, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 987, .adv_w = 57, .box_w = 2, .box_h = 14, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 991, .adv_w = 183, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1003, .adv_w = 186, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1015, .adv_w = 54, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
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
    0, 0, 0, 0, -11, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -12, 0,
    0, 0, 0, 0, -41, 0, -26, 0,
    0, -18, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -33, -26, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -12, 0, 0, 0,
    0, 0, 0, 0, -12, -4, -10, 0,
    0, 0, 0, -7, 0, -7, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -24, -3, 0, -15, 0, 0,
    0, 0, 0, -8, 0, 0, 0, 0,
    0, 0, -48, 0, -41, -18, -3, 0,
    0, 0, 0, 0, 0, -27, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -15, 0, -3, 0, 0, -3, 0, 0,
    0, 0, -19, -3, -24, 0, -3, 0,
    0, 0, -3, 0, -24, -11, -19, 0,
    -15, -11, 0, -16, 0, -16, 0, 0,
    0, 0, 0, 0, -11, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -12, 0,
    0, 0, 0, 0, -41, 0, -26, 0,
    0, -18, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -33, -26, 0, 0, 0,
    0, 0, 0, 0, -19, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -19, 0,
    0, 0, 0, 0, -41, 0, -27, 0,
    0, -18, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -34, -27, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -26, 0, -3, 0,
    0, -7, -3, 0, -26, -11, -19, 0,
    0, 0, 0, -16, 0, -16, 0, 0,
    -27, 0, 0, 0, 0, -24, 0, 0,
    0, 0, -26, -19, 0, -15, 0, 0,
    0, 0, 0, -8, 0, 0, 0, 0,
    0, 0, -33, 0, -44, -15, 0, 0,
    0, 0, 0, 0, 0, -27, 0, 0,
    0, 0, 0, 0, -18, 0, 0, 0,
    0, -3, 0, 0, 0, 0, -19, 0,
    0, 0, 0, 0, -49, -3, -38, 0,
    -4, -18, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -34, -33, 0, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -7, 0, 0, 0,
    0, 0, 0, 0, -7, -3, -4, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    -5, 0, 0, -5, 0, -19, 0, 0,
    -3, 0, -18, -3, 0, -19, 0, 0,
    0, 0, 0, -7, -7, 0, 0, 0,
    0, 0, -19, 0, -22, -15, -4, 0,
    -18, 0, -19, -3, 0, -18, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, -19, 0, -7, -41,
    -41, 0, 0, 0, 0, 0, -19, 0,
    0, 0, 0, 0, -57, 0, -42, 0,
    0, -19, 0, 0, 0, 0, 0, 0,
    0, -42, 0, -45, -34, 0, 0, 0,
    -5, 0, 0, -5, 0, -19, 0, 0,
    -4, 0, -18, 0, 0, -19, 0, 0,
    0, 0, -4, -7, -7, 0, 0, 0,
    0, 0, -19, 0, -22, -15, -4, 0,
    -19, 0, -19, -3, 0, -18, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -7, 0, 0, 0,
    0, 0, 0, 0, -7, 0, -4, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -12, -4, 0, 0,
    0, 0, 0, 0, 0, -7, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, -4, 0, 0, -4, 0, -4, 0,
    0, -23, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -10, 0, 0, 0, 0, 0,
    0, 0, -23, -5, -30, 0, -10, 0,
    0, -12, -10, 0, -30, -12, -23, -10,
    0, 0, 0, -16, 0, -16, 0, 0,
    0, 0, 0, 0, -1, 0, 0, -18,
    -24, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -41, 0, -18, 0,
    0, -18, 0, 0, -5, -3, -4, 0,
    -3, -19, 0, -27, 0, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -5, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    -5, 0, 0, -5, 0, -19, 0, 0,
    -4, 0, -18, -3, 0, -19, 0, 0,
    0, 0, -5, -7, -5, 0, 0, 0,
    0, 0, -19, 0, -22, -15, -4, 0,
    -19, 0, -19, -3, 0, -18, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -3,
    -14, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -24, 0, -3, 0,
    0, -18, 0, 0, 0, 0, 0, 0,
    0, -20, 0, -14, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, -10, 0, -3, -4, 0, -19, 0,
    0, -4, 0, 0, -4, 0, -4, 0,
    0, -23, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    -27, 0, 0, -48, 0, -27, -3, 0,
    0, 0, -24, -19, 0, -15, 0, 0,
    0, -10, 0, -8, 0, 0, 0, 0,
    0, 0, -54, 0, -54, -24, 0, 0,
    -37, 0, -27, 0, 0, -38, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -19,
    -24, 0, 0, 0, 0, 0, -1, 0,
    0, 0, -4, 0, -35, 0, -19, 0,
    0, -18, 0, 0, -7, -3, -4, 0,
    0, -23, 0, -24, -1, -3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, -3, 0, 0,
    -3, 0, 0, 0, 0, -3, 0, 0,
    0, 0, 0, 0, -18, 0, -3, -33,
    -35, 0, 0, 0, 0, 0, -19, 0,
    0, -10, 0, 0, -54, 0, -54, 0,
    0, -19, 0, 0, 0, 0, 0, 0,
    0, -27, 0, -38, -54, 0, 0, 0,
    0, -4, -3, 0, -15, 0, -11, -15,
    -26, -3, 0, 0, -3, 0, -15, 0,
    -4, -12, 0, 0, -34, -3, -24, 0,
    -3, -18, 0, 0, 0, 0, 0, 0,
    0, -19, 0, -26, -19, 0, -3, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -3, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -10, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    0, -3, -22, -24, -3, -15, -3, 0,
    0, 0, 0, -7, 0, 0, 0, -3,
    0, 0, -35, -3, -49, -26, 0, 0,
    0, 0, 0, 0, -3, -34, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -26, -23, 0, -15, 0, 0,
    0, 0, 0, -8, 0, 0, 0, 0,
    0, 0, -27, 0, -54, -19, 0, 0,
    0, 0, 0, 0, 0, -27, 0, 0,
    0, 0, 0, 0, -19, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -19, 0,
    0, 0, 0, 0, -27, 0, -24, 0,
    0, -19, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -27, -24, 0, 0, 0,
    -26, -7, 0, -27, -3, -34, -16, 0,
    0, -3, -24, -27, -3, -15, -3, 0,
    0, -16, 0, -14, 0, 0, 0, -3,
    0, 0, -38, -3, -49, -26, 0, 0,
    -34, 0, -27, 0, -4, -34, 0, 0,
    -5, 0, 0, -5, 0, -19, 0, 0,
    -4, 0, -18, -3, 0, -19, 0, 0,
    0, 0, -5, -7, -5, 0, 0, 0,
    0, 0, -19, 0, -22, -15, -4, 0,
    -18, 0, -19, -3, 0, -18, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -7, 0, 0, 0,
    0, 0, 0, 0, 0, -3, -4, 0,
    -3, 0, 0, -3, 0, -3, 0, 0,
    -27, 0, 0, -48, 0, -27, -3, 0,
    0, 0, -24, -19, 0, -15, 0, 0,
    0, -10, 0, -8, 0, 0, 0, 0,
    0, 0, -54, 0, -54, -24, 0, 0,
    -37, 0, -27, 0, 0, -39, 0, 0,
    0, 0, 0, 0, -18, 0, -3, -33,
    -38, 0, 0, 0, 0, 0, -19, 0,
    0, -10, 0, 0, -54, 0, -54, 0,
    0, -19, 0, 0, 0, 0, 0, 0,
    0, -27, 0, -38, -54, 0, 0, 0,
    0, -7, -3, 0, -18, 0, -16, -27,
    -34, -3, 0, 0, -3, 0, -18, -3,
    -7, -16, 0, 0, -49, -3, -38, 0,
    -3, -18, 0, 0, 0, 0, 0, 0,
    0, -27, 0, -34, -29, 0, -3, 0,
    -34, -12, 0, -41, -5, -41, -26, 0,
    0, -7, -29, -41, -7, -15, -7, 0,
    0, -30, 0, -14, 0, 0, 0, -7,
    0, 0, -54, -4, -71, -34, 0, 0,
    -49, 0, -27, 0, -7, -49, 0, -7,
    0, -12, -7, 0, -22, 0, -26, -44,
    -49, -7, 0, 0, -7, 0, -22, -7,
    -12, -30, 0, 0, -71, -7, -54, 0,
    -7, -22, 0, 0, 0, 0, 0, 0,
    0, -54, 0, -49, -53, 0, 0, 0
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

extern const lv_font_t lv_font_montserrat_16;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t space_ranger_17 = {
#else
lv_font_t space_ranger_17 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 16,          /*The maximum line height required by the font*/
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
    .fallback = &lv_font_montserrat_16,
#endif
    .user_data = NULL,
};



#endif /*#if SPACE_RANGER_17*/
