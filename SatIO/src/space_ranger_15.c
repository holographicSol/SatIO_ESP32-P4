/*******************************************************************************
 * Size: 15 px
 * Bpp: 1
 * Opts: --bpp 1 --size 15 --no-compress --stride 1 --align 1 --font spaceranger.ttf --range 32-127 --format lvgl -o space_ranger_15.c
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



#ifndef SPACE_RANGER_15
#define SPACE_RANGER_15 1
#endif

#if SPACE_RANGER_15

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xcf,

    /* U+0022 "\"" */
    0xde, 0xf6,

    /* U+0023 "#" */
    0x33, 0x19, 0xbf, 0xff, 0xff, 0xff, 0xfc, 0xcc,
    0x66,

    /* U+0024 "$" */
    0xc, 0x1f, 0xff, 0xff, 0xb0, 0xff, 0x9f, 0xf0,
    0xdf, 0xff, 0xff, 0x83, 0x0, 0xc0,

    /* U+0025 "%" */
    0xfb, 0x8f, 0xb0, 0xff, 0xf, 0xf0, 0xf, 0xf0,
    0xff, 0xd, 0xf1, 0xdf,

    /* U+0026 "&" */
    0x7f, 0x1f, 0xc6, 0x3, 0xff, 0xff, 0xf0, 0x6f,
    0xfb, 0xfe,

    /* U+0027 "'" */
    0xfc,

    /* U+0028 "(" */
    0xff, 0xf1, 0x8c, 0x63, 0x1f, 0xf8,

    /* U+0029 ")" */
    0xff, 0xc6, 0x31, 0x8c, 0x7f, 0xf8,

    /* U+002A "*" */
    0x5d, 0x80,

    /* U+002B "+" */
    0x37, 0xfe, 0x63, 0x0,

    /* U+002C "," */
    0xfc,

    /* U+002D "-" */
    0xff, 0xc0,

    /* U+002E "." */
    0xf0,

    /* U+002F "/" */
    0xc, 0x71, 0x8e, 0x31, 0xc6, 0x38,

    /* U+0030 "0" */
    0xff, 0xff, 0xf3, 0xf9, 0xbd, 0x9f, 0xcf, 0xff,
    0xff,

    /* U+0031 "1" */
    0xff, 0x33, 0x33, 0x33,

    /* U+0032 "2" */
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0xfe, 0x3, 0xff,
    0xff,

    /* U+0033 "3" */
    0xff, 0x7f, 0x80, 0xdf, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+0034 "4" */
    0xc1, 0xe0, 0xf0, 0x7f, 0xff, 0xf8, 0xc, 0x6,
    0x3,

    /* U+0035 "5" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+0036 "6" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfe, 0xf, 0xff,
    0xff,

    /* U+0037 "7" */
    0xff, 0xff, 0x6, 0xe, 0x1c, 0x18, 0x38, 0x70,

    /* U+0038 "8" */
    0x7f, 0x1f, 0xe0, 0x3b, 0xfe, 0xff, 0xf0, 0x7f,
    0xff, 0xfe,

    /* U+0039 "9" */
    0xff, 0xff, 0xf0, 0x7f, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+003A ":" */
    0xf0, 0xf0,

    /* U+003B ";" */
    0xf0, 0xfc,

    /* U+003C "<" */
    0x1c, 0xe3, 0x18, 0x60, 0xc3, 0x87,

    /* U+003D "=" */
    0xff, 0xc1, 0xff, 0x80,

    /* U+003E ">" */
    0x61, 0xc3, 0x87, 0x1c, 0xe7, 0x18,

    /* U+003F "?" */
    0xff, 0xbf, 0xf0, 0x1d, 0xff, 0x7f, 0x80, 0x6,
    0x1, 0x80,

    /* U+0040 "@" */
    0xff, 0xff, 0xff, 0x7d, 0xef, 0xbd, 0xff, 0xbf,
    0xf0, 0x7, 0xff, 0xff, 0xe0,

    /* U+0041 "A" */
    0x7, 0x87, 0xc3, 0x63, 0x33, 0x9b, 0xfd, 0xff,
    0xc3,

    /* U+0042 "B" */
    0xff, 0x7f, 0x80, 0xdf, 0xff, 0xfe, 0xf, 0xff,
    0xff,

    /* U+0043 "C" */
    0xff, 0xff, 0xf0, 0x18, 0xc, 0x6, 0x3, 0xff,
    0xff,

    /* U+0044 "D" */
    0xff, 0xff, 0xc0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0045 "E" */
    0xff, 0xbf, 0xfc, 0x1f, 0x7f, 0xdf, 0xb0, 0xf,
    0xff, 0xff,

    /* U+0046 "F" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfe, 0x3, 0x1,
    0x80,

    /* U+0047 "G" */
    0xff, 0xff, 0xfc, 0x3, 0x7e, 0xdf, 0xf0, 0x7f,
    0xff, 0xfe,

    /* U+0048 "H" */
    0xc1, 0xe0, 0xf0, 0x7f, 0xff, 0xfe, 0xf, 0x7,
    0x83,

    /* U+0049 "I" */
    0xf3, 0xff,

    /* U+004A "J" */
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xf, 0xff,
    0xff,

    /* U+004B "K" */
    0xc3, 0x8c, 0x70, 0xce, 0xf, 0xc0, 0xfc, 0xc,
    0xe0, 0xc7, 0xc, 0x38, 0x1, 0xc0, 0xe, 0x0,
    0x60,

    /* U+004C "L" */
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x6, 0x3, 0xff,
    0xff,

    /* U+004D "M" */
    0xe0, 0xfc, 0x1f, 0xc7, 0xfd, 0xfd, 0xb7, 0xbe,
    0xf3, 0x9e, 0x73,

    /* U+004E "N" */
    0xc1, 0xf8, 0xfe, 0x7f, 0xbd, 0xfe, 0x7f, 0x1f,
    0x83,

    /* U+004F "O" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0050 "P" */
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0xfe, 0x3, 0x1,
    0x80,

    /* U+0051 "Q" */
    0xff, 0xbf, 0xec, 0x1b, 0x6, 0xdd, 0xb3, 0x6f,
    0xfb, 0xfe, 0x3, 0x0, 0x60, 0x1c,

    /* U+0052 "R" */
    0xff, 0x9f, 0xf0, 0x6, 0x7f, 0xcf, 0xf9, 0x8c,
    0x31, 0xc6, 0x1c, 0x1, 0xc0, 0x18, 0x1, 0x80,

    /* U+0053 "S" */
    0x7f, 0xdf, 0xf6, 0x1, 0xff, 0x7f, 0xc0, 0x3f,
    0xff, 0xff,

    /* U+0054 "T" */
    0xff, 0xff, 0xc3, 0x1, 0x80, 0xc0, 0x60, 0x30,
    0x18,

    /* U+0055 "U" */
    0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0056 "V" */
    0xe1, 0xb0, 0xdc, 0x67, 0x31, 0x98, 0x6c, 0x3e,
    0xf,

    /* U+0057 "W" */
    0xe3, 0x19, 0x8c, 0x66, 0x7b, 0x9d, 0xec, 0x3f,
    0xb0, 0xf7, 0xc3, 0xcf, 0x6, 0x38,

    /* U+0058 "X" */
    0x63, 0x87, 0x70, 0x3e, 0x1, 0xe0, 0x1e, 0x3,
    0xe0, 0x73, 0x6, 0x38, 0x1, 0xc0, 0xe, 0x0,
    0x70,

    /* U+0059 "Y" */
    0xc1, 0xe0, 0xf0, 0x7f, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+005A "Z" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,
    0xff,

    /* U+005B "[" */
    0xff, 0xf1, 0x8c, 0x63, 0x1f, 0xf8,

    /* U+005C "\\" */
    0xe1, 0x87, 0xc, 0x38, 0x61, 0xc3,

    /* U+005D "]" */
    0xff, 0xc6, 0x31, 0x8c, 0x7f, 0xf8,

    /* U+005E "^" */
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0xfe, 0xf, 0x7,
    0x83,

    /* U+005F "_" */
    0xff, 0xfc,

    /* U+0060 "`" */
    0x6c,

    /* U+0061 "a" */
    0xe, 0x7, 0x81, 0xf0, 0xec, 0x33, 0x9f, 0xe7,
    0xff, 0x83,

    /* U+0062 "b" */
    0xff, 0x7f, 0x80, 0xdf, 0xff, 0xfe, 0xf, 0xff,
    0xff,

    /* U+0063 "c" */
    0xff, 0xbf, 0xfc, 0x1f, 0x0, 0xc0, 0x30, 0x7f,
    0xff, 0xfe,

    /* U+0064 "d" */
    0xff, 0xff, 0xc0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0065 "e" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfe, 0x3, 0xff,
    0xff,

    /* U+0066 "f" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfe, 0x3, 0x1,
    0x80,

    /* U+0067 "g" */
    0xff, 0xff, 0xfc, 0x3, 0x7e, 0xdf, 0xf0, 0x7f,
    0xff, 0xfe,

    /* U+0068 "h" */
    0xc1, 0xe0, 0xf0, 0x7f, 0xff, 0xfe, 0xf, 0x7,
    0x83,

    /* U+0069 "i" */
    0xff, 0xff,

    /* U+006A "j" */
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x1e, 0xf, 0xff,
    0xff,

    /* U+006B "k" */
    0xc3, 0xe3, 0xb3, 0x9f, 0x8f, 0xc6, 0x73, 0x1d,
    0x87,

    /* U+006C "l" */
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x6, 0x3, 0xff,
    0xff,

    /* U+006D "m" */
    0xe0, 0xfc, 0x1f, 0xc7, 0xfd, 0xfd, 0xb7, 0xbe,
    0xf3, 0x9e, 0x73,

    /* U+006E "n" */
    0xc1, 0xf8, 0xfe, 0x7f, 0xbd, 0xfe, 0x7f, 0x1f,
    0x83,

    /* U+006F "o" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0070 "p" */
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0xfe, 0x3, 0x1,
    0x80,

    /* U+0071 "q" */
    0xff, 0xff, 0xf0, 0x78, 0x3d, 0xde, 0x6f, 0xff,
    0xff, 0x3, 0x0,

    /* U+0072 "r" */
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0xfe, 0x33, 0x1d,
    0x87,

    /* U+0073 "s" */
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+0074 "t" */
    0xff, 0xff, 0xc6, 0x3, 0x1, 0x80, 0xc0, 0x60,
    0x30,

    /* U+0075 "u" */
    0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0xff,
    0xff,

    /* U+0076 "v" */
    0xe1, 0xd8, 0x67, 0x18, 0xce, 0x3b, 0x7, 0xc1,
    0xe0, 0x38,

    /* U+0077 "w" */
    0xe3, 0x19, 0x8c, 0x66, 0x7b, 0x9d, 0xec, 0x3f,
    0xb0, 0xf7, 0xc3, 0xcf, 0x6, 0x38,

    /* U+0078 "x" */
    0x63, 0xbb, 0x8f, 0x83, 0xc1, 0xe1, 0xf1, 0xcc,
    0xc7,

    /* U+0079 "y" */
    0xc1, 0xe0, 0xf0, 0x7f, 0xff, 0xf8, 0xf, 0xff,
    0xff,

    /* U+007A "z" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,
    0xff,

    /* U+007B "{" */
    0xf0, 0x7c, 0x36, 0x19, 0x8c, 0xe7, 0xfb, 0xfd,
    0x87,

    /* U+007C "|" */
    0xff, 0xff, 0xff,

    /* U+007D "}" */
    0xc3, 0xe1, 0xb1, 0xd9, 0xcc, 0xc6, 0xc3, 0xe1,
    0xe0,

    /* U+007E "~" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 75, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 50, .box_w = 2, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 3, .adv_w = 93, .box_w = 5, .box_h = 3, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 5, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 14, .adv_w = 162, .box_w = 10, .box_h = 11, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 28, .adv_w = 208, .box_w = 12, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 40, .adv_w = 174, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 50, .adv_w = 50, .box_w = 2, .box_h = 3, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 51, .adv_w = 92, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 57, .adv_w = 92, .box_w = 5, .box_h = 9, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 63, .adv_w = 57, .box_w = 3, .box_h = 3, .ofs_x = 0, .ofs_y = 6},
    {.bitmap_index = 65, .adv_w = 100, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 69, .adv_w = 50, .box_w = 2, .box_h = 3, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 70, .adv_w = 92, .box_w = 5, .box_h = 2, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 72, .adv_w = 50, .box_w = 2, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 73, .adv_w = 103, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 79, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 88, .adv_w = 75, .box_w = 4, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 92, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 101, .adv_w = 161, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 110, .adv_w = 161, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 119, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 128, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 137, .adv_w = 149, .box_w = 8, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 145, .adv_w = 164, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 155, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 164, .adv_w = 50, .box_w = 2, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 166, .adv_w = 50, .box_w = 2, .box_h = 7, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 168, .adv_w = 104, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 174, .adv_w = 100, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 178, .adv_w = 104, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 184, .adv_w = 161, .box_w = 10, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 194, .adv_w = 186, .box_w = 11, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 207, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 216, .adv_w = 163, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 225, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 234, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 243, .adv_w = 163, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 253, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 262, .adv_w = 165, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 272, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 281, .adv_w = 50, .box_w = 2, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 283, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 292, .adv_w = 161, .box_w = 12, .box_h = 11, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 309, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 318, .adv_w = 195, .box_w = 11, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 329, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 338, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 347, .adv_w = 156, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 356, .adv_w = 165, .box_w = 10, .box_h = 11, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 370, .adv_w = 155, .box_w = 11, .box_h = 11, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 386, .adv_w = 176, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 396, .adv_w = 160, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 405, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 414, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 423, .adv_w = 220, .box_w = 14, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 437, .adv_w = 154, .box_w = 12, .box_h = 11, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 454, .adv_w = 161, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 463, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 472, .adv_w = 90, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 478, .adv_w = 103, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 484, .adv_w = 90, .box_w = 5, .box_h = 9, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 490, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 499, .adv_w = 133, .box_w = 7, .box_h = 2, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 501, .adv_w = 47, .box_w = 3, .box_h = 2, .ofs_x = -1, .ofs_y = 9},
    {.bitmap_index = 502, .adv_w = 168, .box_w = 10, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 512, .adv_w = 163, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 521, .adv_w = 164, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 531, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 540, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 549, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 558, .adv_w = 164, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 568, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 577, .adv_w = 50, .box_w = 2, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 579, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 588, .adv_w = 161, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 597, .adv_w = 150, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 606, .adv_w = 195, .box_w = 11, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 617, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 626, .adv_w = 165, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 635, .adv_w = 156, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 644, .adv_w = 165, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 655, .adv_w = 155, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 664, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 673, .adv_w = 159, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 682, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 691, .adv_w = 165, .box_w = 10, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 701, .adv_w = 220, .box_w = 14, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 715, .adv_w = 154, .box_w = 9, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 724, .adv_w = 161, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 733, .adv_w = 151, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 742, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 751, .adv_w = 50, .box_w = 2, .box_h = 12, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 754, .adv_w = 162, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 763, .adv_w = 164, .box_w = 9, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 772, .adv_w = 47, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
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
    0, 0, 0, 0, -10, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -11, 0,
    0, 0, 0, 0, -36, 0, -23, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -29, -23, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -11, 0, 0, 0,
    0, 0, 0, 0, -11, -4, -8, 0,
    0, 0, 0, -6, 0, -6, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -22, -2, 0, -13, 0, 0,
    0, 0, 0, -7, 0, 0, 0, 0,
    0, 0, -42, 0, -36, -16, -2, 0,
    0, 0, 0, 0, 0, -24, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    -13, 0, -2, 0, 0, -2, 0, 0,
    0, 0, -17, -2, -22, 0, -2, 0,
    0, 0, -2, 0, -22, -10, -17, 0,
    -13, -10, 0, -14, 0, -14, 0, 0,
    0, 0, 0, 0, -10, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -11, 0,
    0, 0, 0, 0, -36, 0, -23, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -29, -23, 0, 0, 0,
    0, 0, 0, 0, -17, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -17, 0,
    0, 0, 0, 0, -36, 0, -24, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -30, -24, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -2, 0, 0, -2, 0, 0,
    0, 0, 0, 0, -23, 0, -2, 0,
    0, -6, -2, 0, -23, -10, -17, 0,
    0, 0, 0, -14, 0, -14, 0, 0,
    -24, 0, 0, 0, 0, -22, 0, 0,
    0, 0, -23, -17, 0, -13, 0, 0,
    0, 0, 0, -7, 0, 0, 0, 0,
    0, 0, -29, 0, -38, -13, 0, 0,
    0, 0, 0, 0, 0, -24, 0, 0,
    0, 0, 0, 0, -16, 0, 0, 0,
    0, -2, 0, 0, 0, 0, -17, 0,
    0, 0, 0, 0, -43, -2, -34, 0,
    -4, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -30, -29, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -6, 0, 0, 0,
    0, 0, 0, 0, -6, -2, -4, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    -5, 0, 0, -5, 0, -17, 0, 0,
    -2, 0, -16, -2, 0, -17, 0, 0,
    0, 0, 0, -6, -6, 0, 0, 0,
    0, 0, -17, 0, -19, -13, -4, 0,
    -16, 0, -17, -2, 0, -16, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, -17, 0, -6, -36,
    -36, 0, 0, 0, 0, 0, -17, 0,
    0, 0, 0, 0, -50, 0, -37, 0,
    0, -17, 0, 0, 0, 0, 0, 0,
    0, -37, 0, -40, -30, 0, 0, 0,
    -5, 0, 0, -5, 0, -17, 0, 0,
    -4, 0, -16, 0, 0, -17, 0, 0,
    0, 0, -4, -6, -6, 0, 0, 0,
    0, 0, -17, 0, -19, -13, -4, 0,
    -17, 0, -17, -2, 0, -16, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -6, 0, 0, 0,
    0, 0, 0, 0, -6, 0, -4, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -11, -4, 0, 0,
    0, 0, 0, 0, 0, -6, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, -4, 0, 0, -4, 0, -4, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -8, 0, 0, 0, 0, 0,
    0, 0, -20, -5, -26, 0, -8, 0,
    0, -11, -8, 0, -26, -11, -20, -8,
    0, 0, 0, -14, 0, -14, 0, 0,
    0, 0, 0, 0, -1, 0, 0, -16,
    -22, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -36, 0, -16, 0,
    0, -16, 0, 0, -5, -2, -4, 0,
    -2, -17, 0, -24, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -5, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    -5, 0, 0, -5, 0, -17, 0, 0,
    -4, 0, -16, -2, 0, -17, 0, 0,
    0, 0, -5, -6, -5, 0, 0, 0,
    0, 0, -17, 0, -19, -13, -4, 0,
    -17, 0, -17, -2, 0, -16, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    -12, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -22, 0, -2, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, -18, 0, -12, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, -8, 0, -2, -4, 0, -17, 0,
    0, -4, 0, 0, -4, 0, -4, 0,
    0, -20, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    -24, 0, 0, -42, 0, -24, -2, 0,
    0, 0, -22, -17, 0, -13, 0, 0,
    0, -8, 0, -7, 0, 0, 0, 0,
    0, 0, -48, 0, -48, -22, 0, 0,
    -32, 0, -24, 0, 0, -34, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -17,
    -22, 0, 0, 0, 0, 0, -1, 0,
    0, 0, -4, 0, -31, 0, -17, 0,
    0, -16, 0, 0, -6, -2, -4, 0,
    0, -20, 0, -22, -1, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    0, 0, 0, 0, -16, 0, -2, -29,
    -31, 0, 0, 0, 0, 0, -17, 0,
    0, -8, 0, 0, -48, 0, -48, 0,
    0, -17, 0, 0, 0, 0, 0, 0,
    0, -24, 0, -34, -48, 0, 0, 0,
    0, -4, -2, 0, -13, 0, -10, -13,
    -23, -2, 0, 0, -2, 0, -13, 0,
    -4, -11, 0, 0, -30, -2, -22, 0,
    -2, -16, 0, 0, 0, 0, 0, 0,
    0, -17, 0, -23, -17, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -8, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -2, 0, 0, 0,
    0, -2, -19, -22, -2, -13, -2, 0,
    0, 0, 0, -6, 0, 0, 0, -2,
    0, 0, -31, -2, -43, -23, 0, 0,
    0, 0, 0, 0, -2, -30, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -23, -20, 0, -13, 0, 0,
    0, 0, 0, -7, 0, 0, 0, 0,
    0, 0, -24, 0, -48, -17, 0, 0,
    0, 0, 0, 0, 0, -24, 0, 0,
    0, 0, 0, 0, -17, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -17, 0,
    0, 0, 0, 0, -24, 0, -22, 0,
    0, -17, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -24, -22, 0, 0, 0,
    -23, -6, 0, -24, -2, -30, -14, 0,
    0, -2, -22, -24, -2, -13, -2, 0,
    0, -14, 0, -12, 0, 0, 0, -2,
    0, 0, -34, -2, -43, -23, 0, 0,
    -30, 0, -24, 0, -4, -30, 0, 0,
    -5, 0, 0, -5, 0, -17, 0, 0,
    -4, 0, -16, -2, 0, -17, 0, 0,
    0, 0, -5, -6, -5, 0, 0, 0,
    0, 0, -17, 0, -19, -13, -4, 0,
    -16, 0, -17, -2, 0, -16, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -4, 0, -6, 0, 0, 0,
    0, 0, 0, 0, 0, -2, -4, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    -24, 0, 0, -42, 0, -24, -2, 0,
    0, 0, -22, -17, 0, -13, 0, 0,
    0, -8, 0, -7, 0, 0, 0, 0,
    0, 0, -48, 0, -48, -22, 0, 0,
    -32, 0, -24, 0, 0, -35, 0, 0,
    0, 0, 0, 0, -16, 0, -2, -29,
    -34, 0, 0, 0, 0, 0, -17, 0,
    0, -8, 0, 0, -48, 0, -48, 0,
    0, -17, 0, 0, 0, 0, 0, 0,
    0, -24, 0, -34, -48, 0, 0, 0,
    0, -6, -2, 0, -16, 0, -14, -24,
    -30, -2, 0, 0, -2, 0, -16, -2,
    -6, -14, 0, 0, -43, -2, -34, 0,
    -2, -16, 0, 0, 0, 0, 0, 0,
    0, -24, 0, -30, -25, 0, -2, 0,
    -30, -11, 0, -36, -5, -36, -23, 0,
    0, -6, -25, -36, -6, -13, -6, 0,
    0, -26, 0, -12, 0, 0, 0, -6,
    0, 0, -48, -4, -62, -30, 0, 0,
    -43, 0, -24, 0, -6, -43, 0, -6,
    0, -11, -6, 0, -19, 0, -23, -38,
    -43, -6, 0, 0, -6, 0, -19, -6,
    -11, -26, 0, 0, -62, -6, -48, 0,
    -6, -19, 0, 0, 0, 0, 0, 0,
    0, -48, 0, -43, -47, 0, 0, 0
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

extern const lv_font_t lv_font_montserrat_14;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t space_ranger_15 = {
#else
lv_font_t space_ranger_15 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 14,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .static_bitmap = 0,
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = &lv_font_montserrat_14,
#endif
    .user_data = NULL,
};



#endif /*#if SPACE_RANGER_15*/
