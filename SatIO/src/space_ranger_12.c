/*******************************************************************************
 * Size: 12 px
 * Bpp: 1
 * Opts: --bpp 1 --size 12 --no-compress --stride 1 --align 1 --font spaceranger.ttf --range 32-127 --format lvgl -o space_ranger_12.c
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



#ifndef SPACE_RANGER_12
#define SPACE_RANGER_12 1
#endif

#if SPACE_RANGER_12

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xfc, 0x30,

    /* U+0022 "\"" */
    0xf0,

    /* U+0023 "#" */
    0x6d, 0xfd, 0xb3, 0x6f, 0xed, 0x80,

    /* U+0024 "$" */
    0x18, 0xff, 0xd8, 0xfe, 0x1b, 0x1b, 0xfe, 0x18,

    /* U+0025 "%" */
    0xf6, 0x25, 0x8f, 0xc0, 0x3f, 0x1a, 0xc6, 0xf0,

    /* U+0026 "&" */
    0x7c, 0x60, 0xff, 0xc2, 0xc2, 0xfe,

    /* U+0027 "'" */
    0xc0,

    /* U+0028 "(" */
    0xfc, 0xcc, 0xcc, 0xf0,

    /* U+0029 ")" */
    0xf3, 0x33, 0x33, 0xf0,

    /* U+002A "*" */
    0xd8,

    /* U+002B "+" */
    0x6f, 0x66,

    /* U+002C "," */
    0xc0,

    /* U+002D "-" */
    0xf0,

    /* U+002E "." */
    0xc0,

    /* U+002F "/" */
    0x19, 0x8c, 0xc6, 0x60,

    /* U+0030 "0" */
    0xff, 0xcf, 0xdb, 0xdb, 0xf3, 0xff,

    /* U+0031 "1" */
    0xed, 0xb6, 0xc0,

    /* U+0032 "2" */
    0xfe, 0x3, 0xfe, 0xc0, 0xc0, 0xff,

    /* U+0033 "3" */
    0xfc, 0x6, 0xfe, 0x3, 0x3, 0xfe,

    /* U+0034 "4" */
    0xc7, 0x8f, 0xf8, 0x30, 0x60, 0xc0,

    /* U+0035 "5" */
    0xff, 0xc0, 0xfe, 0x3, 0x3, 0xfe,

    /* U+0036 "6" */
    0xff, 0xc0, 0xfe, 0xc3, 0xc3, 0xfe,

    /* U+0037 "7" */
    0xfe, 0x18, 0x71, 0xc3, 0xc, 0x0,

    /* U+0038 "8" */
    0x7c, 0x6, 0xfe, 0xc3, 0xc3, 0xfe,

    /* U+0039 "9" */
    0xfe, 0xc3, 0xff, 0x3, 0x3, 0xfe,

    /* U+003A ":" */
    0xc3,

    /* U+003B ";" */
    0xc0, 0xc0,

    /* U+003C "<" */
    0x39, 0x98, 0xc3, 0x1c,

    /* U+003D "=" */
    0xf0, 0xf0,

    /* U+003E ">" */
    0x63, 0x8e, 0x77, 0x30,

    /* U+003F "?" */
    0xff, 0x3, 0x7f, 0x0, 0x0, 0x60,

    /* U+0040 "@" */
    0xff, 0xe0, 0xf7, 0x7a, 0xbd, 0xfe, 0x3, 0xfe,

    /* U+0041 "A" */
    0xf, 0xb, 0x1b, 0x33, 0x7f, 0x63,

    /* U+0042 "B" */
    0xfc, 0x6, 0xfe, 0xc3, 0xc3, 0xfe,

    /* U+0043 "C" */
    0xff, 0x83, 0x6, 0xc, 0x1f, 0xc0,

    /* U+0044 "D" */
    0xff, 0x3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0045 "E" */
    0xfe, 0xc3, 0xde, 0xc0, 0xc0, 0xff,

    /* U+0046 "F" */
    0xff, 0x83, 0xfe, 0xc, 0x18, 0x0,

    /* U+0047 "G" */
    0xff, 0xc0, 0xde, 0xc3, 0xc3, 0xfe,

    /* U+0048 "H" */
    0xc7, 0x8f, 0xfe, 0x3c, 0x78, 0xc0,

    /* U+0049 "I" */
    0xc3, 0xf0,

    /* U+004A "J" */
    0x6, 0xc, 0x18, 0x30, 0x7f, 0xc0,

    /* U+004B "K" */
    0xc6, 0x33, 0xf, 0x83, 0x30, 0xc6, 0x30, 0xc0,
    0x38, 0x6,

    /* U+004C "L" */
    0xc1, 0x83, 0x6, 0xc, 0x1f, 0xc0,

    /* U+004D "M" */
    0xe3, 0xf1, 0xfd, 0xfe, 0xfd, 0xde, 0x4c,

    /* U+004E "N" */
    0xc3, 0xf3, 0xfb, 0xdf, 0xcf, 0xc3,

    /* U+004F "O" */
    0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0050 "P" */
    0xfe, 0xc, 0x1f, 0xfc, 0x18, 0x0,

    /* U+0051 "Q" */
    0xff, 0x61, 0xb0, 0xdb, 0x6c, 0xf7, 0xf8, 0x18,
    0x6, 0x1, 0x80,

    /* U+0052 "R" */
    0xfe, 0x3, 0x3f, 0x99, 0x8c, 0x66, 0x30, 0xc,
    0x6, 0x1, 0x80,

    /* U+0053 "S" */
    0x7f, 0xc0, 0x7f, 0x3, 0x3, 0xff,

    /* U+0054 "T" */
    0xfe, 0x30, 0x60, 0xc1, 0x83, 0x0,

    /* U+0055 "U" */
    0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0056 "V" */
    0x63, 0x63, 0x33, 0x13, 0xb, 0xf,

    /* U+0057 "W" */
    0xc4, 0x6d, 0xd9, 0xbb, 0x35, 0x63, 0xb8, 0x77,
    0x0,

    /* U+0058 "X" */
    0x66, 0x1f, 0x3, 0x80, 0xe0, 0x7c, 0x19, 0x80,
    0x30, 0x6,

    /* U+0059 "Y" */
    0xc7, 0x8f, 0xf8, 0x30, 0x7f, 0xc0,

    /* U+005A "Z" */
    0xfe, 0x3c, 0xf3, 0xcf, 0x1f, 0xc0,

    /* U+005B "[" */
    0xfc, 0xcc, 0xcc, 0xf0,

    /* U+005C "\\" */
    0xc3, 0x18, 0x63, 0xc,

    /* U+005D "]" */
    0xf3, 0x33, 0x33, 0xf0,

    /* U+005E "^" */
    0xff, 0x3, 0xff, 0xc3, 0xc3, 0xc3,

    /* U+005F "_" */
    0xfc,

    /* U+0060 "`" */
    0xc0,

    /* U+0061 "a" */
    0x1c, 0x3c, 0x3e, 0x76, 0x7f, 0xc3,

    /* U+0062 "b" */
    0xfc, 0x6, 0xfe, 0xc3, 0xc3, 0xfe,

    /* U+0063 "c" */
    0xfe, 0xc3, 0xc0, 0xc0, 0xc3, 0xfe,

    /* U+0064 "d" */
    0xff, 0x3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0065 "e" */
    0xff, 0x83, 0xfe, 0xc, 0x1f, 0xc0,

    /* U+0066 "f" */
    0xff, 0x83, 0xfe, 0xc, 0x18, 0x0,

    /* U+0067 "g" */
    0xff, 0xc0, 0xde, 0xc3, 0xc3, 0xfe,

    /* U+0068 "h" */
    0xc7, 0x8f, 0xfe, 0x3c, 0x78, 0xc0,

    /* U+0069 "i" */
    0xff, 0xf0,

    /* U+006A "j" */
    0x3, 0x3, 0x3, 0x3, 0xc3, 0x7f,

    /* U+006B "k" */
    0xc6, 0xcc, 0xf8, 0xcc, 0xc6, 0xc6,

    /* U+006C "l" */
    0xc1, 0x83, 0x6, 0xc, 0x1f, 0xc0,

    /* U+006D "m" */
    0xe3, 0xf1, 0xfd, 0xfe, 0xfd, 0xde, 0x4c,

    /* U+006E "n" */
    0xc3, 0xf3, 0xfb, 0xdf, 0xcf, 0xc3,

    /* U+006F "o" */
    0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0070 "p" */
    0xfe, 0xf, 0xfe, 0xc, 0x18, 0x0,

    /* U+0071 "q" */
    0xff, 0xc3, 0xc3, 0xdb, 0xcf, 0xff, 0x6, 0x7,

    /* U+0072 "r" */
    0xfe, 0xf, 0xfe, 0x6c, 0xd8, 0xc0,

    /* U+0073 "s" */
    0xff, 0xc0, 0xfe, 0x3, 0x3, 0xfe,

    /* U+0074 "t" */
    0xfe, 0x30, 0x60, 0xc1, 0x83, 0x0,

    /* U+0075 "u" */
    0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xff,

    /* U+0076 "v" */
    0xc3, 0x66, 0x66, 0x3c, 0x3c, 0x18,

    /* U+0077 "w" */
    0xc4, 0x6d, 0xd9, 0xbb, 0x35, 0x63, 0xb8, 0x77,
    0x0,

    /* U+0078 "x" */
    0x66, 0x7c, 0x38, 0x38, 0x7c, 0x66,

    /* U+0079 "y" */
    0xc7, 0x8f, 0xf8, 0x30, 0x7f, 0xc0,

    /* U+007A "z" */
    0xfe, 0x3c, 0xf3, 0xcf, 0x1f, 0xc0,

    /* U+007B "{" */
    0xf0, 0xd0, 0xd8, 0xcc, 0xfe, 0xc6,

    /* U+007C "|" */
    0xff, 0xff, 0xc0,

    /* U+007D "}" */
    0xc6, 0xc6, 0xcc, 0xc8, 0xd0, 0xf0,

    /* U+007E "~" */
    0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,

    /* U+007F "" */
    0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 60, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 40, .box_w = 2, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 3, .adv_w = 74, .box_w = 4, .box_h = 1, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 4, .adv_w = 132, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 10, .adv_w = 130, .box_w = 8, .box_h = 8, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 18, .adv_w = 166, .box_w = 10, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 26, .adv_w = 139, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 32, .adv_w = 40, .box_w = 2, .box_h = 1, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 33, .adv_w = 74, .box_w = 4, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 37, .adv_w = 74, .box_w = 4, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 41, .adv_w = 46, .box_w = 3, .box_h = 2, .ofs_x = 0, .ofs_y = 5},
    {.bitmap_index = 42, .adv_w = 80, .box_w = 4, .box_h = 4, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 44, .adv_w = 40, .box_w = 2, .box_h = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 45, .adv_w = 74, .box_w = 4, .box_h = 1, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 46, .adv_w = 40, .box_w = 2, .box_h = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 47, .adv_w = 82, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 51, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 57, .adv_w = 60, .box_w = 3, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 60, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 66, .adv_w = 128, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 72, .adv_w = 129, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 78, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 84, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 90, .adv_w = 119, .box_w = 7, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 96, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 102, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 108, .adv_w = 40, .box_w = 2, .box_h = 4, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 109, .adv_w = 40, .box_w = 2, .box_h = 5, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 111, .adv_w = 83, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 115, .adv_w = 80, .box_w = 4, .box_h = 3, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 117, .adv_w = 83, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 121, .adv_w = 129, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 127, .adv_w = 149, .box_w = 9, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 135, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 141, .adv_w = 131, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 147, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 153, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 159, .adv_w = 131, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 165, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 171, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 177, .adv_w = 130, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 183, .adv_w = 40, .box_w = 2, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 185, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 191, .adv_w = 129, .box_w = 10, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 201, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 207, .adv_w = 156, .box_w = 9, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 214, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 220, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 226, .adv_w = 124, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 232, .adv_w = 132, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 243, .adv_w = 124, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 254, .adv_w = 141, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 260, .adv_w = 128, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 266, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 272, .adv_w = 129, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 278, .adv_w = 176, .box_w = 11, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 287, .adv_w = 123, .box_w = 10, .box_h = 8, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 297, .adv_w = 129, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 303, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 309, .adv_w = 72, .box_w = 4, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 313, .adv_w = 82, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 317, .adv_w = 72, .box_w = 4, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 321, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 327, .adv_w = 107, .box_w = 6, .box_h = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 328, .adv_w = 38, .box_w = 2, .box_h = 1, .ofs_x = 0, .ofs_y = 7},
    {.bitmap_index = 329, .adv_w = 134, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 335, .adv_w = 131, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 341, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 347, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 353, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 359, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 365, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 371, .adv_w = 130, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 377, .adv_w = 40, .box_w = 2, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 379, .adv_w = 129, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 385, .adv_w = 129, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 391, .adv_w = 120, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 397, .adv_w = 156, .box_w = 9, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 404, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 410, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 416, .adv_w = 124, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 422, .adv_w = 132, .box_w = 8, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 430, .adv_w = 124, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 436, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 442, .adv_w = 127, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 448, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 454, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 460, .adv_w = 176, .box_w = 11, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 469, .adv_w = 123, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 475, .adv_w = 129, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 481, .adv_w = 121, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 487, .adv_w = 130, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 493, .adv_w = 40, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 496, .adv_w = 129, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 502, .adv_w = 132, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 508, .adv_w = 38, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0}
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
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -9, 0,
    0, 0, 0, 0, -29, 0, -18, 0,
    0, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -23, -18, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -9, 0, 0, 0,
    0, 0, 0, 0, -9, -3, -7, 0,
    0, 0, 0, -5, 0, -5, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -17, -2, 0, -11, 0, 0,
    0, 0, 0, -6, 0, 0, 0, 0,
    0, 0, -34, 0, -29, -12, -2, 0,
    0, 0, 0, 0, 0, -19, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    -11, 0, -2, 0, 0, -2, 0, 0,
    0, 0, -13, -2, -17, 0, -2, 0,
    0, 0, -2, 0, -17, -8, -13, 0,
    -11, -8, 0, -12, 0, -12, 0, 0,
    0, 0, 0, 0, -8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -9, 0,
    0, 0, 0, 0, -29, 0, -18, 0,
    0, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -23, -18, 0, 0, 0,
    0, 0, 0, 0, -13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -13, 0,
    0, 0, 0, 0, -29, 0, -19, 0,
    0, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -24, -19, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -2, 0, 0, -2, 0, 0,
    0, 0, 0, 0, -18, 0, -2, 0,
    0, -5, -2, 0, -18, -8, -13, 0,
    0, 0, 0, -12, 0, -12, 0, 0,
    -19, 0, 0, 0, 0, -17, 0, 0,
    0, 0, -18, -13, 0, -11, 0, 0,
    0, 0, 0, -6, 0, 0, 0, 0,
    0, 0, -23, 0, -31, -11, 0, 0,
    0, 0, 0, 0, 0, -19, 0, 0,
    0, 0, 0, 0, -12, 0, 0, 0,
    0, -2, 0, 0, 0, 0, -13, 0,
    0, 0, 0, 0, -35, -2, -27, 0,
    -3, -12, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -24, -23, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, -5, 0, 0, 0,
    0, 0, 0, 0, -5, -2, -3, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    -4, 0, 0, -4, 0, -13, 0, 0,
    -2, 0, -12, -2, 0, -13, 0, 0,
    0, 0, 0, -5, -5, 0, 0, 0,
    0, 0, -13, 0, -15, -11, -3, 0,
    -12, 0, -13, -2, 0, -12, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, -13, 0, -5, -29,
    -29, 0, 0, 0, 0, 0, -13, 0,
    0, 0, 0, 0, -40, 0, -30, 0,
    0, -13, 0, 0, 0, 0, 0, 0,
    0, -30, 0, -32, -24, 0, 0, 0,
    -4, 0, 0, -4, 0, -13, 0, 0,
    -3, 0, -12, 0, 0, -13, 0, 0,
    0, 0, -3, -5, -5, 0, 0, 0,
    0, 0, -13, 0, -15, -11, -3, 0,
    -13, 0, -13, -2, 0, -12, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, -5, 0, 0, 0,
    0, 0, 0, 0, -5, 0, -3, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -9, -3, 0, 0,
    0, 0, 0, 0, 0, -5, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    0, -3, 0, 0, -3, 0, -3, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -7, 0, 0, 0, 0, 0,
    0, 0, -16, -4, -21, 0, -7, 0,
    0, -9, -7, 0, -21, -9, -16, -7,
    0, 0, 0, -12, 0, -12, 0, 0,
    0, 0, 0, 0, -1, 0, 0, -12,
    -17, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -29, 0, -12, 0,
    0, -12, 0, 0, -4, -2, -3, 0,
    -2, -13, 0, -19, 0, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -4, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    -4, 0, 0, -4, 0, -13, 0, 0,
    -3, 0, -12, -2, 0, -13, 0, 0,
    0, 0, -4, -5, -4, 0, 0, 0,
    0, 0, -13, 0, -15, -11, -3, 0,
    -13, 0, -13, -2, 0, -12, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -2,
    -10, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -17, 0, -2, 0,
    0, -12, 0, 0, 0, 0, 0, 0,
    0, -14, 0, -10, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, -7, 0, -2, -3, 0, -13, 0,
    0, -3, 0, 0, -3, 0, -3, 0,
    0, -16, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -3, 0, 0, 0,
    -19, 0, 0, -34, 0, -19, -2, 0,
    0, 0, -17, -13, 0, -11, 0, 0,
    0, -7, 0, -6, 0, 0, 0, 0,
    0, 0, -38, 0, -38, -17, 0, 0,
    -26, 0, -19, 0, 0, -27, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -13,
    -17, 0, 0, 0, 0, 0, -1, 0,
    0, 0, -3, 0, -25, 0, -13, 0,
    0, -12, 0, 0, -5, -2, -3, 0,
    0, -16, 0, -17, -1, -2, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, -2, 0, 0,
    -2, 0, 0, 0, 0, -2, 0, 0,
    0, 0, 0, 0, -12, 0, -2, -23,
    -25, 0, 0, 0, 0, 0, -13, 0,
    0, -7, 0, 0, -38, 0, -38, 0,
    0, -13, 0, 0, 0, 0, 0, 0,
    0, -19, 0, -27, -38, 0, 0, 0,
    0, -3, -2, 0, -11, 0, -8, -11,
    -18, -2, 0, 0, -2, 0, -11, 0,
    -3, -9, 0, 0, -24, -2, -17, 0,
    -2, -12, 0, 0, 0, 0, 0, 0,
    0, -13, 0, -18, -13, 0, -2, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -2, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, -7, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -2, 0, 0, 0,
    0, -2, -15, -17, -2, -11, -2, 0,
    0, 0, 0, -5, 0, 0, 0, -2,
    0, 0, -25, -2, -35, -18, 0, 0,
    0, 0, 0, 0, -2, -24, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -18, -16, 0, -11, 0, 0,
    0, 0, 0, -6, 0, 0, 0, 0,
    0, 0, -19, 0, -38, -13, 0, 0,
    0, 0, 0, 0, 0, -19, 0, 0,
    0, 0, 0, 0, -13, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -13, 0,
    0, 0, 0, 0, -19, 0, -17, 0,
    0, -13, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -19, -17, 0, 0, 0,
    -18, -5, 0, -19, -2, -24, -12, 0,
    0, -2, -17, -19, -2, -11, -2, 0,
    0, -12, 0, -10, 0, 0, 0, -2,
    0, 0, -27, -2, -35, -18, 0, 0,
    -24, 0, -19, 0, -3, -24, 0, 0,
    -4, 0, 0, -4, 0, -13, 0, 0,
    -3, 0, -12, -2, 0, -13, 0, 0,
    0, 0, -4, -5, -4, 0, 0, 0,
    0, 0, -13, 0, -15, -11, -3, 0,
    -12, 0, -13, -2, 0, -12, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    -2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -3, 0, -5, 0, 0, 0,
    0, 0, 0, 0, 0, -2, -3, 0,
    -2, 0, 0, -2, 0, -2, 0, 0,
    -19, 0, 0, -34, 0, -19, -2, 0,
    0, 0, -17, -13, 0, -11, 0, 0,
    0, -7, 0, -6, 0, 0, 0, 0,
    0, 0, -38, 0, -38, -17, 0, 0,
    -26, 0, -19, 0, 0, -28, 0, 0,
    0, 0, 0, 0, -12, 0, -2, -23,
    -27, 0, 0, 0, 0, 0, -13, 0,
    0, -7, 0, 0, -38, 0, -38, 0,
    0, -13, 0, 0, 0, 0, 0, 0,
    0, -19, 0, -27, -38, 0, 0, 0,
    0, -5, -2, 0, -12, 0, -12, -19,
    -24, -2, 0, 0, -2, 0, -12, -2,
    -5, -12, 0, 0, -35, -2, -27, 0,
    -2, -12, 0, 0, 0, 0, 0, 0,
    0, -19, 0, -24, -20, 0, -2, 0,
    -24, -9, 0, -29, -4, -29, -18, 0,
    0, -5, -20, -29, -5, -11, -5, 0,
    0, -21, 0, -10, 0, 0, 0, -5,
    0, 0, -38, -3, -50, -24, 0, 0,
    -35, 0, -19, 0, -5, -35, 0, -5,
    0, -9, -5, 0, -15, 0, -18, -31,
    -35, -5, 0, 0, -5, 0, -15, -5,
    -9, -21, 0, 0, -50, -5, -38, 0,
    -5, -15, 0, 0, 0, 0, 0, 0,
    0, -38, 0, -35, -37, 0, 0, 0
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

extern const lv_font_t lv_font_montserrat_12;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t space_ranger_12 = {
#else
lv_font_t space_ranger_12 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 11,          /*The maximum line height required by the font*/
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
    .fallback = &lv_font_montserrat_12,
#endif
    .user_data = NULL,
};



#endif /*#if SPACE_RANGER_12*/
