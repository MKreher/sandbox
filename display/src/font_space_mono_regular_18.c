/*******************************************************************************
 * Size: 18 px
 * Bpp: 1
 * Opts: 
 ******************************************************************************/

#include <lvgl.h>

#ifndef FONT_SPACE_MONO_REGULAR_18
#define FONT_SPACE_MONO_REGULAR_18 1
#endif

#if FONT_SPACE_MONO_REGULAR_18

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0x49, 0x24, 0x92, 0x43, 0xf0,

    /* U+0022 "\"" */
    0xff, 0xf4, 0x92,

    /* U+0023 "#" */
    0x24, 0x24, 0x24, 0xff, 0x24, 0x24, 0xff, 0x24,
    0x24,

    /* U+0024 "$" */
    0x10, 0xf3, 0x7c, 0x9d, 0xf, 0xa4, 0xc9, 0xd6,
    0x78, 0x40,

    /* U+0025 "%" */
    0x70, 0x44, 0x22, 0x11, 0x17, 0x30, 0x60, 0x40,
    0xce, 0x88, 0x84, 0x42, 0x20, 0xe0,

    /* U+0026 "&" */
    0x3e, 0x30, 0x8, 0x2, 0x4, 0xc1, 0xf, 0xf4,
    0x12, 0x4, 0x81, 0x20, 0x44, 0x10, 0xfc,

    /* U+0027 "'" */
    0xfd, 0x20,

    /* U+0028 "(" */
    0xe, 0x61, 0x82, 0x8, 0x10, 0x20, 0x40, 0x81,
    0x2, 0x4, 0x8, 0x10, 0x10, 0x30, 0x30, 0x1c,

    /* U+0029 ")" */
    0xe0, 0x30, 0x30, 0x20, 0x20, 0x40, 0x81, 0x2,
    0x4, 0x8, 0x10, 0x20, 0x41, 0x6, 0x19, 0xc0,

    /* U+002A "*" */
    0x8, 0x24, 0x8a, 0x83, 0x8f, 0xf8, 0xe0, 0xa8,
    0x92, 0x8, 0x0,

    /* U+002B "+" */
    0x10, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10,

    /* U+002C "," */
    0xdd, 0xe0,

    /* U+002D "-" */
    0xf8,

    /* U+002E "." */
    0xfc,

    /* U+002F "/" */
    0x6, 0x8, 0x10, 0x60, 0x81, 0x6, 0x8, 0x10,
    0x60, 0x81, 0x6, 0x8, 0x10, 0x60,

    /* U+0030 "0" */
    0x3e, 0x31, 0xb0, 0x70, 0x18, 0xc, 0xc6, 0x63,
    0x1, 0x80, 0xc0, 0xd0, 0x47, 0xc0,

    /* U+0031 "1" */
    0x38, 0x1c, 0x1a, 0x9, 0xc, 0x84, 0x40, 0x20,
    0x10, 0x8, 0x4, 0x2, 0x1f, 0xf0,

    /* U+0032 "2" */
    0x3c, 0x42, 0x81, 0x81, 0x81, 0x3, 0x1e, 0x78,
    0xc0, 0x80, 0x80, 0xff,

    /* U+0033 "3" */
    0xff, 0x1, 0x6, 0xc, 0x10, 0x1e, 0x3, 0x81,
    0x81, 0x81, 0x42, 0x3c,

    /* U+0034 "4" */
    0xe, 0x5, 0x6, 0x86, 0x42, 0x23, 0x11, 0x9,
    0x4, 0xff, 0x81, 0x0, 0x80, 0x40,

    /* U+0035 "5" */
    0xfe, 0x80, 0x80, 0xbc, 0xa2, 0xe1, 0x1, 0x81,
    0x81, 0x83, 0x42, 0x3c,

    /* U+0036 "6" */
    0x3c, 0x42, 0x81, 0x80, 0xbc, 0xc2, 0x81, 0x81,
    0x81, 0x81, 0x42, 0x3c,

    /* U+0037 "7" */
    0xff, 0x1, 0x3, 0x2, 0x4, 0xc, 0x8, 0x10,
    0x30, 0x20, 0x40, 0x40,

    /* U+0038 "8" */
    0x3e, 0x63, 0x41, 0x41, 0x63, 0x3e, 0x43, 0x81,
    0x81, 0x81, 0x42, 0x3c,

    /* U+0039 "9" */
    0x3c, 0x42, 0x81, 0x81, 0x81, 0x43, 0x3d, 0x1,
    0x81, 0x81, 0x42, 0x3c,

    /* U+003A ":" */
    0xfc, 0x0, 0x7, 0xe0,

    /* U+003B ";" */
    0xfc, 0x0, 0x7, 0xe7, 0x80,

    /* U+003C "<" */
    0x3, 0x1e, 0xf0, 0xc0, 0xe0, 0x78, 0xf, 0x1,

    /* U+003D "=" */
    0xff, 0x0, 0x0, 0xff,

    /* U+003E ">" */
    0xc0, 0xf8, 0x1f, 0x3, 0x7, 0x3e, 0xf0, 0x80,

    /* U+003F "?" */
    0x3c, 0x42, 0x81, 0x81, 0x3, 0x1e, 0x10, 0x10,
    0x10, 0x0, 0x38, 0x38,

    /* U+0040 "@" */
    0x7f, 0x0, 0xdd, 0x29, 0x98, 0x4c, 0x26, 0x13,
    0x9, 0x4c, 0xbb, 0xc0,

    /* U+0041 "A" */
    0xe, 0x1, 0x40, 0x6c, 0xd, 0x81, 0x10, 0x63,
    0xc, 0x61, 0x4, 0x7f, 0xcc, 0x19, 0x1, 0x60,
    0x30,

    /* U+0042 "B" */
    0xfe, 0x20, 0xd0, 0x28, 0x14, 0x1b, 0xf1, 0x4,
    0x81, 0x40, 0xa0, 0x50, 0x5f, 0xc0,

    /* U+0043 "C" */
    0x3e, 0x30, 0xb0, 0x30, 0x18, 0x4, 0x2, 0x1,
    0x0, 0x80, 0xc0, 0x50, 0x47, 0xc0,

    /* U+0044 "D" */
    0xfc, 0x42, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,
    0x41, 0x41, 0x42, 0xfc,

    /* U+0045 "E" */
    0xff, 0x2, 0x4, 0x8, 0x1f, 0xe0, 0x40, 0x81,
    0x2, 0x7, 0xf0,

    /* U+0046 "F" */
    0xff, 0x2, 0x4, 0x8, 0x1f, 0xe0, 0x40, 0x81,
    0x2, 0x4, 0x0,

    /* U+0047 "G" */
    0x3e, 0x30, 0xb0, 0x30, 0x18, 0x4, 0x2, 0x7f,
    0x1, 0x80, 0xc0, 0xd0, 0xe7, 0xd0,

    /* U+0048 "H" */
    0x81, 0x81, 0x81, 0x81, 0x81, 0xff, 0x81, 0x81,
    0x81, 0x81, 0x81, 0x81,

    /* U+0049 "I" */
    0xff, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0xff,

    /* U+004A "J" */
    0x7, 0x80, 0x80, 0x40, 0x20, 0x10, 0x8, 0x5,
    0x2, 0x81, 0x40, 0x90, 0x87, 0x80,

    /* U+004B "K" */
    0x83, 0x43, 0x23, 0x13, 0xb, 0x7, 0x82, 0xc1,
    0x30, 0x8c, 0x43, 0x20, 0xd0, 0x30,

    /* U+004C "L" */
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x80, 0xff,

    /* U+004D "M" */
    0xe3, 0xf1, 0xf8, 0xf4, 0x5a, 0x2d, 0x36, 0xdb,
    0x6d, 0xb6, 0xca, 0x65, 0x33, 0x90,

    /* U+004E "N" */
    0xe1, 0xe1, 0xb1, 0xb1, 0x91, 0x99, 0x99, 0x89,
    0x8d, 0x8d, 0x87, 0x87,

    /* U+004F "O" */
    0x3e, 0x31, 0xb0, 0x70, 0x18, 0xc, 0x6, 0x3,
    0x1, 0x80, 0xc0, 0xd0, 0x47, 0xc0,

    /* U+0050 "P" */
    0xfc, 0x82, 0x81, 0x81, 0x81, 0x82, 0xfc, 0x80,
    0x80, 0x80, 0x80, 0x80,

    /* U+0051 "Q" */
    0x3e, 0x31, 0xb0, 0x70, 0x18, 0xc, 0x6, 0x3,
    0x1, 0x80, 0xc0, 0x50, 0x4f, 0xe1, 0xc0, 0x40,
    0x3c,

    /* U+0052 "R" */
    0xfc, 0x82, 0x81, 0x81, 0x81, 0x83, 0xfe, 0x81,
    0x81, 0x81, 0x81, 0x81,

    /* U+0053 "S" */
    0x1e, 0x30, 0x90, 0x28, 0x16, 0x1, 0xf0, 0x3d,
    0x3, 0x80, 0xc0, 0x50, 0x47, 0xc0,

    /* U+0054 "T" */
    0xff, 0x84, 0x2, 0x1, 0x0, 0x80, 0x40, 0x20,
    0x10, 0x8, 0x4, 0x2, 0x1, 0x0,

    /* U+0055 "U" */
    0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
    0x81, 0x81, 0x42, 0x3c,

    /* U+0056 "V" */
    0x40, 0x4c, 0x19, 0x83, 0x10, 0x42, 0x8, 0x63,
    0xc, 0x60, 0x88, 0x1b, 0x3, 0x60, 0x28, 0x7,
    0x0,

    /* U+0057 "W" */
    0xce, 0x79, 0x4f, 0x29, 0xa5, 0x24, 0xa4, 0x94,
    0x96, 0xd2, 0xda, 0x7b, 0x4f, 0x39, 0xc7, 0x38,
    0xe0,

    /* U+0058 "X" */
    0xc1, 0xa0, 0x98, 0xc4, 0xc3, 0x60, 0xe0, 0x50,
    0x6c, 0x22, 0x31, 0x90, 0x58, 0x30,

    /* U+0059 "Y" */
    0xc1, 0xe0, 0xd0, 0xcc, 0x62, 0x21, 0xb0, 0x50,
    0x38, 0x8, 0x4, 0x2, 0x1, 0x0,

    /* U+005A "Z" */
    0xff, 0x80, 0x40, 0x40, 0x60, 0x60, 0x60, 0x60,
    0x60, 0x60, 0x20, 0x20, 0x1f, 0xf0,

    /* U+005B "[" */
    0xf2, 0x49, 0x24, 0x92, 0x49, 0x24, 0x9c,

    /* U+005C "\\" */
    0xc0, 0x81, 0x3, 0x2, 0x4, 0xc, 0x8, 0x10,
    0x30, 0x20, 0x40, 0xc0, 0x81, 0x3,

    /* U+005D "]" */
    0xe4, 0x92, 0x49, 0x24, 0x92, 0x49, 0x3c,

    /* U+005E "^" */
    0x1c, 0x1b, 0xd, 0x8c, 0x66, 0x16, 0xc,

    /* U+005F "_" */
    0xff,

    /* U+0060 "`" */
    0xcc,

    /* U+0061 "a" */
    0x3d, 0x21, 0xa0, 0x50, 0x28, 0x14, 0xa, 0x4,
    0x86, 0x3d, 0x80,

    /* U+0062 "b" */
    0x80, 0x40, 0x20, 0x17, 0xce, 0x36, 0xe, 0x3,
    0x1, 0x80, 0xe0, 0xe8, 0xd3, 0xc0,

    /* U+0063 "c" */
    0x3c, 0x43, 0xc1, 0x80, 0x80, 0x80, 0x81, 0x43,
    0x3c,

    /* U+0064 "d" */
    0x0, 0x80, 0x40, 0x27, 0xd6, 0x3e, 0xe, 0x3,
    0x1, 0x80, 0xe0, 0xd8, 0xa7, 0x90,

    /* U+0065 "e" */
    0x3c, 0x42, 0xc1, 0x81, 0xff, 0x80, 0x81, 0x42,
    0x3c,

    /* U+0066 "f" */
    0x1f, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10,

    /* U+0067 "g" */
    0x3e, 0xb1, 0x70, 0x70, 0x18, 0xc, 0x7, 0x6,
    0xc7, 0x3e, 0x80, 0x40, 0x20, 0x17, 0xf8,

    /* U+0068 "h" */
    0x80, 0x80, 0x80, 0xbc, 0xc2, 0x81, 0x81, 0x81,
    0x81, 0x81, 0x81, 0x81,

    /* U+0069 "i" */
    0x38, 0x70, 0x7, 0x81, 0x2, 0x4, 0x8, 0x10,
    0x20, 0x47, 0xf0,

    /* U+006A "j" */
    0x1c, 0x70, 0x3e, 0x8, 0x20, 0x82, 0x8, 0x20,
    0x82, 0x8, 0x20, 0xbe,

    /* U+006B "k" */
    0x81, 0x2, 0x4, 0x28, 0xd3, 0x2c, 0x70, 0xb1,
    0x32, 0x34, 0x30,

    /* U+006C "l" */
    0xf0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0xff,

    /* U+006D "m" */
    0xb7, 0x64, 0x62, 0x31, 0x18, 0x8c, 0x46, 0x23,
    0x11, 0x88, 0x80,

    /* U+006E "n" */
    0xbc, 0xc2, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
    0x81,

    /* U+006F "o" */
    0x3e, 0x31, 0xb0, 0x70, 0x18, 0xc, 0x7, 0x6,
    0xc6, 0x3e, 0x0,

    /* U+0070 "p" */
    0x9e, 0x51, 0xb0, 0x70, 0x18, 0xc, 0x7, 0x7,
    0xc6, 0xbe, 0x40, 0x20, 0x10, 0x8, 0x0,

    /* U+0071 "q" */
    0x3c, 0xb1, 0x70, 0x70, 0x18, 0xc, 0x7, 0x6,
    0xc7, 0x3e, 0x80, 0x40, 0x20, 0x10, 0x8,

    /* U+0072 "r" */
    0xef, 0x18, 0xc8, 0x24, 0x2, 0x1, 0x0, 0x80,
    0x40, 0xfc, 0x0,

    /* U+0073 "s" */
    0x79, 0xa, 0xf, 0x7, 0xc0, 0x60, 0xe3, 0x7c,

    /* U+0074 "t" */
    0x10, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x1f,

    /* U+0075 "u" */
    0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x43,
    0x3d,

    /* U+0076 "v" */
    0xc1, 0xe0, 0x90, 0xcc, 0x62, 0x21, 0xb0, 0xd8,
    0x28, 0x1c, 0x0,

    /* U+0077 "w" */
    0xce, 0x69, 0x49, 0x29, 0x25, 0x24, 0xa4, 0xf7,
    0x9e, 0xf3, 0xce, 0x31, 0x80,

    /* U+0078 "x" */
    0x41, 0x31, 0x88, 0x86, 0xc1, 0xc1, 0xb0, 0x88,
    0xc6, 0xc1, 0x80,

    /* U+0079 "y" */
    0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x43,
    0x3d, 0x1, 0x1, 0x1, 0x7f,

    /* U+007A "z" */
    0xfe, 0x4, 0x10, 0x41, 0x4, 0x30, 0x40, 0xfe,

    /* U+007B "{" */
    0x39, 0x8, 0x42, 0x10, 0x84, 0xe7, 0x8, 0x42,
    0x10, 0x84, 0x21, 0xc0,

    /* U+007C "|" */
    0xff, 0xff, 0xc0,

    /* U+007D "}" */
    0xe1, 0x8, 0x42, 0x10, 0x84, 0x39, 0xc8, 0x42,
    0x10, 0x84, 0x27, 0x0,

    /* U+007E "~" */
    0x38, 0xb5, 0xa3, 0x80,

    /* U+00C4 "Ä" */
    0x3f, 0xf, 0xc0, 0x0, 0x0, 0xc, 0x7, 0x81,
    0xe0, 0x48, 0x33, 0xc, 0xc2, 0x11, 0x86, 0x7f,
    0x90, 0x2c, 0xf, 0x3,

    /* U+00D6 "Ö" */
    0x77, 0x3b, 0x80, 0x0, 0x3, 0xe3, 0x1b, 0x7,
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc, 0xd,
    0x4, 0x7c,

    /* U+00DC "Ü" */
    0x77, 0x77, 0x0, 0x0, 0x81, 0x81, 0x81, 0x81,
    0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c,

    /* U+00DF "ß" */
    0x3e, 0x31, 0x90, 0x58, 0x64, 0xf2, 0x61, 0x3c,
    0x83, 0x40, 0xa8, 0x56, 0x69, 0xe0,

    /* U+00E4 "ä" */
    0x77, 0x3b, 0x80, 0x7, 0xa4, 0x34, 0xa, 0x5,
    0x2, 0x81, 0x40, 0x90, 0xc7, 0xb0,

    /* U+00F6 "ö" */
    0x77, 0x3b, 0x80, 0x7, 0xc6, 0x36, 0xe, 0x3,
    0x1, 0x80, 0xe0, 0xd8, 0xc7, 0xc0,

    /* U+00FC "ü" */
    0x6e, 0x6e, 0x0, 0x0, 0x81, 0x81, 0x81, 0x81,
    0x81, 0x81, 0x81, 0x43, 0x3d
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 176, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 176, .box_w = 3, .box_h = 12, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 6, .adv_w = 176, .box_w = 6, .box_h = 4, .ofs_x = 3, .ofs_y = 8},
    {.bitmap_index = 9, .adv_w = 176, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 18, .adv_w = 176, .box_w = 7, .box_h = 11, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 28, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 42, .adv_w = 176, .box_w = 10, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 57, .adv_w = 176, .box_w = 3, .box_h = 4, .ofs_x = 4, .ofs_y = 8},
    {.bitmap_index = 59, .adv_w = 176, .box_w = 7, .box_h = 18, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 75, .adv_w = 176, .box_w = 7, .box_h = 18, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 91, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 102, .adv_w = 176, .box_w = 8, .box_h = 7, .ofs_x = 2, .ofs_y = 2},
    {.bitmap_index = 109, .adv_w = 176, .box_w = 3, .box_h = 4, .ofs_x = 4, .ofs_y = -2},
    {.bitmap_index = 111, .adv_w = 176, .box_w = 5, .box_h = 1, .ofs_x = 3, .ofs_y = 4},
    {.bitmap_index = 112, .adv_w = 176, .box_w = 3, .box_h = 2, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 113, .adv_w = 176, .box_w = 7, .box_h = 16, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 127, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 141, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 155, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 167, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 179, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 193, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 205, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 217, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 229, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 241, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 253, .adv_w = 176, .box_w = 3, .box_h = 9, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 257, .adv_w = 176, .box_w = 3, .box_h = 11, .ofs_x = 4, .ofs_y = -2},
    {.bitmap_index = 262, .adv_w = 176, .box_w = 8, .box_h = 8, .ofs_x = 2, .ofs_y = 2},
    {.bitmap_index = 270, .adv_w = 176, .box_w = 8, .box_h = 4, .ofs_x = 2, .ofs_y = 4},
    {.bitmap_index = 274, .adv_w = 176, .box_w = 8, .box_h = 8, .ofs_x = 2, .ofs_y = 2},
    {.bitmap_index = 282, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 294, .adv_w = 176, .box_w = 9, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 306, .adv_w = 176, .box_w = 11, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 323, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 337, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 351, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 363, .adv_w = 176, .box_w = 7, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 374, .adv_w = 176, .box_w = 7, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 385, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 399, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 411, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 423, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 437, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 451, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 463, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 477, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 489, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 503, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 515, .adv_w = 176, .box_w = 9, .box_h = 15, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 532, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 544, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 558, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 572, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 584, .adv_w = 176, .box_w = 11, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 601, .adv_w = 176, .box_w = 11, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 618, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 632, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 646, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 660, .adv_w = 176, .box_w = 3, .box_h = 18, .ofs_x = 5, .ofs_y = -3},
    {.bitmap_index = 667, .adv_w = 176, .box_w = 7, .box_h = 16, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 681, .adv_w = 176, .box_w = 3, .box_h = 18, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 688, .adv_w = 176, .box_w = 9, .box_h = 6, .ofs_x = 1, .ofs_y = 7},
    {.bitmap_index = 695, .adv_w = 176, .box_w = 8, .box_h = 1, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 696, .adv_w = 176, .box_w = 3, .box_h = 2, .ofs_x = 4, .ofs_y = 10},
    {.bitmap_index = 697, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 708, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 722, .adv_w = 176, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 731, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 745, .adv_w = 176, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 754, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 766, .adv_w = 176, .box_w = 9, .box_h = 13, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 781, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 793, .adv_w = 176, .box_w = 7, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 804, .adv_w = 176, .box_w = 6, .box_h = 16, .ofs_x = 3, .ofs_y = -4},
    {.bitmap_index = 816, .adv_w = 176, .box_w = 7, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 827, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 839, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 850, .adv_w = 176, .box_w = 8, .box_h = 9, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 859, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 870, .adv_w = 176, .box_w = 9, .box_h = 13, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 885, .adv_w = 176, .box_w = 9, .box_h = 13, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 900, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 911, .adv_w = 176, .box_w = 7, .box_h = 9, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 919, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 931, .adv_w = 176, .box_w = 8, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 940, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 951, .adv_w = 176, .box_w = 11, .box_h = 9, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 964, .adv_w = 176, .box_w = 9, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 975, .adv_w = 176, .box_w = 8, .box_h = 13, .ofs_x = 2, .ofs_y = -4},
    {.bitmap_index = 988, .adv_w = 176, .box_w = 7, .box_h = 9, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 996, .adv_w = 176, .box_w = 5, .box_h = 18, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 1008, .adv_w = 176, .box_w = 1, .box_h = 18, .ofs_x = 5, .ofs_y = -3},
    {.bitmap_index = 1011, .adv_w = 176, .box_w = 5, .box_h = 18, .ofs_x = 3, .ofs_y = -3},
    {.bitmap_index = 1023, .adv_w = 176, .box_w = 9, .box_h = 3, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 1027, .adv_w = 176, .box_w = 10, .box_h = 16, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1047, .adv_w = 176, .box_w = 9, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1065, .adv_w = 176, .box_w = 8, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1081, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1095, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1109, .adv_w = 176, .box_w = 9, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1123, .adv_w = 176, .box_w = 8, .box_h = 13, .ofs_x = 1, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_1[] = {
    0x0, 0x12, 0x18, 0x1b, 0x20, 0x32, 0x38
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    },
    {
        .range_start = 196, .range_length = 57, .glyph_id_start = 96,
        .unicode_list = unicode_list_1, .glyph_id_ofs_list = NULL, .list_length = 7, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t font_space_mono_regular_18 = {
#else
lv_font_t font_space_mono_regular_18 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 20,          /*The maximum line height required by the font*/
    .base_line = 4,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if FONT_SPACE_MONO_REGULAR_18*/
