#ifndef DEHX7800DAB_ICONS_H
#define DEHX7800DAB_ICONS_H

#include <stdint.h>

void draw_bluetoothIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_iPhoneIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_lupeIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_notekeyIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_phoneIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_repeatIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_shuffleIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_TAIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_TPIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);
void draw_SRtrvIcon(uint16_t x, uint16_t y, uint16_t fgcolor, uint16_t bgcolor);

static unsigned int bluetoothIcon_width=9;
static unsigned int bluetoothIcon_height=13;
static uint8_t bluetoothIcon_img_data[] = {
0x3e,0x3f,0x9d,0xde,0x7d,0x5f,0x1f,0xdf,0xc7,0xd5,0xf9,0xdd,0xcf,0xe3,0xe0
};

static unsigned int iPhoneIcon_width=14;
static unsigned int iPhoneIcon_height=13;
static uint8_t iPhoneIcon_img_data[] = {
0x00,0x78,0x07,0xe0,0x18,0xc0,0x43,0x0d,0x08,0x70,0x23,0xf8,0x9f,0xe4,0xfe,0x13,
0xe0,0x4f,0xa3,0x1e,0xfc,0x13,0xc0
};

static unsigned int lupeIcon_width=13;
static unsigned int lupeIcon_height=13;
static uint8_t lupeIcon_img_data[] = {
0x01,0xc0,0x1f,0x01,0x8c,0x18,0x30,0xc1,0x86,0x0c,0x18,0xc1,0xfc,0x1d,0xc1,0xc0,
0x1c,0x01,0xc0,0x04,0x00,0x00
};

static unsigned int notekeyIcon_width=12;
static unsigned int notekeyIcon_height=13;
static uint8_t notekeyIcon_img_data[] = {
0x00,0x00,0x0f,0x07,0xf0,0xf9,0x08,0x10,0x81,0x08,0x10,0x81,0x08,0x31,0x8f,0x79,
0xff,0x8e,0x70,0x00
};

static unsigned int phoneIcon_width=6;
static unsigned int phoneIcon_height=13;
static uint8_t phoneIcon_img_data[] = {
0x04,0x1f,0xe1,0x86,0x18,0x61,0xff,0xff,0xff,0xfc
};

static unsigned int repeatIcon_width=21;
static unsigned int repeatIcon_height=13;
static uint8_t repeatIcon_img_data[] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x30,0x01,0xff,0xc0,0x1f,0xff,0x30,
0xc0,0x01,0x86,0x00,0x0c,0x33,0xff,0xe0,0x0f,0xfe,0x00,0x30,0x00,0x00,0x80,0x00,
0x00,0x00,0x00
};

static unsigned int shuffleIcon_width=21;
static unsigned int shuffleIcon_height=13;
static uint8_t shuffleIcon_img_data[] = {
0x00,0x00,0x02,0x03,0x87,0x98,0x36,0x1c,0x63,0x18,0xe1,0x30,0x4d,0x03,0x00,0xc1,
0x30,0x0c,0x0b,0x00,0xc0,0x73,0x0c,0xc3,0x8c,0xc3,0x1e,0x3c,0x0c,0x00,0xc0,0x00,
0x00,0x00,0x00
};

static unsigned int TAIcon_width=20;
static unsigned int TAIcon_height=9;
static uint8_t TAIcon_img_data[] = {
    0x7f,0xff,0xee,0x0e,0x0f,0xfb,0xdf,0x7f,0xbd,0xf7,0xfb,0xc0,0x7f,0xbd,0xf7,0xfb,
    0xdf,0x7f,0xbd,0xf7,0x7f,0xff,0xe0
};

static unsigned int TPIcon_width=20;
static unsigned int TPIcon_height=9;
static uint8_t TPIcon_img_data[] = {
    0x7f,0xff,0xee,0x0e,0x0f,0xfb,0xdf,0x7f,0xbd,0xf7,0xfb,0xc0,0xff,0xbd,0xff,0xfb,
    0xdf,0xff,0xbd,0xff,0x7f,0xff,0xe0
};

static unsigned int SRtrvIcon_width=44;
static unsigned int SRtrvIcon_height=9;
static uint8_t SRtrvIcon_img_data[] = {
    0x7f,0xff,0xff,0xff,0xff,0xef,0xc1,0xf0,0x3f,0xff,0xff,0xfb,0xff,0x7d,0xbf,0xff,
    0xff,0xbf,0xf7,0xd0,0xc6,0xef,0xfc,0x3f,0x03,0xbd,0x2e,0xff,0xfd,0xf7,0xdb,0xdf,
    0x5f,0xfb,0xdf,0x7d,0xad,0xf1,0xff,0x83,0xb7,0xd8,0xdf,0xbf,0x7f,0xff,0xff,0xff,
    0xff,0xe0
};

#endif // DEHX7800DAB_ICONS_H
