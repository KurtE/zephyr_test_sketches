//***************************************************
// https://github.com/kurte/ILI9341_GIGA_n
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library
//
// Warning this is Kurt's updated version which allows it to work on different SPI busses.
//
// On Teensy 3.x allows you to use on only one valid hardware CS pin  which must 
// be used for DC
//
// On Teensy 4.x including Micromod you are free to use any digital pin for
// CS and DC, but you might get a modest speed increase if hardware CS is
// used for the DC pin
//
/***************************************************
  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// <SoftEgg>

// Additional graphics routines by Tim Trzepacz, SoftEgg LLC added December 2015
//(And then accidentally deleted and rewritten March 2016. Oops!)
// Gradient support
//----------------
//		fillRectVGradient	- fills area with vertical gradient
//		fillRectHGradient	- fills area with horizontal gradient
//		fillScreenVGradient - fills screen with vertical gradient
// 	fillScreenHGradient - fills screen with horizontal gradient

// Additional Color Support
//------------------------
//		color565toRGB		- converts 565 format 16 bit color to
//RGB
//		color565toRGB14		- converts 16 bit 565 format color to
//14 bit RGB (2 bits clear for math and sign)
//		RGB14tocolor565		- converts 14 bit RGB back to 16 bit
//565 format color

// Low Memory Bitmap Support
//-------------------------
// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap

// String Pixel Length support
//---------------------------
//		strPixelLen			- gets pixel length of given ASCII
//string

// <\SoftEgg>
// Also some of this comes from the DMA version of the library...

/* ILI9341_t3DMA library code is placed under the MIT license
 * Copyright (c) 2016 Frank Bösing
 *
*/

#ifndef _ILI9341_GIGA_N_H_
#define _ILI9341_GIGA_N_H_

// Allow us to enable or disable capabilities, particully Frame Buffer and
// Clipping for speed and size
#ifndef DISABLE_ILI9341_FRAMEBUFFER
// disable for first pass
#define ENABLE_ILI9341_FRAMEBUFFER
#endif

#define LATER_TEXT


// Allow way to override using SPI

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>

#include "ILI9341_fonts.h"
#include "UARTDevice.h"

#define ILI9341_TFTWIDTH 240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP 0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID 0x04
#define ILI9341_RDDST 0x09

#define ILI9341_SLPIN 0x10
#define ILI9341_SLPOUT 0x11
#define ILI9341_PTLON 0x12
#define ILI9341_NORON 0x13

#define ILI9341_RDMODE 0x0A
#define ILI9341_RDMADCTL 0x0B
#define ILI9341_RDPIXFMT 0x0C
#define ILI9341_RDIMGFMT 0x0D
#define ILI9341_RDSELFDIAG 0x0F

#define ILI9341_INVOFF 0x20
#define ILI9341_INVON 0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON 0x29

#define ILI9341_CASET 0x2A
#define ILI9341_PASET 0x2B
#define ILI9341_RAMWR 0x2C
#define ILI9341_RAMRD 0x2E

#define ILI9341_PTLAR 0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL 0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT 0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR 0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1 0xC0
#define ILI9341_PWCTR2 0xC1
#define ILI9341_PWCTR3 0xC2
#define ILI9341_PWCTR4 0xC3
#define ILI9341_PWCTR5 0xC4
#define ILI9341_VMCTR1 0xC5
#define ILI9341_VMCTR2 0xC7

#define ILI9341_RDID1 0xDA
#define ILI9341_RDID2 0xDB
#define ILI9341_RDID3 0xDC
#define ILI9341_RDID4 0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define ILI9341_BLACK 0x0000       /*   0,   0,   0 */
#define ILI9341_NAVY 0x000F        /*   0,   0, 128 */
#define ILI9341_DARKGREEN 0x03E0   /*   0, 128,   0 */
#define ILI9341_DARKCYAN 0x03EF    /*   0, 128, 128 */
#define ILI9341_MAROON 0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE 0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE 0x7BE0       /* 128, 128,   0 */
#define ILI9341_LIGHTGREY 0xC618   /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF    /* 128, 128, 128 */
#define ILI9341_BLUE 0x001F        /*   0,   0, 255 */
#define ILI9341_GREEN 0x07E0       /*   0, 255,   0 */
#define ILI9341_CYAN 0x07FF        /*   0, 255, 255 */
#define ILI9341_RED 0xF800         /* 255,   0,   0 */
#define ILI9341_MAGENTA 0xF81F     /* 255,   0, 255 */
#define ILI9341_YELLOW 0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE 0xFFFF       /* 255, 255, 255 */
#define ILI9341_ORANGE 0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5 /* 173, 255,  47 */
#define ILI9341_PINK 0xF81F


#ifndef CL
#define CL(_r,_g,_b) ((((_r)&0xF8)<<8)|(((_g)&0xFC)<<3)|((_b)>>3))
#endif
#define sint16_t int16_t

// Lets see about supporting Adafruit fonts as well?
#if __has_include(<gfxfont.h>)
#include <gfxfont.h>
#endif

#ifndef _GFXFONT_H_
#define _GFXFONT_H_

/// Font data stored PER GLYPH
typedef struct {
  uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
  uint8_t width;         ///< Bitmap dimensions in pixels
  uint8_t height;        ///< Bitmap dimensions in pixels
  uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
  int8_t xOffset;        ///< X dist from cursor pos to UL corner
  int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct {
  uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
  GFXglyph *glyph;  ///< Glyph array
  uint8_t first;    ///< ASCII extents (first char)
  uint8_t last;     ///< ASCII extents (last char)
  uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;

#endif // _GFXFONT_H_

// These enumerate the text plotting alignment (reference datum point)
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define CL_DATUM 3 // Centre left, same as above
#define MC_DATUM 4 // Middle centre
#define CC_DATUM 4 // Centre centre, same as above
#define MR_DATUM 5 // Middle right
#define CR_DATUM 5 // Centre right, same as above
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
//#define L_BASELINE  9 // Left character baseline (Line the 'A' character would
//sit on)
//#define C_BASELINE 10 // Centre character baseline
//#define R_BASELINE 11 // Right character baseline

#ifdef __cplusplus
// At all other speeds, _pspi->beginTransaction() will use the fastest available
// clock
#define ILI9341_SPICLOCK 30000000
#define ILI9341_SPICLOCK_READ 1000000

class ILI9341_GIGA_n /*: public Print*/ {
public:
  // Constructor
  //   pspi: either  &SPI (6 pin spi connector) or &SPI1 (shield pins)
  //   CS: Chip select pin,  DC: Data/Command pin
  //   RST: optional reset pin
  ILI9341_GIGA_n(struct spi_dt_spec * pspi, const struct gpio_dt_spec *CS, const struct gpio_dt_spec *DC, const struct gpio_dt_spec *RST);
  

  // Begin - main method to initialze the display.
  void begin(uint32_t spi_clock = ILI9341_SPICLOCK,
             uint32_t spi_clock_read = ILI9341_SPICLOCK_READ);
  void sleep(bool enable);
  void setDebugUART(UARTDevice *pser) {
    _pserDBG = pser;
  }

  void pushColor(uint16_t color);
  void fillScreen(uint16_t color);
  inline void fillWindow(uint16_t color) { fillScreen(color); }
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  void fillRectHGradient(int16_t x, int16_t y, int16_t w, int16_t h,
                         uint16_t color1, uint16_t color2);
  void fillRectVGradient(int16_t x, int16_t y, int16_t w, int16_t h,
                         uint16_t color1, uint16_t color2);
  void fillScreenVGradient(uint16_t color1, uint16_t color2);
  void fillScreenHGradient(uint16_t color1, uint16_t color2);

  void setRotation(uint8_t r);
  void setScrollMargins(uint16_t top, uint16_t bottom);
  void setScroll(uint16_t offset);
  void invertDisplay(bool i);
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  // Pass 8-bit (each) R,G,B, get back 16-bit packed color
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

  // color565toRGB		- converts 565 format 16 bit color to RGB
  static void color565toRGB(uint16_t color, uint8_t &r, uint8_t &g,
                            uint8_t &b) {
    r = (color >> 8) & 0x00F8;
    g = (color >> 3) & 0x00FC;
    b = (color << 3) & 0x00F8;
  }

  // color565toRGB14		- converts 16 bit 565 format color to 14 bit RGB (2
  // bits clear for math and sign)
  // returns 00rrrrr000000000,00gggggg00000000,00bbbbb000000000
  // thus not overloading sign, and allowing up to double for additions for
  // fixed point delta
  static void color565toRGB14(uint16_t color, int16_t &r, int16_t &g,
                              int16_t &b) {
    r = (color >> 2) & 0x3E00;
    g = (color << 3) & 0x3F00;
    b = (color << 9) & 0x3E00;
  }

  // RGB14tocolor565		- converts 14 bit RGB back to 16 bit 565 format
  // color
  static uint16_t RGB14tocolor565(int16_t r, int16_t g, int16_t b) {
    return (((r & 0x3E00) << 2) | ((g & 0x3F00) >> 3) | ((b & 0x3E00) >> 9));
  }

  // uint8_t readdata(void);
  uint8_t readcommand8(uint8_t reg, uint8_t index = 0);
  uint16_t readScanLine();
  void setFrameRateControl(uint8_t mode);

  // Added functions to read pixel data...
  uint16_t readPixel(int16_t x, int16_t y);
  void readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);
  
  // This method writes a rectangle of pixel data either to the screen or to frame buffer
  // this is like the fillRect, except instead of contant color, the array contains the
  // the color for each pixel.
  void writeRect(int16_t x, int16_t y, int16_t w, int16_t h,
                 const uint16_t *pcolors);

  // The write sub-rect is like the writeRect, except we only want to output a portion of it, so it needs to
  // skip through portions of the pcolor array to keep things aligned.
  void writeSubImageRect(int16_t x, int16_t y, int16_t w, int16_t h, 
                        int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, 
                        const uint16_t *pcolors);
  void writeSubImageRectBytesReversed(int16_t x, int16_t y, int16_t w, int16_t h, 
                        int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, 
                        const uint16_t *pcolors);
  // writeRect8BPP - 	write 8 bit per pixel paletted bitmap
  //					bitmap data in array at pixels, one byte per
  //pixel
  //					color palette data in array at palette
  void writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                     const uint8_t *pixels, const uint16_t *palette);

  // writeRect4BPP - 	write 4 bit per pixel paletted bitmap
  //					bitmap data in array at pixels, 4 bits per
  //pixel
  //					color palette data in array at palette
  //					width must be at least 2 pixels
  void writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                     const uint8_t *pixels, const uint16_t *palette);

  // writeRect2BPP - 	write 2 bit per pixel paletted bitmap
  //					bitmap data in array at pixels, 4 bits per
  //pixel
  //					color palette data in array at palette
  //					width must be at least 4 pixels
  void writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                     const uint8_t *pixels, const uint16_t *palette);

  // writeRect1BPP - 	write 1 bit per pixel paletted bitmap
  //					bitmap data in array at pixels, 4 bits per
  //pixel
  //					color palette data in array at palette
  //					width must be at least 8 pixels
  void writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                     const uint8_t *pixels, const uint16_t *palette);

  // writeRectNBPP - 	write N(1, 2, 4, 8) bit per pixel paletted bitmap
  //					bitmap data in array at pixels
  //  Currently writeRect1BPP, writeRect2BPP, writeRect4BPP use this to do all
  //  of the work.
  //
  void writeRectNBPP(int16_t x, int16_t y, int16_t w, int16_t h,
                     uint8_t bits_per_pixel, const uint8_t *pixels,
                     const uint16_t *palette);

  // from Adafruit_GFX.h
  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
                        uint16_t color);
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
                        int16_t delta, uint16_t color);
  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, uint16_t color);
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                    int16_t y2, uint16_t color);
  void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
                     int16_t radius, uint16_t color);
  void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
                     int16_t radius, uint16_t color);
  void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w,
                  int16_t h, uint16_t color);
  void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
                uint16_t bg, uint8_t size_x, uint8_t size_y);
  void inline drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
                       uint16_t bg, uint8_t size) {
    drawChar(x, y, c, color, bg, size);
  }
  #ifndef CENTER
  static const int16_t CENTER = 9998;
  #endif
  void setCursor(int16_t x, int16_t y, bool autoCenter = false);
  void getCursor(int16_t *x, int16_t *y);
  void setTextColor(uint16_t c);
  void setTextColor(uint16_t c, uint16_t bg);
  void setTextSize(uint8_t sx, uint8_t sy);
  void inline setTextSize(uint8_t s) { setTextSize(s, s); }
  uint8_t getTextSizeX() { return textsize_x; }
  uint8_t getTextSizeY() { return textsize_y; }
  uint8_t getTextSize();
  void setTextWrap(bool w);
  bool getTextWrap();

  // setOrigin sets an offset in display pixels where drawing to (0,0) will
  // appear
  // for example: setOrigin(10,10); drawPixel(5,5); will cause a pixel to be
  // drawn at hardware pixel (15,15)
  void setOrigin(int16_t x = 0, int16_t y = 0) {
    _originx = x;
    _originy = y;
    // if (Serial) Serial.printf("Set Origin %d %d\n", x, y);
    updateDisplayClip();
  }
  void getOrigin(int16_t *x, int16_t *y) {
    *x = _originx;
    *y = _originy;
  }

  // setClipRect() sets a clipping rectangle (relative to any set origin) for
  // drawing to be limited to.
  // Drawing is also restricted to the bounds of the display

  void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h) {
    _clipx1 = x1;
    _clipy1 = y1;
    _clipx2 = x1 + w;
    _clipy2 = y1 + h;
    // if (Serial) Serial.printf("Set clip Rect %d %d %d %d\n", x1, y1, w, h);
    updateDisplayClip();
  }
  void setClipRect() {
    _clipx1 = 0;
    _clipy1 = 0;
    _clipx2 = _width;
    _clipy2 = _height;
    // if (Serial) Serial.printf("clear clip Rect\n");
    updateDisplayClip();
  }

  // overwrite print functions:

  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);
  size_t println(const char *sz);
  int16_t width(void) { return _width; }
  int16_t height(void) { return _height; }
  uint8_t getRotation(void);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  int16_t getCursorX(void) const { return cursor_x; }
  int16_t getCursorY(void) const { return cursor_y; }
#ifdef LATER_TEXT
  void setFont(const ILI9341_t3_font_t &f);
  void setFont(const GFXfont *f = NULL);
  void setFontAdafruit(void) { setFont(); }
  void drawFontChar(unsigned int c);
  void drawGFXFontChar(unsigned int c);
 
  void getTextBounds(const uint8_t *buffer, uint16_t len, int16_t x, int16_t y,
                     int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
  void getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1,
                     int16_t *y1, uint16_t *w, uint16_t *h);
//  void getTextBounds(const String &str, int16_t x, int16_t y, int16_t *x1,
//                     int16_t *y1, uint16_t *w, uint16_t *h);
  int16_t strPixelLen(const char *str, uint16_t cb=0xffff);  // optional number of characters...
  // Added for compatibility with ILI9341_t3
  uint16_t measureTextWidth(const uint8_t* text, int chars = 0);
  uint16_t measureTextHeight(const uint8_t* text, int chars = 0);

  // added support for drawing strings/numbers/floats with centering
  // modified from tft_ili9341_ESP github library
  // Handle numbers
  int16_t drawNumber(long long_num, int poX, int poY);
  int16_t drawFloat(float floatNumber, int decimal, int poX, int poY);
  // Handle char arrays
//  int16_t drawString(const String &string, int poX, int poY);
  int16_t drawString(const char string[], int16_t len, int poX, int poY);

  void setTextDatum(uint8_t datum);

  // added support for scrolling text area
  // https://github.com/vitormhenrique/ILI9341_t3
  // Discussion regarding this optimized version:
  // http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library
  //
  void setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h);
  void setScrollBackgroundColor(uint16_t color);
  void enableScroll(void);
  void disableScroll(void);
  void scrollTextArea(uint8_t scrollSize);
  void resetScrollBackgroundColor(uint16_t color);
#endif
  // added support to use optional Frame buffer
  enum {
    ILI9341_DMA_INIT = 0x01,
    ILI9341_DMA_EVER_INIT = 0x08,
    ILI9341_DMA_CONT = 0x02,
    ILI9341_DMA_FINISH = 0x04,
    ILI9341_DMA_ACTIVE = 0x80
  };
  void setFrameBuffer(uint16_t *frame_buffer);
  uint8_t
  useFrameBuffer(bool b);  // use the frame buffer?  First call will allocate
  void freeFrameBuffer(void); // explicit call to release the buffer
  void updateScreen(void);    // call to say update the screen now.


#ifdef ENABLE_ILI9341_FRAMEBUFFER
  uint16_t *getFrameBuffer() { return _pfbtft; }
  uint32_t frameCount() { return _dma_frame_count; }
  uint16_t subFrameCount() { return 0 /*_dma_sub_frame_count*/; }
  void setFrameCompleteCB(void (*pcb)(), bool fCallAlsoHalfDone = false);
  void updateChangedAreasOnly(bool updateChangedOnly) {
    _updateChangedAreasOnly = updateChangedOnly;
  }
  bool updateScreenAsync(bool update_cont = false); // call to say update the
                                                    // screen optinoally turn
                                                    // into continuous mode.
  bool asyncUpdateActive(void);
  void waitUpdateAsyncComplete(void);
//  void endUpdateAsync(); // Turn of the continueous mode flag
//  void abortUpdateAsync(); // Use this if there is a hang...
//  void dumpDMASettings();
//  void setSPIDataSize(uint8_t datasize);
//  uint8_t getDMAInterruptStatus();
//  void clearDMAInterruptStatus(uint8_t clear_flags);
#endif

  const struct spi_dt_spec *  _pspi = nullptr;
  const struct spi_dt_spec *_spi_dev = nullptr;
  struct spi_config _config, _config16;

  uint8_t _spi_num = 0;         // Which buss is this spi on?
  uint32_t _SPI_CLOCK = ILI9341_SPICLOCK;      // #define ILI9341_SPICLOCK 30000000
  uint32_t _SPI_CLOCK_READ = ILI9341_SPICLOCK_READ; //#define ILI9341_SPICLOCK_READ 2000000

  int16_t _width = ILI9341_TFTWIDTH;
  int16_t _height = ILI9341_TFTHEIGHT; // Display w/h as modified by current rotation
  int16_t cursor_x = 0; 
  int16_t cursor_y = 0;
  bool _center_x_text = false;
  bool _center_y_text = false;
  int16_t _clipx1 = 0, _clipy1 = 0, _clipx2= ILI9341_TFTWIDTH, _clipy2 = ILI9341_TFTHEIGHT;
  int16_t _originx = 0, _originy = 0;
  int16_t _displayclipx1 = 0, _displayclipy1 = 0, _displayclipx2 = ILI9341_TFTWIDTH, _displayclipy2 = ILI9341_TFTHEIGHT;
  bool _invisible = false;
  bool _standard = true; // no bounding rectangle or origin set.
  
  uint16_t _x0_last = 0xffff;
  uint16_t _x1_last = 0xffff;
  uint16_t _y0_last = 0xffff;
  uint16_t _y1_last = 0xffff;

  inline int min(int x, int y) {return (x <= y)? x : y;}
  inline int max(int x, int y) {return (x >= y)? x : y;}

  inline void updateDisplayClip() {
    _displayclipx1 = max(0, min(_clipx1 + _originx, width()));
    _displayclipx2 = max(0, min(_clipx2 + _originx, width()));

    _displayclipy1 = max(0, min(_clipy1 + _originy, height()));
    _displayclipy2 = max(0, min(_clipy2 + _originy, height()));
    _invisible =
        (_displayclipx1 == _displayclipx2 || _displayclipy1 == _displayclipy2);
    _standard = (_displayclipx1 == 0) && (_displayclipx2 == _width) &&
                (_displayclipy1 == 0) && (_displayclipy2 == _height);
    //if (Serial) {
      // Serial.printf("UDC (%d %d)-(%d %d) %d %d\n", _displayclipx1,
      // _displayclipy1, _displayclipx2,
      //	_displayclipy2, _invisible, _standard);
    //}
  }

  int16_t scroll_x = 0, scroll_y = 0, scroll_width = 0, scroll_height = 0;
  bool scrollEnable = false,
      isWritingScrollArea = false; // If set, 'wrap' text at right edge of display

  uint16_t textcolor = 0xffff, textbgcolor = 0xffff, scrollbgcolor = 0;
  uint32_t textcolorPrexpanded = 0, textbgcolorPrexpanded = 0;
  uint8_t textsize_x = 1, textsize_y = 1; 
  uint8_t rotation = 0;
  uint8_t textdatum = 0;
  bool wrap = true; // If set, 'wrap' text at right edge of display
  const ILI9341_t3_font_t *font = nullptr;
  // Anti-aliased font support
  uint8_t fontbpp = 1;
  uint8_t fontbppindex = 0;
  uint8_t fontbppmask = 1;
  uint8_t fontppb = 8;
  uint8_t *fontalphalut = 0;
  float fontalphamx = 1;

  uint32_t padX = 0;

  // GFX Font support
  const GFXfont *gfxFont = nullptr;
  int8_t _gfxFont_min_yOffset = 0;

  // Opaque font chracter overlap?
  unsigned int _gfx_c_last = 0;
  int16_t _gfx_last_cursor_x = 0, _gfx_last_cursor_y = 0;
  int16_t _gfx_last_char_x_write = 0;
  uint16_t _gfx_last_char_textcolor = 0;
  uint16_t _gfx_last_char_textbgcolor = 0;
  bool gfxFontLastCharPosFG(int16_t x, int16_t y);

  const struct gpio_dt_spec *_cs, *_dc, *_rst;
  UARTDevice *_pserDBG = nullptr;
  #endif
  //uint8_t pcs_data, pcs_command;
  //uint8_t _miso, _mosi, _sclk;

///////////////////////////////
// BUGBUG:: reorganize this area better!
//////////////////////////////

// add support to allow only one hardware CS (used for dc)
  //__IO uint32_t *_dcBSRR; 
  //uint16_t _dcpinmask;
  //__IO uint32_t *_csBSRR; 
  //uint16_t _cspinmask;

//  volatile uint8_t _data_sent_not_completed = 0;  // how much data has been sent that we are waiting for info
  volatile bool  _data_sent_since_last_transmit_complete = 0;  // have we sent anything since


  int16_t _changed_min_x=0, _changed_max_x=0, _changed_min_y=0, _changed_max_y=0;
  bool _updateChangedAreasOnly = false; // current default off,

#ifdef ENABLE_ILI9341_FRAMEBUFFER
  // Add support for optional frame buffer
  uint16_t *_pfbtft;              // Optional Frame buffer
  uint8_t _use_fbtft;             // Are we in frame buffer mode?
  uint16_t *_we_allocated_buffer; // We allocated the buffer;
  void (*_frame_complete_callback)() = nullptr;
  bool _frame_callback_on_HalfDone = false;

  static ILI9341_GIGA_n *_dmaActiveDisplay[2]; // Use pointer to this as a way to get
                                         // back to object...
  //volatile uint8_t _dma_state = 0;            // DMA status
  volatile uint32_t _dma_frame_count = 0;     // Can return a frame count...
  //volatile uint16_t _dma_sub_frame_count = 0; // Can return a frame count...
  volatile bool _async_update_active = false; 
  // GIGA DMA stuff - WIP

  //static void dmaInterrupt(void);
  //static void dmaInterrupt1(void);
  static void async_callback(const struct device *dev, int result, void *data);
  void process_async_callback(void);
#endif
  void charBounds(char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny,
                  int16_t *maxx, int16_t *maxy);

  void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
      __attribute__((always_inline)) {

    //if (_pserDBG) _pserDBG->printf("\tsetAddr(%u, %u, %u, %u)\n", x0, y0, x1, y1);

    if ((x0 != _x0_last) || (x1 != _x1_last)) {
      writecommand_cont(ILI9341_CASET); // Column addr set
      writedata16_cont(x0);             // XSTART
      writedata16_cont(x1);             // XEND
      _x0_last = x0;
      _x1_last = x1;
    }
    if ((y0 != _y0_last) || (y1 != _y1_last)) {
      writecommand_cont(ILI9341_PASET); // Row addr set
      writedata16_cont(y0);             // YSTART
      writedata16_cont(y1);             // YEND
      _y0_last = y0;
      _y1_last = y1;
    }
  }
//. From Onewire utility files

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
  enum {LOW=0, HIGH=1};

  void digitalWrite(const struct gpio_dt_spec *pin, int state) {
    gpio_pin_set_dt(pin, state);
  }

  void beginSPITransaction(uint32_t clock) /*__attribute__((always_inline)) */{
    //memset((void *)&_config, 0, sizeof(_config));
    _config.frequency = clock;
    //_config.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;

    //memset((void *)&_config16, 0, sizeof(_config16));
    _config16.frequency = clock;
    //_config16.operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB;

    digitalWrite(_cs, LOW);

//    _pspi->beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));

  }
  void endSPITransaction() __attribute__((always_inline)) {
  //  _pspi->endTransaction();
    digitalWrite(_cs, HIGH);
  }

  // Start off stupid
  uint8_t _dcpinAsserted = false;


  void setCommandMode() __attribute__((always_inline)) {
    if (!_dcpinAsserted) {
      digitalWrite(_dc, LOW);
      _dcpinAsserted = 1;
    }
  }

  void setDataMode() __attribute__((always_inline)) {
    if (_dcpinAsserted) {
      digitalWrite(_dc, HIGH);
      _dcpinAsserted = 0;
    }
  }

  void outputToSPI(uint8_t c) {
    const struct spi_buf tx_buf = {.buf = &c, .len = sizeof(c)};
    const struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1,
    };

//    uint8_t c_ret;
//    const struct spi_buf rx_buf = {.buf = &c_ret, .len = sizeof(c)};
//    const struct spi_buf_set rx_buf_set = {
//        .buffers = &rx_buf,
//        .count = 1,
//    };

//    spi_transceive(_spi_dev->bus, &_config, &tx_buf_set, &rx_buf_set);
    spi_write(_spi_dev->bus, &_config, &tx_buf_set);

    //_pspi->transfer(c);
  }

  void outputNToSPI(const uint8_t *pc, uint8_t cb) {
//    printk("OutputN(%p, %u): %x %x - %u %x %u\n", pc, cb, pc[0], pc[1], _config.frequency, _config.operation,
#if 0
    uint8_t tmp_buf[16];
    memcpy(tmp_buf, pc, cb);
    const struct spi_buf tx_buf = {.buf = (void*)tmp_buf, .len = cb};
    const struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1,
    };
    spi_write(_spi_dev->bus, &_config, &tx_buf_set);
#else
    while (cb) {
      outputToSPI(*pc++);
      cb--;
    }
#endif    
  }

  void outputToSPI16(uint16_t data) {
    //_pspi->transfer16(data);
    outputToSPI(data >> 8); //MSB
    outputToSPI(data & 0xff);

//    const struct spi_buf tx_buf = {.buf = &data, .len = sizeof(data)};
//    const struct spi_buf_set tx_buf_set = {
//        .buffers = &tx_buf,
//        .count = 1,
//    };

//    spi_write(_spi_dev->bus, &_config16, &tx_buf_set);

    //_pspi->transfer(c);

  }

  void writecommand_cont(uint8_t c) {
    _config.operation |= SPI_HOLD_ON_CS;
    setCommandMode();
    outputToSPI(c);
  }
  void writedata8_cont(uint8_t c) {
    _config.operation |= SPI_HOLD_ON_CS;
    setDataMode();
    outputToSPI(c);
  }

  void writedata16_cont(uint16_t data) {
  //  _config16.operation |= SPI_HOLD_ON_CS;
    setDataMode();
    _config.operation |= SPI_HOLD_ON_CS;
    outputToSPI(data >> 8); //MSB
    outputToSPI(data & 0xff);
//    outputToSPI16(c);
  }

  void writecommand_last(uint8_t c) {
    _config.operation &= ~SPI_HOLD_ON_CS;
    setCommandMode();
    outputToSPI(c);
  }
  void writedata8_last(uint8_t c) {
    _config.operation &= ~SPI_HOLD_ON_CS;
    setDataMode();
    outputToSPI(c);
  }
  void writedata16_last(uint16_t data) {
 //   _config16.operation &= ~SPI_HOLD_ON_CS;
    setDataMode();
    _config.operation |= SPI_HOLD_ON_CS;
    outputToSPI(data >> 8); //MSB
    outputToSPI(data & 0xff);
//    outputToSPI16(c);
  }
//////////////////////////////////////////////////////////////////

  void clearChangedRange() {
    _changed_min_x = 0x7fff;
    _changed_max_x = -1;
    _changed_min_y = 0x7fff;
    _changed_max_y = -1;
  }

  void updateChangedRange(int16_t x, int16_t y, int16_t w, int16_t h)
      __attribute__((always_inline)) {
    if (x < _changed_min_x)
      _changed_min_x = x;
    if (y < _changed_min_y)
      _changed_min_y = y;
    x += w - 1;
    y += h - 1;
    if (x > _changed_max_x)
      _changed_max_x = x;
    if (y > _changed_max_y)
      _changed_max_y = y;
    //if (Serial)Serial.printf("UCR(%d %d %d %d) min:%d %d max:%d %d\n", w, y, w, h, _changed_min_x, _changed_min_y, _changed_max_x, _changed_max_y);
  }

  // could combine with above, but avoids the +-...
  void updateChangedRange(int16_t x, int16_t y) __attribute__((always_inline)) {
    if (x < _changed_min_x)
      _changed_min_x = x;
    if (y < _changed_min_y)
      _changed_min_y = y;
    if (x > _changed_max_x)
      _changed_max_x = x;
    if (y > _changed_max_y)
      _changed_max_y = y;
  }

  void SPITransceiveWriteBufferHelper(uint32_t pixel_count, uint16_t color) {
    // Extracted from several functions, that output the
    // same color N times and we need to keep the CS pin
    // selected
//    if (_pserDBG)_pserDBG->printf("    helper:%u %x)\n", pixel_count, color);
    writecommand_cont(ILI9341_RAMWR);
    setDataMode();
    #define CB_WRITE 160
    uint8_t fill_count = (pixel_count <= (CB_WRITE/2))? pixel_count : (CB_WRITE / 2);
    for (uint16_t i = 0; i < fill_count; i++) s_row_buff[i] = color;

    uint32_t len = pixel_count * 2;
    struct spi_buf tx_buf = { .buf = (void*)s_row_buff, .len = CB_WRITE * 2};
    const struct spi_buf_set tx_buf_set = { .buffers = &tx_buf, .count = 1 };

    if (len > CB_WRITE) {
        _config16.operation |= SPI_HOLD_ON_CS;
        while (len > CB_WRITE) {
            spi_transceive(_spi_dev->bus, &_config16, &tx_buf_set, nullptr);
            len -= CB_WRITE;
        }
      _config16.operation &= ~SPI_HOLD_ON_CS;
    }
    tx_buf.len = len;
    spi_transceive(_spi_dev->bus, &_config16, &tx_buf_set, nullptr);      
  }

  void HLine(int16_t x, int16_t y, int16_t w, uint16_t color)
      __attribute__((always_inline)) {
    if (_pserDBG)_pserDBG->printf("  HL(%d %d %d %x)\n", x, y, w, color);
#ifdef ENABLE_ILI9341_FRAMEBUFFER
    if (_use_fbtft) {
      drawFastHLine(x, y, w, color);
      return;
    }
#endif
    x += _originx;
    y += _originy;

    // Rectangular clipping
    if ((y < _displayclipy1) || (x >= _displayclipx2) || (y >= _displayclipy2))
      return;
    if (x < _displayclipx1) {
      w = w - (_displayclipx1 - x);
      x = _displayclipx1;
    }
    if ((x + w - 1) >= _displayclipx2)
      w = _displayclipx2 - x;
    if (w < 1)
      return;

    setAddr(x, y, x + w - 1, y);
    SPITransceiveWriteBufferHelper(w, color);
  }
  
  void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
      __attribute__((always_inline)) {
    if (_pserDBG)_pserDBG->printf("  VL(%d %d %d %x)\n", x, y, h, color);
#ifdef ENABLE_ILI9341_FRAMEBUFFER
    if (_use_fbtft) {
      drawFastVLine(x, y, h, color);
      return;
    }
#endif
    x += _originx;
    y += _originy;

    // Rectangular clipping
    if ((x < _displayclipx1) || (x >= _displayclipx2) || (y >= _displayclipy2))
      return;
    if (y < _displayclipy1) {
      h = h - (_displayclipy1 - y);
      y = _displayclipy1;
    }
    if ((y + h - 1) >= _displayclipy2)
      h = _displayclipy2 - y;
    if (h < 1)
      return;

    setAddr(x, y, x, y + h - 1);
    SPITransceiveWriteBufferHelper(h, color);
  }
  /**
   * Found in a pull request for the Adafruit framebuffer library. Clever!
   * https://github.com/tricorderproject/arducordermini/pull/1/files#diff-d22a481ade4dbb4e41acc4d7c77f683d
   * Converts  0000000000000000rrrrrggggggbbbbb
   *     into  00000gggggg00000rrrrr000000bbbbb
   * with mask 00000111111000001111100000011111
   * This is useful because it makes space for a parallel fixed-point multiply
   * This implements the linear interpolation formula: result = bg * (1.0 -
   *alpha) + fg * alpha
   * This can be factorized into: result = bg + (fg - bg) * alpha
   * alpha is in Q1.5 format, so 0.0 is represented by 0, and 1.0 is represented
   *by 32
   * @param	fg		Color to draw in RGB565 (16bit)
   * @param	bg		Color to draw over in RGB565 (16bit)
   * @param	alpha	Alpha in range 0-255
   **/
  uint16_t alphaBlendRGB565(uint32_t fg, uint32_t bg, uint8_t alpha)
      __attribute__((always_inline)) {
    alpha = (alpha + 4) >> 3; // from 0-255 to 0-31
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result =
        ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t)((result >> 16) | result); // contract result
  }
  /**
   * Same as above, but fg and bg are premultiplied, and alpah is already in
   * range 0-31
   */
  uint16_t alphaBlendRGB565Premultiplied(uint32_t fg, uint32_t bg,
                                         uint8_t alpha)
      __attribute__((always_inline)) {
    uint32_t result =
        ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t)((result >> 16) | result); // contract result
  }

  void Pixel(int16_t x, int16_t y, uint16_t color)
      __attribute__((always_inline)) {
    x += _originx;
    y += _originy;

    if ((x < _displayclipx1) || (x >= _displayclipx2) || (y < _displayclipy1) ||
        (y >= _displayclipy2))
      return;

#ifdef ENABLE_ILI9341_FRAMEBUFFER
    if (_use_fbtft) {
      updateChangedRange(
          x, y); // update the range of the screen that has been changed;
      _pfbtft[y * _width + x] = color;
      return;
    }
#endif
    // printf("\tPixel(%d, %d, %x)\n", x, y, color);
    setAddr(x, y, x, y);
    writecommand_cont(ILI9341_RAMWR);
    writedata16_cont(color);
    // printf("\tPixel-End\n");
  }
  void drawFontBits(bool opaque, uint32_t bits, uint32_t numbits, int32_t x,
                    int32_t y, uint32_t repeat);
  void drawFontPixel(uint8_t alpha, uint32_t x, uint32_t y);
  uint32_t fetchpixel(const uint8_t *p, uint32_t index, uint32_t x);

  // BUGBUG:: Maybe better way later
  static uint16_t s_row_buff[320]; // 


};

#ifndef ILI9341_swap
#define ILI9341_swap(a, b)                                                     \
  {                                                                            \
    /*typeof(a)*/int16_t t = a;                                                           \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#endif
