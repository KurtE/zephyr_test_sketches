//***************************************************
// https://github.com/kurte/ST77XX_zephyr
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

/* ST77XX_t3DMA library code is placed under the MIT license
 * Copyright (c) 2016 Frank Bösing
 *
*/

#ifndef _ST77XX_zephyr_H_
#define _ST77XX_zephyr_H_

// Allow us to enable or disable capabilities, particully Frame Buffer and
// Clipping for speed and size
#ifndef DISABLE_ST77XX_FRAMEBUFFER
// disable for first pass
#define ENABLE_ST77XX_FRAMEBUFFER
#endif

#define LATER_TEXT


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

#include "ST77XX_zephyr_defines.h"

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

#ifdef __cplusplus
// At all other speeds, _pspi->beginTransaction() will use the fastest available
// clock
#define ST77XX_SPICLOCK 16000000

class ST77XX_zephyr /* : public Print */ {
public:
  // Constructor
  //   pspi: either  &SPI (6 pin spi connector) or &SPI1 (shield pins)
  //   CS: Chip select pin,  DC: Data/Command pin
  //   RST: optional reset pin
  ST77XX_zephyr(struct spi_dt_spec * pspi, const struct gpio_dt_spec *CS, const struct gpio_dt_spec *DC, const struct gpio_dt_spec *RST);
  
  // Begin - main method to initialze the display.
  void commandList(const uint8_t *addr);
  virtual void  begin(uint16_t width=240, uint16_t height=320/*, uint8_t mode=SPI_MODE0 */, uint32_t spi_clock = ST77XX_SPICLOCK );

  // common init stuff used by the different versions init or begin...
  void common_init(const uint8_t *cmd_list);

  void sleep(bool enable);
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

  virtual void setRotation(uint8_t r);
  void     setRowColStart(uint16_t x, uint16_t y);
  uint16_t  rowStart() {return _rowstart;}
  uint16_t  colStart() {return _colstart;}
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

  // Experiment use the callback version (async) amd see if it is faster.
  void writeRectCB(int16_t x, int16_t y, int16_t w, int16_t h,
                 const uint16_t *pcolors, void (*callback)(int result));

  void process_spi_callback (const struct device *dev, int result);

  static void spi_callback (const struct device *dev, int result, void *data) {
    ((ST77XX_zephyr *)data)->process_spi_callback(dev, result);
  }
  void (*_write_rect_cb)(int result) = nullptr;


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
    drawChar(x, y, c, color, bg, size, size);
  }
  #ifndef CENTER
  static const int16_t CENTER = 9998;
//  #endif
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
    // if (Serial) USBSerial.printf("Set Origin %d %d\n", x, y);
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
    // if (Serial) USBSerial.printf("Set clip Rect %d %d %d %d\n", x1, y1, w, h);
    updateDisplayClip();
  }
  void setClipRect() {
    _clipx1 = 0;
    _clipy1 = 0;
    _clipx2 = _width;
    _clipy2 = _height;
    // if (Serial) USBSerial.printf("clear clip Rect\n");
    updateDisplayClip();
  }

  // overwrite print functions:
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

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
  // Added for compatibility with ST77XX_t3
  uint16_t measureTextWidth(const uint8_t* text, int chars = 0);
  uint16_t measureTextHeight(const uint8_t* text, int chars = 0);

  // added support for drawing strings/numbers/floats with centering
  // modified from tft_ST77XX_ESP github library
  // Handle numbers
  int16_t drawNumber(long long_num, int poX, int poY);
  int16_t drawFloat(float floatNumber, int decimal, int poX, int poY);
  // Handle char arrays
//  int16_t drawString(const String &string, int poX, int poY);
  int16_t drawString(const char string[], int16_t len, int poX, int poY);

  void setTextDatum(uint8_t datum);

  // added support for scrolling text area
  // https://github.com/vitormhenrique/ST77XX_t3
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
    ST77XX_DMA_INIT = 0x01,
    ST77XX_DMA_EVER_INIT = 0x08,
    ST77XX_DMA_CONT = 0x02,
    ST77XX_DMA_FINISH = 0x04,
    ST77XX_DMA_ACTIVE = 0x80
  };
  void setFrameBuffer(uint16_t *frame_buffer);
  uint8_t
  useFrameBuffer(bool b);  // use the frame buffer?  First call will allocate
  void freeFrameBuffer(void); // explicit call to release the buffer
  void updateScreen(void);    // call to say update the screen now.


#ifdef ENABLE_ST77XX_FRAMEBUFFER
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

  struct spi_dt_spec *_pspi = nullptr;
  const struct spi_dt_spec *_spi_dev = nullptr;
  struct spi_config _config, _config16;
//  struct spi_config _config16;
//  SPISettings _spiSettings;

  uint8_t _spi_num = 0;         // Which buss is this spi on?
  uint32_t _SPI_CLOCK = ST77XX_SPICLOCK;      // #define ST77XX_SPICLOCK 30000000
//  uint8_t _SPI_MODE = SPI_MODE0;
  int16_t _width = ST77XX_TFTWIDTH;
  int16_t _height = ST77XX_TFTHEIGHT; // Display w/h as modified by current rotation
  int16_t _screenWidth = ST77XX_TFTWIDTH;
  int16_t _screenHeight = ST77XX_TFTHEIGHT; // Display w/h as modified by current rotation
  uint8_t tabcolor = 0;

  int16_t cursor_x = 0; 
  int16_t cursor_y = 0;
  bool _center_x_text = false;
  bool _center_y_text = false;
  int16_t _clipx1 = 0, _clipy1 = 0, _clipx2= ST77XX_TFTWIDTH, _clipy2 = ST77XX_TFTHEIGHT;
  int16_t _originx = 0, _originy = 0;
  int16_t _displayclipx1 = 0, _displayclipy1 = 0, _displayclipx2 = ST77XX_TFTWIDTH, _displayclipy2 = ST77XX_TFTHEIGHT;
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
      // USBSerial.printf("UDC (%d %d)-(%d %d) %d %d\n", _displayclipx1,
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
  uint16_t _colstart = 0, _rowstart = 0, _xstart = 0, _ystart = 0; 
  uint16_t _colstart2 = 0, _rowstart2 = 0;
  uint8_t _rotation = 0;
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

#ifdef ENABLE_ST77XX_FRAMEBUFFER
  // Add support for optional frame buffer
  uint16_t *_pfbtft;              // Optional Frame buffer
  uint8_t _use_fbtft;             // Are we in frame buffer mode?
  uint16_t *_we_allocated_buffer; // We allocated the buffer;
  void (*_frame_complete_callback)() = nullptr;
  bool _frame_callback_on_HalfDone = false;

  static ST77XX_zephyr *_dmaActiveDisplay[2]; // Use pointer to this as a way to get
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
    //if ((x0 != _x0_last) || (x1 != _x1_last)) 
    {
      writecommand_cont(ST77XX_CASET); // Column addr set
      writedata16_cont(x0+_xstart);             // XSTART
      writedata16_cont(x1+_xstart);             // XEND
      _x0_last = x0;
      _x1_last = x1;
    }
    //if ((y0 != _y0_last) || (y1 != _y1_last)) 
    {
      writecommand_cont(ST77XX_RASET); // Row addr set
      writedata16_cont(y0+_ystart);             // YSTART
      writedata16_cont(y1+_ystart);             // YEND
      _y0_last = y0;
      _y1_last = y1;
    }
  }
//. From Onewire utility files

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
  enum {LOW=0, HIGH=1};

  void digitalWrite(const struct gpio_dt_spec *pin, int state) {
    gpio_pin_set_dt(pin, state);
  }

  void beginSPITransaction(uint32_t clock) __attribute__((always_inline)) {
    _config.frequency = clock;
    //_config.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;

    //memset((void *)&_config16, 0, sizeof(_config16));
    _config16.frequency = clock;
    //_config16.operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB;

    digitalWrite(_cs, LOW);
  }

  void beginSPITransaction() __attribute__((always_inline)) {
    _config.frequency = _SPI_CLOCK;
    _config16.frequency = _SPI_CLOCK;
    digitalWrite(_cs, LOW);

  }


  void endSPITransaction() __attribute__((always_inline)) {
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
    spi_write(_spi_dev->bus, &_config, &tx_buf_set);
  }

  void outputToSPI16(uint16_t data) {
    //_pspi->transfer16(data);
    outputToSPI(data >> 8); //MSB
    outputToSPI(data & 0xff);
  }

  void writecommand_cont(uint8_t c) {
    setCommandMode();
    outputToSPI(c);
  }
  void writedata8_cont(uint8_t c) {
    setDataMode();
    outputToSPI(c);
  }

  void writedata16_cont(uint16_t c) {
    setDataMode();
    outputToSPI16(c);
  }

  void writecommand_last(uint8_t c) {
    setCommandMode();
    outputToSPI(c);
  }
  void writedata8_last(uint8_t c) {
    setDataMode();
    outputToSPI(c);
  }
  void writedata16_last(uint16_t c) {
    setDataMode();
    outputToSPI16(c);
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
    //if (Serial)USBSerial.printf("UCR(%d %d %d %d) min:%d %d max:%d %d\n", w, y, w, h, _changed_min_x, _changed_min_y, _changed_max_x, _changed_max_y);
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


  void HLine(int16_t x, int16_t y, int16_t w, uint16_t color)
      __attribute__((always_inline)) {
#ifdef ENABLE_ST77XX_FRAMEBUFFER
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
    writecommand_cont(ST77XX_RAMWR);
    setDataMode();
    for (uint16_t i = 0; i < w; i++) s_row_buff[i] = color;

    struct spi_buf tx_buf = { .buf = (void*)s_row_buff, .len = (size_t)(w * 2 )};
    const struct spi_buf_set tx_buf_set = { .buffers = &tx_buf, .count = 1 };
    spi_transceive(_spi_dev->bus, &_config16, &tx_buf_set, nullptr);
  }
  
  void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
      __attribute__((always_inline)) {
#ifdef ENABLE_ST77XX_FRAMEBUFFER
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
    writecommand_cont(ST77XX_RAMWR);
    setDataMode();
    for (uint16_t i = 0; i < h; i++) s_row_buff[i] = color;

    struct spi_buf tx_buf = { .buf = (void*)s_row_buff, .len = (size_t)(h * 2 )};
    const struct spi_buf_set tx_buf_set = { .buffers = &tx_buf, .count = 1 };
    spi_transceive(_spi_dev->bus, &_config16, &tx_buf_set, nullptr);
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

#ifdef ENABLE_ST77XX_FRAMEBUFFER
    if (_use_fbtft) {
      updateChangedRange(
          x, y); // update the range of the screen that has been changed;
      _pfbtft[y * _width + x] = color;
      return;
    }
#endif
    // printf("\tPixel(%d, %d, %x)\n", x, y, color);
    setAddr(x, y, x, y);
    writecommand_cont(ST77XX_RAMWR);
    writedata16_cont(color);
    // printf("\tPixel-End\n");
  }
  void drawFontBits(bool opaque, uint32_t bits, uint32_t numbits, int32_t x,
                    int32_t y, uint32_t repeat);
  void drawFontPixel(uint8_t alpha, uint32_t x, uint32_t y);
  uint32_t fetchpixel(const uint8_t *p, uint32_t index, uint32_t x);

  // BUGBUG:: Maybe better way later
  static uint16_t s_row_buff[480]; // 


};

#ifndef ST77XX_swap
#define ST77XX_swap(a, b)                                                     \
  {                                                                            \
    /*typeof(a)*/int16_t t = a;                                                           \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif


class ST7796_zephyr : public ST77XX_zephyr   {
public:
  ST7796_zephyr(struct spi_dt_spec * pspi, const struct gpio_dt_spec *CS, const struct gpio_dt_spec *DC, const struct gpio_dt_spec *RST);

  // Allow either init or begin
  void  begin(uint16_t width=320, uint16_t height=480, uint32_t spi_clock = ST77XX_SPICLOCK);

  void  init(uint16_t width=320, uint16_t height=480, uint32_t spi_clock = ST77XX_SPICLOCK) {
      begin(width, height, spi_clock);
  }

  virtual void  setRotation(uint8_t m);

};

class ST7789_zephyr : public ST77XX_zephyr  {
public:
  ST7789_zephyr(struct spi_dt_spec * pspi, const struct gpio_dt_spec *CS, const struct gpio_dt_spec *DC, const struct gpio_dt_spec *RST);

  void  begin(uint16_t width=240, uint16_t height=240, uint32_t spi_clock = ST77XX_SPICLOCK);
  void  init(uint16_t width=240, uint16_t height=240, uint32_t spi_clock = ST77XX_SPICLOCK) {
      begin(width, height, spi_clock);
  }
  virtual void  setRotation(uint8_t m);

};


// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB 0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1
#define INITR_144GREENTAB_OFFSET   0x4
#define INITR_MINI160x80  0x05
#define INITR_MINI160x80_ST7735S 0x06


class ST7735_zephyr : public ST77XX_zephyr  {
public:
  ST7735_zephyr(struct spi_dt_spec * pspi, const struct gpio_dt_spec *CS, const struct gpio_dt_spec *DC, const struct gpio_dt_spec *RST);

  void initB(uint32_t spi_clock = ST77XX_SPICLOCK);                             // for ST7735B displays
  void initR(uint8_t options = INITR_GREENTAB, uint32_t spi_clock = ST77XX_SPICLOCK); // for ST7735R
  virtual void  setRotation(uint8_t m);
};



#endif // __cplusplus

#endif
