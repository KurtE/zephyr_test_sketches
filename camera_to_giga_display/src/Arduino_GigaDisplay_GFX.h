
#ifndef __ARDUINO_GIGADISPLAY_GFX__
#define __ARDUINO_GIGADISPLAY_GFX__
#ifdef LATER
#include "Adafruit_GFX.h"
#include "Arduino_GigaDisplay.h"
//#include "Adafruit_SPITFT.h"


#include "SDRAM.h"

class GigaDisplay_GFX : public Adafruit_GFX {
  public:
    GigaDisplay_GFX();
    ~GigaDisplay_GFX(void);
    void begin();
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void fillScreen(uint16_t color);
    void byteSwap(void);
    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    uint16_t getPixel(int16_t x, int16_t y);
    uint16_t *getBuffer(void) {
      return buffer;
    }
    uint16_t *hasBuffer(void) {
      if (!buffer) {
        begin();
      }
      return buffer;
    }

    void drawGrayscaleBitmapScaled(int16_t width_image, int16_t height_image, uint8_t scale, uint8_t *pixels);
    void drawRGBBitmapScaled(int16_t width_image, int16_t height_image, uint8_t scale, uint16_t *pixels);

    void startWrite();
    void endWrite();
    void startBuffering();
    void endBuffering();

    uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
      return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
    }

  protected:
    uint16_t getRawPixel(int16_t x, int16_t y);
    void drawFastRawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawFastRawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    uint16_t *buffer = nullptr; ///< Raster data: no longer private, allow subclass access

  private:
    Display* display;
    uint32_t sizeof_framebuffer;
    //bool need_refresh = false;
    bool buffering = false;
    uint32_t last_refresh = 0;

};

#endif // LATER
#endif //__ARDUINO_GIGADISPLAY_GFX__