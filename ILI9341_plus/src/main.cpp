/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include "USBSerialDevice.h"

#include "UARTDevice.h"

#include "ILI9341_GIGA_zephyr.h"

#define ATP
//#define PJRC

//static const struct gpio_dt_spec connector_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
//    DT_PATH(zephyr_user), digital_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
//const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));
//const struct device *const ili9341_spi =  DEVICE_DT_GET(DT_CHOSEN(spi_ili9341));
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB )

#if defined(ATP)
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341atp_spi_dev), SPI_OP, 0);
#elif defined(PJRC)
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341prjc_spi_dev), SPI_OP, 0);
#else
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341_spi_dev), SPI_OP, 0);
#endif

inline int min(int a, int b) {
	return (a <= b)? a : b;
}

inline void delay(uint32_t ms) {
	k_sleep(K_MSEC(ms));
}

// Need to do this.
unsigned long micros(void) {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
  return k_cyc_to_us_floor32(k_cycle_get_64());
#else
  return k_cyc_to_us_floor32(k_cycle_get_32());
#endif
 }

#if defined(ATP)
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341atp_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#elif defined(PJRC)
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341pjrc_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#else
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif

ILI9341_GIGA_n tft(&ili9341_spi, &ili9341_pins[0], &ili9341_pins[1], &ili9341_pins[2]);


//=================================================================
// forward references of functions
extern unsigned long testFillScreen();
extern unsigned long testText();
extern unsigned long testLines(uint16_t color);
extern unsigned long testFastLines(uint16_t color1, uint16_t color2);
extern unsigned long testRects(uint16_t color);
extern unsigned long testFilledRects(uint16_t color1, uint16_t color2);
extern unsigned long testFilledCircles(uint8_t radius, uint16_t color);
extern unsigned long testCircles(uint8_t radius, uint16_t color);
extern unsigned long testTriangles();
extern unsigned long testFilledTriangles();
extern unsigned long testRoundRects();
extern unsigned long testFilledRoundRects();
extern void WaitForUserInput();


int main(void)
{
	int ret;



	USBSerial.begin();
#if 0
	USBSerial.printf("cs: %p %u %x\n", ili9341_pins[0].port, ili9341_pins[0].pin, ili9341_pins[0].dt_flags);
	USBSerial.printf("dc: %p %u %x\n", ili9341_pins[1].port, ili9341_pins[1].pin, ili9341_pins[1].dt_flags);
	USBSerial.printf("rst: %p %u %x\n",ili9341_pins[2].port, ili9341_pins[2].pin, ili9341_pins[2].dt_flags);
	if (ARRAY_SIZE(ili9341_pins) > 3) {
		USBSerial.printf("touch cs: %p %u %x\n",ili9341_pins[3].port, ili9341_pins[3].pin, ili9341_pins[3].dt_flags);
  		gpio_pin_configure_dt(&ili9341_pins[3], GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
  		gpio_pin_set_dt(&ili9341_pins[3] , 1);
	}
#endif
	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');
	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');

	tft.setDebugUART(&USBSerial);
	tft.begin();
	tft.setRotation(1);

	tft.fillScreen(ILI9341_BLACK);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_RED);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_GREEN);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_BLUE);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_WHITE);
#if 0
	tft.drawLine(0, tft.height()/2, tft.width()-1, tft.height()/2, ILI9341_RED);

	uint32_t *lpspi4_regs = (uint32_t*)0x403A0000ul;
	USBSerial.printf(" VERID: %08X\n", lpspi4_regs[0x0/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" PARAM: %08X\n", lpspi4_regs[0x4/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CR: %08X\n", lpspi4_regs[0x10/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" SR: %08X\n", lpspi4_regs[0x14/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" IER: %08X\n", lpspi4_regs[0x18/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DER: %08X\n", lpspi4_regs[0x1C/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CFGR0: %08X\n", lpspi4_regs[0x20/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CFGR1: %08X\n", lpspi4_regs[0x24/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DMR0: %08X\n", lpspi4_regs[0x30/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DMR1: %08X\n", lpspi4_regs[0x34/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CCR: %08X\n", lpspi4_regs[0x40/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" FCR: %08X\n", lpspi4_regs[0x58/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" FSR: %08X\n", lpspi4_regs[0x5C/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" TCR: %08X\n", lpspi4_regs[0x60/4]); k_sleep(K_MSEC(100));
//	USBSerial.printf(" TDR: %08X\n", lpspi4_regs[0x64/4]); k_sleep(K_MSEC(100)); // Write only register
	USBSerial.printf(" RSR: %08X\n", lpspi4_regs[0x70/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" RDR: %08X\n", lpspi4_regs[0x74/4]); k_sleep(K_MSEC(100));

#endif
  
	// lets try the graphic tests
  USBSerial.print("Benchmark                Time (microseconds)\n");

  USBSerial.print("Screen fill              ");
  USBSerial.println(testFillScreen());
  WaitForUserInput();
  delay(200);

  USBSerial.print("Text                     ");
  USBSerial.println(testText());
  delay(600);
  WaitForUserInput();

  USBSerial.print("Lines                    ");
  USBSerial.println(testLines(ILI9341_CYAN));
  delay(200);
  

  WaitForUserInput();

  USBSerial.print("Horiz/Vert Lines         ");
  USBSerial.println(testFastLines(ILI9341_RED, ILI9341_BLUE));
  delay(200);
  WaitForUserInput();

  USBSerial.print("Rectangles (outline)     ");
  USBSerial.println(testRects(ILI9341_GREEN));
  delay(200);
  WaitForUserInput();

  USBSerial.print("Rectangles (filled)      ");
  USBSerial.println(testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
  delay(200);
  WaitForUserInput();

  USBSerial.print("Circles (filled)         ");
  USBSerial.println(testFilledCircles(10, ILI9341_MAGENTA));
  WaitForUserInput();

  USBSerial.print("Circles (outline)        ");
  USBSerial.println(testCircles(10, ILI9341_WHITE));
  delay(200);
  WaitForUserInput();

  USBSerial.print("Triangles (outline)      ");
  USBSerial.println(testTriangles());
  delay(200);
  WaitForUserInput();

  USBSerial.print("Triangles (filled)       ");
  USBSerial.println(testFilledTriangles());
  delay(200);
  WaitForUserInput();

  USBSerial.print("Rounded rects (outline)  ");
  USBSerial.println(testRoundRects());
  delay(200);
  WaitForUserInput();

  USBSerial.print("Rounded rects (filled)   ");
  USBSerial.println(testFilledRoundRects());
  WaitForUserInput();



    for (;;) {
	//		gpio_pin_toggle_dt(&led);
  	    for (uint8_t rotation = 0; rotation < 4; rotation++) {
    	    tft.setRotation(rotation);
    	    testText();
    	    delay(1000);
  	    }
	}
	return 0;
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
  tft.fillScreen(ILI9341_GREEN);
  tft.fillScreen(ILI9341_BLUE);
  tft.fillScreen(ILI9341_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
//  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
//  tft.println(1234.56);
//  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
//  tft.println(0xDEADBEEF, HEX);
//  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  tft.fillScreen(ILI9341_BLACK);

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(ILI9341_BLACK);

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  tft.fillScreen(ILI9341_BLACK);

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);

  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  USBSerial.printf("W:%u H:%u\n", tft.width(), tft.height());
  start = micros();
  for (y = 0; y < h; y += 5) {tft.drawFastHLine(0, y, w, color1); USBSerial.printf("H: %d %d %x\n", y, w, color1);}
  for (x = 0; x < w; x += 5) {tft.drawFastVLine(x, 0, h, color2); USBSerial.printf("V: %d %d %x\n", x, h, color2);}

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    tft.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height()) - 1;
  for (i = n; i > 0; i -= 6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx - i2, cy - i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx - i2, cy - i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                      w = tft.width()  + radius,
                      h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (i = min(cx, cy); i > 10; i -= 5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height()) - 1;
  start = micros();
  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (i = min(tft.width(), tft.height()) - 1; i > 20; i -= 6) {
    i2 = i / 2;
    tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
  }

  return micros() - start;
}

void WaitForUserInput() {
    USBSerial.print("Hit key to continue\n");
    while (USBSerial.read() == -1) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}
