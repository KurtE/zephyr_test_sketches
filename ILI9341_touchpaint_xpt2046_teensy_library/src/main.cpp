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

//#include <sample_usbd.h>

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
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include "USBSerialDevice.h"

#include "UARTDevice.h"
#include "ILI9341_GIGA_zephyr.h"

#include "XPT2046_Touchscreen.h"

#define ATP
//#define PJRC

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

static struct spi_dt_spec xpt2046_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(xpt2046_spi_dev), SPI_OP, 0);




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

static const struct gpio_dt_spec xpt2046_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), xpt2046_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

XPT2046_Touchscreen ts(&xpt2046_spi, &xpt2046_pins[0],  &xpt2046_pins[1]);


//=================================================================
// forward references of functions
#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;

#define TS_MINX 337
#define TS_MINY 529
#define TS_MAXX 3729
#define TS_MAXY 3711

// from teensy cores (map)
#include <type_traits>

// map() transforms input "x" from one numerical range to another.  For example, if
// you have analogInput() from 0 to 1023 and you want 5 to 25, use
// map(x, 0, 1023, 5, 25).  When "x" is an integer, the math is performed using
// integers and an integer number is returned.  When "x" is a floating point number,
// math is performed and result returned as floating point, to allow for fine grain
// mapping.
template <class T, class A, class B, class C, class D>
long map(T _x, A _in_min, B _in_max, C _out_min, D _out_max, typename std::enable_if<std::is_integral<T>::value >::type* = 0)
{
	// when the input number is an integer type, do all math as 32 bit signed long
	long x = _x, in_min = _in_min, in_max = _in_max, out_min = _out_min, out_max = _out_max;
	// Arduino's traditional algorithm
#if 0
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
#endif
#if 0
	// st42's suggestion: https://github.com/arduino/Arduino/issues/2466#issuecomment-69873889
	if ((in_max - in_min) > (out_max - out_min)) {
		return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
	} else {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
#endif
	// first compute the ranges and check if input doesn't matter
	long in_range = in_max - in_min;
	long out_range = out_max - out_min;
	if (in_range == 0) return out_min + out_range / 2;
	// compute the numerator
	long num = (x - in_min) * out_range;
	// before dividing, add extra for proper round off (towards zero)
	if (out_range >= 0) {
		num += in_range / 2;
	} else {
		num -= in_range / 2;
	}
	// divide by input range and add output offset to complete map() compute
	long result = num / in_range + out_min;
	// fix "a strange behaviour with negative numbers" (see ArduinoCore-API issue #51)
	//   this step can be deleted if you don't care about non-linear output
	//   behavior extrapolating slightly beyond the mapped input & output range
	if (out_range >= 0) {
		if (in_range * num < 0) return result - 1;
	} else {
		if (in_range * num >= 0) return result + 1;
	}
	return result;
	// more conversation:
	// https://forum.pjrc.com/threads/44503-map()-function-improvements
}
// map() transforms input "x" from one numerical range to another.  For example, if
extern void WaitForUserInput(uint32_t timeout);



int main(void)
{
	int ret;

	USBSerial.begin();
	//SerialX.begin();
	USBSerial.println("Touch Paint started");

//	USBSerial.print("Print connector pins\n");
//  	for (size_t i = 0; i < ARRAY_SIZE(connector_pins); i++) {
//  		USBSerial.printf("%u %p %u\n", i, connector_pins[i].port, connector_pins[i].pin);
//  	}
  	// Main loop, would be nice if we setup events for the two RX queues, but startof KISS
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
  // initialize semaphore... used for touch...
  //k_sem_init(&sync, 0, 1);


	tft.setDebugUART(&USBSerial);
	tft.begin();
	tft.setRotation(0);

	ts.begin();

	tft.fillScreen(ILI9341_BLACK);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_RED);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_GREEN);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_BLUE);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_WHITE);

	uint32_t radius = min(tft.width(), tft.height()) / 2;

	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_RED);
	radius -= 20;
	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_YELLOW);
	radius -= 20;
	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_GREEN);
	radius -= 20;
	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_CYAN);
	radius -= 20;
	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_BLUE);
	radius -= 20;
	tft.fillCircle(tft.width()/2, tft.height()/2, radius, ILI9341_MAGENTA);
	tft.fillCircle(tft.width()/2, tft.height()/2, 3, ILI9341_WHITE);

	WaitForUserInput(1000);
	tft.fillScreen(ILI9341_BLACK);
	uint32_t t0 = micros();
	tft.fillRectHGradient(10, 10, 100, 100, tft.color565(0 << 3, 0, 0), tft.color565(9 << 3, 0, 0));
	uint32_t t1 = micros();
	tft.fillRectHGradient(130, 10, 100, 100, ILI9341_YELLOW, ILI9341_GREEN);
	uint32_t t2 = micros();
	tft.fillRectVGradient(10, 130, 100, 100, tft.color565(0, 0, 0 << 3), tft.color565(0, 0, 9 << 3));
	uint32_t t3 = micros();
	tft.fillRectVGradient(130, 130, 100, 100, ILI9341_CYAN, ILI9341_BLUE);
	uint32_t t4 = micros();
	USBSerial.printf("%u: %u %u %u %u\n", t4-t0, t1-t0, t2-t1, t3-t2, t4-t3);
	WaitForUserInput(2000);


  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
  tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
  tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
  tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
  tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);


    for (;;) {
    	  // See if there's any  touch data for us
	  	if (ts.bufferEmpty()) {
	  		k_sleep(K_MSEC(5));
	    	continue;
	  	}

	  	// Retrieve a point
	  	TS_Point p = ts.getPoint();

	  	// p is in ILI9341_t3 setOrientation 1 settings. so we need to map x and y differently.
      	if (ts.touched()) {
      		USBSerial.printf("TOUCH (%d, %d, %d)", p.x, p.y, p.z);      
	        // Scale from ~0->4000 to tft.width using the calibration #'s
			#ifdef SCREEN_ORIENTATION_1
			  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
			  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
			#else
			  
			  uint16_t px = map(p.y, TS_MAXY, TS_MINY, 0, tft.width());
			  p.y = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
			  p.x = px;
			#endif
			  USBSerial.printf(" (%d %d)\n", p.x, p.y );
			  if (p.y < BOXSIZE) {
			    oldcolor = currentcolor;

			    if (p.x < BOXSIZE) {
			      currentcolor = ILI9341_RED;
			      tft.drawRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    } else if (p.x < BOXSIZE * 2) {
			      currentcolor = ILI9341_YELLOW;
			      tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    } else if (p.x < BOXSIZE * 3) {
			      currentcolor = ILI9341_GREEN;
			      tft.drawRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    } else if (p.x < BOXSIZE * 4) {
			      currentcolor = ILI9341_CYAN;
			      tft.drawRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    } else if (p.x < BOXSIZE * 5) {
			      currentcolor = ILI9341_BLUE;
			      tft.drawRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    } else if (p.x < BOXSIZE * 6) {
			      currentcolor = ILI9341_MAGENTA;
			      tft.drawRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
			    }
			   if (oldcolor != currentcolor) {
			      if (oldcolor == ILI9341_RED)
			        tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
			      if (oldcolor == ILI9341_YELLOW)
			        tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
			      if (oldcolor == ILI9341_GREEN)
			        tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
			      if (oldcolor == ILI9341_CYAN)
			        tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
			      if (oldcolor == ILI9341_BLUE)
			        tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
			      if (oldcolor == ILI9341_MAGENTA)
			        tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);
			    }
			  }
			   if (((p.y - PENRADIUS) > BOXSIZE) && ((p.y + PENRADIUS) < tft.height())) {
			    tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
			  }

	  		}
	  	}
	return 0;
}

extern unsigned long millis(void);

void WaitForUserInput(uint32_t timeout) {
    USBSerial.print("Hit key to continue\n");
    uint32_t time_start = millis();
    while ((USBSerial.read() == -1) && ((uint32_t)(millis() - time_start)  < timeout)) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}
