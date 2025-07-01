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
#include <zephyr/drivers/clock_control.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include "USBSerialDevice.h"

#include "UARTDevice.h"
#include "ILI9341_GIGA_zephyr.h"

#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>


#if !defined(CONFIG_BOARD_ARDUINO_PORTENTA_H7)
#define ATP
//#define PJRC
#endif

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


int camera_ext_clock_enable(void)
{
	int ret;
	uint32_t rate;

	const struct device *cam_ext_clk_dev = DEVICE_DT_GET(DT_NODELABEL(pwmclock));

	if (!device_is_ready(cam_ext_clk_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cam_ext_clk_dev, (clock_control_subsys_t)0);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(cam_ext_clk_dev, (clock_control_subsys_t)0, &rate);
	if (ret < 0) {
		return ret;
	}

	return 0;
}




int main(void)
{
	struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
	struct video_buffer *vbuf = nullptr;
	const struct device *display_dev;
	const struct device *video_dev;
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	size_t bsize;
	int i = 0;

	int ret;

	USBSerial.begin();
	//SerialX.begin();
	USBSerial.println("Camera capture...");
	tft.setDebugUART(&USBSerial);
	tft.begin();
	tft.setRotation(0);

	tft.fillScreen(ILI9341_BLACK);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_RED);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_GREEN);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_BLUE);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_WHITE);

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

	// lets try to start the camera clock
	printk("Starting camera clock\n");
	ret = camera_ext_clock_enable();
	if (ret) {
		printk("    Failed: %d\n", ret);
	}

	// lets get camera information
	video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (!device_is_ready(video_dev)) {
		printk("ERROR: %s device is not ready\n",  video_dev->name);
		return 0;
	}

	printk("INFO: - Device name: %s\n",  video_dev->name);

	/* Get capabilities */
	caps.type = type;
	if (video_get_caps(video_dev, &caps)) {
		printk("ERROR: Unable to retrieve video capabilities\n") ;
		return 0;
	}

	printk("INFO: - Capabilities:\n") ;
	while (caps.format_caps[i].pixelformat) {
		const struct video_format_cap *fcap = &caps.format_caps[i];
		/* four %c to string */
		printk("INFO:   %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n", 
			(char)fcap->pixelformat, (char)(fcap->pixelformat >> 8),
			(char)(fcap->pixelformat >> 16), (char)(fcap->pixelformat >> 24),
			fcap->width_min, fcap->width_max, fcap->width_step, fcap->height_min,
			fcap->height_max, fcap->height_step);
		i++;
	}

	/* Get default/native format */
	fmt.type = type;
	if (video_get_format(video_dev, &fmt)) {
		printk("ERROR: Unable to retrieve video format\n") ;
		return 0;
	}

	/* Set format */
	fmt.width = CONFIG_VIDEO_WIDTH;
	fmt.height = CONFIG_VIDEO_HEIGHT;
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;

	if (video_set_format(video_dev, &fmt)) {
		printk("ERROR: Unable to set up video format\n") ;
		return 0;
	}

	printk("INFO: - Format: %c%c%c%c %ux%u %u\n",   (char)fmt.pixelformat, (char)(fmt.pixelformat >> 8),
		(char)(fmt.pixelformat >> 16), (char)(fmt.pixelformat >> 24), fmt.width, fmt.height,
		fmt.pitch);

	if (caps.min_line_count != LINE_COUNT_HEIGHT) {
		printk("ERROR: Partial framebuffers not supported by this sample\n") ;
		return 0;
	}
	/* Size to allocate for each buffer */
	bsize = fmt.pitch * fmt.height;

	/* Alloc video buffers and enqueue for capture */
	printk("Initialze video buffer list\n");
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		if (buffers[i] == NULL) {
			printk("ERROR: Unable to alloc video buffer\n") ;
			return 0;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}
	/* Set controls */

	struct video_control ctrl = {.id = VIDEO_CID_HFLIP, .val = 1};

	if (IS_ENABLED(CONFIG_VIDEO_HFLIP)) {
		video_set_ctrl(video_dev, &ctrl);
	}

	if (IS_ENABLED(CONFIG_VIDEO_VFLIP)) {
		ctrl.id = VIDEO_CID_VFLIP;
		video_set_ctrl(video_dev, &ctrl);
	}

	/* Start video capture */
	printk("Starting  capture\n");
	if (video_stream_start(video_dev, type)) {
		printk("ERROR: Unable to start capture (interface)\n") ;
		return 0;
	}

	printk("Starting main loop\n");
    for (;;) {
  		k_sleep(K_MSEC(100));
  	}
	return 0;
}

extern unsigned long millis(void);

unsigned long millis(void) { return k_uptime_get_32(); }

void WaitForUserInput(uint32_t timeout) {
    USBSerial.print("Hit key to continue\n");
    uint32_t time_start = millis();
    while ((USBSerial.read() == -1) && ((uint32_t)(millis() - time_start)  < timeout)) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}
