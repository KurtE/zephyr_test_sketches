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
// Turn on this option, to test to ee if camera errors out without screeen
//#define TIMED_WAIT_NO_TFT (1000/6)

// Try using fixed normal memory buffer for display
//#define ILI9341_USE_FIXED_BUFFER
// Hack try to use fixed buffer for camera
//#define CAMERA_USE_FIXED_BUFFER
#define TRY_CAPTURE_SNAPSHOT
//#define TRY_WRITERECTCB
#define USE_ST7796

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

#include "ST77XX_zephyr.h"

#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/devicetree.h>
#include <zephyr/multi_heap/shared_multi_heap.h>


#if !defined(CONFIG_BOARD_ARDUINO_PORTENTA_H7)
#define ATP
//#define PJRC
#endif

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
//const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));
//const struct device *const generic_tft_spi =  DEVICE_DT_GET(DT_CHOSEN(spi_ili9341));
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB )

#if defined(ATP)
static struct spi_dt_spec generic_tft_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341atp_spi_dev), SPI_OP, 0);
#elif defined(PJRC)
static struct spi_dt_spec generic_tft_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341prjc_spi_dev), SPI_OP, 0);
#else
static struct spi_dt_spec generic_tft_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(generic_tft_spi_dev), SPI_OP, 0);
#endif



inline int min(int a, int b) {
	return (a <= b)? a : b;
}

inline void delay(uint32_t ms) {
	k_sleep(K_MSEC(ms));
}

extern unsigned long millis(void);

unsigned long millis(void) { return k_uptime_get_32(); }

// Need to do this.
unsigned long micros(void) {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
  return k_cyc_to_us_floor32(k_cycle_get_64());
#else
  return k_cyc_to_us_floor32(k_cycle_get_32());
#endif
 }

#if defined(ATP)
static const struct gpio_dt_spec generic_tft_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341atp_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#elif defined(PJRC)
static const struct gpio_dt_spec generic_tft_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341pjrc_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#else
static const struct gpio_dt_spec generic_tft_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), generic_tft_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif

#ifdef USE_ST7796
ST7796_zephyr tft(&generic_tft_spi, &generic_tft_pins[0], &generic_tft_pins[1], &generic_tft_pins[2]);
#else
ILI9341_GIGA_n tft(&generic_tft_spi, &generic_tft_pins[0], &generic_tft_pins[1], &generic_tft_pins[2]);
#endif

// Camera/Video global variables.
struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
struct video_buffer *vbuf = nullptr;
const struct device *video_dev;


// map() transforms input "x" from one numerical range to another.  For example, if
extern void WaitForUserInput(uint32_t timeout);
extern void show_all_gpio_regs();
extern void initialize_display();
extern int initialize_video();
extern void	draw_scaled_up_image(uint16_t *pixels);

#if (CONFIG_VIDEO_SOURCE_CROP_WIDTH > 0) && (CONFIG_VIDEO_SOURCE_CROP_WIDTH <= CONFIG_VIDEO_FRAME_WIDTH) && (CONFIG_VIDEO_SOURCE_CROP_HEIGHT > 0) && (CONFIG_VIDEO_SOURCE_CROP_HEIGHT <= CONFIG_VIDEO_FRAME_HEIGHT)
#define CAMERA_IMAGE_WIDTH (CONFIG_VIDEO_SOURCE_CROP_WIDTH)
#define CAMERA_IMAGE_HEIGHT (CONFIG_VIDEO_SOURCE_CROP_HEIGHT)
#else
#define CAMERA_IMAGE_WIDTH (CONFIG_VIDEO_FRAME_WIDTH)
#define CAMERA_IMAGE_HEIGHT (CONFIG_VIDEO_FRAME_HEIGHT)
#endif

#if defined(ILI9341_USE_FIXED_BUFFER) || defined(CAMERA_USE_FIXED_BUFFER)
uint16_t frame_buffer[CAMERA_IMAGE_WIDTH*CONFIG_VIDEO_FRAME_HEIGHT];
#endif

//#if defined(TRY_CAPTURE_SNAPSHOT)
//#if (CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT) <= (320*240)
//uint16_t frame_buffer[CAMERA_IMAGE_WIDTH*CONFIG_VIDEO_FRAME_HEIGHT];
//#else
//uint16_t *frame_buffer = nullptr;
//#endif
//#endif
#ifdef TRY_WRITERECTCB
volatile bool write_rect_active = false;
void writerectCallback(int result) {
	write_rect_active = false;
}
#endif

struct video_selection vselPan = {VIDEO_BUF_TYPE_OUTPUT, VIDEO_SEL_TGT_CROP};

struct video_selection vselNativeSize = {VIDEO_BUF_TYPE_OUTPUT, VIDEO_SEL_TGT_NATIVE_SIZE};


////////////////////////////////
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


////////////////////////////
void blink_led(uint32_t sleep_time, uint32_t count) {
	int ret;
	bool led_state = true;

	static bool led_initialized = false;

	if (!led_initialized) {
		if (!gpio_is_ready_dt(&led)) {
			return;
		}

		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return;
		}
		led_initialized = true;
	}
	while (count) {
		gpio_pin_toggle_dt(&led);
		led_state = !led_state;
		k_msleep(sleep_time);
		gpio_pin_toggle_dt(&led);
		led_state = !led_state;
		k_msleep(sleep_time);
		count--;
	}
}


int main(void)
{

	
	blink_led(500, 1);

	// Start up the Serial
	USBSerial.begin();
	//SerialX.begin();
	USBSerial.println("Camera capture...");

	// initialize the display
	initialize_display();

	initialize_video();

	// show all registers
//	show_all_gpio_regs();
	int frame_count = 0;
	uint32_t read_frame_sum = 0;
	uint32_t write_rect_sum = 0;
	int sum_count = 0;
	bool scale_up = false;
	bool panning_enabled = false;

	int err;

	if ((err = video_get_selection(video_dev, &vselPan)) == 0) {
		USBSerial.printf("Crop Rect:(%u %u) %ux%u\n", vselPan.rect.left, vselPan.rect.top,
			vselPan.rect.width, vselPan.rect.height);
	}

	if ((err = video_get_selection(video_dev, &vselNativeSize)) == 0) {
		USBSerial.printf("Native size Rect:(%u %u) %ux%u\n", vselNativeSize.rect.left, vselNativeSize.rect.top,
			vselNativeSize.rect.width, vselNativeSize.rect.height);

	}

	int pan_delta_x = (vselNativeSize.rect.width - vselPan.rect.width) / 4;

	//======================================================================
	// main loop
	//======================================================================

	printk("Starting main loop\n");
    for (;;) {
		if (USBSerial.available()) {
			while (USBSerial.read() != -1) {}
			USBSerial.println("*** Paused ***");
			int ch;
			while ((ch = USBSerial.read()) == -1) {}
			//if (ch == 'r') {
			//	print_camera_registers();
			//}
			if (ch == 's') {
				scale_up = !scale_up;
			}
			if (ch == 'p') {
				panning_enabled = !panning_enabled;
			}
			while (USBSerial.read() != -1) {}
			continue;
		}

		frame_count++;
		uint32_t start_time = micros();
//		#if defined(TRY_CAPTURE_SNAPSHOT)
//		err = video_capture_snapshot(video_dev, frame_buffer, CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT * 2, K_MSEC(250));
//		static uint8_t dbg_count = 5;
//		uint32_t delta_time = micros() - start_time;
//		if (dbg_count || err || (delta_time > 200000)) {
//			printk("video_capture_snapshot: %d %u %u\n", err, frame_count, micros() - start_time);
//			if (dbg_count) dbg_count--;
//			else dbg_count = 3; // show that we received a few...
//		}
//		if (err) continue;
//		#else
		err = video_dequeue(video_dev, &vbuf, K_MSEC(10000));
		read_frame_sum += (micros() - start_time);
		//printk("Dequeue: %lu\n", micros() - start_time);
		if (err) {
			printk("ERROR: Unable to dequeue video buf\n");
	  		k_sleep(K_MSEC(1000));
	  		continue;
		}
//		#endif
		// We need to byte swap here...
#ifdef TIMED_WAIT_NO_TFT
		USBSerial.println(frame_count);
		delay(TIMED_WAIT_NO_TFT);

#elif defined(ILI9341_USE_FIXED_BUFFER)
        uint16_t *pixels = (uint16_t *) vbuf->buffer;
        for (size_t i=0; i < (CAMERA_IMAGE_WIDTH*CONFIG_VIDEO_FRAME_HEIGHT); i++) {
            frame_buffer[i] = __REVSH(pixels[i]);
        }

		start_time = micros();
		tft.writeRect(0, 0, CAMERA_IMAGE_WIDTH, CONFIG_VIDEO_FRAME_HEIGHT, frame_buffer);
		printk("writeRect: %d %lu\n", frame_count, micros() - start_time);

#else
//#if defined(TRY_CAPTURE_SNAPSHOT)
//		uint16_t *pixels = frame_buffer;
//#else
        uint16_t *pixels = (uint16_t *) vbuf->buffer;
//#endif
        for (size_t i=0; i < (CAMERA_IMAGE_WIDTH*CONFIG_VIDEO_FRAME_HEIGHT); i++) {
            pixels[i] = __REVSH(pixels[i]);
        }

		start_time = micros();
		if (scale_up) {
			draw_scaled_up_image(pixels);
		} else {
			#ifdef TRY_WRITERECTCB
			write_rect_active = true;
			tft.writeRectCB(0, 0, CAMERA_IMAGE_WIDTH, CONFIG_VIDEO_FRAME_HEIGHT, (uint16_t*)pixels, &writerectCallback);
			while (write_rect_active) k_sleep(K_USEC(59));
			#else
			tft.writeRect(0, 0, CAMERA_IMAGE_WIDTH, CONFIG_VIDEO_FRAME_HEIGHT, (uint16_t*)pixels);
			#endif
		}
		write_rect_sum += micros() - start_time;
		//printk("writeRect: %d %lu\n", frame_count, micros() - start_time);
#endif

		//#if !defined(TRY_CAPTURE_SNAPSHOT)
		err = video_enqueue(video_dev, vbuf);
		if (err) {
			printk("ERROR: Unable to requeue video buf\n");
			continue;
		}
		//#endif
		sum_count++;
		if (sum_count == 10) {
			USBSerial.printf("%d %u RD: %u %u WR: %u %u\n", frame_count, sum_count, read_frame_sum, read_frame_sum / sum_count, write_rect_sum, write_rect_sum / sum_count);
			sum_count = 0;
			read_frame_sum = 0;
			write_rect_sum = 0;
		}
		if (panning_enabled) {
			if ((vselPan.rect.left == 0) && (pan_delta_x < 0)) pan_delta_x = -pan_delta_x;
			if ((vselPan.rect.left + vselPan.rect.width) == vselNativeSize.rect.width)  pan_delta_x = -pan_delta_x;
			vselPan.rect.left += pan_delta_x;
			if ((err = video_set_selection(video_dev, &vselPan)) != 0) {
				USBSerial.printf("Eror setting Crop Rect:(%u %u) %ux%u\n", vselPan.rect.left, vselPan.rect.top,
					vselPan.rect.width, vselPan.rect.height);
			}
		}
  	}
	return 0;
}

//----------------------------------------------------------------------------------
// iniitialize the display and do some graphic outputs to just to make sure it
// is working
//----------------------------------------------------------------------------------
void initialize_display() {
	//tft.setDebugUART(&USBSerial);
#ifdef USE_ST7796
  tft.begin();
  tft.setRotation(1);
#else
	tft.begin();
	tft.setRotation(1);
#endif

	tft.fillScreen(ILI9341_BLACK);
	//k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_RED);
	//k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_GREEN);
	//k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_BLUE);
	//k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_WHITE);

//	WaitForUserInput(1000);
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
	k_sleep(K_MSEC(500));
//	WaitForUserInput(2000);


}

//----------------------------------------------------------------------------------
// iniitialize the camera/video
//----------------------------------------------------------------------------------
int initialize_video() {
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	size_t bsize;
	int i = 0;
	int ret = 0;

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
	printk("video_get_format returned: %u %u %x\n", fmt.width, fmt.height, fmt.pixelformat);
	/* Set format */
	fmt.width = CONFIG_VIDEO_FRAME_WIDTH;
	fmt.height = CONFIG_VIDEO_FRAME_HEIGHT;
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	printk("updated to: %u %u %x\n", fmt.width, fmt.height, fmt.pixelformat);

	if (video_set_format(video_dev, &fmt)) {
		printk("ERROR: Unable to set up video format\n") ;
		return 0;
	}

	#if (CONFIG_VIDEO_SOURCE_CROP_WIDTH > 0) && (CONFIG_VIDEO_SOURCE_CROP_HEIGHT > 0)
	printk("Set Video Selection CROP %d %d:\n", CONFIG_VIDEO_SOURCE_CROP_WIDTH, CONFIG_VIDEO_SOURCE_CROP_HEIGHT);
	struct video_selection vselCrop;
	vselCrop.type = VIDEO_BUF_TYPE_OUTPUT;
	vselCrop.target = VIDEO_SEL_TGT_CROP;
	vselCrop.rect.left = CONFIG_VIDEO_SOURCE_CROP_LEFT;
	vselCrop.rect.top = CONFIG_VIDEO_SOURCE_CROP_TOP;
	vselCrop.rect.width = CONFIG_VIDEO_SOURCE_CROP_WIDTH;
	vselCrop.rect.height = CONFIG_VIDEO_SOURCE_CROP_HEIGHT;

	if ((ret = video_set_selection(video_dev, &vselCrop))!= 0) {
		printk("ERROR: %d\n", ret) ;
		return 0;
	}


	#endif


	printk("INFO: - Format: %c%c%c%c %ux%u %u\n",   (char)fmt.pixelformat, (char)(fmt.pixelformat >> 8),
		(char)(fmt.pixelformat >> 16), (char)(fmt.pixelformat >> 24), fmt.width, fmt.height,
		fmt.pitch);

	if (caps.min_line_count != LINE_COUNT_HEIGHT) {
		printk("ERROR: Partial framebuffers not supported by this sample\n") ;
		return 0;
	}
	/* Size to allocate for each buffer */
	/* lets ask for the actual current format */
	if ((ret = video_get_format(video_dev, &fmt)) != 0) {
		printk("call to video_get_format failed: %d\n", ret);
	}
	printk("After call video_get_format: - Format: %c%c%c%c %ux%u %u\n",   (char)fmt.pixelformat, (char)(fmt.pixelformat >> 8),
		(char)(fmt.pixelformat >> 16), (char)(fmt.pixelformat >> 24), fmt.width, fmt.height,
		fmt.pitch);


	bsize = fmt.pitch * fmt.height;

	/* Alloc video buffers and enqueue for capture */
	printk("Initialze video buffer list (%u)\n",  ARRAY_SIZE(buffers));
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		#ifdef CAMERA_USE_FIXED_BUFFER
		if ((i == 0) &&  (buffers[i] != NULL)) {
			// REAL hack change the buffer over to our fixed buffer...
			buffers[i]->buffer = (uint8_t *)frame_buffer;
		}
		#endif

		printk("  %d:%x\n", i, (uint32_t)buffers[i]);
		if (buffers[i] == NULL) {
			printk("ERROR: Unable to alloc video buffer\n") ;
			return 0;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}

//	#if defined(TRY_CAPTURE_SNAPSHOT) && (CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT) > (320*240)
//	frame_buffer = (uint16_t*)buffers[ARRAY_SIZE(buffers) - 1]->buffer;
//	#endif

	/* Set controls */
	struct video_control ctrl = {.id = VIDEO_CID_HFLIP, .val = 1};

	if (IS_ENABLED(CONFIG_VIDEO_HFLIP)) {
		video_set_ctrl(video_dev, &ctrl);
	}

	if (IS_ENABLED(CONFIG_VIDEO_VFLIP)) {
		ctrl.id = VIDEO_CID_VFLIP;
		video_set_ctrl(video_dev, &ctrl);
	}

//#ifdef TRY_CAPTURE_SNAPSHOT
//	video_set_snapshot_mode(video_dev, true);
//#endif
	/* Start video capture */
	printk("Starting  capture\n");
	if (video_stream_start(video_dev, type)) {
		printk("ERROR: Unable to start capture (interface)\n") ;
		return 0;
	}

	// Lets see if we can call video_get_selection
#if 1
	printk("Get Video Selection:\n");
	struct video_selection vsel;
	vsel.type = VIDEO_BUF_TYPE_OUTPUT;
	vsel.target = VIDEO_SEL_TGT_NATIVE_SIZE;
	if ((ret = video_get_selection(video_dev, &vsel))!= 0) {
		printk("ERROR: %d\n", ret) ;
		return 0;
	} else {
		printk("\tTy:%d Tar:%d R:%u %u %u %u\n", vsel.type, vsel.target,
			vsel.rect.left, vsel.rect.top, vsel.rect.width, vsel.rect.height);
	}
#endif

	return 1;

}
uint16_t rows_buffer[480*3];
void draw_scaled_up_image(uint16_t *pixels) {
	// quick and dirty 1.5 scale up
	uint16_t *pixels_row = pixels;
	uint16_t tft_width = tft.width();
	uint16_t tft_height = tft.height();
	uint8_t write_height = 3;
	for (int row = 0; row < tft_height; row += write_height) {
		uint16_t *pixel = pixels_row;
		if ((row + write_height) >= tft_height) {
			write_height = (tft_height - row);
		}
		for (int col = 0; col < tft_width; col += 3) {
			rows_buffer[col] = pixel[0];
			rows_buffer[col+1] = pixel[0];
			rows_buffer[col+2] = pixel[1];
			rows_buffer[tft_width + col] = pixel[0];
			rows_buffer[tft_width + col + 1] = pixel[CAMERA_IMAGE_WIDTH];
			rows_buffer[tft_width + col + 2] = pixel[1];

			rows_buffer[2 * tft_width + col] = pixel[CAMERA_IMAGE_WIDTH];
			rows_buffer[2 * tft_width + col + 1] = pixel[CAMERA_IMAGE_WIDTH];
			rows_buffer[2 * tft_width + col + 2] = pixel[CAMERA_IMAGE_WIDTH + 1];
			pixel += 2;
		}
		tft.writeRect(0, row, tft_width, write_height, rows_buffer);
		pixels_row += 2 * CAMERA_IMAGE_WIDTH;
	}


}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

void WaitForUserInput(uint32_t timeout) {
    USBSerial.print("Hit key to continue\n");
    uint32_t time_start = millis();
    while ((USBSerial.read() == -1) && ((uint32_t)(millis() - time_start)  < timeout)) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}

void print_gpio_regs(const char *name, GPIO_TypeDef *port) {
  //printk("GPIO%s(%p) %08X %08X %08x\n", name, port, port->MODER, port->AFR[0], port->AFR[1]);
  USBSerial.print("GPIO");
  USBSerial.print(name);
  USBSerial.print(" ");
  USBSerial.print(port->MODER, HEX);
  USBSerial.print(" ");
  USBSerial.print(port->AFR[0], HEX);
  USBSerial.print(" ");
  USBSerial.println(port->AFR[1], HEX);
}

void print_PinConfig(const char *name, GPIO_TypeDef *port, const char *regName) {
  uint32_t reg = 0;
  uint8_t numPins = 0;
  uint8_t numBits = 0;
  uint8_t hack = 0;
  USBSerial.print("GPIO");
  USBSerial.print(name);
  USBSerial.print(" ");
  if(strcmp(regName, "M") == 0) {
    USBSerial.print("MODER: ");
    numPins = 16;
    numBits = 2;
    hack = 0;
    reg = port->MODER;
  } else if(strcmp (regName , "AL") == 0) {
    USBSerial.print("AFRL");
    numPins = 8;
    numBits = 4;
    hack = 0;
    reg = port->AFR[0];
  } else {
    USBSerial.print("AFRH");
    numPins = 8;
    numBits = 4;
    hack = 1;
    reg = port->AFR[1];
  }


  for(uint8_t i = 0; i < numPins; i++) {
    unsigned  mask;
    //mask = ((1 << numBits2Extract) << startBit)
    mask = ((1 << numBits) - 1) << (i*numBits);
    //extractedBits = (value & mask) >> startBit
    uint8_t extractedBits = (reg & mask) >> (i*numBits);
    USBSerial.print("("); USBSerial.print(i+(hack*8)); USBSerial.print(")");
    USBSerial.print(extractedBits); USBSerial.print(", ");
  }
  USBSerial.println();
}

void show_all_gpio_regs() {
  print_gpio_regs("A", (GPIO_TypeDef *)GPIOA_BASE);
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "M");
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "AL");
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "AH");

  print_gpio_regs("B", (GPIO_TypeDef *)GPIOB_BASE);
  print_gpio_regs("C", (GPIO_TypeDef *)GPIOC_BASE);
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "M");
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "AL");
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "AH");

  print_gpio_regs("D", (GPIO_TypeDef *)GPIOD_BASE);
  print_gpio_regs("E", (GPIO_TypeDef *)GPIOE_BASE);
  print_gpio_regs("F", (GPIO_TypeDef *)GPIOF_BASE);
  print_gpio_regs("G", (GPIO_TypeDef *)GPIOG_BASE);
  print_gpio_regs("H", (GPIO_TypeDef *)GPIOH_BASE);
  print_gpio_regs("I", (GPIO_TypeDef *)GPIOI_BASE);
  print_gpio_regs("J", (GPIO_TypeDef *)GPIOJ_BASE);
  print_gpio_regs("K", (GPIO_TypeDef *)GPIOK_BASE);
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
extern "C" {

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

SYS_INIT(camera_ext_clock_enable, POST_KERNEL, CONFIG_CLOCK_CONTROL_PWM_INIT_PRIORITY);

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

__stm32_sdram1_section static uint8_t __aligned(32) smh_pool[4*1024*1024];

int smh_init(void) {
    int ret = 0;
    ret = shared_multi_heap_pool_init();
    if (ret != 0) {
        return ret;
    }

    struct shared_multi_heap_region smh_sdram = {
        .attr = SMH_REG_ATTR_EXTERNAL,
        .addr = (uintptr_t) smh_pool,
        .size = sizeof(smh_pool)
    };

    ret = shared_multi_heap_add(&smh_sdram, NULL);
    if (ret != 0) {
        return ret;
    }
	return 0;
}

SYS_INIT(smh_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

} // extern c

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

