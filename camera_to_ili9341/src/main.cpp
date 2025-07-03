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
#define ILI9341_USE_FIXED_BUFFER


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
#include <zephyr/devicetree.h>
#include <zephyr/multi_heap/shared_multi_heap.h>


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


// map() transforms input "x" from one numerical range to another.  For example, if
extern void WaitForUserInput(uint32_t timeout);
extern void show_all_gpio_regs();
extern void initialize_display();

#ifdef ILI9341_USE_FIXED_BUFFER
uint16_t frame_buffer[CONFIG_VIDEO_WIDTH*CONFIG_VIDEO_HEIGHT];
#endif



int main(void)
{
	struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
	struct video_buffer *vbuf = nullptr;
	const struct device *video_dev;
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	size_t bsize;
	int i = 0;

	int ret;

	// Start up the Serial
	USBSerial.begin();
	//SerialX.begin();
	USBSerial.println("Camera capture...");

	// initialize the display
	initialize_display();

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
	printk("Initialze video buffer list (%u)\n",  ARRAY_SIZE(buffers));
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		printk("  %d:%x\n", i, (uint32_t)buffers[i]);
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

	// show all registers
//	show_all_gpio_regs();

	printk("Starting main loop\n");
    for (;;) {
		int err;
		uint32_t start_time = micros();
		err = video_dequeue(video_dev, &vbuf, K_MSEC(10000));
		printk("Dequeue: %lu\n", micros() - start_time);
		if (err) {
			printk("ERROR: Unable to dequeue video buf\n");
	  		k_sleep(K_MSEC(1000));
	  		continue;
		}
		// We need to byte swap here...
		static int frame_count = 0;
		frame_count++;
#ifdef TIMED_WAIT_NO_TFT
		USBSerial.println(frame_count);
		delay(TIMED_WAIT_NO_TFT);

#elif defined(ILI9341_USE_FIXED_BUFFER)
        uint16_t *pixels = (uint16_t *) vbuf->buffer;
        for (size_t i=0; i < (CONFIG_VIDEO_WIDTH*CONFIG_VIDEO_HEIGHT); i++) {
            frame_buffer[i] = __REVSH(pixels[i]);
        }

		start_time = micros();
		tft.writeRect(0, 0, CONFIG_VIDEO_WIDTH, CONFIG_VIDEO_HEIGHT, frame_buffer);
		printk("writeRect: %d %lu\n", frame_count, micros() - start_time);

#else
        uint16_t *pixels = (uint16_t *) vbuf->buffer;
        for (size_t i=0; i < (CONFIG_VIDEO_WIDTH*CONFIG_VIDEO_HEIGHT); i++) {
            pixels[i] = __REVSH(pixels[i]);
        }

		start_time = micros();
		tft.writeRect(0, 0, CONFIG_VIDEO_WIDTH, CONFIG_VIDEO_HEIGHT, (uint16_t*)vbuf->buffer);
		printk("writeRect: %d %lu\n", frame_count, micros() - start_time);
#endif

		err = video_enqueue(video_dev, vbuf);
		if (err) {
			printk("ERROR: Unable to requeue video buf\n");
			continue;
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
//----------------------------------------------------------------------------------
extern unsigned long millis(void);

unsigned long millis(void) { return k_uptime_get_32(); }

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

