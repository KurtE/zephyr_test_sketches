/*
 * Copyright (c) 2019 Linaro Limited
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "usb_serial.h"
#define GRAY_IMAGE

#ifdef CONFIG_TEST
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if !DT_HAS_CHOSEN(zephyr_camera)
#error No camera chosen in devicetree. Missing "--shield" or "--snippet video-sw-generator" flag?
#endif

extern void maybe_send_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height);
extern void main_loop_continuous(const struct device *video_dev, struct video_format *fmt);
extern void main_loop_snapshot(const struct device *video_dev, struct video_format *fmt);
extern void send_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height);
extern void send_raw_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height);
extern void show_all_gpio_regs();

#define F(x) x

static const struct gpio_dt_spec pin_name_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
	DT_PATH(zephyr_user), all_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};


typedef enum { 
	PA_0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7, PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15,
	PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7, PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15,
	PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15,
	PD_0, PD_1, PD_2, PD_3, PD_4, PD_5, PD_6, PD_7, PD_8, PD_9, PD_10, PD_11, PD_12, PD_13, PD_14, PD_15,
	PE_0, PE_1, PE_2, PE_3, PE_4, PE_5, PE_6, PE_7, PE_8, PE_9, PE_10, PE_11, PE_12, PE_13, PE_14, PE_15,
	PF_0, PF_1, PF_2, PF_3, PF_4, PF_5, PF_6, PF_7, PF_8, PF_9, PF_10, PF_11, PF_12, PF_13, PF_14, PF_15,
	PG_0, PG_1, PG_2, PG_3, PG_4, PG_5, PG_6, PG_7, PG_8, PG_9, PG_10, PG_11, PG_12, PG_13, PG_14, PG_15,
	PH_0, PH_1, PH_2, PH_3, PH_4, PH_5, PH_6, PH_7, PH_8, PH_9, PH_10, PH_11, PH_12, PH_13, PH_14, PH_15,
	PI_0, PI_1, PI_2, PI_3, PI_4, PI_5, PI_6, PI_7, PI_8, PI_9, PI_10, PI_11, PI_12, PI_13, PI_14, PI_15,
	PJ_0, PJ_1, PJ_2, PJ_3, PJ_4, PJ_5, PJ_6, PJ_7, PJ_8, PJ_9, PJ_10, PJ_11, PJ_12, PJ_13, PJ_14, PJ_15,
	PK_0, PK_1, PK_2, PK_3, PK_4, PK_5, PK_6, PK_7

} PinNames;

typedef enum {
  LOW     = 0,
  HIGH    = 1,
  CHANGE  = 2,
  FALLING = 3,
  RISING  = 4,
} PinStatus;


 typedef enum {
 INPUT            = 0x0,
  OUTPUT           = 0x1,
  INPUT_PULLUP     = 0x2,
  INPUT_PULLDOWN   = 0x3,
  OUTPUT_OPENDRAIN = 0x4,
} PinMode;



void pinMode(PinNames pinNumber, PinMode pinMode) {
  if (pinMode == INPUT) { // input mode
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_ACTIVE_HIGH);
  } else if (pinMode == INPUT_PULLUP) { // input with internal pull-up
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
  } else if (pinMode == INPUT_PULLDOWN) { // input with internal pull-down
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH);
  } else if (pinMode == OUTPUT) { // output mode
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
  }
}

void digitalWrite(PinNames pinNumber, PinStatus status) {
  gpio_pin_set_dt(&pin_name_pins[pinNumber], status);
}

PinStatus digitalRead(PinNames pinNumber) {
  return (gpio_pin_get_dt(&pin_name_pins[pinNumber]) == 1) ? HIGH : LOW;
}


#define delay(ms) k_sleep(K_MSEC(ms))

int main(void)
{
	struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
	//struct video_buffer *vbuf = &(struct video_buffer){};
	const struct device *video_dev;
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
#if (CONFIG_VIDEO_SOURCE_CROP_WIDTH && CONFIG_VIDEO_SOURCE_CROP_HEIGHT) 
	struct video_selection sel = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
	};
#endif
	//unsigned int frame = 0;
	size_t bsize;
	int i = 0;

	printf("Hello world\n");

	pinMode(PC_13, OUTPUT);
	digitalWrite(PC_13, LOW);
	delay(10);
	digitalWrite(PC_13, HIGH);
	delay(10);

	show_all_gpio_regs();

	video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (!device_is_ready(video_dev)) {
		LOG_ERR("%s: video device is not ready", video_dev->name);
		return 0;
	}

	LOG_INF("Video device: %s", video_dev->name);

	/* Get capabilities */
	caps.type = type;
	if (video_get_caps(video_dev, &caps)) {
		LOG_ERR("Unable to retrieve video capabilities");
		return 0;
	}

	LOG_INF("- Capabilities:");
	while (caps.format_caps[i].pixelformat) {
		const struct video_format_cap *fcap = &caps.format_caps[i];
		/* fourcc to string */
		LOG_INF("  %s width [%u; %u; %u] height [%u; %u; %u]",
			VIDEO_FOURCC_TO_STR(fcap->pixelformat),
			fcap->width_min, fcap->width_max, fcap->width_step,
			fcap->height_min, fcap->height_max, fcap->height_step);
		i++;
	}


	/* Get default/native format */

	fmt.type = type;
	if (video_get_format(video_dev, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return 0;
	}
	LOG_INF("video_get_format(1): ret fmt:%s w:%u h:%u",VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);

	/* try the order of stuff mentioned by @josuah */
#ifndef USE_EXAMAPLE_ORDERS
	LOG_INF("- Video format: %s %ux%u",
		VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);

	if (video_set_format(video_dev, &fmt)) {
		LOG_ERR("Unable to set format");
		return 0;
	}

#if CONFIG_VIDEO_FRAME_HEIGHT || CONFIG_VIDEO_FRAME_WIDTH
#if CONFIG_VIDEO_FRAME_HEIGHT
	fmt.height = CONFIG_VIDEO_FRAME_HEIGHT;
#endif

#if CONFIG_VIDEO_FRAME_WIDTH
	fmt.width = CONFIG_VIDEO_FRAME_WIDTH;
#endif
#endif	

	/* First set the format which has the size of the frame defined */
	LOG_INF("video_set_format: %u %u", fmt.width, fmt.height);
	if (video_set_format(video_dev, &fmt)) {
		LOG_ERR("Unable to set format");
		return 0;
	}

	/* initialize the bsize to the size of the frame */
	bsize = fmt.width * fmt.height * 2;
	/* Set the crop setting if necessary */
#if CONFIG_VIDEO_SOURCE_CROP_WIDTH && CONFIG_VIDEO_SOURCE_CROP_HEIGHT
	sel.target = VIDEO_SEL_TGT_CROP;
	sel.rect.left = CONFIG_VIDEO_SOURCE_CROP_LEFT;
	sel.rect.top = CONFIG_VIDEO_SOURCE_CROP_TOP;
	sel.rect.width = CONFIG_VIDEO_SOURCE_CROP_WIDTH;
	sel.rect.height = CONFIG_VIDEO_SOURCE_CROP_HEIGHT;
	LOG_INF("video_set_selection: VIDEO_SEL_TGT_CROP(%u, %u, %u, %u)", 
			sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
	if (video_set_selection(video_dev, &sel)) {
		LOG_ERR("Unable to set selection crop  (%u,%u)/%ux%u",
			sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
		return 0;
	}
	LOG_INF("Selection crop set to (%u,%u)/%ux%u",
		sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
	bsize = sel.rect.width * sel.rect.height * 2;
#endif

	if (video_get_format(video_dev, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return 0;
	}
	LOG_INF("video_get_format(2): ret fmt:%s w:%u h:%u pitch:%u", VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height, fmt.pitch);
	bsize = fmt.height *fmt.pitch;


#else /* USE_EXAMAPLE_ORDERS */

	/* Set the crop setting if necessary */
#if CONFIG_VIDEO_SOURCE_CROP_WIDTH && CONFIG_VIDEO_SOURCE_CROP_HEIGHT
	sel.target = VIDEO_SEL_TGT_CROP;
	sel.rect.left = CONFIG_VIDEO_SOURCE_CROP_LEFT;
	sel.rect.top = CONFIG_VIDEO_SOURCE_CROP_TOP;
	sel.rect.width = CONFIG_VIDEO_SOURCE_CROP_WIDTH;
	sel.rect.height = CONFIG_VIDEO_SOURCE_CROP_HEIGHT;
	LOG_INF("video_set_selection: VIDEO_SEL_TGT_CROP(%u, %u, %u, %u)", 
			sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
	if (video_set_selection(video_dev, &sel)) {
		LOG_ERR("Unable to set selection crop  (%u,%u)/%ux%u",
			sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
		return 0;
	}
	LOG_INF("Selection crop set to (%u,%u)/%ux%u",
		sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
#endif

	LOG_INF("CONFIG_VIDEO_FRAME_HEIGHT: %u CONFIG_VIDEO_FRAME_WIDTH: %u", CONFIG_VIDEO_FRAME_HEIGHT, CONFIG_VIDEO_FRAME_WIDTH);
#if CONFIG_VIDEO_FRAME_HEIGHT || CONFIG_VIDEO_FRAME_WIDTH
#if CONFIG_VIDEO_FRAME_HEIGHT
	fmt.height = CONFIG_VIDEO_FRAME_HEIGHT;
#endif

#if CONFIG_VIDEO_FRAME_WIDTH
	fmt.width = CONFIG_VIDEO_FRAME_WIDTH;
#endif
#if 1
	/*
	 * Check (if possible) if targeted size is same as crop
	 * and if compose is necessary
	 */
	LOG_INF("video_get_selection VIDEO_SEL_TGT_CROP");
	sel.target = VIDEO_SEL_TGT_CROP;
	err = video_get_selection(video_dev, &sel);
	if (err < 0 && err != -ENOSYS) {
		LOG_ERR("Unable to get selection crop");
		return 0;
	}

	LOG_INF("video_get_selection VIDEO_SEL_TGT_CROP(%u,%u)/%ux%u",
		sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);

	if (err == 0 && (sel.rect.width != fmt.width || sel.rect.height != fmt.height)) {
		LOG_INF("SEL!=FMT S(%u %u) F(%u %u)",  fmt.width, fmt.height, sel.rect.width, sel.rect.height);
		sel.target = VIDEO_SEL_TGT_COMPOSE;
		sel.rect.left = 0;
		sel.rect.top = 0;
		sel.rect.width = fmt.width;
		sel.rect.height = fmt.height;
		err = video_set_selection(video_dev, &sel);
		if (err < 0 && err != -ENOSYS) {
			LOG_ERR("Unable to set selection compose");
			return 0;
		}
	}
#endif

	if (strcmp(CONFIG_VIDEO_PIXEL_FORMAT, "")) {
		fmt.pixelformat = VIDEO_FOURCC_FROM_STR(CONFIG_VIDEO_PIXEL_FORMAT);
	}

#endif
	LOG_INF("- Video format: %s %ux%u",
		VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);

	if (video_set_format(video_dev, &fmt)) {
		LOG_ERR("Unable to set format");
		return 0;
	}

	/* Size to allocate for each buffer */
	if (caps.min_line_count == LINE_COUNT_HEIGHT) {
		bsize = fmt.pitch * fmt.height;
	} else {
		bsize = fmt.pitch * caps.min_line_count;
	}
#endif /*USE_EXAMAPLE_ORDER*/

	/* Alloc video buffers and enqueue for capture */
	LOG_INF("Allocate buffers %u %u", ARRAY_SIZE(buffers), bsize);
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		/*
		 * For some hardwares, such as the PxP used on i.MX RT1170 to do image rotation,
		 * buffer alignment is needed in order to achieve the best performance
		 */
		
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_FOREVER);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to alloc video buffer");
			return 0;
		}
		buffers[i]->type = type;
		LOG_INF(" %u Buffer: %p cb:%u", i, (void*)buffers[i]->buffer, bsize);
		video_enqueue(video_dev, buffers[i]);
	}

	/* Start video capture */
	if (video_stream_start(video_dev, type)) {
		LOG_ERR("Unable to start capture (interface)");
		return 0;
	}

	LOG_INF("Capture started");

	init_usb_serial();

	usb_serial_printf("Print to USB Serial\n");

#if CONFIG_VIDEO_BUFFER_POOL_NUM_MAX == 1
	bool snapshot_mode = true;
#else
	bool snapshot_mode = false;
#endif
	printk("snapshot_mode: %u\n", snapshot_mode);
	if (snapshot_mode) {
		main_loop_snapshot(video_dev, &fmt);
	} else {
		main_loop_continuous(video_dev, &fmt);
	}
}


void main_loop_continuous(const struct device *video_dev, struct video_format *fmt) {
	struct video_buffer *vbuf = &(struct video_buffer){};
	int err;
	unsigned int frame = 0;
	uint32_t loop_count = 0;
	while (1) {
		loop_count++;
		//if ((loop_count & 0x1f) == 0) printf(".");
		err = video_dequeue(video_dev, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			continue;
		}

		if ((loop_count & 0x1f) == 0) LOG_INF("Got frame %u! size: %u; timestamp %u ms",
			frame++, vbuf->bytesused, vbuf->timestamp);

		maybe_send_image(vbuf, fmt->width, fmt->height);

		err = video_enqueue(video_dev, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
		}
	}
}

void main_loop_snapshot(const struct device *video_dev, struct video_format *fmt) {
	struct video_buffer *vbuf = &(struct video_buffer){};
	int err;
	unsigned int frame = 0;

	int ch;
	while (1) {
		while ((ch = usb_serial_read()) != -1) {
			switch(ch) {
	            case 0x10:
	                {
	                	printk("Send JPEG: %u %u - Not supported\n", fmt->width, fmt->height);
	                    usb_serial_printf("NAK CMD CAM start jpg single shoot. END");
	                    //send_jpeg();
	                    usb_serial_printf("READY. END");
	                }
	                break;
	            case 0x30:
	                {
	                	printk("Send BMP: %u %u\n", fmt->width, fmt->height);

						err = video_dequeue(video_dev, &vbuf, K_FOREVER);
						if (err) {
							LOG_ERR("Unable to dequeue video buf");
							continue;
						}

						LOG_DBG("Got frame %u! size: %u; timestamp %u ms",
							frame++, vbuf->bytesused, vbuf->timestamp);

	                    usb_serial_printf(F("ACK CMD CAM start single shoot ... "));
	                    send_image(vbuf, fmt->width, fmt->height);
	                    usb_serial_printf(F("READY. END"));
						err = video_enqueue(video_dev, vbuf);
	                }
	                break;
	            case 1:
	            	{
	            		// Hack lets try sending raw file
	            		printk("Send Raw: %u %u\n", fmt->width, fmt->height);
						err = video_dequeue(video_dev, &vbuf, K_FOREVER);
						if (err) {
							LOG_ERR("Unable to dequeue video buf");
							continue;
						}
						send_raw_image(vbuf, fmt->width, fmt->height);

	            	}
	            	break;
	            default:
	            	break;
	        }
		}
    	k_sleep(K_MSEC(50));
	}
}

	

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


inline uint16_t HTONS(uint16_t x) {
#if defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640)
    return x;
#else  //byte reverse
    return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}

void send_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height)
{
    usb_serial_write(0xFF);
    usb_serial_write(0xAA);

    uint16_t *frameBuffer = (uint16_t*)vbuf->buffer;

    // BUGBUG:: maybe combine with the save to SD card code
    unsigned char bmpFileHeader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
    unsigned char bmpInfoHeader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };

    int rowSize = 4 * ((3 * frame_width + 3) / 4);  // how many bytes in the row (used to create padding)
    int fileSize = 54 + frame_height * rowSize;     // headers (54 bytes) + pixel data

    bmpFileHeader[2] = (unsigned char)(fileSize);
    bmpFileHeader[3] = (unsigned char)(fileSize >> 8);
    bmpFileHeader[4] = (unsigned char)(fileSize >> 16);
    bmpFileHeader[5] = (unsigned char)(fileSize >> 24);

    bmpInfoHeader[4] = (unsigned char)(frame_width);
    bmpInfoHeader[5] = (unsigned char)(frame_width >> 8);
    bmpInfoHeader[6] = (unsigned char)(frame_width >> 16);
    bmpInfoHeader[7] = (unsigned char)(frame_width >> 24);
    bmpInfoHeader[8] = (unsigned char)(frame_height);
    bmpInfoHeader[9] = (unsigned char)(frame_height >> 8);
    bmpInfoHeader[10] = (unsigned char)(frame_height >> 16);
    bmpInfoHeader[11] = (unsigned char)(frame_height >> 24);


    usb_serial_write_buffer(bmpFileHeader, sizeof(bmpFileHeader));  // write file header
    usb_serial_write_buffer(bmpInfoHeader, sizeof(bmpInfoHeader));  // " info header

    unsigned char bmpPad[rowSize - 3 * frame_width];
    for (int i = 0; i < (int)(sizeof(bmpPad)); i++) {  // fill with 0s
        bmpPad[i] = 0;
    }


		#ifdef GRAY_IMAGE
    uint8_t *pfb = (uint8_t*)frameBuffer;
    uint8_t *pfbRow = pfb;
    uint8_t img[3];
    for (int y = frame_height - 1; y >= 0; y--) {  // iterate image array
        pfb = pfbRow;
        for (int x = 0; x < frame_width; x++) {
            //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            img[2] = *pfb;
            img[1] = *pfb;
            img[0] = *pfb;
            usb_serial_write_buffer(img, 3);
            k_sleep(K_USEC(8));
            pfb++;
        }
        usb_serial_write_buffer(bmpPad, (4 - (frame_width * 3) % 4) % 4);  // and padding as needed
        pfbRow += frame_width;
    }
    #else 
    uint16_t *pfb = frameBuffer;
    uint8_t img[3];
    for (int y = frame_height - 1; y >= 0; y--) {  // iterate image array
        pfb = &frameBuffer[y * frame_width];
        for (int x = 0; x < frame_width; x++) {
            //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            uint16_t pixel = HTONS(*pfb++);
            img[2] = (pixel >> 8) & 0xf8;  // r
            img[1] = (pixel >> 3) & 0xfc;  // g
            img[0] = (pixel << 3);         // b
            usb_serial_write_buffer(img, 3);
            k_sleep(K_USEC(8));

        }
        usb_serial_write_buffer(bmpPad, (4 - (frame_width * 3) % 4) % 4);  // and padding as needed
    }
    #endif

    usb_serial_write(0xBB);
    usb_serial_write(0xCC);

    usb_serial_printf(F("ACK CMD CAM Capture Done. END\n"));
    //camera.setHmirror(0);

    k_sleep(K_MSEC(50));
}

void send_raw_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height)
{
    uint8_t *pb = (uint8_t*)vbuf->buffer;
    uint32_t cb = vbuf->bytesused;
	size_t cb_write = 512;

	while (cb) {
		if (cb < 512) cb_write = 512;
		size_t cb_written = usb_serial_write_buffer(pb, cb_write);
		if (!cb_written) {
			printk("Failed to write buffer");
			break;
		}
		cb -= cb_written;
	}   
}

void maybe_send_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height) {
	int ch;
	while ((ch = usb_serial_read()) != -1) {
		switch(ch) {
            case 0x10:
                {
                	printk("Send JPEG: %u %u - Not supported\n", frame_width, frame_height);
                    usb_serial_printf("NAK CMD CAM start jpg single shoot. END");
                    //send_jpeg();
                    usb_serial_printf("READY. END");
                }
                break;
            case 0x30:
                {
                	printk("Send BMP: %u %u\n", frame_width, frame_height);
                    usb_serial_printf(F("ACK CMD CAM start single shoot ... "));
                    send_image(vbuf, frame_width, frame_height);
                    usb_serial_printf(F("READY. END"));
                }
                break;
            default:
            	break;
        }

	}
}

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

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



#if defined(CONFIG_SHARED_MULTI_HEAP)
#include <zephyr/multi_heap/shared_multi_heap.h>

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
#endif /*CONFIG_SHARED_MULTI_HEAP*/

void print_gpio_regs(const char *name, GPIO_TypeDef *port) {
  LOG_INF("GPIO%s(%p) %08X %08X %08x", name, port, port->MODER, port->AFR[0], port->AFR[1]);
  delay(25);
  //USBSerial.print("GPIO");
  //USBSerial.print(name);
  //USBSerial.print(" ");
  //USBSerial.print(port->MODER, HEX);
  //USBSerial.print(" ");
  //USBSerial.print(port->AFR[0], HEX);
  //USBSerial.print(" ");
  //USBSerial.println(port->AFR[1], HEX);
}

void print_PinConfig(const char *name, GPIO_TypeDef *port, const char *regName) {
  uint8_t buffer[80];	
  uint32_t reg = 0;
  uint8_t numPins = 0;
  uint8_t numBits = 0;
  uint8_t hack = 0;
  sprintf(buffer, "GPIO %s ", name);
  if(strcmp(regName, "M") == 0) {
    strcat(buffer, "MODER: ");
    numPins = 16;
    numBits = 2;
    hack = 0;
    reg = port->MODER;
  } else if(strcmp (regName , "AL") == 0) {
    strcat(buffer, "AFRL");
    numPins = 8;
    numBits = 4;
    hack = 0;
    reg = port->AFR[0];
  } else {
    strcat(buffer, "AFRH");
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
    //uint8_t extractedBits = (reg & mask) >> (i*numBits);
    uint8_t buffer2[10];
    sprintf(buffer2, "(%u)", i+(hack*8));
    strcat(buffer, buffer2);
  }
  LOG_INF("%s\n", buffer);
  delay(25);
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

