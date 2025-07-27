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

#include <zephyr/logging/log.h>

#include "usb_serial.h"

#ifdef CONFIG_TEST
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
#endif

#if !DT_HAS_CHOSEN(zephyr_camera)
#error No camera chosen in devicetree. Missing "--shield" or "--snippet video-sw-generator" flag?
#endif

void maybe_send_image(struct video_buffer *vbuf, uint16_t frame_width, uint16_t frame_height);

int main(void)
{
	struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
	struct video_buffer *vbuf = &(struct video_buffer){};
	const struct device *video_dev;
	struct video_format fmt;
	struct video_caps caps;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
#if (CONFIG_VIDEO_SOURCE_CROP_WIDTH && CONFIG_VIDEO_SOURCE_CROP_HEIGHT) ||	\
	CONFIG_VIDEO_FRAME_HEIGHT || CONFIG_VIDEO_FRAME_WIDTH
	struct video_selection sel = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
	};
#endif
	unsigned int frame = 0;
	size_t bsize;
	int i = 0;
	int err;

	printf("Hello world\n");
	/* When the video shell is enabled, do not run the capture loop */
	if (IS_ENABLED(CONFIG_VIDEO_SHELL)) {
		LOG_INF("Letting the user control the device with the video shell");
		return 0;
	}

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
	LOG_INF("video_get_format: ret fmt:%u w:%u h:%u",fmt.pixelformat, fmt.width, fmt.height);

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

	/* Alloc video buffers and enqueue for capture */
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


	/* Grab video frames */
	vbuf->type = type;
	uint32_t loop_count = 0;
	while (1) {
		loop_count++;
		if ((loop_count & 0x1f) == 0) printf(".");
		err = video_dequeue(video_dev, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			return 0;
		}

		LOG_DBG("Got frame %u! size: %u; timestamp %u ms",
			frame++, vbuf->bytesused, vbuf->timestamp);

		maybe_send_image(vbuf, fmt.width, fmt.height);

		err = video_enqueue(video_dev, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			return 0;
		}
	}
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define F(x) x

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

    usb_serial_write(0xBB);
    usb_serial_write(0xCC);

    usb_serial_printf(F("ACK CMD CAM Capture Done. END\n"));
    //camera.setHmirror(0);

    k_sleep(K_MSEC(50));
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
