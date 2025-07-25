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

	/* Set the crop setting if necessary */
#if CONFIG_VIDEO_SOURCE_CROP_WIDTH && CONFIG_VIDEO_SOURCE_CROP_HEIGHT
	sel.target = VIDEO_SEL_TGT_CROP;
	sel.rect.left = CONFIG_VIDEO_SOURCE_CROP_LEFT;
	sel.rect.top = CONFIG_VIDEO_SOURCE_CROP_TOP;
	sel.rect.width = CONFIG_VIDEO_SOURCE_CROP_WIDTH;
	sel.rect.height = CONFIG_VIDEO_SOURCE_CROP_HEIGHT;
	if (video_set_selection(video_dev, &sel)) {
		LOG_ERR("Unable to set selection crop  (%u,%u)/%ux%u",
			sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
		return 0;
	}
	LOG_INF("Selection crop set to (%u,%u)/%ux%u",
		sel.rect.left, sel.rect.top, sel.rect.width, sel.rect.height);
#endif

#if CONFIG_VIDEO_FRAME_HEIGHT || CONFIG_VIDEO_FRAME_WIDTH
#if CONFIG_VIDEO_FRAME_HEIGHT
	fmt.height = CONFIG_VIDEO_FRAME_HEIGHT;
#endif

#if CONFIG_VIDEO_FRAME_WIDTH
	fmt.width = CONFIG_VIDEO_FRAME_WIDTH;
#endif
#if 0
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

	if (err == 0 && (sel.rect.width != fmt.width || sel.rect.height != fmt.height)) {
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
	while (1) {
		printf(".");
		err = video_dequeue(video_dev, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			return 0;
		}

		LOG_DBG("Got frame %u! size: %u; timestamp %u ms",
			frame++, vbuf->bytesused, vbuf->timestamp);

		bool display_frame = false;
		int ch;
		while ((ch = usb_serial_read()) != -1) {
			if (ch == 1) display_frame = true;
		}
		if (display_frame) {
			uint16_t *pixels = (uint16_t*)vbuf->buffer;
			uint16_t count_pixels = vbuf->bytesused / 2;
			while (count_pixels) {
		        usb_serial_write((*pixels >> 8) & 0xFF);
        		usb_serial_write((*pixels) & 0xFF);
        		pixels++;
        		count_pixels--;
        	}
		}

		err = video_enqueue(video_dev, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			return 0;
		}
	}
}

