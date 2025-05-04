/*
 * Copyright (c) 2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/util.h>

static const struct device *const touch_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_touch));

static struct k_sem sync;

static struct {
	size_t x;
	size_t y;
	bool pressed;
} touch_point;

static void touch_event_callback(struct input_event *evt, void *user_data)
{
	if (evt->code == INPUT_ABS_X) {
		touch_point.x = evt->value;
	}
	if (evt->code == INPUT_ABS_Y) {
		touch_point.y = evt->value;
	}
	if (evt->code == INPUT_BTN_TOUCH) {
		touch_point.pressed = evt->value;
	}
	if (evt->sync) {
		k_sem_give(&sync);
	}
}
INPUT_CALLBACK_DEFINE(touch_dev, touch_event_callback, NULL);

int main(void)
{

	if (!device_is_ready(touch_dev)) {
		//LOG_ERR("Device %s not found. Aborting sample.", touch_dev->name);
		return 0;
	}

	k_sem_init(&sync, 0, 1);

	while (1) {
		//k_msleep(REFRESH_RATE);
		k_sem_take(&sync, K_FOREVER);
		printk("TOUCH %s X, Y: (%d, %d)\n", touch_point.pressed ? "PRESS" : "RELEASE",
			touch_point.x, touch_point.y);
	}
	return 0;
}
