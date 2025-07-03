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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "UARTDevice.h"
#include "USBSerialDevice.h"

#if 0
static const struct gpio_dt_spec connector_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), digital_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif 

const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));

UARTDevice SerialX(serial_dev);


inline int min(int a, int b) {
	return (a <= b)? a : b;
}

// lets add LED
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



int main(void)
{
	int ret;


	if (!gpio_is_ready_dt(&led)) {

		LOG_ERR("LED device not ready");
		return 0;
	}


	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	USBSerial.begin();
	SerialX.begin();
	USBSerial.println("USBToSerial test");

#if 0
	USBSerial.print("Print connector pins\n");
  	for (size_t i = 0; i < ARRAY_SIZE(connector_pins); i++) {
  		USBSerial.printf("%u %p %u\n", i, connector_pins[i].port, connector_pins[i].pin);
  	}
#endif

  	// Main loop, would be nice if we setup events for the two RX queues, but startof KISS
  	for (;;) {
  		uint8_t buffer[80];
  		int avail_read, avail_write, cb, cb_read;

  		avail_read = USBSerial.available();
  		if (avail_read) {
  			avail_write = SerialX.availableForWrite();
  			cb = min((int)sizeof(buffer), min(avail_read, avail_write));
  			if (cb) {
  				cb_read = USBSerial.read(buffer, cb);
  				SerialX.write(buffer, cb_read);
  			}
  		}

  		avail_read = SerialX.available();
  		if (avail_read) {
  			avail_write = USBSerial.availableForWrite();
  			cb = min((int)sizeof(buffer), min(avail_read, avail_write));
  			if (cb) {
  				cb_read = SerialX.read(buffer, cb);
  				USBSerial.write(buffer, cb_read);
  			}
  		}



  	}


	return 0;
}
