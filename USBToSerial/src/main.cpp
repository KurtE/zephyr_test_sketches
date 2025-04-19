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

#include <sample_usbd.h>

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

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#include "UARTDevice.h"


static const struct gpio_dt_spec connector_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), digital_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));

UARTDevice SerialX(serial_dev);
UARTDevice USBSerial(usb_uart_dev);

#if 0
#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static bool rx_throttled;
#endif


inline int min(int a, int b) {
	return (a <= b)? a : b;
}

// lets add LED
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static inline void print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate %u", baudrate);
	}
}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static struct usbd_context *sample_usbd;
K_SEM_DEFINE(dtr_sem, 0, 1);

static void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			k_sem_give(&dtr_sem);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		print_baudrate(msg->dev);
	}
}

static int enable_usb_device_next(void)
{
	int err;

	sample_usbd = sample_usbd_init_device(sample_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		err = usbd_enable(sample_usbd);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");

	return 0;
}
#endif /* defined(CONFIG_USB_DEVICE_STACK_NEXT) */

#if 0
static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}

			gpio_pin_toggle_dt(&led);
			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

void usb_print_string(const char *msg) {
	uart_fifo_fill(usb_uart_dev, (const uint8_t *)msg, strlen(msg));
}

#endif

int main(void)
{
	int ret;

#if 0
	if (!device_is_ready(usb_uart_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return 0;
	}

	if (!device_is_ready(serial_dev)) {
		LOG_ERR("Serial device not ready");
		return 0;
	}
	
#endif

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return 0;
	}


	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

//===============================================================	
// BUGBUG:: need to wrap this in class...
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = enable_usb_device_next();
#else
		ret = usb_enable(NULL);
#endif

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

//	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	k_sem_take(&dtr_sem, K_FOREVER);
#else
	while (true) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(usb_uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
#endif

	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(usb_uart_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(usb_uart_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

#ifndef CONFIG_USB_DEVICE_STACK_NEXT
	print_baudrate(usb_uart_dev);
#endif
//	uart_irq_callback_set(usb_uart_dev, interrupt_handler);
	/* Enable rx interrupts */
//	uart_irq_rx_enable(usb_uart_dev);

	USBSerial.begin();
	SerialX.begin();

	USBSerial.print("Print connector pins\n");
  	for (size_t i = 0; i < ARRAY_SIZE(connector_pins); i++) {
  		USBSerial.printf("%u %p %u\n", i, connector_pins[i].port, connector_pins[i].pin);
  	}

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
