/*
 * Copyright (c) 2019 Linaro Limited
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>

#include "usb_serial.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

// keeping it simple c...
enum {RX_BUFFER_SIZE=512, TX_BUFFER_SIZE=512};
static uint8_t _rx_buffer[RX_BUFFER_SIZE];
static struct ring_buf _rx_ringbuf;
static struct k_sem _rx_sem;

static uint8_t _tx_buffer[TX_BUFFER_SIZE];
static struct ring_buf _tx_ringbuf;
static struct k_sem _tx_sem;


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

static void interrupt_handler(const struct device *dev, void *data) {

	uint8_t buf[8];
	int length;
	int ret = 0;

	//printk("$");
	if (!uart_irq_update(usb_uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(usb_uart_dev) && ((length = uart_fifo_read(usb_uart_dev, buf, sizeof(buf))) > 0)) {
		if (length > (int)sizeof(buf)) length = (int)sizeof(buf);
		ret = ring_buf_put(&_rx_ringbuf, &buf[0], length);

		if (ret < 0) {
			break;
		}
	}


	while (uart_irq_tx_ready(usb_uart_dev) && ((length = ring_buf_size_get(&_tx_ringbuf)) > 0)) {
		if (length > (int)sizeof(buf)) length = (int)sizeof(buf);
		ring_buf_peek(&_tx_ringbuf, &buf[0], length);

		ret = uart_fifo_fill(usb_uart_dev, &buf[0], length);
		if (ret < 0) {
			break;
		} else {
			ring_buf_get(&_tx_ringbuf, &buf[0], ret);
		}
	}

	if (ring_buf_size_get(&_tx_ringbuf) == 0) {
		uart_irq_tx_disable(usb_uart_dev);
	}

}



void init_usb_serial()
{
	int ret;

	ret = usb_enable(NULL);

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	LOG_INF("Wait for DTR");

	// initialize our semaphores and buffers.
	k_sem_init(&_rx_sem, 1, 1);
	ring_buf_init(&_rx_ringbuf, sizeof(_rx_buffer), _rx_buffer);
	k_sem_init(&_tx_sem, 1, 1);
	ring_buf_init(&_tx_ringbuf, sizeof(_tx_buffer), _tx_buffer);

	uint8_t loop_count = 0;
	while (true) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(usb_uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));

		}
#if 0
		loop_count++;
		if (loop_count > 20) {
			LOG_WRN("Waiting for DTR timeout");
			break;
		}
#endif		
	}

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
	print_baudrate(usb_uart_dev);
	uart_irq_callback_user_data_set(usb_uart_dev, interrupt_handler, NULL);


	// Clear the two buffers.
	k_sem_take(&_rx_sem, K_FOREVER);
	ring_buf_reset(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	k_sem_take(&_tx_sem, K_FOREVER);
	ring_buf_reset(&_tx_ringbuf);
	k_sem_give(&_tx_sem);

	// Enable the RX interrupt.
	uart_irq_rx_enable(usb_uart_dev);

}

size_t usb_serial_write_buffer(const uint8_t *buffer, size_t size) {
	size_t idx = 0;

	while (1) {
		k_sem_take(&_tx_sem, K_FOREVER);
		uint32_t ret = ring_buf_put(&_tx_ringbuf, &buffer[idx], size-idx);
		k_sem_give(&_tx_sem);
		if (ret < 0) {
			return 0;
		}
		idx += ret;
		if (ret == 0) {
			uart_irq_tx_enable(usb_uart_dev);
			k_yield();
		}
		if (idx == size) {
			break;
		}
	}

	uart_irq_tx_enable(usb_uart_dev);

	return size;
}

size_t usb_serial_write(uint8_t ch) {
	return usb_serial_write_buffer(&ch, 1);
}

size_t usb_serial_printf(const char *format, ...) {
	char buffer[80];
	va_list ap;
	va_start(ap, format);
	int cb_ret = vsnprintf(buffer, sizeof(buffer),format, ap);
	usb_serial_write_buffer((const uint8_t*)buffer, cb_ret);
	return cb_ret;
}

int usb_serial_available() {
	k_sem_take(&_rx_sem, K_FOREVER);
	int ret = ring_buf_size_get(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	return ret;
}

int usb_serial_availableForWrite() {
	int ret;

	k_sem_take(&_rx_sem, K_FOREVER);
	ret = ring_buf_space_get(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	return ret;
}

int usb_serial_peek() {
	uint8_t data;
	k_sem_take(&_rx_sem, K_FOREVER);
	uint32_t cb_ret = ring_buf_peek(&_rx_ringbuf, &data, 1);
	k_sem_give(&_rx_sem);

	return cb_ret? data : -1;
}

int usb_serial_read_buffer(uint8_t *buffer, size_t size) {
    k_sem_take(&_rx_sem, K_FOREVER);
    uint32_t cb_ret = ring_buf_get(&_rx_ringbuf, buffer, size);
	k_sem_give(&_rx_sem);

	return cb_ret;
}

int usb_serial_read() {
	uint8_t data;

    k_sem_take(&_rx_sem, K_FOREVER);
    uint32_t cb_ret = ring_buf_get(&_rx_ringbuf, &data, 1);
	k_sem_give(&_rx_sem);

	return cb_ret? data : -1;
}



