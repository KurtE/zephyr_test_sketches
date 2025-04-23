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
#include "ILI9341_GIGA_zephyr.h"

//static const struct gpio_dt_spec connector_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
//    DT_PATH(zephyr_user), digital_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
//const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));
//const struct device *const ili9341_spi =  DEVICE_DT_GET(DT_CHOSEN(spi_ili9341));
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB /*| SPI_MODE_CPOL | SPI_MODE_CPHA */)
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341_spi_dev), SPI_OP, 0);

//UARTDevice SerialX(serial_dev);
UARTDevice USBSerial(usb_uart_dev);


inline int min(int a, int b) {
	return (a <= b)? a : b;
}


static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};


ILI9341_GIGA_n tft(&ili9341_spi, &ili9341_pins[0], &ili9341_pins[1], &ili9341_pins[2]);

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


int main(void)
{
	int ret;

#if 0
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
#endif

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
	//SerialX.begin();

//	USBSerial.print("Print connector pins\n");
//  	for (size_t i = 0; i < ARRAY_SIZE(connector_pins); i++) {
//  		USBSerial.printf("%u %p %u\n", i, connector_pins[i].port, connector_pins[i].pin);
//  	}
  	// Main loop, would be nice if we setup events for the two RX queues, but startof KISS
	USBSerial.printf("cs: %p %u %x\n", ili9341_pins[0].port, ili9341_pins[0].pin, ili9341_pins[0].dt_flags);
	USBSerial.printf("dc: %p %u %x\n", ili9341_pins[1].port, ili9341_pins[1].pin, ili9341_pins[1].dt_flags);
	USBSerial.printf("rst: %p %u %x\n",ili9341_pins[2].port, ili9341_pins[2].pin, ili9341_pins[2].dt_flags);
	if (ARRAY_SIZE(ili9341_pins) > 3) {
		USBSerial.printf("touch cs: %p %u %x\n",ili9341_pins[3].port, ili9341_pins[3].pin, ili9341_pins[3].dt_flags);
  		gpio_pin_configure_dt(&ili9341_pins[3], GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
  		gpio_pin_set_dt(&ili9341_pins[3] , 1);
	}

	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');
	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');

	tft.setDebugUART(&USBSerial);
	tft.begin();
	tft.setRotation(1);

	tft.fillScreen(ILI9341_BLACK);
	k_sleep(K_MSEC(2000));
	tft.fillScreen(ILI9341_RED);
	k_sleep(K_MSEC(2000));
	tft.fillScreen(ILI9341_GREEN);
	k_sleep(K_MSEC(2000));
	tft.fillScreen(ILI9341_BLUE);
	k_sleep(K_MSEC(2000));
	tft.fillScreen(ILI9341_WHITE);
	tft.drawLine(0, tft.height()/2, tft.width()-1, tft.height()/2, ILI9341_RED);

	uint32_t *lpspi4_regs = (uint32_t*)0x403A0000ul;
	USBSerial.printf(" VERID: %08X\n", lpspi4_regs[0x0/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" PARAM: %08X\n", lpspi4_regs[0x4/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CR: %08X\n", lpspi4_regs[0x10/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" SR: %08X\n", lpspi4_regs[0x14/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" IER: %08X\n", lpspi4_regs[0x18/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DER: %08X\n", lpspi4_regs[0x1C/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CFGR0: %08X\n", lpspi4_regs[0x20/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CFGR1: %08X\n", lpspi4_regs[0x24/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DMR0: %08X\n", lpspi4_regs[0x30/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" DMR1: %08X\n", lpspi4_regs[0x34/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" CCR: %08X\n", lpspi4_regs[0x40/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" FCR: %08X\n", lpspi4_regs[0x58/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" FSR: %08X\n", lpspi4_regs[0x5C/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" TCR: %08X\n", lpspi4_regs[0x60/4]); k_sleep(K_MSEC(100));
//	USBSerial.printf(" TDR: %08X\n", lpspi4_regs[0x64/4]); k_sleep(K_MSEC(100)); // Write only register
	USBSerial.printf(" RSR: %08X\n", lpspi4_regs[0x70/4]); k_sleep(K_MSEC(100));
	USBSerial.printf(" RDR: %08X\n", lpspi4_regs[0x74/4]); k_sleep(K_MSEC(100));

  	for (;;) {
//		gpio_pin_toggle_dt(&led);
		k_sleep(K_MSEC(500));

  	}


	return 0;
}
