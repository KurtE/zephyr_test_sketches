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

#include "USBSerialDevice.h"

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

USBSerialDevice USBSerial(usb_uart_dev);


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

void __attribute__((weak)) _on_1200_bps() {
	NVIC_SystemReset();
}

void USBSerialDevice::baudChangeHandler(const struct device *dev, uint32_t rate) {
	(void)dev; // unused
	if (rate == 1200) {
		usb_disable();
		_on_1200_bps();
	}
}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
int USBSerialDevice::usb_disable() {
	// To avoid Cannot perform port reset: 1200-bps touch: setting DTR to OFF: protocol error
	k_sleep(K_MSEC(100));
	return usbd_disable(USBSerial._usbd);
}

void USBSerialDevice::usbd_next_cb(struct usbd_context *const ctx, const struct usbd_msg *msg) {
	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			usbd_enable(ctx);
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			usbd_disable(ctx);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		uint32_t baudrate;
		uart_line_ctrl_get(USBSerial._uart, UART_LINE_CTRL_BAUD_RATE, &baudrate);
		USBSerial.baudChangeHandler(nullptr, baudrate);
	}
}

int USBSerialDevice::enable_usb_device_next(void) {
	int err;

	_usbd = usbd_init_device(USBSerialDevice::usbd_next_cb);
	if (_usbd == NULL) {
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(_usbd)) {
		err = usbd_enable(_usbd);
		if (err) {
			return err;
		}
	}
	return 0;
}
#endif /* defined(CONFIG_USB_DEVICE_STACK_NEXT) */


void USBSerialDevice::begin() {
	int ret;

#ifndef CONFIG_USB_DEVICE_STACK_NEXT
		usb_enable(NULL);
#ifndef CONFIG_CDC_ACM_DTE_RATE_CALLBACK_SUPPORT
#warning "Can't read CDC baud change, please enable CONFIG_CDC_ACM_DTE_RATE_CALLBACK_SUPPORT"
#else
		cdc_acm_dte_rate_callback_set(usb_dev, SerialUSB_::baudChangeHandler);
#endif
#else
		enable_usb_device_next();
#endif

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

#if 0
//	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");

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
#endif

	// call off to our base classes begin
	UARTDevice::begin();

//	uart_irq_callback_set(usb_uart_dev, interrupt_handler);
	/* Enable rx interrupts */
//	uart_irq_rx_enable(usb_uart_dev);

}

USBSerialDevice::operator bool() {
	uart_line_ctrl_get(_uart, UART_LINE_CTRL_DTR, &dtr);
	return dtr;
}
