#ifndef _USBSERIALDEVICE_H_
#define _USBSERIALDEVICE_H_
#include "UARTDevice.h"

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
extern "C" struct usbd_context *usbd_init_device(usbd_msg_cb_t msg_cb);
#endif

class USBSerialDevice : public UARTDevice {
public:
	USBSerialDevice(const struct device *const uart) : UARTDevice(uart) {
	}

	virtual void begin(); // currently using the information from the device tree

	operator bool() override;


	uint32_t dtr = 0;
	uint32_t baudrate;
	static void baudChangeHandler(const struct device *dev, uint32_t rate);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	struct usbd_context *_usbd;
	int enable_usb_device_next();
	static void usbd_next_cb(struct usbd_context *const ctx, const struct usbd_msg *msg);
	static int usb_disable();
#endif
};

extern USBSerialDevice USBSerial;

#endif