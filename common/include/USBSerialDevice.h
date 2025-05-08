#ifndef _USBSERIALDEVICE_H_
#define _USBSERIALDEVICE_H_
#include "UARTDevice.h"

class USBSerialDevice : public UARTDevice {
public:
	USBSerialDevice(const struct device *const uart) : UARTDevice(uart) {
	}

	virtual void begin(); // currently using the information from the device tree


};

extern USBSerialDevice USBSerial;

#endif