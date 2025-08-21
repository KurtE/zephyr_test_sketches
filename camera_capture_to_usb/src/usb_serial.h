#pragma once
#include <stdio.h>
#include <string.h>

extern void init_usb_serial();
extern size_t usb_serial_write(uint8_t ch);
extern size_t usb_serial_write_buffer(const uint8_t *buffer, size_t size);
extern size_t usb_serial_printf(const char *format, ...);

extern int usb_serial_available();
extern int usb_serial_availableForWrite();
extern int usb_serial_peek();
extern int usb_serial_read_buffer(uint8_t *buffer, size_t size); // currently not like Arduino only what is there...
extern int usb_serial_read();
extern bool received_dtr();