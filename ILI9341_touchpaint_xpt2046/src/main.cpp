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
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include "USBSerialDevice.h"

#include "UARTDevice.h"
#include "ILI9341_GIGA_zephyr.h"

#define ATP
//#define PJRC

const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
//const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));
//const struct device *const ili9341_spi =  DEVICE_DT_GET(DT_CHOSEN(spi_ili9341));
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB )

#if defined(ATP)
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341atp_spi_dev), SPI_OP, 0);
#elif defined(PJRC)
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341prjc_spi_dev), SPI_OP, 0);
#else
static struct spi_dt_spec ili9341_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ili9341_spi_dev), SPI_OP, 0);
#endif


inline int min(int a, int b) {
	return (a <= b)? a : b;
}

inline void delay(uint32_t ms) {
	k_sleep(K_MSEC(ms));
}

// Need to do this.
unsigned long micros(void) {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
  return k_cyc_to_us_floor32(k_cycle_get_64());
#else
  return k_cyc_to_us_floor32(k_cycle_get_32());
#endif
 }

#if defined(ATP)
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341atp_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#elif defined(PJRC)
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341pjrc_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#else
static const struct gpio_dt_spec ili9341_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), ili9341_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif

ILI9341_GIGA_n tft(&ili9341_spi, &ili9341_pins[0], &ili9341_pins[1], &ili9341_pins[2]);



// Lets define our toutch event.
static const struct device *const touch_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_touch));

static struct k_sem sync;

static struct {
  size_t x;
  size_t y;
  bool pressed;
} touch_point;


static void touch_event_callback(struct input_event *evt, void *user_data)
{
  //printk("TECB: %u %u %u\n", evt->code, evt->value, evt->sync);
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




//=================================================================
// forward references of functions
#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;


int main(void)
{
	int ret;

	USBSerial.begin();
	//SerialX.begin();
	USBSerial.println("Touch Paint started");

//	USBSerial.print("Print connector pins\n");
//  	for (size_t i = 0; i < ARRAY_SIZE(connector_pins); i++) {
//  		USBSerial.printf("%u %p %u\n", i, connector_pins[i].port, connector_pins[i].pin);
//  	}
  	// Main loop, would be nice if we setup events for the two RX queues, but startof KISS
#if 0
	USBSerial.printf("cs: %p %u %x\n", ili9341_pins[0].port, ili9341_pins[0].pin, ili9341_pins[0].dt_flags);
	USBSerial.printf("dc: %p %u %x\n", ili9341_pins[1].port, ili9341_pins[1].pin, ili9341_pins[1].dt_flags);
	USBSerial.printf("rst: %p %u %x\n",ili9341_pins[2].port, ili9341_pins[2].pin, ili9341_pins[2].dt_flags);
	if (ARRAY_SIZE(ili9341_pins) > 3) {
		USBSerial.printf("touch cs: %p %u %x\n",ili9341_pins[3].port, ili9341_pins[3].pin, ili9341_pins[3].dt_flags);
  		gpio_pin_configure_dt(&ili9341_pins[3], GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
  		gpio_pin_set_dt(&ili9341_pins[3] , 1);
	}
#endif
	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');
	//USBSerial.printf("SPI readdy? %c\n"), spi_is_ready_dt(&ili9341_spi)? 'Y': 'N');
  // initialize semaphore... used for touch...
  k_sem_init(&sync, 0, 1);


	tft.setDebugUART(&USBSerial);
	tft.begin();
	tft.setRotation(0);

	tft.fillScreen(ILI9341_BLACK);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_RED);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_GREEN);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_BLUE);
	k_sleep(K_MSEC(500));
	tft.fillScreen(ILI9341_WHITE);


  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
  tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
  tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
  tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
  tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);


    for (;;) {
      k_sem_take(&sync, K_FOREVER);
      USBSerial.printf("TOUCH %s X, Y: (%d, %d)\n", touch_point.pressed ? "PRESS" : "RELEASE",
        touch_point.x, touch_point.y);      
  	}
	return 0;
}


void WaitForUserInput() {
    USBSerial.print("Hit key to continue\n");
    while (USBSerial.read() == -1) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}
