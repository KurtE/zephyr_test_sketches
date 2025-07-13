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
// Turn on this option, to test to ee if camera errors out without screeen
//#define TIMED_WAIT_NO_TFT (1000/6)

// Try using fixed normal memory buffer for display
//#define ST77XX_USE_FIXED_BUFFER
// Hack try to use fixed buffer for camera
//#define CAMERA_USE_FIXED_BUFFER
//#define SWAP_PIXEL_BYTES

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include "USBSerialDevice.h"

#include "UARTDevice.h"
#include "ST77XX_zephyr.h"


const struct device *const usb_uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
//const struct device *const serial_dev = DEVICE_DT_GET(DT_CHOSEN(uart_passthrough));
//const struct device *const st77xx_spi =  DEVICE_DT_GET(DT_CHOSEN(spi_ili9341));
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB )

static struct spi_dt_spec st77xx_spi =
  SPI_DT_SPEC_GET(DT_NODELABEL(st7796_spi_dev), SPI_OP, 0);

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

static const struct gpio_dt_spec st77xx_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), st77xx_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

ST7796_zephyr tft(&st77xx_spi, &st77xx_pins[0], &st77xx_pins[1], &st77xx_pins[2]);


// Camera/Video global variables.
// map() transforms input "x" from one numerical range to another.  For example, if
extern void WaitForUserInput(uint32_t timeout);
extern void show_all_gpio_regs();
extern void initialize_display();

#if defined(ST77XX_USE_FIXED_BUFFER) || defined(CAMERA_USE_FIXED_BUFFER)
uint16_t frame_buffer[CONFIG_VIDEO_WIDTH*CONFIG_VIDEO_HEIGHT];
#endif



int main(void)
{

  int ret;

  // Start up the Serial
  USBSerial.begin();
  //SerialX.begin();
  USBSerial.println("Camera capture...");

  // initialize the display
  initialize_display();

  // show all registers
//  show_all_gpio_regs();
  int frame_count = 0;
  uint32_t read_frame_sum = 0;
  uint32_t write_rect_sum = 0;
  int sum_count = 0;

  printk("Starting main loop\n");
    for (;;) {
    if (USBSerial.available()) {
      while (USBSerial.read() != -1) {}
      USBSerial.println("*** Paused ***");
      int ch;
      while ((ch = USBSerial.read()) == -1) {} 
      while (USBSerial.read() != -1) {}
    }
}

  return 0;
}

//----------------------------------------------------------------------------------
// iniitialize the display and do some graphic outputs to just to make sure it
// is working
//----------------------------------------------------------------------------------
void initialize_display() {
  //tft.setDebugUART(&USBSerial);
  tft.begin();
  tft.setRotation(1);

  tft.fillScreen(ST77XX_BLACK);
  k_sleep(K_MSEC(500));
  tft.fillScreen(ST77XX_RED);
  k_sleep(K_MSEC(500));
  tft.fillScreen(ST77XX_GREEN);
  k_sleep(K_MSEC(500));
  tft.fillScreen(ST77XX_BLUE);
  k_sleep(K_MSEC(500));
  tft.fillScreen(ST77XX_WHITE);

//  WaitForUserInput(1000);
  tft.fillScreen(ST77XX_BLACK);
  uint32_t t0 = micros();
  tft.fillRectHGradient(10, 10, 100, 100, tft.color565(0 << 3, 0, 0), tft.color565(9 << 3, 0, 0));
  uint32_t t1 = micros();
  tft.fillRectHGradient(130, 10, 100, 100, ST77XX_YELLOW, ST77XX_GREEN);
  uint32_t t2 = micros();
  tft.fillRectVGradient(10, 130, 100, 100, tft.color565(0, 0, 0 << 3), tft.color565(0, 0, 9 << 3));
  uint32_t t3 = micros();
  tft.fillRectVGradient(130, 130, 100, 100, ST77XX_CYAN, ST77XX_BLUE);
  uint32_t t4 = micros();
  USBSerial.printf("%u: %u %u %u %u\n", t4-t0, t1-t0, t2-t1, t3-t2, t4-t3);
  k_sleep(K_MSEC(500));
//  WaitForUserInput(2000);


}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
extern unsigned long millis(void);

unsigned long millis(void) { return k_uptime_get_32(); }

void WaitForUserInput(uint32_t timeout) {
    USBSerial.print("Hit key to continue\n");
    uint32_t time_start = millis();
    while ((USBSerial.read() == -1) && ((uint32_t)(millis() - time_start)  < timeout)) ;
    while (USBSerial.read() != -1) ;
    USBSerial.print("Done!\n");
}

void print_gpio_regs(const char *name, GPIO_TypeDef *port) {
  //printk("GPIO%s(%p) %08X %08X %08x\n", name, port, port->MODER, port->AFR[0], port->AFR[1]);
  USBSerial.print("GPIO");
  USBSerial.print(name);
  USBSerial.print(" ");
  USBSerial.print(port->MODER, HEX);
  USBSerial.print(" ");
  USBSerial.print(port->AFR[0], HEX);
  USBSerial.print(" ");
  USBSerial.println(port->AFR[1], HEX);
}

void print_PinConfig(const char *name, GPIO_TypeDef *port, const char *regName) {
  uint32_t reg = 0;
  uint8_t numPins = 0;
  uint8_t numBits = 0;
  uint8_t hack = 0;
  USBSerial.print("GPIO");
  USBSerial.print(name);
  USBSerial.print(" ");
  if(strcmp(regName, "M") == 0) {
    USBSerial.print("MODER: ");
    numPins = 16;
    numBits = 2;
    hack = 0;
    reg = port->MODER;
  } else if(strcmp (regName , "AL") == 0) {
    USBSerial.print("AFRL");
    numPins = 8;
    numBits = 4;
    hack = 0;
    reg = port->AFR[0];
  } else {
    USBSerial.print("AFRH");
    numPins = 8;
    numBits = 4;
    hack = 1;
    reg = port->AFR[1];
  }


  for(uint8_t i = 0; i < numPins; i++) {
    unsigned  mask;
    //mask = ((1 << numBits2Extract) << startBit)
    mask = ((1 << numBits) - 1) << (i*numBits);
    //extractedBits = (value & mask) >> startBit
    uint8_t extractedBits = (reg & mask) >> (i*numBits);
    USBSerial.print("("); USBSerial.print(i+(hack*8)); USBSerial.print(")"); 
    USBSerial.print(extractedBits); USBSerial.print(", ");
  }
  USBSerial.println();
}

void show_all_gpio_regs() {
  print_gpio_regs("A", (GPIO_TypeDef *)GPIOA_BASE);
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "M");
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "AL");
  print_PinConfig("A", (GPIO_TypeDef *)GPIOC_BASE, "AH");

  print_gpio_regs("B", (GPIO_TypeDef *)GPIOB_BASE);
  print_gpio_regs("C", (GPIO_TypeDef *)GPIOC_BASE);
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "M");
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "AL");
  print_PinConfig("C", (GPIO_TypeDef *)GPIOC_BASE, "AH");

  print_gpio_regs("D", (GPIO_TypeDef *)GPIOD_BASE);
  print_gpio_regs("E", (GPIO_TypeDef *)GPIOE_BASE);
  print_gpio_regs("F", (GPIO_TypeDef *)GPIOF_BASE);
  print_gpio_regs("G", (GPIO_TypeDef *)GPIOG_BASE);
  print_gpio_regs("H", (GPIO_TypeDef *)GPIOH_BASE);
  print_gpio_regs("I", (GPIO_TypeDef *)GPIOI_BASE);
  print_gpio_regs("J", (GPIO_TypeDef *)GPIOJ_BASE);
  print_gpio_regs("K", (GPIO_TypeDef *)GPIOK_BASE);
}


