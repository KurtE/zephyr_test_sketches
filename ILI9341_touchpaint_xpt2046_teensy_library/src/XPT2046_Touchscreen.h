/* Touchscreen library for XPT2046 Touch Controller Chip
 * Copyright (c) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _XPT2046_Touchscreen_h_
#define _XPT2046_Touchscreen_h_

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>


class TS_Point {
public:
	TS_Point(void) : x(0), y(0), z(0) {}
	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
	int16_t x, y, z;
};

class XPT2046_Touchscreen {
public:
  XPT2046_Touchscreen(struct spi_dt_spec * pspi,  
  	const struct gpio_dt_spec *tirq=nullptr, const struct gpio_dt_spec *tdbg=nullptr)  
		: _pspi(pspi), tirqPin(tirq) {  
			tdbgPin = (struct gpio_dt_spec *)tdbg;
		}
	bool begin();

	TS_Point getPoint();
	bool tirqTouched();
	bool touched();
	void readData(uint16_t *x, uint16_t *y, uint8_t *z);
	bool bufferEmpty();
	uint8_t bufferSize() { return 1; }
	void setRotation(uint8_t n) { rotation = n % 4; }
// protected:
	volatile bool isrWake=true;
	static const struct gpio_dt_spec *tdbgPin;

private:
	void update();
	static void xpt2046_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

  struct spi_dt_spec *  _pspi;
  struct spi_config _config, _config16;
	struct gpio_callback int_gpio_cb;

	const struct gpio_dt_spec *tirqPin;
	uint8_t rotation=1;
	int16_t xraw=0, yraw=0, zraw=0;
	uint32_t msraw=0x80000000;

//	SPIClass *_pspi = nullptr;
};

#endif
