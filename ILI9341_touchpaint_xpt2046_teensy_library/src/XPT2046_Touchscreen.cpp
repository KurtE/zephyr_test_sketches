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

#include "XPT2046_Touchscreen.h"
#include "USBSerialDevice.h"

#define Z_THRESHOLD     400
#define Z_THRESHOLD_INT	75
#define MSEC_THRESHOLD  3

// Should put this in some of our common stuff.
  enum {LOW=0, HIGH=1};
inline void digitalWrite(const struct gpio_dt_spec *pin, int state) {
    gpio_pin_set_dt(pin, state);
}
unsigned long millis(void) { return k_uptime_get_32(); }



static XPT2046_Touchscreen 	*isrPinptr;

static void xpt2046_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

bool XPT2046_Touchscreen::begin()
{
	int r;
	//_pspi->begin();
	//#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)
	USBSerial.println("Touchscreen::Begin called");
  memcpy(&_config, &_pspi->config, sizeof(_config) );
  memcpy(&_config16, &_pspi->config, sizeof(_config) );
  _config.operation = /*(_config.operation & ~SPI_WORD_SIZE_MASK) | */ SPI_WORD_SET(8);
  _config16.operation = /* (_config16.operation & ~SPI_WORD_SIZE_MASK) | */ SPI_WORD_SET(16);
  _config.frequency = 2000000;
  _config16.frequency = 2000000;
  printk("config: %u %x : %u %x : %u %x\n", _pspi->config.frequency, _pspi->config.operation, 
  		_config.frequency, _config.operation,_config16.frequency, _config16.operation);

//  gpio_pin_configure_dt(csPin, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
//	digitalWrite(csPin, HIGH);
	if (tirqPin) {
	  gpio_pin_configure_dt(csPin, GPIO_INPUT | GPIO_ACTIVE_HIGH);
//		attachInterrupt(digitalPinToInterrupt(tirqPin), isrPin, FALLING);
		r = gpio_pin_interrupt_configure_dt(tirqPin, GPIO_INT_EDGE_TO_ACTIVE);
		if (r < 0) {
			//LOG_ERR("Could not configure interrupt GPIO interrupt.");
			return false;
		}

		gpio_init_callback(&int_gpio_cb, xpt2046_isr_handler, BIT(tirqPin->pin));

		r = gpio_add_callback(tirqPin->port, &int_gpio_cb);
		if (r < 0) {
			//LOG_ERR("Could not set gpio callback");
			return false;
		}


		isrPinptr = this;
	}
	return true;
}



static void xpt2046_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	XPT2046_Touchscreen *o = isrPinptr;
	o->isrWake = true;
}

TS_Point XPT2046_Touchscreen::getPoint()
{
	update();
	return TS_Point(xraw, yraw, zraw);
}

bool XPT2046_Touchscreen::tirqTouched()
{
	return (isrWake);
}

bool XPT2046_Touchscreen::touched()
{
	update();
	return (zraw >= Z_THRESHOLD);
}

void XPT2046_Touchscreen::readData(uint16_t *x, uint16_t *y, uint8_t *z)
{
	update();
	*x = xraw;
	*y = yraw;
	*z = zraw;
}

bool XPT2046_Touchscreen::bufferEmpty()
{
	return ((millis() - msraw) < MSEC_THRESHOLD);
}

static int16_t besttwoavg( int16_t x , int16_t y , int16_t z ) {
  int16_t da, db, dc;
  int16_t reta = 0;
  if ( x > y ) da = x - y; else da = y - x;
  if ( x > z ) db = x - z; else db = z - x;
  if ( z > y ) dc = z - y; else dc = y - z;

  if ( da <= db && da <= dc ) reta = (x + y) >> 1;
  else if ( db <= da && db <= dc ) reta = (x + z) >> 1;
  else reta = (y + z) >> 1;   //    else if ( dc <= da && dc <= db ) reta = (x + y) >> 1;

  return (reta);
}

// TODO: perhaps a future version should offer an option for more oversampling,
//       with the RANSAC algorithm https://en.wikipedia.org/wiki/RANSAC
uint8_t XPT2046_Touchscreen::transfer(uint8_t data) {
  int ret;
  uint8_t rx;
  const struct spi_buf tx_buf = {.buf = &data, .len = sizeof(data)};
  const struct spi_buf_set tx_buf_set = {
      .buffers = &tx_buf,
      .count = 1,
  };
  const struct spi_buf rx_buf = {.buf = &rx, .len = sizeof(rx)};
  const struct spi_buf_set rx_buf_set = {
      .buffers = &rx_buf,
      .count = 1,
  };

  ret = spi_transceive(_pspi->bus, &_config, &tx_buf_set, &rx_buf_set);
  if (ret < 0) {
    return 0;
  }

  return rx;
}

uint16_t XPT2046_Touchscreen::transfer16(uint16_t data) {
  int ret;
  uint16_t rx;
  const struct spi_buf tx_buf = {.buf = &data, .len = sizeof(data)};
  const struct spi_buf_set tx_buf_set = {
      .buffers = &tx_buf,
      .count = 1,
  };
  const struct spi_buf rx_buf = {.buf = &rx, .len = sizeof(rx)};
  const struct spi_buf_set rx_buf_set = {
      .buffers = &rx_buf,
      .count = 1,
  };

  ret = spi_transceive(_pspi->bus, &_config16, &tx_buf_set, &rx_buf_set);
  if (ret < 0) {
    return 0;
  }

  return rx;
}



void XPT2046_Touchscreen::update()
{
	int16_t data[6];
	int z;
	if (!isrWake) return;
	uint32_t now = millis();
	if (now - msraw < MSEC_THRESHOLD) return;

#if 1
	// lets try doing the first part with one transfer
	static const uint8_t upd_tx1[] = {0xB1, 0, 0xC1, 0, 0x91};
	uint8_t rx[10];

  struct spi_buf tx_buf = {.buf = (void*)upd_tx1, .len = sizeof(upd_tx1)};
  const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
  struct spi_buf rx_buf = {.buf = &rx, .len = tx_buf.len};
  const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};

  spi_transceive_dt(_pspi, &tx_buf_set, &rx_buf_set);

	int16_t z1 = (int16_t)(((rx[1] << 8)	| rx[2]) >> 3);
	z = z1 + 4095;
	int16_t z2 = (int16_t)(((rx[3] << 8)	| rx[4]) >> 3);
	z -= z2;
	if (z >= Z_THRESHOLD) {
		static const uint8_t upd_tx2[] = { 0, 0x91, 0, 0xD1, 0, 0x91, 0, 0xD1, 0, 0x91};
		tx_buf.buf = (void*)upd_tx2;
		rx_buf.len = tx_buf.len = sizeof(upd_tx2);
  	spi_transceive_dt(_pspi, &tx_buf_set, &rx_buf_set);
  	// first number noisy...
		data[0] = (int16_t)(((rx[2] << 8)	| rx[3]) >> 3);
		data[1] = (int16_t)(((rx[4] << 8)	| rx[5]) >> 3);
		data[2] = (int16_t)(((rx[6] << 8)	| rx[7]) >> 3);
		data[3] = (int16_t)(((rx[8] << 8)	| rx[9]) >> 3);
	}
	else data[0] = data[1] = data[2] = data[3] = 0;	// Compiler warns these values may be used unset on early exit.

	static const uint8_t upd_tx3[] = { 0, 0xD0, 0, 0};
	tx_buf.buf = (void*)upd_tx3;
	rx_buf.len = tx_buf.len = sizeof(upd_tx3);
	spi_transceive_dt(_pspi, &tx_buf_set, &rx_buf_set);
	data[4] = (int16_t)(((rx[0] << 8)	| rx[1]) >> 3);
	data[5] = (int16_t)(((rx[2] << 8)	| rx[3]) >> 3);


#else	
	if (_pspi) {
//		_pspi->beginTransaction(SPI_SETTING);
//		digitalWrite(csPin, LOW);
		transfer(0xB1 /* Z1 */);
		int16_t z1 = transfer16(0xC1 /* Z2 */) >> 3;
		z = z1 + 4095;
		int16_t z2 = transfer16(0x91 /* X */) >> 3;
		z -= z2;
		if (z >= Z_THRESHOLD) {
			transfer16(0x91 /* X */);  // dummy X measure, 1st is always noisy
			data[0] = transfer16(0xD1 /* Y */) >> 3;
			data[1] = transfer16(0x91 /* X */) >> 3; // make 3 x-y measurements
			data[2] = transfer16(0xD1 /* Y */) >> 3;
			data[3] = transfer16(0x91 /* X */) >> 3;
		}
		else data[0] = data[1] = data[2] = data[3] = 0;	// Compiler warns these values may be used unset on early exit.
		data[4] = transfer16(0xD0 /* Y */) >> 3;	// Last Y touch power down
		data[5] = transfer16(0) >> 3;
//		digitalWrite(csPin, HIGH);
		//_pspi->endTransaction();
	}	
	// If we do not have either _pspi or _pflexspi than bail. 
	else return;
#endif

	//Serial.printf("z=%d  ::  z1=%d,  z2=%d  ", z, z1, z2);
	if (z < 0) z = 0;
	if (z < Z_THRESHOLD) { //	if ( !touched ) {
		// Serial.println();
		zraw = 0;
		if (z < Z_THRESHOLD_INT) { //	if ( !touched ) {
			if (tirqPin) isrWake = false;
		}
		return;
	}
	zraw = z;
	
	// Average pair with least distance between each measured x then y
	//Serial.printf("    z1=%d,z2=%d  ", z1, z2);
	//Serial.printf("p=%d,  %d,%d  %d,%d  %d,%d", zraw,
		//data[0], data[1], data[2], data[3], data[4], data[5]);
	int16_t x = besttwoavg( data[0], data[2], data[4] );
	int16_t y = besttwoavg( data[1], data[3], data[5] );
	
	//Serial.printf("    %d,%d", x, y);
	//Serial.println();
	if (z >= Z_THRESHOLD) {
		msraw = now;	// good read completed, set wait
		switch (rotation) {
		  case 0:
			xraw = 4095 - y;
			yraw = x;
			break;
		  case 1:
			xraw = x;
			yraw = y;
			break;
		  case 2:
			xraw = y;
			yraw = 4095 - x;
			break;
		  default: // 3
			xraw = 4095 - x;
			yraw = 4095 - y;
		}
	}
}



