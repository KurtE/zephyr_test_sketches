#ifndef _UARTDEVICE_H_
#define _UARTDEVICE_H_
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2


class UARTDevice {
public:
	UARTDevice(const struct device *const uart) : _uart(uart) {
		k_sem_init(&_rx_sem, 1, 1);
		ring_buf_init(&_rx_ringbuf, sizeof(_rx_buffer), _rx_buffer);
		k_sem_init(&_tx_sem, 1, 1);
		ring_buf_init(&_tx_ringbuf, sizeof(_tx_buffer), _tx_buffer);
	}

	enum {RX_BUFFER_SIZE=512, TX_BUFFER_SIZE=512};


	void begin(); // currently using the information from the device tree
	void flush();
	void end() { }
	size_t write(const uint8_t *buffer, size_t size);
	size_t write(const uint8_t data) { return write(&data, 1); }
	size_t print(const char *sz);
	size_t println(const char *sz);
	size_t println();
	size_t println(unsigned long ul); // total place holder
	size_t printf(const char *format, ...);

	// Add a subset of the arduino ones:
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(long long, int = DEC);
    size_t print(unsigned long long, int = DEC);

    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(long long, int = DEC);
    size_t println(unsigned long long, int = DEC);


	int available();
  	int availableForWrite();
	int peek();
	int read(uint8_t *buffer, size_t size); // currently not like Arduino only what is there...
	int read();


	uint8_t _rx_buffer[RX_BUFFER_SIZE];
	struct ring_buf _rx_ringbuf;
	struct k_sem _rx_sem;

	uint8_t _tx_buffer[TX_BUFFER_SIZE];
	struct ring_buf _tx_ringbuf;
	struct k_sem _tx_sem;

//protected:
	static void interrupt_handler(const struct device *dev, void *user_data);
	void _process_interrupt();
	const struct device *const _uart;
};


#endif