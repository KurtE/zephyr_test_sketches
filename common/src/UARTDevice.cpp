#include "UARTDevice.h"
#include <zephyr/logging/log.h>

// This is more or less the same as the Arduino wrapper...

void UARTDevice::begin() {
	if (!device_is_ready(_uart)) {
		printk("ERROR: UART device not ready");
		return;;
	}
	// probably more later, just make sure rx is setup as well as our handler
	uart_irq_callback_user_data_set(_uart, UARTDevice::interrupt_handler, this);

	// Clear the two buffers.
	k_sem_take(&_rx_sem, K_FOREVER);
	ring_buf_reset(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	k_sem_take(&_tx_sem, K_FOREVER);
	ring_buf_reset(&_tx_ringbuf);
	k_sem_give(&_tx_sem);

	// Enable the RX interrupt.
	uart_irq_rx_enable(_uart);
}


void UARTDevice::flush() {
	while (ring_buf_size_get(&_tx_ringbuf) > 0) {
		k_yield();
	}
	while (uart_irq_tx_complete(_uart) == 0){
		k_yield();
	}

}

size_t UARTDevice::write(const uint8_t *buffer, size_t size) {
	size_t idx = 0;

	while (1) {
		k_sem_take(&_tx_sem, K_FOREVER);
		auto ret = ring_buf_put(&_tx_ringbuf, &buffer[idx], size-idx);
		k_sem_give(&_tx_sem);
		if (ret < 0) {
			return 0;
		}
		idx += ret;
		if (ret == 0) {
			uart_irq_tx_enable(_uart);
			k_yield();
		}
		if (idx == size) {
			break;
		}
	}

	uart_irq_tx_enable(_uart);

	return size;

}

size_t UARTDevice::print(const char *sz) {
	return write((const uint8_t*)sz, strlen(sz));
}

size_t UARTDevice::println(const char *sz) {
	size_t cb = write((const uint8_t*)sz, strlen(sz));
	cb += println();
	return cb;
}

size_t UARTDevice::println() {
	return write((const uint8_t*)"\n", 1);
}

size_t UARTDevice::print(char c) {return print((long)c); }
size_t UARTDevice::print(unsigned char c, int f) {return print((unsigned long int)c, f);}
size_t UARTDevice::print(int v, int f) {return print((long int)v, f);}
size_t UARTDevice::print(unsigned int v, int f) {return print((unsigned long int)v, f);}

size_t UARTDevice::print(long v, int f) {
	char buffer[10];

	switch(f) {
	case 10:
		sprintf(buffer, "%ld", v);
		break;
	case 16:
		sprintf(buffer, "%lx", v);
		break;
	}
	return write((const uint8_t*)buffer, strlen(buffer));
}

size_t UARTDevice::print(unsigned long v, int f) {
	char buffer[10];

	switch(f) {
	case 10:
		sprintf(buffer, "%lu", v);
		break;
	case 16:
		sprintf(buffer, "%lx", v);
		break;
	}
	return write((const uint8_t*)buffer, strlen(buffer));
}

size_t UARTDevice::print(long long v, int f) {
	return print("(LL)");

}
size_t UARTDevice::print(unsigned long long v, int f) {
	return print("(ULL)");
}

size_t UARTDevice::println(char c) {return println((long)c); }
size_t UARTDevice::println(unsigned char c, int f) {return println((unsigned long int)c, f);}
size_t UARTDevice::println(int v, int f) {return println((long int)v, f);}
size_t UARTDevice::println(unsigned int v, int f) {return println((unsigned long int)v, f);}


size_t UARTDevice::println(long v, int f) {
	size_t ret = print(v, f);
	return ret + println();

}


size_t UARTDevice::println(unsigned long ul, int f) {
	size_t ret = print(ul, f);
	return ret + println();
}



size_t UARTDevice::printf(const char *format, ...) {
	char buffer[80];
	va_list ap;
	va_start(ap, format);
	int cb_ret = vsnprintf(buffer, sizeof(buffer),format, ap);
	write((const uint8_t*)buffer, cb_ret);
	return cb_ret;
}


int UARTDevice::available() {
	int ret;

	k_sem_take(&_rx_sem, K_FOREVER);
	ret = ring_buf_size_get(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	return ret;

}

int UARTDevice::availableForWrite() {
	int ret;

	k_sem_take(&_rx_sem, K_FOREVER);
	ret = ring_buf_space_get(&_rx_ringbuf);
	k_sem_give(&_rx_sem);

	return ret;

}

int UARTDevice::peek() {
	uint8_t data;
	k_sem_take(&_rx_sem, K_FOREVER);
	uint32_t cb_ret = ring_buf_peek(&_rx_ringbuf, &data, 1);
	k_sem_give(&_rx_sem);

	return cb_ret? data : -1;

}

int UARTDevice::read(uint8_t *buffer, size_t size) {// currently not like Arduino only what is there...
    k_sem_take(&_rx_sem, K_FOREVER);
    uint32_t cb_ret = ring_buf_get(&_rx_ringbuf, buffer, size);
	k_sem_give(&_rx_sem);

	return cb_ret;
}


int UARTDevice::read() {
	uint8_t data;

    k_sem_take(&_rx_sem, K_FOREVER);
    uint32_t cb_ret = ring_buf_get(&_rx_ringbuf, &data, 1);
	k_sem_give(&_rx_sem);

	return cb_ret? data : -1;

}


void UARTDevice::interrupt_handler(const struct device *dev, void *data) {
	reinterpret_cast<UARTDevice *>(data)->_process_interrupt();
}

// I am not going to have the IRQ take the semaphores for now...

void UARTDevice::_process_interrupt() {
	uint8_t buf[8];
	int length;
	int ret = 0;

	if (!uart_irq_update(_uart)) {
		return;
	}

	while (uart_irq_rx_ready(_uart) && ((length = uart_fifo_read(_uart, buf, sizeof(buf))) > 0)) {
		if (length > (int)sizeof(buf)) length = (int)sizeof(buf);
		ret = ring_buf_put(&_rx_ringbuf, &buf[0], length);

		if (ret < 0) {
			break;
		}
	}


	while (uart_irq_tx_ready(_uart) && ((length = ring_buf_size_get(&_tx_ringbuf)) > 0)) {
		if (length > (int)sizeof(buf)) length = (int)sizeof(buf);
		ring_buf_peek(&_tx_ringbuf, &buf[0], length);

		ret = uart_fifo_fill(_uart, &buf[0], length);
		if (ret < 0) {
			break;
		} else {
			ring_buf_get(&_tx_ringbuf, &buf[0], ret);
		}
	}

	if (ring_buf_size_get(&_tx_ringbuf) == 0) {
		uart_irq_tx_disable(_uart);
	}

}
