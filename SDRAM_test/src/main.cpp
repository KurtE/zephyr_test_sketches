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
//#define ILI9341_USE_FIXED_BUFFER
// Hack try to use fixed buffer for camera
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
#include <zephyr/devicetree.h>
#include <zephyr/multi_heap/shared_multi_heap.h>
#include <zephyr/drivers/dma.h>

inline int min(int a, int b) {
	return (a <= b)? a : b;
}


// Need to do this.
unsigned long micros(void) {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
  return k_cyc_to_us_floor32(k_cycle_get_64());
#else
  return k_cyc_to_us_floor32(k_cycle_get_32());
#endif
 }

//#define BUFFER_SIZE (32768)
#define BUFFER_SIZE (320*240*2)

//#define USE_STATIC_BUFFERS
#ifdef USE_STATIC_BUFFERS
__aligned(32) uint8_t buffer0[BUFFER_SIZE];
__aligned(32) uint8_t buffer1[BUFFER_SIZE];
__aligned(32) uint8_t buffer2[BUFFER_SIZE];
uint8_t *sdram_buffers[3] = {buffer0, buffer1, buffer2};
#else
uint8_t *sdram_buffers[3];
#endif
volatile bool dma_completed;

void dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel, int status)
{
	static bool first_time = true;
	if (first_time) {
		first_time = false;
		printk("\ndma_callback(%p %p %u %d)\n", dma_dev, user_data, channel, status);
	}
    if (status == 0) {

//        LOG_INF("DMA transfer complete on channel %d", channel);
    } else {
        LOG_ERR("DMA transfer failed with status %d on channel %d", status, channel);
    }
    dma_completed = true;
}


static struct dma_block_config g_block_cfgs[6];


int configure_dma_transfer(const struct device *dma_dev, uint8_t *src, uint8_t *dest, size_t size, int channel)
{
	int ret = 0;

	SCB_CleanDCache_by_Addr((uint32_t *)src, size);
    SCB_InvalidateDCache_by_Addr((uint32_t *)sdram_buffers[2], size);

	static bool first_time = true;
	if (first_time) {
		//first_time = false;
		//printk("configure_dma_transfer(%p, %p, %p, %u %u)\n", dma_dev, src, dest, size, channel);

		// lets try building list of send and receive buffers.
		size_t block_size = 65536-32;  // blocks < 64K and we want 32 byte dma boundaries
		uint32_t block_index = 0;

		while (size) {
			if (size < block_size) block_size = size;
			memset (&g_block_cfgs[block_index], 0, sizeof(dma_block_config)); 
	       	g_block_cfgs[block_index].source_address = (uint32_t)src,
	       	g_block_cfgs[block_index].dest_address = (uint32_t)dest,
	       	g_block_cfgs[block_index].block_size = block_size,
			g_block_cfgs[block_index].source_gather_en = 1U;

	        src += block_size;
	        dest += block_size;
	        size -= block_size;
	        if (size) g_block_cfgs[block_index].next_block = &g_block_cfgs[block_index + 1];
	        block_index++;
		}

		struct dma_block_config block_cfg = {
	        .source_address = (uint32_t)src,
	        .dest_address = (uint32_t)dest,
	        .block_size = size,
	    };

		struct dma_config dma_cfg = {
	 		.dma_slot = 0,
	        .channel_direction = MEMORY_TO_MEMORY,
	        .complete_callback_en = true,
	        .source_data_size = 1,
	        .dest_data_size = 1,
	        .source_burst_length = 1,
	        .dest_burst_length = 1,
	        .block_count = block_index,
	        .head_block = &g_block_cfgs[0],
	        .user_data = dest,
	        .dma_callback = dma_callback,
	    };
		//static bool fConfigured = false;
		//if (!fConfigured) {

		ret = dma_config(dma_dev, channel, &dma_cfg);
		if (ret != 0) {
			LOG_ERR("DMA configuration failed with error %d\n", ret);
			return ret;
		}

	} else {
		ret = dma_reload(dma_dev, channel, (uint32_t)src, (uint32_t)dest, size);
		if (ret) {
			LOG_ERR("DMA Reload failed: %d", ret);
		}
	}
    dma_completed = false;
	ret = dma_start(dma_dev, channel);
	if (ret != 0) {
		LOG_ERR("Failed to start DMA transfer with error %d\n", ret);
	}

//	LOG_INF("DMA started");

	return ret;
}


int CheckBuffers(const char *sz, uint8_t *source, uint8_t *dest) {
	int error_count = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		if (source[i] != dest[i]) {
			error_count++;
			if (error_count == 1) printk("%s ", sz);
			if (error_count < 5) printk(" (%i %x %x)", i, source[i], dest[i]);
		}
	}
	if (error_count) printk(" %u", error_count);
	return error_count;
}

int main(void)
{

	const struct device *dma_dev = DEVICE_DT_GET(DT_ALIAS(dmax));
	//struct dma_status dma_status = {0};
	int ret = 0;
	int tx_dma_channel = 1;
	// Start up the Serial
	//SerialX.begin();
	k_sleep(K_MSEC(250));
	printk("SDRAM test...\n");

	//tx_dma_channel = dma_request_channel(dma_dev, 0);
	printk("DMA Channel allocated %d\n", tx_dma_channel);

	for (uint8_t i = 0; i < (sizeof(sdram_buffers) / sizeof(sdram_buffers[0])); i++) {
		#ifndef USE_STATIC_BUFFERS
		sdram_buffers[i] = (uint8_t *)shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 32, BUFFER_SIZE);
		//sdram_buffers[i] = (uint8_t *)k_malloc(BUFFER_SIZE);
		#endif
		printk("Buffer: %u ADDR:%p size:%u\n", i, sdram_buffers[i], BUFFER_SIZE);
	}

	for (uint32_t loop_count = 0;; loop_count++) {
		printk("loop: %u(%x) ", loop_count, loop_count & 0xff);
		memset (sdram_buffers[0], loop_count & 0xff, BUFFER_SIZE);
		memset (sdram_buffers[1], 0xff, BUFFER_SIZE);
		memset (sdram_buffers[2], 0xaa, BUFFER_SIZE);

		ret = configure_dma_transfer(dma_dev, sdram_buffers[0], sdram_buffers[2], BUFFER_SIZE, tx_dma_channel);
		if (ret != 0) {
			LOG_ERR("DMA configuration or transfer failed\n");
		}

		memcpy(sdram_buffers[1], sdram_buffers[0], BUFFER_SIZE);

		CheckBuffers(" Memcpy:", sdram_buffers[0], sdram_buffers[1]);
		while (!dma_completed) k_sleep(K_MSEC(1));
	   	
        __DSB(); // Data Synchronization Barrier
        SCB_InvalidateDCache_by_Addr((uint32_t *)sdram_buffers[2], BUFFER_SIZE);
	   	if (int ret = dma_stop(dma_dev, tx_dma_channel)) {
	   		LOG_ERR("dma_stop failed: %d", ret);
	   	}

		CheckBuffers(" DMA:", sdram_buffers[0], sdram_buffers[2]);
		printk("\n");
		k_sleep(K_MSEC(259));
	}
	return 0;
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
extern unsigned long millis(void);

unsigned long millis(void) { return k_uptime_get_32(); }

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

Z_GENERIC_SECTION(SDRAM1) static uint8_t __aligned(32) smh_pool[4*1024*1024];

int smh_init(void) {
    int ret = 0;
    ret = shared_multi_heap_pool_init();
    if (ret != 0) {
        return ret;
    }

    struct shared_multi_heap_region smh_sdram = {
        .attr = SMH_REG_ATTR_EXTERNAL,
        .addr = (uintptr_t) smh_pool,
        .size = sizeof(smh_pool)
    };

    ret = shared_multi_heap_add(&smh_sdram, NULL);
    if (ret != 0) {
        return ret;
    }
	return 0;
}

SYS_INIT(smh_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
