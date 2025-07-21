#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>

#ifdef CONFIG_BOARD_TEENSYMM 
#define DMA_NODE DT_NODELABEL(edma0)
#else
#define DMA_NODE DT_NODELABEL(dma1)
#endif

#define BUFFER_SIZE 128

__aligned(32) uint8_t tx_buffer[BUFFER_SIZE];
__aligned(32) uint8_t rx_buffer[BUFFER_SIZE];
volatile bool dma_completed;

void dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel, int status) {
	printk("DCB(%p, %u, %d)\n", dma_dev, user_data, channel, status);

	dma_completed = true;
}


int configure_dma_transfer(const struct device *dma_dev, uint8_t *src, uint8_t *dest, size_t size, int channel) {
  dma_completed = false;

  SCB_CleanDCache_by_Addr((uint32_t *)src, size);
  SCB_InvalidateDCache_by_Addr((uint32_t *)dest, size);

  struct dma_block_config block_cfg = {
    .source_address = (uint32_t)(src),
    .dest_address = (uint32_t)(dest),
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
    .block_count = 1,
    .head_block = &block_cfg,
    .user_data = nullptr,
    .dma_callback = dma_callback,
  };

  int ret = dma_config(dma_dev, channel, &dma_cfg);
  if (ret) {
    LOG_ERR("DMA config failed: %d", ret);
    return ret;
  }

  printk("DMAStart(%p, %u)\n", dma_dev, channel);
  ret = dma_start(dma_dev, channel);
  if (ret) {
    LOG_ERR("DMA start failed: %d", ret);
    return ret;
  }

  return 0;
}


void CheckBuffers(uint8_t *source, uint8_t *dest) {
  int error_count = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (source[i] != dest[i]) {
      error_count++;
      if (error_count < 5) printk(" (%i %x %x)", i, source[i], dest[i]);
    }
  }
  if (error_count) {
    printk(" %u\n", error_count);
  } else {
    printk("No errors\n");
  }
}


int main(void)
{

	const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);
	//struct dma_status dma_status = {0};
	k_sleep(K_MSEC(250));
	printk("SDRAM test...\n");

  int ret = 0;
  int tx_dma_channel = 0;

  for (uint8_t channel = 1; channel <= 7; channel++) {
    dma_status dmastat;
    memset(&dmastat, 0, sizeof(dmastat));
    ret = dma_get_status(dma_dev, channel, &dmastat);
    printk("ch:%u stat:%d - %u %u %u %u %u %u %u\n", channel, ret,
           dmastat.busy, dmastat.dir, dmastat.pending_length,
           dmastat.free, dmastat.write_position, dmastat.read_position, (uint32_t)dmastat.total_copied);
    if ((tx_dma_channel == 0) && (ret == 0) && (!dmastat.busy)) tx_dma_channel = channel;
  }


	//tx_dma_channel = dma_request_channel(dma_dev, 0);
	printk("DMA Channel allocated %d\n", tx_dma_channel);

  memset(tx_buffer, 0x42, BUFFER_SIZE);
  memset(rx_buffer, 0xff, BUFFER_SIZE);
  ret = configure_dma_transfer(dma_dev, tx_buffer, rx_buffer, BUFFER_SIZE, tx_dma_channel);
  if (ret != 0) {
    LOG_ERR("DMA configuration or transfer failed\n");
  }
  int wait_count = 0;
  while (!dma_completed && wait_count++ < 1000) {
    k_sleep(K_USEC(100));
  }
  if (!dma_completed) {
    LOG_ERR("DMA transfer timeout");
  }
  printk("Check Buffers: ");
  CheckBuffers(tx_buffer, rx_buffer);
  return 0;
}
