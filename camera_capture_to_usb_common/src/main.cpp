/*
 * Copyright (c) 2019 Linaro Limited
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//#define RAW_TEST_MODE
//#define TRY_SNAPSHOT

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/video.h>

#include <zephyr/logging/log.h>

#include "USBSerialDevice.h"

// #include "usb_serial.h"
#define GRAY_IMAGE

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

inline unsigned long millis(void) { return k_uptime_get_32(); }

extern void maybe_send_image(struct video_buffer *vbuf, uint16_t frame_width,
                             uint16_t frame_height);
extern void main_loop_continuous(const struct device *video_dev,
                                 struct video_format *fmt);
#ifdef TRY_SNAPSHOT
extern void main_loop_snapshot(const struct device *video_dev,
                               struct video_format *fmt);
#endif
extern void send_image(struct video_buffer *vbuf, uint16_t frame_width,
                       uint16_t frame_height);
extern void send_raw_image(struct video_buffer *vbuf, uint16_t frame_width,
                           uint16_t frame_height);
extern void show_all_gpio_regs();

extern int initialize_video(uint8_t camera_index = 0);



#ifndef CONFIG_HWINFO_IMXRT

extern "C" {

static const struct gpio_dt_spec pin_name_pins[] = {DT_FOREACH_PROP_ELEM_SEP(
    DT_PATH(zephyr_user), all_pin_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

// clang-format off
typedef enum {
  PA_0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7,
  PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15,
  PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7,
  PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15,
  PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7,
  PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15,
  PD_0, PD_1, PD_2, PD_3, PD_4, PD_5, PD_6, PD_7,
  PD_8, PD_9, PD_10, PD_11, PD_12, PD_13, PD_14, PD_15,
  PE_0, PE_1, PE_2, PE_3, PE_4, PE_5, PE_6, PE_7,
  PE_8, PE_9, PE_10, PE_11, PE_12, PE_13, PE_14, PE_15,
  PF_0, PF_1, PF_2, PF_3, PF_4, PF_5, PF_6, PF_7,
  PF_8, PF_9, PF_10, PF_11, PF_12, PF_13, PF_14, PF_15,
  PG_0, PG_1, PG_2, PG_3, PG_4, PG_5, PG_6, PG_7,
  PG_8, PG_9, PG_10, PG_11, PG_12, PG_13, PG_14, PG_15,
  PH_0, PH_1, PH_2, PH_3, PH_4, PH_5, PH_6, PH_7,
  PH_8, PH_9, PH_10, PH_11, PH_12, PH_13, PH_14, PH_15,
  PI_0, PI_1, PI_2, PI_3, PI_4, PI_5, PI_6, PI_7,
  PI_8, PI_9, PI_10, PI_11, PI_12, PI_13, PI_14, PI_15,
  PJ_0, PJ_1, PJ_2, PJ_3, PJ_4, PJ_5, PJ_6, PJ_7,
  PJ_8, PJ_9, PJ_10, PJ_11, PJ_12, PJ_13, PJ_14, PJ_15,
  PK_0, PK_1, PK_2, PK_3, PK_4, PK_5, PK_6, PK_7
} PinNames;
// clang-format on

typedef enum {
  LOW = 0,
  HIGH = 1,
  CHANGE = 2,
  FALLING = 3,
  RISING = 4,
} PinStatus;

typedef enum {
  INPUT = 0x0,
  OUTPUT = 0x1,
  INPUT_PULLUP = 0x2,
  INPUT_PULLDOWN = 0x3,
  OUTPUT_OPENDRAIN = 0x4,
} PinMode;

void pinMode(PinNames pinNumber, PinMode pinMode) {
  if (pinMode == INPUT) { // input mode
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_ACTIVE_HIGH);
  } else if (pinMode == INPUT_PULLUP) { // input with internal pull-up
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
  } else if (pinMode == INPUT_PULLDOWN) { // input with internal pull-down
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_INPUT | GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH);
  } else if (pinMode == OUTPUT) { // output mode
    gpio_pin_configure_dt(&pin_name_pins[pinNumber],
                          GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
  }
}

void digitalWrite(PinNames pinNumber, PinStatus status) {
  gpio_pin_set_dt(&pin_name_pins[pinNumber], status);
}

PinStatus digitalRead(PinNames pinNumber) {
  return (gpio_pin_get_dt(&pin_name_pins[pinNumber]) == 1) ? HIGH : LOW;
}

} // extern "C"

#endif

#define delay(ms) k_sleep(K_MSEC(ms))

// Camera/Video global variables.
struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
struct video_buffer *vbuf = nullptr;
const struct device *video_dev;
struct video_format fmt;


//----------------------------------------------------------------------------------
// iniitialize the camera/video
//----------------------------------------------------------------------------------
int main(void) {

  // Start up the Serial
  USBSerial.begin();
  //SerialX.begin();
  USBSerial.println("Camera capture to USB...");

  initialize_video();

  LOG_INF("Capture started");

  USBSerial.begin();
  // SerialX.begin();
#ifdef TRY_SNAPSHOT
#if CONFIG_VIDEO_BUFFER_POOL_NUM_MAX == 1
  bool snapshot_mode = true;
#else
  bool snapshot_mode = false;
#endif
  printk("snapshot_mode: %u\n", snapshot_mode);
  if (snapshot_mode) {
    main_loop_snapshot(video_dev, &fmt);
  } else {
    main_loop_continuous(video_dev, &fmt);
  }
#else
    main_loop_continuous(video_dev, &fmt);
#endif  
}

//----------------------------------------------------------------------------------
// iniitialize the camera/video
//----------------------------------------------------------------------------------
int initialize_video(uint8_t camera_index) {
  struct video_caps caps;
  enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
  size_t bsize;
  int i = 0;
  int ret = 0;

#ifdef DEFER_INIT_CAMERA
  camera_ext_clock_enable();
#endif

  // lets get camera information
#if defined(USE_CAMERA_LISTS) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), cameras)
  video_dev = dt_dcmis[camera_index];

#elif DT_HAS_CHOSEN(zephyr_camera)
  video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
#endif
  if (!device_is_ready(video_dev)) {
    printk("ERROR: %s device is not ready\n",  video_dev->name);

    if ((ret = device_init(video_dev)) < 0) {
      printk("device_init camera(%p) failed:%d\n", video_dev, ret);
      return 0;
    }
  }

  const struct device *dcmi = nullptr;
#if defined(USE_CAMERA_LISTS) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), dcmis)
  dcmi = dt_camera_sensors[camera_index];
#elif DT_HAS_CHOSEN(zephyr_camera_sensor)
  dcmi = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera_sensor));
#endif
  if (dcmi) {
    if (!device_is_ready(dcmi)) {
      if ((ret = device_init(dcmi)) < 0) {
        printk("device_init camera sensor(%p) failed:%d\n", dcmi, ret);
        return false;
      }
    }
  }

  printk("INFO: - Device name: %s\n",  video_dev->name);

  /* Get capabilities */
  caps.type = type;
  if (video_get_caps(video_dev, &caps)) {
    printk("ERROR: Unable to retrieve video capabilities\n") ;
    return 0;
  }

  /* lets see if we can set snapshot mode */
#if defined(VIDEO_CID_SNAPSHOT_MODE)
  printk("Try to set Snapshot mode...\n");
  struct video_ctrl_query cq = {.dev = video_dev, .id = VIDEO_CID_SNAPSHOT_MODE};
  if (video_query_ctrl(&cq) == 0) {
    printk("Before:\n");
    video_print_ctrl(&cq);
  }

  struct video_control ctrl_snapshot = {.id = VIDEO_CID_SNAPSHOT_MODE, .val = 2};
  if (video_set_ctrl(video_dev, &ctrl_snapshot) < 0) {
    printk("Failed to use video_control to set VIDEO_CID_SNAPSHOT_MODE");
  }

  /* lets try to retrieve the value */
  if (video_get_ctrl(video_dev, &ctrl_snapshot) < 0) {
    printk("Failed to retrieve video_control VIDEO_CID_SNAPSHOT_MODE");
  } else {
    printk("After Video Snapshot mode: %u\n", ctrl_snapshot.val);
  }


#endif

#if 0
    LOG_INF("- Capabilities: min buf:%u line: %d %d", caps.min_vbuf_count, 
            caps.min_line_count, caps.max_line_count);
#else
    LOG_INF("- Capabilities: min buf:%u", caps.min_vbuf_count);
#endif  
  while (caps.format_caps[i].pixelformat) {
    const struct video_format_cap *fcap = &caps.format_caps[i];
    /* four %c to string */
    printk("INFO:   %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n",
      (char)fcap->pixelformat, (char)(fcap->pixelformat >> 8),
      (char)(fcap->pixelformat >> 16), (char)(fcap->pixelformat >> 24),
      fcap->width_min, fcap->width_max, fcap->width_step, fcap->height_min,
      fcap->height_max, fcap->height_step);
    i++;
  }


  /* Get default/native format */
  fmt.type = type;
  if (video_get_format(video_dev, &fmt)) {
    printk("ERROR: Unable to retrieve video format\n") ;
    return 0;
  }
  printk("video_get_format returned: %u %u %x\n", fmt.width, fmt.height, fmt.pixelformat);
  /* Set format */
  fmt.width = CONFIG_VIDEO_FRAME_WIDTH;
  fmt.height = CONFIG_VIDEO_FRAME_HEIGHT;
  fmt.pixelformat = VIDEO_PIX_FMT_GREY; //VIDEO_PIX_FMT_RGB565;
  printk("updated to: %u %u %x\n", fmt.width, fmt.height, fmt.pixelformat);

  if (video_set_format(video_dev, &fmt)) {
    printk("ERROR: Unable to set up video format\n") ;
    return 0;
  }

  #if (CONFIG_VIDEO_SOURCE_CROP_WIDTH > 0) && (CONFIG_VIDEO_SOURCE_CROP_HEIGHT > 0)
  printk("Set Video Selection CROP %d %d:\n", CONFIG_VIDEO_SOURCE_CROP_WIDTH, CONFIG_VIDEO_SOURCE_CROP_HEIGHT);
  struct video_selection vselCrop;
  vselCrop.type = VIDEO_BUF_TYPE_OUTPUT;
  vselCrop.target = VIDEO_SEL_TGT_CROP;
  vselCrop.rect.left = CONFIG_VIDEO_SOURCE_CROP_LEFT;
  vselCrop.rect.top = CONFIG_VIDEO_SOURCE_CROP_TOP;
  vselCrop.rect.width = CONFIG_VIDEO_SOURCE_CROP_WIDTH;
  vselCrop.rect.height = CONFIG_VIDEO_SOURCE_CROP_HEIGHT;

  if ((ret = video_set_selection(video_dev, &vselCrop))!= 0) {
    printk("ERROR: %d\n", ret) ;
    return 0;
  }


  #endif


  printk("INFO: - Format: %c%c%c%c %ux%u %u\n",   (char)fmt.pixelformat, (char)(fmt.pixelformat >> 8),
    (char)(fmt.pixelformat >> 16), (char)(fmt.pixelformat >> 24), fmt.width, fmt.height,
    fmt.pitch);

#if 0
  if (caps.min_line_count != LINE_COUNT_HEIGHT) {
    printk("ERROR: Partial framebuffers not supported by this sample\n") ;
    return 0;
  }
#endif
  
  /* Size to allocate for each buffer */
  /* lets ask for the actual current format */
  if ((ret = video_get_format(video_dev, &fmt)) != 0) {
    printk("call to video_get_format failed: %d\n", ret);
  }
  printk("After call video_get_format: - Format: %c%c%c%c %ux%u %u\n",   (char)fmt.pixelformat, (char)(fmt.pixelformat >> 8),
    (char)(fmt.pixelformat >> 16), (char)(fmt.pixelformat >> 24), fmt.width, fmt.height,
    fmt.pitch);


  bsize = fmt.pitch * fmt.height;

  /* Alloc video buffers and enqueue for capture */
  printk("Initialze video buffer list cnt:%u size:%u\n",  ARRAY_SIZE(buffers), bsize);
  for (i = 0; i < ARRAY_SIZE(buffers); i++) {
    buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
              K_FOREVER);
    #ifdef CAMERA_USE_FIXED_BUFFER
    if ((i == 0) &&  (buffers[i] != NULL)) {
      // REAL hack change the buffer over to our fixed buffer..
      printk("  use %p instead of %p\n", buffers[i], frame_buffer);
      buffers[i]->buffer = (uint8_t *)frame_buffer;
    }
    #endif

    printk("  %d:%x\n", i, (uint32_t)buffers[i]);
    if (buffers[i] == NULL) {
      printk("ERROR: Unable to alloc video buffer\n") ;
      return 0;
    }
    buffers[i]->type = type;
    if (video_enqueue(video_dev, buffers[i]) < 0) {
      printk("Error video_enqueue(%p %p)\n", video_dev, buffers[i]);
    }
  }

//  #if defined(TRY_CAPTURE_SNAPSHOT) && (CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT) > (320*240)
//  frame_buffer = (uint16_t*)buffers[ARRAY_SIZE(buffers) - 1]->buffer;
//  #endif

  /* Set controls */
  struct video_control ctrl = {.id = VIDEO_CID_HFLIP, .val = 1};

  if (IS_ENABLED(CONFIG_VIDEO_HFLIP)) {
    video_set_ctrl(video_dev, &ctrl);
  }

  if (IS_ENABLED(CONFIG_VIDEO_VFLIP)) {
    ctrl.id = VIDEO_CID_VFLIP;
    video_set_ctrl(video_dev, &ctrl);
  }

//#ifdef TRY_CAPTURE_SNAPSHOT
//  video_set_snapshot_mode(video_dev, true);
//#endif
  /* Start video capture */
  printk("Starting  capture\n");
  if (video_stream_start(video_dev, type)) {
    printk("ERROR: Unable to start capture (interface)\n") ;
    return 0;
  }

  // Lets see if we can call video_get_selection
#if 1
  printk("Get Video Selection:\n");
  struct video_selection vsel;
  vsel.type = VIDEO_BUF_TYPE_OUTPUT;
  vsel.target = VIDEO_SEL_TGT_NATIVE_SIZE;
  if ((ret = video_get_selection(video_dev, &vsel))!= 0) {
    printk("ERROR: %d\n", ret) ;
    return 0;
  } else {
    printk("\tTy:%d Tar:%d R:%u %u %u %u\n", vsel.type, vsel.target,
      vsel.rect.left, vsel.rect.top, vsel.rect.width, vsel.rect.height);
  }
#endif

  return 1;

}



void main_loop_continuous(const struct device *video_dev,
                          struct video_format *fmt) {
  struct video_buffer *vbuf = nullptr;
  int err;
  unsigned int frame = 0;
  uint32_t loop_count = 0;
  while (1) {
    loop_count++;
    // if ((loop_count & 0x1f) == 0) printf(".");
    err = video_dequeue(video_dev, &vbuf, K_FOREVER);
    if (err) {
      LOG_ERR("Unable to dequeue video buf");
      continue;
    }

    if ((loop_count & 0x1f) == 0)
      LOG_INF("Got frame %u! size: %u; timestamp %u ms", frame++,
              vbuf->bytesused, vbuf->timestamp);

    maybe_send_image(vbuf, fmt->width, fmt->height);

    err = video_enqueue(video_dev, vbuf);
    if (err) {
      LOG_ERR("Unable to requeue video buf");
    }
  }
}

#ifdef RAW_TEST_MODE
void create_test_image(struct video_buffer *vbuf, uint16_t frame_width,
                    uint16_t frame_height) {
  uint16_t *pixels = (uint16_t *)vbuf->buffer;
  vbuf->bytesused = frame_width * frame_height * 2;

  static int color_index = 0;
  const static uint16_t color_table[] = {
		 0x0000, /*ST77XX_BLACK */
		 0xF800, /*ST77XX_RED */
		 0x07E0, /*ST77XX_GREEN */
		 0x001F, /*ST77XX_BLUE */
		 0xFFE0, /*ST77XX_YELLOW */
		 0xFD20, /*ST77XX_ORANGE */
		 0x07FF, /*ST77XX_CYAN */
		 0xFFFF /*ST77XX_WHITE */
	};

	for (int i = 0; i < frame_width * frame_height; i++) pixels[i] = color_table[color_index];
	k_sleep(K_MSEC(250));
	color_index++;
	color_index &= 0x7;
}
#endif



#ifdef TRY_SNAPSHOT
void main_loop_snapshot(const struct device *video_dev,
                        struct video_format *fmt) {
  struct video_buffer *vbuf = nullptr;
  int err;
  unsigned int frame = 0;
  bool continuous_send = false;

  int ch;
  while (1) {
    while ((ch = USBSerial.read()) != -1) {
      printk("Serial command: %x\n", ch);
      switch (ch) {
      case 0x10: {
        printk("Send JPEG: %u %u - Not supported\n", fmt->width, fmt->height);
        USBSerial.printf("NAK CMD CAM start jpg single shoot. END");
        // send_jpeg();
        USBSerial.printf("READY. END");
      } break;
      case 0x20: 
        continuous_send = true;
        break;
      case 0x21: 
        continuous_send = false;
        break;

      case 0x30: {
        printk("Send BMP: %u %u\n", fmt->width, fmt->height);

        err = video_dequeue(video_dev, &vbuf, K_FOREVER);
        if (err) {
          LOG_ERR("Unable to dequeue video buf");
          continue;
        }

        LOG_DBG("Got frame %u! size: %u; timestamp %u ms", frame++,
                vbuf->bytesused, vbuf->timestamp);

        USBSerial.printf("ACK CMD CAM start single shoot ... ");
        send_image(vbuf, fmt->width, fmt->height);
        USBSerial.printf("READY. END");
        err = video_enqueue(video_dev, vbuf);
      } break;
      case 1: {
        // Hack lets try sending raw file
        #ifdef RAW_TEST_MODE
        printk("send test image\n");
        vbuf = buffers[0];
        create_test_image(vbuf, fmt->width, fmt->height);
        send_raw_image(vbuf, fmt->width, fmt->height);

        #else  
        printk("Send Raw: %u %u\n", fmt->width, fmt->height);
        err = video_dequeue(video_dev, &vbuf, K_FOREVER);
        if (err) {
          LOG_ERR("Unable to dequeue video buf");
          continue;
        }
        send_raw_image(vbuf, fmt->width, fmt->height);
        err = video_enqueue(video_dev, vbuf);
        #endif

      } break;
      default:
        printk("Unknown Serial CMD: %x\n", ch);
        break;
      }
    }
    if (continuous_send) {
      printk("Send BMP: %u %u ", fmt->width, fmt->height);
      uint32_t start_time = millis();
      err = video_dequeue(video_dev, &vbuf, K_FOREVER);
      if (err) {
        LOG_ERR("Unable to dequeue video buf");
        continue;
        }
        uint32_t camera_time = millis();

        LOG_DBG("Got frame %u! size: %u; timestamp %u ms", frame++,
                vbuf->bytesused, vbuf->timestamp);

        USBSerial.printf("ACK CMD CAM start single shoot ... ");
        send_image(vbuf, fmt->width, fmt->height);
        uint32_t send_image_time = millis();
        printk(" %u %u\n", camera_time - start_time, send_image_time - camera_time);

        USBSerial.printf("READY. END");
        err = video_enqueue(video_dev, vbuf);

    } else {
      k_sleep(K_MSEC(50));
    }
  }
}
#endif

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline uint16_t HTONS(uint16_t x) {
#if defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640)
  return x;
#else // byte reverse
  return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}

void send_image(struct video_buffer *vbuf, uint16_t frame_width,
                uint16_t frame_height) {
  USBSerial.write(0xFF);
  USBSerial.write(0xAA);

  uint16_t *frameBuffer = (uint16_t *)vbuf->buffer;

  // BUGBUG:: maybe combine with the save to SD card code
  unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                     0,   0,   0, 54, 0, 0, 0};
  unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                     0,  0, 0, 0, 1, 0, 24, 0};

  int rowSize = 4 * ((3 * frame_width + 3) /
                     4); // how many bytes in the row (used to create padding)
  int fileSize = 54 + frame_height * rowSize; // headers (54 bytes) + pixel data

  bmpFileHeader[2] = (unsigned char)(fileSize);
  bmpFileHeader[3] = (unsigned char)(fileSize >> 8);
  bmpFileHeader[4] = (unsigned char)(fileSize >> 16);
  bmpFileHeader[5] = (unsigned char)(fileSize >> 24);

  bmpInfoHeader[4] = (unsigned char)(frame_width);
  bmpInfoHeader[5] = (unsigned char)(frame_width >> 8);
  bmpInfoHeader[6] = (unsigned char)(frame_width >> 16);
  bmpInfoHeader[7] = (unsigned char)(frame_width >> 24);
  bmpInfoHeader[8] = (unsigned char)(frame_height);
  bmpInfoHeader[9] = (unsigned char)(frame_height >> 8);
  bmpInfoHeader[10] = (unsigned char)(frame_height >> 16);
  bmpInfoHeader[11] = (unsigned char)(frame_height >> 24);

  USBSerial.write(bmpFileHeader, sizeof(bmpFileHeader)); // write file header
  USBSerial.write(bmpInfoHeader, sizeof(bmpInfoHeader)); // " info header

  unsigned char bmpPad[rowSize - 3 * frame_width];
  for (int i = 0; i < (int)(sizeof(bmpPad)); i++) { // fill with 0s
    bmpPad[i] = 0;
  }

#define PIX_PER_WRITE 20
#ifdef GRAY_IMAGE
  uint8_t *pfb = (uint8_t *)frameBuffer;
  uint8_t *pfbRow = pfb;
  uint8_t img[3 * PIX_PER_WRITE]; // See if it writes out faster
  uint8_t img_index = 0;
  for (int y = frame_height - 1; y >= 0; y--) { // iterate image array
    pfb = pfbRow;
    for (int x = 0; x < frame_width; x++) {
      // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      img[img_index + 2] = *pfb;
      img[img_index + 1] = *pfb;
      img[img_index + 0] = *pfb;
      img_index += 3;
      if (img_index == sizeof(img)) {
        USBSerial.write(img, img_index);
        k_sleep(K_USEC(8));
        img_index = 0;
      }
      pfb++;
    }
    if (img_index != 0) {
      USBSerial.write(img, img_index);
      k_sleep(K_USEC(8));
      img_index = 0;
    }
    USBSerial.write(bmpPad,
                    (4 - (frame_width * 3) % 4) % 4); // and padding as needed
    pfbRow += frame_width;
    if ((y & 0x3) == 0x3)
      printk("*");
  }
  printk("\n");
#else
  uint16_t *pfb = frameBuffer;
  uint8_t img[3];
  for (int y = frame_height - 1; y >= 0; y--) { // iterate image array
    pfb = &frameBuffer[y * frame_width];
    for (int x = 0; x < frame_width; x++) {
      // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
      uint16_t pixel = HTONS(*pfb++);
      img[2] = (pixel >> 8) & 0xf8; // r
      img[1] = (pixel >> 3) & 0xfc; // g
      img[0] = (pixel << 3);        // b
      USBSerial.write(img, 3);
      k_sleep(K_USEC(8));
    }
    USBSerial.write(bmpPad,
                    (4 - (frame_width * 3) % 4) % 4); // and padding as needed
  }
#endif

  USBSerial.write(0xBB);
  USBSerial.write(0xCC);

  USBSerial.printf("ACK CMD CAM Capture Done. END\n");
  // camera.setHmirror(0);

  k_sleep(K_MSEC(50));
}

void send_raw_image(struct video_buffer *vbuf, uint16_t frame_width,
                    uint16_t frame_height) {
  uint8_t *pb = (uint8_t *)vbuf->buffer;
  uint32_t cb = vbuf->bytesused;
  size_t cb_write = 64;

#ifndef RAW_TEST_MODE
  uint16_t *p = (uint16_t *)vbuf->buffer;
  for (int i = 0; i < frame_width * frame_height; i++) {
    *p = HTONS(*p);
    p++;
  }
#endif

  while (cb) {
    if (cb < cb_write)
      cb_write = cb;
    size_t cb_written = USBSerial.write(pb, cb_write);
    if (cb_written != cb_write) {
      printk("Failed to write buffer");
      break;
    }
    cb -= cb_written;
    pb += cb_written;
    //printk(".");
  }
}

void maybe_send_image(struct video_buffer *vbuf, uint16_t frame_width,
                      uint16_t frame_height) {
  int ch;
  while ((ch = USBSerial.read()) != -1) {
    switch (ch) {
    case 0x10: {
      printk("Send JPEG: %u %u - Not supported\n", frame_width, frame_height);
      USBSerial.printf("NAK CMD CAM start jpg single shoot. END");
      // send_jpeg();
      USBSerial.printf("READY. END");
    } break;
    case 0x30: {
      printk("Send BMP: %u %u\n", frame_width, frame_height);
      USBSerial.printf("ACK CMD CAM start single shoot ... ");
      send_image(vbuf, frame_width, frame_height);
      USBSerial.printf("READY. END");
    } break;

  case 0x1:
      send_raw_image(vbuf, frame_width, frame_height);
      break;
        
    default:
      break;
    }
  }
}

#ifndef CONFIG_HWINFO_IMXRT
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

int camera_ext_clock_enable(void) {
  int ret;
  uint32_t rate;

  const struct device *cam_ext_clk_dev = DEVICE_DT_GET(DT_NODELABEL(pwmclock));

  if (!device_is_ready(cam_ext_clk_dev)) {
    LOG_ERR("Clock start failed - ready");
    return -ENODEV;
  }

  ret = clock_control_on(cam_ext_clk_dev, (clock_control_subsys_t)0);
  if (ret < 0) {
    LOG_ERR("Clock start failed - clock_control_on");
    return ret;
  }

  ret =
      clock_control_get_rate(cam_ext_clk_dev, (clock_control_subsys_t)0, &rate);
  if (ret < 0) {
    LOG_ERR("Clock start failed - get_rate");
    return ret;
  }

  return 0;
}

SYS_INIT(camera_ext_clock_enable, POST_KERNEL,
         CONFIG_CLOCK_CONTROL_PWM_INIT_PRIORITY);

#endif

#if defined(CONFIG_SHARED_MULTI_HEAP)
#include <zephyr/multi_heap/shared_multi_heap.h>

Z_GENERIC_SECTION(SDRAM1) static uint8_t __aligned(32) smh_pool[4 * 1024 * 1024];

int smh_init(void) {
  int ret = 0;
  ret = shared_multi_heap_pool_init();
  if (ret != 0) {
    return ret;
  }

  struct shared_multi_heap_region smh_sdram = {.attr = SMH_REG_ATTR_EXTERNAL,
                                               .addr = (uintptr_t)smh_pool,
                                               .size = sizeof(smh_pool)};

  ret = shared_multi_heap_add(&smh_sdram, NULL);
  if (ret != 0) {
    return ret;
  }
  return 0;
}

SYS_INIT(smh_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /*CONFIG_SHARED_MULTI_HEAP*/

#ifdef CONFIG_HWINFO_IMXRT

void show_all_gpio_regs() {}

#else
void print_gpio_regs(const char *name, GPIO_TypeDef *port) {
  LOG_INF("GPIO%s(%p) %08X %08X %08x", name, port, port->MODER, port->AFR[0],
          port->AFR[1]);
  delay(25);
  // USBSerial.print("GPIO");
  // USBSerial.print(name);
  // USBSerial.print(" ");
  // USBSerial.print(port->MODER, HEX);
  // USBSerial.print(" ");
  // USBSerial.print(port->AFR[0], HEX);
  // USBSerial.print(" ");
  // USBSerial.println(port->AFR[1], HEX);
}

void print_PinConfig(const char *name, GPIO_TypeDef *port,
                     const char *regName) {
  char buffer[80];
  uint32_t reg = 0;
  uint8_t numPins = 0;
  uint8_t numBits = 0;
  uint8_t hack = 0;
  sprintf((char *)buffer, "GPIO %s ", name);
  if (strcmp(regName, "M") == 0) {
    strcat(buffer, "MODER: ");
    numPins = 16;
    numBits = 2;
    hack = 0;
    reg = port->MODER;
  } else if (strcmp(regName, "AL") == 0) {
    strcat(buffer, "AFRL");
    numPins = 8;
    numBits = 4;
    hack = 0;
    reg = port->AFR[0];
  } else {
    strcat(buffer, "AFRH");
    numPins = 8;
    numBits = 4;
    hack = 1;
    reg = port->AFR[1];
  }

  for (uint8_t i = 0; i < numPins; i++) {
    unsigned mask;
    // mask = ((1 << numBits2Extract) << startBit)
    mask = ((1 << numBits) - 1) << (i * numBits);
    // extractedBits = (value & mask) >> startBit
    // uint8_t extractedBits = (reg & mask) >> (i*numBits);
    char buffer2[10];
    sprintf(buffer2, "(%u)", i + (hack * 8));
    strcat(buffer, buffer2);
  }
  LOG_INF("%s\n", buffer);
  delay(25);
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

#endif