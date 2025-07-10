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
#include <zephyr/drivers/i2c.h>

#include <zephyr/input/input.h>
#include "USBSerialDevice.h"

#include "UARTDevice.h"
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/devicetree.h>
#include <zephyr/multi_heap/shared_multi_heap.h>






#ifdef CONFIG_VIDEO_OV2640

typedef struct {
    const char *reg_name;
    uint8_t bank;
    uint16_t reg;
} OV2640_TO_NAME_t;

#define F(X) X

// copy of some of the registers from OV2640.c
#define HSIZE               0x51
#define VSIZE               0x52
#define XOFFL               0x53
#define YOFFL               0x54

#define REG32               0x32
#define COM1                0x03
#define HSTART 0x17
#define HSTOP 0x18
#define VSTART 0x19
#define VSTOP 0x1A
#define VHYX                0x55

static const OV2640_TO_NAME_t OV2640_reg_name_table[] {
    {F(" QS"), 0, 0x44},
    {F(" HSIZE"), 0, 0x51},
    {F(" VSIZE"), 0, 0x52},
    {F(" XOFFL"), 0, 0x53},
    {F(" YOFFL"), 0, 0x54},
    {F(" VHYX"), 0, 0x55},
    {F(" DPRP"), 0, 0x56},
    {F(" TEST"), 0, 0x57},
    {F(" ZMOW"), 0, 0x5A},
    {F(" ZMOH"), 0, 0x5B},
    {F(" ZMHH"), 0, 0x5C},
    {F(" BPADDR"), 0, 0x7C},
    {F(" BPDATA"), 0, 0x7D},
    {F(" SIZEL"), 0, 0x8C},
    {F(" HSIZE8"), 0, 0xC0},
    {F(" VSIZE8"), 0, 0xC1},
    {F(" CTRL1"), 0, 0xC3},
    {F(" MS_SP"), 0, 0xF0},
    {F(" SS_ID"), 0, 0xF7},
    {F(" SS_CTRL"), 0, 0xF7},
    {F(" MC_AL"), 0, 0xFA},
    {F(" MC_AH"), 0, 0xFB},
    {F(" MC_D"), 0, 0xFC},
    {F(" P_CMD"), 0, 0xFD},
    {F(" P_STATUS"), 0, 0xFE},
    {F(" CTRLI"), 0, 0x50},
    {F(" CTRLI_LP_DP"), 0, 0x80},
    {F(" CTRLI_ROUND"), 0, 0x40},
    {F(" CTRL0"), 0, 0xC2},
    {F(" CTRL2"), 0, 0x86},
    {F(" CTRL3"), 0, 0x87},
    {F(" R_DVP_SP"), 0, 0xD3},
    {F(" R_DVP_SP_AUTO_MODE"), 0, 0x80},
    {F(" R_BYPASS"), 0, 0x05},
    {F(" R_BYPASS_DSP_EN"), 0, 0x00},
    {F(" R_BYPASS_DSP_BYPAS"), 0, 0x01},
    {F(" IMAGE_MODE"), 0, 0xDA},
    {F(" RESET"), 0, 0xE0},
    {F(" MC_BIST"), 0, 0xF9},
    {F(" BANK_SEL"), 0, 0xFF},
    {F(" BANK_SEL_DSP"), 0, 0x00},
    {F(" BANK_SEL_SENSOR"), 0, 0x01},
    {F(" GAIN"), 1, 0x00},
    {F(" COM1"), 1, 0x03},
    {F(" REG_PID"), 1, 0x0A},
    {F(" REG_VER"), 1, 0x0B},
    {F(" COM4"), 1, 0x0D},
    {F(" AEC"), 1, 0x10},
    {F(" CLKRC"), 1, 0x11},
    {F(" CLKRC_DOUBLE"), 1, 0x82},
    {F(" CLKRC_DIVIDER_MASK"), 1, 0x3F},
    {F(" COM10"), 1, 0x15},
    {F(" HSTART"), 1, 0x17},
    {F(" HSTOP"), 1, 0x18},
    {F(" VSTART"), 1, 0x19},
    {F(" VSTOP"), 1, 0x1A},
    {F(" MIDH"), 1, 0x1C},
    {F(" MIDL"), 1, 0x1D},
    {F(" AEW"), 1, 0x24},
    {F(" AEB"), 1, 0x25},
    {F(" REG2A"), 1, 0x2A},
    {F(" FRARL"), 1, 0x2B},
    {F(" ADDVSL"), 1, 0x2D},
    {F(" ADDVSH"), 1, 0x2E},
    {F(" YAVG"), 1, 0x2F},
    {F(" HSDY"), 1, 0x30},
    {F(" HEDY"), 1, 0x31},
    {F(" REG32"), 1, 0x32},
    {F(" ARCOM2"), 1, 0x34},
    {F(" REG45"), 1, 0x45},
    {F(" FLL"), 1, 0x46},
    {F(" FLH"), 1, 0x47},
    {F(" COM19"), 1, 0x48},
    {F(" ZOOMS"), 1, 0x49},
    {F(" COM22"), 1, 0x4B},
    {F(" COM25"), 1, 0x4E},
    {F(" BD50"), 1, 0x4F},
    {F(" BD60"), 1, 0x50},
    {F(" REG5D"), 1, 0x5D},
    {F(" REG5E"), 1, 0x5E},
    {F(" REG5F"), 1, 0x5F},
    {F(" REG60"), 1, 0x60},
    {F(" HISTO_LOW"), 1, 0x61},
    {F(" HISTO_HIGH"), 1, 0x62},
    {F(" REG04"), 1, 0x04},
    {F(" REG08"), 1, 0x08},
    {F(" COM2"), 1, 0x09},
    {F(" COM2_STDBY"), 1, 0x10},
    {F(" COM3"), 1, 0x0C},
    {F(" COM7"), 1, 0x12},
    {F(" COM8"), 1, 0x13},
    {F(" COM8_DEFAULT"), 1, 0xC0},
    {F(" COM8_BNDF_EN"), 1, 0x20},
    {F(" COM8_AGC_EN"), 1, 0x04},
    {F(" COM8_AEC_EN"), 1, 0x01},
    {F(" COM9"), 1, 0x14},
    {F(" CTRL1_AWB"), 1, 0x08},
    {F(" VV"), 1, 0x26},
    {F(" REG32"), 1, 0x32},
};

#define CNT_REG_NAME_TABLE \
    (sizeof(OV2640_reg_name_table) / sizeof(OV2640_reg_name_table[0]))

int cameraReadRegister(const struct i2c_dt_spec *spec, uint8_t reg_addr) {
	uint8_t tries = 3;
	uint8_t value;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_read_byte_dt(spec, reg_addr, &value)) {
			return value;
		}
		/* If reading failed wait 5ms before next attempt */
		k_msleep(5);
	}

	return -1;
}

int cameraWriteRegister(const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value) {
	uint8_t tries = 3;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	while (tries-- > 0) {
		if (!i2c_reg_write_byte_dt(spec, reg_addr, value)) {
			return 0;
		}
		/* If writing failed wait 5ms before next attempt */
		k_msleep(5);
	}

	return -1;

}



void Debug_printCameraWriteRegister(uint8_t reg, uint8_t data) {
    static uint8_t showing_bank = 0;
    static uint16_t bank_1_starts_at = 0;
    uint16_t ii = CNT_REG_NAME_TABLE; // initialize to remove warning

    USBSerial.printf("cameraWriteRegister(%x, %x)", reg, data);
    if (reg == 0xff)
        showing_bank = data; // remember which bank we are
    else {
        if (showing_bank == 0) {
            for (ii = 0; ii < CNT_REG_NAME_TABLE; ii++) {
                if (OV2640_reg_name_table[ii].bank != 0)
                    break;
                if (reg == OV2640_reg_name_table[ii].reg)
                    break;
            }
        } else {
            if (bank_1_starts_at == 0) {
                for (ii = bank_1_starts_at; ii < CNT_REG_NAME_TABLE; ii++) {
                    if (OV2640_reg_name_table[ii].bank != 0) {
                        bank_1_starts_at = ii;
                        break;
                    }
                }
            }
            for (ii = bank_1_starts_at; ii < CNT_REG_NAME_TABLE; ii++) {
                if (reg == OV2640_reg_name_table[ii].reg)
                    break;
            }
        }
    }
    if ((ii < CNT_REG_NAME_TABLE) && (reg == OV2640_reg_name_table[ii].reg))
        USBSerial.printf(" - %s\n", OV2640_reg_name_table[ii].reg_name);
    else
        USBSerial.println();
}


void print_camera_registers() {
	// get the camera I2c device
	i2c_dt_spec camera_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(ov2640));

    USBSerial.println("\n*** Camera Registers ***");
    cameraWriteRegister(&camera_i2c, 0xFF, 0x00); // bank 0
    bool showing_bank_0 = true;
    for (uint16_t ii = 0; ii < (sizeof(OV2640_reg_name_table) /
                                sizeof(OV2640_reg_name_table[0]));
         ii++) {
        if ((OV2640_reg_name_table[ii].bank != 0) && showing_bank_0) {
            cameraWriteRegister(&camera_i2c, 0xff, 0x01);
            showing_bank_0 = false;
        }
        uint8_t reg_value = cameraReadRegister(&camera_i2c, OV2640_reg_name_table[ii].reg);
        USBSerial.printf("(%d) %s(%x): %u(%x)\n", OV2640_reg_name_table[ii].bank,
                     OV2640_reg_name_table[ii].reg_name,
                     OV2640_reg_name_table[ii].reg, reg_value, reg_value);
    }

    // Quick and dirty print out WIndows start and end:
    uint8_t reg32_reg = cameraReadRegister(&camera_i2c, REG32);
    uint8_t com1_reg = cameraReadRegister(&camera_i2c, COM1);

    uint16_t hstart = (cameraReadRegister(&camera_i2c, HSTART) << 3) | (reg32_reg & 0x7);
    uint16_t hstop =
        (cameraReadRegister(&camera_i2c, HSTOP) << 3) | ((reg32_reg >> 3) & 0x7);
    uint16_t vstart = (cameraReadRegister(&camera_i2c, VSTART) << 2) | (com1_reg & 0x3);
    uint16_t vstop = (cameraReadRegister(&camera_i2c, VSTOP) << 2) | ((com1_reg >> 2) & 0x3);

    USBSerial.printf(
        "Window Hor: (%u - %u) = %u * 2 = %u Vert: (%u - %u) = %u * 2 = %u\n",
        hstart, hstop, hstop - hstart, (hstop - hstart) * 2, vstart, vstop,
        vstop - vstart, (vstop - vstart) * 2);

    cameraWriteRegister(&camera_i2c, 0xFF, 0x00); // bank 0

    uint8_t vhyx_reg = cameraReadRegister(&camera_i2c, VHYX);

    uint8_t hsize = cameraReadRegister(&camera_i2c, HSIZE) | ((vhyx_reg >> 3) & 1) << 8;
    uint8_t vsize = cameraReadRegister(&camera_i2c, VSIZE) | ((vhyx_reg >> 7) & 1) << 8;

    USBSerial.printf("device\tSize H: %u * 4 = %u V: %u * 4 = %u\n", hsize,
                 hsize * 4, vsize, vsize * 4);

    uint16_t xoffl = cameraReadRegister(&camera_i2c, XOFFL) | ((vhyx_reg >> 0) & 0x7) << 8;
    uint16_t yoffl = cameraReadRegister(&camera_i2c, YOFFL) | ((vhyx_reg >> 4) & 0x7) << 8;
    USBSerial.printf("\t offset H: %u V: %u \n", xoffl, yoffl);
}
#endif

#ifdef CONFIG_VIDEO_OV5640

#define F(X) X

typedef struct {
    const char *reg_name;
    uint16_t reg;
} OV5640_TO_NAME_t;

static const OV5640_TO_NAME_t OV5640_reg_name_table[] {
    {F("SYSTEM_CTROL0"), 0x3008},
    {F("DRIVE_CAPABILITY"), 0x302c},
    {F("SC_PLLS_CTRL0"), 0x303a},
    {F("SC_PLLS_CTRL1"), 0x303b},
    {F("SC_PLLS_CTRL2"), 0x303c},
    {F("SC_PLLS_CTRL3"), 0x303d},
    {F("AEC_PK_MANUAL"), 0x3503},
    {F("X_ADDR_ST_H"), 0x3800},
    {F("X_ADDR_ST_L"), 0x3801},
    {F("Y_ADDR_ST_H"), 0x3802},
    {F("Y_ADDR_ST_L"), 0x3803},
    {F("X_ADDR_END_H"), 0x3804},
    {F("X_ADDR_END_L"), 0x3805},
    {F("Y_ADDR_END_H"), 0x3806},
    {F("Y_ADDR_END_L"), 0x3807},
    {F("X_OUTPUT_SIZE_H"), 0x3808},
    {F("X_OUTPUT_SIZE_L"), 0x3809},
    {F("Y_OUTPUT_SIZE_H"), 0x380a},
    {F("Y_OUTPUT_SIZE_L"), 0x380b},
    {F("X_TOTAL_SIZE_H"), 0x380c},
    {F("X_TOTAL_SIZE_L"), 0x380d},
    {F("Y_TOTAL_SIZE_H"), 0x380e},
    {F("Y_TOTAL_SIZE_L"), 0x380f},
    {F("X_OFFSET_H"), 0x3810},
    {F("X_OFFSET_L"), 0x3811},
    {F("Y_OFFSET_H"), 0x3812},
    {F("Y_OFFSET_L"), 0x3813},
    {F("X_INCREMENT"), 0x3814},
    {F("Y_INCREMENT"), 0x3815},
    {F("TIMING_TC_REG20"), 0x3820},
    {F("TIMING_TC_REG21"), 0x3821},
    {F("PCLK_RATIO"), 0x3824},
    {F("FRAME_CTRL01"), 0x4201},
    {F("FRAME_CTRL02"), 0x4202},
    {F("FORMAT_CTRL00"), 0x4300},
    {F("CLOCK_POL_CONTROL"), 0x4740},
    {F("ISP_CONTROL_01"), 0x5001},
    {F("FORMAT_CTRL"), 0x501F},
    {F("PRE_ISP_TEST_SETTING_1"), 0x503D},
    {F("SCALE_CTRL_1"), 0x5601},
    {F("SCALE_CTRL_2"), 0x5602},
    {F("SCALE_CTRL_3"), 0x5603},
    {F("SCALE_CTRL_4"), 0x5604},
    {F("SCALE_CTRL_5"), 0x5605},
    {F("SCALE_CTRL_6"), 0x5606},
    {F("X_START_H"), 0x5680},
    {F("X_START_L"), 0x5681},
    {F("Y_START_H"), 0x5682},
    {F("Y_START_L"), 0x5683},
    {F("X_WINDOW_H"), 0x5684},
    {F("X_WINDOW_L"), 0x5685},
    {F("Y_WINDOW_H"), 0x5686},
    {F("Y_WINDOW_L"), 0x5687},
    {F("VFIFO_CTRL0C"), 0x460C},
    {F("VFIFO_X_SIZE_H"), 0x4602},
    {F("VFIFO_X_SIZE_L"), 0x4603},
    {F("VFIFO_Y_SIZE_H"), 0x4604},
    {F("VFIFO_Y_SIZE_L"), 0x4605},
    {F("COMPRESSION_CTRL00"), 0x4400},
    {F("COMPRESSION_CTRL01"), 0x4401},
    {F("COMPRESSION_CTRL02"), 0x4402},
    {F("COMPRESSION_CTRL03"), 0x4403},
    {F("COMPRESSION_CTRL04"), 0x4404},
    {F("COMPRESSION_CTRL05"), 0x4405},
    {F("COMPRESSION_CTRL06"), 0x4406},
    {F("COMPRESSION_CTRL07"), 0x4407},
    {F("COMPRESSION_ISI_CTRL"), 0x4408},
    {F("COMPRESSION_CTRL09"), 0x4409},
    {F("COMPRESSION_CTRL0a"), 0x440a},
    {F("COMPRESSION_CTRL0b"), 0x440b},
    {F("COMPRESSION_CTRL0c"), 0x440c},
    {F("COMPRESSION_CTRL0d"), 0x440d},
    {F("COMPRESSION_CTRL0E"), 0x440e},
};

#define CNT_REG_NAME_TABLE \
    (sizeof(OV5640_reg_name_table) / sizeof(OV5640_reg_name_table[0]))


int cameraReadRegister(const struct i2c_dt_spec *spec, uint16_t addr, uint8_t &val) {
	int ret;
	struct i2c_msg msg[2];
	uint8_t addr_buf[2];

	addr_buf[1] = addr & 0xFF;
	addr_buf[0] = addr >> 8;
	msg[0].buf = addr_buf;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)&val;
	msg[1].len = 1;
	msg[1].flags = I2C_MSG_READ | I2C_MSG_STOP | I2C_MSG_RESTART;


	ret = i2c_transfer_dt(spec, msg, 2);
	return ret;
}




void print_camera_registers() {
	// get the camera I2c device
	i2c_dt_spec camera_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(ov5640));

    USBSerial.println("\n*** Camera Registers ***");

    uint8_t reg_value = 0;
    USBSerial.println("\n*** Camera Registers ***");
    for (uint16_t ii = 0; ii < (sizeof(OV5640_reg_name_table) /
                                sizeof(OV5640_reg_name_table[0]));
         ii++) {
        cameraReadRegister(&camera_i2c, OV5640_reg_name_table[ii].reg, reg_value);
        //debug.printf("%s(%x): %u(%x)\n", OV5640_reg_name_table[ii].reg_name,
        //             OV5640_reg_name_table[ii].reg, reg_value, reg_value);
        USBSerial.printf("\t0x%04X: %u(%x)\t//%s\n", 
                     OV5640_reg_name_table[ii].reg, reg_value, reg_value, OV5640_reg_name_table[ii].reg_name);

    }
    USBSerial.println();

}
#endif