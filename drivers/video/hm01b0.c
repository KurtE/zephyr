/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT himax_hm01b0

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "video_device.h"
#include "video_common.h"

LOG_MODULE_REGISTER(hm01b0, CONFIG_VIDEO_LOG_LEVEL);
#define MAX_FRAME_RATE 10
#define MIN_FRAME_RATE 1
#define HM01B0_ID      0x01B0

#define HM01B0_REG8(addr)             ((addr) | VIDEO_REG_ADDR16_DATA8)
#define HM01B0_REG16(addr)            ((addr) | VIDEO_REG_ADDR16_DATA16_BE)
#define HM01B0_CCI_ID                 HM01B0_REG16(0x0000)
#define HM01B0_CCI_STS                HM01B0_REG8(0x0100)
#define HM01B0_CCI_RESET              HM01B0_REG8(0x0103)
#define HM01B0_CCI_GRP_PARAM_HOLD     HM01B0_REG8(0x0104)
#define HM01B0_CCI_INTEGRATION_H      HM01B0_REG16(0x0202)
#define HM01B0_CCI_FRAME_LENGTH_LINES HM01B0_REG16(0x0340)
#define HM01B0_CCI_LINE_LENGTH_PCLK   HM01B0_REG16(0x0342)
#define HM01B0_CCI_WIDTH              HM01B0_REG8(0x0383)
#define HM01B0_CCI_HEIGHT             HM01B0_REG8(0x0387)
#define HM01B0_CCI_BINNING_MODE       HM01B0_REG8(0x0390)
#define HM01B0_CCI_QVGA_WIN_EN        HM01B0_REG8(0x3010)
#define HM01B0_CCI_BIT_CONTROL        HM01B0_REG8(0x3059)
#define HM01B0_CCI_OSC_CLOCK_DIV      HM01B0_REG8(0x3060)

enum hm01b0_resolution {
	RESOLUTION_160x120,
	RESOLUTION_320x240,
	RESOLUTION_320x320,
	RESOLUTION_326x244,
	RESOLUTION_160x120_44,
	RESOLUTION_326x244_44,
	RESOLUTION_320x320_44,
};

//#define KURTE_HACKING
#ifdef KURTE_HACKING
/* Register set */
/* Read only registers */
#define MODEL_ID_H                  0x0000
#define MODEL_ID_L                  0x0001
#define FRAME_COUNT                 0x0005
#define PIXEL_ORDER                 0x0006
/* R&W registers */
/* Sensor mode control */
#define MODE_SELECT                 0x0100
#define IMG_ORIENTATION             0x0101
#define SW_RESET                    0x0103
#define GRP_PARAM_HOLD              0x0104
/* Sensor exposure gain control */
#define INTEGRATION_H               0x0202
#define INTEGRATION_L               0x0203
#define ANALOG_GAIN                 0x0205
#define DIGITAL_GAIN_H              0x020E
#define DIGITAL_GAIN_L              0x020F
/* Frame timing control */
#define FRAME_LEN_LINES_H           0x0340
#define FRAME_LEN_LINES_L           0x0341
#define LINE_LEN_PCK_H              0x0342
#define LINE_LEN_PCK_L              0x0343
/* Binning mode control */
#define READOUT_X                   0x0383
#define READOUT_Y                   0x0387
#define BINNING_MODE                0x0390
/* Test pattern control */
#define TEST_PATTERN_MODE           0x0601
/* Black level control */
#define BLC_CFG                     0x1000
#define BLC_TGT                     0x1003
#define BLI_EN                      0x1006
#define BLC2_TGT                    0x1007
/*  Sensor reserved */
#define DPC_CTRL                    0x1008
#define SINGLE_THR_HOT              0x100B
#define SINGLE_THR_COLD             0x100C
/* VSYNC,HSYNC and pixel shift register */
#define VSYNC_HSYNC_PIXEL_SHIFT_EN  0x1012
/* Automatic exposure gain control */
#define AE_CTRL                     0x2100
#define AE_TARGET_MEAN              0x2101
#define AE_MIN_MEAN                 0x2102
#define CONVERGE_IN_TH              0x2103
#define CONVERGE_OUT_TH             0x2104
#define MAX_INTG_H                  0x2105
#define MAX_INTG_L                  0x2106
#define MIN_INTG                    0x2107
#define MAX_AGAIN_FULL              0x2108
#define MAX_AGAIN_BIN2              0x2109
#define MIN_AGAIN                   0x210A
#define MAX_DGAIN                   0x210B
#define MIN_DGAIN                   0x210C
#define DAMPING_FACTOR              0x210D
#define FS_CTRL                     0x210E
#define FS_60HZ_H                   0x210F
#define FS_60HZ_L                   0x2110
#define FS_50HZ_H                   0x2111
#define FS_50HZ_L                   0x2112
#define FS_HYST_TH                  0x2113
/* Motion detection control */
#define MD_CTRL                     0x2150
#define I2C_CLEAR                   0x2153
#define WMEAN_DIFF_TH_H             0x2155
#define WMEAN_DIFF_TH_M             0x2156
#define WMEAN_DIFF_TH_L             0x2157
#define MD_THH                      0x2158
#define MD_THM1                     0x2159
#define MD_THM2                     0x215A
#define MD_THL                      0x215B
#define STATISTIC_CTRL              0x2000
#define MD_LROI_X_START_H           0x2011
#define MD_LROI_X_START_L           0x2012
#define MD_LROI_Y_START_H           0x2013
#define MD_LROI_Y_START_L           0x2014
#define MD_LROI_X_END_H             0x2015
#define MD_LROI_X_END_L             0x2016
#define MD_LROI_Y_END_H             0x2017
#define MD_LROI_Y_END_L             0x2018
#define MD_INTERRUPT                0x2160
/*  Sensor timing control */
#define QVGA_WIN_EN                 0x3010
#define SIX_BIT_MODE_EN             0x3011
#define PMU_AUTOSLEEP_FRAMECNT      0x3020
#define ADVANCE_VSYNC               0x3022
#define ADVANCE_HSYNC               0x3023
#define EARLY_GAIN                  0x3035
/*  IO and clock control */
#define BIT_CONTROL                 0x3059
#define OSC_CLK_DIV                 0x3060
#define ANA_Register_11             0x3061
#define IO_DRIVE_STR                0x3062
#define IO_DRIVE_STR2               0x3063
#define ANA_Register_14             0x3064
#define OUTPUT_PIN_STATUS_CONTROL   0x3065
#define ANA_Register_17             0x3067
#define PCLK_POLARITY               0x3068
/*
 * Useful value of Himax registers
 */
#define HIMAX_RESET                 0x01
#define PCLK_RISING_EDGE            0x00
#define PCLK_FALLING_EDGE           0x01
#define AE_CTRL_ENABLE              0x00
#define AE_CTRL_DISABLE             0x01

#define HIMAX_LINE_LEN_PCK_FULL     0x178
#define HIMAX_FRAME_LENGTH_FULL     0x109

#define HIMAX_LINE_LEN_PCK_FULL     0x178
#define HIMAX_FRAME_LENGTH_FULL     0x109

#define HIMAX_LINE_LEN_PCK_QVGA     0x178
#define HIMAX_FRAME_LENGTH_QVGA     0x104

#define HIMAX_LINE_LEN_PCK_QQVGA    0x178
#define HIMAX_FRAME_LENGTH_QQVGA    0x084

static const struct video_reg16 default_regs[] = {
    {BLC_TGT,               0x08},          /*  BLC target :8  at 8 bit mode */
    {BLC2_TGT,              0x08},          /*  BLI target :8  at 8 bit mode */
    {0x3044,                0x0A},          /*  Increase CDS time for settling */
    {0x3045,                0x00},          /*  Make symetric for cds_tg and rst_tg */
    {0x3047,                0x0A},          /*  Increase CDS time for settling */
    {0x3050,                0xC0},          /*  Make negative offset up to 4x */
    {0x3051,                0x42},
    {0x3052,                0x50},
    {0x3053,                0x00},
    {0x3054,                0x03},          /*  tuning sf sig clamping as lowest */
    {0x3055,                0xF7},          /*  tuning dsun */
    {0x3056,                0xF8},          /*  increase adc nonoverlap clk */
    {0x3057,                0x29},          /*  increase adc pwr for missing code */
    {0x3058,                0x1F},          /*  turn on dsun */
    {0x3059,                0x1E},
    {0x3064,                0x00},
    {0x3065,                0x04},          /*  pad pull 0 */
    {ANA_Register_17,       0x00},          /*  Disable internal oscillator */

    {BLC_CFG,               0x43},          /*  BLC_on, IIR */

    {0x1001,                0x43},          /*  BLC dithering en */
    {0x1002,                0x43},          /*  blc_darkpixel_thd */
    {0x0350,                0x7F},          /*  Dgain Control */
    {BLI_EN,                0x01},          /*  BLI enable */
    {0x1003,                0x00},          /*  BLI Target [Def: 0x20] */

    {DPC_CTRL,              0x01},          /*  DPC option 0: DPC off   1 : mono   3 : bayer1   5 : bayer2 */
    {0x1009,                0xA0},          /*  cluster hot pixel th */
    {0x100A,                0x60},          /*  cluster cold pixel th */
    {SINGLE_THR_HOT,        0x90},          /*  single hot pixel th */
    {SINGLE_THR_COLD,       0x40},          /*  single cold pixel th */
    {0x1012,                0x00},          /*  Sync. shift disable */
    {STATISTIC_CTRL,        0x07},          /*  AE stat en | MD LROI stat en | magic */
    {0x2003,                0x00},
    {0x2004,                0x1C},
    {0x2007,                0x00},
    {0x2008,                0x58},
    {0x200B,                0x00},
    {0x200C,                0x7A},
    {0x200F,                0x00},
    {0x2010,                0xB8},
    {0x2013,                0x00},
    {0x2014,                0x58},
    {0x2017,                0x00},
    {0x2018,                0x9B},

    {AE_CTRL,               0x01},          /*Automatic Exposure */
    {AE_TARGET_MEAN,        0x64},          /*AE target mean          [Def: 0x3C] */
    {AE_MIN_MEAN,           0x0A},          /*AE min target mean      [Def: 0x0A] */
    {CONVERGE_IN_TH,        0x03},          /*Converge in threshold   [Def: 0x03] */
    {CONVERGE_OUT_TH,       0x05},          /*Converge out threshold  [Def: 0x05] */
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QVGA-2)>>8},          /*Maximum INTG High Byte  [Def: 0x01] */
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QVGA-2)&0xFF},        /*Maximum INTG Low Byte   [Def: 0x54] */
    {MAX_AGAIN_FULL,        0x04},          /*Maximum Analog gain in full frame mode [Def: 0x03] */
    {MAX_AGAIN_BIN2,        0x04},          /*Maximum Analog gain in bin2 mode       [Def: 0x04] */
    {MAX_DGAIN,             0xC0},

    {INTEGRATION_H,         0x01},          /*Integration H           [Def: 0x01] */
    {INTEGRATION_L,         0x08},          /*Integration L           [Def: 0x08] */
    {ANALOG_GAIN,           0x00},          /*Analog Global Gain      [Def: 0x00] */
    {DAMPING_FACTOR,        0x20},          /*Damping Factor          [Def: 0x20] */
    {DIGITAL_GAIN_H,        0x01},          /*Digital Gain High       [Def: 0x01] */
    {DIGITAL_GAIN_L,        0x00},          /*Digital Gain Low        [Def: 0x00] */

    {FS_CTRL,               0x00},          /*Flicker Control */

    {FS_60HZ_H,             0x00},
    {FS_60HZ_L,             0x3C},
    {FS_50HZ_H,             0x00},
    {FS_50HZ_L,             0x32},

    {MD_CTRL,               0x00},
    {FRAME_LEN_LINES_H,     HIMAX_FRAME_LENGTH_QVGA>>8},
    {FRAME_LEN_LINES_L,     HIMAX_FRAME_LENGTH_QVGA&0xFF},
    {LINE_LEN_PCK_H,        HIMAX_LINE_LEN_PCK_QVGA>>8},
    {LINE_LEN_PCK_L,        HIMAX_LINE_LEN_PCK_QVGA&0xFF},
    {QVGA_WIN_EN,           0x01},          /* Enable QVGA window readout */
    {0x0383,                0x01},
    {0x0387,                0x01},
    {0x0390,                0x00},
    {0x3011,                0x70},
    {0x3059,                0x02},
    {OSC_CLK_DIV,           0x0B},
    {IMG_ORIENTATION,       0x00},          /* change the orientation */
    {0x0104,                0x01},
    {0x0000,                0x00},  /* EOF */
};

static const struct video_reg16 himax_qvga_regs[] = {
    {0x0383,                0x01},
    {0x0387,                0x01},
    {0x0390,                0x00},
    {QVGA_WIN_EN,           0x01},// Enable QVGA window readout
    {MAX_INTG_H,            (HIMAX_FRAME_LENGTH_QVGA-2)>>8},
    {MAX_INTG_L,            (HIMAX_FRAME_LENGTH_QVGA-2)&0xFF},
    {FRAME_LEN_LINES_H,     (HIMAX_FRAME_LENGTH_QVGA>>8)},
    {FRAME_LEN_LINES_L,     (HIMAX_FRAME_LENGTH_QVGA&0xFF)},
    {LINE_LEN_PCK_H,        (HIMAX_LINE_LEN_PCK_QVGA>>8)},
    {LINE_LEN_PCK_L,        (HIMAX_LINE_LEN_PCK_QVGA&0xFF)},
    //{BIT_CONTROL, 0x02},
    //{GRP_PARAM_HOLD,        0x01},
    {0x0000,                0x00},  // EOF

};


#endif

struct video_reg hm01b0_160x120_regs[] = {
	{HM01B0_CCI_WIDTH, 0x3},
	{HM01B0_CCI_HEIGHT, 0x3},
	{HM01B0_CCI_BINNING_MODE, 0x3},
	{HM01B0_CCI_QVGA_WIN_EN, 0x1},
	{HM01B0_CCI_FRAME_LENGTH_LINES, 0x80},
	{HM01B0_CCI_LINE_LENGTH_PCLK, 0xD7},
};

struct video_reg hm01b0_320x240_regs[] = {
	{HM01B0_CCI_WIDTH, 0x1},
	{HM01B0_CCI_HEIGHT, 0x1},
	{HM01B0_CCI_BINNING_MODE, 0x0},
	{HM01B0_CCI_QVGA_WIN_EN, 0x1},
	{HM01B0_CCI_FRAME_LENGTH_LINES, 0x104},
	{HM01B0_CCI_LINE_LENGTH_PCLK, 0x178},
};

struct video_reg hm01b0_320x320_regs[] = {
	{HM01B0_CCI_WIDTH, 0x1},
	{HM01B0_CCI_HEIGHT, 0x1},
	{HM01B0_CCI_BINNING_MODE, 0x0},
	{HM01B0_CCI_QVGA_WIN_EN, 0x0},
	{HM01B0_CCI_FRAME_LENGTH_LINES, 0x158},
	{HM01B0_CCI_LINE_LENGTH_PCLK, 0x178},
};

struct video_reg *hm01b0_init_regs[] = {
	[RESOLUTION_160x120] = hm01b0_160x120_regs,
	[RESOLUTION_320x240] = hm01b0_320x240_regs,
	[RESOLUTION_320x320] = hm01b0_320x320_regs,
	[RESOLUTION_326x244] = hm01b0_320x240_regs,
	[RESOLUTION_160x120_44] = hm01b0_160x120_regs,
	[RESOLUTION_326x244_44] = hm01b0_320x240_regs,
	[RESOLUTION_320x320_44] = hm01b0_320x320_regs,
};

struct hm01b0_data {
	struct video_format fmt;
	uint8_t ctrl_val;
};

struct hm01b0_config {
	const struct i2c_dt_spec i2c;
	const uint8_t data_bits;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	struct gpio_dt_spec reset;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios)
	struct gpio_dt_spec pwdn;
#endif
};

#define HM01B0_VIDEO_FORMAT_CAP(width, height, format)                                             \
	{                                                                                          \
		.pixelformat = (format),                                                           \
		.width_min = (width),                                                              \
		.width_max = (width),                                                              \
		.height_min = (height),                                                            \
		.height_max = (height),                                                            \
		.width_step = 0,                                                                   \
		.height_step = 0,                                                                  \
	}

static const struct video_format_cap hm01b0_fmts[] = {
	HM01B0_VIDEO_FORMAT_CAP(160, 120, VIDEO_PIX_FMT_GREY),
	HM01B0_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_GREY),
	HM01B0_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_GREY),
	HM01B0_VIDEO_FORMAT_CAP(326, 244, VIDEO_PIX_FMT_GREY),
	HM01B0_VIDEO_FORMAT_CAP(160, 120, VIDEO_PIX_FMT_GREY_44),
	HM01B0_VIDEO_FORMAT_CAP(326, 244, VIDEO_PIX_FMT_GREY_44),
	HM01B0_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_GREY_44),
	{0},
};

static int hm01b0_apply_configuration(const struct device *dev, enum hm01b0_resolution resolution)
{
	struct hm01b0_data *data = dev->data;
	const struct hm01b0_config *config = dev->config;
	int ret;

	//printk("hm01b0_apply_configuration %u reset %p %u pwndn:%p %u\n", resolution, config->reset.port, config->reset.pin, 
	//	config->pwdn.port, config->pwdn.pin);
	printk("hm01b0_apply_configuration %u reset %p %u\n", resolution, config->reset.port, config->reset.pin );
#ifdef KURTE_HACKING
	if (resolution == RESOLUTION_320x240) {
		ret = video_write_cci_multiregs16(&config->i2c, himax_qvga_regs, ARRAY_SIZE(himax_qvga_regs));
		if (ret != 0) {
			LOG_ERR("Error writing default registers (%d)", ret);
			return ret;
		}
	}

	/* REG_BIT_CONTROL */
	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_BIT_CONTROL, data->ctrl_val);
	if (ret != 0) {
		LOG_ERR("Failed to write BIT_CONTROL reg (%d)", ret);
		return ret;
	}

#else	
	/* Number of registers is the same for all configuration */
	ret = video_write_cci_multiregs(&config->i2c, hm01b0_init_regs[resolution],
					ARRAY_SIZE(hm01b0_160x120_regs));
	if (ret != 0) {
		LOG_ERR("Failed to write config list registers (%d)", ret);
		return ret;
	}

	/* REG_BIT_CONTROL */
	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_BIT_CONTROL, data->ctrl_val);
	if (ret != 0) {
		LOG_ERR("Failed to write BIT_CONTROL reg (%d)", ret);
		return ret;
	}
	/* OSC_CLK_DIV */
	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_OSC_CLOCK_DIV, 0x08);
	if (ret != 0) {
		LOG_ERR("Failed to write OSC_CLK_DIV reg (%d)", ret);
		return ret;
	}
	/* INTEGRATION_H */
	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_INTEGRATION_H,
				  hm01b0_init_regs[resolution][5].data / 2);
	if (ret != 0) {
		LOG_ERR("Failed to write INTEGRATION_H reg (%d)", ret);
		return ret;
	}
#endif	
	/* GRP_PARAM_HOLD */
	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_GRP_PARAM_HOLD, 0x01);
	if (ret != 0) {
		LOG_ERR("Failed to write GRP_PARAM_HOLD reg (%d)", ret);
		return ret;
	}
	return ret;
}

static int hm01b0_get_caps(const struct device *dev, struct video_caps *caps)
{
	caps->min_vbuf_count = 0;
	caps->min_line_count = LINE_COUNT_HEIGHT;
	caps->max_line_count = LINE_COUNT_HEIGHT;
	caps->format_caps = hm01b0_fmts;
	return 0;
}

static int hm01b0_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm01b0_data *data = dev->data;
	size_t idx;
	int ret;

	LOG_DBG("HM01B0 set_fmt: %d x %d, fmt: %s", fmt->width, fmt->height,
		VIDEO_FOURCC_TO_STR(fmt->pixelformat));

	ret = video_format_caps_index(hm01b0_fmts, fmt, &idx);
	if (ret != 0) {
		LOG_ERR("Image resolution not supported\n");
		return ret;
	}

	if (!memcmp(&data->fmt, fmt, sizeof(data->fmt))) {
		return 0;
	}

	/* Check if camera is capable of handling given format */
	ret = hm01b0_apply_configuration(dev, (enum hm01b0_resolution)idx);
	if (ret != 0) {
		/* Camera is not capable of handling given format */
		LOG_ERR("Image resolution not supported");
		return ret;
	}
	data->fmt = *fmt;
	return ret;
}

static int hm01b0_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm01b0_data *data = dev->data;

	*fmt = data->fmt;
	LOG_DBG("HM01B0 get_fmt: %d x %d, fmt: %s", fmt->width, fmt->height,
		VIDEO_FOURCC_TO_STR(fmt->pixelformat));
	return 0;
}

static int hm01b0_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	const struct hm01b0_config *config = dev->config;

	/* SET MODE_SELECT */
	return video_write_cci_reg(&config->i2c, HM01B0_CCI_STS, enable ? 1 : 0);
}

static int hm01b0_soft_reset(const struct device *dev)
{
	const struct hm01b0_config *config = dev->config;
	uint32_t val = 0xff;
	int ret;

	ret = video_write_cci_reg(&config->i2c, HM01B0_CCI_RESET, 0x01);
	if (ret != 0) {
		LOG_ERR("Error writing HM01B0_CCI_RESET (%d)", ret);
		return ret;
	}

	for (int retries = 0; retries < 10; retries++) {
		ret = video_read_cci_reg(&config->i2c, HM01B0_CCI_STS, &val);
		if (ret != 0 || val == 0x0) {
			break;
		}
		k_msleep(100);
	}
	if (ret != 0) {
		LOG_ERR("Soft reset error (%d)", ret);
	}
	return ret;
}

static DEVICE_API(video, hm01b0_driver_api) = {
	.set_format = hm01b0_set_fmt,
	.get_format = hm01b0_get_fmt,
	.set_stream = hm01b0_set_stream,
	.get_caps = hm01b0_get_caps,
};

static bool hm01b0_check_connection(const struct device *dev)
{
	const struct hm01b0_config *config = dev->config;
	uint32_t model_id;
	int ret;

	ret = video_read_cci_reg(&config->i2c, HM01B0_CCI_ID, &model_id);
	if (ret != 0) {
		LOG_ERR("Error reading id reg (%d)", ret);
		return false;
	}
	return (model_id == HM01B0_ID);
}



#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios) || DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
static bool hm01b0_try_reset_pwdn_pins(const struct device *dev, uint8_t iter) {
	const struct hm01b0_config *config = dev->config;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios)
  gpio_pin_set_dt(&config->pwdn, iter >> 1);
#else
  if (iter >> 1) return false; /* cut iterations in half if not defined */
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	gpio_pin_set_dt(&config->reset, iter & 1);
#else
  if (iter & 1) return false; /* cut iterations in half if not defined */
#endif
    // lets try a couple of iterations before we punt
	uint8_t retry_count = 3;
	while (retry_count) {
		k_sleep(K_MSEC(10));
		if (hm01b0_check_connection(dev)) {
			return true;
		}
		retry_count--;
	}
	return false;
}
#endif

static int hm01b0_init(const struct device *dev)
{
	struct hm01b0_data *data = dev->data;
	const struct hm01b0_config *config = dev->config;
	int ret;
	LOG_INF("hm01b0_init called");

	if (config->data_bits == 8) {
		data->ctrl_val = 0x02;
	} else if (config->data_bits == 4) {
		data->ctrl_val = 0x42;
	} else if (config->data_bits == 1) {
		data->ctrl_val = 0x22;
	} else {
		LOG_ERR("Invalid data bits!");
		return -ENODEV;
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios) || DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios)
    /* Power up camera module */
    if (config->pwdn.port != NULL) {
        if (!gpio_is_ready_dt(&config->pwdn)) {
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->pwdn, GPIO_OUTPUT_ACTIVE);
            if (ret < 0) {
            LOG_ERR("Could not clear power down pin: %d", ret);
            return ret;
        }
    }
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
    /* Reset camera module */
    if (config->reset.port != NULL) {
        if (!gpio_is_ready_dt(&config->reset)) {
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Could not set reset pin: %d", ret);
            return ret;
        }
    }
#endif
    bool found_id = false;
    for (uint8_t iter = 0; iter < 4; iter++) {
    	found_id = hm01b0_try_reset_pwdn_pins(dev, iter);
    	if (found_id) break;
    }
#endif


	LOG_INF("hm01b0_init check connection");
	if (!hm01b0_check_connection(dev)) {
		LOG_ERR("%s is not ready", dev->name);
		return -ENODEV;
	}

	ret = hm01b0_soft_reset(dev);
	if (ret != 0) {
		LOG_ERR("error sof reset (%d)", ret);
		return ret;
	}

#ifdef KURTE_HACKING
	ret = video_write_cci_multiregs16(&config->i2c, default_regs, ARRAY_SIZE(default_regs));
	if (ret != 0) {
		LOG_ERR("Error writing default registers (%d)", ret);
		return ret;
	}

	video_write_cci_reg(&config->i2c, PCLK_POLARITY, (0x20 | PCLK_FALLING_EDGE));

#endif

	uint32_t pixelformat = (config->data_bits == 4)? VIDEO_PIX_FMT_GREY : VIDEO_PIX_FMT_GREY;
	struct video_format fmt = {
		.pixelformat = pixelformat,
		.width = 320,
		.height = 240,
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pitch = 160 * video_bits_per_pixel(pixelformat) / BITS_PER_BYTE,
	};

	ret = hm01b0_set_fmt(dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Error setting video format (%d)", ret);
		return ret;
	}
	return 0;
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define HM01B0_RESET_GPIO(inst) .reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {}),
#else
#define HM01B0_RESET_GPIO(inst)
#endif

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(pwdn_gpios)
#define HM01B0_PWDN_GPIO(inst) .pwdn = GPIO_DT_SPEC_INST_GET_OR(inst, pwdn_gpios, {}),
#else
#define HM01B0_PWDN_GPIO(inst)
#endif

#define HM01B0_INIT(inst)                                                                          \
	const struct hm01b0_config hm01b0_config_##inst = {                                        \
		.i2c = I2C_DT_SPEC_INST_GET(inst), 							\
		.data_bits = DT_INST_PROP(0, data_bits), /* Use only 1 pin for data */    \
		HM01B0_RESET_GPIO(inst)                 \
		HM01B0_PWDN_GPIO(inst)};        \
	struct hm01b0_data hm01b0_data_##inst;                                                     \
																								\
	DEVICE_DT_INST_DEFINE(inst, hm01b0_init, NULL, &hm01b0_data_##inst,                       \
			      &hm01b0_config_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,      \
			      &hm01b0_driver_api);                                                 \
	VIDEO_DEVICE_DEFINE(hm01b0_##inst, DEVICE_DT_INST_GET(inst), NULL);

DT_INST_FOREACH_STATUS_OKAY(HM01B0_INIT)
