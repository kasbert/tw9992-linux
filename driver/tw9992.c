// SPDX-License-Identifier: GPL-2.0
/*
 * tw9992.c Renesas TW9992 video decoder driver
 * Copyright (c) 2009 Intel Corporation
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Solutions Corp.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/mutex.h>
#include <linux/delay.h>

 /*
  * register offset
  */
#define TW9992_REG_PRODUCT_ID	0x00 /* Product ID Code Register */
#define TW9992_REG_PRODUCT_REV 0x01 /* Product Revision Code Register */
#define TW9992_REG_INFORM		0x02 /* Input Format */
#define TW9992_REG_STATUS1		0x03 /* Decoder Status Register I */
#define TW9992_VDLOSS 0x80
#define TW9992_REG_DLYCTR		0x04 /* Hysteresis and HSYNC Delay Control */
#define TW9992_REG_AFESEL		0x05 /* AFE Selection */
#define TW9992_REG_ACNTL		0x06 /* Analog Control Register */
#define TW9992_REG_CROP_HI		0x07 /* Cropping Register, High */
#define TW9992_REG_VDELAY_LO	0x08 /* Vertical Delay Register, Low */
#define TW9992_REG_VACTIVE_LO	0x09 /* Vertical Active Register, Low */
#define TW9992_REG_HDELAY_LO	0x0A /* Horizontal Delay Register, Low */
#define TW9992_REG_HACTIVE_LO	0x0B /* Horizontal Active Register, Low */
#define TW9992_REG_CNTRL1		0x0C /* Control Register I */
  //#define TW9992_REG_VSCALE_LO	0x0D /* Vertical Scaling Register, Low */
  //#define TW9992_REG_SCALE_HI	0x0E /* Scaling Register, High */
  //#define TW9992_REG_HSCALE_LO	0x0F /* Horizontal Scaling Register, Low */
#define TW9992_REG_BRIGHT		0x10 /* BRIGHTNESS Control Register */
#define TW9992_REG_CONTRAST	0x11 /* CONTRAST Control Register */
#define TW9992_REG_SHARPNESS	0x12 /* SHARPNESS Control Register I */
#define TW9992_REG_SAT_U		0x13 /* Chroma (U) Gain Register */
#define TW9992_REG_SAT_V		0x14 /* Chroma (V) Gain Register */
#define TW9992_REG_HUE		    0x15 /* Hue Control Register */
// 0x16
#define TW9992_REG_VPEAK1		0x17 /* Vertical Peaking Control 1 */
#define TW9992_REG_CORING		0x18 /* Coring Control Register */
#define TW9992_REG_TMUXSEL		0x19 /* Test MUX Selection */
//#define TW9992_REG_ACNTL2		0x1A /* Analog Control 2 */
//#define TW9992_REG_OUTCTR2		0x1B /* Output Control 2 */
#define TW9992_REG_SDT		    0x1C /* Standard Selection */
#define TW9992_REG_SDTR		0x1D /* Standard Recognition */
// 0x1E
#define TW9992_REG_TEST		0x1F /* Test Control Register */
#define TW9992_REG_CLMPG		0x20 /* Clamping Gain */
#define TW9992_REG_IAGC		0x21 /* Individual AGC Gain */
#define TW9992_REG_AGCGAIN		0x22 /* AGC Gain */
#define TW9992_REG_PEAKWT		0x23 /* White Peak Threshold */
#define TW9992_REG_CLMPL		0x24 /* Clamp level */
#define TW9992_REG_SYNCT		0x25 /* Sync Amplitude */
#define TW9992_REG_MISSCNT		0x26 /* Sync Miss Count Register */
#define TW9992_REG_PCLAMP		0x27 /* Clamp Position Register */
#define TW9992_REG_VCNTL1		0x28 /* Vertical Control I */
#define TW9992_REG_VCNTL2		0x29 /* Vertical Control II */
#define TW9992_REG_CKILL		0x2A /* Color Killer Level Control */
#define TW9992_REG_COMB		0x2B /* Comb Filter Control */
#define TW9992_REG_LDLY		0x2C /* Luma Delay and H Filter Control */
#define TW9992_REG_MISC1		0x2D /* Miscellaneous Control I */
#define TW9992_REG_MISC2		0x2E /* Miscellaneous Control II */
#define TW9992_REG_MISC3		0x2F /* Miscellaneous Control III */
#define TW9992_REG_SDSN		0x30 /* Standard Detection */
#define TW9992_REG_STATUS2		0x31 /* Chip STATUS II */
#define TW9992_REG_HFREF		0x32 /* H monitor */
#define TW9992_REG_CLMD		0x33 /* CLAMP MODE */
#define TW9992_REG_IDCNTL		0x34 /* ID Detection Control */
#define TW9992_REG_CLCNTL		0x35 /* Clamp Control */
#define TW9992_REG_DIFFCLCNTL	0x36 /* Differential Clamping Control */
#define TW9992_REG_DIFFCLCNTL2	0x37 /* Differential Clamping Control 2 */
#define TW9992_REG_DIFFCLCNTL3	0x38 /* Differential Clamping Control 3 */
#define TW9992_REG_DIFFCLCNTL4	0x39 /* Differential Clamping Control 4 */
#define TW9992_REG_SRTDETCTL	0x3A /* Short Detection Control */
#define TW9992_REG_SRTDETCTL1	0x3A /* Short Detection Control 1 */
#define TW9992_REG_SRTDETCTL2	0x3B /* Short Detection Control 2 */
#define TW9992_REG_DIFFCLCNTL5	0x3C /* Differential Clamping Control 5 */
#define TW9992_REG_DIFFCLCNTL6	0x3D /* Differential Clamping Control 6 */
// 0x3E
#define TW9992_REG_LINENBRINT  0x3F /* Linenumber int */
// 0x40-0x47
#define TW9992_REG_IOBUFCTL    0x48 /* IO Buffer Control */
#define TW9992_REG_DATACONV    0x49 /* Data Conversion */
#define TW9992_REG_SYNCCTL     0x4A /* Sync Control */
#define TW9992_REG_OUTPUTCTL   0x4B /* Output Control */
#define TW9992_REG_PWDN        0x4C /* Power Down Register */
#define TW9992_REG_HSOUTADJ    0x4D /* HSYNC Output Adjustment */
#define TW9992_REG_HSOUTADJ_L  0x4D /* HSYNC Output Adjustment Low */
// 0x4F
//#define TW9992_REG_FILLDATA	0x50
#define TW9992_REG_IRQ2        0x51 /* IRQ2 Register */
#define TW9992_REG_IRQ3        0x52 /* IRQ3 Register */
// 0x53
#define TW9992_REG_INTSRCSTAT2 0x54 /* Interrupt Source Status 2 */
#define TW9992_REG_INTSRCSTAT3 0x55 /* Interrupt Source Status 3 */
// 0x56
#define TW9992_REG_IRQ2EN      0x57 /* IRQ2 Enable */
#define TW9992_REG_IRQ3EN      0x58 /* IRQ3 Enable */
// 0x59-0x5F
#define TW9992_REG_GPIOEN      0x60 /* GPIO Enable */
#define TW9992_REG_GPIOOE      0x61 /* GPIO OE */
#define TW9992_REG_GPIOOD      0x62 /* GPIO OD */
#define TW9992_REG_GPIOID      0x63 /* GPIO ID */
// 0x64-0x6F
#define TW9992_REG_MIPICTL     0x70 /* MIPI Control */
#define TW9992_REG_PICWIDTH    0x71 /* Picture Width */
#define TW9992_REG_PICWIDTH_L  0x72 /* Picture Width Low */
#define TW9992_REG_PICHEIGHT   0x73 /* Picture Height */
#define TW9992_REG_PICHEIGHT_L 0x74 /* Picture Height Low */
#define TW9992_REG_BLLNBR      0x75 /* Blank line number */
#define TW9992_REG_BLLNBR_L    0x76 /* Blank line number Low */
#define TW9992_REG_FRMSTCNT    0x77 /* Frame-Start count */
#define TW9992_REG_FRMSTCNT_L  0x78 /* Frame-Start count Low */
#define TW9992_REG_LNSTCNT     0x79 /* Line Start count */
#define TW9992_REG_LNSTCNT_L   0x7A /* Line Start count Low */
#define TW9992_REG_ALNSTCNT    0x7B /* Active Line Start count */
#define TW9992_REG_ALNSTCNT_L  0x7C /* Active Line Start count Low */
#define TW9992_REG_FRMENDCNT   0x7D /* Frame-End count */
#define TW9992_REG_FRMENDCNT_L 0x7E /* Frame-End count Low */
// TODO 0x7F
#define TW9992_REG_VIRT_CH_NBR 0x7F /* VIRTUAL CHANNEL NUMBERS */
#define TW9992_REG_WORD_CNT    0x80 /* WORD_COUNT IN LONG PACKET */
#define TW9992_REG_WORD_CNT_L  0x81 /* WORD_COUNT IN LONG PACKET Low */
// TODO 0x82-0xF5

#define TW9992_REG_MIPI_AN_CTL 0xA2 /* MIPI ANALOG CTRL MISC */

#define TW9992_REG_ACACNT_L    0xC0 /* ACA Control */
#define TW9992_REG_ACAGCNT_L   0xC1 /* ACA Gain Control */


#define TW9992_STD_NTSC_M				0x0
#define TW9992_STD_PAL_BG				0x1
#define TW9992_STD_SECAM				0x2
#define TW9992_STD_NTSC_443				0x3
#define TW9992_STD_PAL_M				0x4
#define TW9992_STD_PAL_CN				0x5
#define TW9992_STD_PAL60				0x6
#define TW9992_STD_AUTOD				0x7

#define TW9992_CON_MIN		0
#define TW9992_CON_DEF		128
#define TW9992_CON_MAX		255

#define TW9992_BRI_MIN		-128
#define TW9992_BRI_DEF		0
#define TW9992_BRI_MAX		127

#define TW9992_HUE_MIN		-127
#define TW9992_HUE_DEF		0
#define TW9992_HUE_MAX		128

#define TW9992_SAT_MIN		0
#define TW9992_SAT_DEF		128
#define TW9992_SAT_MAX		255

#define TW9992_STATUS1_AUTOD_MASK	0x70
#define TW9992_STATUS1_AUTOD_NTSM_M_J	0x00
#define TW9992_STATUS1_AUTOD_PAL_B_G	0x10
#define TW9992_STATUS1_AUTOD_SECAM	0x20
#define TW9992_STATUS1_AUTOD_NTSC_4_43 0x30
#define TW9992_STATUS1_AUTOD_PAL_M	0x40
#define TW9992_STATUS1_AUTOD_PAL_CN	0x50
#define TW9992_STATUS1_AUTOD_PAL_60	0x60
#define TW9992_STATUS1_AUTOD_NA	0x70


#define TW9992_INPUT_CVBS_AIN1 0x00
#define TW9992_INPUT_CVBS_AIN2 0x01
#define TW9992_INPUT_CVBS_AIN3 0x02
#define TW9992_INPUT_CVBS_AIN4 0x03
#define TW9992_INPUT_CVBS_AIN5 0x04
#define TW9992_INPUT_CVBS_AIN6 0x05
#define TW9992_INPUT_SVIDEO_AIN1_AIN2 0x06
#define TW9992_INPUT_SVIDEO_AIN3_AIN4 0x07
#define TW9992_INPUT_SVIDEO_AIN5_AIN6 0x08
#define TW9992_INPUT_YPRPB_AIN1_AIN2_AIN3 0x09
#define TW9992_INPUT_YPRPB_AIN4_AIN5_AIN6 0x0a

#define TW9992_INPUT_CVBS_AIN1 0x00
#define TW9992_INPUT_CVBS_AIN2 0x01
#define TW9992_INPUT_CVBS_AIN3 0x02
#define TW9992_INPUT_CVBS_AIN4 0x03
#define TW9992_INPUT_CVBS_AIN5 0x04
#define TW9992_INPUT_CVBS_AIN6 0x05
#define TW9992_INPUT_CVBS_AIN7 0x06
#define TW9992_INPUT_CVBS_AIN8 0x07
#define TW9992_INPUT_DIFF_CVBS_AIN1_AIN2 0x08
#define TW9992_INPUT_DIFF_CVBS_AIN3_AIN4 0x09
#define TW9992_INPUT_DIFF_CVBS_AIN5_AIN6 0x0a
#define TW9992_INPUT_DIFF_CVBS_AIN7_AIN8 0x0b


#define TW9992_MAX_WIDTH 720
#define TW9992_MAX_HEIGHT 576 // I don't know

#define TW9992_HDELAY 0x28
#define TW9992_VDELAY 0x0e

#define TW9992_CROP_TOP 0
#define TW9992_CROP_LEFT 0
#define TW9992_CROP_WIDTH 672 // Must be dividable by 16
#define TW9992_CROP_HEIGHT 576

#if 0
// Stolen from other old tw9992 driver
#define TW9_PDN_PORT 4
struct reg_value {
    u8 reg;
    u8 value;
};
static struct reg_value tw9992_init_default[] = {
    {0x02,0x40},

    // TODO check if these make any sense

    {0x04,0x00},{0x05,0x09},{0x06,0x00},{0x07,0x02},{0x08,0x12},
    {0x09,0xF0},{0x0A,0x09},{0x0B,0xD1},{0x0C,0xCC},{0x0D,0x00},
    {0x10,0x00},{0x11,0x64},{0x12,0x11},{0x13,0x80},{0x14,0x80},
    {0x15,0x00},{0x17,0x80},{0x18,0x44},{0x19,0x06},{0x1A,0x00},
    {0x1C,0x0F},{0x1D,0x7F},{0x1F,0x00},{0x20,0x50},{0x21,0x22},{0x22,0xF0},
    {0x23,0xD8},{0x24,0xBC},{0x25,0xB8},{0x26,0x44},{0x27,0x38},
    {0x28,0x00},{0x29,0x00},{0x2A,0x78},{0x2B,0x44},{0x2C,0x30},
    {0x2D,0x14},{0x2E,0xA5},{0x2F,0xE0},
    {0x33,0x05},{0x34,0x1A},{0x35,0x00},{0x36,0x5A},{0x37,0x18},{0x38,0xDD},
    {0x39,0x00},{0x3A,0x30},{0x3B,0x00},{0x3C,0x00},{0x3D,0x00},
    {0x3F,0x1A},{0x40,0x80},{0x41,0x00},{0x42,0x00},
    {0x48,0x02},{0x49,0x00},{0x4A,0x81},{0x4B,0x0A},{0x4C,0x00},{0x4D,0x01},
    {0x4E,0x01},
    {0x50,0x00},{0x51,0x00},{0x52,0x00},
    {0x56,0x00},{0x57,0x00},{0x58,0x00},
    {0x60,0x00},{0x61,0x00},{0x62,0x00},
    {0x70,0x01},{0x71,0xA5},{0x72,0xA0},{0x73,0x00},{0x74,0xF0},{0x75,0x00},
    {0x76,0x17},{0x77,0x05},{0x78,0x88},{0x79,0x06},{0x7A,0x28},
    {0x7B,0x46},{0x7C,0xB3},{0x7D,0x06},{0x7E,0x13},{0x7F,0x11},
    {0x80,0x05},{0x81,0xA0},{0x82,0x13},{0x83,0x11},{0x84,0x02},
    {0x85,0x0E},{0x86,0x08},{0x87,0x37},{0x88,0x00},{0x89,0x00},
    {0x8A,0x02},{0x8B,0x33},{0x8C,0x22},{0x8D,0x03},{0x8E,0x22},
    {0x8F,0x01},
    {0x90,0x00},{0x91,0x0C},{0x92,0x00},{0x93,0x0E},{0x94,0x07},{0x95,0xFF},
    {0x96,0x1A},
    {0x9B,0x02},
    {0xA0,0x00},{0xA1,0x00},{0xA2,0x30},{0xA3,0xC0},{0xA4,0x00},
    {0xC0,0x06},{0xC1,0x20},
};

// TODO check if these make any sense
struct reg_value tw9992_Decoder_NTSC[] = {
    //vdelay=0x015=21	vactive=0x0f0=240
    //hdelay=0x014=20	hactive=0x2d0=720
    {0x07,0x02},{0x08,0x15},{0x09,0xf0},{0x0a,0x14},{0x0b,0xd0}
};

struct reg_value tw9992_Decoder_PAL[] = {
    //vdelay=0x017=23	vactive=0x120=288
    //hdelay=0x00f=15	hactive=0x2d0=720
    {0x07,0x12},{0x08,0x17},{0x09,0x20},{0x0a,0x0f},{0x0b,0xd0}

};
#endif

/* Initial number of frames to skip to avoid possible garbage */
#define TW9992_NUM_OF_SKIP_FRAMES       2

static int dbg_input;
module_param(dbg_input, int, 0644);
MODULE_PARM_DESC(dbg_input, "Input number (0-31)");

static const struct v4l2_mbus_framefmt tw9992_csi2_default_fmt = {
    .code = MEDIA_BUS_FMT_UYVY8_1X16,
    .width = TW9992_CROP_WIDTH,
    .height = TW9992_CROP_HEIGHT,
    .colorspace = V4L2_COLORSPACE_SMPTE170M, //V4L2_COLORSPACE_SRGB,
    .ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
    .quantization = V4L2_QUANTIZATION_FULL_RANGE,
    .xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
    .field = V4L2_FIELD_ALTERNATE,
};

static const struct v4l2_mbus_framefmt tw9992_csi2_rgb_fmt = {
    .code = MEDIA_BUS_FMT_RGB565_2X8_LE, // MEDIA_BUS_FMT_RGB565_1X16 does not work with unicam
    .width = TW9992_CROP_WIDTH,
    .height = TW9992_CROP_HEIGHT,
    .colorspace = V4L2_COLORSPACE_SRGB,
    .ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB),
    .quantization = V4L2_QUANTIZATION_FULL_RANGE,
    .xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SRGB),
    .field = V4L2_FIELD_ALTERNATE,
};

static const u32 mbus_codes[] = {
    tw9992_csi2_default_fmt.code,
    tw9992_csi2_rgb_fmt.code,
};
static const unsigned int nmbus_codes = ARRAY_SIZE(mbus_codes);

static unsigned int tw9992_valid_input_mask =
BIT(TW9992_INPUT_CVBS_AIN1) |
BIT(TW9992_INPUT_CVBS_AIN2) |
BIT(TW9992_INPUT_CVBS_AIN3) |
BIT(TW9992_INPUT_CVBS_AIN4) |
BIT(TW9992_INPUT_CVBS_AIN5) |
BIT(TW9992_INPUT_CVBS_AIN6) |
BIT(TW9992_INPUT_CVBS_AIN7) |
BIT(TW9992_INPUT_CVBS_AIN8) |
BIT(TW9992_INPUT_DIFF_CVBS_AIN1_AIN2) |
BIT(TW9992_INPUT_DIFF_CVBS_AIN3_AIN4) |
BIT(TW9992_INPUT_DIFF_CVBS_AIN5_AIN6) |
BIT(TW9992_INPUT_DIFF_CVBS_AIN7_AIN8);

struct tw9992_state {
    struct v4l2_ctrl_handler ctrl_hdl;
    struct v4l2_subdev	sd;
    struct media_pad	pad;
    struct mutex		mutex; /* mutual excl. when accessing chip */
    struct gpio_desc* pwdn_gpio;
    struct gpio_desc* rst_gpio;
    v4l2_std_id		curr_norm;
    struct v4l2_mbus_framefmt fmt;
    bool			powered;
    bool			streaming;
    u8			input;
    struct i2c_client* client;
    u8 hack_addr;
};

#define to_tw9992_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct tw9992_state,	\
					    ctrl_hdl)->sd)

static int tw9992_select_input(struct tw9992_state* state, unsigned int input);

static int tw9992_write(struct tw9992_state* state, unsigned int reg,
    unsigned int value) {
    if (!state->client) return -EINVAL;
    lockdep_assert_held(&state->mutex);
    return i2c_smbus_write_byte_data(state->client, reg & 0xff, value);
}

static int tw9992_read(struct tw9992_state* state, unsigned int reg) {
    if (!state->client) return -EINVAL;
    lockdep_assert_held(&state->mutex);
    return i2c_smbus_read_byte_data(state->client, reg & 0xff);
}

static v4l2_std_id tw9992_std_to_v4l2(u8 status1, u8 status2) {
    /* in case V4L2_IN_ST_NO_SIGNAL */
    if (status1 & TW9992_VDLOSS)
        return V4L2_STD_UNKNOWN;
    if (status2 & 0x80) // DETSTUS Detection in progress
        return V4L2_STD_UNKNOWN;
    switch (status1 & TW9992_STATUS1_AUTOD_MASK) {
    case TW9992_STATUS1_AUTOD_NTSM_M_J:
        return V4L2_STD_NTSC;
    case TW9992_STATUS1_AUTOD_NTSC_4_43:
        return V4L2_STD_NTSC_443;
    case TW9992_STATUS1_AUTOD_PAL_M:
        return V4L2_STD_PAL_M;
    case TW9992_STATUS1_AUTOD_PAL_60:
        return V4L2_STD_PAL_60;
    case TW9992_STATUS1_AUTOD_PAL_B_G:
        return V4L2_STD_PAL;
    case TW9992_STATUS1_AUTOD_SECAM:
        return V4L2_STD_SECAM;
    case TW9992_STATUS1_AUTOD_PAL_CN: // FIXME
        return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
    default:
        return V4L2_STD_UNKNOWN;
    }
    return V4L2_STD_UNKNOWN;
}

static int v4l2_std_to_tw9992(v4l2_std_id std) {
    if (std == V4L2_STD_PAL_60)
        return TW9992_STD_PAL60;
    if (std == V4L2_STD_NTSC_443)
        return TW9992_STD_NTSC_443;
    if (std == V4L2_STD_PAL_M)
        return TW9992_STD_PAL_M;
    if (std == V4L2_STD_PAL_Nc)
        return TW9992_STD_PAL_CN;

    if (std & V4L2_STD_PAL)
        return TW9992_STD_PAL_BG;
    if (std & V4L2_STD_NTSC)
        return TW9992_STD_NTSC_M;
    if (std & V4L2_STD_SECAM)
        return TW9992_STD_SECAM;
    if (std == 0)
        return TW9992_STD_AUTOD;

    return -EINVAL;
}

static u32 tw9992_status_to_v4l2(u8 status1) {
    if (status1 & TW9992_VDLOSS)
        return V4L2_IN_ST_NO_SIGNAL;

    return 0;
}

static inline struct tw9992_state* to_state(struct v4l2_subdev* sd) {
    return container_of(sd, struct tw9992_state, sd);
}

static int tw9992_querystd(struct v4l2_subdev* sd, v4l2_std_id* std) {
    struct tw9992_state* state = to_state(sd);
    int err = mutex_lock_interruptible(&state->mutex);
    if (err)
        return err;

    if (state->streaming) {
        err = -EBUSY;
        goto unlock;
    }

    err = tw9992_write(state, TW9992_REG_SDT, TW9992_STD_AUTOD);
    if (err)
        goto unlock;

    msleep(100);

    int status1 = tw9992_read(state, TW9992_REG_STATUS1);
    int status2 = tw9992_read(state, TW9992_REG_SDT);
    if (status1 < 0)
        goto unlock;
    if (status2 < 0)
        goto unlock;
    *std = tw9992_std_to_v4l2(status1, status2);
    printk("__tw9992_status 0x%02x 0x%02x 0x%02llx\n", status1, status2, *std);

    err = v4l2_std_to_tw9992(state->curr_norm);
    if (err < 0)
        goto unlock;

    err = tw9992_write(state, TW9992_REG_SDT, (err & 0x07));

unlock:
    mutex_unlock(&state->mutex);
    return err;
}

static int tw9992_s_routing(struct v4l2_subdev* sd, u32 input,
    u32 output, u32 config) {
    struct tw9992_state* state = to_state(sd);
    int ret = mutex_lock_interruptible(&state->mutex);

    if (ret)
        return ret;

    if (input > 31 || !(BIT(input) & tw9992_valid_input_mask)) {
        ret = -EINVAL;
        goto out;
    }

    ret = tw9992_select_input(state, input);

    if (ret == 0)
        state->input = input;
out:
    mutex_unlock(&state->mutex);
    return ret;
}

static void tw9992_check_input(struct v4l2_subdev* sd) {
    struct tw9992_state* state = to_state(sd);

    if (state->input != dbg_input)
        if (tw9992_s_routing(sd, dbg_input, 0, 0))
            /* Failed - reset dbg_input */
            dbg_input = state->input;
}

static int tw9992_g_input_status(struct v4l2_subdev* sd, u32* status) {
    struct tw9992_state* state = to_state(sd);
    int ret;

    tw9992_check_input(sd);

    ret = mutex_lock_interruptible(&state->mutex);
    if (ret)
        return ret;

    int status1 = tw9992_read(state, TW9992_REG_STATUS1);
    if (status1 < 0)
        goto unlock;
    printk("__tw9992_status 0x%02x\n", status1);
    *status = tw9992_status_to_v4l2(status1);
unlock:
    mutex_unlock(&state->mutex);
    return ret;
}

static int tw9992_program_std(struct tw9992_state* state) {
    int ret;

    ret = v4l2_std_to_tw9992(state->curr_norm);
    if (ret < 0)
        return ret;

    return tw9992_write(state, TW9992_REG_SDT, (ret & 0x07));
}

static int tw9992_s_std(struct v4l2_subdev* sd, v4l2_std_id std) {
    struct tw9992_state* state = to_state(sd);
    int ret;

    tw9992_check_input(sd);

    ret = mutex_lock_interruptible(&state->mutex);

    if (ret)
        return ret;

    /* Make sure we can support this std */
    ret = v4l2_std_to_tw9992(std);
    if (ret < 0)
        goto out;

    state->curr_norm = std;

    ret = tw9992_program_std(state);
out:
    mutex_unlock(&state->mutex);
    return ret;
}

static int tw9992_g_std(struct v4l2_subdev* sd, v4l2_std_id* norm) {
    struct tw9992_state* state = to_state(sd);

    tw9992_check_input(sd);

    *norm = state->curr_norm;

    return 0;
}

static int tw9992_g_frame_interval(struct v4l2_subdev* sd,
    struct v4l2_subdev_frame_interval* fi) {
    struct tw9992_state* state = to_state(sd);

    if (state->curr_norm & V4L2_STD_525_60) {
        fi->interval.numerator = 1001;
        fi->interval.denominator = 30000;
    } else {
        fi->interval.numerator = 1;
        fi->interval.denominator = 25;
    }

    return 0;
}

static void tw9992_set_power_pin(struct tw9992_state* state, bool on) {
    if (!state->pwdn_gpio)
        return;

    if (on) {
        gpiod_set_value_cansleep(state->pwdn_gpio, 0);
        usleep_range(5000, 10000);
    } else {
        gpiod_set_value_cansleep(state->pwdn_gpio, 1);
    }
}

static void tw9992_set_reset_pin(struct tw9992_state* state, bool on) {
    if (!state->rst_gpio)
        return;

    if (on) {
        gpiod_set_value_cansleep(state->rst_gpio, 1);
    } else {
        gpiod_set_value_cansleep(state->rst_gpio, 0);
        usleep_range(5000, 10000);
    }
}

static int tw9992_set_power(struct tw9992_state* state, bool on) {

    if (on) {
        tw9992_write(state, TW9992_REG_MIPICTL, (state->curr_norm & V4L2_STD_PAL) ? 0x11 : 0x01);
        tw9992_write(state, TW9992_REG_MIPI_AN_CTL, 0x30);
    } else {
        tw9992_write(state, TW9992_REG_MIPICTL, (state->curr_norm & V4L2_STD_PAL) ? 0x91 : 0x81);
        tw9992_write(state, TW9992_REG_MIPI_AN_CTL, 0x0f);
    }
#if 0
    TODO powerdown the device
        u8 val;
    int ret;

    0X3D - DIFFERENTIAL CLAMPING CONTROL 6
        ret = tw9992_write(state, 0x3d, on ? 0x00 : 0x07);
    0X4C - POWER - DOWN REGISTER
        ret = tw9992_write(state, 0x4c, on ? 0x00 : 0x0C);
    0X70 - MIPI CONTROL, VIDEO INPUT FORMAT AND NUMBER OF DATA CHANNELS
        ret = tw9992_write(state, 0X70, on ? 0x11 : 0x91); // FIXME PAL
    0XA2 - MIPI ANALOG CTRL MISC
        ret = tw9992_write(state, 0X70, on ? 0x30 : 0x0f);

    ret = tw9992_write(state, 0x, val);
    if (ret)
        return ret;
#endif

    return 0;
}

static int tw9992_s_power(struct v4l2_subdev* sd, int on) {
    struct tw9992_state* state = to_state(sd);
    int ret;

    ret = mutex_lock_interruptible(&state->mutex);
    if (ret)
        return ret;

    ret = tw9992_set_power(state, on);
    if (ret == 0)
        state->powered = on;

    mutex_unlock(&state->mutex);
    return ret;
}

static int tw9992_log_status(struct v4l2_subdev* sd) {
    struct tw9992_state* state = to_state(sd);
    //struct v4l2_dv_timings timings;
    //int ret;

    v4l2_info(sd, "-----Chip status-----\n");
    v4l2_info(sd, "Chip power: %s\n", state->powered ? "off" : "on");

    v4l2_info(sd, "-----Signal status-----\n");

    int status1 = tw9992_read(state, TW9992_REG_STATUS1);
    int status2 = tw9992_read(state, TW9992_REG_SDT);
    v4l2_std_id std = tw9992_std_to_v4l2(status1, status2);


#if 0
    // Check short circuit status
    cable_det = info->read_cable_det(sd);
    v4l2_info(sd, "Cable detected (+5V power) port A: %s, B: %s, C: %s, D: %s\n",
        ((cable_det & 0x01) ? "Yes" : "No"),
        ((cable_det & 0x02) ? "Yes" : "No"),
        ((cable_det & 0x04) ? "Yes" : "No"),
        ((cable_det & 0x08) ? "Yes" : "No"));
#endif
    v4l2_info(sd, "Composite signal detected: %s\n",
        (status1 & TW9992_VDLOSS) ? "false" : "true");
    // TODO hlock vlock...
#if 0
    v4l2_info(sd, "TMDS signal locked: %s\n",
        no_lock_tmds(sd) ? "false" : "true");
    v4l2_info(sd, "SSPD locked: %s\n", no_lock_sspd(sd) ? "false" : "true");
    v4l2_info(sd, "STDI locked: %s\n", no_lock_stdi(sd) ? "false" : "true");
    v4l2_info(sd, "CP locked: %s\n", no_lock_cp(sd) ? "false" : "true");
    v4l2_info(sd, "CP free run: %s\n",
        (in_free_run(sd)) ? "on" : "off");
    v4l2_info(sd, "Prim-mode = 0x%x, video std = 0x%x, v_freq = 0x%x\n",
        io_read(sd, 0x01) & 0x0f, io_read(sd, 0x00) & 0x3f,
        (io_read(sd, 0x01) & 0x70) >> 4);
#endif
    static const char* s = "UNKNOWN";
    switch (std) {
    case V4L2_STD_NTSC: s = "NTSC";
        break;
    case V4L2_STD_NTSC_443: s = "NTSC443";
        break;
    case V4L2_STD_PAL_M: s = "PAL_M";
        break;
    case V4L2_STD_PAL_60: s = "PAL_60";
        break;
    case V4L2_STD_PAL: s = "PAL";
        break;
    case V4L2_STD_SECAM: s = "SECAM";
        break;
    case V4L2_STD_PAL_Nc: s = "PAL_CN";
        break;
    }
    v4l2_info(sd, "Composite signal standard: %s\n", s);


    v4l2_info(sd, "-----Video Timings-----\n");
#if 0
    if (read_stdi(sd, &stdi))
        v4l2_info(sd, "STDI: not locked\n");
    else
        v4l2_info(sd, "STDI: lcf (frame height - 1) = %d, bl = %d, lcvs (vsync) = %d, %s, %chsync, %cvsync\n",
            stdi.lcf, stdi.bl, stdi.lcvs,
            stdi.interlaced ? "interlaced" : "progressive",
            stdi.hs_pol, stdi.vs_pol);
    if (tw9992_query_dv_timings(sd, &timings))
        v4l2_info(sd, "No video detected\n");
    else
        v4l2_print_dv_timings(sd->name, "Detected format: ",
            &timings, true);
    v4l2_print_dv_timings(sd->name, "Configured format: ",
        &state->timings, true);

    if (no_signal(sd))
        return 0;
#endif

    v4l2_info(sd, "-----Color space-----\n");
    v4l2_info(sd, "Output color space: %s\n",
        (state->fmt.code == tw9992_csi2_rgb_fmt.code) ? "RGB" : "YCbCr");
    return 0;
}


static const char* const test_pattern_menu[] = {
    "Test",
    "Disable",
};

static int tw9992_test_pattern(struct tw9992_state* state, int value) {
    switch (value) {
    case 0:
        tw9992_write(state, 0x7d, 0x86);
        tw9992_write(state, 0x7f,
            (state->curr_norm & V4L2_STD_PAL) ? 0x04 : 0x00);
        tw9992_write(state, 0x90, 0x00);
        break;

    case 1:
        tw9992_write(state, 0x7d, 0x06);
        tw9992_write(state, 0x7f, 0x00);
        tw9992_write(state, 0x90, 0x00);
        break;

    default:
        return -EINVAL;
    }
    return 0;
}

/**
  * private controls
  */
#define V4L2_CID_MUX        (V4L2_CTRL_CLASS_USER | 0x1001)
#define V4L2_CID_STATUS     (V4L2_CTRL_CLASS_USER | 0x1002)
#define V4L2_CID_GET_REG    (V4L2_CTRL_CLASS_USER | 0x1003)
#define V4L2_CID_SET_REG    (V4L2_CTRL_CLASS_USER | 0x1004)
#define V4L2_CID_SET_ADDR    (V4L2_CTRL_CLASS_USER | 0x1005)

static int tw9992_set_mux(struct v4l2_ctrl* ctrl) {
    struct v4l2_subdev* sd = to_tw9992_sd(ctrl);
    struct tw9992_state* state = to_state(sd);

    if (ctrl->val == 0) {
        tw9992_write(state, 0x02, 0x44);
        tw9992_write(state, 0x3b, 0x30);
    } else {
        tw9992_write(state, 0x02, 0x46);
        tw9992_write(state, 0x3b, 0x0c);
    }

    return 0;
}

static int tw9992_get_status(struct v4l2_ctrl* ctrl) {
    struct v4l2_subdev* sd = to_tw9992_sd(ctrl);
    struct tw9992_state* state = to_state(sd);
    u8 data = 0;
    u8 mux;
    u8 val = 0;

    data = tw9992_read(state, 0x02);
    data = data & 0x0f;
    if (data == 0x4)
        mux = 0;
    else
        mux = 1;

    if (mux == 0) {
        data = tw9992_read(state, TW9992_REG_STATUS1);
        if (!(data & 0x80))
            val |= 1 << 0;

        tw9992_write(state, TW9992_REG_IRQ3, 0x03);
        data = tw9992_read(state, TW9992_REG_IRQ3);
        if (data == 0x03)
            val |= 1 << 1;
    } else {
        data = tw9992_read(state, 0x03);
        if (!(data & 0x80))
            val |= 1 << 1;

        tw9992_write(state, TW9992_REG_IRQ3, 0x03);
        data = tw9992_read(state, TW9992_REG_IRQ3);
        if (data == 0x03)
            val |= 1 << 0;
    }

    ctrl->val = val;
    return 0;
}

static int tw9992_get_reg(struct v4l2_ctrl* ctrl) {
    struct v4l2_subdev* sd = to_tw9992_sd(ctrl);
    struct tw9992_state* state = to_state(sd);

    int data = tw9992_read(state, state->hack_addr);
    if (data < 0) {
        return data;
    }
    ctrl->val = data;
    return 0;
}

static int tw9992_set_reg(struct v4l2_ctrl* ctrl) {
    struct v4l2_subdev* sd = to_tw9992_sd(ctrl);
    struct tw9992_state* state = to_state(sd);

    if (!capable(CAP_SYS_RESOURCE))
        return -EPERM;

    int ret = tw9992_write(state, state->hack_addr, ctrl->val);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int tw9992_g_ctrl(struct v4l2_ctrl* ctrl) {
    //struct v4l2_subdev *sd = to_tw9992_sd(ctrl);
    //struct tw9992_state* state = to_state(sd);
// FIXME mutex lock
    switch (ctrl->id) {
    case V4L2_CID_STATUS:
        return tw9992_get_status(ctrl);
    case V4L2_CID_GET_REG:
        return tw9992_get_reg(ctrl);
    }
    return -EINVAL;
}

static int tw9992_s_ctrl(struct v4l2_ctrl* ctrl) {
    struct v4l2_subdev* sd = to_tw9992_sd(ctrl);
    struct tw9992_state* state = to_state(sd);
    int ret = mutex_lock_interruptible(&state->mutex);
    int val;

    if (ret)
        return ret;
    if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
        goto unlock;

    val = ctrl->val;
    switch (ctrl->id) {
    case V4L2_CID_BRIGHTNESS:
        ret = tw9992_write(state, TW9992_REG_BRIGHT, val);
        break;
    case V4L2_CID_HUE:
        /*Hue is inverted according to HSL chart */
        ret = tw9992_write(state, TW9992_REG_HUE, -val);
        break;
    case V4L2_CID_CONTRAST:
        ret = tw9992_write(state, TW9992_REG_CONTRAST, val);
        break;
    case V4L2_CID_SATURATION:
        /*
         *This could be V4L2_CID_BLUE_BALANCE/V4L2_CID_RED_BALANCE
         *Let's not confuse the user, everybody understands saturation
         */
        ret = tw9992_write(state, TW9992_REG_SAT_U, val);
        if (ret < 0)
            break;
        ret = tw9992_write(state, TW9992_REG_SAT_V, val);
        break;
    case V4L2_CID_TEST_PATTERN:
        ret = tw9992_test_pattern(state, val);
        break;
    case V4L2_CID_MUX:
        ret = tw9992_set_mux(ctrl);
        break;
    case V4L2_CID_SET_REG:
        ret = tw9992_set_reg(ctrl);
        break;
    case V4L2_CID_SET_ADDR:
        state->hack_addr = ctrl->val;
        break;
    default:
        ret = -EINVAL;
    }

unlock:
    mutex_unlock(&state->mutex);
    return ret;
}

static const struct v4l2_ctrl_ops tw9992_ctrl_ops = {
    .g_volatile_ctrl = tw9992_g_ctrl,
    .s_ctrl = tw9992_s_ctrl,
};

static const struct v4l2_ctrl_config tw9992_ctrl_set_mux = {
    .ops = &tw9992_ctrl_ops,
    .id = V4L2_CID_MUX,
    .name = "Input Mux",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 11,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config tw9992_ctrl_get_status = {
    .ops = &tw9992_ctrl_ops,
    .id = V4L2_CID_STATUS,
    .name = "Input Status",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .flags = V4L2_CTRL_FLAG_VOLATILE,
};

static const struct v4l2_ctrl_config tw9992_ctrl_get_reg = {
    .ops = &tw9992_ctrl_ops,
    .id = V4L2_CID_GET_REG,
    .name = "getreg",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .flags = V4L2_CTRL_FLAG_VOLATILE,
};

static const struct v4l2_ctrl_config tw9992_ctrl_set_reg = {
    .ops = &tw9992_ctrl_ops,
    .id = V4L2_CID_SET_REG,
    .name = "setreg",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};

static const struct v4l2_ctrl_config tw9992_ctrl_set_addr = {
    .ops = &tw9992_ctrl_ops,
    .id = V4L2_CID_SET_ADDR,
    .name = "setaddr",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .min = 0,
    .max = 255,
    .step = 1,
    .def = 0,
};


static int tw9992_init_controls(struct tw9992_state* state) {
    v4l2_ctrl_handler_init(&state->ctrl_hdl, 5);

    v4l2_ctrl_new_std(&state->ctrl_hdl, &tw9992_ctrl_ops,
        V4L2_CID_BRIGHTNESS, TW9992_BRI_MIN,
        TW9992_BRI_MAX, 1, TW9992_BRI_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &tw9992_ctrl_ops,
        V4L2_CID_CONTRAST, TW9992_CON_MIN,
        TW9992_CON_MAX, 1, TW9992_CON_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &tw9992_ctrl_ops,
        V4L2_CID_SATURATION, TW9992_SAT_MIN,
        TW9992_SAT_MAX, 1, TW9992_SAT_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &tw9992_ctrl_ops,
        V4L2_CID_HUE, TW9992_HUE_MIN,
        TW9992_HUE_MAX, 1, TW9992_HUE_DEF);
    // TODO V4L2_CID_SHARPNESS
    // TODO V4L2_CID_HUE

    v4l2_ctrl_new_std_menu_items(&state->ctrl_hdl, &tw9992_ctrl_ops,
        V4L2_CID_TEST_PATTERN,
        ARRAY_SIZE(test_pattern_menu) - 1,
        0, ARRAY_SIZE(test_pattern_menu) - 1,
        test_pattern_menu);

    v4l2_ctrl_new_custom(&state->ctrl_hdl, &tw9992_ctrl_set_mux, NULL);
    v4l2_ctrl_new_custom(&state->ctrl_hdl, &tw9992_ctrl_get_status, NULL);
    v4l2_ctrl_new_custom(&state->ctrl_hdl, &tw9992_ctrl_get_reg, NULL);
    v4l2_ctrl_new_custom(&state->ctrl_hdl, &tw9992_ctrl_set_reg, NULL);
    v4l2_ctrl_new_custom(&state->ctrl_hdl, &tw9992_ctrl_set_addr, NULL);

    state->sd.ctrl_handler = &state->ctrl_hdl;
    if (state->ctrl_hdl.error) {
        int err = state->ctrl_hdl.error;

        v4l2_ctrl_handler_free(&state->ctrl_hdl);
        return err;
    }
    v4l2_ctrl_handler_setup(&state->ctrl_hdl);

    return 0;
}
static void tw9992_exit_controls(struct tw9992_state* state) {
    v4l2_ctrl_handler_free(&state->ctrl_hdl);
}

static int tw9992_enum_mbus_code(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state,
    struct v4l2_subdev_mbus_code_enum* code) {

    if (code->index >= nmbus_codes)
        return -EINVAL;

    code->code = mbus_codes[code->index];

    return 0;
}

static int tw9992_mbus_fmt(struct v4l2_subdev* sd,
    struct v4l2_mbus_framefmt* fmt) {
    //struct tw9992_state* state = to_state(sd);

    if (fmt->code >= 0x1000 && fmt->code < 0x2000) {
        *fmt = tw9992_csi2_rgb_fmt;
    } else if (fmt->code >= 0x2000 && fmt->code < 0x3000) {
        *fmt = tw9992_csi2_default_fmt;
    } else {
        *fmt = tw9992_csi2_default_fmt;
    }
    //fmt->height = state->curr_norm & V4L2_STD_525_60 ? 480 : 576;
    if (fmt->field == V4L2_FIELD_ALTERNATE) {
        fmt->height /= 2;
    }
    return 0;
}

#if 0
static int tw9992_set_field_mode(struct tw9992_state* state) {

    0X28 - VERTICAL CONTROL I
        1 AFLD R / W Auto field generation control
        0 = Off 1 = On
        0

        0X2D - MISCELLANEOUS CONTROL REGISTER I(MISC1)
        6 EVCNT R / W 0 = Normal operation 1 = Even field counter in special mode 0

        return 0;
}
#endif

static int tw9992_get_pad_format(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state,
    struct v4l2_subdev_format* format) {
    struct tw9992_state* state = to_state(sd);

    if (format->pad != 0)
        return -EINVAL;

    if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
        format->format = *v4l2_subdev_get_try_format(sd, sd_state, format->pad);
    } else {
        format->format = state->fmt;
    }

    return 0;
}

static int tw9992_set_pad_format(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state,
    struct v4l2_subdev_format* format) {
    struct tw9992_state* state = to_state(sd);
    struct v4l2_mbus_framefmt* framefmt;
    struct v4l2_mbus_framefmt* mbus_fmt = &format->format;
    int ret;

    ret = tw9992_mbus_fmt(sd, mbus_fmt);

    if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
        state->fmt = *mbus_fmt;

        if (state->fmt.code == tw9992_csi2_rgb_fmt.code) {
            // RGB565
            tw9992_write(state, TW9992_REG_DATACONV, 0x01);
            tw9992_write(state, TW9992_REG_AFESEL, 0x01);
        } else {
            // YUV422
            tw9992_write(state, TW9992_REG_DATACONV, 0x00);
            tw9992_write(state, TW9992_REG_AFESEL, 0x09);
        }

    } else { // V4L2_SUBDEV_FORMAT_TRY
        framefmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
        *framefmt = *mbus_fmt;
    }

    return ret;
}

static int tw9992_init_cfg(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state) {
    struct tw9992_state* state = to_state(sd);
    int ret;

    ret = tw9992_set_power(state, 1);
    if (ret)
        return ret;
    state->powered = 1;
    //ret = tw9992_write(state, TW9992_REG_MIPICTL, 0x11);// FIXME MIPI, PAL
    tw9992_write(state, TW9992_REG_MIPI_AN_CTL, 0x30);// MIPI ANALOG CTRL MISC

    int width = state->fmt.width * 2;
    tw9992_write(state, TW9992_REG_PICWIDTH, width >> 8);
    tw9992_write(state, TW9992_REG_PICWIDTH_L, width & 0xff);
    tw9992_write(state, TW9992_REG_WORD_CNT, width >> 8);
    tw9992_write(state, TW9992_REG_WORD_CNT_L, width & 0xff);

    // TODO TW9992_CROP_HEIGHT ?

    // Picture tuning
    tw9992_write(state, TW9992_REG_HDELAY_LO, TW9992_HDELAY); // left margin
    tw9992_write(state, TW9992_REG_VDELAY_LO, TW9992_VDELAY); // top margin
    

    tw9992_write(state, TW9992_REG_CNTRL1, 0xdc); // darker picture
    // TODO TW9992_REG_SHARPNESS, 0x11

    /*
    int status1 = tw9992_read(state, TW9992_REG_STATUS1);
    int status2 = tw9992_read(state, TW9992_REG_SDT);
    if (status1 >= 0 && status2 >= 0) {
        v4l2_std_id std = tw9992_std_to_v4l2(status1, status2);
        state->curr_norm = std;
    }
        */

    ret = tw9992_program_std(state);
    if (ret)
        return ret;

    //tw9992_set_field_mode(state);

    struct v4l2_subdev_format fmt = {
        .which = sd_state ? V4L2_SUBDEV_FORMAT_TRY
        : V4L2_SUBDEV_FORMAT_ACTIVE,
    };
    ret = tw9992_set_pad_format(sd, sd_state, &fmt);

    ret = tw9992_write(state, TW9992_REG_ACNTL, 0x80);// SRESET
    return ret;
}

static int tw9992_get_mbus_config(struct v4l2_subdev* sd,
    unsigned int pad,
    struct v4l2_mbus_config* cfg) {
    //struct tw9992_state *state = to_state(sd);

    cfg->type = V4L2_MBUS_CSI2_DPHY;
    cfg->bus.mipi_csi2.num_data_lanes = 1;
    cfg->bus.mipi_csi2.flags = 0;
    return 0;
}


#if 0
static int tw9992_get_selection(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state,
    struct v4l2_subdev_selection* sel) {
    switch (sel->target) {
    case V4L2_SEL_TGT_CROP: {
        sel->r = *v4l2_subdev_get_pad_crop(sd, sd_state, 0);
        return 0;
    }

    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.top = 0;
        sel->r.left = 0;
        sel->r.width = TW9992_MAX_WIDTH;
        sel->r.height = TW9992_MAX_HEIGHT;

        return 0;

    case V4L2_SEL_TGT_CROP_DEFAULT:
    case V4L2_SEL_TGT_CROP_BOUNDS:
        sel->r.top = TW9992_CROP_TOP;
        sel->r.left = TW9992_CROP_LEFT;
        sel->r.width = TW9992_CROP_WIDTH;
        sel->r.height = TW9992_CROP_HEIGHT;

        return 0;
    }

    return -EINVAL;
    return 0;
}

static int tw9992_enum_frame_size(struct v4l2_subdev* sd,
    struct v4l2_subdev_state* sd_state,
    struct v4l2_subdev_frame_size_enum* fse) {
    struct tw9992_state* state = to_state(sd);
    u32 code;

    if (fse->pad > 0)
        return -EINVAL;
#if 0
    if (fse->index >= ARRAY_SIZE(supported_modes))
        return -EINVAL;

    code = imx219_get_format_code(imx219, fse->code);
    if (fse->code != code)
        return -EINVAL;

    fse->min_width = supported_modes[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = supported_modes[fse->index].height;
    fse->max_height = fse->min_height;
#endif
    return 0;
}
#endif

static int tw9992_get_skip_frames(struct v4l2_subdev* sd, u32* frames) {
    // TODO is this needed ?
    *frames = TW9992_NUM_OF_SKIP_FRAMES;

    return 0;
}

static int tw9992_g_pixelaspect(struct v4l2_subdev* sd, struct v4l2_fract* aspect) {
    struct tw9992_state* state = to_state(sd);
    // TODO is this correct ?
    if (state->curr_norm & V4L2_STD_525_60) {
        aspect->numerator = 11;
        aspect->denominator = 10;
    } else {
        aspect->numerator = 54;
        aspect->denominator = 59;
    }

    return 0;
}

static int tw9992_g_tvnorms(struct v4l2_subdev* sd, v4l2_std_id* norm) {
    *norm = V4L2_STD_ALL;
    return 0;
}

static int tw9992_s_stream(struct v4l2_subdev* sd, int enable) {
    struct tw9992_state* state = to_state(sd);
    int ret;

    // TODO check
    if (!enable) {
        state->streaming = enable;
        return 0;
    }

    tw9992_check_input(sd);

    /* Must wait until querystd released the lock */
    ret = mutex_lock_interruptible(&state->mutex);
    if (ret)
        return ret;
    state->streaming = enable;
    mutex_unlock(&state->mutex);
    return 0;
}

static int tw9992_subscribe_event(struct v4l2_subdev* sd,
    struct v4l2_fh* fh,
    struct v4l2_event_subscription* sub) {
    switch (sub->type) {
    case V4L2_EVENT_SOURCE_CHANGE:
        return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
    case V4L2_EVENT_CTRL:
        return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
    default:
        return -EINVAL;
    }
}

static const struct v4l2_subdev_video_ops tw9992_video_ops = {
    .s_std = tw9992_s_std,
    .g_std = tw9992_g_std,
    .g_frame_interval = tw9992_g_frame_interval,
    .querystd = tw9992_querystd,
    .g_input_status = tw9992_g_input_status,
    .s_routing = tw9992_s_routing,
    .g_pixelaspect = tw9992_g_pixelaspect,
    .g_tvnorms = tw9992_g_tvnorms,
    .s_stream = tw9992_s_stream,
};

static const struct v4l2_subdev_core_ops tw9992_core_ops = {
    .log_status = tw9992_log_status,
    .s_power = tw9992_s_power,
    .subscribe_event = tw9992_subscribe_event,
    .unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops tw9992_pad_ops = {
    .init_cfg = tw9992_init_cfg,
    .enum_mbus_code = tw9992_enum_mbus_code,
    .set_fmt = tw9992_set_pad_format,
    .get_fmt = tw9992_get_pad_format,
    .get_mbus_config = tw9992_get_mbus_config,
    //.get_selection = tw9992_get_selection,
    //.enum_frame_size = tw9992_enum_frame_size,
};

static const struct v4l2_subdev_sensor_ops tw9992_sensor_ops = {
    .g_skip_frames = tw9992_get_skip_frames,
};

static const struct v4l2_subdev_ops tw9992_ops = {
    .core = &tw9992_core_ops,
    .video = &tw9992_video_ops,
    .pad = &tw9992_pad_ops,
    .sensor = &tw9992_sensor_ops,
};

static int tw9992_select_input(struct tw9992_state* state, unsigned int input) {
    int ret;

    ret = tw9992_write(state, TW9992_REG_INFORM, 0x40 | (input & 0x0f));

    switch (input) {
    case TW9992_INPUT_CVBS_AIN1:
    case TW9992_INPUT_CVBS_AIN2:
    case TW9992_INPUT_CVBS_AIN3:
    case TW9992_INPUT_CVBS_AIN4:
    case TW9992_INPUT_CVBS_AIN5:
    case TW9992_INPUT_CVBS_AIN6:
    case TW9992_INPUT_CVBS_AIN7:
    case TW9992_INPUT_CVBS_AIN8:
        break;
    case TW9992_INPUT_DIFF_CVBS_AIN1_AIN2:
    case TW9992_INPUT_DIFF_CVBS_AIN3_AIN4:
    case TW9992_INPUT_DIFF_CVBS_AIN5_AIN6:
    case TW9992_INPUT_DIFF_CVBS_AIN7_AIN8:
        // 0X36 - DIFFERENTIAL CLAMPING CONTROL 1
        // 0X37 - DIFFERENTIAL CLAMPING CONTROL 2
        // 0X38 - DIFFERENTIAL CLAMPING CONTROL 3
        // 0X39 - DIFFERENTIAL CLAMPING CONTROL 4
        // 0X3C - DIFFERENTIAL CLAMPING CONTROL 5
        // 0X3D - DIFFERENTIAL CLAMPING CONTROL 6
        break;
    default:
        return -1;
    }

    return ret;
}

static int init_device(struct tw9992_state* state) {
    int ret;
    int i;

    mutex_lock(&state->mutex);

    tw9992_set_power_pin(state, true);
    tw9992_set_reset_pin(state, false);

    tw9992_select_input(state, TW9992_INPUT_CVBS_AIN1);

    ret = tw9992_init_cfg(&state->sd, 0);
    if (ret)
        goto out_unlock;

    /* Select first valid input */
    // TODO remove ?
    for (i = 0; i < 32; i++) {
        if (BIT(i) & tw9992_valid_input_mask) {
            ret = tw9992_select_input(state, i);

            if (ret == 0) {
                state->input = i;
                break;
            }
        }
    }

out_unlock:
    mutex_unlock(&state->mutex);

    return ret;
}

static int tw9992_probe(struct i2c_client* client) {
    //const struct i2c_device_id *id = i2c_client_get_device_id(client);
    //struct device_node *np = client->dev.of_node;
    struct tw9992_state* state;
    struct v4l2_subdev* sd;
    int ret;

    /* Check if the adapter supports the needed features */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;

    state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
    if (state == NULL)
        return -ENOMEM;

    state->client = client;

#if 0
    state->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "powerdown",
        GPIOD_OUT_HIGH);
    if (IS_ERR(state->pwdn_gpio)) {
        ret = PTR_ERR(state->pwdn_gpio);
        v4l_err(client, "request for power pin failed: %d\n", ret);
        return ret;
    }

    state->rst_gpio = devm_gpiod_get_optional(&client->dev, "reset",
        GPIOD_OUT_HIGH);
    if (IS_ERR(state->rst_gpio)) {
        ret = PTR_ERR(state->rst_gpio);
        v4l_err(client, "request for reset pin failed: %d\n", ret);
        return ret;
    }

#endif

    state->fmt = tw9992_csi2_default_fmt;
    mutex_init(&state->mutex);
    state->curr_norm = V4L2_STD_PAL_BG;
    state->powered = true;
    state->input = 0;
    sd = &state->sd;
    v4l2_i2c_subdev_init(sd, client, &tw9992_ops);
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

    ret = tw9992_init_controls(state);
    if (ret)
        goto err_unregister_vpp_client;

    state->pad.flags = MEDIA_PAD_FL_SOURCE;
    sd->entity.function = MEDIA_ENT_F_ATV_DECODER;
    ret = media_entity_pads_init(&sd->entity, 1, &state->pad);
    if (ret)
        goto err_free_ctrl;

    ret = init_device(state);
    if (ret)
        goto err_media_entity_cleanup;

    ret = v4l2_async_register_subdev(sd);
    if (ret)
        goto err_free_irq;

    mutex_lock(&state->mutex);
    ret = tw9992_read(state, TW9992_REG_PRODUCT_ID);
    mutex_unlock(&state->mutex);
    if (ret < 0)
        goto err_v4l2_async_unregister;

    if (ret != 0x92) {
        v4l_err(client, "invalid chip id 0x%x found @ 0x%02x (%s)\n",
            ret, client->addr, client->adapter->name);
        goto err_v4l2_async_unregister;
    }

    v4l_info(client, "chip id 0x%x found @ 0x%02x (%s)\n",
        ret, client->addr, client->adapter->name);

    return 0;

err_v4l2_async_unregister:
    v4l2_async_unregister_subdev(sd);
err_free_irq:
err_media_entity_cleanup:
    media_entity_cleanup(&sd->entity);
err_free_ctrl:
    tw9992_exit_controls(state);
err_unregister_vpp_client:
    //err_unregister_csi_client:
    mutex_destroy(&state->mutex);
    return ret;
}

static void tw9992_remove(struct i2c_client* client) {
    struct v4l2_subdev* sd = i2c_get_clientdata(client);
    struct tw9992_state* state = to_state(sd);

    v4l2_async_unregister_subdev(sd);

    media_entity_cleanup(&sd->entity);
    tw9992_exit_controls(state);

    tw9992_set_reset_pin(state, true);
    tw9992_set_power_pin(state, false);

    mutex_destroy(&state->mutex);
}

static const struct i2c_device_id tw9992_id[] = {
    { "tw9992", 0 },
    {},
};
MODULE_DEVICE_TABLE(i2c, tw9992_id);

#ifdef CONFIG_PM_SLEEP
static int tw9992_suspend(struct device* dev) {
    struct v4l2_subdev* sd = dev_get_drvdata(dev);
    struct tw9992_state* state = to_state(sd);

    return tw9992_set_power(state, false);
}

static int tw9992_resume(struct device* dev) {
    struct v4l2_subdev* sd = dev_get_drvdata(dev);
    struct tw9992_state* state = to_state(sd);
    int ret;

    ret = init_device(state);
    if (ret < 0)
        return ret;

    ret = tw9992_set_power(state, state->powered);
    if (ret)
        return ret;

    return 0;
}

static SIMPLE_DEV_PM_OPS(tw9992_pm_ops, tw9992_suspend, tw9992_resume);
#define TW9992_PM_OPS (&tw9992_pm_ops)

#else
#define TW9992_PM_OPS NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id tw9992_of_id[] = {
    {.compatible = "tw9992", },
    { },
};

MODULE_DEVICE_TABLE(of, tw9992_of_id);
#endif

static struct i2c_driver tw9992_driver = {
    .driver = {
           .name = KBUILD_MODNAME,
           .pm = TW9992_PM_OPS,
           .of_match_table = of_match_ptr(tw9992_of_id),
           },
    .probe = tw9992_probe,
    .remove = tw9992_remove,
    .id_table = tw9992_id,
};

module_i2c_driver(tw9992_driver);

MODULE_DESCRIPTION("Renesas TW9992 video decoder driver");
MODULE_AUTHOR("Jarkko Sonninen");
MODULE_LICENSE("GPL v2");
