/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/mt_i2c.h>
#include <platform/upmu_common.h>
#include "ddp_hal.h"
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define LCM_ID  (0x0080)

#define REGFLAG_DELAY	(0XFE)
#define REGFLAG_END_OF_TABLE (0xFF)	// END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE								0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


#define GPIO_LCM_3V3_EN 	(GPIO58 | 0x80000000)
#define GPIO_LCM_RST 		(GPIO158 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) 	(lcm_util.udelay(n))
#define MDELAY(n) 	(lcm_util.mdelay(n))


//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)  lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

 struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
	{0xC0, 8, {0xC7, 0x00, 0x00, 0x00, 0x1E, 
			0x10, 0x60, 0xE5}},
	{0xC1, 8, {0xC0, 0x01, 0x00, 0x00, 0x1D,
			0x00, 0xF0, 0xC8}},
	{0xC2, 8, {0xC0, 0x02, 0x00, 0x00, 0x1D,
			0x2A, 0xA0, 0x9F}},	
	{0xC3, 8, {0xC0, 0x02, 0x00, 0x00, 0x1E,
			0x2A, 0xA0, 0x9F}},
	{0xC4, 8, {0xC0, 0x02, 0x00, 0x00, 0x1D,
			0x10, 0x80, 0xB8}},
	{0xC5, 8, {0xC0, 0x02, 0x00, 0x00, 0x1E,
			0x10, 0xA0, 0xB8}},
	{0xC6, 8, {0xC7, 0x00, 0x02, 0x00, 0x1E, 
			0x10, 0xA0, 0xEC}},
	{0xC7, 8, {0xC7, 0x00, 0x00, 0x00, 0x1F,
			0x10, 0x60, 0xE5}},
	{0xC8, 1, {0xFF}},
	{0xB0, 5, {0x00, 0x08, 0x0C, 0x14, 0x14}},
	{0xBA, 1, {0x20}},
	{0xBB, 7, {0x55, 0x55, 0x55, 0x55, 0x55, 
			0x55, 0x55}},
	/*add by wuyujing, set dimming steps & time*/
	{0xD5, 1, {0x00}},
	{0xD7, 1, {0x00}},
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}},
	{0xE7, 5, {0x00, 0x00, 0x00, 0x00, 0x30}},
	{0xE1, 1, {0x10}},/*add by wuyujing, enable dimming*/
	{0xCA, 1, {0x04}},
	{0xE2, 1, {0x0A}},
	{0xE3, 1, {0x00}},
	{0xE7, 1, {0x00}},
	{0xED, 8, {0x48, 0x00, 0xE0, 0x13, 0x08,
			0x00, 0x92, 0x08}},
	{0xFD, 6, {0x00, 0x08, 0x1C, 0x00, 0x00, 
			0x01}},
	{0xC3, 11, {0x11, 0x24, 0x04, 0x0A, 0x01, 
			0x04, 0x00, 0x1C, 0x10, 0xF0, 
			0x00}},
	{0xEA, 5, {0x7F, 0x20, 0x00, 0x00, 0x00}},
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}},
	{0xB0, 3, {0x01, 0x01, 0x01}},
	{0xB1, 3, {0x05, 0x05, 0x05}},
	{0xB2, 3, {0xD0, 0xD0, 0xD0}},
	{0xB4, 3, {0x37, 0x37, 0x37}},
	{0xB5, 3, {0x05, 0x05, 0x05}},
	{0xB6, 3, {0x54, 0x54, 0x54}},
	{0xB7, 3, {0x24, 0x24, 0x24}},
	{0xB8, 3, {0x24, 0x24, 0x24}},
	{0xB9, 3, {0x14, 0x14, 0x14}},
	{0xBA, 3, {0x14, 0x14, 0x14}},
	{0xBC, 3, {0x00, 0xF8, 0xB2}},
	{0xBE, 3, {0x23, 0x00, 0x90}},
	//{0xCA, 1, {0x80}},
	{0xCB, 12, {0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00}},
	{0xCC, 12, {0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19}},
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x03}},
	{0xF1, 6, {0x10, 0x00, 0x00, 0x00, 0x01,
			0x2F}},
	{0xF6, 1, {0x0A}},
	{0xCA, 1, {0x21}},/*ACL enable*/
	{0xCB, 1, {0x21}},/*ACL level 2*/
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x05}},
	{0xC0, 7, {0x06, 0x02, 0x02, 0x22, 0x00, 0x00, 0x01}},

	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}},
	{0xFE, 2, {0x00, 0x80}},
	//{0xF1, 3, {0x22, 0x22, 0x32}},
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},

	//{0x35, 1, {0x00}},
	{0x36, 1, {0x02}},/*×óÓÒ¾µÏñµ÷Õû*/
	{0x51, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	{0x4f, 1, {0x01}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;
	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width 	= FRAME_WIDTH;
	params->height 	= FRAME_HEIGHT;

#if 0
	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
#endif

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq  	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;

	
	params->dsi.packet_size = 256;
	// Video mode setting       
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	params->dsi.word_count= FRAME_WIDTH * 3;	//DSI CMD mode need set these two bellow params, different to 6577
	//params->dsi.vertical_active_line=FRAME_HEIGHT;
	params->dsi.vertical_sync_active 	= 10;
	params->dsi.vertical_backporch		= 20;
	params->dsi.vertical_frontporch		= 20;
	params->dsi.vertical_active_line		= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active	= 8;
	params->dsi.horizontal_backporch	= 64;//20
	params->dsi.horizontal_frontporch	= 64;//20
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/*modify by wuyujing ,disable spread spectrum clock to resolve flash white line*/
	params->dsi.ssc_disable=1;
	/*modify by wuyuing, avoid interference RF*/
	params->dsi.PLL_CLOCK = 205;

#if 0
	//......................ESD....................................	
	params->dsi.noncont_clock = 1;
	params->dsi.noncont_clock_period = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;	
	//...............................................................
#endif
}

static void lcm_init(void)
{
	pr_info("%s\n", __func__);
	lcm_set_gpio_output(GPIO_LCM_RST, GPIO_OUT_ZERO);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCM_3V3_EN, GPIO_OUT_ONE);

	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCM_RST, GPIO_OUT_ONE);

	MDELAY(30);

	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting) /sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	pr_info("%s\n", __func__);
 	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) /sizeof(struct LCM_setting_table), 1);
	MDELAY(120);


	lcm_set_gpio_output(GPIO_LCM_RST, GPIO_OUT_ZERO);
	MDELAY(15);
	lcm_set_gpio_output(GPIO_LCM_3V3_EN, GPIO_OUT_ZERO);
}

static void lcm_resume(void)
{
	pr_err("%s\n", __func__);
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] =
		(x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	data_array[3] = 0x00053902;
	data_array[4] =
		(y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[5] = (y1_LSB);
	data_array[6] = 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	return 1;
}

/*add by wuyujing,backlight curve y=power(x, 2.2)*/
static unsigned char backlight_curve[256] = {
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   1,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
	  2,   2,   3,   3,   3,   3,   3,   4,   4,   4,
	  4,   5,   5,   5,   5,   6,   6,   6,   6,   7,
	  7,   7,   8,   8,   8,   9,   9,   9,  10,  10,
	 11,  11,  11,  12,  12,  13,  13,  13,  14,  14,
	 15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
	 20,  20,  21,  22,  22,  23,  23,  24,  25,  25,
	 26,  26,  27,  28,  28,  29,  30,  30,  31,  32,
	 33,  33,  34,  35,  35,  36,  37,  38,  39,  39,
	 40,  41,  42,  43,  43,  44,  45,  46,  47,  48,
	 49,  49,  50,  51,  52,  53,  54,  55,  56,  57,
	 58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
	 68,  69,  70,  71,  73,  74,  75,  76,  77,  78,
	 79,  81,  82,  83,  84,  85,  87,  88,  89,  90,
	 91,  93,  94,  95,  97,  98,  99, 100, 102, 103,
	105, 106, 107, 109, 110, 111, 113, 114, 116, 117,
	119, 120, 121, 123, 124, 126, 127, 129, 130, 132,
	133, 135, 137, 138, 140, 141, 143, 145, 146, 148,
	149, 151, 153, 154, 156, 158, 159, 161, 163, 165,
	166, 168, 170, 172, 173, 175, 177, 179, 181, 182,
	184, 186, 188, 190, 192, 194, 196, 197, 199, 201,
	203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
	223, 225, 227, 229, 231, 234, 236, 238, 240, 242,
	244, 246, 248, 251, 253, 255,
};
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
    unsigned char cmd = 0x51;
    unsigned char count = 1;
    unsigned char val = (unsigned char)level & 0xff;
    unsigned int real_val = 0;

#define LOWEST_BRIGHTNESS_OFFSET (33)

    if (0 == val)
        real_val = val;
    else
        real_val = backlight_curve[val] + LOWEST_BRIGHTNESS_OFFSET;

    if (real_val >= 0xff)
        real_val = 0xff;

    pr_info("%s val:%d level:%d real_val:%d\n", __func__, val, level, real_val);

    dsi_set_cmdq_V22(handle, cmd, count, (unsigned char *)(&real_val), 1);

    MDELAY(20);
}

/*add by wuyujing, support acl,start*/
static unsigned int acl_level = ACL_ON;

static void rm67120_set_acl(void *handle, unsigned int level)
{
	unsigned char cmd;
	unsigned char param[5];

	pr_info("%s level:%d\n", __func__, level);

	/* change to page 3 */
	cmd = 0xF0;
	param[0] = 0x55;
	param[1] = 0xAA;
	param[2] = 0x52;
	param[3] = 0x08;
	param[4] = 0x03;

	dsi_set_cmdq_V22(handle, cmd, 5, param, 1);
	/*must have this delay*/
	MDELAY(20);

	/* set acl  */
	cmd = 0xCA;
	if (ACL_ON == level) {
		acl_level = ACL_ON;
		param[0] =  0x21;
	} else if (ACL_OFF == level) {
		acl_level = ACL_OFF;
		param[0] = 0x20;
	}else {
		pr_err("%s invalid param:%d\n", __func__, level);
		return;
	}

	dsi_set_cmdq_V22(handle, cmd, 1, param, 1);
	/*must have this delay*/
	MDELAY(20);
	/* back to pre mode,page 0 */
	cmd = 0xF0;
	param[0] = 0x55;
	param[1] = 0xAA;
	param[2] = 0x52;
	param[3] = 0x08;
	param[4] = 0x00;

	dsi_set_cmdq_V22(handle, cmd, 5, param, 1);

	MDELAY(20);
}
/*add by wuyujing, support acl,end*/
static void rm67120_get_acl(unsigned int *level)
{
	*level = acl_level;
}


LCM_DRIVER rm67120_dsi_vdo_hd_gvo_drv = 
{
	.name			= "rm67120_dsi_vdo_hd_gvo",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           		= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,	
	.compare_id     	= lcm_compare_id,
	.set_backlight_cmdq    	= lcm_setbacklight_cmdq,
	/*add by wuyujing, support acl*/
	.set_acl_cmdq 		= rm67120_set_acl,
	.get_acl		= rm67120_get_acl,
#if (LCM_DSI_CMD_MODE)
	.update         		= lcm_update,
#endif
};

