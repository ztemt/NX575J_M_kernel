/* BEGIN PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/disp_drv_platform.h>
#else
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/string.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...)  _dprintf(fmt, ##args)
#else
#define LCD_DEBUG(fmt, args...)  pr_info(fmt, ##args)
#endif


const static unsigned char LCD_MODULE_ID = 0x02;
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH 	(720)
#define FRAME_HEIGHT	(1280)


#define REGFLAG_DELAY	0xFC
#define REGFLAG_END_OF_TABLE	0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


#define GPIO_LCD_BIAS_ENP 	(GPIO58 | 0x80000000)    
#define GPIO_LCD_BIAS_ENN 	(GPIO89 | 0x80000000)  
#define GPIO_LCD_RST 		(GPIO158 | 0x80000000)

#define GPIO_LCD_ID0 (GPIO21 | 0x80000000)
#define GPIO_LCD_ID1 (GPIO57 | 0x80000000)


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

/*add by wuyujing, for cabc*/
static unsigned int lcm_isok = 0;
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

#define UDELAY(n)		(lcm_util.udelay(n))
#define MDELAY(n)		(lcm_util.mdelay(n))

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[128];
};

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	if (GPIO == 0xFFFFFFFF) {
#ifdef BUILD_LK
		printf("[LK/LCM] GPIO_LCD_RST =   0x%x\n", GPIO_LCD_RST);
#endif
		return;
	}

	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}

//update initial param for IC nt35521 0.01
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,4,{0xAA,0x55,0x25,0x01}},
	{0x6F,1,{0x21}},
	{0xF7,1,{0x01}},
	{REGFLAG_DELAY, 1, {}},
	{0x6F, 1, {0x21}},
	{0xF7, 1, {0x00}},
	{0xFF, 4, {0xAA,0x55,0x25,0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xB1, 2, {0x68,0x27}},
	/*modify by wuyujing,set PWM freq 30KHz*/
	{0xD9, 3, {0x1, 0x1, 0x60}},

	{0xB6, 1, {0x08}},
	{0xB8, 4, {0x01,0x00,0x08,0x08}},
	{0xBB, 2, {0x22,0x22}},
	{0xBC, 2, {0x00,0x00}},
	{0xBD, 5, {0x02,0x68,0x10,0x10,0x00}},
	{0xC8, 1, {0x80}},
	{0xD6, 2, {0x44, 0x04}},/*add by wuyujing,adjust dimming steps,make brightness change quickly*/
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	{0xB3, 2, {0x37,0x37}},
	{0xB4, 2, {0x19,0x19}},
	{0xB5, 2, {0x05,0x05}},
	{0xB9, 2, {0x46,0x46}},
	{0xBA, 2, {0x35,0x35}},
	{0xBC, 2, {0x90,0x00}},
	{0xBD, 2, {0x90,0x00}},
	/*add by wuyujing, programed VCOM, not configure 0XBE ant more*/
	//{0xBE, 1, {0x3B}},
	{0xC0, 1, {0x0C}},
	{0xCA, 1, {0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
	{0xEE, 1, {0x01}},
	/*modify by wuyujing,gamma curve 2.2-2.4*/
	{0xB0, 16, {0x00,0x00,0x00,0x19,0x00,0x49,0x00,0x60,0x00,0x7D,0x00,0xAA,0x00,0xCC,0x00,0xFC}},
	{0xB1, 16, {0x01,0x24,0x01,0x65,0x01,0x97,0x01,0xE7,0x02,0x28,0x02,0x2A,0x02,0x67,0x02,0xA2}},
	{0xB2, 16, {0x02,0xD0,0x03,0x06,0x03,0x29,0x03,0x64,0x03,0x81,0x03,0x93,0x03,0xB3,0x03,0xC9}},
	{0xB3, 4, {0x03,0xF4,0x03,0xFF}},
	/*gamma end*/
	{0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x03}},
	{0xB0, 2, { 0x00,0x00}},
	{0xB1, 2, { 0x00,0x00}},
	{0xB2, 5, { 0x07,0x00,0xA0,0x02,0x00}},
	{0xB6, 5, { 0x05,0x00,0x00,0x00,0x00}},
	{0xBA, 5, { 0x53,0x00,0xA0,0x00,0x00}},
	{0xBB, 5, { 0x53,0x00,0xA0,0x00,0x00}},
	{0xC4, 2, { 0x60}},
	{0xC5, 2, { 0xC0}},
	{0xF0, 5, { 0x55,0xAA,0x52,0x08,0x05}},
	{0xB0, 2, { 0x17,0x06}},
	{0xB1, 2, { 0x17,0x06}},
	{0xB2, 2, { 0x17,0x06}},
	{0xB4, 2, { 0x17,0x06}},
	{0xB5, 2, { 0x17,0x06}},
	{0xB8, 1, { 0x0C}},
	{0xB9, 1, { 0x00}},
	{0xBA, 1, { 0x00}},
	{0xBC, 1, { 0x02}},
	{0xBD, 5, { 0x03, 0x01, 0x01, 0x00, 0x03}},
	{0xC0, 1, { 0x07}},
	{0xC4, 1, { 0xA1}},
	{0xC8, 2, { 0x03,0x20}},
	{0xC9, 2, { 0x01,0x21}},
	{0xD1, 5, { 0x00,0x04,0xFC,0x07,0x14}},
	{0xD2, 5, { 0x10,0x05,0x00,0x03,0x16}},
	{0xD1, 5, { 0x00,0x05,0x04,0x00,0x00}},
	{0xD2, 5, { 0x00,0x05,0x08,0x00,0x00}},
	{0xE5, 1, { 0x06}},
	{0xE6, 1, { 0x06}},
	{0xE7, 1, { 0x06}},
	{0xE9, 1, { 0x06}},
	{0xEA, 1, { 0x06}},
	{0xED, 1, { 0x30}},
	{0xF0, 5, { 0x55,0xAA,0x52,0x08,0x06}},
	{0xF0, 5, { 0x55,0xAA,0x52,0x08,0x06}},
	{0xB0, 2, { 0x08,0x2E}},
	{0xB1, 2, { 0x2D,0x31}},
	{0xB2, 2, { 0x31,0x18}},
	{0xB3, 2, { 0x16,0x12}},
	{0xB4, 2, { 0x10,0x34}},
	{0xB5, 2, { 0x34,0x34}},
	{0xB6, 2, { 0x34,0x00}},
	{0xB7, 2, { 0x34,0x34}},
	{0xB8, 2, { 0x34,0x34}},
	{0xB9, 2, { 0x34,0x34}},
	{0xBA, 2, { 0x31,0x34}},
	{0xBB, 2, { 0x34,0x34}},
	{0xBC, 2, { 0x34,0x34}},
	{0xBD, 2, { 0x01,0x34}},
	{0xBE, 2, { 0x34,0x34}},
	{0xBF, 2, { 0x34,0x11}},
	{0xC0, 2, { 0x13,0x17}},
	{0xC1, 2, { 0x19,0x31}},
	{0xC2, 2, { 0x31,0x2D}},
	{0xC3, 2, { 0x2E,0x09}},
	{0xE5, 2, { 0x34,0x34}},
	{0xC4, 2, { 0x01,0x2D}},
	{0xC5, 2, { 0x2E,0x31}},
	{0xC6, 2, { 0x31,0x11}},
	{0xC7, 2, { 0x13,0x17}},
	{0xC8, 2, { 0x19,0x34}},
	{0xC9, 2, { 0x34,0x34}},
	{0xCA, 2, { 0x31,0x09}},
	{0xCB, 2, { 0x34,0x34}},
	{0xCC, 2, { 0x34,0x34}},
	{0xCD, 2, { 0x34,0x34}},
	{0xCE, 2, { 0x31,0x34}},
	{0xCF, 2, { 0x34,0x34}},
	{0xD0, 2, { 0x34,0x34}},
	{0xD1, 2, { 0x08,0x34}},
	{0xD2, 2, { 0x34,0x34}},
	{0xD3, 2, { 0x34,0x18}},
	{0xD4, 2, { 0x16,0x12}},
	{0xD5, 2, { 0x10,0x31}},
	{0xD6, 2, { 0x31,0x2E}},
	{0xD7, 2, { 0x2D,0x00}},
	{0xE6, 2, { 0x34,0x34}},
	{0xD8, 5, { 0x00,0x00,0x00,0x00,0x00}},
	{0xD9, 5, { 0x00,0x00,0x00,0x00,0x00}},
	{0xE7, 1, { 0x00}},
	{0x53, 1, { 0x2c}},
	/*add by wuyujing, set backlight 0x00,avoid splash screen when power on*/
	{0x51, 1, { 0x00}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {}},
};

#if 0					
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	//Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {}},
	{REGFLAG_DELAY, 20, {}},

	// Sleep Mode On
	{0x10, 0, {}},
	{REGFLAG_DELAY, 160, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_level_setting[] = {
    {0x55, 1, {0x01}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for(i = 0; i < count; i++) {
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

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	//video mode timing

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active	= 2;
	params->dsi.vertical_backporch		= 14; /*modify by wuyujing, avoid affect RF*/
	params->dsi.vertical_frontporch		= 12; /*modify by wuyujing, avoid affect RF*/
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 10;
	params->dsi.horizontal_backporch	= 80; /*modify by wuyujing, avoid affect RF*/
	params->dsi.horizontal_frontporch	= 60; /*modify by wuyujing, avoid affect RF*/
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	//improve clk quality
	/*modify by wuyujing, avoid affect RF*/
	params->dsi.PLL_CLOCK = 204; //this value must be in MTK suggested table
	params->dsi.compatibility_for_nvk = 1;
	params->dsi.ssc_disable = 1;

	/*add by wuyujing, for esd check,start*/
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	/*add by wuyujing, for esd check,end*/
}

static void lcm_init(void)
{
	//enable VSP & VSN
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ONE);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ONE);

	MDELAY(20);

	//reset high to low to high
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(30);

	// when phone initial , config output high, enable backlight drv chip  
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  

	LCD_DEBUG("%s\n", __func__);
}


static void lcm_suspend(void)
{
	//Back to MP.P7 baseline , solve LCD display abnormal On the right
	// when phone sleep , config output low, disable backlight drv chip  
	//lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ZERO);
	lcm_isok = 0;

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//reset low
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(5);
	//disable VSP & VSN
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, GPIO_OUT_ZERO);
	MDELAY(5);
	LCD_DEBUG("%s\n", __func__);
}
static void lcm_resume(void)
{
	lcm_init();
#if 0
	//enable VSP & VSN
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	msleep(50);

	//reset low to high
	lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	mdelay(5);   
	lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
	mdelay(5); 
	lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	msleep(10);	

	push_table(lcm_initialization_setting_tm, sizeof(lcm_initialization_setting_tm) / sizeof(struct LCM_setting_table), 1);
	//Back to MP.P7 baseline , solve LCD display abnormal On the right
	//when sleep out, config output high ,enable backlight drv chip  
	lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);

	LCD_DEBUG("uboot:tm_nt35521_lcm_resume\n");
#endif
}

static unsigned int lcm_compare_id(void)
{	
	unsigned int id0 =1;
	unsigned int id1 =1;

	/* fix me after two ic ok */
	return 1;

	mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCD_ID0, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_LCD_ID0, GPIO_PULL_UP);

	mt_set_gpio_mode(GPIO_LCD_ID0, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_ID0, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCD_ID1, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_LCD_ID1, GPIO_PULL_UP);

	id0 = mt_get_gpio_in(GPIO_LCD_ID0);
	id1 = mt_get_gpio_in(GPIO_LCD_ID1);
#ifdef BUILD_LK
	printf("%s,id0 =%x id1 =%x\n",__func__,id0,id1);
#else
	printk("%s,id0 =%x id1 =%x\n",__func__,id0,id1);
#endif

	if ((id0 == 0) && ( id1 == 0))
		return 1;
	else
		return 0;
}


/*add by wuyujing,support for cacb,start*/
static unsigned int cabc_level = CABC_OFF;

static void lcm_set_cabc(unsigned int level)
{
	unsigned char val = 0x0;

	switch (level){
	case CABC_OFF:
		val = 0x0;
		break;
	case CABC_UI:
		val = 0x1;
		break;
	case CABC_STILL:
		val = 0x2;
		break;
	case CABC_MOVING:
		val = 0x3;
		break;
	default:
		pr_err("invalid param:%d\n", level);
		return;
	}

	pr_info("%s cabc level:%d val:%x\n", __func__, level, val);

	lcm_cabc_level_setting[0].para_list[0] = val;
	cabc_level = level;
	push_table(lcm_cabc_level_setting,sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table),1);
}

static void lcm_get_cabc(unsigned int *level)
{
      *level = cabc_level;
}

static void lcm_is_lcmon(unsigned int *isok)
{
	*isok = lcm_isok;
}
/*add by wuyujing,support for cacb,end*/


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

#define LOWEST_BRIGHTNESS_OFFSET	(1)
#define HIGHEST_BRIGHTNESS_MAX		(242)

	if (val >= HIGHEST_BRIGHTNESS_MAX)
		val = HIGHEST_BRIGHTNESS_MAX;

	if (0 == val)
		real_val = 0;
	else
		real_val = backlight_curve[val] + LOWEST_BRIGHTNESS_OFFSET;

	if (real_val >= 0xff)
		real_val = 0xff;

	pr_info("%s val:%d level:%d real_val:%d\n", __func__, val, level, real_val);
	dsi_set_cmdq_V22(handle, cmd, count, (unsigned char *)(&real_val), 1);
	MDELAY(20);
	lcm_isok = 1;
}

LCM_DRIVER nt35521s_hd720_cpt_lcm_drv =
{
	.name           	= "nt35521s_hd720_dsi_video_cpt",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init          		= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.compare_id     	= lcm_compare_id,
	.set_backlight_cmdq  = lcm_setbacklight_cmdq,
	/*add by wuyujing, support cabc*/
	.set_cabc		= lcm_set_cabc,
	.get_cabc		= lcm_get_cabc,
	.is_lcmon		= lcm_is_lcmon,
};
