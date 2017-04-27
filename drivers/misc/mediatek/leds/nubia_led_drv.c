/***********************************************************************************/
/* File Name: nubia_led_drv.c */
/* File Description: this file is used to make breathled driver to be added in kernel or module. */

/*  Copyright (c) 2016, NUBIA corporation department 2 section 3 wifi group xiaofeng. All rights reserved.           */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: nubia, Inc.,            */
/*
*/

/***********************************************************************************/
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <leds_hal.h>
#include "leds_drv.h"
#include "leds_sw.h"
#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>
#include "nubia_led_drv.h"

extern struct mutex leds_pmic_mutex;

int g_breath_time;
int g_breath_once_flag = 1;

static struct timer_list nubia_breathlight_timer;

static  float duty_mapping[16] = {
	0.123, 0.338, 0.523, 0.707, 
	0.926, 1.107, 1.291, 1.507, 
	1.691, 1.876, 2.091, 2.276,
	2.460, 2.676, 2.860, 3.075,
};

enum nubia_outn_mode{
	SW_RESET,	    // 0  soft_reset , all regs revert to default value.
	CONST_ON,	    // 1 work on a constant lightness.
	CONST_OFF,	    // 2 darkness is comming
	AUTO_BREATH, 	// 3 self breathing, used in sences such as missing message.
	STEP_FADE_IN,	// 4  fade in means that the lightness is getting stronger.
	STEP_FADE_OUT,	// 5  fade out means that the lightness is getting weaker
	BREATH_ONCE,     // 6 only breath once, touch the home menu for instance.
	RESERVED,		// 7 reserverd.
};

enum nubia_rom_channel{
	LED_LEFT = 8,
	LED_HOME = 16,
	LED_RIGHT = 32,
};

enum hw_channel{
	PMIC_ISINK0 = 1,
	PMIC_ISINK1 = 2,
	PMIC_ISINK2 = 4,
	PMIC_ISINK3 = 8,
};

#define BUFFER_LENGTH 20

#define NUBIA_LEDS_DEBUG  1 // for debug


#ifdef CONFIG_NUBIA_PMIC_LEDS_NX575J
static int g_isink_step = ISINK_0;
static int g_isink_duty = 24;
static int g_isink_fsel = ISINK_1KHZ;
static int g_tr1 = 0x00;
static int g_tr2 = 0x01;
static int g_tf1 = 0x00;
static int g_tf2 = 0x01;
static int g_ton = 0x01;
static int g_toff = 0x03;
#elif defined(CONFIG_NUBIA_PMIC_LEDS_NX573J)
static int g_isink_step = ISINK_0;
static int g_isink_duty = 3;
static int g_isink_fsel = ISINK_1KHZ;
static int g_tr1 = 0x00;
static int g_tr2 = 0x01;
static int g_tf1 = 0x00;
static int g_tf2 = 0x01;
static int g_ton = 0x01;
static int g_toff = 0x03;
#else
static int g_isink_step = ISINK_0;
static int g_isink_duty = 3;
static int g_isink_fsel = ISINK_1KHZ;
static int g_tr1 = 0x00;
static int g_tr2 = 0x01;
static int g_tf1 = 0x00;
static int g_tf2 = 0x01;
static int g_ton = 0x01;
static int g_toff = 0x03;
#endif





#if 0 
#define LED_ERROR(fmt,args...)  printk(KERN_ERR"[NUBIA_BREATHLED:%s,%d]" fmt,__func__,__LINE__,##args)
#define LED_DEBUG(fmt,args...)  printk(KERN_DEBUG"[NUBIA_BREATHLED:%s,%d]" fmt,__func__,__LINE__,##args)
#else
#define LED_ERROR(fmt,args...)  do {} while (0)
#define LED_DEBUG(fmt,args...)  do {} while (0)
#endif


static int nubia_pmic_isink0_const_on(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink0_const_on\n");
        pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);	/* sw workround for sync leds status */
    	pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);

        LED_DEBUG("step=%d, duty=%d,fsel=%d\n", g_isink_step,g_isink_duty,g_isink_fsel);
        pmic_set_register_value(PMIC_ISINK_CH0_STEP, g_isink_step);  /* 4mA */
        pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, g_isink_duty);
        pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, g_isink_fsel);

        pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
	#else
    	pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_FSEL, ISINK_1KHZ);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_ON);
	#endif

    return ret;
}


static int nubia_pmic_isink1_const_on(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink1_const_on\n");
        pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);	/* sw workround for sync leds status */
        pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
		pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		
		LED_DEBUG("step=%d, duty=%d,fsel=%d\n", g_isink_step,g_isink_duty,g_isink_fsel);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, g_isink_step);	/* 4mA */
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, g_isink_duty);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, g_isink_fsel);
		
        pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);   
    #else
    	pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_MODE, ISINK_PWM_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_FSEL, ISINK_1KHZ);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);//set it second time according leds.c in mtk native impl
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_ON);
    #endif

    return ret;

}

static int nubia_pmic_isink0_const_off(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink0_const_off\n");
        pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
    #else
        pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
    #endif

    return ret;
}

static int nubia_pmic_isink1_const_off(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink1_const_off\n");
        pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
    #else
        pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
    #endif

    return ret;
}

static int nubia_pmic_isink0_auto_breath(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink0_auto_breath\n");
        pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);   /* 0x0326 */
        pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down 0x0274*/
        pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0); /* 0x026E */
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);    /* 0x0328 */

        LED_DEBUG("step=%d,duty=%d,fsel=%d,tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d\n",g_isink_step,g_isink_duty,g_isink_fsel,g_tr1,g_tr2,g_tf1,g_tf2,g_ton,g_toff);
        pmic_set_register_value(PMIC_ISINK_CH0_STEP, g_isink_step);	/* 4mA 0x0312*/
        pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, g_tr1);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL, g_tr2);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL, g_tf1);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL, g_tf2);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, g_ton);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, g_toff);
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, g_isink_duty);   /* 0x0312 */
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, g_isink_fsel); /* 0x0310 */

        pmic_set_register_value(PMIC_ISINK_CHOP0_EN, 1);    /* 0x0326 */
		pmic_set_register_value(PMIC_ISINK_CH0_BIAS_EN, 1); /* 0x0326 */
		pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);    /* 0x0326 */
    #else
    	pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TR1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TR2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TF1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TF2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TON_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TOFF_SEL, 0x03);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_FSEL, ISINK_05HZ);	
		pmic_set_register_value(MT6351_PMIC_ISINK_CHOP0_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_BIAS_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_ON);
    #endif

    return ret;
}


static int nubia_pmic_isink1_auto_breath(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink1_auto_breath\n");
        pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
        pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
        pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);

		LED_DEBUG("step=%d,duty=%d,fsel=%d,tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d\n",g_isink_step,g_isink_duty,g_isink_fsel,g_tr1,g_tr2,g_tf1,g_tf2,g_ton,g_toff);
    	pmic_set_register_value(PMIC_ISINK_CH1_STEP, g_isink_step);	/* 4mA */
        pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL, g_tr1);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL, g_tr2);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL, g_tf1);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL, g_tf2);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL, g_ton);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL, g_toff);
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, g_isink_duty);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, g_isink_fsel);
        pmic_set_register_value(PMIC_ISINK_CHOP1_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH1_BIAS_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
    #else
    	pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TR1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TR2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TF1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TF2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TON_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TOFF_SEL, 0x03);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_FSEL, ISINK_05HZ);	
		pmic_set_register_value(MT6351_PMIC_ISINK_CHOP1_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_BIAS_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_ON);
    #endif


    return ret;
}

static int nubia_pmic_isink0_breath_once(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink0_breath_once\n");
        pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_OFF);
        pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
        pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);

		LED_DEBUG("step=%d,duty=%d,fsel=%d,tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d\n",g_isink_step,g_isink_duty,g_isink_fsel,g_tr1,g_tr2,g_tf1,g_tf2,g_ton,g_toff);
		pmic_set_register_value(PMIC_ISINK_CH0_STEP, g_isink_step);	/* 4mA */
        pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL, g_tr1);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL, g_tr2);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL, g_tf1);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL, g_tf2);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL, g_ton);
		pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL, g_toff);
		pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, g_isink_duty);
		pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, g_isink_fsel);
        pmic_set_register_value(PMIC_ISINK_CHOP0_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH0_BIAS_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH0_EN, NLED_ON);
    #else
        pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK0_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_MODE, ISINK_BREATH_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TR1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TR2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TF1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TF2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TON_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH0_TOFF_SEL, 0x03);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM0_FSEL, ISINK_05HZ);	
		pmic_set_register_value(MT6351_PMIC_ISINK_CHOP0_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_BIAS_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH0_EN, NLED_ON);
    #endif

    return ret;
}

static int nubia_pmic_isink1_breath_once(void)
{
    int ret = 0;
    
    #if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
        LED_DEBUG("nubia_pmic_isink1_breath_once\n");
        pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_OFF);
        pmic_set_register_value(PMIC_CLK_DRV_32K_CK_PDN, 0x0);	/* Disable power down */
        pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);

		LED_DEBUG("step=%d,duty=%d,fsel=%d,tr1=%d,tr2=%d,tf1=%d,tf2=%d,ton=%d,toff=%d\n",g_isink_step,g_isink_duty,g_isink_fsel,g_tr1,g_tr2,g_tf1,g_tf2,g_ton,g_toff);
		pmic_set_register_value(PMIC_ISINK_CH1_STEP, g_isink_step);	/* 4mA */
        pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL, g_tr1);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL, g_tr2);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL, g_tf1);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL, g_tf2);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL, g_ton);
		pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL, g_toff);
		pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, g_isink_duty);
		pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, g_isink_fsel);
        pmic_set_register_value(PMIC_ISINK_CHOP1_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH1_BIAS_EN, 1);
		pmic_set_register_value(PMIC_ISINK_CH1_EN, NLED_ON);
    #else
    	pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_OFF);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_32K_CK_PDN,0x0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_PDN, 0);
		pmic_set_register_value(MT6351_PMIC_RG_DRV_ISINK1_CK_CKSEL, 0);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_STEP, ISINK_3);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TR1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TR2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TF1_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TF2_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TON_SEL, 0x02);
		pmic_set_register_value(MT6351_PMIC_ISINK_BREATH1_TOFF_SEL, 0x03);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_DUTY, 15);
		pmic_set_register_value(MT6351_PMIC_ISINK_DIM1_FSEL, ISINK_05HZ);	
		pmic_set_register_value(MT6351_PMIC_ISINK_CHOP1_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_BIAS_EN, 1);
		pmic_set_register_value(MT6351_PMIC_ISINK_CH1_EN, NLED_ON);
    #endif

    return ret;
}

static int channels_by_project_breath_once(struct nubia_breathlight_control_data *led)
{
    int ret = 0;

    if (PMIC_ISINK0 == led->outn || PMIC_ISINK1== led->outn)
        led->breath_outn = PMIC_ISINK0 | PMIC_ISINK1;
        
    return ret;
}

static int nubia_set_pmic_reg_mode_breath_once(struct nubia_breathlight_control_data *led)
{
	int ret = 0;
	int tall_int;
	float tall;
	float tr1_sel;
	float tr2_sel;
	float tf1_sel;
	float tf2_sel;
	float ton_sel;
	float toff_sel;

	if(g_breath_once_flag == 0)
	{
	    LED_DEBUG("g_breath_once_flag == 0\n");
		return 0;
	}else{
		g_breath_once_flag = 0;
	}

    LED_DEBUG("led->outn = %d\n",led->outn);

    channels_by_project_breath_once(led);// by project

    LED_DEBUG("led->breath_outn = %d\n",led->breath_outn);
    if (0 == led->breath_outn)
    {
        LED_DEBUG("breath once has no hw isink channel!\n");
        return 0;
    }
    
    if (PMIC_ISINK0 & led->breath_outn)
        nubia_pmic_isink0_breath_once();

    if (PMIC_ISINK1 & led->breath_outn)
        nubia_pmic_isink1_breath_once();

	tr1_sel = duty_mapping[0];
	tr2_sel = duty_mapping[1];
	tf1_sel = duty_mapping[0];
	tf2_sel = duty_mapping[1];
	ton_sel = duty_mapping[1];
	toff_sel = duty_mapping[3] * 2.0;//the toff time is so special according to spec

	tall = tr1_sel + tr2_sel + tf1_sel + tf2_sel + ton_sel + toff_sel;
	tall_int = (int) tall;
	g_breath_time = tall_int;
	LED_DEBUG("g_breath_time = %d\n",g_breath_time);
		
	nubia_breathlight_timer.expires = jiffies + g_breath_time * HZ;
	add_timer(&nubia_breathlight_timer);//we add timer here,but where we do del_timer or del_timer_sync?		

	return ret;
}

static int nubia_breathlight_soft_reset( struct nubia_breathlight_control_data *led)
{
	int ret=0;
	return ret;
}

void nubia_breathlight_const_on(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG("led->outn = %d\n", led->outn);

    if (PMIC_ISINK0 & led->outn)
        nubia_pmic_isink0_const_on();

    if (PMIC_ISINK1 & led->outn)
        nubia_pmic_isink1_const_on();

	#if 0
		mutex_lock(&leds_pmic_mutex);//pmic mutex

		isink_num = LED_LEFT;
		nubia_set_pmic_reg_mode_const_on(led,isink_num);
		
		mutex_unlock(&leds_pmic_mutex);
		msleep(6);
    #endif
	
}

void nubia_breathlight_const_off(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG("led->outn = %d\n", led->outn);
    
    if (PMIC_ISINK0 == led->outn)
        nubia_pmic_isink0_const_off();

    if (PMIC_ISINK1 == led->outn)
        nubia_pmic_isink1_const_off();
}

void nubia_breathlight_auto_breath(struct nubia_breathlight_control_data *led) // self breath 0 - 255 version.
{
	LED_DEBUG("led->outn = %d\n", led->outn);
    
    if (PMIC_ISINK0 == led->outn)
        nubia_pmic_isink0_auto_breath();

    if (PMIC_ISINK1 == led->outn)
        nubia_pmic_isink1_auto_breath();
}

void nubia_breathlight_step_fade_in(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG(" entry\n");
}

void nubia_breathlight_step_fade_out(struct nubia_breathlight_control_data *led)
{
	LED_DEBUG(" entry\n");
}

void nubia_breathlight_breath_once(struct nubia_breathlight_control_data *led)
{
    nubia_set_pmic_reg_mode_breath_once(led);
}


static ssize_t blink_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return sprintf(buf, "%d\n", led->blink_mode);
}

static ssize_t  blink_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{

	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);
	sscanf(buf, "%d", &led->blink_mode);
	
#ifndef CONFIG_NUBIA_BREATH_USE_WORKSTRUCT
	switch (led->blink_mode) {
		case SW_RESET:
			nubia_breathlight_soft_reset(led);
			nubia_breathlight_const_off(led);
			break;

		case CONST_ON: 
			nubia_breathlight_const_on(led);
			break;

		case CONST_OFF:
			nubia_breathlight_const_off(led);
			break;

		case AUTO_BREATH:
			nubia_breathlight_auto_breath(led);
			break;

		case STEP_FADE_IN:
		    nubia_breathlight_step_fade_in(led);
			break;

		case STEP_FADE_OUT:
			nubia_breathlight_step_fade_out(led);
			break;

		case BREATH_ONCE:
			nubia_breathlight_breath_once(led);
			break;

		default:
			break;
	}
#else
	if((strcmp(led->grade_parameter, last_grade_parameter) != 0) || (strcmp(led->fade_parameter, last_fade_parameter) != 0) ||(strcmp(led->outn, last_outn)!= 0))
	{
		schedule_work(&led->work);
	}

#endif
	return count;
}

static ssize_t fade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct nubia_breathlight_control_data *led;
	char *after, *parm2,*parm3;

	
	unsigned long delay_off,delay_off_1;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long delay_on = simple_strtoul(buf, &after, 10);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	delay_off = simple_strtoul(parm2, &after, 10);

	while(isspace(*after))
		after++;
	parm3 = after;
	delay_off_1 = simple_strtoul(parm3, &after, 10);
	led->Rise_Fall_time = (int)delay_on;
	led->Hold_time = (int)delay_off;
	led->Off_time = (int)delay_off_1; 
	LED_DEBUG("fade_time=%d ,on_time=%d , off_time=%d\n",
		led->Rise_Fall_time,led->Hold_time,led->Off_time);
	return count;
}

static ssize_t fade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return snprintf(buf, BUFFER_LENGTH, "%4d %4d %4d\n",
			led->Rise_Fall_time, led->Hold_time, led->Off_time);
}
	
static ssize_t grade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

	struct nubia_breathlight_control_data *led;
	char *after, *parm2;
	unsigned long parameter_two;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	unsigned long parameter_one = simple_strtoul(buf, &after, 10);

	
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	parameter_two = simple_strtoul(parm2, &after, 10);

	led->min_grade=(int)parameter_one;

	
	led->max_grade=(int)parameter_two;

	LED_DEBUG("min_grade=%d , max_grade=%d\n",
		led->min_grade,led->max_grade);
	return count;
}
	
static ssize_t grade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return snprintf(buf, BUFFER_LENGTH,	"%4d %4d\n",
			led->min_grade,led->max_grade);
}

static int convert_rom_channel_to_hw_channel(int rom_channel)
{
    int hw_channel = 0;
    
    if (rom_channel&LED_LEFT)
        hw_channel = PMIC_ISINK0;
    if (rom_channel&LED_HOME)
        hw_channel = PMIC_ISINK2;
    if (rom_channel&LED_RIGHT)
        hw_channel = PMIC_ISINK1;

    return hw_channel;
}

static ssize_t outn_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct nubia_breathlight_control_data *led;
	char *after;
    int hw_channel = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	int parameter_one = (int)simple_strtoul(buf, &after, 10);
	
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);
    LED_DEBUG("rom channel=%d\n", parameter_one);

    hw_channel = convert_rom_channel_to_hw_channel(parameter_one);
	led->outn = hw_channel;
    
	LED_DEBUG("hw channel=%d\n", led->outn);
	return count;
}

static ssize_t outn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct nubia_breathlight_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct nubia_breathlight_control_data, cdev);

	return sprintf(buf, "%d\n",led->outn);	
}


#if NUBIA_LEDS_DEBUG
static ssize_t debug_on_light_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;	
}

static ssize_t debug_on_light_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	char *after, *parm2,*parm3;

	g_isink_step = simple_strtoul(buf, &after, 10);
    
	while(isspace(*after))
		after++;
    
	parm2 = after;
	g_isink_duty = simple_strtoul(parm2, &after, 10);

	while(isspace(*after))
		after++;
    
	parm3 = after;
	g_isink_fsel = simple_strtoul(parm3, &after, 10);
    
	LED_DEBUG("step=%d ,duty=%d , fsel=%d\n", g_isink_step,g_isink_duty,g_isink_fsel);
    
	return count;
}

static ssize_t debug_fade_time_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;	
}

static ssize_t debug_fade_time_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	char *after, *temp;

	g_tr1 = simple_strtoul(buf, &after, 16);
    
	while(isspace(*after))
		after++;
    
	temp = after;
	g_tr2 = simple_strtoul(temp, &after, 16);

	while(isspace(*after))
		after++;
    
	temp = after;
	g_tf1 = simple_strtoul(temp, &after, 16);

 	while(isspace(*after))
		after++;

  	temp = after;
	g_tf2 = simple_strtoul(temp, &after, 16);

    while(isspace(*after))
		after++; 

   	temp = after;
	g_ton = simple_strtoul(temp, &after, 16);  

    while(isspace(*after))
		after++; 

    temp = after;
	g_toff = simple_strtoul(temp, &after, 16);  
    
	LED_DEBUG("g_tr1=%d,g_tr2=%d,g_tf1=%d,g_tf2=%d,g_ton=%d,g_toff=%d\n",g_tr1,g_tr2,g_tf1,g_tf2,g_ton,g_toff);
    
	return count;
}
#endif

static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);
static DEVICE_ATTR(blink_mode, 0664, blink_mode_get, blink_mode_set);
#if NUBIA_LEDS_DEBUG
static DEVICE_ATTR(debug_on_light, 0664, debug_on_light_get, debug_on_light_set);
static DEVICE_ATTR(debug_fade_time, 0664, debug_fade_time_get, debug_fade_time_set);
#endif


static struct attribute *nubia_led_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	&dev_attr_blink_mode.attr,
	#if NUBIA_LEDS_DEBUG
	&dev_attr_debug_on_light.attr,
	&dev_attr_debug_fade_time.attr,
	#endif
	NULL
};

static const struct attribute_group nubia_led_attr_group = {
	.attrs = nubia_led_attrs,
};

void nubia_led_work(struct work_struct * work)
{
	struct nubia_breathlight_control_data *led_data =
		container_of(work, struct nubia_breathlight_control_data, work);

		switch (led_data->blink_mode) {
		case SW_RESET:
			nubia_breathlight_soft_reset(led_data);
			nubia_breathlight_const_off(led_data);
			break;

		case CONST_ON: 
			nubia_breathlight_const_on(led_data);
			break;

		case CONST_OFF:
			nubia_breathlight_const_off(led_data);
			break;

		case AUTO_BREATH:
			nubia_breathlight_auto_breath(led_data);
			break;

		case STEP_FADE_IN:
		    nubia_breathlight_step_fade_in(led_data);
			break;

		case STEP_FADE_OUT:
			nubia_breathlight_step_fade_out(led_data);
			break;

		case BREATH_ONCE:
			nubia_breathlight_breath_once(led_data);
			break;

		default:
			break;
	}
}


static void timer_breath_once_func(unsigned long data)
{
    struct nubia_breathlight_control_data *led = (struct nubia_breathlight_control_data *)data;
    
	LED_DEBUG(" entry\n");
    
	if (PMIC_ISINK0 & led->breath_outn)
        nubia_pmic_isink0_const_off();

    if (PMIC_ISINK1 & led->breath_outn)
        nubia_pmic_isink1_const_off();

	g_breath_once_flag = 1;
}

static void nubia_breathlight_preinit(void)
{
	LED_DEBUG(" entry\n");
	/*
	pmic_set_register_value(MT6351_PMIC_CHRIND_EN_SEL,1);
	msleep(10);
	//printk(KERN_ERR"xiaofeng:set PMIC_CHRIND_EN to 0\n");
	pmic_set_register_value(PMIC_CHRIND_EN,0);//xiaofeng add
	*/
}

static int nubia_breathlight_probe(struct platform_device *pdev) 
{
	int ret;
	struct nubia_breathlight_control_data *breathlight_ctl_data;
	LED_DEBUG(" entry\n");

	breathlight_ctl_data = kzalloc(sizeof(struct nubia_breathlight_control_data), GFP_KERNEL);
	if (!breathlight_ctl_data) {
		ret = -ENOMEM;
		goto err;
	}

	breathlight_ctl_data->cdev.name = "nubia_led";

	INIT_WORK(&breathlight_ctl_data->work, nubia_led_work);//AW  not use this logic

	ret = led_classdev_register(&pdev->dev, &breathlight_ctl_data->cdev);
	if (ret)
		goto init_fail;

	ret = sysfs_create_group(&breathlight_ctl_data->cdev.dev->kobj, &nubia_led_attr_group);
	if (ret)
		goto init_fail;

	platform_set_drvdata(pdev,breathlight_ctl_data);

    setup_timer(&nubia_breathlight_timer,timer_breath_once_func,(unsigned long)breathlight_ctl_data);

	nubia_breathlight_preinit();

	LED_DEBUG("after nubia_breathlight_preinit!\n");
	
	return 0;

 init_fail:
    led_classdev_unregister(&breathlight_ctl_data->cdev);
    cancel_work_sync(&breathlight_ctl_data->work);
 err:	
	kfree(breathlight_ctl_data);
	breathlight_ctl_data = NULL;

	return ret;
}
/*----------------------------------------------------------------------------*/
static int nubia_breathlight_remove(struct platform_device *pdev)
{
	struct nubia_breathlight_control_data *led_data = platform_get_drvdata(pdev);

	led_classdev_unregister(&led_data->cdev);//add para parent->del para parent due to conflict with leds.h

	sysfs_remove_group(&led_data->cdev.dev->kobj, &nubia_led_attr_group);
	LED_DEBUG(" entry\n");
	return 0;
}

static int nubia_breathlight_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TO BE FIX
	return 0;
}

static int nubia_breathlight_resume(struct platform_device *pdev)
{
	//TO BE FIX
	return 0;
}

static struct platform_device nubia_breathlight_dev = {
	.name = "pmic_breathlight",
	.id = -1
};

static struct platform_driver nubia_breathlight_drv = {
	.probe      = nubia_breathlight_probe,
	.remove     = nubia_breathlight_remove,
	.suspend = nubia_breathlight_suspend,
	.resume = nubia_breathlight_resume,
	.driver     =
	{
		.name  = "pmic_breathlight",
		.owner  = THIS_MODULE,
	}
};

static int __init nubia_breathlight_init(void)
{
	int err = 0;
	LED_DEBUG(" entry\n");
	err = platform_device_register(&nubia_breathlight_dev);
	if(err)
	{
		LED_ERROR("failed to register nubia_breathlight_dev,err =%d\n",err);
		return err;
	}
	
	err = platform_driver_register(&nubia_breathlight_drv);
	if(err)
	{
		LED_ERROR("failed to register nubia_breathlight_drv,err =%d\n",err);
		return -ENODEV;
	}
	
    return err;
}

static void __exit nubia_breathlight_exit(void)
{
	LED_DEBUG(" entry\n");
	del_timer(&nubia_breathlight_timer);
}


late_initcall(nubia_breathlight_init);
module_exit(nubia_breathlight_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("<bai.bo1@zte.com.cn>, lvsen");
MODULE_DESCRIPTION("nubia breathlight Linux driver");
MODULE_ALIAS("platform:nubia_led_drv");
MODULE_LICENSE("GPL");
