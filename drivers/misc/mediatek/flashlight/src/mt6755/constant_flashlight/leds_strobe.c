/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
#define GPIO_UNSUPPORTED 0xff
#define GPIO_SUPPORTED 0
#define GPIO_MODE_GPIO 0

#define GPIO_DIR_OUT 1

#define KD2684_REG_ENABLE      0x01
#define KD2684_REG_TIMING      0x08
#define KD2684_REG_FLASH_LED1  0x03
#define KD2684_REG_TORCH_LED1  0x05
//NX573 flash ID
#define KD2684_ID             0x01//bits [5:3]
#define LM3643_ID             0x00//bits [5:3]
//NX575 flash ID
#define KD2681_ID             0x00//bits [5:3]
#define LM3648_ID             0x00 //bits [5:3]

//KD2681 and LM3643 shared the same ID, only NX573 use CONFIG_ZTEMT_SUB_DUAL_FLASHLIGHT_AND_LCD_BKL
#ifdef CONFIG_ZTEMT_SUB_DUAL_FLASHLIGHT_AND_LCD_BKL
static int current_project = 573;
#else
static int current_project = 575;
#endif

//static int gIsTorch[18] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//static int gLedDuty[18] = { 0, 32, 64, 96, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/* current(mA) 50,94,141,188,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */
#define e_DutyNum 16

static int torchLEDReg_KD2684[e_DutyNum] = {40,48,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//93,187//for KTD2684 flash
static int torchLEDReg_LM3643[e_DutyNum] = {85,103,0,0,0,0,0,0,0,0,0,0,0,0};//93,187//For LM3643 flash
static int torchLEDReg_KD2681[e_DutyNum] = {42,51,0,0,0,0,0,0,0,0,0,0,0,0};//93,187//For KTD2681 flash
static int torchLEDReg_LM3648[e_DutyNum] = {42,51,0,0,0,0,0,0,0,0,0,0,0,0};//93,187//For LM3648 flash

static int flashLEDReg_KD2684[e_DutyNum] = {3,7,15,23,31,39,47,55,63,71,79,87,95,103,111,127};//for KTD2684 flash   93,187,280,374,468,562,655,749,843,937,1030,1124,1218,1312,1405,1499
static int flashLEDReg_LM3643[e_DutyNum] = {3,7,15,23,31,39,47,55,63,71,79,87,95,103,111,127};//For LM3643 flash
static int flashLEDReg_KD2681[e_DutyNum] = {1,3,7,11,15,19,23,27,31,35,39,43,47,51,55,63};//For KTD2681 flash
static int flashLEDReg_LM3648[e_DutyNum] = {1,3,7,11,15,19,23,27,31,35,39,43,47,51,55,63};////LM3648


/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *KD2684_i2c_client = NULL;

struct KD2684_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct KD2684_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct KD2684_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

int dual_flash_id = 0;//add by ztemt tanyijun
int deviceID = 0x08; //add by ztemt kangxiong
/* i2c access*/
/*
static int KD2684_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct KD2684_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/



int KD2684_write_reg( u8 reg, u8 val)
{
	int ret = 0;
//	struct KD2684_chip_data *chip = i2c_get_clientdata(client);
    
//	mutex_lock(&chip->lock);
//	ret = i2c_smbus_write_byte_data(client, reg, val);
//	mutex_unlock(&chip->lock);
    char buf[2] = {(char) reg, (char)val };

    pr_err("KD2684_write_reg reg = 0x%x val= 0x%x \n ",reg,val);

	KD2684_i2c_client->ext_flag |= I2C_A_FILTER_MSG;
    KD2684_i2c_client->addr = 0x63;
	   
    ret= i2c_master_send(KD2684_i2c_client, buf, 2);

	if (ret < 0)
		pr_err("failed writting at 0x%02x\n", reg);
	return ret;
}

 int KD2684_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct KD2684_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */




static int KD2684_chip_init(struct KD2684_chip_data *chip)
{
	return 0;
}

static int KD2684_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct KD2684_chip_data *chip;
	struct KD2684_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	pr_err("KD2684_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		pr_err("KD2684 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct KD2684_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		pr_err("KD2684 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct KD2684_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (KD2684_chip_init(chip) < 0)
		goto err_chip_init;

	KD2684_i2c_client = client;
	KD2684_i2c_client->addr = 0x63;
	pr_err("KD2684 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	pr_err("KD2684 probe is failed\n");
	return -ENODEV;
}

static int KD2684_remove(struct i2c_client *client)
{
	struct KD2684_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define KD2684_NAME "leds-KD2684"
static const struct i2c_device_id KD2684_id[] = {
	{KD2684_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id KD2684_of_match[] = {
	{.compatible = "mediatek,STROBE_MAIN"},
	{},
};
#endif

static struct i2c_driver KD2684_i2c_driver = {
	.driver = {
		   .name = KD2684_NAME,
#ifdef CONFIG_OF
		   .of_match_table = KD2684_of_match,
#endif
		   },
	.probe = KD2684_probe,
	.remove = KD2684_remove,
	.id_table = KD2684_id,
};

struct KD2684_platform_data KD2684_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_KD2684={ I2C_BOARD_INFO(KD2684_NAME, 0xC6), \
													.platform_data = &KD2684_pdata,};
static int __init KD2684_init(void)
{
	pr_err("KD2684_init\n");
	i2c_register_board_info(1, &i2c_KD2684, 1);
	return i2c_add_driver(&KD2684_i2c_driver);
}

static void __exit KD2684_exit(void)
{
	i2c_del_driver(&KD2684_i2c_driver);
}


module_init(KD2684_init);
module_exit(KD2684_exit);

MODULE_DESCRIPTION("Flash driver for KD2684");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;
	val = KD2684_read_reg(KD2684_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
	int buf[2];
	int currentID = 0;
	//int regValue=0;	

    
	if(gDuty <0)
		gDuty =0;
	if(gDuty>15)
		gDuty =15;
	
    flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH);

	KD2684_write_reg(0x08, 0x1f); //hljing: set timeout as 1440ms

	currentID = (deviceID >>3)&0x07;
	if(gDuty <=1)
	{
		buf[0] = 0x1;
		buf[1] = 0x79;//0xAB;
		
		if(currentID== KD2681_ID&&current_project == 575){//KTD2681
			KD2684_write_reg(KD2684_REG_TORCH_LED1, (torchLEDReg_KD2681[gDuty]&0x7F)+0x80);
			buf[1] = 0x7B;//0xAB;
		}else if((currentID == LM3643_ID)&&current_project == 573){//LM3643
			KD2684_write_reg(KD2684_REG_TORCH_LED1, torchLEDReg_LM3643[gDuty]&0x7F);
		}else if(currentID ==LM3648_ID&&current_project == 575){//LM3648
			KD2684_write_reg(KD2684_REG_TORCH_LED1, (torchLEDReg_LM3648[gDuty]&0x7F)+0x80);
			buf[1] = 0x7B;//0xAB;
		}else{//KTD2684 or else
			KD2684_write_reg(KD2684_REG_TORCH_LED1, torchLEDReg_KD2684[gDuty]&0x7F);
		}
	}
	else{
		buf[0] = 0x1;
		buf[1] = 0x7D;//0xAF;
		if(currentID == KD2681_ID&&current_project == 575){//KTD2681
			KD2684_write_reg(KD2684_REG_FLASH_LED1, (flashLEDReg_KD2681[gDuty]&0x7F)+0x80);
			buf[1] = 0x7F;//0xAF;
		}else if((currentID == LM3643_ID)&&current_project == 573){//LM3643
			KD2684_write_reg(KD2684_REG_FLASH_LED1, flashLEDReg_LM3643[gDuty]&0x7F);
		}else if(currentID==LM3648_ID&&current_project == 575){//LM3648
			KD2684_write_reg(KD2684_REG_FLASH_LED1, (flashLEDReg_LM3648[gDuty]&0x3F)+0x80);
			buf[1] = 0x7F;//0xAF;
		}else{//KTD2684 or else
			KD2684_write_reg(KD2684_REG_FLASH_LED1, flashLEDReg_KD2684[gDuty]&0x7F);
		}		
	}

    KD2684_write_reg( buf[0], buf[1]);
	
	PK_DBG("kdebug FL_Enable line=%d��gDuty=%d\n", __LINE__,gDuty);
	return 0;
}



int FL_Disable(void)
{
	int buf[2];
	buf[0] = 0x1;
	buf[1] = 0xA0;
	KD2684_write_reg( buf[0], buf[1]);
	/*if(mt_set_gpio_out(CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO))
			{PK_DBG("[CAMERA FLASH] set gpio failed!! (FLASH_EN)\n");}*/
	 flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_LOW);
	//pr_err(" FL_Disable line kangxiong =%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	//int buf[2];
	//if (duty > 17)
	//	duty = 17;
	//if (duty < 0)
	//	duty = 0;
	//gDuty = duty;
//	buf[0] = 9;
//	buf[1] = gLedDuty[duty];
//	KD2684_write_reg(KD2684_i2c_client, buf[0], buf[1]);

	if(duty<0)		
		duty=0;	
	else if(duty>=e_DutyNum)		
		duty=e_DutyNum-1;	
	gDuty=duty;

 	PK_DBG(" FL_dim_duty line=%d, gDuty =%d\n", __LINE__,gDuty);
	return 0;
}




int FL_Init(void)
{
	//int buf[2];
/*
      	if(mt_set_gpio_mode(CAMERA_FLASH_EN_PIN,CAMERA_FLASH_EN_PIN_M_GPIO))
		       {PK_DBG("[CAMERA FLASH] set gpio mode failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_dir(CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT))
			{PK_DBG("[CAMERA FLASH] set gpio dir failed!! (FLASH_EN)\n");}
        if(mt_set_gpio_out(CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE))
			{PK_DBG("[CAMERA FLASH] set gpio failed!! (FLASH_EN)\n");}
*/
 	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH);
        mdelay(2);

	
	//buf[0] = 0X1;
	//buf[1] = 0xA3;
	//KD2684_write_reg( buf[0], buf[1]);
       deviceID = readReg(0x0C);
       printk("kdebug flash1_KD2684 FL_Init deviceID is: 0x%x\n",deviceID);
	
	pr_err(" FL_Init line=%d\n", __LINE__);
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG
	    ("KD2684 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg); */
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		pr_err("FLASHLIGHT_ONOFF 222: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		pr_err(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
