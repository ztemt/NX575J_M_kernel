/* drivers/hwmon/mt6516/amit/TMD2725X.c - TMD2725X ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * By LiuQiang of Nubia Modified
 */

#include <hwmsensor.h>
#include <cust_alsps.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <mach/gpio_const.h>
#include <mt-plat/mt_gpio.h>
#include <linux/uaccess.h>
#include "ams_tmd2725.h"
#include "ams_i2c.h"
#include "ams_tmd2725_ic.h"
#include "ams_tmd2725_prox.h"
#include "ams_tmd2725_als.h"
/*----------------------------------------------------------------------------*/

#define DEBUG_ALS_FAC

#define USE_ALS_FAC
/*#undef  USE_ALS_FAC*/ //whether use the ALS FAC value

#ifdef DEBUG_ALS_FAC
int gvalue=0;
static bool flag_is_set_tp_info = false; //default is 0
u32 g_value_target_cail;
#define ALS_CAL_PATH         "/persist/sensors/rgbc_cfg"
#define SCALE_FACTOR(x, y) (x)/(y)
#endif

#ifdef TMD2725X_IC_API_FUNC
extern int  tmd2725_read_als(struct tmd2725_chip *chip);
extern int  tmd2725_get_lux(struct tmd2725_chip *chip);
extern int  tmd2725_irq_handler(struct tmd2725_chip *chip);
extern int  tmd2725_get_id(struct tmd2725_chip *chip, u8 *id, u8 *rev, u8 *auxid) ;
extern void tmd2725_set_defaults(struct tmd2725_chip *chip);
extern int  tmd2725_flush_regs(struct tmd2725_chip *chip);
extern void tmd2725_read_prox(struct tmd2725_chip *chip);
extern void tmd2725_get_prox(struct tmd2725_chip *chip);
extern int  tmd2725_configure_prox_mode(struct tmd2725_chip *chip, u8 state);
extern int  tmd2725_configure_als_mode(struct tmd2725_chip *chip, u8 state);
extern int  tmd2725_update_enable_reg(struct tmd2725_chip *chip);
#endif

#define TMD2725X_DEV_NAME     "TMD2725X"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               pr_debug(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    pr_debug(APS_TAG fmt, ##args)

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1


/*----------------------------------------------------------------------------*/
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
int g_err=0;
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *TMD2725X_i2c_client;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id TMD2725X_i2c_id[] = { {TMD2725X_DEV_NAME, 0}, {} };

/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int TMD2725X_i2c_remove(struct i2c_client *client);
static int TMD2725X_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int TMD2725X_i2c_resume(struct i2c_client *client);
static int TMD2725X_remove(void);
static int TMD2725X_local_init(void);


extern int mt_gpio_set_debounce(unsigned int gpio, unsigned int debounce);

#ifdef FACTORY_MACRO_PS
//static int TMD2725X_prox_offset_cal_process(void);
static int TMD2725X_prox_thres_calibrate(struct i2c_client *client);
static int TMD2725X_get_persist_prox_thres(struct tmd2725_chip *chip);
#endif


static int TMD2725X_init_flag = -1; /* 0<==>OK -1 <==> fail*/
static struct alsps_init_info TMD2725X_init_info = {
		.name = "TMD2725X",
		.init = TMD2725X_local_init,
		.uninit = TMD2725X_remove,
};
static DEFINE_MUTEX(TMD2725X_mutex);
enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS = 2,
};
static unsigned long long int_top_time;

/*----------------------------------------------------------------------------*/
struct TMD2725X_i2c_addr {	/*define a series of i2c slave address */
	u8 write_addr;
	u8 ps_thd;		/*PS INT threshold */
};
/*----------------------------------------------------------------------------*/
struct tmd2725_chip  g_tmd2725_chip;
struct TMD2725X_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct irq_work;
/*by qliu4244 add sensors function start*/
#ifdef FACTORY_MACRO_PS
	struct delayed_work ps_cal_poll_work;
#endif
#ifdef CONFIG_OF
	struct device_node *irq_node;
#endif
/*by qliu4244 add sensors function end*/

	struct TMD2725X_i2c_addr addr;

	/*misc */
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als */
	atomic_t als_deb_on;	/*indicates if the debounce is on */
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_mask;	/*mask ps: always return far away */
	atomic_t ps_debounce;	/*debounce time after enabling ps */
	atomic_t ps_deb_on;	/*indicates if the debounce is on */
	atomic_t ps_deb_end;	/*the jiffies representing the end of debounce */
	atomic_t ps_suspend;

	/*data */
	u16 als;
	u16 ps;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL - 1];
	u32 als_value[C_CUST_ALS_LEVEL];
#ifdef FACTORY_MACRO_PS
	bool		debug;
#endif
	atomic_t als_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_cmd_val;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_high;	/*the cmd value can't be read, stored in ram */
	atomic_t ps_thd_val_low;	/*the cmd value can't be read, stored in ram */
	ulong enable;		/*enable mask */
	ulong pending_intr;	/*pending interrupt */
#ifdef TMD2725X_IC_API_FUNC
    struct tmd2725_chip* tmd2725_chip;
#endif
};
static struct TMD2725X_priv *g_TMD2725X_ptr;
static struct TMD2725X_priv *TMD2725X_obj;
static unsigned int alsps_irq;

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,als_ps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver TMD2725X_i2c_driver = {
	.probe = TMD2725X_i2c_probe,
	.remove = TMD2725X_i2c_remove,
	.detect = TMD2725X_i2c_detect,
	.suspend = TMD2725X_i2c_suspend,
	.resume = TMD2725X_i2c_resume,
	.id_table = TMD2725X_i2c_id,
	.driver = {
		.name = TMD2725X_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

#ifdef CONFIG_CUSTOM_KERNEL_ALSPS_NX575
#define GPIO_ALS_EINT_PIN         (GPIO12|0x80000000)
/*LDO EN CONFIG START*/
#define GPIO_ALS_LDO_EN         (GPIO22|0x80000000)
/*LDO EN CONFIG END*/
#else
#define GPIO_ALS_EINT_PIN         (GPIO12|0x80000000)
#endif
#define GPIO_ALS_EINT_PIN_M_EINT  GPIO_MODE_00

/*------------------------i2c function for 89-------------------------------------*/
int TMD2725X_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{
	int res = 0;

	mutex_lock(&TMD2725X_mutex);
	switch (i2c_flag) {
	case I2C_FLAG_WRITE:
		/* client->addr &= I2C_MASK_FLAG; */
		res = i2c_master_send(client, buf, count);
		/* client->addr &= I2C_MASK_FLAG; */
		break;

	case I2C_FLAG_READ:
		/*
		   client->addr &= I2C_MASK_FLAG;
		   client->addr |= I2C_WR_FLAG;
		   client->addr |= I2C_RS_FLAG;
		 */
		res = i2c_master_send(client, buf, count & 0xFF);
		/* client->addr &= I2C_MASK_FLAG; */
		res = i2c_master_recv(client, buf, count >> 0x08);
		break;
	default:
		APS_LOG("TMD2725X_i2c_master_operate i2c_flag command not support!\n");
		break;
	}
	if (res <= 0)
		goto EXIT_ERR;

	mutex_unlock(&TMD2725X_mutex);
	return res;
 EXIT_ERR:
	mutex_unlock(&TMD2725X_mutex);
	APS_ERR("TMD2725X_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
int TMD2725X_get_addr(struct alsps_hw *hw, struct TMD2725X_i2c_addr *addr)
{
	if (!hw || !addr)
		return -EFAULT;

	addr->write_addr = hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/
static void TMD2725X_power(struct alsps_hw *hw, unsigned int on)
{

}

/*----------------------------------------------------------------------------*/
static long TMD2725X_enable_als(struct i2c_client *client, int enable)
{
	if (enable) {
		atomic_set(&TMD2725X_obj->als_deb_on, 1);
		atomic_set(&TMD2725X_obj->als_deb_end,
			   jiffies + atomic_read(&TMD2725X_obj->als_debounce) / (1000 / HZ));
	}
	    tmd2725_configure_als_mode(TMD2725X_obj->tmd2725_chip,(u8)enable);
     return 0;
}

/*----------------------------------------------------------------------------*/
static long TMD2725X_enable_ps(struct i2c_client *client, int enable)
{
#ifdef FACTORY_MACRO_PS
	long res = 0;
	static bool only_once_load = false;
#endif
	if (enable) {
		//atomic_set(&TMD2725X_obj->ps_deb_on, 1);
		//atomic_set(&TMD2725X_obj->ps_deb_end, jiffies + atomic_read(&TMD2725X_obj->ps_debounce) / (1000 / HZ));


#ifdef FACTORY_MACRO_PS
  	if(only_once_load == false)
  	{
  	    res = TMD2725X_get_persist_prox_thres(TMD2725X_obj->tmd2725_chip); // add baibo 20170206
  	    if(res >= 0)
  	    {
  		    only_once_load = true;
  	    }
  	}
#endif
}
    AMS_MUTEX_LOCK(&TMD2725X_obj->tmd2725_chip->ps_lock);
    tmd2725_configure_prox_mode(TMD2725X_obj->tmd2725_chip,(u8)enable);
    AMS_MUTEX_UNLOCK(&TMD2725X_obj->tmd2725_chip->ps_lock);

	return 0;
}



#ifdef FACTORY_MACRO_PS
int sensor_read_file(char *file_path, char *read_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = 0;

	APS_DBG("read infomation : size =%d\n", count);
	if (NULL == file_path) {
		APS_DBG("file_path is NULL\n");
		goto error;
	}

	file_p = filp_open(file_path, O_RDONLY , 0);
	if (IS_ERR(file_p)) {
		APS_ERR("%s file does not exist!!\n", file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_read(file_p, (char*)read_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		APS_ERR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	APS_DBG("read ok\n");
	set_fs(old_fs);
	filp_close(file_p, NULL);

	return vfs_retval;

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	return -EINVAL;
}

int sensor_write_file(char *file_path, const char *write_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	APS_DBG("write infomation : size =%d\n", count);
	if (NULL == file_path) {
		APS_DBG("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_CREAT|O_RDWR|O_TRUNC , 0666);
	if (IS_ERR(file_p)) {
		APS_DBG("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_write(file_p, (char*)write_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		APS_DBG("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	APS_DBG("write ok\n");

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	return vfs_retval;
}

#endif


#ifdef FACTORY_MACRO_PS
static int TMD2725X_get_persist_prox_thres(struct tmd2725_chip *chip)
{
	int rc = 0;
	u8 data[2] = {0};

	rc = sensor_read_file(PS_CAL_FILE_PATH, data, sizeof(data));
	if (rc < 0) {
		APS_ERR("%s read file error\n", __FUNCTION__);
		return -1;
	}

    if (data[0] >= TMD2725X_THRES_MIN && data[0] <= TMD2725X_THRES_MAX && data[0] > data[1]) {
        chip->params.prox_thres_near = data[0];
        chip->params.prox_thres_far = data[1];
    }

    APS_DBG("%s near_val = %d, far_val = %d\n", __FUNCTION__,chip->params.prox_thres_near, chip->params.prox_thres_far);

	return rc ;
}

static int TMD2725X_prox_calibrate_is_done(void)
{
    int done = 1;

	int rc;
	u8 data[2] = {0};

	rc = sensor_read_file(PS_CAL_FILE_PATH, data, sizeof(data));
	if (rc < 0) {
		APS_ERR("read file error\n");
		done = 0;
	}

    APS_DBG("%s near_val = %d, far_val = %d\n", __FUNCTION__,data[0], data[1]);

    if (data[0] < TMD2725X_THRES_MIN || data[0] > TMD2725X_THRES_MAX) {
       done = 0;
    }

    return done;
}

static int TMD2725X_prox_thres_calibrate(struct i2c_client *client)
{
    int i;
    int near_val = 0;
    int ret = 0;
    u8 prox_buf[2] = {0};

    struct tmd2725_chip *chip = TMD2725X_obj->tmd2725_chip;

    APS_DBG("%s\n", __FUNCTION__);

    for(i = 0; i < PS_AVG_TIME; i++)
	{
		tmd2725_read_prox(chip);
		near_val += chip->prx_inf.raw;
		msleep(30);
	}

	near_val = near_val / PS_AVG_TIME;

    if (near_val < TMD2725X_THRES_MIN || near_val > TMD2725X_THRES_MAX) {
        APS_ERR("%s thres calibration failed!\n", __FUNCTION__);
        return -1;
    }

    chip->params.prox_thres_near = near_val;
    chip->params.prox_thres_far = near_val / 2 + 10;

    APS_DBG("%s near_val = %d, far_val = %d\n", __FUNCTION__,near_val, chip->params.prox_thres_far);

    prox_buf[0] = chip->params.prox_thres_near;
	prox_buf[1] = chip->params.prox_thres_far;

	ret = sensor_write_file(PS_CAL_FILE_PATH, prox_buf, sizeof(prox_buf));
    if (ret < 0) {
        APS_ERR("write file fail\n");
        return ret;
    }

    return chip->params.prox_thres_near;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/

static ssize_t TMD2725X_show_chip_name(struct device_driver *ddri, char *buf)
{
       ssize_t res;

       APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%s\n", TMD2725X_CHIP_NAME);
	return res;
}

static ssize_t TMD2725X_show_prox_thres_min(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_THRES_MIN);
	return res;
}

static ssize_t TMD2725X_show_prox_thres_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_THRES_MAX);
	return res;
}

static ssize_t TMD2725X_show_prox_thres(struct device_driver *ddri, char *buf)
{
    ssize_t res = 0;
    u8 data[2] = {0};

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res= sensor_read_file(PS_CAL_FILE_PATH, data, sizeof(data));
	if (res < 0) {
		APS_ERR("read file error\n");
		return -1;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", data[0]);

	return res;
}

static ssize_t TMD2725X_store_prox_thres(struct device_driver *ddri,const char *buf, size_t count)
{
	ssize_t res;
	 int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);

	if (value == 1)
	{
		TMD2725X_prox_thres_calibrate(TMD2725X_obj->client); // add baibo 20170206
	}

	return count;
}

static ssize_t TMD2725X_show_prox_data_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);

	return res;
}

static ssize_t TMD2725X_show_prox_calibrate_start(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return res;
}

static ssize_t TMD2725X_store_prox_calibrate_start(struct device_driver *ddri, const char *buf, size_t count)
{
       ssize_t res;
	 int value = 0;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);
	if (value == 1)
	{
		TMD2725X_obj->debug = true;
		TMD2725X_obj->tmd2725_chip->prx_cail = true;
		schedule_delayed_work(&TMD2725X_obj->ps_cal_poll_work, msecs_to_jiffies(100));
		APS_LOG("ps_cal_poll_work scheduled\n");
	}
	else
	{
		TMD2725X_obj->debug = false;
		TMD2725X_obj->tmd2725_chip->prx_cail= false;
		cancel_delayed_work(&TMD2725X_obj->ps_cal_poll_work);
		APS_LOG("ps_cal_poll_work cancelled\n");
	}
	return count;
}

static ssize_t TMD2725X_show_prox_calibrate_verify(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return res;
}

static ssize_t TMD2725X_store_prox_calibrate_verify(struct device_driver *ddri, const char *buf, size_t count)
{
       //ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	//res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return count;
}

static ssize_t TMD2725X_show_prox_data_safe_range_min(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_SAFE_RANGE_MIN);
	return res;
}

static ssize_t TMD2725X_show_prox_data_safe_range_max(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_SAFE_RANGE_MAX);
	return res;
}
static ssize_t TMD2725X_show_prox_offset_cal_start(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return res;
}

static ssize_t TMD2725X_store_prox_offset_cal_start(struct device_driver *ddri, const char *buf, size_t count)
{
       ssize_t res;
	int value = 0;

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}
	res = sscanf(buf, "%d", &value);

	APS_DBG("%s value = %d\n", __FUNCTION__,value);
	if (value == 1)
	{
		TMD2725X_obj->debug = true;
		TMD2725X_obj->tmd2725_chip->prx_cail = true;
		schedule_delayed_work(&TMD2725X_obj->ps_cal_poll_work, msecs_to_jiffies(100));
		APS_LOG("ps_cal_poll_work scheduled\n");
	}
	else
	{
		TMD2725X_obj->debug = false;
		TMD2725X_obj->tmd2725_chip->prx_cail = false;
		cancel_delayed_work(&TMD2725X_obj->ps_cal_poll_work);
		APS_LOG("ps_cal_poll_work cancelled\n");
	}
	return count;
}
static ssize_t TMD2725X_show_prox_offset_cal(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	//res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_obj->prox_offset);
	return res;
}

static ssize_t TMD2725X_store_prox_offset_cal(struct device_driver *ddri, const char *buf, size_t count)
{
       ssize_t res;
	 int value = 0;

	APS_DBG("@@##   %s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = sscanf(buf, "%d", &value);

	if (value == 1)
	{
		// TMD2725X_prox_offset_cal_process(); // comment by baibo
	}

	return count;
}
static ssize_t TMD2725X_show_prox_offset_cal_verify(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return res;
}

static ssize_t TMD2725X_store_prox_offset_cal_verify(struct device_driver *ddri, const char *buf, size_t count)
{
       //ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

	//res = snprintf(buf, PAGE_SIZE, "%d\n", TMD2725X_DATA_MAX);
	return count;
}
static ssize_t TMD2725X_show_prox_manual_calibrate_threshold(struct device_driver *ddri, char *buf)
{
       ssize_t res;

	APS_DBG("%s\n", __FUNCTION__);

	if(!TMD2725X_obj)
	{
		APS_ERR("TMD2725X_obj is null!!\n");
		return 0;
	}

    if (TMD2725X_prox_calibrate_is_done()){
        res = snprintf(buf, PAGE_SIZE, "%d\n", (TMD2725X_THRES_MIN+TMD2725X_THRES_MAX)/2);
    }else{
        res = snprintf(buf, PAGE_SIZE, "%d\n", 0);
    }

	return res;
}

#ifdef DEBUG_ALS_FAC
static void tmd2725_als_parameter_init(struct tmd2725_chip *chip)
{
	chip->als_cal_data.base = 1;
	chip->als_cal_data.cur = 1;
	chip->als_cal_data.flag = 0;
}

static int tmd2725_als_calibrate_work(struct tmd2725_chip *chip, const char *cal_data)
{
    int err=0;
	if (unlikely(IS_ERR_OR_NULL(cal_data))) {
		ALSPS_ERR("NULL\n");
		return -1;
	}

	/*copy mem directly instead of parse string*/
    chip->als_cal_data.base=gvalue;
	memcpy(&chip->als_cal_data.cur, cal_data, sizeof(chip->als_cal_data.cur));

	if (!chip->als_enabled) {
		err=tmd2725_configure_als_mode(chip, 1);
	if (err < 0) {
		ALSPS_ERR("disable failed.\n");
	}
	}
	err = tmd2725_get_lux(chip);
	if (err < 0) {
		ALSPS_ERR("get mean lux value error\n");
		goto als_cal_exit;
	}
	chip->als_cal_data.cur = chip->als_inf.prev_lux;
	chip->als_cal_data.cur = (chip->als_cal_data.cur > 0) ? chip->als_cal_data.cur : 1;

	ALSPS_LOG("als_cal_data.base = %d\n", chip->als_cal_data.base);
	ALSPS_LOG("als_cal_data.cur = %d\n", chip->als_cal_data.cur);
	if (chip->als_cal_data.base / chip->als_cal_data.cur > 6) {
		err = -EINVAL;
		chip->als_cal_data.flag = 0;
		ALSPS_ERR("calibrate param invalid; data->rgb_cal_data.flag=%d\n",
				chip->als_cal_data.flag);
		goto als_cal_exit;
	}

	chip->als_cal_data.flag = (chip->als_cal_data.base > 0) ? 1 : 0;

	if (chip->als_cal_data.flag) {
		chip->als_inf.cal_lux= chip->als_inf.prev_lux * SCALE_FACTOR(chip->als_cal_data.base,
			chip->als_cal_data.cur);
	} else {
		tmd2725_als_parameter_init(chip);
	}

    ALSPS_LOG("rgb_cal_data.flag = %d\n", chip->als_cal_data.flag);

	err = sensor_write_file(ALS_CAL_PATH,
								(const char *)&(chip->als_cal_data),
								sizeof(struct tmd2725_als_cal_data));
	if (err < 0) {
		ALSPS_ERR("save rgb cal parameters failed\n");
		goto als_cal_exit;
	}

als_cal_exit:
	err=tmd2725_configure_als_mode(chip, chip->als_enabled);
	if (err < 0) {
		ALSPS_ERR("disable failed.\n");
	}
	return err;

}

static ssize_t tmd2725_als_fac_calibrate_show(struct device_driver *dev, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
    ssize_t res;

	res = sensor_read_file(ALS_CAL_PATH,(char *)&(chip->als_cal_data),sizeof(struct tmd2725_als_cal_data));

    if (res < 0) {
        ALSPS_ERR("read rgbc calibration value failed\n");
        return res;
    }
    res = snprintf(buf, PAGE_SIZE, "base : %d, cur : %d, flag : %d\n",
                            chip->als_cal_data.base,chip->als_cal_data.cur,chip->als_cal_data.flag);
    return res;
}

static ssize_t tmd2725_als_fac_calibrate_store(struct device_driver *dev,
				const char *buf, size_t size)
{
	int err;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	if (IS_ERR_OR_NULL(buf)) {
		ALSPS_ERR("NULL.\n");
		return -EINVAL;
	}
    sscanf(buf, "%d", &gvalue);
	err = tmd2725_als_calibrate_work(chip, buf);

	if (err < 0) {
		ALSPS_ERR("als calibrate work failed.\n");
	}
	return size;
}

ssize_t TMD2725X_show_light_value(struct device_driver *ddri, char *buf)
{
	ssize_t res=0;
	AMS_MUTEX_LOCK(&g_TMD2725X_ptr->tmd2725_chip->lock);
	tmd2725_read_als(g_TMD2725X_ptr->tmd2725_chip);
	tmd2725_get_lux(g_TMD2725X_ptr->tmd2725_chip);
	AMS_MUTEX_UNLOCK(&g_TMD2725X_ptr->tmd2725_chip->lock);

	/*res = sprintf(buf, "%d", (int)(g_TMD2725X_ptr->tmd2725_chip->als_inf.lux+((g_TMD2725X_ptr->tmd2725_chip->als_inf.lux*7)/10)));*/
	res = sprintf(buf, "%d",g_TMD2725X_ptr->tmd2725_chip->als_inf.cal_lux);
	return res;
}
ssize_t TMD2725X_show_light_chip_name(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!g_TMD2725X_ptr)
	{
		ALSPS_ERR("tmd2725x_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "%s\n", TMD2725X_CHIP_NAME);
	return res;
}

ssize_t TMD2725X_show_tp_cfg_color(struct device_driver *ddri, char *buf)
{
    ssize_t res=0;

    flag_is_set_tp_info=true;
	return res;
}

ssize_t TMD2725X_set_light_calibration(struct device_driver *ddri, const char *buf, size_t count)
{
    ssize_t res=0;
	return res;
}
#endif

static DRIVER_ATTR(chip_name,                           S_IRUGO, TMD2725X_show_chip_name, NULL);
static DRIVER_ATTR(prox_thres_min,                      S_IRUGO, TMD2725X_show_prox_thres_min, NULL);
static DRIVER_ATTR(prox_manual_calibrate_threshold,     S_IRUGO, TMD2725X_show_prox_manual_calibrate_threshold, NULL);
static DRIVER_ATTR(prox_thres_max,                      S_IRUGO, TMD2725X_show_prox_thres_max, NULL);
static DRIVER_ATTR(prox_data_max,                       S_IRUGO, TMD2725X_show_prox_data_max, NULL);
static DRIVER_ATTR(prox_data_safe_range_min,            S_IRUGO, TMD2725X_show_prox_data_safe_range_min, NULL);
static DRIVER_ATTR(prox_data_safe_range_max,            S_IRUGO, TMD2725X_show_prox_data_safe_range_max, NULL);

static DRIVER_ATTR(prox_thres,                          S_IWUSR | S_IRUGO, TMD2725X_show_prox_thres, TMD2725X_store_prox_thres);
static DRIVER_ATTR(prox_calibrate_start,                S_IWUSR | S_IRUGO, TMD2725X_show_prox_calibrate_start, TMD2725X_store_prox_calibrate_start);
static DRIVER_ATTR(prox_calibrate_verify,               S_IWUSR | S_IRUGO, TMD2725X_show_prox_calibrate_verify, TMD2725X_store_prox_calibrate_verify);
static DRIVER_ATTR(prox_offset_cal_start,               S_IWUSR | S_IRUGO, TMD2725X_show_prox_offset_cal_start, TMD2725X_store_prox_offset_cal_start);
static DRIVER_ATTR(prox_offset_cal,                     S_IWUSR | S_IRUGO, TMD2725X_show_prox_offset_cal, TMD2725X_store_prox_offset_cal);
static DRIVER_ATTR(prox_offset_cal_verify,              S_IWUSR | S_IRUGO, TMD2725X_show_prox_offset_cal_verify, TMD2725X_store_prox_offset_cal_verify);

#ifdef DEBUG_ALS_FAC
static DRIVER_ATTR(light_value,                         S_IRUGO, TMD2725X_show_light_value, NULL);
static DRIVER_ATTR(fac_calibrate,                       S_IWUSR | S_IRUGO, tmd2725_als_fac_calibrate_show,tmd2725_als_fac_calibrate_store);
static DRIVER_ATTR(light_chip_name,                     S_IRUGO, TMD2725X_show_light_chip_name, NULL);
static DRIVER_ATTR(tp_cfg,                              S_IWUSR | S_IRUGO, TMD2725X_show_tp_cfg_color, TMD2725X_set_light_calibration);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *TMD2725X_attr_list[] = {
	&driver_attr_chip_name,
	&driver_attr_prox_thres_min,
	&driver_attr_prox_thres_max,
	&driver_attr_prox_thres,
	&driver_attr_prox_data_max,
	&driver_attr_prox_calibrate_start,
	&driver_attr_prox_calibrate_verify,
	&driver_attr_prox_data_safe_range_min,
	&driver_attr_prox_data_safe_range_max,
	&driver_attr_prox_offset_cal_start,
	&driver_attr_prox_offset_cal,
	&driver_attr_prox_offset_cal_verify,
	&driver_attr_prox_manual_calibrate_threshold,
#ifdef DEBUG_ALS_FAC
    &driver_attr_light_value,
    &driver_attr_fac_calibrate,
    &driver_attr_light_chip_name,
    &driver_attr_tp_cfg,
#endif
};

/*----------------------------------------------------------------------------*/
static int TMD2725X_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(TMD2725X_attr_list)/sizeof(TMD2725X_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, TMD2725X_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", TMD2725X_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
	static int TMD2725X_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(TMD2725X_attr_list)/sizeof(TMD2725X_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, TMD2725X_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
#endif  /*FACTORY_MACRO_PS*/


irqreturn_t tmd2725_irq(int irq, void *handle)
{
	struct TMD2725X_priv *obj = g_TMD2725X_ptr;

	if (!obj)
		return IRQ_HANDLED;

	disable_irq_nosync(alsps_irq);
	int_top_time = sched_clock();
	schedule_work(&obj->irq_work);
	return IRQ_HANDLED;
}

/*by qliu4244 add sensors function start*/
int TMD2725X_irq_registration(struct i2c_client *client)
{
	int ints[2] = {0};
	int err = 0;
	struct TMD2725X_priv *obj = i2c_get_clientdata(client);

#ifdef CONFIG_OF
	if(obj == NULL){
		APS_ERR("TMD2725X_obj is null!\n");
		return -EINVAL;
	}
	g_TMD2725X_ptr = obj;
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
#endif
	/*configure to GPIO function, external interrupt*/
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, 1);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef CONFIG_OF
	if(obj->irq_node != NULL){
		of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		APS_LOG("ins[0] = %d, ints[1] = %d\n", ints[0], ints[1]);
		mt_gpio_set_debounce(ints[0], ints[1]+1);

		alsps_irq = irq_of_parse_and_map(obj->irq_node, 0);
		if(alsps_irq != 0){
			err = request_irq(alsps_irq, tmd2725_irq, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);
			if(err < 0){
				APS_ERR("request_irq failed!\n");
				return -EFAULT;
			}else{
				disable_irq(alsps_irq);
				msleep(20);
				enable_irq(alsps_irq);
			}
		}else{
			APS_ERR("irq_of_parse_and_map failed!\n");
			return -EFAULT;
		}
	}else{
		APS_ERR("TMD2725X_obj->irq_node is null!\n");
		return -EFAULT;
	}
	return 0;
#endif
}
/*by qliu4244 add sensors function end*/
/******************************************************************************
 * Function Configuration
******************************************************************************/
int TMD2725X_read_als(struct i2c_client *client, u16 *data)
{

    tmd2725_read_als(TMD2725X_obj->tmd2725_chip);
    tmd2725_get_lux(TMD2725X_obj->tmd2725_chip);
#ifdef USE_ALS_FAC
    if(TMD2725X_obj->tmd2725_chip->als_cal_data.flag)
    {
	    *data= TMD2725X_obj->tmd2725_chip->als_inf.lux * SCALE_FACTOR(TMD2725X_obj->tmd2725_chip->als_cal_data.base,TMD2725X_obj->tmd2725_chip->als_cal_data.cur);
    }
    else{
	    *data=TMD2725X_obj->tmd2725_chip->als_inf.lux;
         /*printk("aaron no fac_als chip->als_cal_data.flag:%d\n",TMD2725X_obj->tmd2725_chip->als_cal_data.flag);*/
    }
#else
	*data=TMD2725X_obj->tmd2725_chip->als_inf.lux;
#endif
     return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_get_als_value(struct TMD2725X_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

	for (idx = 0; idx < obj->als_level_num; idx++) {
		if (als < obj->hw->als_level[idx])
			break;
	}

	if (idx >= obj->als_value_num) {
		APS_ERR("TMD2725X_get_als_value exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if (1 == atomic_read(&obj->als_deb_on)) {
		unsigned long endt = atomic_read(&obj->als_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&obj->als_deb_on, 0);


		if (1 == atomic_read(&obj->als_deb_on))
			invalid = 1;

	}

	if (!invalid) {
		int level_high = obj->hw->als_level[idx];
		int level_low = (idx > 0) ? obj->hw->als_level[idx - 1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
		int value_low = (idx > 0) ? obj->hw->als_value[idx - 1] : 0;
		int value_diff = value_high - value_low;
		int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value =
				(level_diff * value_low + (als - level_low) * value_diff +
				 ((level_diff + 1) >> 1)) / level_diff;

		 /*APS_DBG("ALS_AARON: %d [%d, %d] => %d [%d, %d]\n", als, level_low, level_high, value,value_low, value_high);*/
#ifdef DEBUG_ALS_FAC
	g_value_target_cail=value;
#endif
		return value;
		/* APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]); */
		return obj->hw->als_value[idx];
	}
	    /* APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]); */
	return -1;
}

/*----------------------------------------------------------------------------*/
long TMD2725X_read_ps(struct i2c_client *client, u16 *data)
{
	 tmd2725_read_prox(TMD2725X_obj->tmd2725_chip);

	 *data =TMD2725X_obj->tmd2725_chip->prx_inf.raw;

	 printk("tmd2725_PSDATA:%d\n",*data);
     return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_get_ps_value(struct TMD2725X_priv *obj, u16 ps)
{
	if (1 == atomic_read(&TMD2725X_obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&TMD2725X_obj->ps_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&TMD2725X_obj->ps_deb_on, 0);

	}
   tmd2725_get_prox(TMD2725X_obj->tmd2725_chip);
#ifdef FACTORY_MACRO_PS
		if (obj->debug == true)
		{
			return (int)ps;
		}
		else
		{
	        return TMD2725X_obj->tmd2725_chip->prx_inf.detected;
		}
#else
   return TMD2725X_obj->tmd2725_chip->prx_inf.detected;
#endif
}


#ifdef FACTORY_MACRO_PS
/*----------------------------------------------------------------------------*/
static void TMD2725X_cal_poll_work(struct work_struct *work)
{
	struct TMD2725X_priv *obj = TMD2725X_obj;
	int err;

	TMD2725X_read_ps(obj->client, &obj->ps);
	APS_LOG("@@## TMD2725X_cal_poll_work rawdata ps=%d!\n",obj->ps);

	err = ps_report_interrupt_data(obj->ps);
	if (err)
		APS_ERR("call ps_report_interrupt_data fail = %d\n", err);

	schedule_delayed_work(&TMD2725X_obj->ps_cal_poll_work, msecs_to_jiffies(100));
}
#endif /*FACTORY_MACRO_PS*/

/*----------------------------------------------------------------------------*/
/*for interrupt work mode support*/
static void TMD2725X_irq_work(struct work_struct *work)
{
	struct TMD2725X_priv *obj = (struct TMD2725X_priv *)container_of(work, struct TMD2725X_priv, irq_work);

	tmd2725_irq_handler(obj->tmd2725_chip);

	enable_irq(alsps_irq);
	return;
}


/******************************************************************************
 * Function Configuration
******************************************************************************/
static int TMD2725X_open(struct inode *inode, struct file *file)
{
	file->private_data = TMD2725X_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int set_psensor_threshold(struct i2c_client *client)
{
   tmd2725_init_prox_mode(TMD2725X_obj->tmd2725_chip);
   return 0;
}

/*----------------------------------------------------------------------------*/
static long TMD2725X_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct TMD2725X_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int threshold[2]={115,75};

	switch (cmd) {
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = TMD2725X_enable_ps(obj->client, 1);
			if (err) {
				APS_ERR("enable ps fail: %ld\n", err);
				goto err_out;
			}

			set_bit(CMC_BIT_PS, &obj->enable);
		} else {
			err = TMD2725X_enable_ps(obj->client, 0);
			if (err) {
				APS_ERR("disable ps fail: %ld\n", err);
				goto err_out;
			}

			clear_bit(CMC_BIT_PS, &obj->enable);
		}
		break;

	case ALSPS_GET_PS_MODE:
		enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_DATA:
		err = TMD2725X_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;


		dat = TMD2725X_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_RAW_DATA:
		err = TMD2725X_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;


		dat = obj->ps;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		if (enable) {
			err = TMD2725X_enable_als(obj->client, 1);
			if (err) {
				APS_ERR("enable als fail: %ld\n", err);
				goto err_out;
			}
			set_bit(CMC_BIT_ALS, &obj->enable);
		} else {
			err = TMD2725X_enable_als(obj->client, 0);
			if (err) {
				APS_ERR("disable als fail: %ld\n", err);
				goto err_out;
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
		}
		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_DATA:
		err = TMD2725X_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;


		dat = TMD2725X_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		err = TMD2725X_read_als(obj->client, &obj->als);
		if (err)
			goto err_out;


		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
		/*----------------------------------for factory mode test---------------------------------------*/
	case ALSPS_GET_PS_TEST_RESULT:
		err = TMD2725X_read_ps(obj->client, &obj->ps);
		if (err)
			goto err_out;

		if (obj->ps > atomic_read(&obj->ps_thd_val_high))
			ps_result = 0;
		else
			ps_result = 1;

		if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&dat, ptr, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_IOCTL_GET_CALI:
		break;

	case ALSPS_IOCTL_SET_CALI:
		break;

	case ALSPS_SET_PS_THRESHOLD:
		if (copy_from_user(threshold, ptr, sizeof(threshold))) {
			err = -EFAULT;
			goto err_out;
		}
		APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],
			threshold[1]);
		atomic_set(&obj->ps_thd_val_high, 115);
		atomic_set(&obj->ps_thd_val_low, 75);	/* need to confirm */

		set_psensor_threshold(obj->client);

		break;

	case ALSPS_GET_PS_THRESHOLD_HIGH:
		APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_THRESHOLD_LOW:
		APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[1]);
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[1]))) {
			err = -EFAULT;
			goto err_out;
		}
		break;
			/*------------------------------------------------------------------------------------------*/
	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

 err_out:
	return err;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_COMPAT
static long TMD2725X_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_MODE, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_MODE, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_DATA, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_RAW_DATA, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_SET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_ALS_MODE, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_MODE, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_ALS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_DATA, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_RAW_DATA, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_TEST_RESULT:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_TEST_RESULT, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_IOCTL_CLR_CALI:
		err = file->f_op->unlocked_ioctl(file, ALSPS_IOCTL_CLR_CALI, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_IOCTL_GET_CALI:
		err = file->f_op->unlocked_ioctl(file, ALSPS_IOCTL_GET_CALI, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_IOCTL_SET_CALI:
		err = file->f_op->unlocked_ioctl(file, ALSPS_IOCTL_SET_CALI, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_SET_PS_THRESHOLD:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_THRESHOLD, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_THRESHOLD_HIGH, (unsigned long)arg32);
		break;
	case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_THRESHOLD_LOW, (unsigned long)arg32);
		break;
	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations TMD2725X_fops = {
	.owner = THIS_MODULE,
	.open = TMD2725X_open,
	.release = TMD2725X_release,
	.unlocked_ioctl = TMD2725X_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = TMD2725X_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice TMD2725X_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &TMD2725X_fops,
};

/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
/* struct TMD2725X_priv *obj = i2c_get_clientdata(client); */
	APS_FUN();
	mt_set_gpio_pull_select(GPIO_ALS_LDO_EN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_ALS_LDO_EN, 0);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_resume(struct i2c_client *client)
{
/* struct TMD2725X_priv *obj = i2c_get_clientdata(client); */
	APS_FUN();
	mt_set_gpio_pull_select(GPIO_ALS_LDO_EN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_ALS_LDO_EN, 1);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TMD2725X_DEV_NAME);
	return 0;
}
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("TMD2725X_obj als enable value = %d\n", en);

	mutex_lock(&TMD2725X_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &TMD2725X_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &TMD2725X_obj->enable);
	mutex_unlock(&TMD2725X_mutex);
	if (!TMD2725X_obj) {
		APS_ERR("TMD2725X_obj is null!!\n");
		return -1;
	}

	res = TMD2725X_enable_als(TMD2725X_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
	struct TMD2725X_priv *obj = NULL;
	if (!TMD2725X_obj) {
		APS_ERR("TMD2725X_obj is null!!\n");
		return -1;
	}
	obj = TMD2725X_obj;
	err = TMD2725X_read_als(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
		*value = TMD2725X_get_als_value(obj, obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("TMD2725X_obj ps enable value = %d\n", en);

    /* comment by baibo 20170312
	mutex_lock(&TMD2725X_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &TMD2725X_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &TMD2725X_obj->enable);

	mutex_unlock(&TMD2725X_mutex);
	*/

	if (!TMD2725X_obj) {
		APS_ERR("TMD2725X_obj is null!!\n");
		return -1;
	}
	res = TMD2725X_enable_ps(TMD2725X_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;
	if (!TMD2725X_obj) {
		APS_ERR("TMD2725X_obj is null!!\n");
		return -1;
	}
	err = TMD2725X_read_ps(TMD2725X_obj->client, &TMD2725X_obj->ps);
	if (err)
		err = -1;
	else {
		*value = TMD2725X_get_ps_value(TMD2725X_obj, TMD2725X_obj->ps);
		if (*value < 0)
			err = -1;
		    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct TMD2725X_priv *obj;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0,num=0;
#ifdef TMD2725X_IC_API_FUNC
	u8 aid, rev, auxid;
#endif
	APS_FUN();
    /*LDO EN CONFIG START*/
    mt_set_gpio_mode(GPIO_ALS_LDO_EN , GPIO_MODE_00);
	mt_set_gpio_pull_select(GPIO_ALS_LDO_EN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_ALS_LDO_EN, 1);
    mt_set_gpio_dir(GPIO_ALS_LDO_EN , GPIO_DIR_OUT);
    msleep(5);
    /*LDO EN CONFIG END*/
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	TMD2725X_obj = obj;
	TMD2725X_obj->tmd2725_chip= &g_tmd2725_chip;
	obj->tmd2725_chip->client=client;
	obj->hw = hw;
	TMD2725X_get_addr(obj->hw, &obj->addr);

	/*for interrupt work mode support*/
	INIT_WORK(&obj->irq_work, TMD2725X_irq_work);

#ifdef FACTORY_MACRO_PS
	INIT_DELAYED_WORK(&obj->ps_cal_poll_work,TMD2725X_cal_poll_work);
#endif
	obj->client = client;
	i2c_set_clientdata(client, obj);
#ifdef	TMD2725X_IC_API_FUNC
    printk(KERN_ERR"tmd2725_get_id:%s",__func__);
    for(num=0;num<=5;num++){
        tmd2725_get_id(obj->tmd2725_chip, &aid, &rev, &auxid);
        APS_ERR("%s: device id:%02x device aux id:%02x device rev:%02x\n", __func__,aid, auxid, rev);
        if(aid==228){
            break;
        }else{
            if(num>=4){
                g_err=-6;
                err=g_err;
                APS_ERR("NO such this device or address\n");
                goto exit;
            }
         continue;
        }
    }

	tmd2725_set_defaults(obj->tmd2725_chip);
	tmd2725_flush_regs(obj->tmd2725_chip);

	err = TMD2725X_irq_registration(client);
	if (err != 0) {
		APS_ERR("registration failed: %d\n", err);
		return err;
	}
#endif

	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val, 0xC1);
	atomic_set(&obj->ps_thd_val_high, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw->ps_threshold_low);
#ifdef FACTORY_MACRO_PS
	obj->debug = false;
#endif
    // begin baibo 20170206
    obj->tmd2725_chip->prx_inf.detected = PROX_FAR;
	obj->tmd2725_chip->params.prox_thres_far = PS_THRES_FAR;
	obj->tmd2725_chip->params.prox_thres_near = PS_THRES_NEAR;
	obj->tmd2725_chip->params.prox_thres_oil_near = PS_THRES_OIL_NEAR;
	obj->tmd2725_chip->params.prox_thres_oil_far = PS_THRES_OIL_FAR;
    // end baibo 20170206

    // begin liuqiang 20170306 for als factory cail
	tmd2725_als_parameter_init(TMD2725X_obj->tmd2725_chip);
    // end   liuqiang 20170306 for als factory cail

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level) / sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value) / sizeof(obj->hw->als_value[0]);
	/*modified gain 16 to 1/5 according to actual thing */
	/* (1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value */
	/*obj->als_modulus = (400 * 100 * ZOOM_TIME) / (1 * 150);*/
	/* (400)/16*2.72 here is amplify *100 / *16 */
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);


	TMD2725X_i2c_client = client;

	err = misc_register(&TMD2725X_device);
	if (err) {
		APS_ERR("TMD2725X_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;

#ifdef FACTORY_MACRO_PS
	if((err = TMD2725X_create_attr(&(TMD2725X_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif
	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;

	als_ctl.is_support_batch = false;


	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);

#ifdef	TMD2725X_IC_API_FUNC_SYS
    mutex_init(&TMD2725X_obj->tmd2725_chip->lock);
    err=TMD2725X_create_prox_debug_attr(&client->dev);
	if (err){
		APS_ERR("register prox attr err = %d\n", err);
		goto exit_sys_create_attr_failed;

    }
    err=TMD2725X_create_als_debug_attr(&client->dev);
	if (err){
		APS_ERR("register als attr err = %d\n", err);
		goto exit_sys_create_attr_failed;
    }
#endif
#ifdef	TMD2725X_IC_API_FUNC
	ams_i2c_write_direct(TMD2725X_i2c_client, 0x80, 0x01);
	INIT_WORK(&TMD2725X_obj->tmd2725_chip->work_prox, tmd2725_prox_thread);
#endif
	TMD2725X_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

#ifdef FACTORY_MACRO_PS
exit_create_attr_failed:
#endif
#ifdef TMD2725X_IC_API_FUNC_SYS
exit_sys_create_attr_failed:
#endif
exit_sensor_obj_attach_fail:
	misc_deregister(&TMD2725X_device);
exit_misc_device_register_failed:
	kfree(obj);
exit:
	TMD2725X_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	TMD2725X_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int TMD2725X_i2c_remove(struct i2c_client *client)
{
	int err;
#ifdef FACTORY_MACRO_PS
	if((err = TMD2725X_delete_attr(&(TMD2725X_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("stk3x1x_delete_attr fail: %d\n", err);
	}
#endif

#ifdef	TMD2725X_IC_API_FUNC_SYS
    TMD2725X_delete_prox_debug_attr(&client->dev);
    TMD2725X_delete_als_debug_attr(&client->dev);
#endif
	err = misc_deregister(&TMD2725X_device);
	if (err)
		APS_ERR("misc_deregister fail: %d\n", err);
	TMD2725X_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  TMD2725X_local_init(void)
{
	APS_FUN();
    if(hw!=NULL){
	    TMD2725X_power(hw, 1);
    }
	if (i2c_add_driver(&TMD2725X_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
    if(g_err==-6)
        return g_err;
    if(TMD2725X_init_flag==-1)
        return TMD2725X_init_flag;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int TMD2725X_remove(void)
{
	APS_FUN();
	TMD2725X_power(hw, 0);
	i2c_del_driver(&TMD2725X_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init TMD2725X_init(void)
{
	const char *name = "mediatek,TMD2725X";
	APS_FUN();
	hw =   get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");
	alsps_driver_add(&TMD2725X_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit TMD2725X_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(TMD2725X_init);
module_exit(TMD2725X_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Qiang Liu");
MODULE_DESCRIPTION("TMD2725X driver");
MODULE_LICENSE("GPL");
