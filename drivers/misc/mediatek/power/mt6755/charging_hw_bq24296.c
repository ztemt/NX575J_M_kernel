/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
 * Revision:   1.0
 * Modtime:   11 Aug 2005 10:28:16
 * Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
 
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_meter.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq24296.h"
#include <mach/mt_sleep.h>
#include <kd_flashlight.h>

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
unsigned int wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
unsigned int wireless_charger_gpio_number = 0;
#endif

#endif

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
#endif

kal_bool charging_type_det_done = KAL_TRUE;

const unsigned int VBAT_CV_VTH[] = {
	3504000, 3520000, 3536000, 3552000,
	3568000, 3584000, 3600000, 3616000,
	3632000, 3648000, 3664000, 3680000,
	3696000, 3712000, 3728000, 3744000,
	3760000, 3776000, 3792000, 3808000,
	3824000, 3840000, 3856000, 3872000,
	3888000, 3904000, 3920000, 3936000,
	3952000, 3968000, 3984000, 4000000,
	4016000, 4032000, 4048000, 4064000,
	4080000, 4096000, 4112000, 4128000,
	4144000, 4160000, 4176000, 4192000,
	4208000, 4224000, 4240000, 4256000,
	4272000, 4288000, 4304000
};

const unsigned int CS_VTH[] = {
	51200, 57600, 64000, 70400,
	76800, 83200, 89600, 96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
};

const unsigned int INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_150_00_MA, CHARGE_CURRENT_500_00_MA,
	    CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_2000_00_MA,
	    CHARGE_CURRENT_MAX
};

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */


/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */

static unsigned int charging_error;
static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);
/* ============================================================ // */
unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	if (val < array_size) {
		return parameter[val];
	} else {
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		return parameter[0];
	}
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	unsigned int i;

	battery_log(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;

	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);      // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {/* max value in the last element */

			if (pList[i] <= level)
				return pList[i];

		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {/* max value in the first element */

			if (pList[i] <= level)
				return pList[i];

		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}
#if !defined(CONFIG_POWER_EXT)
static unsigned int is_chr_det(void)
{
	unsigned int val = 0;
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#else
	val = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);
#endif
	battery_log(BAT_LOG_CRTI, "[is_chr_det] %d\n", val);

	return val;
}
#endif

static unsigned int charging_hw_init(void *data)
{
	unsigned int status = STATUS_OK;

	bq24296_set_en_hiz(0x0);
#ifdef CONFIG_NUBIA_PM_COMMON_FEATURE
	bq24296_set_vindpm(0x8);	/* VIN DPM check 4.52V for weak charger check*/
#else
	bq24296_set_vindpm(0xA);	/* VIN DPM check 4.68V */
#endif
	bq24296_set_reg_rst(0x0);
	bq24296_set_wdt_rst(0x1);	/* Kick watchdog */
	bq24296_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	bq24296_set_iprechg(0x3);	/* Precharge current 512mA */
	bq24296_set_iterm(0x0);	/* Termination current 128mA */

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
  #ifdef CONFIG_NUBIA_PM_COMMON_FEATURE
			 bq24296_set_vreg(0xfe);	/* VREG 4.400V */
  #else
		   bq24296_set_vreg(0x32);	/* VREG 4.304V */
  #endif
#else
	bq24296_set_vreg(0x2C);	/* VREG 4.208V */
#endif

	bq24296_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq24296_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq24296_set_en_term(0x1);	/* Enable termination */
	bq24296_set_watchdog(0x1);	/* WDT 40s */
	bq24296_set_en_timer(0x0);	/* Disable charge timer */
	bq24296_set_int_mask(0x0);	/* Disable fault interrupt */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
	}
#endif
	return status;
}

static unsigned int charging_dump_register(void *data)
{
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_FULL, "charging_dump_register\r\n");
	bq24296_dump_register();

	return status;
}

static unsigned int charging_enable(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int *) (data);

	if (KAL_TRUE == enable) {
		bq24296_set_en_hiz(0x0);
		bq24296_set_chg_config(0x1);	/* charger enable */
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
if (mt_usb_is_device())
#endif
		{
			bq24296_set_chg_config(0x0);
			if (charging_get_error_state()) {
				pr_debug("[charging_enable] bq24296_set_hz_mode(0x1)\n");
				bq24296_set_en_hiz(0x1);	/* disable power path */
			}
		}
	}

	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int status;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	status = STATUS_OK;

	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, *(unsigned int *) data);
	register_value =
	    charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), set_cv_voltage);
	battery_log(BAT_LOG_CRTI, "charging_set_cv_voltage register_value=0x%x %d %d\n",
		    register_value, *(unsigned int *) data, set_cv_voltage);
	bq24296_set_vreg(register_value);

	return status;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned char ret_val = 0;
	unsigned char ret_force_20pct = 0;

	/* Get current level */
	bq24296_read_interface(bq24296_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	/* Get Force 20% option */
	bq24296_read_interface(bq24296_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK,
			       CON2_FORCE_20PCT_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 64) + 512;

	if (ret_force_20pct == 0) {
		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
		*(unsigned int *) data = ret_val;
	} else {
		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH_20PCT); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
		/* return (int)(ret_val<<1)/10; */
		*(unsigned int *) data = (int)(ret_val << 1) / 10;
	}

	return status;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *) data;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
	bq24296_set_ichg(register_value);

	return status;
}

static unsigned int charging_set_input_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int current_value = *(unsigned int *) data;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size, set_chr_current);

	bq24296_set_iinlim(register_value);

	return status;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int ret_val;

	ret_val = bq24296_get_chrg_stat();

	if (ret_val == 0x3)
		*(unsigned int *) data = KAL_TRUE;
	else
		*(unsigned int *) data = KAL_FALSE;

	return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_reset_watch_dog_timer\r\n");

	bq24296_set_wdt_rst(0x1);	/* Kick watchdog */

	return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short int register_value;
	unsigned int voltage = *(unsigned int *) (data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
#else
	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_VTH, register_value);
#endif

	return status;
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#else
	*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_VCDT_HV_DET);
#endif

	return status;
}

static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	battery_log(BAT_LOG_CRTI, "bat exist for evb\n");
#else
	unsigned int val = 0;

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#else
	val = pmic_get_register_value(MT6351_PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(MT6351_PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(MT6351_PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif
#endif
	return status;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 1;
	battery_log(BAT_LOG_CRTI, "chr exist for fpga\n");
#else
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	*(kal_bool *) (data) = pmic_get_register_value_nolock(PMIC_RGS_CHRDET);
#else
	*(kal_bool *) (data) = pmic_get_register_value_nolock(MT6351_PMIC_RGS_CHRDET);
#endif
#endif
	return status;
}

kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	int wireless_state = 0;

	if (wireless_charger_gpio_number != 0) {
#ifdef CONFIG_MTK_LEGACY
		wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
#else
/*K.S. way here*/
#endif
		if (wireless_state == WIRELESS_CHARGER_EXIST_STATE) {
			*(CHARGER_TYPE *) (data) = WIRELESS_CHARGER;
			battery_log(BAT_LOG_CRTI, "WIRELESS_CHARGER!\n");
			return status;
		}
	} else {
		battery_log(BAT_LOG_CRTI, "wireless_charger_gpio_number=%d\n", wireless_charger_gpio_number);
	}

	if (g_charger_type != CHARGER_UNKNOWN && g_charger_type != WIRELESS_CHARGER) {
		*(CHARGER_TYPE *) (data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "return %d!\n", g_charger_type);
		return status;
	}
#endif

	if (is_chr_det() == 0) {
		g_charger_type = CHARGER_UNKNOWN;
		*(CHARGER_TYPE *) (data) = CHARGER_UNKNOWN;
		battery_log(BAT_LOG_CRTI, "[charging_get_charger_type] return CHARGER_UNKNOWN\n");
		return status;
	}

	charging_type_det_done = KAL_FALSE;
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
	charging_type_det_done = KAL_TRUE;
	g_charger_type = *(CHARGER_TYPE *) (data);

#endif

	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = KAL_FALSE;
#else

	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif

	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");
	kernel_restart("battery service reboot system");
#endif

	return status;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	/* close flashlight */
	checkAndRelease();
	/*added dump_stack to see who the caller is */
	dump_stack();
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

#if 0				/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *) data = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	unsigned int increase = *(unsigned int *) (data);
	unsigned int charging_status = KAL_FALSE;

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_340000_V;
#else
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
#endif

	charging_get_charging_status(&charging_status);
	if (KAL_FALSE == charging_status) {
		charging_set_cv_voltage(&cv_voltage);	/* Set CV */
		bq24296_set_ichg(0x0);	/* Set charging current 500ma */
		bq24296_set_chg_config(0x1);	/* Enable Charging */
	}

	if (increase == KAL_TRUE) {
		bq24296_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 1");
		msleep(85);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 1");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 2");
		msleep(85);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 2");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 3");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 3");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 4");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 4");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 5");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 5");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 6");
		msleep(485);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 6");
		msleep(50);

		pr_debug("mtk_ta_increase() end\n");

		bq24296_set_iinlim(0x2);	/* 500mA */
		msleep(200);
	} else {
		bq24296_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 1");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 1");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 2");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 2");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 3");
		msleep(281);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 3");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 4");
		msleep(85);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 4");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 5");
		msleep(85);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 5");
		msleep(85);

		bq24296_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 6");
		msleep(485);

		bq24296_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 6");
		msleep(50);

		pr_debug("mtk_ta_decrease() end\n");

		bq24296_set_iinlim(0x2);	/* 500mA */
	}

	return STATUS_OK;
}

static unsigned int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_get_diso_state(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}


static unsigned int charging_get_error_state(void)
{
	return charging_error;
}

static unsigned int charging_set_error_state(void *data)
{
	unsigned int status = STATUS_OK;
	charging_error = *(unsigned int *) (data);

	return status;
}

static unsigned int (*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init, charging_dump_register, charging_enable, charging_set_cv_voltage,
	    charging_get_current, charging_set_current, charging_set_input_current,
	    charging_get_charging_status, charging_reset_watch_dog_timer,
	    charging_set_hv_threshold, charging_get_hv_status, charging_get_battery_status,
	    charging_get_charger_det_status, charging_get_charger_type,
	    charging_get_is_pcm_timer_trigger, charging_set_platform_reset,
	    charging_get_platform_boot_mode, charging_set_power_off,
	    charging_get_power_source, charging_get_csdac_full_flag,
	    charging_set_ta_current_pattern, charging_set_error_state, charging_diso_init,
	    charging_get_diso_state};

/*
* FUNCTION
*		Internal_chr_control_handler
*
* DESCRIPTION
*		 This function is called to set the charger hw
*
* CALLS
*
* PARAMETERS
*		None
*
* RETURNS
*
*
* GLOBALS AFFECTED
*	   None
*/
int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	int status;

	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL)
			status = charging_func[cmd](data);
		else {
			battery_log(BAT_LOG_CRTI, "[chr_control_interface]cmd:%d not supported\n", cmd);
			status = STATUS_UNSUPPORTED;
		}
	} else
		status = STATUS_UNSUPPORTED;

	return status;
}
