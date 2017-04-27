/* Copyright (c) 2016, NUBIA corporation department 2 section 3 wifi group xiaofeng. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __NUBIA_LED_DRV_H__
#define __NUBIA_LED_DRV_H__

/****************************************************************************
 * structures
 ***************************************************************************/
struct nubia_breathlight_control_data {
	struct led_classdev cdev;
	struct work_struct work;

	//add pmic-specific parameters
	/*
	int 32k_ck_pdn;	//pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down
	int ck_pdn;		//pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
	int ck_cksel;	       //pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
	
	int ch_mode;
	int ch_step;
	int ch_en;

	int t_rising_1;
	int t_rising_2;
	int t_falling_1;
	int t_falling_2;
	int t_on;
	int t_off;
	int dim_duty;
	int dim_fsel;
	int ch_bias_en;
	int chop_en;
	*/
	
    int Rise_Fall_time; // intergrated as a ingrater.
	int Hold_time;
	int Off_time;
	
	int min_grade;
    int max_grade;

    int breath_outn;
	int outn;
	int fade_parameter;
	int grade_parameter;
	int blink_mode;
};

#endif /* __NUBIA_LED_DRV_H__ */

