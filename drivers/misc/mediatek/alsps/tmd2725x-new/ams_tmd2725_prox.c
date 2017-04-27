/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* proximity detection (prox) functionality within the
* AMS-TAOS TMD2725 family of devices.
*/

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#include "ams_tmd2725_ic.h"
#include "ams_i2c.h"

#define PRX_DBG // midify baibo 20170206


// local function prototypes
void tmd2725_init_prox_mode(struct tmd2725_chip *chip);
int tmd2725_offset_calibration(struct tmd2725_chip *chip);
void tmd2725_init_prox_first_report_far_event(struct tmd2725_chip *chip); // add baibo 20170210


void tmd2725_do_prox_state(struct tmd2725_chip *chip)
{
	switch (chip->prox_state) {
	case PROX_STATE_INIT:
		tmd2725_init_prox_mode(chip);
		break;
	case PROX_STATE_CALIB:
		tmd2725_offset_calibration(chip);
		break;
	case PROX_STATE_WAIT_AND_CALIB:
		// TODO: change trigger from wait to looking for a certain
		// number of readings that are stable (delta <5 counts)
		msleep(100);

		tmd2725_offset_calibration(chip);
		tmd2725_init_prox_mode(chip);
		break;
	default:
		break;
	}

	chip->prox_state = PROX_STATE_NONE;
}

void tmd2725_prox_thread(struct work_struct *work)
{
	struct tmd2725_chip *chip
		= container_of(work, struct tmd2725_chip, work_prox);

	tmd2725_do_prox_state(chip);
}

void tmd2725_schedule_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state)
{
	chip->prox_state = prox_state;
	schedule_work(&chip->work_prox);
}

void tmd2725_do_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state)
{
	chip->prox_state = prox_state;
	tmd2725_do_prox_state(chip);
}

/**************************/
/* General Prox Functions */
/**************************/

void tmd2725_read_prox(struct tmd2725_chip *chip)
{
	ams_i2c_blk_read(chip->client, TMD2725_REG_PDATA,
			&chip->shadow[TMD2725_REG_PDATA], 1);
	chip->prx_inf.raw = chip->shadow[TMD2725_REG_PDATA];
}


void tmd2725_get_prox(struct tmd2725_chip *chip)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;
	bool update_thresholds = false;
	bool update_offset = false;

#ifdef PRX_DBG
	dev_info(&chip->client->dev,
		"%s:detected:%d raw:%d min:%d max:%d\n",
		__func__, chip->prx_inf.detected, chip->prx_inf.raw,
		chip->params.prox_th_min, chip->params.prox_th_max);
    dev_info(&chip->client->dev,
        "prox_thres_far:%d prox_thres_near:%d prox_thres_oil_far:%d prox_thres_oil_near:%d\n",
        chip->params.prox_thres_far,
        chip->params.prox_thres_near,
        chip->params.prox_thres_oil_far,
        chip->params.prox_thres_oil_near);
#endif

	if (chip->prox_state != PROX_STATE_NONE) {
#ifdef PRX_DBG
		dev_info(&chip->client->dev,
			"%s: prox state is %d, calibration is pending.\n",
			__func__, (int) chip->prox_state);
#endif
		return;
	}

	if (chip->prx_inf.detected == PROX_FAR) {//FAR
		if (chip->prx_inf.raw > chip->params.prox_th_max) {
			dev_info(&chip->client->dev, "%s: prox detect\n", __func__);
			if (chip->prox_state == PROX_STATE_NONE) {
			    chip->prx_inf.detected = PROX_NEAR;
			    chip->params.prox_oil_state = OIL_NONE;
			    dev_info(&chip->client->dev, "!!FAR-->NEAR\n");
				chip->params.prox_th_min = chip->params.prox_thres_far;
				chip->params.prox_th_max = chip->params.prox_thres_oil_near;
				update_thresholds = true;
			}
		}

		if (chip->prx_inf.raw < chip->params.prox_th_min) {
			if (chip->prox_state == PROX_STATE_NONE) {
				dev_info(&chip->client->dev, "!!FAR-->FAR\n");
				chip->prx_inf.detected = PROX_FAR;
				chip->params.prox_th_min = 0;
				chip->params.prox_th_max = chip->params.prox_thres_near;
				update_thresholds = true;
				if (chip->params.prox_oil_state) {
					dev_info(&chip->client->dev, "!!OIL FAR-->FAR\n");
				    chip->params.prox_oil_state = OIL_NONE;
				    tmd2725_schedule_prox_work(chip, PROX_STATE_CALIB);
				}
			}
		}
	} else if (chip->prx_inf.detected == PROX_NEAR){//NEAR
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
			dev_info(&chip->client->dev, "%s: prox release\n", __func__);

			if (chip->prox_state == PROX_STATE_NONE) {
			    chip->prx_inf.detected = PROX_FAR;
				if (chip->params.prox_th_max != TMD2725X_DATA_MAX) {
					//chip->params.poffset = min(chip->params.poffset + 10, 255);
					//update_offset = true;
					//tmd2725_schedule_prox_work(chip, PROX_STATE_INIT);
					update_thresholds = true;
					dev_info(&chip->client->dev, "!!NEAR-->FAR\n");
					chip->params.prox_th_min = 0;
					chip->params.prox_th_max = chip->params.prox_thres_near;
				} else {
					//tmd2725_schedule_prox_work(chip, PROX_STATE_WAIT_AND_CALIB);
					update_thresholds = true;
					dev_info(&chip->client->dev, "!!OIL NEAR-->FAR\n");
					chip->params.prox_th_min = chip->params.prox_thres_near - 10;
					chip->params.prox_th_max = chip->params.prox_thres_oil_far + 10;
				}
			}
		} else if (chip->prx_inf.raw > chip->params.prox_th_max) {
			if (chip->prox_state == PROX_STATE_NONE) {
				chip->prx_inf.detected = PROX_NEAR;
				chip->params.prox_oil_state = OIL_STATE;
				dev_info(&chip->client->dev, "!!OIL NEAR-->NEAR\n");
				chip->params.prox_th_min = chip->params.prox_thres_oil_far;
				chip->params.prox_th_max = TMD2725X_DATA_MAX;
				update_thresholds = true;
			}
		}
	}

	if (update_thresholds) {
		dev_info(&chip->client->dev, "%s:update_thresholds: min:%d max:%d\n", __func__,
			chip->params.prox_th_min, chip->params.prox_th_max);
		ams_i2c_write(client, sh, TMD2725_REG_PILT,
				chip->params.prox_th_min & 0xff);
		ams_i2c_write(client, sh, TMD2725_REG_PIHT,
				chip->params.prox_th_max & 0xff);
	}
	if (update_offset) {
		dev_info(&chip->client->dev, "%s:update_offset: %d\n", __func__,
			chip->params.poffset);
		if(chip->params.poffset <= 0){
		ams_i2c_write(client, sh, TMD2725_REG_POFFSET_L,
				chip->params.poffset * (-1));
		}
		else{
		ams_i2c_write(client, sh, TMD2725_REG_POFFSET_L,
				chip->params.poffset );
		}
		ams_i2c_write(client, sh, TMD2725_REG_POFFSET_H,
				chip->params.poffset < 0 ? 1 : 0);
	}
}

void tmd2725_report_prox(struct tmd2725_chip *chip)
{
	if (chip->p_idev) {
		if (chip->prox_state == PROX_STATE_NONE) {
			input_report_abs(chip->p_idev, ABS_DISTANCE,
					chip->prx_inf.detected ? 0 : 1);
			input_sync(chip->p_idev);
		}
	}
}

void tmd2725_set_prox_mode(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	u8 rPGCFG0;
	u8 rPGCFG1;
	u16 lux = chip->als_inf.lux;

	if (lux <= INDOOR_LUX_TRIGGER) // Indoor light conditions
	{
		chip->amsIndoorMode = true;
		rPGCFG0 = PG_PULSE_16US | 4;       // Pulse len and # pulses
		rPGCFG1 = PGAIN_4 | PDRIVE_MA(75); // Gain and drive current
	}
	if (lux >= OUTDOOR_LUX_TRIGGER) // Outdoor (bright sun) light conditions
	{
		chip->amsIndoorMode = false;
		rPGCFG0 = PG_PULSE_4US | 22;       // Pulse len and # pulses
		rPGCFG1 = PGAIN_2 | PDRIVE_MA(75); // Gain and drive current
	}

	// If a change was made then push it to the device
	if (rPGCFG0 != sh[TMD2725_REG_PGCFG0])
		ams_i2c_write(chip->client, sh, TMD2725_REG_PGCFG0, rPGCFG0);
	if (rPGCFG1 != sh[TMD2725_REG_PGCFG1])
		ams_i2c_write(chip->client, sh, TMD2725_REG_PGCFG1, rPGCFG1);
}

int tmd2725_offset_calibration(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	u8 saveenab;
	int calwait = 0;
	int ret;

	/* save PEN state */
	ams_i2c_read(chip->client, TMD2725_REG_ENABLE, &saveenab);

	/* turn on power, disable prox/als/wait */
	ams_i2c_modify(chip->client, sh, TMD2725_REG_ENABLE,
			TMD2725_EN_ALL | TMD2725_PON,
			TMD2725_PON);

	/* enable calib intr, disable prox intr */
	ams_i2c_modify(chip->client, sh, TMD2725_REG_INTENAB,
			TMD2725_PIEN|TMD2725_CIEN, TMD2725_CIEN);

	/*
	** Prox Offset calibration
	**   binsrch_target (7 counts)
	**   prox averaging (2 reading window)
	**   prox_auto_offset_adjust
	*/
	ams_i2c_modify(chip->client, sh, TMD2725_REG_CALIBCFG,
			TMD2725_MASK_BINSRCH_TARGET |
				TMD2725_MASK_PROX_DATA_AVG |
				TMD2725_MASK_PROX_AUTO_OFFSET_ADJUST,
			(0x03 << TMD2725_SHIFT_BINSRCH_TARGET) |
				(1 << TMD2725_SHIFT_PROX_DATA_AVG) |
				(1 << TMD2725_SHIFT_PROX_AUTO_OFFSET_ADJUST));

	/* trigger calibration sequence */
	dev_info(&chip->client->dev, "offset calibration started.\n");
	chip->amsCalComplete = false;

	ams_i2c_modify(chip->client, sh, TMD2725_REG_CALIB,
			TMD2725_MASK_START_OFFSET_CALIB,
			0x01 << TMD2725_SHIFT_START_OFFSET_CALIB);

	/* amsCalComplete set true in IRQ Handler */
	for (calwait = 0; (!chip->amsCalComplete) && (calwait < 10); calwait++) {
		// TODO: change to signal and eliminate amsCalComplete & sleep
		msleep(10);
	}

	if (calwait < 10) {
		ret = 0;
	} else {
		dev_err(&chip->client->dev, "%s: No Calibration IRQ, exiting spin loop\n", __func__);
		ret = -1;
	}

	// get updated prox offset
	ams_i2c_blk_read(chip->client, TMD2725_REG_POFFSET_L,
			&chip->shadow[TMD2725_REG_POFFSET_L], 2);
	chip->params.poffset = chip->shadow[TMD2725_REG_POFFSET_L];
	if (chip->shadow[TMD2725_REG_POFFSET_H] & TMD2725_MASK_POFFSET_H) {
		chip->params.poffset *= -1;
	}
	/* enablecalib intr prox intr, disable calib intr */
	ams_i2c_modify(chip->client, sh, TMD2725_REG_INTENAB,
			TMD2725_PIEN|TMD2725_CIEN, TMD2725_PIEN);

	ams_i2c_modify(chip->client, sh, TMD2725_REG_ENABLE,
			TMD2725_EN_ALL, saveenab);

	dev_info(&chip->client->dev, "@@##  Optical Crosstalk calibration complete offset = %d.\n", chip->params.poffset);

	return ret;
}

void tmd2725_init_prox_mode(struct tmd2725_chip *chip)
{
	chip->params.prox_th_min = 0;
	//chip->params.prox_th_max = 115;
	chip->params.prox_th_max = chip->params.prox_thres_near; // modify baibo 20170206

    dev_info(&chip->client->dev, "%s:update_thresholds: min:%d max:%d\n",__func__,chip->params.prox_th_min, chip->params.prox_th_max);

	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PILT,
			chip->params.prox_th_min);
	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PIHT,
			chip->params.prox_th_max);
}

void tmd2725_init_prox_first_report_far_event(struct tmd2725_chip *chip)
{
	chip->params.prox_th_min = PS_THRES_FIRST_REPOST_FAR_EVENT;
	chip->params.prox_th_max = PS_THRES_FIRST_REPOST_FAR_EVENT;

    dev_info(&chip->client->dev, "%s:update_thresholds: min:%d max:%d\n",__func__,chip->params.prox_th_min, chip->params.prox_th_max);

	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PILT,
			chip->params.prox_th_min);
	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PIHT,
			chip->params.prox_th_max);
}


int tmd2725_configure_prox_mode(struct tmd2725_chip *chip, u8 state)
{
	extern void tmd2725_reg_log(struct tmd2725_chip *chip);
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) // Turning on prox
	{
		// Configure default proximity settings
		tmd2725_init_prox_mode(chip);

		tmd2725_init_prox_first_report_far_event(chip); // add baibo 20170210

		ams_i2c_modify(client, sh, TMD2725_REG_PERS,
				TMD2725_MASK_PROX_PERS,
				chip->params.persist & TMD2725_MASK_PROX_PERS);
		ams_i2c_write(client, sh, TMD2725_REG_PGCFG0,
				chip->params.prox_pulse_cnt |
				chip->params.prox_pulse_len);

		ams_i2c_modify(client, sh, TMD2725_REG_PGCFG1,
		        TMD2725_MASK_PGAIN|TMD2725_MASK_PDRIVE,
				(chip->params.prox_gain & TMD2725_MASK_PGAIN) |
				(chip->params.prox_drive & TMD2725_MASK_PDRIVE));

        dev_info(&chip->client->dev, "@@## tmd2725_configure_prox_mode!.\n");
		ams_i2c_write(client, sh, TMD2725_REG_PTIME, P_TIME_US(2816));

		tmd2725_offset_calibration(chip);

		// Enable Proximity and Proximity Interrupt
		ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
				TMD2725_PEN | TMD2725_PON,
				TMD2725_PEN | TMD2725_PON);
		ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
				TMD2725_PIEN, TMD2725_PIEN);


		chip->prx_enabled = true;
		chip->amsFirstProx = true;
		chip->amsIndoorMode = true;
	}
	else // Turning off prox
	{
		// Disable Proximity and Proximity Interrupt
		ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
				TMD2725_PEN , 0);
		ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
				TMD2725_PIEN, 0);

		// If nothing else is enabled set PON = 0
		if (!(sh[TMD2725_REG_ENABLE] & TMD2725_EN_ALL))
			ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
					TMD2725_PON, 0x00);

		chip->prx_enabled = false;
		chip->amsFirstProx = true;
		chip->prx_inf.detected = PROX_FAR;
	}

	return(0);
}

/*****************/
/* ABI Functions */
/*****************/
#ifdef TMD2725X_IC_API_FUNC_SYS
static ssize_t tmd2725_device_prox_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	tmd2725_read_prox(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmd2725_device_prox_detected(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	u8 val;

	// if prox intr enabled, just grab the flag that has been set
	ams_i2c_read(chip->client, TMD2725_REG_INTENAB, &val);
	if (!(val & TMD2725_PIEN)) {
		tmd2725_read_prox(chip);
		tmd2725_get_prox(chip);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmd2725_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << chip->params.prox_gain);
}

static ssize_t tmd2725_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	u8 regVal = 0;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;

	switch(gain)
	{
	case 0:
		regVal = 0;
		break;
	case 1:
		regVal = PGAIN_1;
		break;
	case 2:
		regVal = PGAIN_2;
		break;
	case 4:
		regVal = PGAIN_4;
		break;
	case 8:
		regVal = PGAIN_8;
		break;
	default:
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);
	rc = ams_i2c_modify(chip->client, chip->shadow, TMD2725_REG_PGCFG1,
			TMD2725_MASK_PGAIN, regVal);
	chip->params.prox_gain = chip->shadow[TMD2725_REG_PGCFG1] & TMD2725_MASK_PGAIN;
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2725_prox_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	u8 rPoffsetl = 0;
	u8 rPoffseth = 0;
	int prxofs = 0;

	// must read it from chip to get calibration result
	ams_i2c_read(chip->client, TMD2725_REG_POFFSET_L, &rPoffsetl);
	ams_i2c_read(chip->client, TMD2725_REG_POFFSET_H, &rPoffseth);
	prxofs = (rPoffseth & 0x01) ? -((int)rPoffsetl) : ((int)rPoffsetl);

	return snprintf(buf, PAGE_SIZE, "%d\n", prxofs);
}

static ssize_t tmd2725_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d\n",
			((chip->params.persist & 0xf0) >> 4));
}

static ssize_t tmd2725_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long persist;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_i2c_modify(chip->client, chip->shadow, TMD2725_REG_PERS, TMD2725_MASK_PROX_PERS, persist);
	chip->params.persist = chip->shadow[TMD2725_REG_PERS];
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2725_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmd2725_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long enable;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &enable);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	switch(enable)
	{
	case 0:
		// Disable prox
		tmd2725_configure_prox_mode(chip, 0);
		break;

	case 1:
		// Enable prox
		tmd2725_configure_prox_mode(chip, 1);
		break;
	default:
		return -EINVAL;
	}

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2725_prox_regs_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	u8 rEnable = 0;
	u8 rPtime = 0;
	u8 rPiltl = 0;
	u8 rPihtl = 0;
	u8 rPers = 0;
	u8 rPgcfg0 = 0;
	u8 rPgcfg1 = 0;
	u8 rCfg1 = 0;
	u8 rStatus = 0;
	u8 rCfg2 = 0;
	u8 rIntenab = 0;

	ams_i2c_read(chip->client, TMD2725_REG_ENABLE, &rEnable);
	ams_i2c_read(chip->client, TMD2725_REG_PTIME, &rPtime);
	ams_i2c_read(chip->client, TMD2725_REG_PILT, &rPiltl);
	ams_i2c_read(chip->client, TMD2725_REG_PIHT, &rPihtl);
	ams_i2c_read(chip->client, TMD2725_REG_PERS, &rPers);
	ams_i2c_read(chip->client, TMD2725_REG_PGCFG0, &rPgcfg0);
	ams_i2c_read(chip->client, TMD2725_REG_PGCFG1, &rPgcfg1);
	ams_i2c_read(chip->client, TMD2725_REG_CFG1, &rCfg1);
	ams_i2c_read(chip->client, TMD2725_REG_STATUS, &rStatus);
	ams_i2c_read(chip->client, TMD2725_REG_CFG2, &rCfg2);
	ams_i2c_read(chip->client, TMD2725_REG_INTENAB, &rIntenab);

	return snprintf(buf, PAGE_SIZE,
		"ENABLE =   %2x\nPTIME  =   %2x\nPILT   =   %2x\nPIHT   =   %2x\n"
		"PERS   =   %2x\nPGCFG0 =   %2x\nPGCFG1 =   %2x\nCFG1   =   %2x\n"
		"CFG2   =   %2x\nSTATUS =   %2x\nINTENAB=   %2x\n"
		"%s\n%s settings\n",
			rEnable,
			rPtime,
			rPiltl,
			rPihtl,
			rPers,
			rPgcfg0,
			rPgcfg1,
			rCfg1,
			rCfg2,
			rStatus,
			rIntenab,
			chip->prx_inf.detected ? "Prox Detect" : "Prox Release",
			chip->amsIndoorMode ? "Indoor" : "Outdoor");
}

DEVICE_ATTR(tmd_prx_raw,       0444, tmd2725_device_prox_raw,      NULL);
DEVICE_ATTR(tmd_prx_detect,    0444, tmd2725_device_prox_detected, NULL);
DEVICE_ATTR(tmd_prx_gain,      0644, tmd2725_prox_gain_show,      tmd2725_prox_gain_store);
DEVICE_ATTR(tmd_prx_offset,    0444, tmd2725_prox_offset_show,     NULL);
DEVICE_ATTR(tmd_prx_persist,   0644, tmd2725_prox_persist_show,    tmd2725_prox_persist_store);
DEVICE_ATTR(tmd_prx_enable,    0644, tmd2725_prox_enable_show,     tmd2725_prox_enable_store);
DEVICE_ATTR(tmd_prx_regs,      0444, tmd2725_prox_regs_show,       NULL);

static struct attribute *TMD2725X_ps_debug_attr_list[] = {
    &dev_attr_tmd_prx_raw.attr,
    &dev_attr_tmd_prx_detect.attr,
    &dev_attr_tmd_prx_gain.attr,
    &dev_attr_tmd_prx_offset.attr,
    &dev_attr_tmd_prx_persist.attr,
    &dev_attr_tmd_prx_enable.attr,
    &dev_attr_tmd_prx_regs.attr,
    NULL
};

static struct attribute_group ps_tmd_attribute_group = {
	.attrs = TMD2725X_ps_debug_attr_list
};

int TMD2725X_create_prox_debug_attr(struct device* dev)
{
	int err = 0;
	err = sysfs_create_group(&dev->kobj,&ps_tmd_attribute_group);
	if (err < 0) {
		printk(KERN_ERR"unable to create ps tmd attribute file\n");
		return -3;
	}
	return err;
}
/*----------------------------------------------------------------------------*/
int TMD2725X_delete_prox_debug_attr(struct device* dev)
{
    int err = 0;
	sysfs_remove_group(&dev->kobj,&ps_tmd_attribute_group);
    return err;
}
/*----------------------------------------------------------------------------*/
#endif
