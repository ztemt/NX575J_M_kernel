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
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "ams_tmd2725_ic.h"
#include "ams_i2c.h"

//#define LUX_DBG
#define GAIN1	0
#define GAIN4	1
#define GAIN16	2
#define GAIN64	3
#define DISABLE_IRQ_ALS_OP

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const restorable_als_regs[] = {
	TMD2725_REG_ATIME,
	TMD2725_REG_WTIME,
	TMD2725_REG_PERS,
	TMD2725_REG_CFG0,
	TMD2725_REG_CFG1,
};

static int tmd2725_flush_als_regs(struct tmd2725_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
				__func__, reg);
			break;
		}
	}

	return rc;
}

int tmd2725_read_als(struct tmd2725_chip *chip)
{
	int ret;

	ret = ams_i2c_blk_read(chip->client, TMD2725_REG_CH0DATA,
			&chip->shadow[TMD2725_REG_CH0DATA], 4 * sizeof(u8));

	if (ret >= 0) {
		chip->als_inf.als_ch0 = le16_to_cpu(
			*((const __le16 *) &chip->shadow[TMD2725_REG_CH0DATA]));
		chip->als_inf.als_ch1 = le16_to_cpu(
			*((const __le16 *) &chip->shadow[TMD2725_REG_CH1DATA]));
		ret = 0;
	}

	return ret;
}

static void tmd2725_calc_lux_coef(struct tmd2725_chip *chip)
{
	struct tmd2725_lux_segment *pls = &chip->params.lux_segment[0];

	chip->als_inf.lux1_ch0_coef = chip->params.d_factor * pls[0].ch0_coef;
	chip->als_inf.lux1_ch1_coef = chip->params.d_factor * pls[0].ch1_coef;
	chip->als_inf.lux2_ch0_coef = chip->params.d_factor * pls[1].ch0_coef;
	chip->als_inf.lux2_ch1_coef = chip->params.d_factor * pls[1].ch1_coef;
}

static void tmd2725_calc_cpl(struct tmd2725_chip *chip)//除去增益影响,adc时间影响
{
	u32 cpl;
	u32 sat;
	u8 atime;

	atime = chip->shadow[TMD2725_REG_ATIME];

	// max(cpl) == 64x * (2816 * 256) =  46,137,344 (0x02C00000)
	cpl = atime;
	cpl *= INTEGRATION_CYCLE;
	cpl *= als_gains[(chip->shadow[TMD2725_REG_CFG1] & TMD2725_MASK_AGAIN)];

	sat = min_t(u32, TMD2725_MAX_ALS_VALUE, (u32) atime << 10);
	sat = sat * 8 / 10;

	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

int tmd2725_configure_als_mode(struct tmd2725_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) // Turning on ALS
	{
		tmd2725_calc_lux_coef(chip);

		chip->shadow[TMD2725_REG_ATIME] = chip->params.als_time;
		tmd2725_calc_cpl(chip);

		/* set PERS.apers to 2 consecutive ALS values out of range */
		chip->shadow[TMD2725_REG_PERS] &= (~TMD2725_MASK_APERS);
		chip->shadow[TMD2725_REG_PERS] |= 0x02;

		tmd2725_flush_als_regs(chip);

/* comment by baibi 20170312
#ifndef DISABLE_IRQ_ALS_OP
	    ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
				TMD2725_AIEN, TMD2725_AIEN);
#endif
*/

		ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
				TMD2725_WEN | TMD2725_AEN | TMD2725_PON,
				TMD2725_WEN | TMD2725_AEN | TMD2725_PON);
		chip->als_enabled = true;
	}
	else  // Turning off ALS
	{
		// Disable ALS, Wait and ALS Interrupt

/* comment by baibi 20170312
#ifndef DISABLE_IRQ_ALS_OP
	ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
				TMD2725_AIEN, 0);
#endif
*/

		ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
				TMD2725_WEN | TMD2725_AEN, 0);
		chip->als_enabled = false;

		// If nothing else is enabled set PON = 0;
		if(!(sh[TMD2725_REG_ENABLE] & TMD2725_EN_ALL))
			ams_i2c_modify(client, sh, TMD2725_REG_ENABLE,
			TMD2725_PON, 0);
	}

	return 0;
}

static int tmd2725_set_als_gain(struct tmd2725_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg;
	//u8 saved_enable;

	switch (gain) {
	case 1:
		ctrl_reg = AGAIN_1;
		break;
	case 4:
		ctrl_reg = AGAIN_4;
		break;
	case 16:
		ctrl_reg = AGAIN_16;
		break;
	case 64:
		ctrl_reg = AGAIN_64;
		break;
	default:
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}

	// Turn off ALS, so that new ALS gain value will take effect at start of
	// new integration cycle.
	// New ALS gain value will then be used in next lux calculation.
	//ams_i2c_read(chip->client, TMD2725_REG_ENABLE, &saved_enable);
	//ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE, 0);
	rc = ams_i2c_modify(chip->client, chip->shadow, TMD2725_REG_CFG1,
			TMD2725_MASK_AGAIN, ctrl_reg);
	//ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE, saved_enable);

	if (rc >= 0) {
		chip->params.als_gain = chip->shadow[TMD2725_REG_CFG1];
		dev_info(&chip->client->dev, "%s: new als gain %d\n",
				__func__, ctrl_reg);
	}

	return rc;
}

static void tmd2725_inc_gain(struct tmd2725_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2725_REG_CFG1] & TMD2725_MASK_AGAIN);

	if (gain > GAIN16)
		return;
	else if (gain < GAIN4)
		gain = als_gains[GAIN4];
	else if (gain < GAIN16)
		gain = als_gains[GAIN16];
	else
		gain = als_gains[GAIN64];

	rc = tmd2725_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2725_calc_cpl(chip);
}

static void tmd2725_dec_gain(struct tmd2725_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2725_REG_CFG1] & 0x03);

	if (gain == GAIN1)
		return;
	else if (gain > GAIN16)
		gain = als_gains[GAIN16];
	else if (gain > GAIN4)
		gain = als_gains[GAIN4];
	else
		gain = als_gains[GAIN1];

	rc = tmd2725_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2725_calc_cpl(chip);
}

int tmd2725_get_lux(struct tmd2725_chip *chip)
{
	int64_t ch0;
	int64_t ch1;
	int64_t lux1;
	int64_t lux2;
	int64_t lux1_raw=0;
	int64_t lux2_raw=0;
	int64_t lux;

	ch0 = (int64_t)chip->als_inf.als_ch0;
	ch1 = (int64_t)chip->als_inf.als_ch1;

	/*
	**  Lux1 = 41*(Ch0-(0.26*Ch1)) / (Atime*Again)
	**  Lux2 = 41*((0.8*Ch0)-(0.27*Ch1)) / (Atime*Again)
	**  Lux = Max(Lux1,Lux2,0)
	**
	**  Lux1 = (((coef1*(ch0_coef1*1000))*Ch0)-((coef1*(ch1_coef1*1000))*Ch1)) / (Atime*1000)*Again)
	**  Lux2 = (((coef2*(ch0_coef2*1000))*Ch0)-((coef2*(ch1_coef2*1000))*Ch1)) / (Atime*1000)*Again)
	**
	**  where for default NO GLASS:
	**     coef1 = 41
	**     ch0_coef1 = 1
	**     ch1_coef1 = .26
	**     coef2 = 41
	**     ch0_coef2 = .8
	**     ch1_coef2 = .27
	*/

    lux1_raw = (int64_t)(chip->als_inf.lux1_ch0_coef * ch0) -
		       (int64_t)(chip->als_inf.lux1_ch1_coef * ch1);
    lux2_raw = (int64_t)(chip->als_inf.lux2_ch0_coef * ch0) -
		       (int64_t)(chip->als_inf.lux2_ch1_coef * ch1);

    if(lux1_raw< 0 && lux2_raw< 0){
        return (int)chip->als_inf.prev_lux; // use previous value
    }

    lux1_raw = (lux1_raw > 0) ? lux1_raw : 0;
    lux2_raw = (lux2_raw > 0) ? lux2_raw : 0;

    lux1 = lux1_raw /(int64_t)chip->als_inf.cpl;
    lux2 = lux2_raw /(int64_t)chip->als_inf.cpl;

	lux = max(lux1, lux2);
	lux = min((int64_t)TMD2725_MAX_LUX, max((int64_t)0, lux));

#ifdef LUX_DBG
	dev_info(&chip->client->dev,
		"%s: lux:%lld [%lld, %lld, %lld, %lld] %d [%d %d] [%d %d] %d [%d, ((%d*%d) / 1000)] (sat:%d)\n",
		__func__, lux,
		ch0, ch1, lux1, lux2,
		chip->params.d_factor,
		chip->params.lux_segment[0].ch0_coef,
		chip->params.lux_segment[0].ch1_coef,
		chip->params.lux_segment[1].ch0_coef,
		chip->params.lux_segment[1].ch1_coef,
		chip->als_inf.cpl,
		chip->shadow[TMD2725_REG_ATIME],
		als_gains[(chip->params.als_gain & TMD2725_MASK_AGAIN)], INTEGRATION_CYCLE,
		chip->als_inf.saturation);
#endif // #ifdef LUX_DBG

	if (lux < 0) {
		dev_info(&chip->client->dev, "%s: lux < 0 use prev.\n", __func__);
		return (int)chip->als_inf.prev_lux; // use previous value
	}

	chip->als_inf.lux = (u16)lux;
    chip->als_inf.prev_lux =(u16) lux;

	if (!chip->als_gain_auto) {
		if (ch0 <= TMD2725_MIN_ALS_VALUE) {
			dev_info(&chip->client->dev, "%s: darkness (%lld <= %d)\n",
				__func__, ch0, TMD2725_MIN_ALS_VALUE);
		} else if (ch0 >= chip->als_inf.saturation) {
			dev_info(&chip->client->dev, "%s: saturation (%lld >= %d\n",
				__func__, ch0, chip->als_inf.saturation);
		}
	} else {
		if (ch0 < 100) {
			dev_info(&chip->client->dev, "%s: AUTOGAIN INC\n", __func__);
			tmd2725_inc_gain(chip);
			//tmd2725_flush_als_regs(chip);
		} else if (ch0 >= chip->als_inf.saturation || ch1 >= chip->als_inf.saturation) {
			dev_info(&chip->client->dev, "%s: AUTOGAIN DEC\n", __func__);
			tmd2725_dec_gain(chip);
			//tmd2725_flush_als_regs(chip);
		}
	}

	return 0;
}

int tmd2725_update_als_thres(struct tmd2725_chip *chip, bool on_enable)
{
	s32 ret;
	u16 deltaP = chip->params.als_deltaP;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.als_ch0;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > (saturation / 2) ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;
	}

	*((__le16 *) &chip->shadow[TMD2725_REG_AILT]) = cpu_to_le16(from);
	*((__le16 *) &chip->shadow[TMD2725_REG_AIHT]) = cpu_to_le16(to);

	dev_info(&chip->client->dev, "%s: low:0x%x  hi:0x%x, oe:%d cur:%d deltaP:%d (%d) sat:%d\n",
		__func__, from, to, on_enable, cur, deltaP, chip->params.als_deltaP, saturation);

	ret = ams_i2c_reg_blk_write(chip->client, TMD2725_REG_AILT,
			&chip->shadow[TMD2725_REG_AILT],
			(TMD2725_REG_AIHT_HI - TMD2725_REG_AILT) + 1);

	return (ret < 0) ? ret : 0;
}

void tmd2725_report_als(struct tmd2725_chip *chip)
{
	int lux;
	int rc;

	if (chip->a_idev) {
		rc = tmd2725_get_lux(chip);
		if (!rc) {
			lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
			tmd2725_update_als_thres(chip, 0);
		} else {
			tmd2725_update_als_thres(chip, 1);
		}
	}
}

/*****************/
/* ABI Functions */
/*****************/
#ifdef TMD2725X_IC_API_FUNC_SYS
static ssize_t tmd2725_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	AMS_MUTEX_LOCK(&chip->lock);

	tmd2725_read_als(chip);
	tmd2725_get_lux(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmd2725_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	int k;

	AMS_MUTEX_LOCK(&chip->lock);

	k = snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d\n",
		chip->params.d_factor,
		chip->params.lux_segment[0].ch0_coef,
		chip->params.lux_segment[0].ch1_coef,
		chip->params.lux_segment[1].ch0_coef,
		chip->params.lux_segment[1].ch1_coef);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return k;
}

static ssize_t tmd2725_lux_table_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	u32 d_factor, ch0_coef1, ch1_coef1, ch0_coef2, ch1_coef2;

	if (5 != sscanf(buf, "%10d,%10d,%10d,%10d,%10d", &d_factor,
			&ch0_coef1, &ch1_coef1, &ch0_coef2, &ch1_coef2));
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->params.d_factor = d_factor;
	chip->params.lux_segment[0].ch0_coef = ch0_coef1;
	chip->params.lux_segment[0].ch1_coef = ch1_coef1;
	chip->params.lux_segment[1].ch0_coef = ch0_coef2;
	chip->params.lux_segment[1].ch1_coef = ch1_coef2;

	tmd2725_calc_lux_coef(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tmd2725_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmd2725_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tmd2725_configure_als_mode(chip, 1);
	else
		tmd2725_configure_als_mode(chip, 0);

	return size;
}

static ssize_t tmd2725_auto_gain_enable_show(struct device *dev,
					     struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd2725_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	return size;
}

static ssize_t tmd2725_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[(chip->params.als_gain & TMD2725_MASK_AGAIN)],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd2725_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int i = 0;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 4 && gain != 16 &&
			gain != 60 && gain != 64)
		return -EINVAL;

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (i > 3) {
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, (int)gain);
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->lock);

	if (gain) {
		chip->als_gain_auto = false;
		rc = tmd2725_set_als_gain(chip, als_gains[i]);
		if (!rc)
			tmd2725_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return rc ? -EIO : size;
}

static ssize_t tmd2725_als_cpl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t tmd2725_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TMD2725_REG_PERS]) & TMD2725_MASK_APERS)));
}

static ssize_t tmd2725_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long persist;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	chip->shadow[TMD2725_REG_PERS] &= ~TMD2725_MASK_APERS;
	chip->shadow[TMD2725_REG_PERS] |= ((u8)persist & TMD2725_MASK_APERS);

	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tmd2725_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	int t;

	t = chip->shadow[TMD2725_REG_ATIME];
	t *= INTEGRATION_CYCLE;
	return snprintf(buf, PAGE_SIZE, "%dms (%dus)\n", t / 1000, t);
}

static ssize_t tmd2725_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long itime;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &itime);
	if (rc)
		return -EINVAL;
	itime *= 1000;
	itime /= INTEGRATION_CYCLE;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TMD2725_REG_ATIME] = (u8) itime;
	chip->params.als_time = chip->shadow[TMD2725_REG_ATIME];
	tmd2725_calc_cpl(chip);
	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2725_als_wtime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int t;
	u8 wlongcurr;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	AMS_MUTEX_LOCK(&chip->lock);

	t = chip->shadow[TMD2725_REG_WTIME];

	wlongcurr = chip->shadow[TMD2725_REG_CFG0] & TMD2725_MASK_WLONG;
	if (wlongcurr)
		t *= 12;

	t *= INTEGRATION_CYCLE;
	t /= 1000;

	AMS_MUTEX_UNLOCK(&chip->lock);

	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmd2725_als_wtime_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	unsigned long wtime;
	int wlong;
	int rc;

	rc = kstrtoul(buf, 10, &wtime);
	if (rc)
		return -EINVAL;

	wtime *= 1000;
	if (wtime > (256 * INTEGRATION_CYCLE))
	{
		wlong = 1;
		wtime /= 12;
	}
	else
		wlong = 0;
	wtime /= INTEGRATION_CYCLE;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->shadow[TMD2725_REG_WTIME] = (u8) wtime;
	if (wlong)
		chip->shadow[TMD2725_REG_CFG0] |= TMD2725_MASK_WLONG;
	else
		chip->shadow[TMD2725_REG_CFG0] &= ~TMD2725_MASK_WLONG;

	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}


static ssize_t tmd2725_als_deltaP_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tmd2725_als_deltaP_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	rc = kstrtoul(buf, 10, &deltaP);
	if (rc || deltaP > 100)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->params.als_deltaP = deltaP;
	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tmd2725_als_ch0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.als_ch0);
}

static ssize_t tmd2725_als_ch1_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.als_ch1);
}


#ifdef LUX_DBG
static ssize_t tmd2725_als_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "LUX: %d CH0: %d CH1:%d\n",
		chip->als_inf.lux,chip->als_inf.als_ch0, chip->als_inf.als_ch1);
}

static ssize_t tmd2725_als_adc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = &g_tmd2725_chip;
	u32 ch0, ch1;

	if (2 != sscanf(buf, "%10d,%10d", &ch0, &ch1))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	chip->als_inf.als_ch0 = ch0;
	chip->als_inf.als_ch1 = ch1;

	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}
#endif // #ifdef LUX_DBG

	DEVICE_ATTR(tmd_als_Itime,         0664, tmd2725_als_itime_show,        tmd2725_als_itime_store);
	DEVICE_ATTR(tmd_als_Wtime,         0664, tmd2725_als_wtime_show,        tmd2725_als_wtime_store);
	DEVICE_ATTR(tmd_als_lux,           0444, tmd2725_device_als_lux,        NULL);
	DEVICE_ATTR(tmd_als_gain,          0664, tmd2725_als_gain_show,         tmd2725_als_gain_store);
	DEVICE_ATTR(tmd_als_cpl,           0444, tmd2725_als_cpl_show,          NULL);
	DEVICE_ATTR(tmd_als_thresh_deltaP, 0664, tmd2725_als_deltaP_show,       tmd2725_als_deltaP_store);
	DEVICE_ATTR(tmd_als_auto_gain,     0664, tmd2725_auto_gain_enable_show, tmd2725_auto_gain_enable_store);
	DEVICE_ATTR(tmd_als_lux_table,     0664, tmd2725_lux_table_show,        tmd2725_lux_table_store);
	DEVICE_ATTR(tmd_als_power_state,   0664, tmd2725_als_enable_show,       tmd2725_als_enable_store);
	DEVICE_ATTR(tmd_als_persist,       0664, tmd2725_als_persist_show,      tmd2725_als_persist_store);
	DEVICE_ATTR(tmd_als_ch0,           0444, tmd2725_als_ch0_show,          NULL);
	DEVICE_ATTR(tmd_als_ch1,           0444, tmd2725_als_ch1_show,          NULL);
#ifdef LUX_DBG
    DEVICE_ATTR(tmd_als_adc,           0664, tmd2725_als_adc_show,          tmd2725_als_adc_store);
#endif // #ifdef LUX_DBG

static struct attribute *TMD2725X_als_debug_attr_list[] = {
    &dev_attr_tmd_als_Itime.attr,
    &dev_attr_tmd_als_Wtime.attr,
    &dev_attr_tmd_als_lux.attr,
    &dev_attr_tmd_als_gain.attr,
    &dev_attr_tmd_als_cpl.attr,
    &dev_attr_tmd_als_thresh_deltaP.attr,
    &dev_attr_tmd_als_auto_gain.attr,
    &dev_attr_tmd_als_lux_table.attr,
    &dev_attr_tmd_als_power_state.attr,
    &dev_attr_tmd_als_persist.attr,
    &dev_attr_tmd_als_ch0.attr,
    &dev_attr_tmd_als_ch1.attr,
#ifdef LUX_DBG
    &dev_attr_tmd_als_adc.attr,
#endif
    NULL
};

static struct attribute_group als_tmd_attribute_group = {
	.attrs = TMD2725X_als_debug_attr_list
};

/*----------------------------------------------------------------------------*/
int TMD2725X_create_als_debug_attr(struct device* dev)
{
	int err = 0;

	err = sysfs_create_group(&dev->kobj,&als_tmd_attribute_group);
	if (err < 0) {
		printk(KERN_ERR"unable to create als tmd attribute file\n");
		return -3;
	}
	return err;
}
/*----------------------------------------------------------------------------*/
int TMD2725X_delete_als_debug_attr(struct device* dev)
{
    int err = 0;
	sysfs_remove_group(&dev->kobj,&als_tmd_attribute_group);
    return err;
}
/*----------------------------------------------------------------------------*/
#endif
