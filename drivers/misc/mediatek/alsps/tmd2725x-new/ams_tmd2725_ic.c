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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "ams_tmd2725.h"
#include "ams_tmd2725_ic.h"
#include "ams_i2c.h"
#include "ams_tmd2725_prox.h"
#include "ams_tmd2725_als.h"
/* Registers to restore */
static u8 const restorable_regs[] = {
	TMD2725_REG_PILT,
	TMD2725_REG_PIHT,
	TMD2725_REG_PERS,
	TMD2725_REG_PGCFG0,
	TMD2725_REG_PGCFG1,
	TMD2725_REG_CFG1,
	TMD2725_REG_PTIME,
	TMD2725_REG_ATIME,
};

int tmd2725_irq_handler(struct tmd2725_chip *chip)
{
	u8 status;
	int ret;
	struct device *dev = &chip->client->dev; // add baibo 20170206

	ret = ams_i2c_read(chip->client, TMD2725_REG_STATUS, &chip->shadow[TMD2725_REG_STATUS]);
	status = chip->shadow[TMD2725_REG_STATUS];
    //printk("interrupt kindles of::%s:%u\n",__func__,status);
    dev_info(dev, "%s: status=%u\n", __func__, status);
	if (status == 0) {
		return 0;  // not our interrupt
	}

	do {

		// Clear the interrupts we'll process
		ams_i2c_write_direct(chip->client, TMD2725_REG_STATUS, status);

		/*************/
		/* ALS       */
		/*************/
		if (status & TMD2725_ST_ALS_IRQ)
		{
			tmd2725_read_als(chip);
			// Change the prox settings based on lux (indoor/outdoor light conditions)
			//tmd2725_set_prox_mode(chip); //comment by baibo 20170123 for FAE
			dev_info(dev, "%s: als int\n", __func__);
		}

		/*************/
		/* Proximity */
		/*************/
		if (status & TMD2725_ST_PRX_IRQ)
		{
			// Read Prox, determine detect/release, report results
            dev_info(dev, "%s: Proximity int\n", __func__);
			tmd2725_read_prox(chip);
			tmd2725_get_prox(chip);
			ps_report_interrupt_data(chip->prx_inf.detected); // add by baibo 20170208
		}

		/***************/
		/* Calibration */
		/***************/
		if (status & TMD2725_ST_CAL_IRQ)
		{
			chip->amsCalComplete = true;

			/*
			** Calibration has completed, no need for more
			**  calibration interrupts. These events are one-shots.
			**  next calibration start will re-enable.
			*/
			ams_i2c_modify(chip->client, chip->shadow,
				TMD2725_REG_INTENAB, TMD2725_CIEN, 0);
			dev_info(dev, "%s: Calibration int\n", __func__);
		}

		ret = ams_i2c_read(chip->client, TMD2725_REG_STATUS, &chip->shadow[TMD2725_REG_STATUS]);
		status = chip->shadow[TMD2725_REG_STATUS];
	} while (status != 0);

	return 1;  // we handled the interrupt
}

int tmd2725_flush_regs(struct tmd2725_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

int tmd2725_update_enable_reg(struct tmd2725_chip *chip)
{
	return ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE,
			chip->shadow[TMD2725_REG_ENABLE]);
}

void tmd2725_set_defaults(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

    mutex_init(&chip->ps_lock);
	// Clear the register shadow area
	memset(chip->shadow, 0x00, sizeof(chip->shadow));
	dev_info(dev, "%s: use defaults\n", __func__);
	chip->params.prox_th_min = 64;
	chip->params.prox_th_max = 128;
	chip->params.persist = PRX_PERSIST(1) | ALS_PERSIST(2);
	chip->params.prox_pulse_cnt = 15; // modify baibo 20170206
	chip->params.prox_gain = PGAIN_2;
	chip->params.prox_drive = PDRIVE_MA(100); // modify baibo 20170206
	chip->params.prox_offset = 0;
	chip->params.prox_pulse_len = PG_PULSE_8US; // modify baibo 20170206
	chip->params.als_gain = AGAIN_64;
	chip->params.als_deltaP = 10;
	chip->params.als_time = AW_TIME_MS(400);
	chip->params.d_factor = 581;
    chip->params.lux_segment[0].ch0_coef = 1000;
    chip->params.lux_segment[0].ch1_coef = 192;
    chip->params.lux_segment[1].ch0_coef = 769;
    chip->params.lux_segment[1].ch1_coef = 86;
	chip->als_gain_auto = true;

	// Copy the default values into the register shadow area
	sh[TMD2725_REG_PILT]    = (chip->params.prox_th_min & 0xff);
	sh[TMD2725_REG_PIHT]    = (chip->params.prox_th_max & 0xff);
	sh[TMD2725_REG_PERS]    = chip->params.persist;
	sh[TMD2725_REG_PGCFG0]  = (chip->params.prox_pulse_cnt -1 ) | chip->params.prox_pulse_len;
	sh[TMD2725_REG_ATIME]   = chip->params.als_time;
	sh[TMD2725_REG_CFG1]    = chip->params.als_gain;
	sh[TMD2725_REG_PTIME]   = P_TIME_US(2816);
	sh[TMD2725_REG_PGCFG1] = (chip->params.prox_gain & TMD2725_MASK_PGAIN) |
		chip->params.prox_drive;

	tmd2725_flush_regs(chip);
}

int tmd2725_get_id(struct tmd2725_chip *chip, u8 *id, u8 *rev, u8 *auxid)
{
	ams_i2c_read(chip->client, TMD2725_REG_AUXID, auxid);
	ams_i2c_read(chip->client, TMD2725_REG_REVID, rev);
	ams_i2c_read(chip->client, TMD2725_REG_ID, id);

	return 0;
}
