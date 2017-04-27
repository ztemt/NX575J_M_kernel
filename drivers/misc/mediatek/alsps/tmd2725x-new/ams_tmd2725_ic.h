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
* proximity detection (prox), Gesture, and Beam functionality within the
* AMS-TAOS TMD2725 family of devices.
*/

#ifndef __TMD2725_H
#define __TMD2725_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#ifndef TMD2725X_IC_API_FUNC_SYS
#define TMD2725X_IC_API_FUNC_SYS
#endif
#ifdef TMD2725X_IC_API_FUNC_SYS/*TMD2725X_IC_API_FUNC_SYS*/
//#define ABI_SET_GET_REGISTERS
extern unsigned int mt_gpio_to_irq(unsigned int gpio);
#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
		printk(KERN_INFO "%s: Mutex Lock\n", __func__); \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		printk(KERN_INFO "%s: Mutex Unlock\n", __func__); \
		mutex_unlock(m); \
	}
#else
#define AMS_MUTEX_LOCK(m) { \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		mutex_unlock(m); \
	}
#endif
#endif/*TMD2725X_IC_API_FUNC_SYS*/
extern struct tmd2725_chip  g_tmd2725_chip;

#ifndef FACTORY_MACRO_PS
#define FACTORY_MACRO_PS // add baibo 20170206
#endif

#ifdef FACTORY_MACRO_PS
#define PS_CAL_FILE_PATH                   "/persist/sensors/xtalk_cal"
#define TMD2725X_CHIP_NAME 	"tmd2725"
#define TMD2725X_DATA_MAX		255
#define TMD2725X_DATA_SAFE_RANGE_MIN		1
#define TMD2725X_DATA_SAFE_RANGE_MAX		150
#define TMD2725X_THRES_MIN 	30
#define TMD2725X_THRES_MAX 	240
#define PS_AVG_TIME                     6
#endif
// begin baibo 20170206
#define PS_THRES_FAR                    50
#define PS_THRES_NEAR                   80
#define PS_THRES_OIL_NEAR                254
#define PS_THRES_OIL_FAR                150
#define PS_THRES_FIRST_REPOST_FAR_EVENT                230
// end baibo 20170206

enum tmd2725_regs {
	TMD2725_REG_ENABLE     = 0x80,
	TMD2725_REG_ATIME      = 0x81,
	TMD2725_REG_PTIME      = 0x82,
	TMD2725_REG_WTIME      = 0x83,
	TMD2725_REG_AILT       = 0x84,
	TMD2725_REG_AILT_HI    = 0x85,
	TMD2725_REG_AIHT       = 0x86,
	TMD2725_REG_AIHT_HI    = 0x87,
	TMD2725_REG_PILT       = 0x88,
	TMD2725_REG_PIHT       = 0x8A,
	TMD2725_REG_PERS       = 0x8C,
	TMD2725_REG_CFG0       = 0x8D,
	TMD2725_REG_PGCFG0     = 0x8E,
	TMD2725_REG_PGCFG1     = 0x8F,

	TMD2725_REG_CFG1       = 0x90,
	TMD2725_REG_REVID      = 0x91,
	TMD2725_REG_ID         = 0x92,
	TMD2725_REG_STATUS     = 0x93,
	TMD2725_REG_CH0DATA    = 0x94,
	TMD2725_REG_CH0DATA_HI = 0x95,
	TMD2725_REG_CH1DATA    = 0x96,
	TMD2725_REG_CH1DATA_HI = 0x97,
	TMD2725_REG_PDATA      = 0x9C,

	TMD2725_REG_ADCDATA_L  = 0x9D,
	TMD2725_REG_AUXID      = 0x9E,
	TMD2725_REG_CFG2       = 0x9F,

	TMD2725_REG_CFG3       = 0xAB,
	TMD2725_REG_CFG4       = 0xAC,
	TMD2725_REG_CFG5       = 0xAD,

	TMD2725_REG_POFFSET_L  = 0xC0,
	TMD2725_REG_POFFSET_H  = 0xC1,

	TMD2725_REG_AZ_CONFIG  = 0xD6,
	TMD2725_REG_CALIB      = 0xD7,
	TMD2725_REG_CALIBCFG   = 0xD9,
	TMD2725_REG_CALIBSTAT  = 0xDC,
	TMD2725_REG_INTENAB    = 0xDD,
};

enum tmd2725__reg {
	TMD2725_MASK_BINSRCH_TARGET = 0xe0,	//0x70
	TMD2725_SHIFT_BINSRCH_TARGET = 5,	//4/

	TMD2725_MASK_START_OFFSET_CALIB = 0x01,
	TMD2725_SHIFT_START_OFFSET_CALIB = 0,

	TMD2725_MASK_PROX_PERS = 0xf0,
	TMD2725_SHIFT_PROX_PERS = 4,

	TMD2725_MASK_PDRIVE = 0x1f,
	TMD2725_SHIFT_PDRIVE = 0,

	TMD2725_MASK_PGAIN = 0xC0,
	TMD2725_SHIFT_PGAIN = 6,

	TMD2725_MASK_AGAIN = 0x03,
	TMD2725_SHIFT_AGAIN = 0,

	TMD2725_MASK_APERS = 0x0f,
	TMD2725_SHIFT_APERS = 0,

	TMD2725_MASK_WLONG = 0x04,
	TMD2725_SHIFT_WLONG = 2,

	TMD2725_MASK_POFFSET_H = 0x01,
	TMD2725_SHIFT_POFFSET_H = 0,

	TMD2725_MASK_PROX_DATA_AVG = 0x07,
	TMD2725_SHIFT_PROX_DATA_AVG = 0,

	TMD2725_MASK_PROX_AUTO_OFFSET_ADJUST = 0x08,
	TMD2725_SHIFT_PROX_AUTO_OFFSET_ADJUST = 3,
};

enum tmd2725_en_reg {
	TMD2725_PON  = (1 << 0),
	TMD2725_AEN  = (1 << 1),
	TMD2725_PEN  = (1 << 2),
	TMD2725_WEN  = (1 << 3),
	TMD2725_EN_ALL = (TMD2725_AEN |
			  TMD2725_PEN |
			  TMD2725_WEN),
};

enum tmd2725_status {
	TMD2725_ST_PGSAT_AMBIENT  = (1 << 0),
	TMD2725_ST_PGSAT_RELFLECT = (1 << 1),
	TMD2725_ST_ZERODET    = (1 << 1),
	TMD2725_ST_CAL_IRQ    = (1 << 3),
	TMD2725_ST_ALS_IRQ    = (1 << 4),
	TMD2725_ST_PRX_IRQ    = (1 << 5),
	TMD2725_ST_PRX_SAT    = (1 << 6),
	TMD2725_ST_ALS_SAT    = (1 << 7),
};

enum tmd2725_intenab_reg {
	TMD2725_ZIEN = (1 << 2),
	TMD2725_CIEN = (1 << 3),
	TMD2725_AIEN = (1 << 4),
	TMD2725_PIEN = (1 << 5),
	TMD2725_PSIEN = (1 << 6),
	TMD2725_ASIEN = (1 << 7),
};

#define MAX_REGS 256
struct device;

enum tmd2725_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tmd2725_prox_state {
	PROX_STATE_NONE = 0,
	PROX_STATE_INIT,
	PROX_STATE_CALIB,
	PROX_STATE_WAIT_AND_CALIB
};


enum tmd2725_oil_state {
	OIL_NONE = 0,
	OIL_STATE = 1,
};

enum tmd2725_prox_distance {
    PROX_NONE              = -1,
    PROX_NEAR              = 1,
    PROX_FAR               = 0,
};


enum tmd2725_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
	PGAIN_1        = (0 << TMD2725_SHIFT_PGAIN),
	PGAIN_2        = (1 << TMD2725_SHIFT_PGAIN),
	PGAIN_4        = (2 << TMD2725_SHIFT_PGAIN),
	PGAIN_8        = (3 << TMD2725_SHIFT_PGAIN),
	PG_PULSE_4US   = (0 << 6),
	PG_PULSE_8US   = (1 << 6),
	PG_PULSE_16US  = (2 << 6),
	PG_PULSE_32US  = (3 << 6),
};

// pldrive
#define PDRIVE_MA(p)   (((u8)((p) / 6) - 1) & 0x3f)
#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define INTEGRATION_CYCLE 2816
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

// lux
#define INDOOR_LUX_TRIGGER	6000
#define OUTDOOR_LUX_TRIGGER	10000
#define TMD2725_MAX_LUX		0xffff
#define TMD2725_MAX_ALS_VALUE	0xffff
#define TMD2725_MIN_ALS_VALUE	10

struct tmd2725_lux_segment {
	u32 ch0_coef;
	u32 ch1_coef;
};

struct tmd2725_parameters {
    u8      prox_thres_far;
	u8      prox_thres_near;
	u8      prox_thres_oil_near;
	u8      prox_thres_oil_far;
	u8      prox_oil_state;
	u16 prox_th_min;
	u16 prox_th_max;
	s16 poffset;
	u8 prox_gain;
	u8 prox_drive;
	u8 persist;
	u8 prox_pulse_cnt;
	s16 prox_offset;
	u8 prox_pulse_len;

	u8 als_time;
	u16 als_deltaP;
	u8 als_gain;
	u32 d_factor;

	struct tmd2725_lux_segment lux_segment[2];
};

struct tmd2725_als_info {
	u16      als_ch0; // photopic channel
	u16      als_ch1; // ir channel
	u32     cpl;
	int64_t lux1_ch0_coef;
	int64_t lux1_ch1_coef;
	int64_t lux2_ch0_coef;
	int64_t lux2_ch1_coef;
	u32     saturation;
	u16     lux;
    u16     prev_lux;
    u16     cal_lux;
};

struct tmd2725_prox_info {
	u16 raw;
	int detected;
};

struct tmd2725_als_cal_data{
    int base;
    int cur;
    u8 flag;
};

struct tmd2725_chip {
	struct alsps_hw *hw;
	struct work_struct irq_work;
#ifdef CONFIG_OF
	struct device_node *irq_node;
#endif
	struct mutex lock;
	struct mutex ps_lock;
	struct i2c_client *client;
	struct tmd2725_prox_info prx_inf;
	struct tmd2725_als_info als_inf;
	struct tmd2725_parameters params;
	struct tmd2725_i2c_platform_data *pdata;
	u8 shadow[MAX_REGS];

	struct input_dev *p_idev;
	struct input_dev *a_idev;
#ifdef ABI_SET_GET_REGISTERS
	struct input_dev *d_idev;
#endif // #ifdef ABI_SET_GET_REGISTERS

	int in_suspend;
	int wake_irq;
	int irq_pending;

	bool unpowered;
	bool als_enabled;
	bool als_gain_auto;
	bool prx_enabled;
    bool prx_cail;
	bool amsCalComplete;
	bool amsFirstProx;
	bool amsIndoorMode;

	enum tmd2725_prox_state prox_state;
	struct work_struct work_prox;
    struct tmd2725_als_cal_data als_cal_data;

	u8 device_index;
};

// Must match definition in ../arch file
struct tmd2725_i2c_platform_data {
	/* The following callback for power events received and handled by
	   the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct device *dev, enum tmd2725_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);

	char const *prox_name;
	char const *als_name;
	struct tmd2725_parameters parameters;
	bool proximity_can_wake;
	bool als_can_wake;
};
//int tmd2725_irq_handler(struct tmd2725_chip *chip);
//int tmd2725_get_id(struct tmd2725_chip *chip, u8 *id, u8 *rev, u8 *auxid);
//void tmd2725_set_defaults(struct tmd2725_chip *chip);
//int tmd2725_flush_regs(struct tmd2725_chip *chip);
#endif /* __TMD2725_H */
