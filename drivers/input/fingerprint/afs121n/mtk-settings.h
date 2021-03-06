/* MicroArray Fprint Driver Code
 * mtk-settings.h
 * Date: 2016-09-10
 * Version: v4.1.2
 * Author: czl&cpy&guq
 * Contact: [czl|cpy|guq]@microarray.com.cn
 */

#ifndef __MTK_SETTINGS_H_
#define __MTK_SETTINGS_H_



#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include "madev.h"
#include <linux/spi/spi.h>
#include <linux/wakelock.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include "mt_spi.h"
#include "madev.h"
#include <linux/wakelock.h>
#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif
#include <mt_spi.h>
#include "mt_spi_hal.h"
/*#include "mach/memory.h"*/
#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_gpio.h>
#endif				/* defined(CONFIG_MTK_LEGACY) */
#if defined(CONFIG_MTK_CLKMGR)
#include <mach/mt_clkmgr.h>
#endif				/* defined(CONFIG_MTK_CLKMGR) */

//macro settings
#define MA_DRV_NAME             "gf_spi"

#define MA_DTS_NAME            	"mediatek,fingerprint"//"mediatek,hct_finger"

#define MA_EINT_DTS_NAME        "mediatek,fingerprint"//"mediatek,hct_finger"
//macro settings end



//call madev function
extern int mas_plat_probe(struct platform_device *pdev);
extern int mas_plat_remove(struct platform_device *pdev);

extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);
extern void nubia_enable_spi_clk(struct spi_device *spidev, bool en);
/* add for spi cls ctl start */
struct mt_spi_t {
    struct platform_device *pdev;
    void __iomem *regs;
    int irq;
    int running;
    struct wake_lock wk_lock;
    struct mt_chip_conf *config;
    struct spi_master *master;

    struct spi_transfer *cur_transfer;
    struct spi_transfer *next_transfer;

    spinlock_t lock;
    struct list_head queue;
    //#if !defined(CONFIG_MTK_CLKMGR)
    struct clk *clk_main;
    //#endif
};


//the interface called by madev
void mas_select_transfer(struct spi_device *spi, int len);
int mas_finger_get_gpio_info(struct platform_device *pdev);
int mas_finger_set_gpio_info(int cmd);
void mas_enable_spi_clock(struct spi_device *spi);
void mas_disable_spi_clock(struct spi_device *spi);
unsigned int mas_get_irq(void);
int mas_get_platform(void);

#endif
