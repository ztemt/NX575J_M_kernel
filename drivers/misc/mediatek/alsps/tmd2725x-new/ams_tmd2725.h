/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for TMD2725X als/ps sensor chip.
 */
#ifndef __TMD2725X_H__
#define __TMD2725X_H__

#include <linux/ioctl.h>
#include <alsps.h>

#define TMD2725X_SUCCESS						0
#define TMD2725X_ERR_I2C						-1
#define TMD2725X_ERR_STATUS					-3
#define TMD2725X_ERR_SETUP_FAILURE				-4
#define TMD2725X_ERR_GETGSENSORDATA			-5
#define TMD2725X_ERR_IDENTIFICATION			-6

#ifndef TMD2725X_IC_API_FUNC
#define TMD2725X_IC_API_FUNC
#endif

#ifndef TMD2725X_IC_API_FUNC_SYS
#define TMD2725X_IC_API_FUNC_SYS
#endif
#endif
