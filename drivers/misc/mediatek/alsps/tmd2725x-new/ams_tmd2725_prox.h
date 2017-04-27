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

#ifndef __AMS_TMD2725_PROX_H
#define __AMS_TMD2725_PROX_H
#ifndef TMD2725X_IC_API_FUNC_SYS
#define TMD2725X_IC_API_FUNC_SYS
#endif

#ifdef TMD2725X_IC_API_FUNC_SYS
extern int TMD2725X_create_prox_debug_attr(struct device* dev);
extern int TMD2725X_delete_prox_debug_attr(struct device* dev);
#endif

extern void tmd2725_read_prox(struct tmd2725_chip *chip);
extern void tmd2725_get_prox(struct tmd2725_chip *chip);
extern void tmd2725_report_prox(struct tmd2725_chip *chip);
extern void tmd2725_set_prox_mode(struct tmd2725_chip *chip);
extern void tmd2725_init_prox_mode(struct tmd2725_chip *chip);
extern int tmd2725_configure_prox_mode(struct tmd2725_chip *chip, u8 state);
extern void tmd2725_prox_thread(struct work_struct *work);
extern void tmd2725_schedule_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state);
extern int tmd2725_offset_calibration(struct tmd2725_chip *chip);

#endif /*__AMS_TMD2725_PROX_H */
