/*
 * user_charge.c
 *
 *  Created on: 2020年12月2日
 *      Author: Sky
 */

#include "user_charge.h"
#include "hal_mcu.h"
#include "clock.h"
#include "log.h"
#include	"protocol.h"
#include "command_center.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "display.h"
chargeDetDef chargeParas;

switchDef switchParams;
///***********************************************************************************************************
//  *  @brief			 the charge fully pin low process
//  *
//  *  @param [in] :		none
//  *
//  *  @param [out] :		none
//  *
//  *  @return :		none
//  *
//  *  @note :
//  ************************************************************************************************************/
//void fullyPinFallingCb(void) {
//	if (chargeParas.vVettingStts != Charging_Vetting) {
//		chargeParas.vVettingStts = Charging_Vetting;
//		chargeParas.vChargeTick = hal_systick();
//	}
//	chargeParas.fullyGot = 0;
//}
void fullyPinLowCb(void) {
	if (chargeParas.vVettingStts != Fully_Vetting){
		chargeParas.vVettingStts = Fully_Vetting;
		chargeParas.vChargeTick = hal_systick();
	}
}
///***********************************************************************************************************
//  *  @brief			 the charge fully pin high process
//  *
//  *  @param [in] :		none
//  *
//  *  @param [out] :		none
//  *
//  *  @return :		none
//  *
//  *  @note :
//  ************************************************************************************************************/
//void fullyPinRisingCb(void) {
//	if (chargeParas.vVettingStts != Fully_Vetting){
//		chargeParas.vVettingStts = Fully_Vetting;
//		chargeParas.vChargeTick = hal_systick();
//	}
//}
void fullyPinHighCb(void) {
	if (chargeParas.vVettingStts != Charging_Vetting) {
		chargeParas.vVettingStts = Charging_Vetting;
		chargeParas.vChargeTick = hal_systick();
	}
	chargeParas.fullyGot = 0;
}
/***********************************************************************************************************
  *  @brief			 the switch pin low process
  *
  *  @param [in] :		stts point to an uint8 value,which use for output the switch status
  *
  *  @param [out] :		stts:    0/1/2    detecting/switch off/switch on
  *
  *  @return :		none
  *
  *  @note :
  ************************************************************************************************************/
void powerPinLowCb(uint8 *stts)  {
	if(switchParams.Stts==LowVetting){
		*stts=switchParams.Stts;
		return;
	}
	if (switchParams.vVettingStts!=LowVetting){
		switchParams.sttsGot=0;
		switchParams.vVettingStts = LowVetting;
		switchParams.vSwitchTick = hal_systick();
	}
}
/***********************************************************************************************************
  *  @brief			 the switch pin high process
  *
  *  @param [in] :		stts point to an uint8 value,which use for output the switch status
  *
  *  @param [out] :		stts:    0/1/2    detecting/switch off/switch on
  *
  *  @return :		none
  *
  *  @note :
  ************************************************************************************************************/
void powerPinHiCb(uint8 *stts) {
	if(switchParams.Stts==SwitchHi){
		*stts=switchParams.Stts;
		return;
	}
	if (switchParams.vVettingStts != HiVetting){
		switchParams.sttsGot=0;
		switchParams.vVettingStts = HiVetting;
		switchParams.vSwitchTick = hal_systick();
	}
}
/***********************************************************************************************************
  *  @brief			 the switch status detect event
  *
  *  @param [in] :		stts point to an uint8 value,which use for output the switch status
  *
  *  @param [out] :		stts:    0/1/2    detecting/switch off/switch on
  *
  *  @return :		none
  *
  *  @note :
  ************************************************************************************************************/
void  switchSttsDectEvent(uint8 *stts){
	if(!hal_gpio_read(GPIO_CHARGE_DET)){
		if(hal_gpio_read(GPIO_KEY_POWER)){
			powerPinHiCb(stts);
		}else{
			powerPinLowCb(stts);
		}
		if (!switchParams.sttsGot&&(hal_ms_intv(switchParams.vSwitchTick) >200)){
			if(switchParams.sttsGot){
				switchParams.lastStts=switchParams.Stts;
			}
			if(switchParams.vVettingStts == HiVetting){
				switchParams.Stts=SwitchHi;
			}else{
				switchParams.Stts=SwitchLow;
			}
			switchParams.sttsGot=1;
			*stts=switchParams.Stts;
		}
		osal_start_timerEx(command_center_TaskID, SWITCH_EVT, 100);

	}else{
		*stts=0;
		memset(&switchParams,0,sizeof(switchParams));
		osal_stop_timerEx(command_center_TaskID, SWITCH_EVT);
//		LOG("\n stop");

	}
}
/***********************************************************************************************************
  *  @brief			 charge status detecting event
  *
  *  @param [in] :		none
  *
  *  @param [out] :		none
  *
  *  @return :		none
  *
  *  @note :
  ************************************************************************************************************/

void	chargeSttsDectEvent(void){
	if(!hal_gpio_read(GPIO_CHARGE_DET)){
		if(hal_gpio_read(GPIO_CHARGE_FULL)){
//			fullyPinRisingCb();
			fullyPinHighCb();
		}else{
//			fullyPinFallingCb();
			fullyPinLowCb();
		}
		osal_start_reload_timer(command_center_TaskID, CHARGE_END_EVENT, 300);
//	LOG("\n r start");

	}else{
		memset(&chargeParas,0,sizeof(chargeParas));
		osal_stop_timerEx(command_center_TaskID, CHARGE_END_EVENT);
//	LOG("\n stop");
	}
	if(!osal_get_timeoutEx(command_center_TaskID, SWITCH_EVT))
		osal_start_timerEx(command_center_TaskID, SWITCH_EVT, 100);
}
/***********************************************************************************************************
  *  @brief				get current charging status
  *
  *
  *  @param [in] :		result point to an uint8 value,which use for output the Charging status
  *
  *  @param [out] :		result point to the Charging status
  *
  *  @return :		none
  *
  *  @note :
  ************************************************************************************************************/
void getChargeStts(uint8 * result) {
	if (!hal_gpio_read(GPIO_CHARGE_DET)) {
//		LOG(" \n   **********check  stts=%d",chargeParas.vVettingStts);
		if (chargeParas.vVettingStts) {
			if (chargeParas.fullyGot && chargeParas.vVettingStts == Fully_Vetting) {
				*result = Battery_fully;
			} else if (hal_ms_intv(chargeParas.vChargeTick) > VALID_FULLY_DET_TIMER) {
				if (chargeParas.vVettingStts == Fully_Vetting) {
					*result = Battery_fully;
					chargeParas.fullyGot = 1;
				} else {
					if (*result == Battery_fully && (!getSystemStts())) {
						batteryPercentDisplay(0);
					}
					*result = Sys_Charging;
					chargeParas.fullyGot = 0;
				}
			} else {
				if (*result == Battery_fully && (!getSystemStts())) {
					batteryPercentDisplay(0);
				}
				*result = Sys_Charging;
				chargeParas.fullyGot = 0;
			}
		}
	} else {
		chargeParas.fullyGot = 0;
		*result = Non_Charging;
	}
}
