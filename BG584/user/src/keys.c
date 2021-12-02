/**************************************************************************************************
Filename:       keys.c
Revised:        Date: 2020.9.12
Revision:       1.0

Description:


Copyright 2012 Boutgh R&D.. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Boughg R&D. (the "License").  You may not use this Software unless you
agree to abide by the terms of the License. The License limits your use,
and you acknowledge, that the Software may not be modified,copied or
distributed unless embedded on a Texas Bough LTD., which is integrated into
your product.  Other than for the foregoing purpose, you may not use,
reproduce, copy, prepare derivative works of, modify, distribute, perform,
display or sell this Software and/or its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED THIS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
BOUGH OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE,
STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED
TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES,
LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,
TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Bouth R&D at www.bough.com.cn
**************************************************************************************************/

/*-----------------------------------------------------------------------------------------------*/
/* Copyright(c) 2012 Bough Technology Corp. All rights reserved.                                 */
/*-----------------------------------------------------------------------------------------------*/

/*********************************************************************
* INCLUDES
*/
#include <string.h>
#include "types.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* Driver */
#include "pwrmgr.h"
#include "gpio.h"
#include "protocol.h"
#include "keys.h"
#include "user_charge.h"
/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* VARIABLES
*/
static uint8 upKeyPressTimes=0;
static uint8 downKeyPressTimes=0;
static uint8 arrowKeyPressTimes=0;
static uint8 modeKeyPressTimes=0;
static protocol_CBs_t debounceCBs;
static protocol_CBs_t keysChange;
static uint16 keys_Save_keys = 0;

/*********************************************************************
* FUNCTIONS
*/

/*********************************************************************
 * gpio_wakeup_Task
 * Task gpio wakeup sample code
 * The followinng code shows P14 wakeup the system when there is a posedge or negedge.
 */
static void posedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(type == POSEDGE)
	{
		osal_start_timerEx(debounceCBs.task_id,debounceCBs.events,HAL_KEY_DEBOUNCE_TIME);
	}
	else
	{
		
	}
}

static void negedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(type == NEGEDGE)
	{
		osal_start_timerEx(debounceCBs.task_id,debounceCBs.events,HAL_KEY_DEBOUNCE_TIME);
	}
	else
	{
		
	}
}

/*********************************************************************
 * @fn      keys_Init
 *
 * @brief   Initialization function for the driver. Only call by hal_init().
 *
 * @param   none
 *
 * @return  none
 */
void keys_Init(void)
{
	keys_Save_keys = 0;
	gpio_struct_t gpio_struct[KEY_MAX];
	uint8 i=0;
	//KEY_POWER
//	gpio_struct[i].pin = GPIO_KEY_POWER;
//	gpio_struct[i].ioe = IE;
//	gpio_struct[i].bit_action = Bit_DISABLE;
//	gpio_struct[i].pull_type = WEAK_PULL_UP;
//	gpio_struct[i].wakeup_pol = NEGEDGE;
//	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;
	//KEY_LIGHT_UP
	gpio_struct[i].pin = GPIO_KEY_LIGHT_UP;
	gpio_struct[i].ioe = IE;
	gpio_struct[i].bit_action = Bit_DISABLE;
	gpio_struct[i].pull_type = WEAK_PULL_UP;
	gpio_struct[i].wakeup_pol = NEGEDGE;
	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;

	//KEY_LIGHT_DOWN
	gpio_struct[i].pin = GPIO_KEY_LIGHT_DOWN;
	gpio_struct[i].ioe = IE;
	gpio_struct[i].bit_action = Bit_DISABLE;
	gpio_struct[i].pull_type = WEAK_PULL_UP;
	gpio_struct[i].wakeup_pol = NEGEDGE;
	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;

	//KEY_ARROW
	gpio_struct[i].pin = GPIO_KEY_ARROW;
	gpio_struct[i].ioe = IE;
	gpio_struct[i].bit_action = Bit_DISABLE;
	gpio_struct[i].pull_type = WEAK_PULL_UP;
	gpio_struct[i].wakeup_pol = NEGEDGE;
	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;

	//KEY_MODE
	gpio_struct[i].pin = GPIO_KEY_MODE;
	gpio_struct[i].ioe = IE;
	gpio_struct[i].bit_action = Bit_DISABLE;
	gpio_struct[i].pull_type = WEAK_PULL_UP;
	gpio_struct[i].wakeup_pol = NEGEDGE;
	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;

	//GPIO_CHARGE_DET
	gpio_struct[i].pin = GPIO_CHARGE_DET;
	gpio_struct[i].ioe = IE;
	gpio_struct[i].bit_action = Bit_DISABLE;
	gpio_struct[i].pull_type = WEAK_PULL_UP;
	gpio_struct[i].wakeup_pol = NEGEDGE;
	gpio_struct[i].wakeup_t.posedgeHdl = posedge_callback_wakeup;
	gpio_struct[i++].wakeup_t.negedgeHdl = negedge_callback_wakeup;

	//GPIO_CHARGE_FULL
//	gpio_struct[i].pin = GPIO_CHARGE_FULL;
//	gpio_struct[i].ioe = IE;
//	gpio_struct[i].bit_action = Bit_DISABLE;
//	gpio_struct[i].pull_type = PULL_DOWN;
//	gpio_struct[i].wakeup_pol = NEGEDGE;
//	gpio_struct[i].wakeup_t.posedgeHdl = fullyPinRisingCb;
//	gpio_struct[i++].wakeup_t.negedgeHdl = fullyPinFallingCb;
	//KEY_LIGHT_UP
//	gpio_struct[0].pin = GPIO_KEY_LIGHT_UP;
//	gpio_struct[0].ioe = IE;
//	gpio_struct[0].bit_action = Bit_DISABLE;
//	gpio_struct[0].pull_type = WEAK_PULL_UP;
//	gpio_struct[0].wakeup_pol = NEGEDGE;
//	gpio_struct[0].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[0].wakeup_t.negedgeHdl = negedge_callback_wakeup;
//
//	//KEY_LIGHT_DOWN
//	gpio_struct[1].pin = GPIO_KEY_LIGHT_DOWN;
//	gpio_struct[1].ioe = IE;
//	gpio_struct[1].bit_action = Bit_DISABLE;
//	gpio_struct[1].pull_type = WEAK_PULL_UP;
//	gpio_struct[1].wakeup_pol = NEGEDGE;
//	gpio_struct[1].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[1].wakeup_t.negedgeHdl = negedge_callback_wakeup;
//
//	//KEY_ARROW
//	gpio_struct[2].pin = GPIO_KEY_ARROW;
//	gpio_struct[2].ioe = IE;
//	gpio_struct[2].bit_action = Bit_DISABLE;
//	gpio_struct[2].pull_type = WEAK_PULL_UP;
//	gpio_struct[2].wakeup_pol = NEGEDGE;
//	gpio_struct[2].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[2].wakeup_t.negedgeHdl = negedge_callback_wakeup;
//
//	//KEY_MODE
//	gpio_struct[3].pin = GPIO_KEY_MODE;
//	gpio_struct[3].ioe = IE;
//	gpio_struct[3].bit_action = Bit_DISABLE;
//	gpio_struct[3].pull_type = WEAK_PULL_UP;
//	gpio_struct[3].wakeup_pol = NEGEDGE;
//	gpio_struct[3].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[3].wakeup_t.negedgeHdl = negedge_callback_wakeup;
//
//	//GPIO_CHARGE_DET
//	gpio_struct[4].pin = GPIO_CHARGE_DET;
//	gpio_struct[4].ioe = IE;
//	gpio_struct[4].bit_action = Bit_DISABLE;
//	gpio_struct[4].pull_type = WEAK_PULL_UP;
//	gpio_struct[4].wakeup_pol = NEGEDGE;
//	gpio_struct[4].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[4].wakeup_t.negedgeHdl = negedge_callback_wakeup;
//
//	//GPIO_CHARGE_FULL
//	gpio_struct[5].pin = GPIO_CHARGE_FULL;
//	gpio_struct[5].ioe = IE;
//	gpio_struct[5].bit_action = Bit_DISABLE;
//	gpio_struct[5].pull_type = PULL_DOWN;
//	gpio_struct[5].wakeup_pol = NEGEDGE;
//	gpio_struct[5].wakeup_t.posedgeHdl = posedge_callback_wakeup;
//	gpio_struct[5].wakeup_t.negedgeHdl = negedge_callback_wakeup;
	
	for (uint8 i = 0; i < KEY_MAX; i++) {
		hal_gpio_pin_init(gpio_struct[i].pin, gpio_struct[i].ioe);
		hal_gpio_pull_set(gpio_struct[i].pin, gpio_struct[i].pull_type);
		hal_gpio_wakeup_set(gpio_struct[i].pin, gpio_struct[i].wakeup_pol);
		hal_gpioin_register(gpio_struct[i].pin, gpio_struct[i].wakeup_t.posedgeHdl, gpio_struct[i].wakeup_t.negedgeHdl);
	}
}
/*********************************************************************
 * @fn      keys_RegisterCBs
 *
 * @brief   Register for keys change callback.
 *
 * @param   task_id, event
 *
 * @return  none
 */
void keys_RegisterCBs(protocol_CBs_t cbs)
{
	keysChange.task_id = cbs.task_id;
	keysChange.events = cbs.events;
}

/*********************************************************************
 * @fn      keys_RegisterDebounceCBs
 *
 * @brief   Register for keys debounce callback.
 *
 * @param   task_id, event
 *
 * @return  none
 */
void keys_RegisterDebounceCBs(protocol_CBs_t cbs)
{
	debounceCBs.task_id = cbs.task_id;
	debounceCBs.events = cbs.events;
}

/*********************************************************************
 * @fn      keys_Debounce_Handle
 *
 * @brief   Handle keys debounce event.
 *
 * @param   none
 *
 * @return  none
 */
void keys_Debounce_Handle(void)
{
	uint16 keys = 0;
	
	if ( keys == keys_Save_keys )
	{
		if ( keysChange.task_id && keysChange.events )
		{
			osal_set_event(keysChange.task_id, keysChange.events);
		}
	}
	
	keys_Save_keys = 0;
}

/*********************************************************************
 * @fn      keys_read
 *
 * @brief   Handle keys debounce event.
 *
 * @param   none
 *
 * @return  uint16
 */
uint16 keys_read(void) {
	uint16 keys = 0;

//	if (!hal_gpio_read(GPIO_KEY_POWER)) {
//		keys |= KEY_POWER_ON;
//	}

	if (!hal_gpio_read(GPIO_KEY_LIGHT_UP)) {
		keys |= KEY_LIGHT_UP;
	}

	if (!hal_gpio_read(GPIO_KEY_LIGHT_DOWN)) {
		keys |= KEY_LIGHT_DOWN;
	}

	if (!hal_gpio_read(GPIO_KEY_ARROW)) {
		keys |= KEY_ARROW;
	}

	if (!hal_gpio_read(GPIO_KEY_MODE)) {
		keys |= KEY_MODE;
	}

	if (!hal_gpio_read(GPIO_CHARGE_DET)) {
		keys |= KEY_CHARGE_DET;
//		keys &=~ KEY_CHARGE_FULL;
	}
//	if (hal_gpio_read(GPIO_CHARGE_FULL)) {
//		if (!hal_gpio_read(GPIO_CHARGE_DET)){
//			keys |= KEY_CHARGE_FULL;
//		}
//	}

	return keys;
}


//void userKeysAdapter(uint8 inKeys,uint32 * value, uint8 mode) {
//	if (mode == SUPPORT_LONG_MODE) {
//		if (inKeys & KEY_LIGHT_UP) {
//			if ((* value & LIGHT_HOLDING) == 0) {				//
//				upKeyPressTimes = 1;								//first Pressed
//				* value |= LIGHT_HOLDING;
//			} else if (upKeyPressTimes++ >= LONG_UP_KEY_TIMES) {
//				if (!(* value & LONG_LIGHT_GOT)) {
//					* value |= LONG_LIGHT_GOT | LONG_LIGHT_UP;
//				} else if (upKeyPressTimes >= 2 * LONG_UP_KEY_TIMES) {
//					upKeyPressTimes = LONG_UP_KEY_TIMES;
//				}
//			}
//		} else {													//Key release;
//			* value &= CLEAR_UP_KEY_INFO;
//			if (upKeyPressTimes && (upKeyPressTimes < LONG_UP_KEY_TIMES)) {
//				* value |= (uint32) KEY_LIGHT_UP;
//			}
//			upKeyPressTimes = 0;
//		}
//	}else if(mode == QUICK_CHANGE_MODE){
//		if (inKeys & KEY_LIGHT_UP) {
//			if ((* value & LIGHT_HOLDING) ==0) {				//
//				upKeyPressTimes = 1;								//first Pressed
//				* value |= LIGHT_HOLDING|KEY_LIGHT_UP;
//			} else if (upKeyPressTimes++ >= QUICK_KEYS_START_TIMES) {
//				if(upKeyPressTimes%2==0){
//					* value|=KEY_LIGHT_UP;
//				}
//			}
//		} else {													//Key release;
//			* value &= CLEAR_UP_KEY_INFO;
//			upKeyPressTimes = 0;
//		}
//	}
//}

/***********************************************************************************************************
  *  @brief  					support Long press mode key adapter
  *
  *
  *  @param [in] :			inKeys: IO status ,which output from function  keys_read
  *  @param [in] :			chkKey: which key will be detected.
  *  @param [in] :			value: point to a uint32  value , use to input and output the key value and detect status. See
  *								KEY_LIGHT_UP				0x02
  *								LONG_LIGHT_UP				0x00000200
  *								LONG_LIGHT_GOT				0x00020000
  *								LIGHT_UP_HOLDING			0x02000000
  *								CLEAR_UP_KEY_INFO			(~0x02020202)
  *  @param [in] :			longThreshold: long press time threshold ,20ms/step
  *  @param [in] :			presstimes:  point to a uint8 register,use to record the time of the key pressed .
  *
  *  @param [out] :			value: point to a uint32  value , use to input and output the key value and detect status.
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
static void userSupportLongModeAdapter(uint8 inKeys, uint32 * value, uint8 chkKey, uint8 longThreshold, uint8 *presstimes) {
	uint32 holding = chkKey << 24;
	uint32 longGot = chkKey << 16;
	uint32 LongValue = chkKey << 8;
	uint32 clearKeyInfo = ~(holding | longGot | LongValue | chkKey);
	if (inKeys & chkKey) {
		if ((*value & holding) == 0) {
			*presstimes = 1;								//first Pressed
			*value |= holding;
		} else if ((*presstimes)++ >= longThreshold) {
			if (!(*value & longGot)) {
				*value |= longGot | LongValue;
			} else if (*presstimes >= 2 * longThreshold) {
				*presstimes = longThreshold;
			}
		}
	} else {
		*value &= clearKeyInfo;
		if (*presstimes && (*presstimes < longThreshold)) {
			*value |= (uint32) chkKey;
		}
		*presstimes = 0;
	}
}
/***********************************************************************************************************
  *  @brief  					support quickly  key  mode key adapter
  *
  *
  *  @param [in] :			inKeys: IO status ,which output from function  keys_read
  *  @param [in] :			chkKey: which key will be detected.
  *  @param [in] :			value: point to a uint32  value , use to input and output the key value and detect status. See
  *								KEY_LIGHT_UP				0x02
  *								LONG_LIGHT_UP				0x00000200
  *								LONG_LIGHT_GOT				0x00020000
  *								LIGHT_UP_HOLDING			0x02000000
  *								CLEAR_UP_KEY_INFO			(~0x02020202)
  *  @param [in] :			longThreshold: long press time threshold ,20ms/step,   threshold <=240 ,means threshold time<=4.8s
  *  @param [in] :			presstimes:  point to a uint8 register,use to record the time of the key pressed .
  *  @param [in] :			status:  0/1 disable/enable  quick keys.
  *
  *  @param [out] :			value: point to a uint32  value , use to input and output the key value and detect status.
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/

static void userSupportQuickModeAdapter(uint8 inKeys, uint32 * value, uint8 chkKey, uint8 longThreshold, uint8 *presstimes,quickKeyDef status) {
	uint32 holding = chkKey << 24;
	uint32 longGot = chkKey << 16;
	uint32 LongValue = chkKey << 8;
	uint32 clearKeyInfo = ~(holding | longGot | LongValue | chkKey);
	if (inKeys & chkKey) {
		if ((* value & holding) ==0) {				//
			*presstimes = 1;//first Pressed
			* value |= holding|chkKey;
		} else if ((*presstimes)++ >= longThreshold) {
			//if(*presstimes%2==0) {
			if (EnableQuickKeys==status) {
				*value |= chkKey;
				if (*presstimes >= 250)
					*presstimes = 240;
			}
			//}
		}
	} else {													//Key release;
		* value &= clearKeyInfo;
		*presstimes = 0;
	}
}
/***********************************************************************************************************
  *  @brief
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void userKeysAdapter(uint8 inKeys,uint32 * value) {
//	if (mode == SUPPORT_LONG_MODE) {
//		userSupportLongModeAdapter(inKeys,value,KEY_LIGHT_UP,LONG_UP_KEY_TIMES,&upKeyPressTimes);
//	}else if(mode == QUICK_CHANGE_MODE){
//		userSupportQuickModeAdapter(inKeys,value,KEY_LIGHT_UP,QUICK_KEYS_START_TIMES,&upKeyPressTimes);
//	}
		userSupportQuickModeAdapter(inKeys,value,KEY_LIGHT_DOWN,QUICK_KEYS_START_TIMES,&downKeyPressTimes,EnableQuickKeys);
		userSupportQuickModeAdapter(inKeys,value,KEY_LIGHT_UP,QUICK_KEYS_START_TIMES,&upKeyPressTimes,EnableQuickKeys);
		userSupportQuickModeAdapter(inKeys,value,KEY_ARROW,QUICK_KEYS_START_TIMES,&arrowKeyPressTimes,DisableQuickKeys);
		//userSupportQuickModeAdapter(inKeys,value,KEY_MODE,QUICK_KEYS_START_TIMES,&modeKeyPressTimes,DisableQuickKeys);
		userSupportLongModeAdapter(inKeys,value,KEY_MODE,LONG_MODE_KEY_TIMES,&modeKeyPressTimes);
}
/*************************** (C) COPYRIGHT 2020 Bough*****END OF FILE*****************************/
