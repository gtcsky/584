/**************************************************************************************************
Filename:       command_center.c
Revised:        Date: 2020.8.25
Revision:       1.0

Description:


Copyright 2012 Boutgh R&D.. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Boughg R&D. (the "License").  You may not use this Software unless you
agree to abide by the terms of the License. The License limits your use,
and you acknowledge, that the Software may not be modified,copied or
distributed unless embedded on a Texas Bough LTD., which0 is integrated into
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
#include "types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"

/* GATT */
#include "gatt.h"

#include "gattservapp.h"

/* Profiles */
#include "peripheral.h"
#include "LumeControlprofile.h"
#include "LumeCubeprofile.h"

/* Application */
#include "systems_parameters.h"
#include "bleSmartPeripheral.h"
#include "simpleGATTprofile.h"
#include "command_center.h"
#include "protocol_uart.h"
#include "battery.h"
#include "display.h"
#include "keys.h"
#include "i2c.h"
#include "user_i2c.h"
#include "light.h"
#include "temp.h"
#include "hal_mcu.h"
#include "pwrmgr.h"
#include "user_color.h"
#include "user_lightEffect.h"
#include "clock.h"
#include "stdlib.h"
#include	"user_flash.h"
#include	"string.h"
#include "user_charge.h"
#include "user_utils.h"

/*********************************************************************
* MACROS
*/
#define KEY_FUNC									KEY_MODE//KEY_POWER
#define KEY_LIGHTUP									KEY_LIGHT_UP
#define KEY_LIGHTDOWN								KEY_LIGHT_DOWN
#define KEY_TRUNUP									KEY_ARROW
#define KEY_TRUNDOWN								KEY_MODE//KEY_FUNC_DOWN
#define CHARGE_DET									KEY_CHARGE_DET
#define CHARGE_FULL								KEY_CHARGE_FULL

// Define for test mode
#define TESTITEM_NONE								0
#define TESTITEM_L_R_ON								1
#define TESTITEM_L_R_OFF							2
#define TESTITEM_L_G_ON								3
#define TESTITEM_L_G_OFF							4
#define TESTITEM_L_B_ON								5
#define TESTITEM_L_B_OFF							6
#define TESTITEM_L_C_ON								7
#define TESTITEM_L_C_OFF							8
#define TESTITEM_L_W_ON								9
#define TESTITEM_L_W_OFF							10
#define TESTITEM_I_R								11
#define TESTITEM_I_G								12
#define TESTITEM_I_B								13
#define TESTITEM_I_C								14
#define TESTITEM_I_W								15
#define TESTITEM_I_STAB								16
#define TESTITEM_I_SLEEP							17


#define	TEST_MODE_RATE						0.03
/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* VARIABLES
*/
uint8 command_center_TaskID;   // Task ID for internal task/event processing

static bool CCS_MODE_SYSTEM = false;
static bool CCS_MODE_DISPLAY = false;
static bool Flag_Status_hot_display = false;

extern displayParamsStruct displayParams;

//static uint8 KEY_MODE_Press_times = 0;
static uint8 KEY_FUNC_Press_times = 0;
//static uint8 KEY_LIGHTUP_Press_times = 0;
//static uint8 KEY_LIGHTDOWN_Press_times = 0;
//static uint8 KEY_ARROR_Press_times = 0;

//static uint8 KEY_TRUNDOWN_Press_times = 0;

chargeTypeDef Flag_Status_Charge = Non_Charging;	//1=charging, 2=full

battInfoDef  battInfo;

extern	PcaDataStruct pcaDataStruct;
extern	float vTemperatureCoe;

static bool Flag_bleioc = false;

static bool fIsSystemHot = false;
static bool HotDisplayOn = false;
static float vBattCompensation=0;
//static bool CCS_TIME_REMAIN = false;

//static uint8 EffectMode = CCS_EFFECTMODE_NORMAL;
//static bool EffectModeLedStatus = CCS_EFFECTMODE_LEDOFF;
//const uint8 trigger_brightness[9] = {100,75,50,25,12,6,3,2,1};	//unit = %
const uint16 trigger_ontime_effemode[14] = {0,0,0,0,0,0,8,16,33,66,125,250,500,1000};	//unit = ms
typedef enum{
	NormalMode=0,
	TestModeByUser,
	TestModeByAte,
	BurnInTest,
}testModeDef;

// Define for test mode
static uint8 TESTMODE = NormalMode;
//static uint8 KEY_TESTMODE_Press_times = 0;
static uint8 Test_Item = TESTITEM_NONE;
static uint8 Test_Key_Fun = 30;
static uint8 Test_Key_Light = 30;
static bool Test_Key_Fun_up_pass = false;
static bool Test_Key_Fun_down_pass = false;
static bool Test_Key_Light_up_pass = false;
static bool Test_Key_Light_down_pass = false;
static bool Test_Key_Fun_pass = false;
static uint32 vKeyValue = 0;
//chgSttsDef	fullyParams={
//		.counter=0,
//		.fullyCheckIndex=IDLE_STATUS
//		};
uint8 vAdcErrorTimes=0;
static uint8 errorDisplay=0;

uint8 	vSystemEepromError=0;
uint8	fIsSystemOffByBattLow=0;
extern	colorStructType  color;
uint16 defaultTestTimerConst=1800;		//1800s
uint16 vBurnInTestTimer=0;
uint8 fIsInvalidMacAddr=0;
/*********************************************************************
* FUNCTIONS
*/
static void command_center_ProcessOSALMsg(osal_event_hdr_t* pMsg);
//static void protocol_uartCB_Evt(PROTOCOL_Evt_t* pev);
static void CCS_keysChange_handle(void);
//static void CCS_Effectmode_handle( void );
static void CCS_DATATRANSFER_PROCESS( uint16 flag, displayParamsStruct *ParamsStruct );
static bool CCS_NotifyStatus(void);
static void CCS_EventsPro_TestMode(void);
static void resetStoreConfigTime(void);
void startCharging(void);
/*********************************************************************
 * @fn      command_center_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void command_center_Init(uint8 task_id) {
	hal_gpio_pin_init(P27, OEN);
	hal_gpio_write(P27, 1);
	command_center_TaskID = task_id;

	protocol_CBs_t cbs;
	cbs.task_id = command_center_TaskID;
	cbs.events = CCS_BATT_VALUE_EVT;
	cbs.restartEvents = CCS_BATT_CHECK_EVT;

	hal_gpio_pin_init(SW_RESET_MCU, IE);
	hal_gpio_pull_set(SW_RESET_MCU, WEAK_PULL_UP);

	//eeprom
	userI2cInitial();
	//initial battery
	batt_init();
	batt_RegisterCBs_Value(cbs);
//	osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 1000);

	//initial temp
	cbs.task_id = command_center_TaskID;
	cbs.events = CCS_TEMP_VALUE_EVT;
	cbs.restartEvents = CCS_TEMP_CHECK_EVT;
	temp_init();
	temp_RegisterCBs_Value(cbs);
	osal_start_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT, 1000);
	//initial key
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
	cbs.events = CCS_KEY_CHANGE_EVT;
	keys_RegisterCBs(cbs);
#endif	//#if (defined HAL_KEY) && (HAL_KEY == TRUE)

	//display
	display_Init();
	memset(&battInfo, 0, sizeof(battInfo));
	battInfo.vCurrentBattLv = Max_Batt_level;
	displayParams.battLv = Max_Batt_level;
	loadSystemSetting(&displayParams);
	if (displayParams.arrowIndex >= CustomizeEffect)
		displayParams.arrowIndex = HuesSetting;
	hal_pwrmgr_register(MOD_LCD_On, NULL, NULL);
	fIsInvalidMacAddr = invalidMacAddrCheck();
	if (!vSystemEepromError) {
		hal_pwrmgr_register(MOD_I2C0, NULL, userI2cInitial);
	} else {
		readExceptionStts();
	}
	hal_gpio_pin_init(P27, OEN);
	hal_gpio_write(P27, 0);
	if (!hal_gpio_read(GPIO_KEY_POWER)) {
//			LOG("\n on -low");
		CCS_Systems_on();
		if (!hal_gpio_read(GPIO_KEY_LIGHT_UP) && !hal_gpio_read(GPIO_KEY_LIGHT_DOWN)) {
			TESTMODE = BurnInTest;
			Test_Item = TESTITEM_NONE;
			CCS_EventsPro_TestMode();
		}
		osal_start_timerEx(command_center_TaskID, CHARGE_END_EVENT, 1000);
	} else {
//			LOG("\n on -high");
		Flag_Status_Charge = Sys_Charging;
		CCS_MODE_DISPLAY = true;
		hal_pwrmgr_lock(MOD_LCD_On);
		CCS_Systems_off(Off_By_Hook);
		osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 10);
		startCharging();
		osal_start_timerEx(command_center_TaskID, CHARGE_END_EVENT, 100);
	}
}

/*********************************************************************
 * @fn      bleSmartPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void command_center_ProcessOSALMsg(osal_event_hdr_t* pMsg)
{
	switch(pMsg->event)
	{
		default:
			// do nothing
		break;
	}
}
/*********************************************************************
 *
 * 充电状态,根据当前电池两端电压估算充电电流
 *
 **********************************************************************/
//void getChargingCurrentByVolt(float *vtCurrent,float vtVolt){
//	if(vtVolt<=KEEP_VOLT_THRESHOLD){
//		*vtCurrent=MAX_CHARGE_CURRENT;
//	}else{
//		if(vtVolt>=CHRAGE_FULL_VOLT)
//			*vtCurrent=0.00;
//		else
//			*vtCurrent=MAX_CHARGE_CURRENT*(1-(vtVolt-KEEP_VOLT_THRESHOLD)*(1.0/(CHRAGE_FULL_VOLT-KEEP_VOLT_THRESHOLD)));
//	}
//}
void getChargingCurrentByVolt(float *vtCurrent, float vtVolt) {
	if (vtVolt <= KEEP_VOLT_THRESHOLD) {
		if (CCS_MODE_SYSTEM) {
			*vtCurrent = MAX_CHARGE_CURRENT;
		} else {
			if (vtVolt >= CHG_VOLT_FLOAT_HTHRESHOLD) {
				*vtCurrent = MAX_CHARGE_CURRENT;
			} else if (vtVolt <= CHG_VOLT_FLOAT_EMPTY) {
				*vtCurrent=0.1;
			} else if (vtVolt <= CHG_VOLT_FLOAT_LTHRESHOLD) {
				*vtCurrent = MAX_CHARGE_CURRENT * (1 + 0.3 * (vtVolt-CHG_VOLT_FLOAT_EMPTY) / (CHG_VOLT_FLOAT_LTHRESHOLD - CHG_VOLT_FLOAT_EMPTY));
			} else {
				*vtCurrent = MAX_CHARGE_CURRENT * (1 + 0.3 * (CHG_VOLT_FLOAT_HTHRESHOLD - vtVolt) / (CHG_VOLT_FLOAT_HTHRESHOLD - CHG_VOLT_FLOAT_LTHRESHOLD));
			}
		}
	} else {
		if (vtVolt >= CHRAGE_FULL_VOLT)
			*vtCurrent = 0.01;
		else
			*vtCurrent = MAX_CHARGE_CURRENT * (1 - (vtVolt - KEEP_VOLT_THRESHOLD) * (1.0 / (CHRAGE_FULL_VOLT - KEEP_VOLT_THRESHOLD)));
	}
	//displayFloat(64,3, *vtCurrent,3,'A');
}
/***************************************************************************
 *
 *  	电压补偿.灯开启时的电压补偿
 *
 ***************************************************************************/
void battVoltCompensation(float *volt) {
	float vtCurrent=0;
	float vChargingCurrent=0;
	if((Sys_Charging==Flag_Status_Charge)&&(!hal_gpio_read(GPIO_CHARGE_DET))){
		getChargingCurrentByVolt(&vChargingCurrent,*volt);
		*volt-=vChargingCurrent*BATTERY_RESISTANCE;
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n VchgGap=%d.%dv\n",(uint16)(vChargingCurrent*BATTERY_RESISTANCE),((uint16)(vChargingCurrent*BATTERY_RESISTANCE*1000))%1000);
#endif
	}
	if ( getLightStts()) {																					//在灯开启的情况下
		if (displayParams.arrowIndex<Style1Setting) {
			float temp=(*volt)*PWM_FRQ_CONST;
			if(displayParams.arrowIndex<=SaturationSetting){
				vtCurrent+=pcaDataStruct.valueOfRed*RED_POWER_RATING/temp;
				vtCurrent+=pcaDataStruct.valueOfGreen*GREEN_POWER_RATING/temp;
				vtCurrent+=pcaDataStruct.valueOfBlue*BLUE_POWER_RATING/temp;
			}else if(displayParams.arrowIndex<=ColorTempSetting){
				vtCurrent+=pcaDataStruct.valueOfCw*CW_POWER_RATING/temp;
				vtCurrent+=pcaDataStruct.valueOfMw*MW_POWER_RATING/temp;
			}
		}else{
			if (PreinstallEffect == displayParams.arrowIndex){
				if((displayParams.preinstallEffectNo<=3)||(displayParams.preinstallEffectNo>=8)||(displayParams.preinstallEffectNo==6)){
					vtCurrent=CW_POWER_RATING*MAX_CW_DUTY*displayParams.brightness/((*volt)*100);
				}else{
					vtCurrent=BLUE_POWER_RATING*MAX_MW_DUTY*displayParams.brightness/((*volt)*100);
				}
			}else if(CustomizeEffect == displayParams.arrowIndex){
				if(displayParams.fIsFromRGBMode)
					vtCurrent=CW_POWER_RATING*MAX_CW_DUTY*displayParams.brightness/((*volt)*100);
				else
					vtCurrent=BLUE_POWER_RATING*MAX_MW_DUTY*displayParams.brightness/((*volt)*100);
			}
		}
	}
	*volt += vtCurrent * DISCHARGE_RESISTENCE;
//	LOG("current=%d.%03dv\n b=%d,\t c=%d",(uint16)(vtCurrent * DISCHARGE_RESISTENCE),((uint16)(vtCurrent*DISCHARGE_RESISTENCE*1000))%1000,pcaDataStruct.valueOfBlue,pcaDataStruct.valueOfCw);
#if (BATT_VOLT_INFO_PRINT==1)
	LOG("\n VLedGap=%d.%03dv\n",(uint16)(vtCurrent * DISCHARGE_RESISTENCE),((uint16)(vtCurrent*DISCHARGE_RESISTENCE*1000))%1000);
#endif
//	if(fIsDischarging){
//		*volt+=MAX_DISCHARGE_CURRENT*DISCHARGE_RESISTENCE;
//	}
//	 *volt;
}
/************************************************************************************
 *
 * RisingEnable	:FALSE     level允许上升
 * 				:TRUE     level不允许上升
 *
 *************************************************************************************/
void  processBattLevel(u8 *lv,u8 *finalLv,u8 RisingEnable){
	if (*lv >= 4) {
		if(!RisingEnable){
			if (*finalLv >= Max_Batt_level)
				*finalLv = Max_Batt_level;
		}else{
			*finalLv = Max_Batt_level;
		}
	} else if (*lv >= 3) {
		if(!RisingEnable){
			if (*finalLv >= 3)
				*finalLv = 3;
		}else{
			*finalLv = 3;
		}
	} else if (*lv >= 2) {
		if(!RisingEnable){
			if (*finalLv >= 2)
				*finalLv = 2;
		}else{
			*finalLv = 2;
		}
	} else if (*lv >= 1) {
		if(!RisingEnable){
			if (*finalLv >= 1)
				*finalLv = 1;
		}else{
			*finalLv = 1;
		}
	} else if (*lv < 1) {
		*finalLv = 0;
	}
}
/**************************************************************
 *
 *
 * return percent
 */
uint8 setBattLevel(u8 *lv, float *volt, u8 *finalLv) {
//	if (!fIsSystemInitialing && !vDisableGetBattTimer) {
	if (*volt < 2.0) {
		*lv = 0;
		*volt = 2.0;
		*finalLv = 0;
		LOG("\n error volt=%d",*volt*1000);
		return 0;
	}
	if (Sys_Charging == Flag_Status_Charge) {														//充电IC处于工作状态,读取充电IC中的电池数据

		//#if defined (BATT_VOLT_INFO_DISPLAY)
#if (BATT_VOLT_INFO_DISPLAY==1)
		displayFloat(0, 3, *volt, 3, 'V');
#endif
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n Vavg=%d.%03dv\n",(uint16)*volt,((uint16)(*volt*1000))%1000);
#endif
		battVoltCompensation(volt);
		//#if defined (BATT_VOLT_INFO_DISPLAY)
#if (BATT_VOLT_INFO_DISPLAY==1)
		displayFloat(0, 5, *volt, 3, 'V');
#endif
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n Vcomp=%d.%03dv\n",(uint16)*volt,((uint16)(*volt*1000))%1000);
#endif
		*lv = getBattPercentByVolt(*volt);
		*lv /= 20;
		if (false == getLightStts()) {
			if (*lv >= 4) {
				*finalLv = Max_Batt_level;
			} else if (*lv >= 3 && (*finalLv <= 3)) {												//充电状态,非LED点亮模式,不允许测量误差引起的电量等级下降
				*finalLv = 3;
			} else if (*lv >= 2 && (*finalLv <= 2)) {
				*finalLv = 2;
			} else if (*lv >= 1 && (*finalLv <= 1)) {
				*finalLv = 1;
			} else if (*lv < 1 && (*finalLv <= 0)) {
				*finalLv = 0;
			}
			if (!CCS_MODE_SYSTEM) {
				processBattLevel(lv, finalLv, FALSE);
			}
			return getBattPercentByVolt(*volt);
		} else {
			//LOG("\n light on");
			processBattLevel(lv, finalLv, TRUE);
			return getBattPercentByVolt(*volt);
		}
	} else {
		uint8 tempPercent = 0;
#if (BATT_VOLT_INFO_DISPLAY==1)
		displayFloat(0, 3, *volt, 3, 'V');
#endif
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n Vavg=%d.%03dv\n",(uint16)*volt,((uint16)(*volt*1000))%1000);
#endif
		battVoltCompensation(volt);
		//#if defined (BATT_VOLT_INFO_DISPLAY)
#if (BATT_VOLT_INFO_DISPLAY==1)
		displayFloat(0, 5, *volt, 3, 'V');
#endif
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n Vcomp=%d.%03dv\n",(uint16)*volt,((uint16)(*volt*1000))%1000);
#endif
		*lv = getBattPercentByVolt(*volt);
		tempPercent = *lv;
		//temperatureDisplay(80, 5, *lv);
		*lv /= 20;
		processBattLevel(lv, finalLv, FALSE);
		return tempPercent;
	}
}

void	restartBattInfoCollection(void){
	battInfo.fIsFirst5TimesVoltGot=0;
	battInfo.fIsPowerOnFirstVoltGot=0;
	battInfo.vRecordCnt=0;
}

/***************************************************************************
 *
 *  	电压补偿.灯开启时的电压补偿
 *
 ***************************************************************************/
void chkLightEffectModeVoltCompensation(float *volt, uint8 *flag) {
	float vtMin = 5.0;
	int i = 0;

	battInfo.battArray[battInfo.battRecordIndex] = *volt;
	if (++battInfo.battRecordIndex >= BATT_ARRAY_SIZE)
		battInfo.battRecordIndex = 0;
	if (!battInfo.fIsFirst5TimesVoltGot) {
		battInfo.vRecordCnt++;
		if (battInfo.vRecordCnt >= (GENERATE_BATT_VOLT_TIMES)) {
			battInfo.fIsFirst5TimesVoltGot = true;
			battInfo.vRecordCnt = 0;
		}
	}

	if (fIsLightEffectOn) {
		//LOG("\n  vLightEfftectVoltDectCn%d, \n",battInfo.vLightEfftectVoltDectCnt);
		if (++battInfo.vLightEfftectVoltDectCnt > BATT_ARRAY_SIZE) {
			battInfo.vLightEfftectVoltDectCnt = PARAMS_DATA_RESET;
			for (; i < BATT_ARRAY_SIZE; i++) {
				if (vtMin > battInfo.battArray[i]) {
					vtMin = battInfo.battArray[i];
				}
			}
			*volt = vtMin;
			*flag = 1;
		} else {
			*flag = 0;
		}
	} else {
		//LOG("\n  1st=%d, 5times=%d\n",battInfo.fIsPowerOnFirstVoltGot,battInfo.fIsFirst5TimesVoltGot);
		if (!battInfo.fIsPowerOnFirstVoltGot) {
			battInfo.fIsPowerOnFirstVoltGot = true;
			*flag = 1;										//user the voltage immediately
		} else if (battInfo.fIsFirst5TimesVoltGot) {
			float vtMax = 0;
			float vtTotal = 0;
			//LOG("\n battRecordIndex=%d  \n",battInfo.battRecordIndex);
			uint8 backIndex = (battInfo.battRecordIndex) ? (battInfo.battRecordIndex - 1) : (BATT_ARRAY_SIZE - 1);
			uint8 readIndex = 0;
			//LOG("\n backIndex=%d  \n",backIndex);
			for (uint8 i = 0; i < GENERATE_BATT_VOLT_TIMES; i++) {
				readIndex = ((backIndex - i) >= 0) ? (backIndex - i) : (backIndex + BATT_ARRAY_SIZE - 1 - i);
				if (vtMin > battInfo.battArray[readIndex]) {
					vtMin = battInfo.battArray[readIndex];
				}
				if (vtMax < battInfo.battArray[readIndex]) {
					vtMax = battInfo.battArray[readIndex];
				}
				vtTotal += battInfo.battArray[readIndex];
//				LOG("\ni=%d\n",readIndex);
//				LogFloat(battInfo.battArray[readIndex],3);
			}
			vtTotal -= vtMax;
			vtTotal -= vtMin;
			*volt = vtTotal / (GENERATE_BATT_VOLT_TIMES - 2);
//			LogFloat(*volt,3);
			*flag = 1;
		} else {
			*flag = 1;
		}
	}
}

void	 startLightEffectEvent(void){
	osal_start_reload_timer( command_center_TaskID, TIMER_LIGHT_EFFECT , 5);
}
void	stopLightEffectEvent(void){
	osal_stop_timerEx( command_center_TaskID, TIMER_LIGHT_EFFECT );
}

void startCharging(void) {
	//battInfo.fBattFullyWaiting=0;
#if ((USER_EVNET_PRINT==1)&&(USER_CHARGE_PRINT==1))
	LOG("\n charging start\n");
#endif
	restartBattInfoCollection();
	osal_start_timerEx(command_center_TaskID, CHARGE_END_EVENT, 100);
}

//void setBatteryFully(void) {
//	Flag_Status_Charge = Battery_fully;
//	LOG("\n need check fully \n");
//	osal_stop_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT);
//	if (!CCS_MODE_SYSTEM) {
//		osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 10);
//	} else {
//		osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
//	}
//	battInfo.vCurrentBattLv = Max_Batt_level;
//	displayParams.battLv = battInfo.vCurrentBattLv;
//}

void	 stopCharging(void){
	Flag_Status_Charge = Non_Charging;
	//displayParams.battLv = battInfo.vCurrentBattLv;
	if(displayParams.battLv!=battInfo.vCurrentBattLv){
		displayParams.battLv = battInfo.vCurrentBattLv;
		resetStoreConfigTime();
	}
}

/************************************************************************
 *
 *	Store Current System Setting in the coming  1.5s
 *
 *************************************************************************/
static void resetStoreConfigTime() {
	osal_start_timerEx(command_center_TaskID, STORE_CURRENT_SETTING, TIME_1S5);
}
/*********************************************************************
 * @fn      command_center_ProcessEvent
 *
 * @brief   command center application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 command_center_ProcessEvent(uint8 task_id, uint16 events) {
	VOID task_id; // OSAL required parameter that isn't used in this function

//	if (hal_gpio_read(GPIO_KEY_POWER)){
//		if(!hal_gpio_read(GPIO_CHARGE_DET))
//			CCS_Systems_off();
//	}
	if (events & SYS_EVENT_MSG) {
		uint8* pMsg;

		if ((pMsg = osal_msg_receive(command_center_TaskID)) != NULL) {
			command_center_ProcessOSALMsg((osal_event_hdr_t*) pMsg);

			// Release the OSAL message
			VOID osal_msg_deallocate(pMsg);
		}

		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
	if (events & CCS_BLERF_EVT)
	 {
	  if ( GetgapProfileState() == GAPROLE_CONNECTED )
	  {
	   osal_start_timerEx( command_center_TaskID, CCS_BLERF_EVT, 5 );
	  }
	  else
	  {
	   uint8 current_adv_enabled_status = false;
	   GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &current_adv_enabled_status );
	  }

	  return (events ^ CCS_BLERF_EVT);
	 }


	if(events &TIMER_LIGHT_EFFECT){
		lightEffectFunc();
		return (events ^ TIMER_LIGHT_EFFECT);
	}

	if (events & CCS_BATT_CHECK_EVT) {
		batt_measure();
		return (events ^ CCS_BATT_CHECK_EVT);
	}
	if (events & STORE_CURRENT_SETTING) {
		if(!TESTMODE){
			storeSystemSetting(&displayParams);
			LOG("\n end of  store");
		}
		return (events ^ STORE_CURRENT_SETTING);
	}
	if (events & CHARGE_END_EVENT) {
		chargeSttsDectEvent();
		return (events ^ CHARGE_END_EVENT);
	}
	if (events & SWITCH_EVT) {
		uint8 switchStts = 0;
		switchSttsDectEvent(&switchStts);
//		LOG("\n stts=%d  ,%d",switchStts,CCS_MODE_SYSTEM);
		if(SelectOff==switchStts){
			if(fIsSystemOffByBattLow)
				fIsSystemOffByBattLow=0;
			if (CCS_MODE_SYSTEM) {
				CCS_Systems_off(Off_By_Manual);
			}
		}else if(SelectOn==switchStts){
			if (!CCS_MODE_SYSTEM) {
				if((Flag_Status_Charge == Sys_Charging||Battery_fully==Flag_Status_Charge)&&!fIsSystemOffByBattLow){
					CCS_Systems_on();
				}
			}
		}
		return (events ^ SWITCH_EVT);
	}

	if (events & CCS_BATT_VALUE_EVT) {
//		return (events ^ CCS_BATT_VALUE_EVT);
		uint8 batt_perc = batt_get_percent();
		uint16 batt_vol = batt_get_voltage();
#if (BATT_VOLT_INFO_PRINT==1)
		LOG("\n Vimmi=%d.%03dv\n",batt_vol/1000,batt_vol%1000);
#endif
		getChargeStts(&Flag_Status_Charge);
		if (TESTMODE&&TESTMODE!=BurnInTest) {
			displayFloat(0,0,batt_vol*0.001,2,'v');
		}
//		LOG("\n   ad event \n");
//		LOG("\n_stts:%d \n",getClkRstStts(MOD_ADCC)>>16);
		//LOG("\n   Flag_Status_Charge=%d \n",Flag_Status_Charge);
		if (CCS_MODE_SYSTEM) {
			uint8 tempLv = batt_perc / 20;
			float vBattVolt = batt_vol * 0.001;
			uint8 fIsNeedUpdateBattLv = 0;
			chkLightEffectModeVoltCompensation(&vBattVolt, &fIsNeedUpdateBattLv);
			if (fIsNeedUpdateBattLv){
				setBattLevel(&tempLv, &vBattVolt, &battInfo.vCurrentBattLv);
			}
			if(displayParams.battLv!=battInfo.vCurrentBattLv){
				displayParams.battLv = battInfo.vCurrentBattLv;
				resetStoreConfigTime();
			}
			vBattCompensation=vBattVolt;
			if (!fIsLightEffectOn || (fIsLightEffectOn && fIsNeedUpdateBattLv)) {
				if (vBattVolt <= BATT_VOLT_MIN) {
					if (++battInfo.vBattMinTimes > 1) {
						if (CCS_MODE_SYSTEM) {
							if (Flag_Status_Charge != Sys_Charging) {
#if (USER_EVNET_PRINT==1)
								LOG("\n Turn Off by battery low");
#endif
								CCS_Systems_off(Off_By_Battery);
							} else if (battInfo.fIsFirst5TimesVoltGot) {
#if (USER_EVNET_PRINT==1)
								LOG("\n Turn Off by battery low@Charging");
#endif
//								if(vBattVolt<=BATT_VOLT_MIN-0.20){
									CCS_Systems_off(Off_By_Battery);
									fIsSystemOffByBattLow=1;
//								}
//								else{
//									battInfo.vBattMinTimes=0;
//								}
							}
						}
					}
				} else {
					battInfo.vBattMinTimes = 0;
				}
			}
			if (Flag_Status_Charge == Battery_fully) {
				battInfo.vCurrentBattLv = Max_Batt_level;
				if(displayParams.battLv!=battInfo.vCurrentBattLv){
					displayParams.battLv = battInfo.vCurrentBattLv;
					resetStoreConfigTime();
				}
				batterDisplay(Max_Batt_level);
			} else if (Flag_Status_Charge == Sys_Charging) {
#if ((USER_EVNET_PRINT==1)&&(USER_CHARGE_PRINT==1))
				LOG("\n Charging");
#endif
				if (!fIsSystemHot) {
					if (!battInfo.vDisplayBattLv) {
						if (battInfo.vCurrentBattLv > 0)
							battInfo.vDisplayBattLv = battInfo.vCurrentBattLv - 1;
					}
					//LOG("\n  battInfo.vDisplayBattLv:%d \n",battInfo.vDisplayBattLv);
					batterDisplay(battInfo.vDisplayBattLv++);
					battInfo.vDisplayBattLv = (battInfo.vDisplayBattLv > Max_Batt_level) ? 0 : battInfo.vDisplayBattLv;
				}
				osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
			} else {
				battInfo.vDisplayBattLv = 0;
				stopCharging();
				if(!fIsSystemHot)
					batterDisplay(battInfo.vCurrentBattLv);
			}
			//}
			osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
		} else {
			if (Flag_Status_Charge == Battery_fully) {
				if (!CCS_MODE_DISPLAY) {
					//lcd_on();
					reInitialLCD();
					lcd_clear();
					CCS_MODE_DISPLAY = true;
				}

				batteryPercentDisplay(100);
				batterDisplay(Max_Batt_level);
				if(displayParams.battLv!=battInfo.vCurrentBattLv){
					displayParams.battLv = battInfo.vCurrentBattLv;
					resetStoreConfigTime();
				}
				osal_stop_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT);
				osal_start_reload_timer(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
			} else if (Flag_Status_Charge == Sys_Charging) {
#if ((USER_EVNET_PRINT==1)&&(USER_CHARGE_PRINT==1))
				LOG("  \n __Sys_Charging__ \n  ");
#endif
				if (!CCS_MODE_DISPLAY) {
					reInitialLCD();
					lcd_clear();
					CCS_MODE_DISPLAY = true;
					battInfo.vDisplayBattLv=0;
				}
				uint8 tempLv = batt_perc / 20;
				float vBattVolt = batt_vol * 0.001;

				uint8 fIsNeedUpdateBattLv = 0;
				chkLightEffectModeVoltCompensation(&vBattVolt, &fIsNeedUpdateBattLv);
				if (fIsNeedUpdateBattLv){
					battInfo.vBattPercent=setBattLevel(&tempLv, &vBattVolt, &battInfo.vCurrentBattLv);
				}
				if (battInfo.vBattPercent > 99) {
					battInfo.vBattPercent = 99;
				}
				//batteryPercentDisplay(battInfo.vBattPercent);
				if(displayParams.battLv!=battInfo.vCurrentBattLv){
					displayParams.battLv = battInfo.vCurrentBattLv;
					resetStoreConfigTime();
				}
				if(!battInfo.vDisplayBattLv){
					if(battInfo.vCurrentBattLv > 0)
						battInfo.vDisplayBattLv=battInfo.vCurrentBattLv - 1;
				}
				batterDisplay(battInfo.vDisplayBattLv++);
				battInfo.vDisplayBattLv=(battInfo.vDisplayBattLv>Max_Batt_level)?0:battInfo.vDisplayBattLv;
				osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
			} else {
				if(!battInfo.vDisplayBattLv)
					battInfo.vDisplayBattLv=0;
				LOG("  \n __Non__ \n  ");
				lcd_off();
				CCS_MODE_DISPLAY = false;
				stopCharging();
				osal_stop_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT);
			}
		}
		return (events ^ CCS_BATT_VALUE_EVT);
	}


	if (events & CCS_KEY_CHANGE_EVT) {
		CCS_keysChange_handle();
		return (events ^ CCS_KEY_CHANGE_EVT);
	}

	if (events & CCS_BLEIOC_EVT) {
		if (CCS_MODE_SYSTEM) {
			if (GetgapProfileState() == GAPROLE_CONNECTED) {
				Flag_bleioc = true;
			} else {
				Flag_bleioc^=0x01;
				osal_start_timerEx(command_center_TaskID, CCS_BLEIOC_EVT, 500);
			}

			updateBLEDisplay(Flag_bleioc);
		}
//		else
//		{
//			Flag_bleioc = false;
//			updateBLEDisplay(Flag_bleioc);
//		}

		return (events ^ CCS_BLEIOC_EVT);
	}

//	if (events & CCS_LEDFIXMODE_EVT) {
//
//		return (events ^ CCS_LEDFIXMODE_EVT);
//	}
//
//	if (events & CCS_LEDEFFECTMODE_EVT) {
//		CCS_Effectmode_handle();
//		return (events ^ CCS_LEDEFFECTMODE_EVT);
//	}
	if (events & CCS_TEMP_CHECK_EVT) {
		temp_measure();
//		LOG("\n start temp\n");
		if (fIsSystemHot) {
			osal_start_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT, TEMPERATURE_DET_PERIOD_HOT_1S);
		}

		else {
			if (CCS_MODE_SYSTEM)
				osal_start_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT, TEMPEARTURE_DET_PERIOD_2S);
		}

		return (events ^ CCS_TEMP_CHECK_EVT);
	}

	if (events & CCS_TEMP_VALUE_EVT) {

		float temp = temp_get_value();
		if (TESTMODE&&TESTMODE!=BurnInTest) {
			displayFloat(80,6,temp,1,ICON_Degree_ADDRESS);
		}
//		displayParams.hues = (uint16)temp;
//		updateHuesDisplay(&displayParams);
		//OLED_ShowNum(64,2,temp);
		//displayFloat(64,2,temp,1,' ');
#if (TEMPERATURE_INFO_PRINT==1)
		LOG("\n temp:%d.%d",(uint16)temp,((uint16)(temp*10))%10);
#endif
		if (CCS_MODE_SYSTEM) {
			if (temp > OVER_TEMPERATURE_VOLT_Lv1) {
				CCS_Systems_off(Off_By_Temperature);
			}

			else if (temp > OVER_TEMPERATURE_VOLT_Lv0) {
				if (!fIsSystemHot) {
					fIsSystemHot = true;
					vTemperatureCoe=TEMPERATURE_COE_LOW;
					uint16 flag = CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
					CCS_DATATRANSFER_PROCESS(flag, &displayParams);
				}

				if (HotDisplayOn) {
					HotDisplayOn = false;
					updateHotDisplay(false);
				}

				else {
					HotDisplayOn = true;
					updateHotDisplay(true);
				}
			}

			else if (temp < NORMAL_TEMPERATURE_VOLT && fIsSystemHot) {
				fIsSystemHot = false;
				vTemperatureCoe=TEMPERATURE_COE_NORMAL;
				if (HotDisplayOn) {
					HotDisplayOn = false;
					updateHotDisplay(false);
				}

				fIsSystemHot = false;
				batterDisplay(displayParams.battLv);

				uint16 flag = CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
				CCS_DATATRANSFER_PROCESS(flag, &displayParams);
			}

			else {
				if (fIsSystemHot) {
					if (HotDisplayOn) {
						HotDisplayOn = false;
						updateHotDisplay(false);
					}

					else {
						HotDisplayOn = true;
						updateHotDisplay(true);
					}
				}
			}
		}

		return (events ^ CCS_TEMP_VALUE_EVT);
	}
	if (events & CCS_GATT_NOTIFY_EVT) {
		CCS_NotifyStatus();
		LOG("\n___connected!___\n");
		return (events ^ CCS_GATT_NOTIFY_EVT);
	}

	if (events & CCS_TESTMODE_EVT) {
		CCS_EventsPro_TestMode();
		return (events ^ CCS_TESTMODE_EVT);
	}

	// Discard unknown events
	return 0;
}

///*********************************************************************
// * @fn      protocol_uartCB_Evt
// *
// * @brief   command center application Task event processor.  This function
// *          is called to process all events for the task.  Events
// *          include timers, messages and any other user defined events.
// *
// * @param   task_id  - The OSAL assigned task ID.
// * @param   events - events to process.  This is a bit map and can
// *                   contain more than one event.
// *
// * @return  events not processed
// */
//static void protocol_uartCB_Evt(PROTOCOL_Evt_t* pev)
//{
//	switch(pev->event)
//	{
//		
//	}
//}
/***************************************************
 *
 *		Up Key Function
 *
 ****************************************************/
void	  keyFuncUpProcess(u16 *flag){
	restartBattInfoCollection();
	 if (CustomizeEffect == displayParams.arrowIndex){
		 LedStruct.pfncustomizeEffectOverCallBack();
		 turnOffAllLightEffect();
	}
	if (HuesSetting == displayParams.arrowIndex) {
		if (displayParams.hues < 360) {
			displayParams.hues++;
		} else {
			displayParams.hues = 0;
		}
		updateHuesDisplay(&displayParams);

		*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;

		*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
	} else if (SaturationSetting == displayParams.arrowIndex) {
		if (displayParams.saturation < MAX_Saturation) {
			displayParams.saturation++;
			updateSaturationDisplay(&displayParams);

			*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
			displayParams.command = CCS_LIGHT_MODE_HSI;
			displayParams.mode = 0;

			*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
		}
	} else if (ColorTempSetting == displayParams.arrowIndex) {
		if (displayParams.colorTemperature < MAX_ColorTemp) {
			displayParams.colorTemperature++;
			updateColorTempDisplay( &displayParams);

			*flag = CCS_FLAG_TEMPERATURE;
			displayParams.command = CCS_LIGHT_MODE_CCT;
			displayParams.mode = 0;

			*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
		}
	} else if (Style1Setting == displayParams.arrowIndex) { //灯效1 调节
		if (displayParams.style1Value++ >= MAX_LightEffect)
			displayParams.style1Value = 0;
		displayParams.mode = displayParams.style1Value+1;
		displayParams.preinstallEffectNo = displayParams.style1Value + 1;
		updateLightEffectDisplay(&displayParams);
//		LOG("mode=%d  value=%d",displayParams.mode,displayParams.style1Value);
		//startLightEffect(&displayParams);

		*flag = CCS_FLAG_EFFECTSMODE;
		displayParams.command = CCS_LIGHT_MODE_FIXEDMODE;
		*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);

	}
}
/***************************************************
 *
 *		Down Key Function
 *
 ****************************************************/
void	  keyFuncDownProcess(u16 *flag){
	restartBattInfoCollection();
	 if (CustomizeEffect == displayParams.arrowIndex){
		 LedStruct.pfncustomizeEffectOverCallBack();
		 turnOffAllLightEffect();
	}
	if (HuesSetting == displayParams.arrowIndex)
	{
		if (displayParams.hues > 0)
		{
			displayParams.hues--;
		}
		else
		{
			displayParams.hues = 360;
		}
		updateHuesDisplay(&displayParams);

		*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;

		*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
	}
	else if (SaturationSetting == displayParams.arrowIndex)
	{
		if (displayParams.saturation > MIX_Saturation)
		{
			displayParams.saturation--;
			updateSaturationDisplay(&displayParams);

			*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
			displayParams.command = CCS_LIGHT_MODE_HSI;
			displayParams.mode = 0;

			*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
		}
	}
	else if (ColorTempSetting == displayParams.arrowIndex)
	{
		if (displayParams.colorTemperature > MIN_ColorTemp)
		{
			displayParams.colorTemperature--;
			updateColorTempDisplay( &displayParams);

			*flag = CCS_FLAG_TEMPERATURE;
			displayParams.command = CCS_LIGHT_MODE_CCT;
			displayParams.mode = 0;

			*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
		}
	} else if (Style1Setting == displayParams.arrowIndex) { //灯效1 调节
		if (displayParams.style1Value-- <= 0)
			displayParams.style1Value = MAX_LightEffect;
		displayParams.mode = displayParams.style1Value+1;
		displayParams.preinstallEffectNo=displayParams.style1Value+1;
//		LOG("mode=%d  value=%d",displayParams.mode,displayParams.style1Value);
		updateLightEffectDisplay(&displayParams);
//		startLightEffect(&displayParams);

		*flag = CCS_FLAG_EFFECTSMODE;
		displayParams.command = CCS_LIGHT_MODE_FIXEDMODE;
		*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);

	}
}
/***************************************************
 *
 *		Function  Key Function
 *
 ****************************************************/
void keyFunctionProcess(u16 *flag,uint16 key) {
	restartBattInfoCollection();

	if (key & KEY_MODE_LONG) {
		if(displayParams.arrowIndex== Max_Arrow_Index){
			turnOffAllLightEffect();
			displayParams.arrowIndex--;
		}
		else if (displayParams.arrowIndex<= Mix_Arrow_Index) {
			displayParams.arrowIndex= Max_Arrow_Index ;
		}else{
			displayParams.arrowIndex--;
		}
	} else {
		if (displayParams.arrowIndex++ >= Max_Arrow_Index) {
			displayParams.arrowIndex = Mix_Arrow_Index;
			turnOffAllLightEffect();
		}
	}


	//HSI
	if (HuesSetting == displayParams.arrowIndex) {
		*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;
	}

	//saturation
	else if (SaturationSetting == displayParams.arrowIndex) {
		*flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;
	}

	//colortemp
	else if (ColorTempSetting == displayParams.arrowIndex) {
		*flag = CCS_FLAG_TEMPERATURE;
		displayParams.command = CCS_LIGHT_MODE_CCT;
		displayParams.mode = 0;
	}

	//fixed mode
	else if (Style1Setting == displayParams.arrowIndex) {
		*flag = CCS_FLAG_EFFECTSMODE;
		displayParams.command = CCS_LIGHT_MODE_FIXEDMODE;
		displayParams.mode = displayParams.style1Value + 1;
		displayParams.preinstallEffectNo = displayParams.style1Value + 1;
		//startLightEffect(&displayParams);
	}

	updateArrowDisplay(&displayParams);
	*flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
	CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
}
/***************************************************
 *
 *		Brightness +  Key Function
 *
 ****************************************************/
void keyBrightnessIncProcess(u16 *flag) {
//	displayParams.command = CCS_LIGHT_MODE_CCT;

//	LOG("\nDisplayModeIndex=%d \n",displayParams.DisplayModeIndex);			//	参考值: displayModeEnum
//	LOG("\n arrowIndex=%d \n",displayParams.arrowIndex);			//	光标序号	(0~6)		参考值:settingIndexEnum
//	LOG("\n battLv=%d \n",displayParams.battLv);			//	电量标志	(0~6)
//	LOG("\n command=%d \n",displayParams.command);			//
//	LOG("\n mode=%d \n",displayParams.mode);			//
//	LOG("\n hues=%d \n",displayParams.hues);			//
//	LOG("\n saturation=%d \n",displayParams.saturation);			//
//	LOG("\n brightness=%d \n",displayParams.brightness);			//
//	LOG("\n colorTemperature=%d \n",displayParams.colorTemperature);			//
//	LOG("\n style1Value=%d \n",displayParams.style1Value);			//
//	LOG("\n effectmode=%d \n",displayParams.effectmode);				//	自定义特效模式  1/2:呼吸灯/爆闪
//	LOG("\n times=%d \n",displayParams.times);
//	LOG("\n trigger_brightness=%d \n",displayParams.trigger_brightness);
//	LOG("\n trigger_ontime=%d \n",displayParams.trigger_ontime);
//	LOG("\n preinstallEffectNo=%d \n",displayParams.preinstallEffectNo);	//	预设特效编号
//	LOG("\n customizeEffectTimes=%d \n",displayParams.customizeEffectTimes);	//	自定义特效循环次数 1~99:次数   100:无限循环
//	LOG("\n customizeEffectFreq=%d \n",displayParams.customizeEffectFreq);	//	自定义特效频率
//	LOG("\n customizeOneShot=%d \n",displayParams.customizeOneShot);	//	自定义特效单次闪烁
//	LOG("\n fIsEffectMode=%d \n",displayParams.fIsEffectMode);	//	自定义特效模式+预设特效模式=1.普通模式=0
//	LOG("\n fIsFromRGBMode=%d \n",displayParams.fIsFromRGBMode);	//	0/1 :从RGB/色温模式进入自定义特效模式
//	LOG("\n backupArrowIndex=%d \n",displayParams.backupArrowIndex);	//	0/1 :从RGB/色温模式进入自定义特效模式
	restartBattInfoCollection();
	if (displayParams.brightness < 100) {
		displayParams.brightness++;
		updateBrightnessDisplay(&displayParams);

		*flag = CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		displayParams.mode = 0;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
	}
}
/***************************************************
 *
 *		Brightness -  Key Function
 *
 ****************************************************/
void keyBrightnessDecProcess(u16 *flag) {
	restartBattInfoCollection();
	if (displayParams.brightness > 1) {
		displayParams.brightness--;
		updateBrightnessDisplay(&displayParams);

		*flag = CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
		displayParams.mode = 0;
		CCS_DATATRANSFER_PROCESS(*flag, &displayParams);
	}
}
/*********************************************************************
 * @fn      command_center_keysChange_handle
 *
 * @brief   command center application Task event handle.
 *
 * @param   none
 *
 * @return  none
 */


//		KEY_LIGHT_UP			0x00000002
//		LONG_LIGHT_UP			0x00000200
//		LONG_LIGHT_GOT			0x00020000
//		KEY_LIGHT_HOLDING		0x02000000
//
//
static void CCS_keysChange_handle(void)
{
	uint16 keys = keys_read();
	uint16 userKey=0;
//	userKeysAdapter(keys,&vKeyValue,SUPPORT_LONG_MODE);
	userKeysAdapter(keys,&vKeyValue);
	if(vKeyValue&0x0000ffff){
		userKey=vKeyValue&0x0000ffff;
		vKeyValue&=~KEYS_MASK;
	}

	if (keys & CHARGE_DET) {
		keys &= ~(CHARGE_DET );
#if ((USER_EVNET_PRINT==1)&&(USER_CHARGE_PRINT==1))
		LOG("\n start chg");
#endif
		if (Flag_Status_Charge != Sys_Charging) {
			Flag_Status_Charge = Sys_Charging;

			if (!CCS_MODE_SYSTEM) {
				osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 10);
				startCharging();
			}
		}
		if(CCS_MODE_SYSTEM){
			startCharging();
			if(!osal_get_timeoutEx(command_center_TaskID, CCS_BATT_CHECK_EVT))
				osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 1000);
		}
		osal_start_timerEx(command_center_TaskID, CHARGE_END_EVENT, 100);
	}
	else {
		keys &= ~(CHARGE_DET | CHARGE_FULL);
		if (hal_gpio_read(GPIO_CHARGE_DET)) {
			if (Flag_Status_Charge != Non_Charging) {
				stopCharging();
				if (!CCS_MODE_SYSTEM) {
					osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 10);
				}
			}
		}
		//battInfo.fBattFullyWaiting = 0;
		if(CCS_MODE_SYSTEM){
			if(!osal_get_timeoutEx(command_center_TaskID, CCS_BATT_CHECK_EVT))
				osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 1000);
		}
	}
	
	if (userKey &KEY_MODE_LONG && keys & KEY_LIGHTUP && keys & KEY_LIGHTDOWN) {
//		if (KEY_TESTMODE_Press_times > 50) {
			Test_Key_Fun_up_pass = false;
			Test_Key_Fun_down_pass = false;
			Test_Key_Light_up_pass = false;
			Test_Key_Light_down_pass = false;

//			KEY_TESTMODE_Press_times = 0;
			TESTMODE = TestModeByUser;
			Test_Item = TESTITEM_NONE;
			Test_Key_Fun = 30;
			Test_Key_Light = 30;
			CCS_EventsPro_TestMode();
//		}
//			else {
//			KEY_TESTMODE_Press_times++;
//		}

		osal_start_timerEx(command_center_TaskID, CCS_KEY_CHANGE_EVT, 50);
		return;
	} else {
//		KEY_TESTMODE_Press_times = 0;
	}
	if (TESTMODE) {
		if (keys & KEY_FUNC) {
			KEY_FUNC_Press_times++;
//			if (KEY_FUNC_Press_times > 50) {
//				KEY_FUNC_Press_times = 0;
//				CCS_Systems_off();
//				return;
//			} else {
//				KEY_FUNC_Press_times++;
//			}
		} else if (KEY_FUNC_Press_times) {
			if (TESTMODE == TestModeByUser) {
				if (!Test_Item) {
					if (Test_Key_Fun_up_pass && Test_Key_Fun_down_pass && Test_Key_Light_up_pass && Test_Key_Light_down_pass) {
						//displayFactoryInfo(ClearAllInfo);
						uint16 batt_vol = batt_get_voltage();
						float temp = temp_get_value();
						if (batt_vol < 4000 || batt_vol >= 4300) {
							if (!errorDisplay)
								displayFactoryInfo(ClearAllInfo);
							displayFactoryInfo(VoltageError);
							errorDisplay = 1;
						} else {
							if (errorDisplay) {
								displayFactoryInfo(ClearAllInfo);
								errorDisplay = 0;
							}
						}
						if ((temp != 0) && (temp < 15 || temp > 40)) {
							if (!errorDisplay)
								displayFactoryInfo(ClearAllInfo);
							displayFactoryInfo(TemperatureError);
							errorDisplay = 1;
						}
						if (!errorDisplay) {
							Test_Item = TESTITEM_I_R;
							CCS_EventsPro_TestMode();
						}
					}
				}
			}else if (TESTMODE == TestModeByAte) {
					OLED_ShowNum(80, 0, 8);
				if (Test_Key_Fun_up_pass && Test_Key_Fun_down_pass && Test_Key_Light_up_pass && Test_Key_Light_down_pass) {
					OLED_ShowNum(80, 0, 9);
					if(!Test_Key_Fun_pass){
						uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_Key };
						hal_uart_send_buff(data,sizeof(data));
						Test_Key_Fun_pass=TRUE;
					}
				}
			}
			KEY_FUNC_Press_times = 0;
			return;
		} else {
			KEY_FUNC_Press_times = 0;
			if (Test_Item == TESTITEM_NONE) {
			}
		}
		if (Test_Item == TESTITEM_NONE) {
			if (keys & KEY_TRUNDOWN) {
				if (Test_Key_Fun < 50) {
					Test_Key_Fun++;
					OLED_ShowNum(100, 2, Test_Key_Fun);
					if (Test_Key_Fun > 40) {
						Test_Key_Fun_up_pass = true;
					}
				}
			}
			if (keys & KEY_TRUNUP) {
				if (Test_Key_Fun > 10) {
					Test_Key_Fun--;
					OLED_ShowNum(100, 2, Test_Key_Fun);

					if (Test_Key_Fun < 20) {
						Test_Key_Fun_down_pass = true;
					}
				}
			}
			if (userKey & KEY_LIGHTUP) {
				if (Test_Key_Light < 50) {
					Test_Key_Light++;
					OLED_ShowNum(100, 4, Test_Key_Light);
					if (Test_Key_Light > 40)
						Test_Key_Light_up_pass = true;
				}
			}
			if (userKey & KEY_LIGHTDOWN) {
				if (Test_Key_Light > 10) {
					Test_Key_Light--;
					OLED_ShowNum(100, 4, Test_Key_Light);
					if (Test_Key_Light < 20)
						Test_Key_Light_down_pass = true;

				}
			}
		}
		if (TESTMODE == BurnInTest) {
			if (Test_Item >= TESTITEM_L_R_OFF)
				if ((keys & KEY_TRUNUP) || (keys & KEY_LIGHTUP)) {
					defaultTestTimerConst = (defaultTestTimerConst >= 15000) ? 600 : (defaultTestTimerConst + 1200);
				} else if ((keys & KEY_TRUNDOWN) || (keys & KEY_LIGHTDOWN)) {
					defaultTestTimerConst = (defaultTestTimerConst <= 600) ? 15000 : (defaultTestTimerConst - 1200);
				}
			vBurnInTestTimer = defaultTestTimerConst;
			displayFloat(20 + 16, 2, vBurnInTestTimer, 0, ' ');

			if (keys) {
				if(keys & KEY_FUNC)
					osal_start_timerEx(command_center_TaskID, CCS_KEY_CHANGE_EVT, 20);
				else{
					osal_start_timerEx(command_center_TaskID, CCS_KEY_CHANGE_EVT, 150);
				}
			}
			return;
		}

		if (keys) {
			osal_start_timerEx(command_center_TaskID, CCS_KEY_CHANGE_EVT, 20);
		}
		return;
	}
//	if(userKey){
//		LOG("\n userKey=0x%04x",userKey);
//	}
	//On/Off
	if (CCS_MODE_SYSTEM&&userKey&&!fIsInvalidMacAddr) {
		uint16 flag = 0;
		if (userKey & KEY_MODE) {						//Function switch
//			LOG("\n mode key");
			keyFunctionProcess(&flag,userKey);
		}else if(userKey &KEY_MODE_LONG){
			keyFunctionProcess(&flag,userKey);
		}
		if (userKey& KEY_ARROW) {
//			LOG("\n arrow key");
			//KEY_ARROR_Press_times = 0;
			if(displayParams.isBrightnessAdjisting>1){
				displayParams.isBrightnessAdjisting&=0x01;
			}
			displayParams.isBrightnessAdjisting^=1;
			switchArrow(&displayParams);
		}

		//light up
		if (userKey & KEY_LIGHTUP) {
			//KEY_LIGHTUP_Press_times = 0;
			if(displayParams.isBrightnessAdjisting)
				keyBrightnessIncProcess(&flag);
			else
				keyFuncUpProcess(&flag);
		}

		//light down
		if (userKey& KEY_LIGHTDOWN) {
			//KEY_LIGHTDOWN_Press_times = 0;
			if (displayParams.isBrightnessAdjisting)
				keyBrightnessDecProcess(&flag);
			else
				keyFuncDownProcess(&flag);
		}
	}

	//Start a new timer is system is onmode
	if (keys) {
		osal_start_timerEx(command_center_TaskID, CCS_KEY_CHANGE_EVT, 20);
	}

}

///*********************************************************************
//* @fn      CCS_LUMECONTROL_DATA_PROCESS
//*
//* @brief   Process an package of recive from apps
//*
//* @param   pMsg - message to process
//*
//* @return  none
//*/
//bool CCS_LUMECONTROL_DATA_PROCESS( uint8 *pPkg )
//{
//
//
//	return true;
//}

/*********************************************************************
* @fn      CCS_SIMPLEGATT_DATA_PROCESS
*
* @brief   Process an package of recive from apps
*
* @param   pMsg - message to process
*
* @return  bool
*/
bool CCS_SIMPLEGATT_DATA_PROCESS( uint8 *pPkg )
{
	if ( *(pPkg+1) )
	{
		uint16 flag = 0;

		if ( *(pPkg+1) == CCS_LIGHT_MODE_CCT )	//白光模式
		{
			if(displayParams.arrowIndex>=PreinstallEffect){
				turnOffAllLightEffect();
			}
			displayParams.arrowIndex = ColorTempSetting;
			updateArrowDisplay(&displayParams);

			//亮度
			if(displayParams.brightness!= *(pPkg+3)){
				displayParams.isBrightnessAdjisting=true;
				displayParams.brightness = *(pPkg+3);
			}else{
				displayParams.isBrightnessAdjisting=false;
			}
			switchArrow(&displayParams);
			updateBrightnessDisplay(&displayParams);

			//色温
			if ( *(pPkg+4) )
			{
				flag = CCS_FLAG_TEMPERATURE;
				displayParams.colorTemperature = *(pPkg+4);
				updateColorTempDisplay( &displayParams);
			}

			displayParams.command = CCS_LIGHT_MODE_CCT;
			displayParams.mode = 0;

			flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
		}

		else if ( *(pPkg+1) == CCS_LIGHT_MODE_HSI )	//彩光模式
		{
//			uint32 test=clk_hclk();
//			LOG("\n__CLK:%d__\n",test/1000);
			if(displayParams.arrowIndex>=PreinstallEffect){
				turnOffAllLightEffect();
			}
			//亮度

//			displayParams.brightness = *(pPkg+3);
//			updateBrightnessDisplay(&displayParams);
			if(displayParams.brightness!= *(pPkg+3)){
				displayParams.isBrightnessAdjisting=true;
				displayParams.brightness = *(pPkg+3);
				updateBrightnessDisplay(&displayParams);
			}else{
				displayParams.isBrightnessAdjisting=false;
			}

			//饱和度
			if (*(pPkg + 7) <= 100) {
				if(*(pPkg + 6) >1){
					return true;					//error data	360=0x168
				}
				if ((displayParams.arrowIndex != HuesSetting) && (displayParams.arrowIndex != SaturationSetting)) {
					flag |= CCS_FLAG_SATURATION | CCS_FLAG_HUE;
					displayParams.arrowIndex = HuesSetting;
					displayParams.saturation = *(pPkg + 7);
					displayParams.hues = *(pPkg + 5);
					displayParams.hues |= *(pPkg + 6) << 8;
					lcd_clear();
					updateArrowDisplay(&displayParams);
					displayParams.isBrightnessAdjisting=false;
					//updateHuesDisplay(&displayParams);
				} else {
					flag |= CCS_FLAG_SATURATION | CCS_FLAG_HUE;
					uint8 differentHues=0;
					uint8 differentSaturation=0;
					if(( *(pPkg + 5)|( *(pPkg + 6) << 8)) != displayParams.hues)
						differentHues=1;
					if(*(pPkg + 7) !=  displayParams.saturation)
						differentSaturation=1;
					if(differentHues||(differentHues&differentSaturation)){
						displayParams.saturation = *(pPkg + 7);
						displayParams.hues = *(pPkg + 5);
						displayParams.hues |= *(pPkg + 6) << 8;
						if (displayParams.arrowIndex != HuesSetting) {
							clearSecondLineDisplay();
							displayParams.arrowIndex = HuesSetting;
							updateArrowDisplay(&displayParams);
						}else{
							updateHuesDisplay(&displayParams);
						}
						displayParams.isBrightnessAdjisting=false;
					}else if(differentSaturation){
						displayParams.saturation = *(pPkg + 7);
						displayParams.hues = *(pPkg + 5);
						displayParams.hues |= *(pPkg + 6) << 8;
						if (displayParams.arrowIndex != SaturationSetting) {
							//lcd_clear();
							clearSecondLineDisplay();
							displayParams.arrowIndex = SaturationSetting;
							updateArrowDisplay(&displayParams);
						}else{
							updateSaturationDisplay(&displayParams);
						}
						displayParams.isBrightnessAdjisting=false;
					}
				}
			}
			switchArrow(&displayParams);

//			//饱和度
//			if ( *(pPkg+7) <= 100 )
//			{
//				flag |= CCS_FLAG_SATURATION;
//				displayParams.saturation = *(pPkg+7);
//			}
//
//			//色相
//			{
//				flag |= CCS_FLAG_HUE;
//				if(displayParams.arrowIndex!=HuesSetting)
//					lcd_clear();
//				displayParams.arrowIndex = HuesSetting;
//				updateArrowDisplay(&displayParams);
//				displayParams.hues = *(pPkg+5);
//				displayParams.hues |= *(pPkg+6)<<8;
//				updateHuesDisplay(&displayParams);
//			}

			displayParams.command = CCS_LIGHT_MODE_HSI;
			displayParams.mode = 0;

			flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
		}

		else if ( *(pPkg+1) == CCS_LIGHT_MODE_FIXEDMODE )	//固定模式
		{
//			if (  *(pPkg+2) == 1 ||  *(pPkg+2) == 2 ||  *(pPkg+2) == 3 ||  *(pPkg+2) == 6 )
			{
				//亮度
				if(displayParams.brightness!= *(pPkg+3)){
					displayParams.isBrightnessAdjisting=true;
					displayParams.brightness = *(pPkg+3);
					updateBrightnessDisplay(&displayParams);
				}else{
					displayParams.isBrightnessAdjisting=false;
				}
				switchArrow(&displayParams);
				flag |= CCS_FLAG_MODE;
				uint8 backArrowIndex=displayParams.arrowIndex;
				uint8 backModeValue=displayParams.style1Value;
//				LOG("\narr:%d,  oValue=%d,cValue=%d",backArrowIndex,backModeValue,*(pPkg+2));
				if( Style1Setting==backArrowIndex&&((backModeValue+1)==*(pPkg+2)))
					return true;
//				LOG("\nupdate");
				displayParams.arrowIndex = Style1Setting;
				updateArrowDisplay(&displayParams);
				displayParams.style1Value = *(pPkg+2);
				displayParams.mode = displayParams.style1Value;

//				if ( displayParams.style1Value == 6 )
//				{
//					displayParams.style1Value = 4;
//				}
				//LOG("value=%d",displayParams.style1Value);
				if(displayParams.style1Value)
					displayParams.style1Value-=1;
//				LOG("mode=%d  value=%d",displayParams.mode,displayParams.style1Value);
				updateLightEffectDisplay(&displayParams);

				displayParams.command = CCS_LIGHT_MODE_FIXEDMODE;
				flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
				CCS_DATATRANSFER_PROCESS(flag, &displayParams);
			}
//			else
//			{
//				displayParams.command = CCS_LIGHT_MODE_OFF;
//				uint8 databuf[] = {5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0};
//				light_ctrl(databuf);
//			}
		}

		else if ( *(pPkg+1) == CCS_LIGHT_MODE_EFFECTMODE )	//特效模式
		{
			//亮度
			if(displayParams.brightness!= *(pPkg+3)){
				displayParams.isBrightnessAdjisting=true;
				displayParams.brightness = *(pPkg+3);
				updateBrightnessDisplay(&displayParams);
			}else{
				displayParams.isBrightnessAdjisting=false;
			}
			switchArrow(&displayParams);
					//LOG("\n brightness=%d\n",displayParams.brightness);
			//CCT
//			if ( *(pPkg+4) != 0xff )
			{

				if ( *(pPkg+4) != 0xff )
				{
					flag |= CCS_FLAG_TEMPERATURE;
					displayParams.colorTemperature = *(pPkg+4);
					displayParams.arrowIndex = ColorTempSetting;
					updateArrowDisplay(&displayParams);
					updateColorTempDisplay( &displayParams);
					displayParams.fIsFromRGBMode=FALSE;
					//LOG("\n colorTemp=%d\n",displayParams.colorTemperature);
				}
			}

			//HSI
//			if ( *(pPkg+7) != 0xff )
			{
				if ( *(pPkg+7) != 0xff )
				{
					uint8 tempSaturation=displayParams.saturation;
					uint8 tempHues=displayParams.saturation;

					flag |= CCS_FLAG_SATURATION;
					displayParams.saturation = *(pPkg+7);

					flag |= CCS_FLAG_HUE;
					displayParams.hues = *(pPkg+5);
					displayParams.hues |= *(pPkg+6)<<8;

					if(tempSaturation!=displayParams.saturation)
						displayParams.arrowIndex = SaturationSetting;
					else if(tempHues!=displayParams.hues)
						displayParams.arrowIndex = HuesSetting;
					updateArrowDisplay(&displayParams);
					if(displayParams.arrowIndex ==HuesSetting)
						updateHuesDisplay(&displayParams);
					else if(SaturationSetting==displayParams.arrowIndex)
						updateSaturationDisplay(&displayParams);
					displayParams.fIsFromRGBMode=TRUE;
					//LOG("\n hues=%d",displayParams.hues);
					//LOG("\n saturation=%d",displayParams.saturation);
				}
			}

			flag |= CCS_FLAG_EFFECTSMODE;
			displayParams.effectmode = *(pPkg+8);	//mode
			flag |= CCS_FLAG_TIMES;
			displayParams.times = *(pPkg+9);		//times
			flag |= CCS_FLAG_FREQ;
			displayParams.freq = *(pPkg+10);		//Freq
//			LOG("\neffectmode=%d  times=%d",displayParams.effectmode,displayParams.times);
//			LOG("\ncolorTemperature=%d\n",displayParams.colorTemperature);
//			LOG("\nhues=%d\n",displayParams.hues);
			displayParams.customizeEffectMode=displayParams.effectmode;
			displayParams.customizeEffectTimes=displayParams.times;
			displayParams.customizeEffectFreq=displayParams.freq;
			//displayParams.arrowIndex = CustomizeEffect;

			displayParams.command = CCS_LIGHT_MODE_EFFECTMODE;
			displayParams.mode = 0;
			flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
		}
	}

	else	//LED off
	{
		displayParams.command = CCS_LIGHT_MODE_OFF;
		uint8 databuf[] = {5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0};
		light_ctrl(databuf);
	}

	return true;
}

/*********************************************************************
* @fn      PHD_Effectmode_handle
*
* @brief   APP Effectmode handle
*
* @param   none
*
* @return  none
*/
//static void CCS_Effectmode_handle(void) {
//	uint16 flag = 0;
//
//	if (EffectMode == CCS_EFFECTMODE_FLASH) {
//		if (EffectModeLedStatus == CCS_EFFECTMODE_LEDOFF) {
//			EffectModeLedStatus = CCS_EFFECTMODE_LEDON;
//
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//
//			flag = CCS_FLAG_BRIGHTNESS;
//			updateBrightnessDisplay(&displayParams);
//
//			flag = CCS_FLAG_TEMPERATURE;
//			displayParams.arrowIndex = ColorTempSetting;
//			updateArrowDisplay(&displayParams);
//
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//			flag |= CCS_FLAG_MODE;
//			displayParams.mode = 0;
//			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
//		} else {
//			EffectModeLedStatus = CCS_EFFECTMODE_LEDOFF;
//
//			displayParams.command = CCS_LIGHT_MODE_OFF;
//			uint8 databuf[] = { 5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0 };
//			light_ctrl(databuf);
//		}
//
//		osal_start_timerEx(command_center_TaskID, CCS_LEDEFFECTMODE_EVT, displayParams.freq / 2);
//	} else if (EffectMode == CCS_EFFECTMODE_TRIGGER) {
//		if (EffectModeLedStatus == CCS_EFFECTMODE_LEDOFF) {
//			EffectModeLedStatus = CCS_EFFECTMODE_LEDON;
//
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//
//			flag = CCS_FLAG_BRIGHTNESS;
//			updateBrightnessDisplay(&displayParams);
//
//			flag = CCS_FLAG_TEMPERATURE;
//			displayParams.arrowIndex = ColorTempSetting;
//			updateArrowDisplay(&displayParams);
//
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//			flag |= CCS_FLAG_MODE;
//			displayParams.mode = 0;
//			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
//
//			osal_start_timerEx(command_center_TaskID, CCS_LEDEFFECTMODE_EVT, trigger_ontime_effemode[displayParams.trigger_ontime]);
//		} else {
//			EffectMode = CCS_EFFECTMODE_NORMAL;
//			EffectModeLedStatus = CCS_EFFECTMODE_LEDOFF;
//
//			displayParams.command = CCS_LIGHT_MODE_OFF;
//			uint8 databuf[] = { 5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0 };
//			light_ctrl(databuf);
//		}
//	} else {
//		EffectModeLedStatus = CCS_EFFECTMODE_LEDOFF;
//	}
//}

/*********************************************************************
* @fn      CCS_DATATRANSFER_PROCESS
*
* @brief   Process an package of recive from apps
*
* @param   pMsg - message to process
*
* @return  none
*/
static void CCS_DATATRANSFER_PROCESS( uint16 flag, displayParamsStruct *ParamsStruct )
{
	uint8 index = 0;
	uint8 databuf[20] = {0};
	resetStoreConfigTime();
	//LOG("\n   CCS_data_in____ \n");
	flag |= CCS_FLAG_BRIGHTNESS;
	
	if ( flag & CCS_FLAG_HUE ||
		flag & CCS_FLAG_SATURATION )
	{
		flag |= CCS_FLAG_HUE;
		flag |= CCS_FLAG_SATURATION;
	}
	
	//len
	index++;
	
	//flag
	databuf[index] = HI_UINT16(flag);
	index++;
	databuf[index] = LO_UINT16(flag);
	index++;
	
	if ( flag & CCS_FLAG_COMMAND )
	{
		databuf[index] = ParamsStruct->command;
		index++;
	}
	
	if ( flag & CCS_FLAG_MODE )
	{
		databuf[index] = ParamsStruct->mode;
		index++;
	}
	
	if ( flag & CCS_FLAG_BRIGHTNESS )
	{
		if ( fIsSystemHot )
		{
			if ( flag & CCS_FLAG_ONTIME )
			{
//				databuf[index] = (ParamsStruct->trigger_brightness)*80/100;
				databuf[index] = (ParamsStruct->trigger_brightness)*80/100;
			}
			else
			{
//				databuf[index] = (ParamsStruct->brightness)*80/100;
				databuf[index] = (ParamsStruct->brightness)*80/100;
			}
			index++;
		}
		else
		{
			if ( flag & CCS_FLAG_ONTIME )
			{
//				databuf[index] = ParamsStruct->trigger_brightness;
				databuf[index] = (ParamsStruct->trigger_brightness)*100/100;
			}
			else
			{
//				databuf[index] = ParamsStruct->brightness;
				databuf[index] = (ParamsStruct->brightness)*100/100;
			}
			index++;
		}
	}
	
	if ( flag & CCS_FLAG_TEMPERATURE )
	{
		databuf[index] = ParamsStruct->colorTemperature;
		index++;
	}
	
	if ( flag & CCS_FLAG_HUE )
	{
		databuf[index] = HI_UINT16(ParamsStruct->hues);
		index++;
		databuf[index] = LO_UINT16(ParamsStruct->hues);
		index++;
	}
	
	if ( flag & CCS_FLAG_SATURATION )
	{
		databuf[index] = ParamsStruct->saturation;
		index++;
	}
	
	if ( flag & CCS_FLAG_EFFECTSMODE )
	{
		databuf[index] = ParamsStruct->effectmode;	
		index++;
	}
	
	if ( flag & CCS_FLAG_TIMES )
	{
		databuf[index] = ParamsStruct->times;
		index++;
	}
	
	if ( flag & CCS_FLAG_FREQ )
	{
		databuf[index] = ParamsStruct->freq;
		index++;
	}
	
	if ( flag & CCS_FLAG_ONTIME )
	{
		databuf[index] = ParamsStruct->trigger_ontime;
		index++;
	}
	
	databuf[0] = index;
	light_ctrl(databuf);
	
	//Notify status to APP
	CCS_NotifyStatus();
}

/*********************************************************************
* @fn      PHD_NotifyStatus
*
* @brief   Notify status to APP
*
* @param   none
*
* @return  true/false
*/
static bool CCS_NotifyStatus(void)
{
//	if ( GetNotifyStatus() == true )
	{
		uint8 databuf[20] = {0};
		
		databuf[0] = displayParams.command;
		databuf[1] = displayParams.mode;
		databuf[2] = displayParams.brightness;
		databuf[3] = displayParams.colorTemperature;
		databuf[4] = LO_UINT16(displayParams.hues);;
		databuf[5] = HI_UINT16(displayParams.hues);;
		databuf[6] = displayParams.saturation;
		databuf[7] = displayParams.effectmode;
		databuf[8] = displayParams.times;
		databuf[9] = displayParams.freq;
		if ( SUCCESS == SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, 10, databuf ) )
		{
			return true;
		}
		else
		{
			return false;
		}		
	}
//	else
//	{
//		return false;
//	}
}

/*********************************************************************
 * @fn      command_center_Systems_on
 *
 * @brief   Systems online.
 *
 * @param   none
 *
 * @return  none
 */
void CCS_Systems_on(void)
{
	uint8 batt_pec = batt_get_percent();
	
//	if ( batt_pec < 5 )
//	{
//		return;
//	}
	temp_init();
	CCS_MODE_SYSTEM = true;

	uint8 advertising_enable;
	GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advertising_enable );
	if ( !advertising_enable )
	{
		advertising_enable = true;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertising_enable);
	}
//	userI2cInitial();
	if (!CCS_MODE_DISPLAY) {
		CCS_MODE_DISPLAY = true;
//		display_Init();
		reInitialLCD();
		//lcd_on();
	}
	lcd_clear();
	Flag_bleioc = true;
	updateBLEDisplay(Flag_bleioc);
	osal_start_timerEx( command_center_TaskID, CCS_BLEIOC_EVT, 1000 );
	
	// start batt det
	//batt_measure();
	if(Flag_Status_Charge == Sys_Charging){
		osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S);
//		memset(&fullyParams,0,sizeof(fullyParams));
	}
	else{
		osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, BATT_DET_PERIOD_1S+1000);
		restartBattInfoCollection();
	}
	osal_start_timerEx( command_center_TaskID, CCS_TEMP_CHECK_EVT, TEMPERATURE_DET_PERIOD_HOT_1S );

	// update dispaly
//	displaySystemMenu(&displayParams);
//	updateArrowDisplay(&displayParams);
	
	// update light
	//HSI
	uint16 flag = 0;
	if (HuesSetting == displayParams.arrowIndex)
	{
		flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;
	}
	
	//saturation
	else if (SaturationSetting == displayParams.arrowIndex)
	{
		flag = CCS_FLAG_HUE | CCS_FLAG_SATURATION;
		displayParams.command = CCS_LIGHT_MODE_HSI;
		displayParams.mode = 0;
	}
	
	//colortemp
	else if (ColorTempSetting == displayParams.arrowIndex)
	{
		flag = CCS_FLAG_TEMPERATURE;
		displayParams.command = CCS_LIGHT_MODE_CCT;
		displayParams.mode = 0;
	}
	
	//fixed mode
	else if (Style1Setting == displayParams.arrowIndex )
	{
		flag = CCS_FLAG_EFFECTSMODE;
		displayParams.command = CCS_LIGHT_MODE_FIXEDMODE;
		displayParams.mode = displayParams.style1Value+1;
	}
	else
	{
		
	}
#if	(USER_EVNET_PRINT==1)
	LOG("\n battlv=%d",displayParams.battLv);
#endif

	hal_gpio_write(GPIO_LIGHT_POWER, GPIO_HL_LIGHT_POWER_OFF);
//	hal_gpio_write(GPIO_LIGHT_EN, GPIO_HL_LIGHT_EN_OFF);
	updateArrowDisplay(&displayParams);
	updateDeviceInfo2Flash();
	
	flag |= CCS_FLAG_COMMAND | CCS_FLAG_MODE | CCS_FLAG_BRIGHTNESS;
	CCS_DATATRANSFER_PROCESS(flag, &displayParams);
}

/*********************************************************************
 * @fn      CCS_Systems_off
 *
 * @brief   Systems outline.
 *
 * @param   none
 *
 * @return  none
 */
void CCS_Systems_off(systemOffReasonDef info)
 {
	//storeSystemSetting(&displayParams);
	CCS_MODE_SYSTEM = false;
	Flag_bleioc = false;
	fIsSystemHot = false;
	TESTMODE = NormalMode;
	setTemperature(10.0);
	vTemperatureCoe = TEMPERATURE_COE_NORMAL;
	//RF
//	if ( GetgapProfileState() == GAPROLE_CONNECTED )
//	{
//		GAPRole_TerminateConnection();
//	}
#if	(USER_EVNET_PRINT==1)
	LOG("\n battlv=%d",displayParams.battLv);
#endif
	LOG("\n System Off Info=%d\n",info);

	GAPRole_TerminateConnection();
	osal_start_timerEx(command_center_TaskID, CCS_BLERF_EVT, 5);

	uint8 current_adv_enabled_status;

	//Find the current GAP advertisement status
	GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);

	if (current_adv_enabled_status == TRUE) {
		current_adv_enabled_status = FALSE;
		//change the GAP advertisement status to opposite of current status
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status);
	}
	osal_stop_timerEx(command_center_TaskID, CCS_BLEIOC_EVT);

	displayParams.command = CCS_LIGHT_MODE_OFF;
	uint8 databuf[] = { 5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0 };
	light_ctrl(databuf);
	turnOffAllLightEffect();
	//LCD
	if (Flag_Status_Charge == Non_Charging) {
		//LOG("\n start not chg");
		lcd_off();
		CCS_MODE_DISPLAY = false;
		restartBattInfoCollection();
	} else {
//		if (Flag_Status_Charge >= Sys_Charging) {
			osal_start_timerEx(command_center_TaskID, CCS_BATT_CHECK_EVT, 20);
//		}
		CCS_keysChange_handle();
		lcd_clear();
	}

	osal_stop_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT);
	osal_stop_timerEx(command_center_TaskID, CCS_TEMP_VALUE_EVT);

}

/*********************************************************************
 * @fn      CCS_GET_HOTDISPLAYSTATUS
 *
 * @brief   
 *
 * @param   
 *
 * @return  bool
 */
bool CCS_GET_HOTDISPLAYSTATUS(void)
{
	return Flag_Status_hot_display;
}

/*********************************************************************
 * @fn      CCS_GET_ChargeStatus
 *
 * @brief   
 *
 * @param   
 *
 * @return  bool
 */
bool CCS_GET_ChargeStatus(void)
{
	return Flag_Status_Charge;
}

/*********************************************************************
 * @fn      CCS_GET_BLE_Status
 *
 * @brief   
 *
 * @param   
 *
 * @return  bool
 */
bool CCS_GET_BLE_Status(void)
{
	return Flag_bleioc;
}

/*********************************************************************
* @fn      PHD_EventsPro_TestMode
*
* @brief   Test mode handle.
*
* @param   none
*
* @return  none
*/
static void CCS_EventsPro_TestMode(void) {
	if (TESTMODE == TestModeByUser) {
//		uint16 flag = 0;
		switch (Test_Item) {
		default:
		case TESTITEM_NONE:
			entryFactoryMode();
			break;

		case TESTITEM_L_R_ON: {
			pwmInit();
			setRedData(PWM_PERIOD_CONST * TEST_MODE_RATE);

			Test_Item = TESTITEM_L_R_OFF;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
		}
			break;

		case TESTITEM_L_R_OFF: {
			setRedData(0);
			Test_Item = TESTITEM_L_G_ON;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 10);
		}
			break;

		case TESTITEM_L_G_ON: {
			setGreenData(PWM_PERIOD_CONST * TEST_MODE_RATE);

			Test_Item = TESTITEM_L_G_OFF;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
		}
			break;

		case TESTITEM_L_G_OFF: {
			setGreenData(0);
			Test_Item = TESTITEM_L_B_ON;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 10);
		}
			break;

		case TESTITEM_L_B_ON: {
			setBlueData(PWM_PERIOD_CONST * TEST_MODE_RATE);

			Test_Item = TESTITEM_L_B_OFF;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
		}
			break;

		case TESTITEM_L_B_OFF: {
			setBlueData(0);
			Test_Item = TESTITEM_L_C_ON;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 10);
		}
			break;

		case TESTITEM_L_C_ON: {
			setCoolData(PWM_PERIOD_CONST * TEST_MODE_RATE);

			Test_Item = TESTITEM_L_C_OFF;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
		}
			break;

		case TESTITEM_L_C_OFF: {
			setCoolData(0);

			Test_Item = TESTITEM_L_W_ON;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 10);
		}
			break;

		case TESTITEM_L_W_ON: {
			setWarmData(PWM_PERIOD_CONST * TEST_MODE_RATE);

			Test_Item = TESTITEM_L_W_OFF;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
		}
			break;

		case TESTITEM_L_W_OFF: {
			setWarmData(0);
		}
			break;

		case TESTITEM_I_R: {
			factoryModeRGBOn(0,100);

			Test_Item = TESTITEM_I_G;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_G: {
			factoryModeRGBOn(120,100);

			Test_Item = TESTITEM_I_B;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_B: {
			factoryModeRGBOn(240,100);

			Test_Item = TESTITEM_I_C;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_C: {
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = 1;
//
//			flag |= CCS_FLAG_BRIGHTNESS;
//			displayParams.brightness = 100;
//
//			flag |= CCS_FLAG_TEMPERATURE;
//			displayParams.colorTemperature = MAX_ColorTemp;
//
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//			flag |= CCS_FLAG_MODE;
//			displayParams.mode = 0;
//			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
			factoryModeColorTempOn(MAX_ColorTemp,100);
			Test_Item = TESTITEM_I_W;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_W: {
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = 1;
//
//			flag |= CCS_FLAG_BRIGHTNESS;
//			displayParams.brightness = 100;
//
//			flag |= CCS_FLAG_TEMPERATURE;
//			displayParams.colorTemperature = MIN_ColorTemp;
//
//			flag |= CCS_FLAG_COMMAND;
//			displayParams.command = CCS_LIGHT_MODE_CCT;
//			flag |= CCS_FLAG_MODE;
//			displayParams.mode = 0;
//			CCS_DATATRANSFER_PROCESS(flag, &displayParams);
			factoryModeColorTempOn(MAX_ColorTemp,100);

			Test_Item = TESTITEM_I_STAB;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_STAB: {
			uint8 databuf[] = { 5, 0x00, CCS_FLAG_COMMAND | CCS_FLAG_MODE, CCS_LIGHT_MODE_OFF, 0 };
			light_ctrl(databuf);

			Test_Item = TESTITEM_I_SLEEP;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
		}
			break;

		case TESTITEM_I_SLEEP: {
			osal_stop_timerEx(command_center_TaskID, STORE_CURRENT_SETTING);
			loadDefaultSetting(&displayParams);
			storeSystemSetting(&displayParams);
			//CCS_Systems_off();
			HW_RESET_MCU(false);
		}
			break;
		}
	}else if(TESTMODE==BurnInTest){
		switch (Test_Item) {
		default:
		case TESTITEM_NONE:
			entryBurnInTest();
			Test_Item=TESTITEM_L_R_ON;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 2000);
			break;
		case TESTITEM_L_R_ON:
			pwmInit();
//			setRedData(PWM_PERIOD_CONST * TEST_MODE_RATE);
			factoryModeRGBOn(0,100);
			Test_Item = TESTITEM_L_R_OFF;
			vBurnInTestTimer=defaultTestTimerConst;
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			break;
		case TESTITEM_L_R_OFF:
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			if(vBurnInTestTimer&&!--vBurnInTestTimer){
//				setRedData(0);
				Test_Item = TESTITEM_L_G_ON;
			}else{
				displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			}
			break;
		case TESTITEM_L_G_ON:
			OLED_ShowString(20,2,"G:");
			factoryModeRGBOn(120,100);
			Test_Item = TESTITEM_L_G_OFF;
			vBurnInTestTimer=defaultTestTimerConst;
			displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			break;
		case TESTITEM_L_G_OFF:
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			if(vBurnInTestTimer&&!--vBurnInTestTimer){
//				setGreenData(0);
				Test_Item = TESTITEM_L_B_ON;
			}else{
				displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			}
			break;
		case TESTITEM_L_B_ON:
			OLED_ShowString(20,2,"B:");
			factoryModeRGBOn(240,100);
			Test_Item = TESTITEM_L_B_OFF;
			vBurnInTestTimer=defaultTestTimerConst;
			displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			break;
		case TESTITEM_L_B_OFF:
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			if(vBurnInTestTimer&&!--vBurnInTestTimer){
//				setBlueData(0);
				Test_Item = TESTITEM_L_C_ON;
			}else{
				displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			}
			break;
		case TESTITEM_L_C_ON:
			OLED_ShowString(20,2,"C:");
			factoryModeColorTempOn(MAX_ColorTemp,100);
			Test_Item = TESTITEM_L_C_OFF;
			vBurnInTestTimer=defaultTestTimerConst;
			displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			break;
		case TESTITEM_L_C_OFF:
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			if(vBurnInTestTimer&&!--vBurnInTestTimer){
//				setBlueData(0);
				Test_Item = TESTITEM_L_W_ON;
			}else{
				displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			}
			break;
		case TESTITEM_L_W_ON:
			OLED_ShowString(20,2,"W:");
			factoryModeColorTempOn(MIN_ColorTemp,100);
			Test_Item = TESTITEM_L_W_OFF;
			vBurnInTestTimer=defaultTestTimerConst;
			displayFloat(20+16,2,vBurnInTestTimer,0,' ');
			osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			break;
		case TESTITEM_L_W_OFF:
			if(vBurnInTestTimer&&!--vBurnInTestTimer){
//				setBlueData(0);
				HW_RESET_MCU(false);
			}else{
				displayFloat(20+16,2,vBurnInTestTimer,0,' ');
				osal_start_timerEx(command_center_TaskID, CCS_TESTMODE_EVT, 1000);
			}
			break;
		}
	}
}

bool getSystemStts(void) {
	return CCS_MODE_SYSTEM;
}
void updateSystemStts(bool stts){
	 CCS_MODE_SYSTEM=stts;
}
/**********************************************************************
 *
 * backup :   	true/false      backup current status true/false
 *
 ***********************************************************************/
void HW_RESET_MCU(bool backup) {
	if (backup)
		storeExceptionStts();
	hal_gpio_write(SW_RESET_MCU, 0);
	while (1);
}
/***********************************************************************************************************
  *  @brief  			display FW version,software version and hardware version
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void versionDisplay(void) {
	uint8 fw[] = { "FW:21102701" };
	systems_param_Get_param(ITEM_FIRMWARE_REV, &fw[2], &fw[3]);
	fw[2] = ':';
	OLED_ShowString(2, 2, fw);

	uint8 sw[] = {"SW:20211027"};
//	systems_param_Get_param(ITEM_SOFTWARE_REV, &sw[2], &fw[3]);
//	sw[2] = ':';
	OLED_ShowString(2, 4, sw);

	uint8 hw[] = { "HW:V17" };
//	systems_param_Get_param(ITEM_HARDWARE_REV, &hw[2], &fw[3]);
//	hw[2] = ':';
	OLED_ShowString(2, 6, hw);
	OLED_ShowNum(80, 0, TESTMODE);
	OLED_ShowNum(100, 2, Test_Key_Fun);
	OLED_ShowNum(100, 4, Test_Key_Light);
}
/***********************************************************************************************************
  *  @brief
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void entryFactoryMode(void) {
	uint8 current_adv_enabled_status;

	//Find the current GAP advertisement status
	GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);

	if (current_adv_enabled_status == false) {
		current_adv_enabled_status = true;
		//change the GAP advertisement status to opposite of current status
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status);
	}

	CCS_MODE_SYSTEM = true;

	uint8 advertising_enable;
	GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advertising_enable);

	if (!advertising_enable) {
		advertising_enable = true;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertising_enable);
	}

	CCS_MODE_SYSTEM = true;
	Flag_Status_Charge = Non_Charging;
	turnOffAllLightEffect();
	Test_Key_Fun=30;
	Test_Key_Light=30;
	lcd_on();
	CCS_MODE_DISPLAY = true;
	hal_gpio_pin_init(ADC_TEMP_POWER, OEN);
	hal_gpio_write(ADC_TEMP_POWER, 1);
	osal_start_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT, 1000);
	osal_start_timerEx(command_center_TaskID, CCS_GETTEMP_EVT, 100);

	batt_measure();
	if(fIsInvalidMacAddr){
		TESTMODE = NormalMode;
		updateArrowDisplay(&displayParams);
		return;
	}
	versionDisplay();

	OLED_ShowNum(100, 2, Test_Key_Fun);
	OLED_ShowNum(100, 4, Test_Key_Light);
}

void	 entryBurnInTest(void){
	CCS_MODE_SYSTEM = true;
	Flag_Status_Charge = Non_Charging;

	lcd_on();
	CCS_MODE_DISPLAY = true;
	hal_gpio_pin_init(ADC_TEMP_POWER, OEN);
	hal_gpio_write(ADC_TEMP_POWER, 1);
	osal_start_timerEx(command_center_TaskID, CCS_TEMP_CHECK_EVT, 1000);
	osal_start_timerEx(command_center_TaskID, CCS_GETTEMP_EVT, 100);
	OLED_ShowString(20,2,"R:");
}
/***********************************************************************************************************
  *  @brief
  *
  *
  *  @param [in] :
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :
  ************************************************************************************************************/
void	 factoryModeRGBOn(uint16 hues,uint8 brightness){
	uint8 flag=0;
	flag |= CCS_FLAG_COMMAND;
	displayParams.command = 1;

	flag |= CCS_FLAG_BRIGHTNESS;
	displayParams.brightness = brightness;

	flag |= CCS_FLAG_HUE;
	displayParams.hues = hues;

	flag |= CCS_FLAG_SATURATION;
	displayParams.saturation = 100;

	flag |= CCS_FLAG_COMMAND;
	displayParams.command = CCS_LIGHT_MODE_HSI;
	flag |= CCS_FLAG_MODE;
	displayParams.mode = 0;
	CCS_DATATRANSFER_PROCESS(flag, &displayParams);
}

void	 factoryModeColorTempOn(uint8 cTemp,uint8 brightness){
	uint8 flag=0;
	flag |= CCS_FLAG_COMMAND;
	displayParams.command = 1;

	flag |= CCS_FLAG_BRIGHTNESS;
	displayParams.brightness = brightness;

	flag |= CCS_FLAG_TEMPERATURE;
	displayParams.colorTemperature = cTemp;

	flag |= CCS_FLAG_COMMAND;
	displayParams.command = CCS_LIGHT_MODE_CCT;
	flag |= CCS_FLAG_MODE;
	displayParams.mode = 0;
	CCS_DATATRANSFER_PROCESS(flag, &displayParams);
}
/*********************************************************************
 * @fn      protocol_uartCB_Handle
 *
 * @brief   Handle uart callback evt.
 *
 * @param
 *
 * @return  bool
 */
bool protocol_uartCB_Handle(uint16 len, uint8 *pPkg)
{
	if ( *pPkg == UART_SOP )
	{
		if ( *(pPkg+1) == CMD_HI_a0 && *(pPkg+2) == CMD_LO_TESTMODE_INTO)
		{
			if ( *(pPkg+4) == OPTION_PD_TEST_STANDBY )
			{
				if(!fIsInvalidMacAddr){
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_STANDBY };
					hal_uart_send_buff(data,sizeof(data));
				}
				TESTMODE = TestModeByAte;
//				Test_Key_Power_pass = false;
				Test_Key_Light_up_pass = false;
				Test_Key_Light_down_pass = false;
				Test_Key_Fun_pass = false;
				Test_Key_Fun_up_pass = false;
				Test_Key_Fun_down_pass = false;

				color.red = 0;
				color.green = 0;
				color.blue = 0;
				color.cw = 0;
				color.mw = 0;
				color.brightness = 0;
				light_update();

				entryFactoryMode();
				return true;
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_BATT )
			{
				uint16 batt_vol = (uint16)batt_get_voltage();
				uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 4, CMD_LO_TESTMODE_INTO, OPTION_PD_TEST_BATT, HI_UINT16(batt_vol), LO_UINT16(batt_vol) };
				hal_uart_send_buff(data,sizeof(data));
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_TEMP )
			{
				uint16 temp = temp_get_value();
				uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 4, CMD_LO_TESTMODE_INTO, OPTION_PD_TEST_TEMP, HI_UINT16(temp), LO_UINT16(temp) };
				hal_uart_send_buff(data,sizeof(data));
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_MAC )
			{
				uint8 macAddr[6]={0};
				getSystemId(macAddr);
				uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 4, CMD_LO_TESTMODE_INTO, OPTION_PD_TEST_MAC, macAddr[0], macAddr[1],macAddr[2],macAddr[3],macAddr[4],macAddr[5] };
				hal_uart_send_buff(data,sizeof(data));
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_LED_R )
			{
				if ( Test_Key_Light_up_pass && Test_Key_Light_down_pass && Test_Key_Fun_pass && Test_Key_Fun_up_pass && Test_Key_Fun_down_pass )
				{
					OLED_ShowNum(80, 0, 5);
					updateHotDisplay(false);

					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_LED_R };
					hal_uart_send_buff(data,sizeof(data));
					factoryModeRGBOn(0,100);

				}
				else
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_OTH };
					hal_uart_send_buff(data,sizeof(data));
					return true;
				}
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_LED_G )
			{
				if ( Test_Key_Light_up_pass && Test_Key_Light_down_pass && Test_Key_Fun_pass && Test_Key_Fun_up_pass && Test_Key_Fun_down_pass )
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_LED_G };
					hal_uart_send_buff(data,sizeof(data));

					factoryModeRGBOn(120,100);
				}
				else
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_OTH };
					hal_uart_send_buff(data,sizeof(data));
					return true;
				}
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_LED_B )
			{
				if ( Test_Key_Light_up_pass && Test_Key_Light_down_pass && Test_Key_Fun_pass && Test_Key_Fun_up_pass && Test_Key_Fun_down_pass )
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_LED_B };
					hal_uart_send_buff(data,sizeof(data));

					factoryModeRGBOn(240,100);
				}
				else
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_OTH };
					hal_uart_send_buff(data,sizeof(data));
					return true;
				}
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_LED_C )
			{
				if ( Test_Key_Light_up_pass && Test_Key_Light_down_pass && Test_Key_Fun_pass && Test_Key_Fun_up_pass && Test_Key_Fun_down_pass )
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_LED_C };
					hal_uart_send_buff(data,sizeof(data));

					factoryModeColorTempOn(MAX_ColorTemp,100);
				}
				else
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_OTH };
					hal_uart_send_buff(data,sizeof(data));
					return true;
				}
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_LED_M )
			{
				if ( Test_Key_Light_up_pass && Test_Key_Light_down_pass && Test_Key_Fun_pass && Test_Key_Fun_up_pass && Test_Key_Fun_down_pass )
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_LED_M };
					hal_uart_send_buff(data,sizeof(data));
					factoryModeColorTempOn(MIN_ColorTemp,100);
				}
				else
				{
					uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_OTH };
					hal_uart_send_buff(data,sizeof(data));
					return true;
				}
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_STANDBY )
			{
				uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_STANDBY };
				hal_uart_send_buff(data,sizeof(data));
				color.red = 0;
				color.green = 0;
				color.blue = 0;
				color.cw = 0;
				color.mw = 0;
				color.brightness = 0;
				light_update();
			}
			else if ( *(pPkg+4) == OPTION_PD_TEST_CURRENT_SLEEP )
			{
				uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 2, CMD_LO_TESTMODE_INTO,OPTION_PD_TEST_CURRENT_SLEEP };
				hal_uart_send_buff(data,sizeof(data));
				osal_stop_timerEx(command_center_TaskID, STORE_CURRENT_SETTING);
				loadDefaultSetting(&displayParams);
				storeSystemSetting(&displayParams);
				CCS_Systems_off(Off_By_Uart);
			}
			else
			{

			}
		}
		else if ( *(pPkg+1) == CMD_HI_a0 && *(pPkg+2) == CMD_LO_SLEEPMODE )
		{
			uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_ACK, 1, CMD_LO_SLEEPMODE };
			hal_uart_send_buff(data,sizeof(data));
			osal_stop_timerEx(command_center_TaskID, STORE_CURRENT_SETTING);
			loadDefaultSetting(&displayParams);
			storeSystemSetting(&displayParams);
			CCS_Systems_off(Off_By_Uart);
		}
		else
		{
			uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_CMD };
			hal_uart_send_buff(data,sizeof(data));
			return true;
		}
	}
	else
	{
		uint8 data[] = {UART_SOP, CMD_HI_80, CMD_LO_NACK, 1, SAPP_PKG_ERR_SOP };
		hal_uart_send_buff(data,sizeof(data));
		return true;
	}
	return false;

}
void userIOInit(void){
	hal_gpio_pin_init(GPIO_KEY_POWER, IE);
	hal_gpio_pull_set(GPIO_KEY_POWER, WEAK_PULL_UP);
}


float getCompensationVolt(void){
	return vBattCompensation;
}
/*************************** (C) COPYRIGHT 2021 Bough*****END OF FILE*****************************/
