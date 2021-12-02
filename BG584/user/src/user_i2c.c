/*
 * user_i2c.c
 *
 *  Created on: 2020年11月28日
 *      Author: Sky
 */

#include "types.h"
#include "hal_mcu.h"
#include "clock.h"
#include "log.h"
#include "error.h"
#include "OSAL.h"
#include "pwrmgr.h"
#include "user_i2c.h"
#include "command_center.h"
#include	"user_flash.h"

static void* eepromi2c;

#define I2C_OP_TIMEOUT  100   //100ms for an Byte operation
/***********************************************************************************************************
  *  @brief  		use I2C write one byte to target
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
static void _hal_i2c_send_byte(void* pi2c, uint8_t data)
{
	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)pi2c;
	pi2cdev->IC_DATA_CMD = data;  //data
}
/***********************************************************************************************************
  *  @brief			write  data to EEPROM
  *
  *
  *  @param [in] :	slave_addr      DevAddress Target device address
  *  @param [in] :	reg      		write start address of the EEPROM
  *  @param [in] :	data      		data Pointer to data buffer
  *  @param [in] :	size      		Size Amount of data to be sent
  *
  *  @param [out] :			None
  *
  *  @return :		read success or fail
  *
  *  @note :	size<=page size;
  ************************************************************************************************************/
int _hal_i2c_write_s(void* pi2c, uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t size) {
	I2C_INIT_TOUT(to);
	int ret = PPlus_SUCCESS;
	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *) pi2c;

	if (pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1) {
		return PPlus_ERR_INVALID_PARAM;
	}
	ret = hal_i2c_addr_update(pi2c, slave_addr);
	if (ret)
		return ret;
	HAL_ENTER_CRITICAL_SECTION();
	ret = hal_i2c_tx_start(pi2c);
	if (!ret) {
		 _hal_i2c_send_byte(pi2c, reg);
		 ret=hal_i2c_send(pi2c, data, size);
	}
	HAL_EXIT_CRITICAL_SECTION();
	if (ret)
		return ret;
	return hal_i2c_wait_tx_completed(pi2c);
}
/***********************************************************************************************************
  *  @brief			write  data to EEPROM
  *
  *
  *  @param [in] :	slave_addr      DevAddress Target device address
  *  @param [in] :	reg      		write start address of the EEPROM
  *  @param [in] :	data      		data Pointer to data buffer
  *  @param [in] :	size      		Size Amount of data to be sent
  *
  *  @param [out] :			None
  *
  *  @return :		read success or fail
  *
  *  @note :
  ************************************************************************************************************/
int userEepromWrite(void* pi2c, uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t size) {
	uint8_t cnt;
	int ret = PPlus_SUCCESS;
	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *) pi2c;

	if (pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1) {
		return PPlus_ERR_INVALID_PARAM;
	}
	while (size) {
		cnt = (size > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : size;
		size -= cnt;
		uint8 retryTimes = 6;
		do {
			ret = _hal_i2c_write_s(pi2c, slave_addr, reg, data, cnt);
			if (ret) {
				if (--retryTimes) {
					int tick = hal_systick();
					while (hal_ms_intv(tick) <3){
					}

				} else {
					break;
				}
			}
		} while (ret);

		if (ret != PPlus_SUCCESS) {
			return ret;
		}
		data += cnt;
		if(size){
			reg+=cnt;
			if(reg>=(EEPROM_PAGE_SIZE*EEPROM_TOTAL_PAGES)){
				return	PPlus_ERR_DATA_ALIGN;
			}
			int tick = hal_systick();
			while (hal_ms_intv(tick) <3);
		}
	}
	return ret;
}

/***********************************************************************************************************
  *  @brief			read data form EEPROM address
  *
  *
  *  @param [in] :	slave_addr      DevAddress Target device address
  *  @param [in] :	reg      		Read start address
  *  @param [in] :	data      		data Pointer to data buffer
  *  @param [in] :	size      		Size Amount of data to be read
  *
  *  @param [out] :			data      		data Pointer to data buffer
  *
  *  @return :		read success or fail
  *
  *  @note :
  ************************************************************************************************************/
int userEepromRead(void* pi2c, uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t size) {
	uint8_t cnt;
	int ret = PPlus_SUCCESS;
	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *) pi2c;

	if (pi2cdev != AP_I2C0 && pi2cdev != AP_I2C1) {
		return PPlus_ERR_INVALID_PARAM;
	}

	while (size) {
		cnt = (size > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : size;
		size -= cnt;
		uint8 retryTimes = 6;
		do {
			ret = _hal_i2c_read_s(pi2c, slave_addr, reg, data, cnt);
			if (ret) {
				if (--retryTimes) {
					int tick = hal_systick();
					while (hal_ms_intv(tick) < 3) {
					}

				} else {
					break;
				}
			}
		} while (ret);
		if (ret != PPlus_SUCCESS) {
			return ret;
		}
		data += cnt;
		if (size) {
			reg += cnt;
			if (reg >= (EEPROM_PAGE_SIZE * EEPROM_TOTAL_PAGES)) {
				return PPlus_ERR_DATA_ALIGN;
			}
		}
	}
	return ret;
}


void userI2cInitial(void){
	if(!vSystemEepromError){
		hal_i2c_pin_init(I2C_0, GPIO_IIC_DATA, GPIO_IIC_CLK);//hal_i2c_init();
		eepromi2c = hal_i2c_init(I2C_0, I2C_CLOCK_400K);
	}else{
//		hal_gpio_pin_init(GPIO_KEY_POWER, OEN);
//		hal_gpio_write(GPIO_IIC_DATA,1);
//		hal_gpio_write(GPIO_IIC_CLK,1);
	}
}
/***********************************************************************************************************
  *  @brief		store  system status information into EEPROM
  *
  *
  *  @param [in] :  sysParams pointer to an displayParamsStruct structure that
  *         			contains the system  status information .
  *
  *  @param [out] :
  *
  *  @return :
  *
  *  @note :	if EEPROM is  Invalid ,the system info will be stored into internal FLASH
  ************************************************************************************************************/
int storeSystemSetting(displayParamsStruct *sysParams) {
	int ret = 0;
	uint8 retryTimes = 3;
	if (vSystemEepromError) {
		LOG("\n vSystemEepromError");
		storeExceptionStts();
		return Inactive_Eeprom;
	}
	do {
		ret = userEepromWrite(eepromi2c, IIC_CLK_EEPROMADDR, IIC_STARTADDR_EEPROM, (uint8 *) sysParams, sizeof(displayParamsStruct));
		if (--retryTimes && ret) {
			vSystemEepromError = true;
			storeExceptionStts();
			break;
		}
	} while (ret);
	return ret;
}
/***********************************************************************************************************
  *  @brief	load all default setting
  *
  *
  *  @param [in] :	 sysParams pointer to an displayParamsStruct structure that
  *         			contains the system  status information .
  *
  *  @param [out] :		the updated sysParams
  *
  *  @return :			None
  *
  *  @note :
  ************************************************************************************************************/
void loadDefaultSetting(displayParamsStruct *sysParams) {

	sysParams->command = CCS_LIGHT_MODE_HSI;
	sysParams->mode = 0;
	sysParams->hues = DEFAULT_HUES;
	sysParams->DisplayModeIndex = IdleIamgeDisplay;
	sysParams->brightness = DEFAULT_BRIGHTNESS;
	sysParams->saturation = DEFAULT_SATURATION;
	sysParams->arrowIndex = DEFAULT_ARROR_INDEX;
	sysParams->colorTemperature = DEFAULT_COLOR_TEMP;
	sysParams->style1Value = DEFAULT_STYLE1_VALUE;
	sysParams->battLv = Max_Batt_level;
	sysParams->newEeprom = NOT_NEW_EEPROM;
	sysParams->effectmode = 1;						//	自定义特效模式  1/2:呼吸灯/爆闪
	sysParams->times = 0xff;
	sysParams->freq = 1;
	sysParams->preinstallEffectNo = 1;					//	预设特效编号
	sysParams->customizeEffectMode = 1;				//	自定义特效模式  1/2:呼吸灯/爆闪
	sysParams->customizeEffectTimes = 0xFF;			//	自定义特效循环次数 1~99:次数   100:无限循环
	sysParams->customizeEffectFreq = 1;				//	自定义特效频率
	sysParams->customizeOneShot = 0;					//	自定义特效单次闪烁
	sysParams->fIsEffectMode = 0;						//	自定义特效模式+预设特效模式=1.普通模式=0
	sysParams->fIsFromRGBMode = 0;					//	0/1 :从RGB/色温模式进入自定义特效模式
	sysParams->isBrightnessAdjisting = 0;				//	0/1 :从RGB/色温模式进入自定义特效模式

}
/***********************************************************************************************************
  *  @brief  		load  system status information from EEPROM
  *
  *  @param [in] :	 sysParams pointer to an displayParamsStruct structure that
  *         				contains the system  status information .
  *
  *  @param [out] :		the updated sysParams
  *
  *  @return :			None
  *
  *  @note :
  ************************************************************************************************************/
void loadSystemSetting(displayParamsStruct *sysParams) {
	uint8 retryTimes = 3;
	int ret = 0;
	do {
		ret = userEepromRead(eepromi2c, IIC_CLK_EEPROMADDR, IIC_STARTADDR_EEPROM, (uint8 *) sysParams, sizeof(displayParamsStruct));
		if (--retryTimes&&ret) {
			vSystemEepromError = true;
			break;
		}
	} while (ret);
	if (!vSystemEepromError) {
		if (sysParams->newEeprom != NOT_NEW_EEPROM) {
			loadDefaultSetting(sysParams);
			storeSystemSetting(sysParams);
		}
	}else{
			loadDefaultSetting(sysParams);
//			userI2cInitial();
	}
}

//
//void writeInfo(uint8_t* data, uint8_t size) {
//	userEepromWrite(eepromi2c, IIC_CLK_EEPROMADDR, IIC_STARTADDR_EEPROM, data, size);
//}
//void readInfo(uint8_t* data, uint8_t size) {
//	userEepromRead(eepromi2c, IIC_CLK_EEPROMADDR, IIC_STARTADDR_EEPROM, data, size);
//}

