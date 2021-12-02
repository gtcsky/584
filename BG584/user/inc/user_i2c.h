/*
 * user_i2c.h
 *
 *  Created on: 2020年11月28日
 *      Author: Sky
 */

#ifndef BG584_USER_INC_USER_I2C_H_
#define BG584_USER_INC_USER_I2C_H_
#include "i2c.h"
#include "protocol.h"

#define EEPROM_SIZE_2K		2
#define EEPROM_SIZE_4K		4
#define EEPROM_SIZE_8K		8
#define EEPROM_SIZE_16K	16
#define EEPROM_SIZE_32K	32
#define EEPROM_SIZE_64K	64
#define EEPROM_SIZE_128K	128
#define EEPROM_SIZE_256K	256
#define EEPROM_SIZE_512K	512


#define EEPROM_SIZE EEPROM_SIZE_2K

#if(EEPROM_SIZE == EEPROM_SIZE_2K)
	#define	EEPROM_PAGE_SIZE				8	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			32
#elif(EEPROM_SIZE == EEPROM_SIZE_4K)
	#define	EEPROM_PAGE_SIZE				16	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			32
#elif(EEPROM_SIZE == EEPROM_SIZE_8K)
	#define	EEPROM_PAGE_SIZE				16	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			64
#elif(EEPROM_SIZE == EEPROM_SIZE_16K)
	#define	EEPROM_PAGE_SIZE				16	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			128
#elif(EEPROM_SIZE == EEPROM_SIZE_32K)
	#define	EEPROM_PAGE_SIZE				32	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			128
#elif(EEPROM_SIZE == EEPROM_SIZE_64K)
	#define	EEPROM_PAGE_SIZE				32	//(Bytes)
	#define	EEPROM_TOTAL_PAGES			256
#endif


#define Inactive_Eeprom                  (150) 		/*crypto verify error*/

int userEepromWrite(void* pi2c,uint8_t slave_addr,uint8_t reg,uint8_t* data,uint8_t size);
int userEepromRead( void* pi2c, uint8_t slave_addr, uint8_t reg, uint8_t* data, uint8_t size);
void userI2cInitial(void);
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
  *  @note :	if EEPROM is  Invalid ,the system info will store to internal FLASH
  ************************************************************************************************************/
int storeSystemSetting(displayParamsStruct *sysParams);
/***********************************************************************************************************
  *  @brief  		load  system status information from EEPROM
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
void loadSystemSetting(displayParamsStruct *sysParams) ;
void loadDefaultSetting(displayParamsStruct *sysParams);
void writeInfo(uint8_t* data, uint8_t size) ;
void readInfo(uint8_t* data, uint8_t size) ;
#endif /* BG584_USER_INC_USER_I2C_H_ */
