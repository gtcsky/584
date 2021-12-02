/**************************************************************************************************
Filename:       display.c
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
#include "types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "common.h"

/* Application */
#include "protocol.h"
#include "display.h"
#include "user_character.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "spi.h"
#include "command_center.h"
#include "systems_parameters.h"
#include "log.h"

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
static hal_spi_t lcd_spi_handle = {SPI0};

static bool fIsChargeDisplaying = false;
static bool fIsHotDisplayOn = false;
displayParamsStruct displayParams;

/*********************************************************************
* FUNCTIONS
*/
static int lcd_bus_init(void);
//static int lcd_bus_deinit(void);
static void write_cmd(uint8_t cmd);
static void write_data(uint8_t Data);
static void lcd_config(void);
static void OLED_Set_Pos(unsigned char x, unsigned char y);
static void OLED_ShowChar(uint8 x,uint8 y,uint8 chr);
//static void OLED_ShowOneSegment(uint8 x,uint8 y,uint8  data1,uint8 data2);
static uint32 oled_pow(uint8 m,uint8 n);
static uint8 getNumLen(uint32 num);
static uint8 OLED_ShowChineseNum(uint8 x, uint8 y, uint32 num);
static void OLED_ShowCHinese(uint8 x,uint8 y,uint8 no);
//static void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
//static void HexDigitDis(uint8 x,uint8 y,uint8 value);
static void oledDisplayChineseString(uint8 x,uint8 y,const uint8 *str,uint8 len);

/*********************************************************************
 * @fn      lcd_bus_init
 *
 * @brief   lcd bus(SPI) initial
 *
 * @param   data
 *
 * @return  none
 */
static int lcd_bus_init(void)
{
	int ret;
	spi_Cfg_t cfg =
	{
      .sclk_pin = GPIO_DISPLAY_CLK,
      .ssn_pin = GPIO_DISPLAY_CS,
      .MOSI = GPIO_DISPLAY_MOSI,
      .MISO = GPIO_DISPLAY_MISO,
      .baudrate = 2400000,
      .spi_tmod = SPI_TXD,//SPI_TRXD,
      .force_cs = false,

      .int_mode = true,
    };
  
    ret = hal_spi_bus_init(&lcd_spi_handle,cfg);//spi init
	hal_gpio_DS_control(GPIO_DISPLAY_RST,Bit_ENABLE);
    hal_gpio_DS_control(cfg.sclk_pin,Bit_ENABLE);
    hal_gpio_DS_control(cfg.MOSI,Bit_ENABLE);
    //pad_ds_control(cfg.sclk_pin,Bit_ENABLE);
    //pad_ds_control(cfg.ssn_pin,Bit_ENABLE);
    //pad_ds_control(cfg.MOSI,Bit_ENABLE);
    //pad_ds_control(cfg.MISO,Bit_ENABLE);
    return ret;
}
void lcdBusDeinit(void){
	hal_spi_bus_deinit(&lcd_spi_handle);
}
/*********************************************************************
 * @fn      lcd_bus_deinit
 *
 * @brief   lcd bus(SPI) deinitial
 *
 * @param   data
 *
 * @return  none
 */
//static int lcd_bus_deinit(void)
//{
//    return hal_spi_bus_deinit(&lcd_spi_handle);
//}

/*********************************************************************
 * @fn      write_cmd
 *
 * @brief   write cmd to lcd
 *
 * @param   data
 *
 * @return  none
 */
static void  write_cmd(uint8 cmd)
{
	hal_gpio_write(GPIO_DISPLAY_DC,0);
	hal_gpio_write(GPIO_DISPLAY_CS,0);
	hal_spi_send_byte(&lcd_spi_handle,cmd);
	hal_gpio_write(GPIO_DISPLAY_CS,1);
	hal_gpio_write(GPIO_DISPLAY_DC,1);
//	hal_spi_TxComplete(&lcd_spi_handle);
}

/*********************************************************************
 * @fn      write_data
 *
 * @brief   write uint8 data to lcd
 *
 * @param   data
 *
 * @return  none
 */
static void  write_data(uint8_t Data)
{
	hal_gpio_write(GPIO_DISPLAY_DC,1);
	hal_gpio_write(GPIO_DISPLAY_CS,0);
	hal_spi_send_byte(&lcd_spi_handle,Data);
	hal_gpio_write(GPIO_DISPLAY_CS,1);
	hal_gpio_write(GPIO_DISPLAY_DC,1);
}

/*********************************************************************
 * @fn      lcd_config
 *
 * @brief   lcd initial
 *
 * @param   data
 *
 * @return  none
 */
static void lcd_config(void)
{
	hal_gpio_write(GPIO_DISPLAY_RST,0);
	WaitUs(15); // 15 us
	hal_gpio_write(GPIO_DISPLAY_RST,1);
	WaitUs(15); // 15 us
	hal_gpio_write(GPIO_DISPLAY_RST,0);
	WaitUs(15); // 15 us
	hal_gpio_write(GPIO_DISPLAY_RST,1);
	WaitUs(15); // 15 us
	write_cmd(0xAE); /*display off*/
	
	write_cmd(0x00); /*set lower column address*/
	write_cmd(0x10); /*set higher column address*/
	write_cmd(0x40); /*set display start line*/
	
	write_cmd(0xB0); /*set page address*/
	write_cmd(0x81); /*contract control*/
	write_cmd(0xcf); /*128*/
	
	write_cmd(0xA1); /*set segment remap*/
	
	write_cmd(0xA6); /*normal / reverse*/
	
	write_cmd(0xA8); /*multiplex ratio*/
	write_cmd(0x3F); /*duty = 1/64*/
	
	write_cmd(0xC8); /*Com scan direction*/
	
	write_cmd(0xD3); /*set display offset*/
	write_cmd(0x00);
	write_cmd(0xD5); /*set osc division*/
	write_cmd(0x80);
	write_cmd(0xD9); /*set pre-charge period*/
	write_cmd(0x22);
	
	write_cmd(0xDA); /*set COM pins*/
	write_cmd(0x12);
	
	write_cmd(0xdb); /*set vcomh*/
	write_cmd(0x30);
	write_cmd(0x8d); /*set charge pump disable*/

	//write_cmd(0x10);		//使用外置DC
	write_cmd(0x14);		//使用内置DC

	lcd_clear(); /* Clear Screen */
	write_cmd(0xAF); /*display ON*/
	WaitMs(1);
	//WaitUs(1000); // 15 us
}

/*********************************************************************
 * @fn      OLED_Set_Pos
 *
 * @brief   定位OLED 显示坐标
 *
 * @param   data
 *
 * @return  none
 */
static void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	write_cmd(0xb0 + y);
	write_cmd(((x & 0xf0) >> 4) | 0x10);
	write_cmd((x & 0x0f) | 0x01);
}

/*********************************************************************
 * @fn		OLED_ShowChar
 *
 * @brief	在指定位置显示一个字符,包括部分字符
 *
 * @param   x:0~127, y:0~63, 
 *			mode:0=反白显示;1=正常显示
 *			size:选择字体 16/12
 *
 * @return  none
 */
static void OLED_ShowChar(uint8 x,uint8 y,uint8 chr)
{
	unsigned char c=0,i=0;
	c=chr-' ';//得到偏移后的值
	if(x>Max_Column-1)
	{
		x=0;
		y=y+2;
	}
	//		if(SIZE ==16)
	//			{
	OLED_Set_Pos(x,y);
	for(i=0;i<8;i++)
	{
		write_data(F8X16[c*16+i]);
	}
	
	OLED_Set_Pos(x,y+1);
	for(i=0;i<8;i++)
	{
		write_data(F8X16[c*16+i+8]);
	}
}

/*********************************************************************
 * @fn		OLED_ShowOneSegment
 *
 * @brief	写单条Segment 数据(16条com线)数据
 *
 * @param   
 *
 * @return  none
 */
//static void OLED_ShowOneSegment(uint8 x,uint8 y,uint8  data1,uint8 data2)
//{
//	OLED_Set_Pos(x,y);
//	write_data(data1);
//	OLED_Set_Pos(x,y+1);
//	write_data(data2);
//}

/*********************************************************************
 * @fn		oled_pow
 *
 * @brief	m^n函数
 *
 * @param   
 *
 * @return  none
 */
static uint32 oled_pow(uint8 m,uint8 n)
{
	uint32 result=1;
	while(n--)result*=m;
	return result;
}

/*********************************************************************
 * @fn		getNumLen
 *
 * @brief	
 *
 * @param   
 *
 * @return  none
 */
static uint8 getNumLen(uint32 num)
{
	uint8 len=1;
	while (1)
	{
		if (num < 10)
		{
			return	len;
		}
		else
		{
			num /= 10;
		}
		len++;
	}
}

/*********************************************************************
 * @fn		OLED_ShowNum
 *
 * @brief	显示数字 并返回数字长度
 *
 * @param   
 *
 * @return  none
 */
uint8 OLED_ShowNum(uint8 x, uint8 y, uint32 num)
{
	uint8 t, temp;
	uint8 enshow = 0;
	uint8 len = getNumLen(num);
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + 8 * t, y, ' ');
				continue;
			} else
				enshow = 1;
		}
		OLED_ShowChar(x + 8 * t, y, temp + '0');
	}
	return len;
}

/*********************************************************************
 * @fn		OLED_ShowChineseNum
 *
 * @brief	显示字符 并返回字符长度
 *
 * @param   
 *
 * @return  none
 */
static uint8 OLED_ShowChineseNum(uint8 x, uint8 y, uint32 num)
{
	uint8 t, temp;
	uint8 enshow = 0;
	uint8 len = getNumLen(num);
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowCHinese(x + 16 * t, y, CHINESE_SPACE);
				continue;
			}
			else
			{
				enshow = 1;
			}
		}
		OLED_ShowCHinese(x + 16 * t, y, temp);
	}
	return len;
}

/*********************************************************************
 * @fn		OLED_ShowString
 *
 * @brief	显示一个字符号串
 *
 * @param   
 *
 * @return  none
 */
void OLED_ShowString(uint8 x,uint8 y,uint8 *chr)
{
	unsigned char j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j]);
		x += 8;
		if (x > 120)
		{
			x = 0;
			y += 2;
		}
		j++;
	}
}

/*********************************************************************
 * @fn		OLED_ShowCHinese
 *
 * @brief	显示汉字
 *
 * @param   
 *
 * @return  none
 */
static void OLED_ShowCHinese(uint8 x,uint8 y,uint8 no)
{
	uint8 t, adder = 0;
	OLED_Set_Pos(x, y);
	for (t = 0; t < 16; t++)
	{
		write_data(Hzk[2 * no][t]);
		adder += 1;
	}
	OLED_Set_Pos(x, y + 1);
	for (t = 0; t < 16; t++)
	{
		write_data(Hzk[2 * no + 1][t]);
		adder += 1;
	}
}

/*********************************************************************
 * @fn		OLED_DrawBMP
 *
 * @brief	显示显示BMP图片128×64
 *
 * @param   起始点坐标(x,y),x的范围0～127，y为页的范围0～7
 *
 * @return  none
 */
//static void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
//{
//	unsigned int j = 0;
//	unsigned char x, y;
//	
//	if (y1 % 8 == 0)
//	{
//		y = y1 / 8;
//	}
//	else
//	{
//		y = y1 / 8 + 1;
//	}
//	
//	for (y = y0; y < y1; y++)
//	{
//		OLED_Set_Pos(x0, y);
//		for (x = x0; x < x1; x++)
//		{
//			write_data(BMP[j++]);
//		}
//	}
//}

/*********************************************************************
 * @fn		HexDigitDis
 *
 * @brief	
 *
 * @param   
 *
 * @return  none
 */
//static void HexDigitDis(uint8 x,uint8 y,uint8 value)
//{
//	uint8 vtTemp;
//	vtTemp=(value>>4)&0x0F;
//	if(vtTemp<=9)
//	{
//		OLED_ShowChar(x, y,vtTemp+'0');
//	}
//	else
//	{
//		OLED_ShowChar(x, y,vtTemp+'A'-10);
//	}
//	
//	vtTemp=value&0x0F;
//	if(vtTemp<=9)
//	{
//		OLED_ShowChar(x+8, y,vtTemp+'0');
//	}
//	else
//	{
//		OLED_ShowChar(x+8, y,vtTemp+'A'-10);
//	}
//}

/*********************************************************************
 * @fn		oledDisplayChineseString
 *
 * @brief	
 *
 * @param   
 *
 * @return  none
 */
static void oledDisplayChineseString(uint8 x,uint8 y,const uint8 *str,uint8 len)
{
	uint8 i=0;
	for(;i<len;i++)
	{
		OLED_ShowCHinese(x+16*i,y,*(str+i));
	}
}



/*********************************************************************
 * @fn      display_Init
 *
 * @brief   Initialization display
 *
 * @param   none
 *
 * @return  none
 */
void display_Init( void )
{
	lcd_bus_init();
	lcd_config();
	//loadDefaultSetting();
}
void reInitialLCD(void) {

	lcd_bus_init();
	lcd_config();
	hal_pwrmgr_lock(MOD_LCD_On);
}
/*********************************************************************
 * @fn      lcd_on
 *
 * @brief   true on lcd
 *
 * @param   data
 *
 * @return  none
 */
void lcd_on(void)
{
	lcd_config();
	reInitialLCD();
	lcd_clear();
//	write_cmd(0xAE); /*display off*/
//	write_cmd(0x8d); /*set charge pump disable*/
//	write_cmd(0x14);		//使用内置DC
//	write_cmd(0xAF); /*display ON*/
	
	WaitMs(1);
}

/*********************************************************************
 * @fn      lcd_off
 *
 * @brief   true off lcd
 *
 * @param   data
 *
 * @return  none
 */
void lcd_off(void)
{
//	write_cmd(0xAE); /*display off*/
//	write_cmd(0x8d); /*set charge pump disable*/
//	write_cmd(0x10);		//使用外置DC
	
	write_cmd(0xAE); /*display off*/
	write_cmd(0x8d); /*set charge pump disable*/
	write_cmd(0x10);		//使用外置DC
	WaitMs(1);
	fIsChargeDisplaying = false;
	lcdBusDeinit();
	hal_pwrmgr_unlock(MOD_LCD_On);
}

/*********************************************************************
 * @fn      lcd_clear
 *
 * @brief   true off lcd
 *
 * @param   data
 *
 * @return  none
 */
void lcd_clear(void)
{
	fIsChargeDisplaying = false;
	 
	unsigned char x, y;
	//	fIsChargeDisplaying = false;
	write_cmd(0x00);
	write_cmd(0x10); /*set higher column address*/
	
	for (y = 0; y < 8; y++)
	{
		write_cmd(0xB0 + y); /*set page address*/
		write_cmd(0x00);
		write_cmd(0x10);
		for (x = 0; x < 128; x++)
		{
			write_data(0x00);
		}
		write_data(0x00);
	}
}

/*********************************************************************
 * @fn		displaySystemMenu
 *
 * @brief	待机画面显示
 *
 * @param   
 *
 * @return  none
 */
void displaySystemMenu(displayParamsStruct * disParams)
{
	updateHotDisplay(false);
	if (IdleIamgeDisplay==disParams->DisplayModeIndex ) {
		switchArrow(disParams);
		batterDisplay(disParams->battLv);
		OLED_ShowCHinese(ICON_BRIGHTNESS_X, ICON_BRIGHTNESS_Y, ICON_BRIGHTNESS_ADDRESS);			//亮度图标
		updateBrightnessDisplay(disParams);
		if(disParams->arrowIndex>=CustomizeEffect)
			disParams->arrowIndex=HuesSetting;
		else if(disParams->arrowIndex==CustomizeEffect)
			return;
		switch(disParams->arrowIndex){
		case HuesSetting:
			oledDisplayChineseString(HSI_X,HSI_Y,HSI_STRING,3);
			OLED_ShowCHinese(ICON_HUES_X, ICON_HUES_Y, ICON_HUES_ADDRESS);							//色调图标
			updateHuesDisplay(disParams);
			break;
		case SaturationSetting:
			oledDisplayChineseString(HSI_X,HSI_Y,HSI_STRING,3);
			OLED_ShowCHinese(ICON_Saturation_X, ICON_Saturation_Y, ICON_Saturation_ADDRESS);				//饱和度图标
			updateSaturationDisplay(disParams);
			break;
		case ColorTempSetting:
			oledDisplayChineseString(HSI_X,HSI_Y,CCT_STRING,3);
			OLED_ShowCHinese(ICON_BRIGHTNESS_X, ICON_ColorTemp_Y, ICON_ColorTemp_ADDRESS);			//色温图标
			updateColorTempDisplay(disParams);
			break;
		case Style1Setting:
			oledDisplayChineseString(HSI_X,HSI_Y,MLM_STRING,3);
			OLED_ShowCHinese(ICON_Flash_X, ICON_Flash_Y, ICON_Flash_ADDRESS);							//右侧闪光灯图标
			OLED_ShowCHinese(ICON_Style1_X, ICON_Style1_Y, disParams->style1Value+1);
			oledDisplayChineseString(ICON_Style1_X+16,ICON_Style1_Y,EMPTY_STRING,4);
			break;
		}


	}else if(ModeTDisplay==disParams->DisplayModeIndex){
		OLED_ShowCHinese(ICON_BRIGHTNESS_X, ICON_BRIGHTNESS_Y, ICON_BRIGHTNESS_ADDRESS);			//亮度图标
		updateBrightnessDisplay(disParams);
		OLED_ShowCHinese(ICON_BRIGHTNESS_X, ICON_ColorTemp_Y, ICON_ColorTemp_ADDRESS);			//色温图标
		updateColorTempDisplay(disParams);
	}else if(CountDownDisplay==disParams->DisplayModeIndex){
//		OLED_ShowCHinese(ICON_T_X, ICON_T_Y, ICON_T_Addr);											//反显T图标
//		OLED_ShowString(Value_CD_X, Value_CD_Y, "    ");
	}else if(ChargingAtPowerDown==disParams->DisplayModeIndex){
		lcd_clear();
		batterDisplay(disParams->battLv);
		return;
	}
}

/*********************************************************************
 * @fn		batteryPercentDisplay
 *
 * @brief	待机画面电池百分比显示
 *
 * @param   
 *
 * @return  none
 */
void batteryPercentDisplay(uint8 level) {
	if (!level) {
		OLED_ShowChar(ICON_Percent_X, ICON_Percent_Y, ' ');
		OLED_ShowChar(ICON_Percent_X + 8, ICON_Percent_Y, ' ');
		OLED_ShowChar(ICON_Percent_X + 16, ICON_Percent_Y, ' ');
		OLED_ShowChar(ICON_Percent_X + 24, ICON_Percent_Y, ' ');
		OLED_ShowChar(ICON_Percent_X + 32, ICON_Percent_Y, ' ');
	} else {

		if (level != 100) {
			OLED_ShowChar(ICON_Percent_X, ICON_Percent_Y, ' ');
			OLED_ShowChar(ICON_Percent_X + 8, ICON_Percent_Y, level/10 + '0');
		} else {
			OLED_ShowChar(ICON_Percent_X, ICON_Percent_Y, '1');
			OLED_ShowChar(ICON_Percent_X + 8, ICON_Percent_Y, '0');
		}
		OLED_ShowChar(ICON_Percent_X + 16, ICON_Percent_Y, level % 10 + '0');
		OLED_ShowCHinese(ICON_Percent_X + 24, ICON_Percent_Y, Value_Percent_Addr);
	}
}
//
///*****************************************************************************************
// *
// *剩余时间显示
// *
//*/
//void updateRemainingTimeByValue(uint16 data)
//{
//	OLED_ShowChar(ValueOfClock_X, Icon_Clock_Y, ' ');
//	OLED_ShowChar(ValueOfClock_X + 8, Icon_Clock_Y, ' ');
//	OLED_ShowChar(ValueOfClock_X + 16, Icon_Clock_Y, ' ');
//	OLED_ShowChar(ValueOfClock_X + 24, Icon_Clock_Y, ' ');
//
//	displayParams.remainingTime=data;
//	if ( data < 10 )
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y, data);
//		OLED_ShowChar(ValueOfClock_X+8,Icon_Clock_Y,LOW_CASE_m);
//	}
//	else if(data<60)
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y,data/10);
//		OLED_ShowNum(ValueOfClock_X+8,Icon_Clock_Y, data%10);
//		OLED_ShowChar(ValueOfClock_X+16,Icon_Clock_Y,LOW_CASE_m);
//	}
//	else if(data<600)	//<10h
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y,data/60);
//		OLED_ShowChar(ValueOfClock_X+8, Icon_Clock_Y, '.');
//		OLED_ShowNum(ValueOfClock_X+12,Icon_Clock_Y,data*10/60%10);
//		OLED_ShowChar(ValueOfClock_X+20,Icon_Clock_Y,LOW_CASE_h);
//	}
//	else if(data<6000)	//<100h
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y,data/60);
//		OLED_ShowChar(ValueOfClock_X+16,Icon_Clock_Y,LOW_CASE_h);
//	}
//	else if(data<59941)	//<999
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y,data/6000);
//		OLED_ShowNum(ValueOfClock_X+8,Icon_Clock_Y,data/600%10);
//		OLED_ShowNum(ValueOfClock_X+16,Icon_Clock_Y,data/60%10);
//		OLED_ShowChar(ValueOfClock_X+24,Icon_Clock_Y,LOW_CASE_h);
//	}
//	else
//	{
//		OLED_ShowNum(ValueOfClock_X,Icon_Clock_Y,9);
//		OLED_ShowNum(ValueOfClock_X+8,Icon_Clock_Y,9);
//		OLED_ShowNum(ValueOfClock_X+16,Icon_Clock_Y,9);
//		OLED_ShowChar(ValueOfClock_X+24,Icon_Clock_Y,LOW_CASE_h);
//	}
//}

/*********************************************************************
 * @fn		batterDisplay
 *
 * @brief	电池显示
 *
 * @param   
 *
 * @return  none
 */
void batterDisplay(uint8 level)
{
	uint8	vtBattIconTableAddr = 0;
	
	if (level > Max_Batt_level)
	{
		level = Max_Batt_level;
	}
	
	if( CCS_GET_HOTDISPLAYSTATUS() == false )
	{
		switch (level)
		{
			default:
			case 0:
			vtBattIconTableAddr=ICON_batt_lv0_Addr;
			break;
			
			case 1:
			vtBattIconTableAddr=ICON_batt_lv1_Addr;
			break;
			
			case 2:
			vtBattIconTableAddr=ICON_batt_lv2_Addr;
			break;
			
			case 3:
			vtBattIconTableAddr=ICON_batt_lv3_Addr;
			break;
			
			case 4:
			vtBattIconTableAddr=ICON_batt_lv4_Addr;
			break;
		}
		
		OLED_ShowCHinese(ICON_batt_X0, ICON_batt_Y, vtBattIconTableAddr);
		OLED_ShowCHinese(ICON_batt_X1, ICON_batt_Y, vtBattIconTableAddr + 1);
	}
	
	if( CCS_GET_ChargeStatus() != 0 )
	{
		if ( !fIsChargeDisplaying )
		{
			fIsChargeDisplaying = true;
//			OLED_ShowCHinese(ICON_charge_X0, ICON_charge_Y, ICON_charge_TableAddr);
			OLED_ShowCHinese(ICON_charge_X0, ICON_charge_Y, ICON_charge_TableAddr);
			OLED_ShowCHinese(ICON_charge_X0+16, ICON_charge_Y, ICON_charge_TableAddr+1);
		}
	}
	else// if(!fIsCharging&&fIsChargeDisplaying)
	{
//		if(!fIsSystemOff)
		if ( fIsChargeDisplaying )
		{						//关机充电指示由再次进入睡眠来完成,防止图标闪烁
			fIsChargeDisplaying = false;
//			OLED_ShowCHinese(ICON_charge_X0, ICON_charge_Y, CHINESE_SPACE);
			OLED_ShowCHinese(ICON_charge_X0, ICON_charge_Y, CHINESE_SPACE);
			OLED_ShowCHinese(ICON_charge_X0+16, ICON_charge_Y, CHINESE_SPACE);
		}
	}
	
	if ( CCS_GET_BLE_Status() )
	{
		updateBLEDisplay(true);
	}
}

/*********************************************************************
 * @fn		updateArrowDisplay
 *
 * @brief	光标显示
 *
 * @param   
 *
 * @return  none
 */
void updateArrowDisplay(displayParamsStruct * disParams)
 {
	if (fIsInvalidMacAddr) {
		lcd_clear();
		OLED_ShowString(HSI_X + 32, HSI_Y + 2, "MAC ERROR");
		OLED_ShowString(HSI_X + 16, HSI_Y + 4, macAscii);
	} else {
		displaySystemMenu(&displayParams);
		switch (disParams->arrowIndex) {
		default:
		case HuesSetting:
			updateHuesDisplay(&displayParams);
			break;

		case SaturationSetting:
			updateSaturationDisplay(&displayParams);
			break;

		case ColorTempSetting:
			updateColorTempDisplay(&displayParams);
			break;

		case Style1Setting:
		case Style2Setting:
		case Style3Setting:
			updateLightEffectDisplay(&displayParams);
			break;
		}
	}

}
/*********************************************************
 *
 *
 *
 */
void switchArrow(displayParamsStruct * disParams) {
	if (disParams->isBrightnessAdjisting) {
		OLED_ShowChar(0, 3, ' ');
		OLED_ShowChar(0, 6, ICON_Arrow_ADDRESS);
	} else {
		OLED_ShowChar(0, 3, ICON_Arrow_ADDRESS);
		OLED_ShowChar(0, 6, ' ');
	}
}
/*********************************************************************
 * @fn		updateBLEDisplay
 *
 * @brief	BLE显示
 *
 * @param   
 *
 * @return  none
 */
void updateBLEDisplay(bool flag)
{
	if ( flag )
	{
		OLED_ShowCHinese(ICON_BLE_X, ICON_BLE_Y, ICON_BLE_ADDRESS);
	}
	else
	{
		OLED_ShowString(ICON_BLE_X, ICON_BLE_Y, "  ");
	}
}

/*********************************************************************
 * @fn		updateHuesDisplay
 *
 * @brief	色调值显示
 *
 * @param   
 *
 * @return  none
 */
void updateHuesDisplay(displayParamsStruct * disParams)
{
	uint8 len=0;
	OLED_ShowCHinese(Value_Hues_X+8+16*3, ICON_HUES_Y,CHINESE_SPACE);
	OLED_ShowCHinese(Value_Hues_X+8+16*4, ICON_HUES_Y,CHINESE_SPACE);
	len = OLED_ShowChineseNum(Value_Hues_X,ICON_HUES_Y,disParams->hues);
	OLED_ShowChar(Value_Hues_X+16*len, ICON_HUES_Y, ICON_Degree_ADDRESS);
	if(len<3)
	{
		OLED_ShowCHinese(Value_Hues_X+8+16*2, ICON_HUES_Y,CHINESE_SPACE);
	}
	if(len<2)
	{
		OLED_ShowCHinese(Value_Hues_X+8+16*1, ICON_HUES_Y,CHINESE_SPACE);
	}
}


/*********************************************************************
 * @fn		updateSaturationDisplay
 *
 * @brief	饱和度值显示
 *
 * @param   
 *
 * @return  none
 */
void updateSaturationDisplay(displayParamsStruct * disParams)
{
	uint8 len=0;
		OLED_ShowCHinese(Value_Saturation_X+16*4, ICON_HUES_Y,CHINESE_SPACE);
		OLED_ShowCHinese(Value_Saturation_X+16*3, ICON_HUES_Y,CHINESE_SPACE);
		len=OLED_ShowChineseNum(Value_Saturation_X,ICON_Saturation_Y,disParams->saturation);
		if(len<3)
			OLED_ShowCHinese(Value_Saturation_X+16*2, ICON_HUES_Y,CHINESE_SPACE);
		if(len<2)
			OLED_ShowCHinese(Value_Saturation_X+16*1, ICON_HUES_Y,CHINESE_SPACE);
}


void	 clearSecondLineDisplay(void){
	for(uint8 i=0;i<7;i++)
		OLED_ShowCHinese(ICON_HUES_X+16*i, ICON_HUES_Y,CHINESE_SPACE);
}
/*********************************************************************
 * @fn		updateBrightnessDisplay
 *
 * @brief	亮度值显示
 *
 * @param   
 *
 * @return  none
 */
void updateBrightnessDisplay(displayParamsStruct * disParams)
{
	uint8 len = 0;
	len = OLED_ShowChineseNum(Value_Brightness_X,ICON_BRIGHTNESS_Y,disParams->brightness);
	OLED_ShowChar(Value_Brightness_X+16*len,ICON_BRIGHTNESS_Y, ' ');
	OLED_ShowCHinese(Value_Brightness_X+8+16*len, ICON_BRIGHTNESS_Y, Value_Percent_Addr);
	if(len<3)
	{
		OLED_ShowCHinese(Value_Brightness_X+8+16*3, ICON_BRIGHTNESS_Y,CHINESE_SPACE);
	}
	if(len<2)
	{
		OLED_ShowCHinese(Value_Brightness_X+8+16*2, ICON_BRIGHTNESS_Y,CHINESE_SPACE);
	}
}

/*********************************************************************
 * @fn		updateColorTempDisplay
 *
 * @brief	色温值显示
 *
 * @param   
 *
 * @return  none
 */
void updateColorTempDisplay(displayParamsStruct * disParams)
{
	uint8 len = 0;
//	len = OLED_ShowNum(Value_ColorTemp_X, ICON_ColorTemp_Y, disParams->colorTemperature*100);
//	OLED_ShowChar(Value_ColorTemp_X+8*len, ICON_ColorTemp_Y, 'K');
	len=OLED_ShowChineseNum(Value_ColorTemp_X,ICON_ColorTemp_Y,disParams->colorTemperature*100);
	OLED_ShowCHinese(Value_ColorTemp_X+16*len, ICON_ColorTemp_Y, CHINESE_K);
}

/*********************************************************************
 * @fn		updateLightEffectDisplay
 *
 * @brief	灯效样式显示
 *
 * @param   
 *
 * @return  none
 */
void updateLightEffectDisplay(displayParamsStruct * disParams)
{
	if(Style1Setting==disParams->arrowIndex)
		OLED_ShowCHinese(ICON_Style1_X, ICON_Style1_Y,1+disParams->style1Value);
//	else if(Style2Setting==disParams->arrowIndex)
//		OLED_ShowCHinese(ICON_Style1_X, ICON_Style1_Y,1+disParams->style2Value);
//	else if(Style3Setting==disParams->arrowIndex)
//		OLED_ShowCHinese(ICON_Style1_X, ICON_Style1_Y,1+disParams->style3Value);
}

/*********************************************************************
 * @fn		updateHotDisplay
 *
 * @brief	过热显示
 *
 * @param   
 *
 * @return  none
 */
void updateHotDisplay(bool flag)
{
	fIsHotDisplayOn = flag;

//	if(fIsHotNow)
//	{
		if(fIsHotDisplayOn)
		{
			OLED_ShowCHinese(ICON_Hot_X, ICON_Hot_Y,ICON_Hot_Addr);
			OLED_ShowCHinese(ICON_Hot_X1, ICON_Hot_Y,ICON_Hot_Addr+1);
			fIsHotDisplayOn=TRUE;
		}
		else
		{
			OLED_ShowCHinese(ICON_Hot_X, ICON_Hot_Y,ICON_batt_lv0_Addr);
			OLED_ShowCHinese(ICON_Hot_X1, ICON_Hot_Y,ICON_batt_lv0_Addr+1);
			fIsHotDisplayOn=FALSE;
		}
//	}
}

/*********************************************************************
 * @fn		temperatureDisplay
 *
 * @brief	测试显示
 *
 * @param   
 *
 * @return  none
 */
void temperatureDisplay(uint8 x, uint8 y, uint32 value)
{
//	//return;
//	uint8 vtTemperature_X = 0;
//	uint8 vtTemperature_Y = 0;
//	uint8 vtVolt_X = 0;
//	uint8 vtVolt_Y = 0;
//	uint8 vtCVolt_X = 0;
//	uint8 vtCVolt_Y = 0;
//	if (fIsSystemTempGot)
//	{
//		fIsSystemTempGot = 0;
//		if (IdleIamgeDisplay == displayParams.DisplayModeIndex)
//		{
//			vtTemperature_X = 52;
//			vtTemperature_Y = 0;
//			vtVolt_X = 52;
//			vtVolt_Y = 2;
//			vtCVolt_X =vtVolt_X+8 ;
//			vtCVolt_Y = 4;
//		}
//		else
//		{
//			vtTemperature_X = 8;
//			vtTemperature_Y = 2;
//			vtVolt_X = 80;
//			vtVolt_Y = 2;
//			vtCVolt_X =vtVolt_X ;
//			vtCVolt_Y = 4;
//		}
//		OLED_ShowChar(vtTemperature_X, vtTemperature_Y, vSystemTemperature / 10 + '0');
//		OLED_ShowChar(vtTemperature_X+8, vtTemperature_Y, vSystemTemperature % 10 + '0');
//		OLED_ShowChar(vtTemperature_X + 16, vtTemperature_Y, ICON_Degree_ADDRESS);
//		//---------------------------------------------------------------------
//		OLED_ShowChar(vtVolt_X, vtVolt_Y, ((uint8) vTestBatt) + '0');
//		OLED_ShowChar(vtVolt_X + 8, vtVolt_Y, '.');
//		OLED_ShowChar(vtVolt_X + 13, vtVolt_Y, ((uint8) (vTestBatt * 10)) % 10 + '0');
//		OLED_ShowChar(vtVolt_X + 21, vtVolt_Y, ((uint16) (vTestBatt * 100)) % 10 + '0');
//		OLED_ShowChar(vtVolt_X + 29, vtVolt_Y, 'V');
//		//---------------------------------------------------------------------
//		OLED_ShowChar(vtCVolt_X, vtCVolt_Y, ((uint8) vTestCompBatt) + '0');
//		OLED_ShowChar(vtCVolt_X + 8, vtCVolt_Y, '.');
//		OLED_ShowChar(vtCVolt_X + 13, vtCVolt_Y, ((uint8) (vTestCompBatt * 10)) % 10 + '0');
//		OLED_ShowChar(vtCVolt_X + 21, vtCVolt_Y, ((uint16) (vTestCompBatt * 100)) % 10 + '0');
//		//---------------------------------------------------------------------
//		OLED_ShowChar(vtCVolt_X, vtCVolt_Y+2, ((uint8) vSystemVdd) + '0');
//		OLED_ShowChar(vtCVolt_X + 8, vtCVolt_Y+2, '.');
//		OLED_ShowChar(vtCVolt_X + 13, vtCVolt_Y+2, ((uint8) (vSystemVdd * 10)) % 10 + '0');
//		OLED_ShowChar(vtCVolt_X + 21, vtCVolt_Y+2, ((uint16) (vSystemVdd * 100)) % 10 + '0');
//	}
	
	OLED_ShowString(x, y,"    ");
	OLED_ShowChineseNum(x, y, value);
}

void temperatureDisplay_char(uint8 x, uint8 y, uint8 value)
{
	OLED_ShowString(x,y,"    ");
	OLED_ShowCHinese(x, y , value);
}

/************************************************************************
 *
 * 浮点小数显示
 *	x,y:起始坐标
 *	size: 小数点后显示几位
 *	per:显示单位
 */
void displayFloat(UINT8 x,UINT8 y,float fNum,UINT8 size,UINT8 per){
	UINT16 vtInt=0;
	UINT8 len=0,dotPos=0,i=0;
	vtInt=(UINT16)fNum;
//	OLED_ShowString(x,y,"    ");
	len=OLED_ShowNum(x,y,vtInt);
	dotPos=x+8*len;
	if(size){
		OLED_ShowChar(dotPos, y, '.');
		for(i=1;i<=size;i++){
			OLED_ShowChar(dotPos+5+(i-1)*8, y, ((u16) (fNum * oled_pow(10,i))) % 10 + '0');
		}
		OLED_ShowChar(dotPos +5+(size)*8, y, per);
	}else{
		OLED_ShowChar(dotPos , y, per);
		OLED_ShowChar(dotPos+8 , y, ' ');
	}
}
void displayFactoryInfo(u8 item) {
	switch (item) {
	case NonError:
		break;
	case TemperatureError:
		oledDisplayChineseString(32, 2,TEMP_ERROR , 4);
		break;
	case VoltageError:
		lcd_clear();
		oledDisplayChineseString(32, 4, VOLT_ERROR, 4);
		break;
	case ClearAllInfo:
		lcd_clear();
		break;
	}
}

/*************************** (C) COPYRIGHT 2012 Bough*****END OF FILE*****************************/
