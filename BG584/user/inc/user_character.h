/**************************************************************************************************
Filename:       user_character.h
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
contact Bouth R&D at www.bough.com.hk
**************************************************************************************************/

/*-----------------------------------------------------------------------------------------------*/
/* Copyright(c) 2012 Bough Technology Corp. All rights reserved.                                 */
/*-----------------------------------------------------------------------------------------------*/
#ifndef USER_CHARACTER_H_
#define USER_CHARACTER_H_

/*********************************************************************
* INCLUDES
*/
#include "types.h"
#include "display.h"

//const u8 ArrowArray[7][2]={{0,0},{0,1},{0,3},{0,5},{86,3},{86,5},{86,5}};
/****************************************8*16的点阵************************************/
const unsigned char  F8X16[]=
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//! 1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//" 2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//# 3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$ 4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//% 5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//& 6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//' 7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//( 8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//) 9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//* 10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+ 11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//, 12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//- 13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//. 14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,/// 15
  0x00,0xE0,0xF0,0x18,0x08,0x18,0xF0,0xE0,0x00,0x0F,0x1F,0x30,0x20,0x30,0x1F,0x0F,/*"0",16*/
  0x00,0x00,0x10,0x10,0xF8,0xF8,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x3F,0x20,0x20,/*"1",17*/
  0x00,0x70,0x78,0x08,0x08,0x08,0xF8,0xF0,0x00,0x30,0x38,0x2C,0x26,0x23,0x31,0x30,/*"2",18*/
  0x00,0x30,0x38,0x08,0x08,0x88,0xF8,0x70,0x00,0x18,0x38,0x21,0x21,0x23,0x3E,0x1C,/*"3",19*/
  0x00,0x00,0x80,0xC0,0x70,0xF8,0xF8,0x00,0x00,0x06,0x07,0x25,0x24,0x3F,0x3F,0x24,/*"4",20*/
  0x00,0xF8,0xF8,0x88,0x88,0x88,0x08,0x08,0x00,0x19,0x39,0x20,0x20,0x31,0x1F,0x0E,/*"5",21*/
  0x00,0xE0,0xF0,0x98,0x88,0x98,0x90,0x00,0x00,0x0F,0x1F,0x31,0x20,0x20,0x3F,0x1F,/*"6",22*/
  0x00,0x18,0x18,0x08,0x88,0xE8,0x78,0x18,0x00,0x00,0x00,0x3E,0x3F,0x01,0x00,0x00,/*"7",23*/
  0x00,0x70,0xF8,0x88,0x08,0x88,0xF8,0x70,0x00,0x1C,0x3E,0x23,0x21,0x23,0x3E,0x1C,/*"8",24*/
  0x00,0xF0,0xF8,0x08,0x08,0x18,0xF0,0xE0,0x00,0x01,0x13,0x32,0x22,0x33,0x1F,0x0F,/*"9",25*/
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//: 26+32
//  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//; 27+32
//  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//< 28+32
//  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//= 29+32
//  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//> 30+32
//  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//? 31+32
  0x10,0xF0,0xF0,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x3F,0x21,0x00,0x20,0x3F,0x3F,//h 27+32
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m 28+32
  0x00,0x0C,0x12,0x12,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"º",29+32*/
  0x00,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,/*"▶",30+32*/
  0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,0x00,/*":",31+32*/
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@ 32+32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A 33+32
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B 34+32
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C 35+32
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D 36+32
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E 37+32
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F 38+32
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G 39+32
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H 40+32
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I 41+32
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J 42+32
  0x08,0xF8,0xF8,0xC8,0xE8,0x38,0x18,0x08,0x20,0x3F,0x3F,0x21,0x27,0x3E,0x38,0x20,/*"K",43*/ //+32
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L 44+32
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M 45+32
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N 46+32
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O 47+32
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P 48+32
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q 49+32
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R 50+32
  0x00,0x70,0xF8,0x88,0x08,0x08,0x38,0x38,0x00,0x38,0x38,0x21,0x21,0x23,0x3E,0x1C,/*"S",51*/
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T 52+32
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U 53+32
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V 54+32
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W 55+32
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X 56+32
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y 57+32
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z 58+32
  0x00,0x0C,0x12,0x12,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"º",59+32*/
  0x00,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,/*"▶",60+32*/
  0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,0x00,/*":",61+32*/

  0x00,0xFC,0xFC,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x3F,0x3F,0x30,0x30,0x30,0x30,0x30,  // processing bar start 62+32
  0x0C,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x30,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,   // processing bar end 63+32		95
  0x0C,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x30,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,   // 	96=0x60
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a 65+32=0x71
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b 66+32=0x72
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c 67+32=0x73
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d 68+32=0x74
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e 69+32=0x75
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f 70+32=0x76
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g 71+32=0x77
  0x10,0xF0,0xF0,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x3F,0x21,0x00,0x20,0x3F,0x3F,//h 72+32=0x78
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i 73+32=0x79
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j 74+32=0x7A
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k 75+32=0x7B
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l 76+32=0x7C
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m 77+32=0x7D
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n 78+32=0x7E
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o 79+32=0x7F
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p 80+32=0x80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q 81+32=0x81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r 82+32=0x82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s 83+32=0x83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t 84+32=0x84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u 85+32=0x85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v 86+32=0x86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w 87+32=0x87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x 88+32=0x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y 89+32=0x89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z 90+32=0x8A
};


const unsigned char  Hzk[][16]={

		//0
		{0x00,0x00,0x00,0xFC,0xFE,0xFF,0x07,0x03,0x83,0xC3,0x67,0xFF,0xFE,0xFC,0x00,0x00},
		{0x00,0x00,0x00,0x3F,0x7F,0xFF,0xE6,0xC3,0xC1,0xC0,0xE0,0xFF,0x7F,0x3F,0x00,0x00},
		//1
		{0x00,0x00,0x00,0x00,0x08,0x0C,0x0E,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0x00,0x00},
		//2
		{0x00,0x00,0x00,0x1C,0x1E,0x0F,0x07,0x03,0x03,0x83,0xC7,0xFF,0xFE,0x7C,0x00,0x00},
		{0x00,0x00,0x00,0xE0,0xF0,0xF8,0xFC,0xFE,0xEF,0xE7,0xE3,0xE1,0xE0,0xE0,0x00,0x00},
		//3
		{0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x63,0xF3,0xFB,0xDF,0x8F,0x07,0x03,0x00,0x00},
		{0x00,0x00,0x00,0x38,0x70,0xE0,0xC0,0xC0,0xC0,0xC1,0xE3,0xFF,0x7F,0x3E,0x00,0x00},
		//4
		{0x00,0x00,0x00,0xFE,0xFF,0xFF,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x01,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFF,0x03,0x03,0x00,0x00},
		//5
		{0x00,0x00,0x00,0x7F,0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0x83,0x03,0x00,0x00},
		{0x00,0x00,0x00,0x40,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE1,0xFF,0x7F,0x3F,0x00,0x00},
		//6
		{0x00,0x00,0x00,0xFC,0xFE,0xFF,0xC7,0xC3,0xC3,0xC3,0xC3,0x83,0x82,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x3F,0x7F,0xFF,0xE1,0xC0,0xC0,0xC0,0xE1,0xFF,0x7F,0x3F,0x00,0x00},
		//7
		{0x00,0x00,0x00,0x03,0x03,0x03,0x03,0xC3,0xE3,0xF3,0x3F,0x1F,0x0F,0x07,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},
		//8
		{0x00,0x00,0x00,0x00,0x3C,0x7E,0xE7,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x3C,0x7E,0xFF,0xE3,0xC1,0xC1,0xC1,0xE3,0xFF,0x7E,0x3C,0x00,0x00},
		//9
		{0x00,0x00,0x00,0xFC,0xFE,0xFF,0x87,0x03,0x03,0x03,0x87,0xFF,0xFE,0xFC,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x41,0xC3,0xC3,0xC3,0xC3,0xC3,0xE3,0xFF,0x7F,0x3F,0x00,0x00},

		//H -10
		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0x80,0x80,0x80,0x80,0x80,0xFF,0xFF,0xFF,0x00,0x00},
		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01,0x01,0xFF,0xFF,0xFF,0x00,0x00},
		//S-11
		{0x00,0x00,0x00,0x38,0x7C,0xFE,0xC7,0x83,0x83,0x83,0x83,0x03,0x03,0x02,0x00,0x00},
		{0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC1,0xC1,0xC1,0xC1,0xE3,0x7F,0x3E,0x1C,0x00,0x00},
		//亮度图标  12
		{0x80,0x80,0x0C,0xCC,0xE0,0x30,0x18,0x1B,0x1B,0x18,0x30,0xE0,0xCC,0x0C,0x80,0x80},
		{0x01,0x01,0x30,0x33,0x07,0x0C,0x18,0xD8,0xD8,0x18,0x0C,0x07,0x33,0x30,0x01,0x01},
		//色温图标   13
		{0x00,0x00,0x00,0x80,0x98,0x3C,0x66,0xC3,0xC3,0x66,0x3C,0x98,0x80,0x00,0x00,0x00},
		{0x0C,0x1E,0x33,0x61,0x61,0x33,0x1E,0x0C,0x0C,0x1E,0x33,0x61,0x61,0x33,0x1E,0x0C},
		//闪光灯图标  14
		{0xF8,0xF8,0x0C,0x0E,0x0E,0x8E,0xCE,0xEE,0xEE,0x0E,0x0E,0x8E,0xCE,0x4C,0xF8,0xF8},
		{0x3F,0x7F,0xE4,0xE6,0xE3,0xE1,0xE0,0xEF,0xEF,0xE6,0xE3,0xE1,0xE0,0xE0,0x7F,0x3F},
		//循环图标  15
		{0xF0,0xF8,0x1C,0x0C,0x00,0x0C,0x1E,0x3F,0x0C,0x0C,0x0C,0x0C,0x0C,0x1C,0xF8,0xF0},
		{0x07,0x0F,0x1C,0x18,0x18,0x18,0x18,0x18,0x7E,0x3C,0x18,0x00,0x18,0x1C,0x0F,0x07},
		//锁图标	16
		{0x00,0x00,0x00,0x00,0x00,0xF8,0xFC,0x06,0x06,0x06,0xFC,0xF8,0x00,0x00,0x00,0x00},
		{0x3C,0x7E,0x67,0x63,0x63,0x63,0x63,0x63,0x63,0x63,0x63,0x63,0x63,0x67,0x7E,0x3C},
		//M 17
//		{0x00,0x00,0x00,0xFF,0xFE,0x1C,0x38,0x70,0xF0,0x70,0x38,0x1C,0xFE,0xFF,0x00,0x00},
//		{0x00,0x00,0x00,0x7F,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x7F,0x00,0x00},
		{0x00,0x00,0xFC,0xFF,0xFE,0x1C,0x38,0x70,0xF0,0x70,0x38,0x1C,0xFE,0xFF,0xFC,0x00},
		{0x00,0x00,0x7F,0x7F,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x7F,0x7F,0x00},
		//L 18
//		{0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
//		{0x00,0x00,0x00,0x7F,0x7F,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x00,0x00},
		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x7F,0x7F,0x7F,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x00,0x00},
		// C 19
		{0x00,0x00,0x00,0xF8,0xFC,0xFE,0x0F,0x07,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00},
		{0x00,0x00,0x00,0x1F,0x3F,0x7F,0xF0,0xE0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00},

		//20
		{0x00,0x00,0x00,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x03,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},
		//batt0 21-22
		{0xF8,0xF8,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18},
		{0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0},
		{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},

		//batt1 23-24
		{0xF8,0xF8,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18},
		{0xFF,0xFF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0},
		{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},

		//batt2 25-26
		{0xF8,0xF8,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x18,0x18},
		{0xFF,0xFF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xC0,0xC0},
		{0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},

		//batt3 27-28
		{0xF8,0xF8,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x98,0x98},
		{0xFF,0xFF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xCF,0xCF},
		{0x98,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xCF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},

		//batt4 29-30
		{0xF8,0xF8,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0x98,0x98},
		{0xFF,0xFF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xCF,0xCF},
		{0x98,0x18,0x18,0x98,0x98,0x98,0x18,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xCF,0xC0,0xC0,0xCF,0xCF,0xCF,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},

		//I  31
		{0x00,0x00,0x00,0x00,0x00,0x03,0x03,0xFF,0xFF,0xFF,0x03,0x03,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0x00,0x00,0x00},

		// chinese space   32
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

		//K  33
		{0x00,0x00,0x00,0xFE,0xFF,0xFF,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x7F,0xFF,0xFF,0x03,0x07,0x0E,0x1C,0x38,0x70,0xE0,0xC0,0x00,0x00},

		{0xE8,0x08,0xE8,0xE8,0xE8,0x08,0xE8,0xE8,0xE8,0x08,0x78,0xC0,0x00,0x00,0x00,0x00},		//34
		{0x2F,0x20,0x2F,0x2F,0x2F,0x20,0x2F,0x2F,0x2F,0x20,0x3C,0x07,0x00,0x00,0x00,0x00},
		// %  35
		{0x00,0x38,0x7C,0xC6,0xC6,0x7C,0x38,0x80,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x00,0x00},
		{0x00,0x60,0x70,0x38,0x1C,0x0E,0x07,0x03,0x1D,0x3E,0x63,0x63,0x3E,0x1C,0x00,0x00},


		//CHARGE 36-37
//		{0x60,0x60,0xF8,0xF8,0x18,0x18,0x18,0xD8,0xD8,0x98,0x18,0x30,0x60,0xC0,0xC0,0xC0},
//		{0x0C,0x0C,0x3F,0x3F,0x30,0x31,0x31,0x37,0x37,0x33,0x31,0x18,0x0C,0x06,0x07,0x07},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0xC0,0xF0,0xF0},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x7F,0x7F},
		{0x30,0x30,0x30,0xB0,0xB0,0x30,0x30,0x60,0xC0,0x80,0x80,0x80,0x00,0x00,0x00,0x00},
		{0x60,0x62,0x62,0x6F,0x6F,0x67,0x62,0x30,0x18,0x0D,0x0F,0x0F,0x02,0x02,0x02,0x02},

		//BLE	38
		{0x00,0x00,0x00,0x60,0xF0,0x98,0x08,0xF8,0xF8,0x00,0x80,0xC0,0x60,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x30,0x78,0xCD,0x87,0xFF,0xFF,0x07,0x0D,0x18,0x30,0x00,0x00,0x00},

//		{0x00,0x00,0xFE,0xFE,0x06,0xF6,0xF6,0xC6,0xC6,0xF6,0xF6,0x06,0xF6,0xF6,0x36,0x36},
//		{0x00,0x00,0x3F,0x3F,0x30,0x37,0x37,0x30,0x30,0x37,0x37,0x30,0x37,0x37,0x36,0x36},
//		{0xF6,0xF6,0x06,0x36,0x36,0xF6,0xF6,0x36,0x36,0x06,0x3E,0x3E,0xF0,0xF0,0x00,0x00},
//		{0x37,0x37,0x30,0x30,0x30,0x37,0x37,0x30,0x30,0x30,0x3E,0x3E,0x07,0x07,0x00,0x00},
		//hot	39-40
		{0xF8,0xF8,0x18,0xD8,0xD8,0x18,0x18,0xD8,0xD8,0x18,0x98,0xD8,0xD8,0xD8,0xD8,0x98},
		{0xFF,0xFF,0xC0,0xDF,0xDF,0xC3,0xC3,0xDF,0xDF,0xC0,0xCF,0xDF,0xD8,0xD8,0xDF,0xCF},
		{0x18,0xD8,0xD8,0xD8,0xD8,0xD8,0xD8,0x18,0xF8,0xF8,0xC0,0xC0,0x00,0x00,0x00,0x00},
		{0xC0,0xC0,0xC0,0xDF,0xDF,0xC0,0xC0,0xC0,0xF8,0xF8,0x1F,0x1F,0x00,0x00,0x00,0x00},


//		{0x00,0xE0,0xF8,0x1C,0x0C,0x06,0x06,0xFE,0xFE,0x86,0x86,0x8C,0x9C,0xF8,0xE0,0x00},		//30		clock
//		{0x00,0x07,0x1F,0x38,0x30,0x60,0x60,0x61,0x61,0x61,0x61,0x31,0x39,0x1F,0x07,0x00},

//		{0x00,0x00,0x10,0x20,0x40,0x80,0x00,0xFE,0x04,0x88,0x50,0x20,0x00,0x00,0x00,0x00},		//31		BLE
//		{0x00,0x00,0x10,0x08,0x04,0x02,0x01,0xFF,0x41,0x22,0x14,0x08,0x00,0x00,0x00,0x00},

		{0x40,0x30,0xEF,0x24,0x64,0x48,0x48,0x7F,0x48,0x48,0x48,0x7F,0x48,0x48,0x40,0x00},
		{0x01,0x01,0x7F,0x21,0x11,0x00,0xFF,0x49,0x49,0x49,0x49,0x49,0xFF,0x00,0x00,0x00},		/*"错",41*/

		{0x40,0x42,0xCC,0x00,0x00,0x80,0x9E,0x92,0x92,0x92,0x92,0x92,0x9E,0x80,0x00,0x00},
		{0x00,0x00,0x7F,0x20,0x94,0x84,0x44,0x24,0x14,0x0F,0x14,0x24,0x44,0x84,0x84,0x00},		/*"误",42*/

		{0x10,0x60,0x02,0x8C,0x00,0x00,0xFE,0x92,0x92,0x92,0x92,0x92,0xFE,0x00,0x00,0x00},
		{0x04,0x04,0x7E,0x01,0x40,0x7E,0x42,0x42,0x7E,0x42,0x7E,0x42,0x42,0x7E,0x40,0x00},		/*"温",43*/

		{0x00,0x00,0xFC,0x24,0x24,0x24,0xFC,0x25,0x26,0x24,0xFC,0x24,0x24,0x24,0x04,0x00},
		{0x40,0x30,0x8F,0x80,0x84,0x4C,0x55,0x25,0x25,0x25,0x55,0x4C,0x80,0x80,0x80,0x00},		/*"度",44*/

		//H 45
		{0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00},
		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFF,0x00,0x00},
		//S 46
		{0x00,0x00,0x00,0x00,0xE0,0xF0,0xB8,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0xC0,0xC1,0xC3,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C,0x18,0x00,0x00},
		//I 47
		{0x00,0x00,0x00,0x00,0x00,0x18,0x18,0xF8,0xF8,0xF8,0x18,0x18,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0x00,0x00,0x00},
		//C 48
		{0x00,0x00,0x00,0xC0,0xE0,0xF0,0x78,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00},
		{0x00,0x00,0x00,0x1F,0x3F,0x7F,0xF0,0xE0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00},
		//T 49
		{0x00,0x00,0x00,0x18,0x18,0x18,0x18,0xF8,0xF8,0xF8,0x18,0x18,0x18,0x18,0x00,0x00},
		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},
		//M 50
		{0x00,0x00,0xF8,0xF8,0xF0,0xE0,0xC0,0x80,0x80,0x80,0xC0,0xE0,0xF0,0xF8,0xF8,0x00},
		{0x00,0x00,0xFF,0xFF,0xFF,0x00,0x01,0x03,0x07,0x03,0x01,0x00,0xFF,0xFF,0xFF,0x00},
		//L 51
		{0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x00},

		{0x00,0x00,0xF8,0x88,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x88,0xF8,0x00,0x00,0x00},
		{0x00,0x00,0x1F,0x08,0x08,0x08,0x08,0x7F,0x88,0x88,0x88,0x88,0x9F,0x80,0xF0,0x00},		/*"电",52*/

		{0x00,0x00,0xFE,0x02,0x82,0x82,0x82,0x82,0xFA,0x82,0x82,0x82,0x82,0x82,0x02,0x00},
		{0x80,0x60,0x1F,0x40,0x40,0x40,0x40,0x40,0x7F,0x40,0x40,0x44,0x58,0x40,0x40,0x00},		/*"压",53*/

//		{0x10,0x60,0x02,0x8C,0x00,0x00,0xFE,0x92,0x92,0x92,0x92,0x92,0xFE,0x00,0x00,0x00},
//		{0x04,0x04,0x7E,0x01,0x40,0x7E,0x42,0x42,0x7E,0x42,0x7E,0x42,0x42,0x7E,0x40,0x00},		/*"温",54*/
//
//		{0x00,0x00,0xFC,0x24,0x24,0x24,0xFC,0x25,0x26,0x24,0xFC,0x24,0x24,0x24,0x04,0x00},
//		{0x40,0x30,0x8F,0x80,0x84,0x4C,0x55,0x25,0x25,0x25,0x55,0x4C,0x80,0x80,0x80,0x00},		/*"度",55*/
//
//		{0x00,0x40,0x42,0x44,0x58,0x40,0x40,0x7F,0x40,0x40,0x50,0x48,0xC6,0x00,0x00,0x00},
//		{0x00,0x40,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0xFF,0x00,0x00,0x00},		/*"当",56*/
//
//		{0x08,0x08,0xE8,0x29,0x2E,0x28,0xE8,0x08,0x08,0xC8,0x0C,0x0B,0xE8,0x08,0x08,0x00},
//		{0x00,0x00,0xFF,0x09,0x49,0x89,0x7F,0x00,0x00,0x0F,0x40,0x80,0x7F,0x00,0x00,0x00},		/*"前",57*/
//
//		{0x00,0x02,0x0C,0xC0,0x00,0x40,0x20,0x10,0x0C,0x23,0xCC,0x10,0x20,0x40,0x40,0x00},
//		{0x02,0x02,0x7F,0x00,0x00,0x00,0x01,0x09,0x11,0x21,0xD1,0x0D,0x03,0x00,0x00,0x00},		/*"冷",58*/
//
//		{0x00,0xFC,0x44,0x44,0xFC,0x02,0x26,0x2A,0xE2,0x26,0x29,0x21,0x29,0x25,0x00,0x00},
//		{0x00,0x1F,0x08,0x08,0x5F,0x21,0x99,0x87,0x4D,0x55,0x25,0x55,0x4D,0x81,0x81,0x00},		/*"暖",59*/
//
//		{0x00,0x10,0x88,0xC4,0x33,0x40,0x48,0x48,0x48,0x7F,0x48,0xC8,0x48,0x48,0x40,0x00},
//		{0x02,0x01,0x00,0xFF,0x00,0x02,0x0A,0x32,0x02,0x42,0x82,0x7F,0x02,0x02,0x02,0x00},		/*"待",60*/
//
//		{0x10,0x10,0xD0,0xFF,0x90,0x10,0x00,0xFE,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00},
//		{0x04,0x03,0x00,0xFF,0x00,0x83,0x60,0x1F,0x00,0x00,0x00,0x3F,0x40,0x40,0x78,0x00},		/*"机",61*/
//
//
//		{0x00,0x00,0x10,0x11,0x16,0x10,0x10,0xF0,0x10,0x10,0x14,0x13,0x10,0x00,0x00,0x00},
//		{0x81,0x81,0x41,0x41,0x21,0x11,0x0D,0x03,0x0D,0x11,0x21,0x41,0x41,0x81,0x81,0x00},		/*"关",62*/
//
//		{0x10,0x10,0x10,0xFF,0x90,0x20,0x98,0x88,0x88,0xE9,0x8E,0x88,0x88,0xA8,0x98,0x00},
//		{0x02,0x42,0x81,0x7F,0x00,0x00,0x80,0x84,0x4B,0x28,0x10,0x28,0x47,0x80,0x00,0x00},	/*"按",63*/
//
//		{0x00,0x80,0x60,0xF8,0x87,0x80,0x84,0x84,0x84,0xFE,0x82,0x83,0x82,0x80,0x80,0x00},
//		{0x01,0x00,0x00,0xFF,0x00,0x40,0x40,0x40,0x40,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},	/*"任",64*/
//
//		{0x10,0x10,0x12,0xD2,0x56,0x5A,0x52,0x53,0x52,0x5A,0x56,0xD2,0x12,0x10,0x10,0x00},
//		{0x40,0x30,0x00,0x77,0x85,0x85,0x8D,0xB5,0x85,0x85,0x85,0xE7,0x00,0x10,0x60,0x00},	/*"意",65*/
//
//		{0x40,0x30,0xEF,0x24,0x24,0x80,0xE4,0x9C,0x10,0x54,0x54,0xFF,0x54,0x7C,0x10,0x00},
//		{0x01,0x01,0x7F,0x21,0x51,0x26,0x18,0x27,0x44,0x45,0x45,0x5F,0x45,0x45,0x44,0x00},	/*"键",66*/
//
//		{0x80,0x82,0x82,0x82,0xFE,0x82,0x82,0x82,0x82,0x82,0xFE,0x82,0x82,0x82,0x80,0x00},
//		{0x00,0x80,0x40,0x30,0x0F,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00},	/*"开",67*/
//
//		{0x10,0x10,0xF0,0x1F,0x10,0xF0,0x00,0x40,0xE0,0x58,0x47,0x40,0x50,0x60,0xC0,0x00},
//		{0x40,0x22,0x15,0x08,0x16,0x21,0x00,0x00,0xFE,0x42,0x42,0x42,0x42,0xFE,0x00,0x00},	/*"始",68*/
//
//
//		{0x10,0x60,0x02,0x8C,0x00,0xFE,0x02,0xF2,0x02,0xFE,0x00,0xF8,0x00,0xFF,0x00,0x00},
//		{0x04,0x04,0x7E,0x01,0x80,0x47,0x30,0x0F,0x10,0x27,0x00,0x47,0x80,0x7F,0x00,0x00},/*"测",69*/
//
//		{0x40,0x40,0x42,0xCC,0x00,0x90,0x90,0x90,0x90,0x90,0xFF,0x10,0x11,0x16,0x10,0x00},
//		{0x00,0x00,0x00,0x3F,0x10,0x28,0x60,0x3F,0x10,0x10,0x01,0x0E,0x30,0x40,0xF0,0x00},/*"试",70*/
//
//		{0x00,0x00,0xF8,0x49,0x4A,0x4C,0x48,0xF8,0x48,0x4C,0x4A,0x49,0xF8,0x00,0x00,0x00},
//		{0x10,0x10,0x13,0x12,0x12,0x12,0x12,0xFF,0x12,0x12,0x12,0x12,0x13,0x10,0x10,0x00},/*"单",71*/
//
//		{0x00,0x80,0x60,0xF8,0x07,0x10,0x90,0x10,0x11,0x16,0x10,0x10,0xD0,0x10,0x00,0x00},
//		{0x01,0x00,0x00,0xFF,0x40,0x40,0x41,0x5E,0x40,0x40,0x70,0x4E,0x41,0x40,0x40,0x00},/*"位",72*/
//
//		{0x00,0x04,0x24,0x24,0x24,0x24,0x24,0xFF,0x24,0x24,0x24,0x24,0x24,0x04,0x00,0x00},
//		{0x21,0x21,0x11,0x09,0xFD,0x83,0x41,0x23,0x05,0x09,0x11,0x29,0x25,0x41,0x41,0x00},/*"表",73*/
//
//
//		{0x40,0x30,0xEF,0x24,0x64,0x48,0x48,0x7F,0x48,0x48,0x48,0x7F,0x48,0x48,0x40,0x00},
//		{0x01,0x01,0x7F,0x21,0x11,0x00,0xFF,0x49,0x49,0x49,0x49,0x49,0xFF,0x00,0x00,0x00},/*"错",74*/
//
//		{0x40,0x42,0xCC,0x00,0x00,0x80,0x9E,0x92,0x92,0x92,0x92,0x92,0x9E,0x80,0x00,0x00},
//		{0x00,0x00,0x7F,0x20,0x94,0x84,0x44,0x24,0x14,0x0F,0x14,0x24,0x44,0x84,0x84,0x00},/*"误",75*/
//
//		{0x08,0x08,0x08,0xF8,0x08,0x08,0x08,0x10,0x10,0xFF,0x10,0x10,0x10,0xF0,0x00,0x00},
//		{0x10,0x30,0x10,0x1F,0x08,0x88,0x48,0x30,0x0E,0x01,0x40,0x80,0x40,0x3F,0x00,0x00},/*"功",76*/
//
//		{0x08,0xCC,0x4A,0x49,0x48,0x4A,0xCC,0x18,0x00,0x7F,0x88,0x88,0x84,0x82,0xE0,0x00},
//		{0x00,0xFF,0x12,0x12,0x52,0x92,0x7F,0x00,0x00,0x7E,0x88,0x88,0x84,0x82,0xE0,0x00},/*"能",77*/
//
//		{0x40,0x40,0x40,0xFF,0x20,0x20,0x20,0x04,0x04,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00},
//		{0x00,0x00,0x00,0x1F,0x08,0x84,0x42,0x20,0x18,0x07,0x40,0x80,0x40,0x3F,0x00,0x00},/*"切",78*/
//
//		{0x10,0x10,0x10,0xFF,0x90,0x20,0x10,0xE8,0x27,0x24,0xE4,0x34,0x2C,0xE0,0x00,0x00},
//		{0x02,0x42,0x81,0x7F,0x00,0x84,0x84,0x47,0x24,0x1C,0x07,0x1C,0x24,0x47,0x84,0x00},/*"换",79*/
//
//		{0x40,0x42,0xCC,0x00,0x00,0xFE,0x82,0x92,0x92,0xFE,0x92,0x92,0x82,0xFE,0x00,0x00},
//		{0x00,0x00,0x3F,0x10,0x88,0x7F,0x00,0x1E,0x12,0x12,0x12,0x5E,0x80,0x7F,0x00,0x00},/*"调",80*/
//
//		{0x04,0x44,0x44,0x44,0x5F,0x44,0xC4,0x44,0x44,0x44,0x5F,0x44,0xC4,0x04,0x04,0x00},
//		{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x08,0x10,0x08,0x07,0x00,0x00,0x00},/*"节",81*/
//
//
//		{0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
//		{0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00},/*-  82*/
//
//		{0x00,0x04,0x04,0x74,0x54,0x54,0x55,0x56,0x54,0x54,0x54,0x74,0x04,0x04,0x00,0x00},
//		{0x84,0x83,0x41,0x21,0x1D,0x05,0x05,0x05,0x05,0x05,0x7D,0x81,0x81,0x85,0xE3,0x00},/*"亮",83*/
//
//		{0x00,0x00,0xFC,0x24,0x24,0x24,0xFC,0x25,0x26,0x24,0xFC,0x24,0x24,0x24,0x04,0x00},
//		{0x40,0x30,0x8F,0x80,0x84,0x4C,0x55,0x25,0x25,0x25,0x55,0x4C,0x80,0x80,0x80,0x00},/*"度",84*/
//
//
//		{0x00,0x00,0x80,0x80,0x80,0x80,0x80,0xFC,0xFC,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
//		{0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x3F,0x3F,0x01,0x01,0x01,0x01,0x01,0x00,0x00},/*+ 85*/
//
//		{0x40,0x44,0x54,0x64,0x45,0x7E,0x44,0x44,0x44,0x7E,0x45,0x64,0x54,0x44,0x40,0x00},
//		{0x00,0x00,0x00,0xFF,0x49,0x49,0x49,0x49,0x49,0x49,0x49,0xFF,0x00,0x00,0x00,0x00},/*"普",86*/
//
//		{0x40,0x42,0xCC,0x00,0x00,0xE2,0x22,0x2A,0x2A,0xF2,0x2A,0x26,0x22,0xE0,0x00,0x00},
//		{0x80,0x40,0x3F,0x40,0x80,0xFF,0x89,0x89,0x89,0xBF,0x89,0xA9,0xC9,0xBF,0x80,0x00},/*"通",87*/
//
//		{0x00,0xE0,0x00,0xFF,0x10,0x20,0x08,0x08,0x08,0xFF,0x08,0x08,0xF8,0x00,0x00,0x00},
//		{0x01,0x00,0x00,0xFF,0x00,0x81,0x41,0x31,0x0D,0x03,0x0D,0x31,0x41,0x81,0x81,0x00},/*"快",88*/
//
//		{0x40,0x40,0x42,0xCC,0x00,0x04,0xF4,0x94,0x94,0xFF,0x94,0x94,0xF4,0x04,0x00,0x00},
//		{0x00,0x40,0x20,0x1F,0x20,0x48,0x44,0x42,0x41,0x5F,0x41,0x42,0x44,0x48,0x40,0x00},/*"速",89*/
//
//		{0x04,0x04,0x84,0xC4,0xA4,0x9C,0x85,0x86,0x84,0x84,0xA4,0xC4,0x84,0x04,0x04,0x00},
//		{0x00,0x80,0x80,0x40,0x30,0x0F,0x00,0x00,0x00,0x7F,0x80,0x80,0x81,0xF0,0x00,0x00},/*"充",90*/
//
//		{0x00,0x00,0xF8,0x88,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x88,0xF8,0x00,0x00,0x00},
//		{0x00,0x00,0x1F,0x08,0x08,0x08,0x08,0x7F,0x88,0x88,0x88,0x88,0x9F,0x80,0xF0,0x00},/*"电",91*/
//
//		{0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0xFF,0x10,0x10,0x10,0x10,0xF0,0x00,0x00,0x00},
//		{0x00,0x00,0x0F,0x04,0x04,0x04,0x04,0xFF,0x04,0x04,0x04,0x04,0x0F,0x00,0x00,0x00},/*"中",92*/
//
//		{0x10,0x60,0x02,0xCC,0x80,0x80,0xFC,0x40,0x20,0xFF,0x10,0x08,0xF8,0x00,0x00,0x00},
//		{0x04,0x04,0x7E,0x01,0x00,0x00,0x3F,0x40,0x40,0x4F,0x40,0x44,0x47,0x40,0x78,0x00},/*"池",93*/
//
//		{0x00,0x00,0xFE,0x02,0x82,0x82,0x82,0x82,0xFA,0x82,0x82,0x82,0x82,0x82,0x02,0x00},
//		{0x80,0x60,0x1F,0x40,0x40,0x40,0x40,0x40,0x7F,0x40,0x40,0x44,0x58,0x40,0x40,0x00},/*"压",94*/
//		//H 95
//		{0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00},
//		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFF,0x00,0x00},
//		//S 96
//		{0x00,0x00,0x00,0x00,0xE0,0xF0,0xB8,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
//		{0x00,0x00,0x00,0x00,0xC0,0xC1,0xC3,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C,0x18,0x00,0x00},
//		//I 97
//		{0x00,0x00,0x00,0x00,0x00,0x18,0x18,0xF8,0xF8,0xF8,0x18,0x18,0x00,0x00,0x00,0x00},
//		{0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0x00,0x00,0x00},
//		//C 98
//		{0x00,0x00,0x00,0xC0,0xE0,0xF0,0x78,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00},
//		{0x00,0x00,0x00,0x1F,0x3F,0x7F,0xF0,0xE0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00},
//		//T 99
//		{0x00,0x00,0x00,0x18,0x18,0x18,0x18,0xF8,0xF8,0xF8,0x18,0x18,0x18,0x18,0x00,0x00},
//		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00},
//		//M 100
//		{0x00,0x00,0xF8,0xF8,0xF0,0xE0,0xC0,0x80,0x80,0x80,0xC0,0xE0,0xF0,0xF8,0xF8,0x00},
//		{0x00,0x00,0xFF,0xFF,0xFF,0x00,0x01,0x03,0x07,0x03,0x01,0x00,0xFF,0xFF,0xFF,0x00},
//		//L 101
//		{0x00,0x00,0x00,0xF8,0xF8,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
//		{0x00,0x00,0x00,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x00,0x00,0x00},
//		//mv 102
//		{0x40,0x80,0x40,0x40,0x80,0x40,0x40,0x80,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0xC0},
//		{0x00,0x7F,0x00,0x00,0x7F,0x00,0x00,0x7F,0x00,0x00,0x07,0x38,0x40,0x38,0x07,0x00},
//		//温度 103
//		{0x80,0x40,0x40,0x80,0x00,0x80,0xC0,0x40,0x40,0x80,0x00,0x00,0x00,0x00,0x00,0x00},
//		{0x01,0x02,0x02,0x01,0x00,0x1F,0x30,0x20,0x20,0x10,0x00,0x00,0x00,0x00,0x00,0x00},
};
//
const uint8		CCT_STRING[3]={48,48,49};
const uint8		MLM_STRING[3]={50,51,50};
const uint8		HSI_STRING[3]={45,46,47};
const uint8		EMPTY_STRING[4]={32,32,32,32};

const uint8		TEMP_ERROR[4]={43,44,41,42};										//温度错误
const uint8         	VOLT_ERROR[4]={52,53,41,42};										//电压错误

#endif	//#ifndef USER_CHARACTER_H_
