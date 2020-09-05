/*
 * FreeModbus Libary: BARE Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */


#include "app.h"
#include "platform.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
#include "I2C_STM32F0.h"
#include "math.h"
/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4

//---------------------------------------SHT-------------------------------//
#define SHTaddress   0x44
uint8_t val[20],check1,check2;//buffer chua gia tri thanh ghi va check fuction thuc hien thanh cong
uint8_t fetch[20]={0x2C,0x06};// High Repeatability - Enable Clock stretching
//https://www.mouser.com/datasheet/2/682/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf
float temp,humid;
/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];

uint8_t setup[20]={PWR_MGMT_1,0};
long timeval=0;
uint8_t dem=ACCEL_XOUT_H ;
/* ----------------------- Start implementation -----------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;
USART_InitTypeDef       UART_InitStructure;
NVIC_InitTypeDef        NVIC_InitStructure;
I2C_InitTypeDef         I2C_InitStructure;
TIM_TypeDef             TIM_InitStructure;
USART_InitTypeDef       USART_InitStructure;
void
app_dbg_fatal( const int8_t* s, uint8_t c )
{
    int delayVal = 50;
	(void)s;
	(void)c;
	while (1)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		vMBPortTimersDelay(delayVal);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		vMBPortTimersDelay(delayVal);
	}
}

void
main_app( void )
{
    eMBErrorCode    eStatus;
    (void)eStatus;

	/* ----------------------- Test Porting -------------------------------------*/

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
       //UART_config();
    eStatus = eMBInit( MB_RTU, 0x01, 0, 115200, MB_PAR_NONE );

	/* Enable the Modbus Protocol Stack. */
    eStatus = eMBEnable(  );
    //Init I2C 2 mode fast 400KHZ va low 100KHZ
     I2C1_Master_Config(FASTMODE);

	for( ;; )
    {
        ( void )eMBPoll(  );
        vMBPortTimersDelay(100);
        //check1=I2C_write(I2C1,MPU,fetch,2,1,NO_END);// thu tu la ten I2C/dia chi slave/buffer doc hoac ghi/so byte doc hoac ghi/co ket thuc duong truyen khong hay tiep tuc?

        //ghi gia tri doc vao val, gui lenh iu cau xuat do am nhiet do
        check1=I2C_write(I2C1,SHTaddress,fetch,2,10,NO_END);
        check2=I2C_read(I2C1,SHTaddress,val,6,10,END);

        //tinh toan gia tri
        temp=((val[0]<<8)|val[1])/0xFFFF;
        temp=(175*temp)-45;
        humid=((val[3]<<8)|val[4])/0xFFFF;
        humid*=100;

        usRegInputBuf[0] = temp;//gia tri nhiet
        usRegInputBuf[1] = humid;//gia tri do am
        usRegInputBuf[2] = check1;//check ghi I2C thanh cong khong //1 la Ok //0 la Fail
        usRegInputBuf[3] = check2;//check doc I2C thanh cong khong
        //usRegInputBuf[3] = I2C_scan(I2C1); // scan đia chi salve gan 0 nhat
    }
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if( ( usAddress >= REG_INPUT_START )
			&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
	{
		iRegIndex = ( int )( usAddress - usRegInputStart );
		while( usNRegs > 0 )
		{
			*pucRegBuffer++ =
					( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
			*pucRegBuffer++ =
					( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
			iRegIndex++;
			usNRegs--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
				 eMBRegisterMode eMode )
{
	(void)pucRegBuffer;
	(void)usAddress;
	(void)usNRegs;
	(void)eMode;
	return MB_ENOREG;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
			   eMBRegisterMode eMode )
{
	(void)pucRegBuffer;
	(void)usAddress;
	(void)usNCoils;
	(void)eMode;
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	(void)pucRegBuffer;
	(void)usAddress;
	(void)usNDiscrete;
	return MB_ENOREG;
}



