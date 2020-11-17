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
#include "math.h"
#include "io_cfg.h"
#include "xprintf.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
//---------------------------------------SHT-------------------------------//

//https://www.mouser.com/datasheet/2/682/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf

char *QueueStatus[3]={"Empty","Available","Full"};
/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
int val=0, store=0;

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
    struct Queue* q = createQueue();
    uart1_init(230400);
        xprintf_stream_io_out = uart1_putc;
        xprintf("Hi :) !!!\n");
    //eMBErrorCode    eStatus;
    //(void)eStatus;

	/* ----------------------- Test Porting -------------------------------------*/

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
       //UART_config();
    //eStatus = eMBInit( MB_RTU, 0x01, 0, 115200, MB_PAR_NONE );

	/* Enable the Modbus Protocol Stack. */
    //eStatus = eMBEnable(  );

	for( ;; )
    {
        xprintf("Enqueue:\n");
        for(int i=0; i<80; i++){
        enQueue(q,val++);
        xprintf(QueueStatus[getRear(q,&store)]);
        xprintf("   ");
        printfloat(store);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4,!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4));
        //vMBPortTimersDelay(1);
        }
        xprintf("Dequeue:\n");
        for(int i=0; i<100; i++){
        deQueue(q,&store);
        xprintf(QueueStatus[getFront(q,&store)]);
        xprintf("   ");
        printfloat(store);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4,!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4));
        //vMBPortTimersDelay(1);
        }
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

void printfloat(float val_float){
    int val_int[2];
    val_int[0]=val_float;
    val_int[1]=(float)(val_float*100-val_int[0]*100);
    xprintf("%d.%d\n",val_int[0],val_int[1]);
}
