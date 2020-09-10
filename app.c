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
#include "io_cfg.h"
#include "xprintf.h"
#include "SHTx.h"
/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
//---------------------------------------SHT-------------------------------//
#define SHTaddress   0x44
uint8_t val[20],check1,check2;//buffer chua gia tri thanh ghi va check fuction thuc hien thanh cong
uint8_t fetch[20]={0x2C,0x06};// High Repeatability - Enable Clock stretching
//https://www.mouser.com/datasheet/2/682/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf
float temp,humid;
int tem_int,humid_int;
char buffer[6];
Raw_SHT_Data *dataSHT;
int SHT_address_store;
int error=0;
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
{   	uart1_init(115200);
        xprintf_stream_io_out = uart1_putc;
        xprintf("Hi :) !!!\n");
        dataSHT=(Raw_SHT_Data*)val;
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
    //Init I2C 2 mode fast 400KHZ va low 100KHZ
       SHT_init(SHT_default_address);
	for( ;; )
    {
        vMBPortTimersDelay(1000);
        error = SHT_Read_Raw(&temp,&humid);
        //tem_int=temp;
        //humid_int=humid;
        xprintf("Nhiet do: ");
        printfloat(temp);
        xprintf("Do am: ");
        printfloat(humid);
        xprintf("THIS VALUE != 0 IS ERROR: %d\n",error);
        xprintf("\n");
        while(error!=0) error=SHT_Reset();
        GPIO_WriteBit(GPIOA, GPIO_Pin_4,!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4));
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
void I2C_Soft_int(){
    // cap clock cho ngoai vi va I2C						// su dung kenh I2C2 cua STM32
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    // cau hinh chan SDA va SCL
    GPIOlib_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;						//PA9 - SCL, PA10 - SDA
    GPIOlib_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOlib_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOlib_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIOlib_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIOlib_InitStructure);
    GPIO_WriteBit(GPIOA,SCL,1);
    GPIO_WriteBit(GPIOA,SDA,1);
}

void delay_us(long time){
    time*=SystemCoreClock/1000000;
    while(time--);
}

void generate_start(){
    GPIO_WriteBit(GPIOA,SDA,1);
    delay_us(1);
    GPIO_WriteBit(GPIOA,SCL,1);
    delay_us(1);
    GPIO_WriteBit(GPIOA,SDA,0);
    delay_us(10);
    GPIO_WriteBit(GPIOA,SCL,0);
    delay_us(10);
}

void generate_stop(){
    GPIO_WriteBit(GPIOA,SCL,0);
    delay_us(1);
    GPIO_WriteBit(GPIOA,SDA,0);
    delay_us(1);
    GPIO_WriteBit(GPIOA,SCL,1);
    delay_us(10);
    GPIO_WriteBit(GPIOA,SDA,1);
    delay_us(10);
}
uint8_t write_byte(uint8_t txByte){
    uint8_t    mask,error;
    for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
    {
      if((mask & txByte) == 0) GPIO_WriteBit(GPIOA,SDA,0); // masking txByte, write bit to SDA-Line
      else                     GPIO_WriteBit(GPIOA,SDA,1);
      delay_us(1);               // data set-up time (t_SU;DAT)
      GPIO_WriteBit(GPIOA,SCL,1);                         // generate clock pulse on SCL
      delay_us(5);               // SCL high time (t_HIGH)
      GPIO_WriteBit(GPIOA,SCL,0);
      delay_us(1);               // data hold time(t_HD;DAT)
    }
    GPIO_WriteBit(GPIOA,SDA,1);                           // release SDA-line
    GPIO_WriteBit(GPIOA,SCL,1);                          // clk #9 for ack
    delay_us(1);                 // data set-up time (t_SU;DAT)
    if(GPIO_ReadInputDataBit(GPIOA,SDA)) error = 1;
    else error = 0;// check ack from i2c slave
    GPIO_WriteBit(GPIOA,SCL,0);
    delay_us(20);
    return error;
}
uint8_t read_byte(uint8_t *rxByte,uint8_t ack, long timeout)
{
  uint8_t mask,error;
  *rxByte = 0x00;
  GPIO_WriteBit(GPIOA,SDA,1);                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  {
    GPIO_WriteBit(GPIOA,SCL,1);                          // start clock on SCL-line
    delay_us(1);                // clock set-up time (t_SU;CLK)
    error = I2c_Wait(timeout);// wait while clock streching
    delay_us(3);                // SCL high time (t_HIGH)
    if(GPIO_ReadInputDataBit(GPIOA,SDA)) *rxByte |= mask;        // read bit
    GPIO_WriteBit(GPIOA,SCL,0);
    delay_us(1);                // data hold time(t_HD;DAT)
  }
  if(ack) GPIO_WriteBit(GPIOA,SDA,0);   //ACK           // send acknowledge if necessary
  else           GPIO_WriteBit(GPIOA,SDA,1);  //NACK
  delay_us(1);                  // data set-up time (t_SU;DAT)
  GPIO_WriteBit(GPIOA,SCL,1);                          // clk #9 for ack
  delay_us(5);                  // SCL high time (t_HIGH)
  GPIO_WriteBit(GPIOA,SCL,0);
  GPIO_WriteBit(GPIOA,SDA,1);                           // release SDA-line
  delay_us(20);                 // wait to see byte package on scope

  return error;                          // return with no error
}
uint8_t I2c_Wait(uint8_t timeout)
{
  uint8_t error = 0;

  while(!GPIO_ReadInputDataBit(GPIOA,SCL))
  {
    if(timeout-- == 0) return 1;
    delay_us(1000);
  }

  return error;
}
uint8_t I2C_Soft_Write(uint8_t address,uint8_t* buffer,uint8_t num,uint8_t end){
    uint8_t error=0;
    generate_start();
    error=write_byte(address<<1);
    if(error) return 1;
    for(int i=0;i<num;i++){
    error=write_byte(*buffer++);
    if(error) {return 2;}
    }
    if(end) generate_stop();
    return error;
}
uint8_t I2C_Soft_Read(uint8_t address,uint8_t* buffer,uint8_t num,long timeout,uint8_t end){
    uint8_t error=0;
    generate_start();
    error=write_byte((address<<1)|1);
    if(error) return 1;

    for(int i=0;i<num;i++){
    if(i!=num-1 && i!= 0) error=read_byte(buffer++,1,0);
    else if(i==0) error=read_byte(buffer++,1,timeout);
    else error=read_byte(buffer++,0,0);

    if(error) {return 2;}
    }
    if(end) generate_stop();
    return error;
}
void printfloat(float val_float){
    int val_int[2];
    val_int[0]=val_float;
    val_int[1]=(float)(val_float*100-val_int[0]*100);
    xprintf("%d.%d\n",val_int[0],val_int[1]);
}
