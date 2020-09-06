#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C"
{
#endif

//#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
#include <stdint.h>


extern void main_app();
extern void app_dbg_fatal(const int8_t* s, uint8_t c);
extern void I2C_Config();
extern void read_data();
extern void read(I2C_TypeDef* I2Cx,char address,char *buffer,char val);
extern void send(I2C_TypeDef* I2Cx,char address,char *buffer,char val);
extern void Configure_Master(I2C_TypeDef* I2Cx);
extern uint8_t I2C_Soft_Read(uint8_t address,uint8_t* buffer,uint8_t num,long timeout,uint8_t end);
extern uint8_t I2C_Soft_Write(uint8_t address,uint8_t* buffer,uint8_t num,uint8_t end);
extern uint8_t I2c_Wait(uint8_t timeout);
extern uint8_t read_byte(uint8_t *rxByte,uint8_t ack, long timeout);
extern uint8_t write_byte(uint8_t txByte);
extern void generate_stop();
extern void generate_start();
extern void delay_us(long time);
extern void I2C_Soft_int();
extern uint8_t val[20];
extern uint8_t fetch[20];
extern uint8_t setup[20];

#define I2C_Chanel      		I2C1
#define MPU                     0x68
#define ACCEL_XOUT_H            0x3B
#define PWR_MGMT_1              0x6B
#define SCL                     GPIO_Pin_9
#define SDA                     GPIO_Pin_10
#ifdef __cplusplus
}
#endif

#endif //__APP_H__
