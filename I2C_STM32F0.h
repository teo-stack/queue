#ifndef I2C_STM32F0_H
#define I2C_STM32F0_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
I2C_InitTypeDef         I2Clib_InitStructure;
GPIO_InitTypeDef        GPIOlib_InitStructure;

typedef enum {NO_END = 0, END = !NO_END} FuncState;
typedef enum {FASTMODE = 0x10310309, LOWMODE = 0x30310309} SpeedMode;
enum{FAIL=0,SUCCCESS=1};
enum{multitime=10000};

extern uint8_t I2C_write(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num,long timeout,FuncState state);
extern uint8_t I2C_read(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num,long timeout,FuncState state);
extern uint8_t I2C_scan(I2C_TypeDef* I2Cx);
extern void I2C1_Master_Config(SpeedMode mode);

#ifdef __cplusplus
}
#endif
#endif
