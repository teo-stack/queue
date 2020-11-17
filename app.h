#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C"
{
#endif

//#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"
#include <stdint.h>
#include "system.h"
#include "queuell.h"

extern void main_app();
extern void app_dbg_fatal(const int8_t* s, uint8_t c);

void printfloat(float val_float);

#ifdef __cplusplus
}
#endif

#endif //__APP_H__
