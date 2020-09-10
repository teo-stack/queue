#ifndef SHTX_H
#define SHTX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <I2C_STM32F0.h>
#define POLYNOMIAL  0x131
typedef enum {SHT_default_address = 0x44, SHT_other_address = 0x45} SHT_address;
typedef enum {SHT_HIGHREPEAT = 0x2C06,
             CMD_SOFT_RESET  = 0x30A2} SHT_command;
typedef struct {
    uint8_t tempval[2];
    uint8_t CRC_temp;
    uint8_t humidval[2];
    uint8_t CRC_humid;
}Raw_SHT_Data;
extern void SHT_init(SHT_address address);
extern int SHT_Write(SHT_command command,FuncState state);
extern int SHT_Read(char* buffer,long timeout,int nbr,FuncState state);
extern int SHT_CRC_check(int CRCval,char* data, char nbrOfBytes);
extern int SHT_Read_Raw(float *temp,float *humid);
extern int SHT_address_store;
extern int SHT_CalcCrc(char* data, char nbrOfBytes);
extern int SHT_Reset(void);
extern int SHT_Soft_Reset(void);
extern void wait(long ms);
#ifdef __cplusplus
}
#endif
#endif
