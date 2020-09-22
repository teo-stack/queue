#ifndef SHTX_H
#define SHTX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <I2C_STM32F0.h>

#define POLYNOMIAL  0x131
typedef enum {SHT_default_address = 0x44, SHT_other_address = 0x45} SHT_address;
typedef enum {
    CMD_READ_SERIALNBR  = 0x3780, // read serial number
    CMD_READ_STATUS     = 0xF32D, // read status register
    CMD_CLEAR_STATUS    = 0x3041, // clear status register
    CMD_HEATER_ENABLE   = 0x306D, // enabled heater
    CMD_HEATER_DISABLE  = 0x3066, // disable heater
    CMD_SOFT_RESET      = 0x30A2, // soft reset
    CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
    CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
    CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
    CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
    CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
    CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
    CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
    CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
    CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
    CMD_NO_SLEEP        = 0x303E,
} SHT_Command;
typedef enum{
    CMD_MEAS_PERI_05_H  = 0x2032, // measurement: periodic 0.5 mps, high repeatability
    CMD_MEAS_PERI_05_M  = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
    CMD_MEAS_PERI_05_L  = 0x202F, // measurement: periodic 0.5 mps, low repeatability
    CMD_MEAS_PERI_1_H   = 0x2130, // measurement: periodic 1 mps, high repeatability
    CMD_MEAS_PERI_1_M   = 0x2126, // measurement: periodic 1 mps, medium repeatability
    CMD_MEAS_PERI_1_L   = 0x212D, // measurement: periodic 1 mps, low repeatability
    CMD_MEAS_PERI_2_H   = 0x2236, // measurement: periodic 2 mps, high repeatability
    CMD_MEAS_PERI_2_M   = 0x2220, // measurement: periodic 2 mps, medium repeatability
    CMD_MEAS_PERI_2_L   = 0x222B, // measurement: periodic 2 mps, low repeatability
    CMD_MEAS_PERI_4_H   = 0x2334, // measurement: periodic 4 mps, high repeatability
    CMD_MEAS_PERI_4_M   = 0x2322, // measurement: periodic 4 mps, medium repeatability
    CMD_MEAS_PERI_4_L   = 0x2329, // measurement: periodic 4 mps, low repeatability
    CMD_MEAS_PERI_10_H  = 0x2737, // measurement: periodic 10 mps, high repeatability
    CMD_MEAS_PERI_10_M  = 0x2721, // measurement: periodic 10 mps, medium repeatability
    CMD_MEAS_PERI_10_L  = 0x272A, // measurement: periodic 10 mps, low repeatability
} SHT_Period_Mode;
typedef enum{
    CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
    CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
    CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
} SHT_ClockStr_Mode;
typedef enum{
    CMD_MEAS_POLLING_H  = 0x2400, // measurement: polling, high repeatability
    CMD_MEAS_POLLING_M  = 0x240B, // measurement: polling, medium repeatability
    CMD_MEAS_POLLING_L  = 0x2416, // measurement: polling, low repeatability
} SHT_Polling_Mode;
typedef enum{
    NO_ERROR,
    ADDRESS_ERROR,
    BYTE1_ERROR,
    BYTE2_ERROR,
    BYTE3_ERROR,
    BYTE4_ERROR,
    BYTE5_ERROR,
    BYTE6_ERROR,
    CRC_ERROR,
    PARA_ERROR
} SHT_Error;
typedef struct {
    uint8_t tempval[2];
    uint8_t CRC_temp;
    uint8_t humidval[2];
    uint8_t CRC_humid;
}Raw_SHT_Data;

typedef struct {
    uint8_t any_val[2];
    uint8_t CRC_val;
}Raw_2B_Data;

void SHT_init(SHT_address address,SpeedMode speed);
SHT_Error SHT_Write(uint16_t command,FuncState state);
SHT_Error SHT_Read(uint8_t* buffer,long timeout,int nbr,FuncState state);
SHT_Error SHT_CRC_check(int CRCval,uint8_t* data, uint8_t nbrOfBytes);
SHT_Error SHT_Read_ClockStr(float *temp,float *humid,int timeout,SHT_ClockStr_Mode mode);
SHT_Error SHT_Read_Polling(float *temp,float *humid,int timeout,SHT_Polling_Mode mode);
SHT_Error SHT_Start_Period(SHT_Period_Mode mode);
SHT_Error SHT_Read_Period(float *temp,float *humid);
int SHT_CalcCrc(uint8_t* data, uint8_t nbrOfBytes);
SHT_Error SHT_Hard_Reset(void);
SHT_Error SHT_Soft_Reset(void);
SHT_Error SHT_Read_Status(uint16_t* status);
SHT_Error SHT_Clear_Flags(void);
SHT_Error SHT_Read_Serial(uint32_t* status);
SHT_Error SHT_Enable_Heater(void);
SHT_Error SHT_Disable_Heater(void);
float SHT_Temp_Cal(uint16_t temp);
float SHT_Humid_Cal(uint16_t humid);
void wait(long ms);
int SHT_address_store,speed_store;
#ifdef __cplusplus
}
#endif
#endif
