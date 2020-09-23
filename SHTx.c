#include <SHTx.h>
int SHT_address_store,speed_store;float temp_store,humid_store;
//--------------------SHT Init-------------------//
void SHT_Init(SHT_address address,SpeedMode speed){
    SHT_address_store=address;
    speed_store=speed;
    I2C1_Master_Config(speed_store);
}
//--------------------SHT Hard Reset--------------------//
SHT_Error SHT_Hard_Reset(void){
    SHT_Error error=NO_ERROR;
    I2C1_Reconfig();
    wait(1000);
    error = SHT_Soft_Reset();
    return error;
}
//--------------------SHT Write--------------------//
SHT_Error SHT_Write(uint16_t command , FuncState state){
    SHT_Error error=NO_ERROR;
    uint8_t buffer[2];
    buffer[0]=(command>>8) & 0xFF;
    buffer[1]=command & 0xFF;
    error = I2C_write_test(I2C1,SHT_address_store,buffer,2,state);
    return error;
}
//--------------------SHT Write CRC--------------------//
SHT_Error SHT_Write_Command_Data(uint16_t command ,uint16_t value , FuncState state){
    SHT_Error error=NO_ERROR;
    uint8_t buffer[5];
    Raw_4B_Data *data;
    data=(Raw_4B_Data *)buffer;
    data->command[0]    = (command>>8)&0xFF;
    data->command[1]    = (command)&0xFF;
    data->any_val[0]    = (value>>8)&0xFF;
    data->any_val[1]    = (value)&0xFF;
    data->CRC_val       =  SHT_CalcCrc(data->any_val,2);
    error = I2C_write_test(I2C1,SHT_address_store,buffer,5,state);
    return error;
}
//--------------------SHT Read--------------------//
SHT_Error SHT_Read(uint8_t* buffer,long timeout,int nbr,FuncState state){
    SHT_Error error=NO_ERROR;
    error = I2C_read_test(I2C1,SHT_address_store,buffer,nbr,timeout,state);
    return error;
}
//--------------------SHT CRC check--------------------//
SHT_Error SHT_CRC_check(int CRCval,uint8_t* data, uint8_t nbrOfBytes){
    int CRC_check=0;
    SHT_Error error=NO_ERROR;
    CRC_check=SHT_CalcCrc(data,nbrOfBytes);
    if(CRC_check!=CRCval) error=CRC_ERROR;
    return error;
}
//--------------------SHT Read ClockStr--------------------//
SHT_Error SHT_Read_ClockStr(float *temp,float *humid,long timeout,SHT_ClockStr_Mode mode){
    uint8_t buffer[6];
    Raw_SHT_Data *dataSHT;
    SHT_Error error=NO_ERROR;
    if(mode==CMD_MEAS_CLOCKSTR_L || mode==CMD_MEAS_CLOCKSTR_H || mode==CMD_MEAS_CLOCKSTR_M)
        error = SHT_Write(mode, NO_END);
    else
        error = PARA_ERROR;
    if(error==NO_ERROR)
        error = SHT_Read(buffer,timeout,6,END);
    if(error==NO_ERROR){
        dataSHT=(Raw_SHT_Data*)buffer;
        error = SHT_CRC_check(dataSHT->CRC_temp,dataSHT->tempval,2);
        error |= SHT_CRC_check(dataSHT->CRC_humid,dataSHT->humidval,2);
    }
    if(error==NO_ERROR){
        *temp=SHT_Temp_Cal(((dataSHT->tempval[0]<<8)|dataSHT->tempval[1]));
        *humid=SHT_Humid_Cal(((dataSHT->humidval[0]<<8)|dataSHT->humidval[1]));
    }

    return error;
}
//--------------------SHT Read Polling--------------------//
SHT_Error SHT_Read_Polling(float *temp,float *humid,long timeout,SHT_Polling_Mode mode){
    uint8_t buffer[6];
    Raw_SHT_Data *dataSHT;
    SHT_Error error=NO_ERROR;
    if(mode==CMD_MEAS_POLLING_H || mode==CMD_MEAS_POLLING_L || mode==CMD_MEAS_POLLING_M)
        error = SHT_Write(mode, NO_END);
    else
        error = PARA_ERROR;
    if(error==NO_ERROR){
        while(timeout--)
        {
          error = SHT_Read(buffer,1,6,END);
          if(error == NO_ERROR) break;
          //wait(100);
        }
      }
    if(error==NO_ERROR){
        dataSHT=(Raw_SHT_Data*)buffer;
        error = SHT_CRC_check(dataSHT->CRC_temp,dataSHT->tempval,2);
        error |= SHT_CRC_check(dataSHT->CRC_humid,dataSHT->humidval,2);
    }
    if(error==NO_ERROR){
        *temp=SHT_Temp_Cal(((dataSHT->tempval[0]<<8)|dataSHT->tempval[1]));
        *humid=SHT_Humid_Cal(((dataSHT->humidval[0]<<8)|dataSHT->humidval[1]));
    }

    return error;
}
//--------------------SHT CalcCrc--------------------//
int SHT_CalcCrc(uint8_t* data, uint8_t nbrOfBytes)
{
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
    {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else           crc = (crc << 1);
        }
    }

    return crc;
}
//--------------------SHT Soft Reset--------------------//
SHT_Error SHT_Soft_Reset(void){
    SHT_Error error=NO_ERROR;
    error = SHT_Write(CMD_SOFT_RESET, END);
    if(error==NO_ERROR) wait(50);
    return error;
}
SHT_Error SHT_Read_Status(uint16_t* status){
    uint8_t buffer[3];
    Raw_2B_Data *dataSHT;
    SHT_Error error=NO_ERROR;
    error = SHT_Write(CMD_READ_STATUS, NO_END);
    if(error==NO_ERROR)
        error = SHT_Read(buffer,50,3,END);
    if(error==NO_ERROR){
        dataSHT=(Raw_2B_Data *)buffer;
        error = SHT_CRC_check(dataSHT->CRC_val,dataSHT->any_val,2);
    }
    if(error==NO_ERROR){
        *status=(dataSHT->any_val[0]<<8)|dataSHT->any_val[1];
    }
    return error;
}
//--------------------SHT Clar Flags--------------------//
SHT_Error SHT_Clear_Flags(void){
    SHT_Error error=NO_ERROR;
    error = SHT_Write(CMD_CLEAR_STATUS,END);
    return error;
}
//--------------------SHT Read Serial--------------------//
SHT_Error SHT_Read_Serial(uint32_t* status){
    uint8_t buffer[6];
    uint32_t SerialNum[2];
    Raw_SHT_Data *dataSHT;
    SHT_Error error=NO_ERROR;
        error = SHT_Write(CMD_READ_SERIALNBR, NO_END);
    if(error==NO_ERROR)
        error = SHT_Read(buffer,100,6,END);
    if(error==NO_ERROR){
        dataSHT=(Raw_SHT_Data*)buffer;
        error = SHT_CRC_check(dataSHT->CRC_temp,dataSHT->tempval,2);
        error |= SHT_CRC_check(dataSHT->CRC_humid,dataSHT->humidval,2);
    }
    if(error==NO_ERROR){
        SerialNum[0]=(dataSHT->tempval[0]<<8)|dataSHT->tempval[1];
        SerialNum[1]=(dataSHT->humidval[0]<<8)|dataSHT->humidval[1];
        *status=(SerialNum[0]<<16)|SerialNum[1];
    }
    return error;
}
//--------------------SHT Start Peroid--------------------//
SHT_Error SHT_Start_Period(SHT_Period_Mode mode){
    SHT_Error error=NO_ERROR;
    if(mode==CMD_MEAS_PERI_05_H || mode==CMD_MEAS_PERI_05_M || mode==CMD_MEAS_PERI_05_L)
        error = SHT_Write(mode, END);
    else if(mode==CMD_MEAS_PERI_1_H || mode==CMD_MEAS_PERI_1_M || mode==CMD_MEAS_PERI_1_L)
        error = SHT_Write(mode, END);
    else if(mode==CMD_MEAS_PERI_2_H || mode==CMD_MEAS_PERI_2_M || mode==CMD_MEAS_PERI_2_L)
        error = SHT_Write(mode, END);
    else if(mode==CMD_MEAS_PERI_4_H || mode==CMD_MEAS_PERI_4_M || mode==CMD_MEAS_PERI_4_L)
        error = SHT_Write(mode, END);
    else if(mode==CMD_MEAS_PERI_10_H || mode==CMD_MEAS_PERI_10_M || mode==CMD_MEAS_PERI_10_L)
        error = SHT_Write(mode, END);
    else
        error = PARA_ERROR;
    return error;
}
//--------------------SHT Read Peroid--------------------//
SHT_Error SHT_Read_Period(float *temp,float *humid){
    uint8_t buffer[6];
    Raw_SHT_Data *dataSHT;
    SHT_Error error=NO_ERROR;
        error = SHT_Write(CMD_FETCH_DATA, NO_END);
    if(error==NO_ERROR)
          error = SHT_Read(buffer,1,6,END);
    if(error==NO_ERROR){
        dataSHT=(Raw_SHT_Data*)buffer;
        error = SHT_CRC_check(dataSHT->CRC_temp,dataSHT->tempval,2);
        error |= SHT_CRC_check(dataSHT->CRC_humid,dataSHT->humidval,2);
    }
    if(error==NO_ERROR){
        *temp=SHT_Temp_Cal(((dataSHT->tempval[0]<<8)|dataSHT->tempval[1]));
        *humid=SHT_Humid_Cal(((dataSHT->humidval[0]<<8)|dataSHT->humidval[1]));
    }

    return error;
}
//---------------------SHT Get Alert Limit---------------------//
SHT_Error SHT_Get_Alert_Limit(float* humidityHighSet,   float* temperatureHighSet,
                             float* humidityHighClear, float* temperatureHighClear,
                             float* humidityLowClear,  float* temperatureLowClear,
                             float* humidityLowSet,    float* temperatureLowSet)
{
  SHT_Error  error = NO_ERROR;  // error code

  // read humidity & temperature alter limits, high set
  if(error == NO_ERROR) error = SHT_Write(CMD_R_AL_LIM_HS,NO_END);
  if(error == NO_ERROR) error = SHT_Read_Alert_Limit(humidityHighSet,
                                                         temperatureHighSet);

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, high clear
    if(error == NO_ERROR) error = SHT_Write(CMD_R_AL_LIM_HC,NO_END);
    if(error == NO_ERROR) error = SHT_Read_Alert_Limit(humidityHighClear,
                                                           temperatureHighClear);
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low clear
    if(error == NO_ERROR) error = SHT_Write(CMD_R_AL_LIM_LC,NO_END);
    if(error == NO_ERROR) error = SHT_Read_Alert_Limit(humidityLowClear,
                                                           temperatureLowClear);
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low set
    if(error == NO_ERROR) error = SHT_Write(CMD_R_AL_LIM_LS,NO_END);
    if(error == NO_ERROR) error = SHT_Read_Alert_Limit(humidityLowSet,
                                                           temperatureLowSet);
  }

  return error;
}
//---------------------SHT Set Alert Limit---------------------//
SHT_Error SHT_Set_Alert_Limit(float humidityHighSet,   float temperatureHighSet,
                             float humidityHighClear, float temperatureHighClear,
                             float humidityLowClear,  float temperatureLowClear,
                             float humidityLowSet,    float temperatureLowSet)
{
  SHT_Error  error = NO_ERROR;  // error code

  // write humidity & temperature alter limits, high set

  if(error == NO_ERROR) error = SHT_Write_Alert_Limit(CMD_W_AL_LIM_HS,humidityHighSet,
                                                          temperatureHighSet);


  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, high clear
    if(error == NO_ERROR) error = SHT_Write_Alert_Limit(CMD_W_AL_LIM_HC,humidityHighClear,
                                                            temperatureHighClear);
  }

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low clear
    if(error == NO_ERROR) error = SHT_Write_Alert_Limit(CMD_W_AL_LIM_LC,humidityLowClear,
                                                            temperatureLowClear);
  }

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low set
    if(error == NO_ERROR) error = SHT_Write_Alert_Limit(CMD_W_AL_LIM_LS,humidityLowSet,
                                                            temperatureLowSet);

  }

  return error;
}
//---------------------SHT Write Alert Limit---------------------//
SHT_Error SHT_Write_Alert_Limit(uint16_t command,float humid, float temp)
{
  SHT_Error  error = NO_ERROR;
  uint16_t rawdata;
  if((humid < 0.0f) || (humid > 100.0f)
  || (temp < -45.0f) || (temp > 130.0f))
  {
    error = PARA_ERROR;
  }
  else
  {
    rawdata = (SHT_Calc_Raw_Humid(humid) & 0xFE00) | ((SHT_Calc_Raw_Temp(temp)>> 7) & 0x001FF);
    error = SHT_Write_Command_Data(command,rawdata,END);
  }

  return error;
}
//---------------------SHT Read Alert Limit---------------------//
SHT_Error SHT_Read_Alert_Limit(float* humid, float* temp)
{
  SHT_Error  error = NO_ERROR;           // error code
  uint8_t buffer[3];
  uint16_t value;
  Raw_2B_Data *data;
  error = SHT_Read(buffer,1,3,END);
  if(error == NO_ERROR){
      data=(Raw_2B_Data *)buffer;
      error=SHT_CRC_check(data->CRC_val,data->any_val,2);
  }
  if(error == NO_ERROR)
  {
      value=(data->any_val[0]<<8)|data->any_val[1];
    *humid = SHT_Humid_Cal(value & 0xFE00);
    *temp = SHT_Temp_Cal(value << 7);
  }

  return error;
}
//--------------------SHT Calc Raw Temp-------------------//
uint16_t SHT_Calc_Raw_Temp(float temp)
{
  // calculate raw temperature [ticks]
  // rawT = (temperature + 45) / 175 * (2^16-1)
  return (temp + 45.0f) / 175.0f * 65535.0f;
}

//--------------------SHT Calc Raw Humid-------------------//
uint16_t SHT_Calc_Raw_Humid(float humid)
{
  // calculate raw relative humidity [ticks]
  // rawRH = humidity / 100 * (2^16-1)
  return humid / 100.0f * 65535.0f;
}
//--------------------SHT Enable Heater--------------------//
SHT_Error SHT_Enable_Heater(void){
    SHT_Error error=NO_ERROR;
    error = SHT_Write(CMD_HEATER_ENABLE,END);
    return error;
}
//--------------------SHT Disable Heater--------------------//
SHT_Error SHT_Disable_Heater(void){
    SHT_Error error=NO_ERROR;
    error = SHT_Write(CMD_HEATER_DISABLE,END);
    return error;
}
//--------------------SHT Temp Cal--------------------//
float SHT_Temp_Cal(uint16_t temp){
    float val=(float)temp;
    val=175*(val/0xFFFF)-45;
    return val;
}
//--------------------SHT Humid Cal--------------------//
float SHT_Humid_Cal(uint16_t humid){
    float val=(float)humid;
    val=100*val/0xFFFF;
    return val;
}
//--------------------wait--------------------//
void wait(long ms){
    ms*=multitime;
    while(ms--);
}
