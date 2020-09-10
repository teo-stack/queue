#include <SHTx.h>
int SHT_address_store;float temp_store,humid_store;
void SHT_init(SHT_address address){
    SHT_address_store=address;
    I2C1_Master_Config(FASTMODE);
}
int SHT_Reset(void){
    I2C1_Deconfig();
    wait(1000);
    //I2C1->ISR=0;
    SHT_init(SHT_address_store);
    wait(1000);
    return SHT_Soft_Reset();
}
int SHT_Write(SHT_command command, FuncState state){
    int error=0;
    char buffer[2];
    buffer[0]=(command>>8) & 0xFF;
    buffer[1]=command & 0xFF;
    error = I2C_write_test(I2C1,SHT_address_store,buffer,2,state);
    return error;
}
int SHT_Read(char* buffer,long timeout,int nbr,FuncState state){
    int error=0;
    error = I2C_read_test(I2C1,SHT_address_store,buffer,nbr,timeout,state);
    return error;
}
int SHT_CRC_check(int CRCval,char* data, char nbrOfBytes){
    int CRC_check=0;
    int error=0;
    CRC_check=SHT_CalcCrc(data,nbrOfBytes);
    if(CRC_check!=CRCval) error=1;
    return error;
}
int SHT_Read_Raw(float *temp,float *humid){
    char buffer[6];
    Raw_SHT_Data *dataSHT;
    int error=0;
    error = SHT_Write(SHT_HIGHREPEAT, NO_END);
    if(error==0)
    error = SHT_Read(buffer,50,6,END);
    if(error==0){
    dataSHT=(Raw_SHT_Data*)buffer;
    error = SHT_CRC_check(dataSHT->CRC_temp,dataSHT->tempval,2);
    error += SHT_CRC_check(dataSHT->CRC_humid,dataSHT->humidval,2);
    if(error==0){
      *temp=(float)((dataSHT->tempval[0]<<8)|dataSHT->tempval[1]);
      *temp=175*(*temp/0xFFFF)-45;
        *humid=(float)((dataSHT->humidval[0]<<8)|dataSHT->humidval[1]);
       *humid=100*(*humid)/0xFFFF;}
}

    return error;
}
float SHT_Temp(){
    return temp_store;
}
float SHT_Humid(){
    return humid_store;
}
int SHT_CalcCrc(char* data, char nbrOfBytes)
{
  char bit;        // bit mask
  char crc = 0xFF; // calculated checksum
  char byteCtr;    // byte counter

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
int SHT_Soft_Reset(void){
    int error=0;
    error = SHT_Write(CMD_SOFT_RESET, END);
    if(error==0) wait(50);
    return error;
}
void wait(long ms){
    ms*=multitime;
    while(ms--);
}
