#include <I2C_STM32F0.h>

//-----------------------I2C_write-------------------------//
uint8_t I2C_write(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num_byte,long timeout,FuncState state){
    if(timeout==0){
        return FAIL;
    }
    else{
        long timecount=timeout*multitime;
        I2C_TransferHandling(I2Cx,address<<1,num_byte,I2C_SoftEnd_Mode,I2C_Generate_Start_Write);
        while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXIS))&&(--timecount));
        if(timecount==0) {
            return 2;}
        else{
            for(int i=0;i<num_byte;i++){
                I2C_SendData(I2Cx,*buffer++);
                while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE));
            }
            while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TC));
            I2C_GenerateSTOP(I2Cx, state);
            while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));
            return SUCCESS;
        }
    }
}

//-----------------------I2C_write_test-------------------------//
uint8_t I2C_write_test(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num_byte,FuncState state){
    uint8_t error=0;
    I2C_TransferHandling(I2Cx,address<<1,num_byte,I2C_SoftEnd_Mode,I2C_Generate_Start_Write);
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXIS))&&(!I2C_GetFlagStatus(I2Cx,I2C_FLAG_NACKF)));
    if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_NACKF)) error=1;
    else{
        for(int i=0;i<num_byte;i++){
            I2C_SendData(I2Cx,*buffer++);
            while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)));
            if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_NACKF)) {
                error=i+2;
                break;
            }
        }
        if (I2C_GetFlagStatus(I2Cx,I2C_FLAG_TC) && error==0) I2C_GenerateSTOP(I2Cx, state);
        //while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));
    }
    return error;
}

//-----------------------I2C_read-------------------------//
uint8_t I2C_read(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num_byte,long timeout,FuncState state){
    if(timeout==0){
        return FAIL;
    }
    else{
        long timecount=timeout*multitime;
        I2C_TransferHandling(I2Cx,address<<1,num_byte,I2C_SoftEnd_Mode,I2C_Generate_Start_Read);
        for(int i=0;i<num_byte;i++){
            timecount=timeout*multitime;
            while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_RXNE))&&(--timecount));
            if(timecount==0) break;
            *buffer++=I2C_ReceiveData(I2Cx);
        }
        if(timecount==0) {
            return 2;}
        else{
            I2C_GenerateSTOP(I2Cx, state);
            while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));
            return SUCCESS;
        }
    }

}
//-----------------------I2C_read_test-------------------------//
uint8_t I2C_read_test(I2C_TypeDef* I2Cx,uint8_t address,uint8_t* buffer,uint8_t num_byte,long timeout,FuncState state){

    uint8_t error=0;
    long timecount;
    I2C_TransferHandling(I2Cx,address<<1,num_byte,I2C_SoftEnd_Mode,I2C_Generate_Start_Read);
    for(int i=0;i<num_byte;i++){
        timecount=timeout*multitime;
        while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_RXNE))&&(--timecount));
        if(I2C_GetFlagStatus(I2Cx,I2C_FLAG_NACKF)) {
            I2C_ClearFlag(I2Cx,I2C_FLAG_NACKF);
            return 1;}
        if(timecount==0) {error=i+2;break;}
        *buffer++=I2C_ReceiveData(I2Cx);
    }
    if(error==0) {I2C_GenerateSTOP(I2Cx, state);}
    //while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));

    return error;
}
//-----------------------I2C_scan-------------------------//
uint8_t I2C_scan(I2C_TypeDef* I2Cx){
    uint8_t address,check=0,buf=0;
    for(address=0;address<128;address++){
        check=I2C_read(I2Cx,address,&buf,1,1,END);
        if(check) {return address;break;}
    }
    if(!check)  return 0xFF;
}

//-----------------------I21_Master_Config-------------------------//
void I2C1_Master_Config(SpeedMode mode){

    // cap clock cho ngoai vi va I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);							// su dung kenh I2C2 cua STM32
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
    // cau hinh chan SDA va SCL
    GPIOlib_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;						//PA9 - SCL, PA10 - SDA
    GPIOlib_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOlib_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIOlib_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIOlib_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIOlib_InitStructure);

    // cau hinh I2C
    I2Clib_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2Clib_InitStructure.I2C_DigitalFilter = 0x00;
    I2Clib_InitStructure.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2Clib_InitStructure.I2C_OwnAddress1 = 0; //
    I2Clib_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2Clib_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2Clib_InitStructure.I2C_Timing = mode;//0x10310309
    I2C_Init(I2C1, &I2Clib_InitStructure);
    // cho phep bo I2C hoat dong
    I2C_Cmd(I2C1, ENABLE);

}
//----------------------I2C1_Deconfig-------------------//
void I2C1_Reconfig(void){

    I2C_SoftwareResetCmd(I2C1);

}
