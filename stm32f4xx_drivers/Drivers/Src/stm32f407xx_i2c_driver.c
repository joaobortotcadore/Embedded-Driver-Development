/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 16 de set de 2022
 *      Author: joaobortotcadore
 */

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SystemClk;

    uint8_t clksrc, temp;

    clksrc = ((RCC->CFGR >> 2) & 0x3); //7.3.3 RCC clock configuration register (RCC_CFGR) - SWS use 2 bits, 0x3 is mask

    if(clksrc == 0 ) // HSI
    {
        SystemClk = 16000000;
    }else if(clksrc == 1) // HSE
    {
        SystemClk = 8000000;
    }else if (clksrc == 2) // PLL
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    //for AHB
    temp = ((RCC->CFGR >> 4 ) & 0xF); //HPRE use 4 bits, 0xF is mask

    if(temp < 8)
    {
        ahbp = 1;
    }else
    {
        ahbp = AHB_PreScaler[temp-8];
    }

    //apb1
    temp = ((RCC->CFGR >> 10 ) & 0x7); //HPRE use 3 bits, 0x7 is mask

    if(temp < 4)
    {
        apb1p = 1;
    }else
    {
        apb1p = APB1_PreScaler[temp-4];
    }

    pclk1 =  (SystemClk / ahbp) /apb1p;

    return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0 ;

//    //enable the clock for the i2cx peripheral
//    I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

    //ack control bit
    tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    //configure the FREQ field of CR2
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() /1000000U ;
    pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= ( 1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    //CCR calculations
    uint16_t ccr_value = 0;
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        //mode is standard mode
        ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
        tempreg |= (ccr_value & 0xFFF);
    }else
    {
        //mode is fast mode
        tempreg |= ( 1 << 15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
        }else
        {
            ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
        }
        tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;

    //TRISE Configuration
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        //mode is standard mode

        tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

    }else
    {
        //mode is fast mode
        tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

    }

    pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

