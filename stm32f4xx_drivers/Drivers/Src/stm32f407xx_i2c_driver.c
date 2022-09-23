/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 16 de set de 2022
 *      Author: joaobortotcadore
 */

#include "stm32f407xx_i2c_driver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

/* Private functions - set this functions as static */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

uint32_t RCC_GetPLLOutputClock(void)
{
	//this function not be used in the course, lesson 189 ~5:03 min
	return 0;
}

/**
 * @fn uint32_t RCC_GetPCLK1Value(void)
 * @brief
 *
 * @return
 */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SystemClk;

    uint8_t clksrc, temp, ahbp,apb1p;

    clksrc = ((RCC->CFGR >> 2) & 0x3); //7.3.3 RCC clock configuration register (RCC_CFGR) - SWS use 2 bits, 0x3 is mask

    if(clksrc == 0 ) // HSI
    {
        SystemClk = 16000000;
    }else if(clksrc == 1) // HSE
    {
        SystemClk = 8000000;
    }else if (clksrc == 2) // PLL
    {
        SystemClk = RCC_GetPLLOutputClock(); //we dont use PLL in the course, lesson 189 ~5:03 min
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

/**
 * @fn void I2C_Init(I2C_Handle_t*)
 * @brief
 *
 * @param pI2CHandle
 */
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
    	// not selected Sm mode because it is default
        ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
        tempreg |= (ccr_value & 0xFFF);
    }else
    {
        //mode is fast mode
        tempreg |= ( 1 << 15); // to select Fm mode
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

    //TRISE Configuration - Section 57, lecture 199
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
/**
 * @fn void I2C_DeInit(I2C_RegDef_t*)
 * @brief
 *
 * @param pI2Cx
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * @fn void I2C_MasterSendData(I2C_Handle_t*, uint8_t*, uint32_t, uint8_t, uint8_t)
 * @brief
 *
 * @param pI2CHandle
 * @param pTxbuffer
 * @param Len
 * @param SlaveAddr
 * @param Sr
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

    //1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    //2. confirm that start generation is completed by checking the SB flag in the SR1
    //   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

    //3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

    //4. wait until address phase is completed by checking the ADDR flag in teh SR1
    while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

    //procedure to read only 1 byte from slave
    if(Len == 1)
    {
        //Disable Acking
        I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

        //clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        //wait until  RXNE becomes 1
        while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

        //generate STOP condition - section 60, lesson 210
        if(Sr == I2C_DISABLE_SR )
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        //read data in to buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

    }

    //procedure to read data from slave when Len > 1
    if(Len > 1)
    {
        //clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        //read the data until Len becomes zero
        for ( uint32_t i = Len ; i > 0 ; i--)
        {
            //wait until RXNE becomes 1
            while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

            if(i == 2) //if last 2 bytes are remaining
            {
                //Disable Acking
                I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

                //generate STOP condition
                if(Sr == I2C_DISABLE_SR )
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            //read the data from data register in to buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            //increment the buffer address
            pRxBuffer++;
        }

    }

    //re-enable ACKing
    if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
    }

}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == I2C_ACK_ENABLE)
    {
        //enable the ack
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
    }else
    {
        //disable the ack
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
    }
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
        //pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
    }else
    {
        pI2Cx->CR1 &= ~(1 << 0);
    }

}

/**
 * @fn uint8_t I2C_GetFlagStatus(I2C_RegDef_t*, uint32_t)
 * @brief
 *
 * @param pI2Cx
 * @param FlagName
 * @return
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* IMPLEMENTATION OF STATIC / PRIVATE FUNCTIONS */
/**
 * @fn void I2C_GenerateStartCondition(I2C_RegDef_t*)
 * @brief helper function to i2c
 *
 * @param pI2Cx
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

/**
 * @fn void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t*, uint8_t)
 * @brief
 *
 * @param pI2Cx
 * @param SlaveAddr
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //moving to give space for read/write bit
	SlaveAddr &= ~(1); //SlaveAddr is slave address + r/w bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1; //moving to give space for read/write bit
    SlaveAddr |= 1; //SlaveAddr is slave address + r/w bit=1
    pI2Cx->DR = SlaveAddr;
}

/**
 * @fn void I2C_ClearADDRFlag(I2C_RegDef_t)
 * @brief
 *
 * @param pI2CHandle
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}
