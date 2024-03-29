/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 16 de set de 2022
 *      Author: joaobortotcadore

 *      >>>>>>>>>>>>>>>>>>>>>>>> Common problems in I2C <<<<<<<<<<<<<<<<<<<<<<<<
 *      -----------------------------------------------------------------------
 *      Problem-1: SDA and SCL line not held HIGH
 *      Voltage after 12C pin initialization
 *
 *      Reason-1
 *      Not activating the pullup resistors if you are using the internal
 *      pull up resistor of an I/O line
 *
 *      Debug Tip:
 *      worth checking the configuration register of arn 1/O line to see
 *      whether the pullups are really activated or no, best way is to
 *      dump the register contents.
 *      -----------------------------------------------------------------------
 *      Problem-2: ACK failure
 *
 *      Reason-1
 *      Generating the address phase with wrong slave address
 *
 *      Debug Tip:
 *      verify the slave address appearing on the SDA line by using
 *      logic analyser
 *
 *      Reason-2:
 *      Not enabling the ACKing feature in the 12C control register
 *
 *      Debug Tip:
 *      Cross check the 120C Control register ACK enable field
 *      -----------------------------------------------------------------------
 *      Problem-3: Master is not producing the clock
 *
 *      Debug Tip 1:
 *      First Check whether 12C peripheral clock is enabled and set to
 *      at least 2mhz to produce standard mode i2c serial clock
 *      frequency
 *
 *      Debug Tip 2:
 *      Check whether GPIOs which you used for SCL and SDA
 *      functionality are configured properly for the alternate functionality
 *      -----------------------------------------------------------------------
 */

#include "stm32f407xx_i2c_driver.h"
//uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
//uint16_t APB1_PreScaler[4] = {2,4,8,16};

/* Private functions - set this functions as static */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle); //section 63, lecture 224
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle); //section 63, lecture 224
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

//uint32_t RCC_GetPLLOutputClock(void)
//{
//	//this function not be used in the course, lesson 189 ~5:03 min
//	return 0;
//}
//
///**
// * @fn uint32_t RCC_GetPCLK1Value(void)
// * @brief
// *
// * @return
// */
//uint32_t RCC_GetPCLK1Value(void)
//{
//    uint32_t pclk1, SystemClk;
//
//    uint8_t clksrc, temp, ahbp,apb1p;
//
//    clksrc = ((RCC->CFGR >> 2) & 0x3); //7.3.3 RCC clock configuration register (RCC_CFGR) - SWS use 2 bits, 0x3 is mask
//
//    if(clksrc == 0 ) // HSI
//    {
//        SystemClk = 16000000;
//    }else if(clksrc == 1) // HSE
//    {
//        SystemClk = 8000000;
//    }else if (clksrc == 2) // PLL
//    {
//        SystemClk = RCC_GetPLLOutputClock(); //we dont use PLL in the course, lesson 189 ~5:03 min
//    }
//
//    //for AHB
//    temp = ((RCC->CFGR >> 4 ) & 0xF); //HPRE use 4 bits, 0xF is mask
//
//    if(temp < 8)
//    {
//        ahbp = 1;
//    }else
//    {
//        ahbp = AHB_PreScaler[temp-8];
//    }
//
//    //apb1
//    temp = ((RCC->CFGR >> 10 ) & 0x7); //HPRE use 3 bits, 0x7 is mask
//
//    if(temp < 4)
//    {
//        apb1p = 1;
//    }else
//    {
//        apb1p = APB1_PreScaler[temp-4];
//    }
//
//    pclk1 =  (SystemClk / ahbp) /apb1p;
//
//    return pclk1;
//}

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
    tempreg = 0; // to avoid the master receive NACK in the adress phase - section 66, lecture 235
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
	I2C_ClearADDRFlag(pI2CHandle);

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
        I2C_ClearADDRFlag(pI2CHandle);

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
        I2C_ClearADDRFlag(pI2CHandle); //modified on section 63, lecture 225

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

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)  //section 62, lecture 218
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

    }

    return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr) //section 62, lecture 218
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    //Interrupt handling for both master and slave mode of a device

    uint32_t temp1, temp2, temp3;

    temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
    temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
    //1. Handle For interrupt generated by SB event
    //  Note : SB flag is only applicable in Master mode
    if(temp1 && temp3)
    {
        //The interrupt is generated because of SB event
        //This block will not be executed in slave mode because for slave SB is always zero
        //In this block lets executed the address phase
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
        }else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
        }
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
    //2. Handle For interrupt generated by ADDR event
    //Note : When master mode : Address is sent
    //       When Slave mode   : Address matched with own address
    if(temp1 && temp3)
    {
        // interrupt is generated because of ADDR event
        I2C_ClearADDRFlag(pI2CHandle);
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
    //3. Handle For interrupt generated by BTF(Byte Transfer Finished) event //section 63, lecture 221
    if(temp1 && temp3)
    {
        //BTF flag is set
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            //make sure that TXE is also set .
            if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
            {
                //BTF, TXE = 1
                if(pI2CHandle->TxLen == 0 )
                {
                    //1. generate the STOP condition
                    if(pI2CHandle->Sr == I2C_DISABLE_SR)
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

                    //2. reset all the member elements of the handle structure.
                    I2C_CloseSendData(pI2CHandle);

                    //3. notify the application about transmission complete
                    I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

                }
            }

        }else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
        {
            ; //NOTHING TO DO
        }
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
    //4. Handle For interrupt generated by STOPF event //section 63, lecture 222
    // Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
    //The below code block will not be executed by the master since STOPF will not set in master mode
    if(temp1 && temp3)
    {
        //STOF flag is set
        //Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        //Notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
    }


    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
    //5. Handle For interrupt generated by TXE event //section 63, lecture 223
    if(temp1 && temp2 && temp3)
    {
        //Check for device mode
        if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
        {
            //TXE flag is set
            //We have to do the data transmission
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }else
        {
            //slave
            //make sure that the slave is really in transmitter mode
            if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
            }
        }
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
    //6. Handle For interrupt generated by RXNE event
    if(temp1 && temp2 && temp3)
    {
        //check device mode .
        if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
        {
            //The device is master

            //RXNE flag is set
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }else
        {
            //slave
            //make sure that the slave is really in receiver mode
            if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) // section 65, lesson 232
{

    uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
    temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/* Check for Bus error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
    if(temp1  && temp2 )
    {
        //This is Bus error

        //Implement the code to clear the buss error flag
        pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

        //Implement the code to notify the application about the error
       I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

/* Check for arbitration lost error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
    if(temp1  && temp2)
    {
        //This is arbitration lost error

        //Implement the code to clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

    }

/* Check for ACK failure  error */

    temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
    if(temp1  && temp2)
    {
        //This is ACK failure error

        //Implement the code to clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
    }

/* Check for Overrun/underrun error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
    if(temp1  && temp2)
    {
        //This is Overrun/underrun

        //Implement the code to clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
    }

/* Check for Time out error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
    if(temp1  && temp2)
    {
        //This is Time out error

        //Implement the code to clear the Time out error flag
        pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
    }

}

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

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) //section 63, lecture 226
{
    //Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
    }

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) //section 63, lecture 226
{
    //Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
    pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C) //section 66, lesson 235
{
    return (uint8_t) pI2C->DR;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) //section 66, lesson 235
{
    pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi) //section 66, lecture 234
{
    if(EnorDi == ENABLE)
    {
           pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
           pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
           pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
    }else
    {
           pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
           pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
           pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
    }
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

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) //section 63, lecture 224
{

    if(pI2CHandle->TxLen > 0)
    {
        //1. load the data in to DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        //2. decrement the TxLen
        pI2CHandle->TxLen--;

        //3. Increment the buffer address
        pI2CHandle->pTxBuffer++;

    }

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) //section 63, lecture 224
{
    //We have to do the data reception
    if(pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;

    }


    if(pI2CHandle->RxSize > 1)
    {
        if(pI2CHandle->RxLen == 2)
        {
            //clear the ack bit
            I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
        }

            //read DR
            *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
            pI2CHandle->pRxBuffer++;
            pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0 )
    {
        //close the I2C data reception and notify the application

        //1. generate the stop condition
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        //2 . Close the I2C rx
        I2C_CloseReceiveData(pI2CHandle);

        //3. Notify the application
        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
    }
}

/**
 * @fn void I2C_ClearADDRFlag(I2C_Handle_t)
 * @brief
 *
 * @param pI2CHandle
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) //UPDATE THIS FUNCTION AT section 63, lecture 224
{
    uint32_t dummy_read;
    //check for device mode
    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
    {
        //device is in master mode
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RxSize  == 1)
            {
                //first disable the ack
                I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

                //clear the ADDR flag ( read SR1 , read SR2)
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read; //to prevent usuned warning
            }

        }
        else
        {
            //clear the ADDR flag ( read SR1 , read SR2)
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read; //to prevent usuned warning

        }

    }
    else
    {
        //device is in slave mode
        //clear the ADDR flag ( read SR1 , read SR2)
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read; //to prevent usuned warning
    }


}
