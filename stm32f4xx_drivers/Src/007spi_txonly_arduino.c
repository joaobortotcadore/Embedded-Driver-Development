/*
 * 007spi_tx_testing.c
 *
 *  Created on: 21 de ago de 2022
 *      Author: joaobortotcadore
 */

#include "stm32f407xx.h"
#include "string.h"

/* Test the SPI_SendData API to send the string "Hello world" and use the below configurations
 * 1. SPI-2 Master mode
 * 2. SCLK = MAX possible
 * 3. DFF=0 and DFF=1
*/

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
*/

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
//	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

/**
 * @fn void GPIO_ButtonInit(void)
 * @brief initialization Gpio Button
 */
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

/* simple delay by software */
void delay(void)
{
    for(uint32_t i =0; i< 500000/2; i++);
}


int main(void)
{
	char user_data[] = "Hello world";

	//initialize the GPIO pin Button
	GPIO_ButtonInit();

	//initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/* making SSOE=1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware
	 * When SPE=1, NSS will be pulled to LOW
	 * When SPE=0, NSS will be pulled to HIGH
	 * */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//to send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,dataLen,1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//confirm first if SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) ); //if SPI is busy, macro return 1 and get stuck in the loop

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
