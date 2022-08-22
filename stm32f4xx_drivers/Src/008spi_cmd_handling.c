/*
 * 008spi_cmd_handling.c
 *
 *  Created on: 21 de ago de 2022
 *      Author: joaobortotcadore
 */

#include "stm32f407xx.h"
#include "string.h"

//command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

//arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

//arduino led
#define LED_PIN					9

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

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
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
	GPIO_Handle_t GpioLed, GpioBtn;

	/* initialization GpioBtn */
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);

	/* initialization GpioLed */
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);
}

/* simple delay by software */
void delay(void)
{
    for(uint32_t i =0; i< 500000/2; i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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
		//wait untill button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		///1. CMD_LED_CTRL 	<pin no(1)> 	<value(1)>///
		uint8_t cmdCode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		SPI_SendData(SPI2,&cmdCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte) )
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args,2);
		}
		///end of COMMAND_LED_CTRL///

		///1. CMD_SENSOR_READ 	<analog pin number(1)>///
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//change command code byte and send command
		cmdCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2,&cmdCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte) )
		{
			args[0] = ANALOG_PIN0;
			//send arguments
			SPI_SendData(SPI2,args,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data from analog read
			delay();

			//send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}
		///end of CMD_SENSOR_READ///

		//confirm first if SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) ); //if SPI is busy, macro return 1 and get stuck in the loop

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
