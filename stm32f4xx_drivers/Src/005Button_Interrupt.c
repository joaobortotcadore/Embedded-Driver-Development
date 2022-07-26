/*
 * 002Led_Button.c
 *
 *  Created on: 19 de jul de 2022
 *      Author: joaobortotcadore
 */

#include "stm32f407xx.h"
#include "string.h" //for memset

#define HIGH 			ENABLE
#define BTN_PRESSED 	HIGH

/* simple delay by software */
void delay(void)
{
    for(uint32_t i =0; i< 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0, sizeof(GpioLed));
	memset(&GpioBtn,0, sizeof(GpioBtn));
	/* initialization GpioLed */
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);


	/* initialization GpioBtn */
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler(void)
{
    delay(); //200ms - wait untill button de-bouncing gets over
    GPIO_IRQHandling(GPIO_PIN_NO_5); //clear pending event from EXTI line
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
