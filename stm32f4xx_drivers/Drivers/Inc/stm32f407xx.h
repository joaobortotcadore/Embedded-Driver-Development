/*
 * stm32f407xx.h
 *
 *  Created on: May 3, 2022
 *      Author: joao.cadore
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h> //for uint32_t typedef

/* Volatile is used to inform the compiler that the variable value can be
 * changed any time without any task given by the source code. Volatile is
 * usually applied to a variable when we are declaring it. */
#define __vo volatile

/****************** START: Processor Specific Details *******************/

/* ARM Cortex Mx Processor NVIC ISERx register Addresses */
#define NVIC_ISER0		( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1		( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2		( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3		( (__vo uint32_t*)0xE000E10C )

/* ARM Cortex Mx Processor NVIC ICERx register Addresses */
#define NVIC_ICER0		( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1		( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2		( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3		( (__vo uint32_t*)0xE000E18C )

/* ARM Cortex Mx Processor Priority Register Address Calculation */
#define NVIC_PR_BASE_ADDR   ( (__vo uint32_t*)0xE000e400 )

#define NO_PR_BITS_IMPLEMENTED  4

/* Base address if FLASH and SRAM memories */
#define FLASH_BASE              0x08000000U
#define SRAM1_BASE              0x20000000U //112KB => 112*1024 = 114688 <==> 1C000
#define SRAM2_BASE              0x2001C000U
#define ROM                     0x1FFF0000U
#define SRAM                    SRAM1_BASEADDR

/* AHBx and APBx Bus Peripheral base address */
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         0x40010000U
#define AHB1PERIPH_BASE         0x40020000U
#define AHB2PERIPH_BASE         0x50000000U

/* Base addresses if peripherals which are hanging on AHB1 bus */
#define GPIOA_BASE              (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE              (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE              (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE              (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE              (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE              (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE              (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE              (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE              (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE              (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE              (AHB1PERIPH_BASE + 0x2800)
#define RCC_BASE                (AHB1PERIPH_BASE + 0x3800)

/* Base addresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASE              (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE              (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE              (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASE              (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE              (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASE            (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE            (APB1PERIPH_BASE + 0x4800)

#define UART4_BASE             (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE             (APB1PERIPH_BASE + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 bus */
#define SPI1_BASE              (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE            (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE            (APB2PERIPH_BASE + 0x1400)
#define EXTI_BASE              (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE            (APB2PERIPH_BASE + 0x3800)

/**************peripheral register definition structures**************/
/* Note: Registers of a peripheral are specific to MCU
 * e.g: Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;      /*!< GPIO port mode register     Address offset: 0x00*/
	__vo uint32_t OTYPER;    /*!< GPIO port output type register     Address offset: 0x04*/
	__vo uint32_t OSPEEDR;   /*!< GPIO port output speed register     Address offset: 0x08*/
    __vo uint32_t PUPDR;     /*!< GPIO port pull-up/pull-down register     Address offset: 0x0C*/
    __vo uint32_t IDR;       /*!< GPIO port input data register     Address offset: 0x10*/
    __vo uint32_t ODR;       /*!< GPIO port output data register     Address offset: 0x14*/
    __vo uint32_t BSRR;       /*!< GPIO port bit set/reset register     Address offset: 0x18*/
    __vo uint32_t LCKR;      /*!< GPIO port configuration lock register     Address offset: 0x1C*/
    __vo uint32_t AFR[2];    /*!< GPIO alternate function <low register[0] / high register[1]>     Address offset: L-0x20 / H-0x24*/
}GPIO_RegDef_t;

/**
 * @struct RCC_RegDef_t
 * @brief peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    __vo uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    __vo uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    __vo uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    __vo uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    __vo uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    __vo uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    __vo uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATENR;
    __vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/**
 * @struct EXTI_RegDef_t
 * @brief peripheral register definition structure for EXTI.
 * Section 28, lesson 109
 */
typedef struct
{
	__vo uint32_t IMR;      /*!< GPIO port mode register     Address offset: 0x00*/
	__vo uint32_t EMR;    /*!< GPIO port output type register     Address offset: 0x04*/
	__vo uint32_t RTSR;   /*!< GPIO port output speed register     Address offset: 0x08*/
    __vo uint32_t FTSR;     /*!< GPIO port pull-up/pull-down register     Address offset: 0x0C*/
    __vo uint32_t SWIER;       /*!< GPIO port input data register     Address offset: 0x10*/
    __vo uint32_t PR;       /*!< GPIO port output data register     Address offset: 0x14*/
}EXTI_RegDef_t;

/**
 * @struct SYSCFG_RegDef_t
 * @brief peripheral register definition structure for SYSCFG.
 * Section 28, lesson 110
 * Use temp1 to identify which register is the EXTICR[temp1] pin
 * Use temp2 to identify the position of the EXTIx pins
 * SYSCFG_EXTICR1: EXTICR[0] = 0..3
 * SYSCFG_EXTICR2: EXTICR[1] = 4..7
 * SYSCFG_EXTICR3: EXTICR[2] = 8..11
 * SYSCFG_EXTICR4: EXTICR[3] = 12..15
 *
 * Example: PA5. Port A code is 0000
 * temp1 = 5/4 = 1 -> EXTICR[1]
 * temp2 = 5%4 = 1 -> 1*4 (shift by 4 because EXTIx use blocks of 4 bits). So in EXTI5 we must have 0000 assigned
 *
 * Example: PC13. Port C code is 0010
 * temp1 = 13/4 = 3 -> EXTICR[3]
 * temp2 = 13%4 = 1 -> 1*4 (shift by 4 because EXTIx use blocks of 4 bits). So in EXTI13 we must have 0010 assigned
 */
typedef struct
{
	__vo uint32_t MEMRMP;      		/*!< Used for specific configurations on memory remap		Address offset: 0x00*/
	__vo uint32_t PMC;    			/*!< Peripheral mode configuration register    				Address offset: 0x04*/
	__vo uint32_t EXTICR[4];  		/*!< External interrupt configuration register    			Address offset: 0x08-0x0C-0x10-0x14*/
    uint32_t RESERVED1[2];     		/*!< (???)												    Reserved: 0x18-0x1C*/
    __vo uint32_t CMPCR;       		/*!< Compensation cell control register      				Address offset: 0x2C*/
    uint32_t RESERVED2[2];     		/*!< (???)    												Reserved: 0x24-0x28*/
    __vo uint32_t CFGR;       		/*!< (???)   												Address offset: 0x2C*/
}SYSCFG_RegDef_t;



/* peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t) */

#define GPIOA (GPIO_RegDef_t*)GPIOA_BASE
#define GPIOB (GPIO_RegDef_t*)GPIOB_BASE
#define GPIOC (GPIO_RegDef_t*)GPIOC_BASE
#define GPIOD (GPIO_RegDef_t*)GPIOD_BASE
#define GPIOE (GPIO_RegDef_t*)GPIOE_BASE
#define GPIOF (GPIO_RegDef_t*)GPIOF_BASE
#define GPIOG (GPIO_RegDef_t*)GPIOG_BASE
#define GPIOH (GPIO_RegDef_t*)GPIOH_BASE
#define GPIOI (GPIO_RegDef_t*)GPIOI_BASE
#define GPIOJ (GPIO_RegDef_t*)GPIOJ_BASE
#define GPIOK (GPIO_RegDef_t*)GPIOK_BASE

#define RCC 	((RCC_RegDef_t*)RCC_BASE)

#define EXTI 	((EXTI_RegDef_t*)EXTI_BASE) //Section 28, lesson 109

#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASE) //Section 28, lesson 111

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 10) )

/* Clock enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN()      ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 23) )

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()      ( RCC->APB2ENR |= (1 << 14) )
#define SPI3_PCLK_EN()      ( RCC->APB2ENR |= (1 << 15) )
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= (1 << 13) )

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 18) )
#define USART4_PCLK_EN()      ( RCC->APB1ENR |= (1 << 19) )
#define USART5_PCLK_EN()      ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()      ( RCC->APB2ENR |= (1 << 5) )

/* Clock Enable Macros for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1<<14) )

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 10) )

/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 23) )

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 13) )

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 18) )
#define USART4_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 19) )
#define USART5_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 5) )

/* Clock Disable Macros for SYSCFG peripheral */
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1<<14) )

/* Macros to reset GPIOx peripherals */
//do..while condition zero loop - technique in C programming to execute multiple C statements using single macro
#define GPIOA_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 0)); 	(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 1)); 	(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 2)); 	(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 3)); 	(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 4)); 	(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 5)); 	(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 6)); 	(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 7)); 	(RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 8)); 	(RCC->AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 9)); 	(RCC->AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 10)); 	(RCC->AHB1RSTR &= ~(1 << 10));}while(0)

/**
 * @def GPIO_BASEADDR_TO_CODE()
 * @param x
 * @brief returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	   ((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :\
										(x == GPIOJ) ? 9 :\
										(x == GPIOK) ? 10 :0 )

/* IRQ (Interrupt ReQuest) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: complete this list for others peripherals - page 372 RM - Table 61. Vector table for STM32F405xx/07xx and STM32F415xx/17xx */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/* Generic MACROS */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
