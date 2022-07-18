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
#define EXT1_BASE              (APB2PERIPH_BASE + 0x3C00)
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

#define RCC ((RCC_RegDef_t*)RCC_BASE)

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

/* Generic MACROS */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
