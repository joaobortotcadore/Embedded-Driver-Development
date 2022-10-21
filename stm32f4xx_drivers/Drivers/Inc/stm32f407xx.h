/*
 * stm32f407xx.h
 *
 *  Created on: May 3, 2022
 *      Author: joao.cadore
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h> //for NULL
#include <stdint.h> //for uint32_t typedef

/* Volatile is used to inform the compiler that the variable value can be
 * changed any time without any task given by the source code. Volatile is
 * usually applied to a variable when we are declaring it. */
#define __vo volatile
#define __weak __attribute__((weak))

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

typedef struct
{
    __vo uint32_t CR1;      /*!< SPI control register 1 (not used in I2S mode)      Address offset: 0x00*/
    __vo uint32_t CR2;      /*!< SPI control register 2                             Address offset: 0x04*/
    __vo uint32_t SR;       /*!< SPI status register                                Address offset: 0x08*/
    __vo uint32_t DR;       /*!< SPI data register                                  Address offset: 0x0C*/
    __vo uint32_t CRCPR;    /*!< SPI CRC polynomial register (not used in I2S mode) Address offset: 0x10*/
    __vo uint32_t RXCRCR;   /*!< SPI RX CRC register (not used in I2S mode)         Address offset: 0x14*/
    __vo uint32_t TXCRCR;   /*!< SPI TX CRC register (not used in I2S mode)         Address offset: 0x18*/
    __vo uint32_t I2SCFGR;  /*!< SPI_I2S configuration register                     Address offset: 0x1C*/
    __vo uint32_t I2SPR;    /*!< SPI_I2S prescaler register                         Address offset: 0x20*/
}SPI_RegDef_t;

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
 * @struct I2C_RegDef_t
 * @brief peripheral register definition structure for I2C.
 * Section 53, lesson 182
 */
typedef struct
{
    __vo uint32_t CR1;      /*!< Control register 1                     Address offset: 0x00*/
    __vo uint32_t CR2;      /*!< Control register 2                     Address offset: 0x04*/
    __vo uint32_t OAR1;     /*!< I2C Own address register 1             Address offset: 0x08*/
    __vo uint32_t OAR2;     /*!< I2C Own address register 2             Address offset: 0x0C*/
    __vo uint32_t DR;       /*!< I2C Data register                      Address offset: 0x10*/
    __vo uint32_t SR1;      /*!< I2C Status register 1                  Address offset: 0x14*/
    __vo uint32_t SR2;      /*!< I2C Status register 2                  Address offset: 0x18*/
    __vo uint32_t CCR;      /*!< I2C Clock control register             Address offset: 0x1C*/
    __vo uint32_t TRISE;    /*!< I2C TRISE register                     Address offset: 0x20*/
    __vo uint32_t FLTR;     /*!< I2C FLTR register                      Address offset: 0x24*/
}I2C_RegDef_t;

/**
 * @struct USART_RegDef_t
 * @brief peripheral register definition structure for USART.
 * Section 71, lesson 250
 */
typedef struct
{
    __vo uint32_t SR;         /*!< TODO,                                            Address offset: 0x00 */
    __vo uint32_t DR;         /*!< TODO,                                            Address offset: 0x04 */
    __vo uint32_t BRR;        /*!< TODO,                                            Address offset: 0x08 */
    __vo uint32_t CR1;        /*!< TODO,                                            Address offset: 0x0C */
    __vo uint32_t CR2;        /*!< TODO,                                            Address offset: 0x10 */
    __vo uint32_t CR3;        /*!< TODO,                                            Address offset: 0x14 */
    __vo uint32_t GTPR;       /*!< TODO,                                            Address offset: 0x18 */
} USART_RegDef_t;

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

#define SPI1    ((SPI_RegDef_t*) SPI1_BASE) //section 36, lesson 135
#define SPI2    ((SPI_RegDef_t*) SPI2_BASE) //section 36, lesson 135
#define SPI3    ((SPI_RegDef_t*) SPI3_BASE) //section 36, lesson 135

#define I2C1    ((I2C_RegDef_t*) I2C1_BASE) //section 53, lesson 181
#define I2C2    ((I2C_RegDef_t*) I2C2_BASE) //section 53, lesson 181
#define I2C3    ((I2C_RegDef_t*) I2C3_BASE) //section 53, lesson 181

#define USART1  ((USART_RegDef_t*)USART1_BASE) //section 71, lesson 250
#define USART2  ((USART_RegDef_t*)USART2_BASE) //section 71, lesson 250
#define USART3  ((USART_RegDef_t*)USART3_BASE) //section 71, lesson 250
#define UART4   ((USART_RegDef_t*)UART4_BASE) //section 71, lesson 250
#define UART5   ((USART_RegDef_t*)UART5_BASE) //section 71, lesson 250
#define USART6  ((USART_RegDef_t*)USART6_BASE) //section 71, lesson 250

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
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= (1 << 13) )

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()      ( RCC->APB1ENR |= (1 << 19) )
#define USART5_PCLK_EN()      ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()      ( RCC->APB1ENR |= (1 << 5) )

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
#define SPI2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 13) )

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 18) )
#define USART4_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 19) )
#define USART5_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 5) )

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

/* Macros to reset SPIx peripherals */
//do..while condition zero loop - technique in C programming to execute multiple C statements using single macro
//SPI1RST APB2RSTR
#define SPI1_REG_RESET()  		do{(RCC->APB1RSTR |= (1 << 12)); 	(RCC->APB1RSTR &= ~(1 << 12));}while(0)
//SPI2RST APB1RSTR
//SPI3RST APB1RSTR
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
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_I2C1_EV      31 //section 61, lecture 216
#define IRQ_NO_I2C1_ER      32 //section 61, lecture 216
#define IRQ_NO_USART1       37 //section 74, lecture 261
#define IRQ_NO_USART2       38 //section 74, lecture 261
#define IRQ_NO_USART3       39 //section 74, lecture 261
#define IRQ_NO_UART4        52 //section 74, lecture 261
#define IRQ_NO_UART5        53 //section 74, lecture 261
#define IRQ_NO_USART6       71 //section 74, lecture 261

#define NVIC_IRQ_PRIO0      0
#define NVIC_IRQ_PRIO1      1
#define NVIC_IRQ_PRIO2      2
#define NVIC_IRQ_PRIO3      3
#define NVIC_IRQ_PRIO4      4
#define NVIC_IRQ_PRIO5      5
#define NVIC_IRQ_PRIO6      6
#define NVIC_IRQ_PRIO7      7
#define NVIC_IRQ_PRIO8      8
#define NVIC_IRQ_PRIO9      9
#define NVIC_IRQ_PRIO10     10
#define NVIC_IRQ_PRIO11     11
#define NVIC_IRQ_PRIO12     12
#define NVIC_IRQ_PRIO13     13
#define NVIC_IRQ_PRIO14     14
#define NVIC_IRQ_PRIO15     15

/* Generic MACROS */
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/* BIT POSITION DEFINITIONS OF SPI PERIPHERAL */
/* Bit position definitions of SPI_CR1 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/* Bit position definitions of SPI_CR2 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
/* reserved 				3*/
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/* Bit position definitions of SPI_SR */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/* BIT POSITION DEFINITIONS OF I2C PERIPHERAL */
/* Bit position definitions of I2C_CR1 */
#define I2C_CR1_PE                      0
#define I2C_CR1_NOSTRETCH               7
#define I2C_CR1_START                   8
#define I2C_CR1_STOP                    9
#define I2C_CR1_ACK                     10
#define I2C_CR1_SWRST                   15

/* Bit position definitions I2C_CR2 */
#define I2C_CR2_FREQ                    0
#define I2C_CR2_ITERREN                 8
#define I2C_CR2_ITEVTEN                 9
#define I2C_CR2_ITBUFEN                 10

/* Bit position definitions I2C_OAR1 */
#define I2C_OAR1_ADD0                    0
#define I2C_OAR1_ADD71                   1
#define I2C_OAR1_ADD98                   8
#define I2C_OAR1_ADDMODE                15

/* Bit position definitions I2C_SR1 */
#define I2C_SR1_SB                      0
#define I2C_SR1_ADDR                    1
#define I2C_SR1_BTF                     2
#define I2C_SR1_ADD10                   3
#define I2C_SR1_STOPF                   4
#define I2C_SR1_RXNE                    6
#define I2C_SR1_TXE                     7
#define I2C_SR1_BERR                    8
#define I2C_SR1_ARLO                    9
#define I2C_SR1_AF                      10
#define I2C_SR1_OVR                     11
#define I2C_SR1_TIMEOUT                 14

/* Bit position definitions I2C_SR2 */
#define I2C_SR2_MSL                     0
#define I2C_SR2_BUSY                    1
#define I2C_SR2_TRA                     2
#define I2C_SR2_GENCALL                 4
#define I2C_SR2_DUALF                   7

/* Bit position definitions I2C_CCR */
#define I2C_CCR_CCR                      0
#define I2C_CCR_DUTY                    14
#define I2C_CCR_FS                      15

/*******************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************/
/* Bit position definitions USART_CR1 */
#define USART_CR1_SBK                   0
#define USART_CR1_RWU                   1
#define USART_CR1_RE                    2
#define USART_CR1_TE                    3
#define USART_CR1_IDLEIE                4
#define USART_CR1_RXNEIE                5
#define USART_CR1_TCIE                  6
#define USART_CR1_TXEIE                 7
#define USART_CR1_PEIE                  8
#define USART_CR1_PS                    9
#define USART_CR1_PCE                   10
#define USART_CR1_WAKE                  11
#define USART_CR1_M                     12
#define USART_CR1_UE                    13
#define USART_CR1_OVER8                 15

/* Bit position definitions USART_CR2 */
#define USART_CR2_ADD                   0
#define USART_CR2_LBDL                  5
#define USART_CR2_LBDIE                 6
#define USART_CR2_LBCL                  8
#define USART_CR2_CPHA                  9
#define USART_CR2_CPOL                  10
#define USART_CR2_STOP                  12
#define USART_CR2_LINEN                 14

/* Bit position definitions USART_CR3 */
#define USART_CR3_EIE                   0
#define USART_CR3_IREN                  1
#define USART_CR3_IRLP                  2
#define USART_CR3_HDSEL                 3
#define USART_CR3_NACK                  4
#define USART_CR3_SCEN                  5
#define USART_CR3_DMAR                  6
#define USART_CR3_DMAT                  7
#define USART_CR3_RTSE                  8
#define USART_CR3_CTSE                  9
#define USART_CR3_CTSIE                 10
#define USART_CR3_ONEBIT                11

/* Bit position definitions USART_SR */
#define USART_SR_PE                     0
#define USART_SR_FE                     1
#define USART_SR_NE                     2
#define USART_SR_ORE                    3
#define USART_SR_IDLE                   4
#define USART_SR_RXNE                   5
#define USART_SR_TC                     6
#define USART_SR_TXE                    7
#define USART_SR_LBD                    8
#define USART_SR_CTS                    9

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h" //lesson 184
#include "stm32f407xx_usart_driver.h" //lesson 250
#include "stm32f407xx_rcc_driver.h"
#endif /* INC_STM32F407XX_H_ */
