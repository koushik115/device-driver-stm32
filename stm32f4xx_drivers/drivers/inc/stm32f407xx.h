/*
 * stm32f407xx.h
 *
 *  Created on: Aug 6, 2025
 *      Author: Koushik
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
/*
 * Memory Base Addresses
 */
#define FLASH_BASE_ADDR				0x08000000UL	// Main memory
#define SRAM1_BASE_ADDR				0x20000000UL	// SRAM1 Base Address
#define SRAM2_BASE_ADDR				0x2001C000UL	// SRAM2 Base Address
#define SYSTEM_MEMORY_BASE_ADDR		0x1FFF0000UL	// System Memory Base Address
#define SRAM_DEFAULT				SRAM1_BASE_ADDR // Default SRAM will be SRAM1 for the application

/*
 * AHBx and APBx Bus Peripheral Base Addresses
 */
#define AHB2_PERIPH_BASE_ADDR		0x50000000UL // AHB2 Bus Address (USB OTG FS)
#define AHB1_PERIPH_BASE_ADDR		0x40020000UL // AHB1 Bus Address (GPIOA)
#define APB2_PERIPH_BASE_ADDR		0x40010000UL // APB2 Bus Address (TIM1)
#define APB1_PERIPH_BASE_ADDR		0x40000000UL // APB1 Bus Address (TIM2)
#define PERIPH_BASE_ADDR			APB1_PERIPH_BASE_ADDR

/*
 * Base Address of Peripheral which are hanging on AHB1 Bus
 */
#define RCC_PERIPH_BASE_ADDR		(AHB1_PERIPH_BASE_ADDR + 0x3800)
#define GPIOA_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x2000)

/*
 * Base Address of Peripheral which are hanging on APB1 Bus
 */
#define I2C1_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5800)
#define I2C3_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5C00)
#define SPI2_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x3C00)
#define USART2_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_PERIPH_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x5000)

/*
 * Base Address of Peripheral which are hanging on APB2 Bus
 */
#define SPI1_PERIPH_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x3000)
#define USART1_PERIPH_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x1000)
#define USART6_PERIPH_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x1400)
#define EXTI_PERIPH_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x3C00)
#define SYSCFG_PERIPH_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x3800)


/*********************************** Peripheral Register Structures ***************************/
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t Reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t Reserved2;
	volatile uint32_t Reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t Reserved4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t Reserved5;
	volatile uint32_t Reserved6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t Reserved7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t Reserved8;
	volatile uint32_t Reserved9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t Reserved10;
	volatile uint32_t Reserved11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO_RegDef_t;

/*
 * Peripheral Definition
 */
#define RCC		((RCC_RegDef_t *)RCC_PERIPH_BASE_ADDR)
#define GPIOA	((GPIO_RegDef_t *)GPIOA_BASE_ADDR)
#define GPIOB	((GPIO_RegDef_t *)GPIOB_BASE_ADDR)
#define GPIOC	((GPIO_RegDef_t *)GPIOC_BASE_ADDR)
#define GPIOD	((GPIO_RegDef_t *)GPIOD_BASE_ADDR)
#define GPIOE	((GPIO_RegDef_t *)GPIOE_BASE_ADDR)
#define GPIOF	((GPIO_RegDef_t *)GPIOF_BASE_ADDR)
#define GPIOG	((GPIO_RegDef_t *)GPIOG_BASE_ADDR)
#define GPIOH	((GPIO_RegDef_t *)GPIOH_BASE_ADDR)
#define GPIOI	((GPIO_RegDef_t *)GPIOI_BASE_ADDR)

/*
 * Clock Enable Macro for GPIOx
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macro for I2Cx
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macro for SPIx
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macro for USARTx/UARTx
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable Macro for SYSCFG
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macro for GPIOx
 */
#define GPIOA_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DS()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macro for I2Cx
 */
#define I2C1_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macro for SPIx
 */
#define SPI1_PCLK_DS()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macro for USARTx/UARTx
 */
#define USART1_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DS()		(RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock Disable Macro for SYSCFG
 */
#define SYSCFG_PCLK_DS()	(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407XX_H_ */
