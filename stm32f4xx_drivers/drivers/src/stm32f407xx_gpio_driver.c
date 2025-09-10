/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 7, 2025
 *      Author: kaush
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable) {
	if (Enable) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DS();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DS();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DS();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DS();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DS();
		}
	}
}

/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t regValue = 0x00;
	// Non - Interrupt Mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG_MODE) {
		// Pin Mode Configuration
		regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= regValue;
	} else {
		// Interrupt Mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_INT_FALL_EDGE) {
			// 1. Configure the FTSR register
			EXTI->FTSR |= (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 2. Clear the corresponding RTSR register
			EXTI->RTSR &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<= GPIO_MODE_INT_RISE_EDGE) {
			// 1. Configure the RTSR register
			EXTI->RTSR |= (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 2. Clear the corresponding FTSR register
			EXTI->FTSR &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<= GPIO_MODE_INT_BOTH_EDGE) {
			// 1. Configure the FTSR register
			EXTI->FTSR |= (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// 2. Configure the RTSR register
			EXTI->RTSR |= (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO Port selection in SYSCFG_EXTICR
		uint8_t regAddrOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t regOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		regValue = 0x00;
		if (pGPIOHandle->pGPIOx == GPIOA)
			regValue = 0;
		else if (pGPIOHandle->pGPIOx == GPIOB)
			regValue = 1;
		else if (pGPIOHandle->pGPIOx == GPIOC)
			regValue = 2;
		else if (pGPIOHandle->pGPIOx == GPIOD)
			regValue = 3;
		else if (pGPIOHandle->pGPIOx == GPIOE)
			regValue = 4;
		else if (pGPIOHandle->pGPIOx == GPIOF)
			regValue = 5;
		else if (pGPIOHandle->pGPIOx == GPIOG)
			regValue = 6;
		else if (pGPIOHandle->pGPIOx == GPIOH)
			regValue = 7;
		else if (pGPIOHandle->pGPIOx == GPIOI)
			regValue = 8;

		// Enable the SYSCFG clock
		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[regAddrOffset] &= ~(0x0F << (regOffset * 4));
		SYSCFG->EXTICR[regAddrOffset] |= (regValue << (regOffset * 4));

		// Configure the interrupt masking using IMR register
		EXTI->IMR |= (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// Pin Speed Configuration
	regValue = 0x00;
	regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= regValue;

	// Pull-Up/Pull-Down Configuration
	regValue = 0x00;
	regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= regValue;

	// Output Configuration
	regValue = 0x00;
	regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= regValue;

	// If mode is configured as alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_MODE) {
		// Alternate function configuration
		regValue = 0x00;
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode
					<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL &= ~(0x0F
					<< (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= regValue;
		} else {
			uint8_t pinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			regValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode
					<< (4 * pinOffset));
			pGPIOHandle->pGPIOx->AFRH &= ~(0x0F << (4 * pinOffset));
			pGPIOHandle->pGPIOx->AFRH |= regValue;
		}
	}
}

void GPIO_Deinit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	return (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x1);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) (pGPIOx->IDR);
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {
	if (value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << pinNumber);
	else if (value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(1 << pinNumber);
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber);
}

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable) {
	if (Enable) {
		if (IRQNumber < 32) {
			*NVIC_ISER0 |= (0x01 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ISER1 |= (0x01 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (0x01 << (IRQNumber % 32));
		}
	} else {
		if (IRQNumber < 32) {
			*NVIC_ICER0 |= (0x01 << IRQNumber);
		} else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ICER1 |= (0x01 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (0x01 << (IRQNumber % 32));
		}
	}
}

void GPIOIRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t NVIC_IPRx = IRQNumber / 4;
	uint8_t NVIC_IPRx_Offset = IRQNumber % 4;

	uint8_t shift_amount = (NVIC_IPRx_Offset * 8) + 4;

	*(NVIC_IPR0 + NVIC_IPRx) &= ~(IRQPriority << shift_amount);
	*(NVIC_IPR0 + NVIC_IPRx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t pinNumber);

