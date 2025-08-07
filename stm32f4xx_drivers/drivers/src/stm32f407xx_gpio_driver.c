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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable);
void GPIO_IRQHandling(uint8_t pinNumber);

