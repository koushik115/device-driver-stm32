/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Koushik
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

static void delayUS(uint32_t delay) {
	while(delay--);
	return;
}

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    /* Loop forever */
	GPIO_Handle_t GPIOUserConfig;
	GPIOUserConfig.pGPIOx = GPIOD;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PUSHPULL;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriphClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOUserConfig);

	while(1) {
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		delayUS(1000000);
	}
	for(;;);
}
