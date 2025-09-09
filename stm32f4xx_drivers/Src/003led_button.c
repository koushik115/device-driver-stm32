/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Koushik
 * @brief          : Main program body
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

static void delayUS(uint32_t delay) {
	while(delay--);
	return;
}

int main(void)
{
    /* Loop forever */
	GPIO_Handle_t GPIOUserConfig, GPIOButtonConfig;
	GPIOUserConfig.pGPIOx = GPIOD;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PUSHPULL;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriphClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOUserConfig);
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GPIOUserConfig);
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GPIOUserConfig);
	GPIOUserConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GPIOUserConfig);

	// Configuration for button
	GPIOButtonConfig.pGPIOx = GPIOB;
	GPIOButtonConfig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOButtonConfig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOButtonConfig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULLUP_PULLDOWN;
	GPIOButtonConfig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriphClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOButtonConfig);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0) {
			delayUS(1000000 / 4);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}
	for(;;);
}
