/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Aug 7, 2025
 *      Author: kaush
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 *  GPIO Pin Number
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/*
 *  GPIO Modes
 */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT 			1
#define GPIO_MODE_ALT_MODE			2
#define GPIO_MODE_ANALOG_MODE		3
#define GPIO_MODE_INT_FALL_EDGE		4
#define GPIO_MODE_INT_RISE_EDGE 	5
#define GPIO_MODE_INT_BOTH_EDGE 	6

/*
 *  GPIO Output Types
 */
#define GPIO_OUTPUT_TYPE_PUSHPULL	0
#define GPIO_OUTPUT_TYPE_OPENDRAIN  1

/*
 *  GPIO Output Speeds
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/*
 *  GPIO Pull-Up and Pull-Down Configuration
 */
#define GPIO_NO_PULLUP_PULLDOWN		0
#define GPIO_PIN_PULLUP				1
#define GPIO_PIN_PULLDOWN			2
/*************************** APIs Supported  by this driver *********************************/

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable);

/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
