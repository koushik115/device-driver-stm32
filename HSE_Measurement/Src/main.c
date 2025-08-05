/**
  ******************************************************************************
  * @file    main.c
  * @author  Koushik
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include<stdint.h>

#define RCC_BASE_ADDR              	0x40023800UL
#define RCC_CR_REG_OFFSET			0x00UL
#define RCC_CR_REG_ADDR				(RCC_BASE_ADDR + RCC_CR_REG_OFFSET)
#define RCC_CFGR_REG_OFFSET        	0x08UL
#define RCC_CFGR_REG_ADDR           (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )
#define RCC_AHB1ENR_REG_OFFSET		0x30UL
#define RCC_AHB1ENR_REG_ADDR		(RCC_BASE_ADDR + RCC_AHB1ENR_REG_OFFSET)

#define GPIOA_BASE_ADDR             0x40020000UL
#define GPIOA_MODER_REG_OFFSET		0x00UL
#define GPIOA_MODER_REG_ADDR		(GPIOA_BASE_ADDR + GPIOA_MODER_REG_OFFSET)
#define GPIOA_AFRH_REG_OFFSET		0x24UL
#define GPIOA_AFRH_REG_ADDR			(GPIOA_BASE_ADDR + GPIOA_AFRH_REG_OFFSET)

int main(void)
{
	// Set HSEON bit in RCC_CR Register
	uint32_t *pRCCCrReg = (uint32_t *)RCC_CR_REG_ADDR;
	*pRCCCrReg |= (1 << 16);

	// Wait until HSE Oscillator stables
	while(!(*pRCCCrReg & (1 << 17)));

	// Switch the system clock to HSE
	uint32_t *pRCCCfgrReg = (uint32_t *)RCC_CFGR_REG_ADDR;
	*pRCCCfgrReg |= (1 << 0);

	// Wait until clock status switches to HSE
	while(((*pRCCCfgrReg >> 2) & 0x03) != 0x01);

	// Configure the MCO1 to HSE
	*pRCCCfgrReg &= ~(0x03 << 21);
	*pRCCCfgrReg |= (1 << 22);

	// Enable the peripheral clock for GPIOA
	uint32_t *pRCCAhb1EnrReg = (uint32_t *)RCC_AHB1ENR_REG_ADDR;
	*pRCCAhb1EnrReg |= (1 << 0);

	uint32_t *pGPIOAModeRReg = (uint32_t *)GPIOA_MODER_REG_ADDR;
	*pGPIOAModeRReg &= ~(0x03 << 16);
	*pGPIOAModeRReg |= (0x02 << 16);

	uint32_t *pGPIOAAfrhReg = (uint32_t *)GPIOA_AFRH_REG_ADDR;
	*pGPIOAAfrhReg &= ~(0x0F << 0);
	for(;;);
}
