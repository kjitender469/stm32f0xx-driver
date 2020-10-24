/*
 * stm32f030xx_gpio_driver.c
 *
 *  Created on: 09-Jun-2020
 *      Author: Jitender Kumar
 *      E-mail: kjitender469@gmail.com
 */

#include "stm32f030xx_gpio_driver.h"

/**********************************************************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/


/*
 * 	Peripheral Clock Setup
 * */

/**********************************************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 * *********************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
	}
}



/*
 * 	Init and De-Init
 * */

/**********************************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize the GPIO Port
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * *********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configure the mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// The Non Interrupt Mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing the two bits of GPIOx pin
		pGPIOHandle->pGPIOx->MODER |= temp;														// setting the two bits of GPIOx pin

	}else
	{
		// Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// configure the FTSR (Falling Trigger Selection Register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding RTSR(Rising Trigger Selection Register) bit.
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// configure the RTSR (Rising Trigger Selection Register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding FTSR(Falling Trigger Selection Register) bit.
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// configure the both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);

		// Enable SYSCFG Peripheral Clock
		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing the two bits of GPIOx pin
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;														// setting the two bits of GPIOx pin
	temp = 0;

	// Configure the pull-up pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// clearing the two bits of GPIOx pin
	pGPIOHandle->pGPIOx->PUPDR |= temp;															// setting the two bits of GPIOx pin
	temp = 0;

	// Configure the Output Type Settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clearing one bit of GPIOx pin
	pGPIOHandle->pGPIOx->OTYPER |= temp;														// setting the two bits of GPIOx pin
	temp = 0;

	// Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the alternate functionality
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));											// clearing 4 bit of GPIOx pin
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));	// setting 4 bits of GPIOx pin
	}
}



/**********************************************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function De initialize the GPIO Port(Reset the GPIO Port)
 *
 * @param[in]		- base address of GPIO Port
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
}



/*
 * Data read and write
 * */

/**********************************************************************************************************
 * @fn				- GPIO_ReadFromInuptPin
 *
 * @brief			- Read the state of GPIOx Pin
 *
 * @param[in]		- base address of GPIOx Port
 * @param[in]		- PinNumber of GPIOx Port
 *
 * @return			- 0 or 1
 *
 * @Note			-
 *
 * *********************************************************************************************************/
uint8_t GPIO_ReadFromInuptPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}



/**********************************************************************************************************
 * @fn				- GPIO_ReadFromInuptPort
 *
 * @brief			- Read the entire port of GPIOx
 *
 * @param[in]		- base address of GPIOx Port
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
uint16_t GPIO_ReadFromInuptPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}



/**********************************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Write 0 or 1 value to GPIOx pin
 *
 * @param[in]		- base address of GPIOx port
 * @param[in]		- pin number of GPIOx port
 * @param[in]		- value to write
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		// write 0 to the output data register at the corresponding pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}



/**********************************************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Write some value to the GPIOx Port
 *
 * @param[in]		- base address of GPIOx Port
 * @param[in]		- value to write
 *
 * @return			- none
 *
 * @Note			- none
 *
 * *********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}



/**********************************************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- toggle the pin of GPIOx
 *
 * @param[in]		- base address of GPIOx port
 * @param[in]		- pin number of GPIOx port
 *
 * @return			- none
 *
 * @Note			- none
 *
 * *********************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*
 * 	IRQ Configuration and ISR handling
 * */

/**********************************************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- This determine the processor specific configuration for Interrupt at the processor side.
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		// program ISER register
		*NVIC_ISER |= (1 << IRQNumber);

	}else
	{
		// program ISER register
		*NVIC_ICER |= (1 << IRQNumber);
	}
}



/**********************************************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- Each priority field holds a priority value, 0-192. The processor implements only bits[7:6] of each field,
 *   				  bits [5:0] read as zero and ignore writes. This means writing 255 to a priority register saves value 192
 *   				  to the register.
 *
 * *********************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;				/* To identify which IPR register */
	uint8_t IPRx_section = IRQNumber % 4;		/* To identify which section of IPR register is to program. Each section is of 8 bits. */

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	/* In Cortex M0 2 bits are implemented. Bit[7:6] bits [5:0] read as zero and ignore writes. */
	*(NVIC_PR_BASE_ADDR + IPRx) |= ( IRQPriority << shift_amount );
}



/**********************************************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 * *********************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI pr register corresponding to the pin number.
	if(EXTI->PR & (1 << PinNumber))		// means interrupt is pending so clear that pending register bit.
	{
		// clear pending register(PR) bit.
		EXTI->PR |= (1 << PinNumber);		// 1-clear	0-Set
	}
}



