/*
 * stm32f030xx_gpio_driver.h
 *
 *  Created on: 09-Jun-2020
 *      Author: Jitender Kumar
 *      E-mail: kjitender469@gmail.com
 */

#ifndef INC_STM32F030XX_GPIO_DRIVER_H_
#define INC_STM32F030XX_GPIO_DRIVER_H_

#include "stm32f030xx.h"



/*
 * 	This is a configuration structure for GPIO pins
 * */
typedef struct
{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/* possible pin modes from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* possible pin speeds from @GPIO_PIN_SPEEDS */
	uint8_t GPIO_PinPuPdControl;	/* possible pin control from @GPIO_PIN_PULLUP_PULLDOWN */
	uint8_t GPIO_PinOPType;			/* possible pin output type from @GPIO_PIN_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;		/* possible pin alternate function from @GPIO_PIN_ALT_FN */

}GPIO_PinConfig_t;

/*
 * 	This is a Handle structure for GPIO pin
 * */
typedef struct
{

	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t 		*pGPIOx;		/* This holds the base address of GPIO port to which the pin belongs */
	GPIO_PinConfig_t 	GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 *
 * GPIO pin numbers
 * */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_PIN_MODES
 *
 * GPIO pin possible modes
 * */
#define GPIO_MODE_IN		0		// Input mode
#define GPIO_MODE_OUT		1		// Output mode
#define GPIO_MODE_ALTFN		2		// Alternate function mode
#define GPIO_MODE_ANALOG	3		// Analog mode
#define GPIO_MODE_IT_FT		4 		// Input falling edge trigger(IT_FT) used in case of Interrupt
#define GPIO_MODE_IT_RT		5		// Input rising edge trigger(IT_RT) used in case of Interrupt
#define GPIO_MODE_IT_RFT	6		// Input rising edge falling edge trigger(IT_RFT) used in case of Interrupt


/*
 * @GPIO_PIN_OUTPUT_TYPES
 *
 * GPIO pin possible output types
 * */
#define GPIO_OP_TYPE_PP		0		// Output type push pull
#define GPIO_OP_TYPE_OD		1		// Output type open drain


/*
 * @GPIO_PIN_SPEEDS
 *
 * GPIO pin possible output speeds
 * */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3


/*
 * @GPIO_PIN_PULLUP_PULLDOWN
 *
 * GPIO pin pull up pull down configuration
 * */
#define GPIO_PIN_NO_PUPD		0		// no pull up pull down
#define GPIO_PIN_PU			1		// pull up
#define GPIO_PIN_PD			2		// pull down



/**********************************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 * ********************************************************************************************/

/*
 * 	Peripheral Clock Setup
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


/*
 * 	Init and De-Init
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and write
 * */
uint8_t GPIO_ReadFromInuptPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInuptPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * 	IRQ Configuration and ISR handling
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F030XX_GPIO_DRIVER_H_ */



