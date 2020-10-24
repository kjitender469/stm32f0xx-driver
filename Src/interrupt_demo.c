/*
 * interrupt_demo.c
 *
 *  Created on: 11-Jun-2020
 *      Author: Jitender Kumar
 */

#include <string.h>
#include "stm32f030xx.h"
#include "stm32f030xx_gpio_driver.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for(uint32_t i = 0; i < 500000/4; i++);
}

int main(void)
{
	GPIO_Handle_t GPIO_Led, GPIO_Btn;

	memset(&GPIO_Led, 0, sizeof(GPIO_Led));		/* To initialize the GPIO_Led variable to avoid taking garbage values */
	memset(&GPIO_Btn, 0, sizeof(GPIO_Led));

	// for onboard blue LED
	GPIO_Led.pGPIOx = GPIOC;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIO_Led);

	// For GPIO Button
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_Btn);

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0_1, NVIC_IRQ_PRIORITY_2);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);

	while(1);

//	while(1)
//	{
//		if(GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
//		{
//			delay();
//			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_8);
//		}
//	}
	return 0;
}

void EXTI0_1_IRQHandler(void)
{
	delay();
	// Interrupt handling first i.e to reset the Pending Register bit using driver API GPIO_IRQHandling() function.
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_8);
}




