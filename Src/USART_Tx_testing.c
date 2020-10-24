/*
 * USART_Tx_testing.c
 *
 *  Created on: 16-Aug-2020
 *      Author: Jitender Kumar
 */


#include "stm32f030xx.h"
#include <string.h>
#include <stdio.h>

/*
 * USART1_Tx	--> PA9
 * USART1_Rx	--> PA10
 *
 * USART2_Tx	--> PA2
 * USART2_Rx	--> PA3
 *
 * */


USART_Handle_t USART1Handle;

// Transmit data buffer
char Tx_buffer[1024] = "STM32\n\r";

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


/*
 * USART1 GPIO pins configuration
 * */
void USART1_GPIO_Init(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 1; 		// For PA9 and PA10
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	//I2CPins.pGPIOx->AFR[0] = 1;

	// USART1_Tx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USARTPins);

	//USARTPins.pGPIOx->ODR |= ( 1 << 2 );

	// USART1_Rx
	//USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	//GPIO_Init(&USARTPins);
}

/*
 * USART peripheral configuration
 * */
void USART1_Init(void)
{
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_BaudRate = USART_STD_BAUD_9600;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1Handle.USART_Config.USART_Parity = USART_PARITY_DISABLE;

	USART_Init(&USART1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIO_Btn;
	// For GPIO Button
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_Btn);
}

void Murtaza_USART_Init(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->pUSARTx == USART1)
	{
		//if(is_usart_en[0])
			//return 1; // 1 = usart1 already enabled
		//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;								// 00020000
//		RCC->AHBENR |= 0x00020000;								// 00020000
//
//		GPIOA->MODER = ((GPIOA->MODER & 0xffc3ffff) | 0x00280000);
//		GPIOA->OTYPER |= 0x00000600;
//		GPIOA->PUPDR = ((GPIOA->PUPDR & 0xffc3ffff) | 0x00100000);
//		GPIOA->AFR[1] = ((GPIOA->AFR[1] & 0xfffff00f) | 0x00000110);
//		GPIOA->OSPEEDR  |= 0x003c0000;
		//RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		//NVIC_EnableIRQ(USART1_IRQn);
		//NVIC->ISER[0] = (1 << ((uint32_t)(USART1_IRQn) & 0x1F));
		//USARTx->CR1 = 0x0000002d;
		//USARTx->BRR =(uint16_t)(48000000/38400);
		return 0;
	}
	else if (pUSARTHandle->pUSARTx == USART2)
	{
		//if(is_usart_en[1])
		//return 2; // 2 = usart2 already enabled
		//RCC_RegDef_t RCC;

		//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;								// 00020000
		RCC->AHBENR |= 0x00020000;

		//GPIO_RegDef_t GPIOA;

		GPIOA->MODER = ((GPIOA->MODER & 0xffffff0f) | 0x000000a0);
		GPIOA->OTYPER |= 0x0000000c;
		GPIOA->PUPDR = ((GPIOA->PUPDR & 0xffffff0f) | 0x00100050);
		GPIOA->AFR[0] = ((GPIOA->AFR[0] & 0xffff00ff) | 0x00001100);
		GPIOA->OSPEEDR  |= 0x000000f0;

		//RCC->APB1ENR |= RCC_APB1ENR_USART2EN;							// 00020000
		RCC->APB1ENR |= 0x00020000;

		//NVIC_EnableIRQ(USART2_IRQn);
		//NVIC->ISER[0] = (1 << ((uint32_t)(USART2_IRQn) & 0x1F));
		pUSARTHandle->pUSARTx->CR1 = 0x0000002d;
		pUSARTHandle->pUSARTx->BRR =((uint16_t)(48000000/115200));
		return 0;
	}
}

void USART_write_char(USART_Handle_t *pHandle, uint8_t data)
{

	uint32_t *temp = 0x4000441c;
	//*temp = 0x50;
	uint32_t temp2 = *temp;

	//uint32_t tt = pHandle->pUSARTx->TDR;

	uint32_t *temp_data_add = 0x40004428;

	//pHandle->pUSARTx->TDR = 0x00ff & data;
	*temp_data_add = 0x00ff & data;
	//uint32_t *temp = 0x4001381cU;
	//uint32_t temp2 = *temp;
   //while((pHandle->pUSARTx->ISR & 0x00000040) != 0x00000040);
	while((temp2 & 0x00000040) != 0x00000040);


}

void USART_write_str(USART_Handle_t *pHandle,char* data)
{
	while(*data)
	{
		USART_write_char(pHandle->pUSARTx,*data);
		data++;
	}
}

int main(void)
{

 	GPIO_ButtonInit();

	// Initialize the GPIO pins for USART communication
	USART1_GPIO_Init();

	//RCC_RegDef_t RCC_Reg;
	//RCC_Reg.CFGR3 |= ( 3 << 0 );
	uint32_t *rcc_temp = 0x40021030;
	//uint32_t value = *rcc_temp;

	//*rcc_temp = 0x3;

	// Initialize the USART peripheral configuration
	USART1_Init();

	// enable the USART peripheral
	USART_PeripheralControl(USART1, ENABLE);

	//USART1Handle.pUSARTx->CR1 |= ( 1 << USART_CR1_TE );

	while(1)
	{
		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );

		delay();

		// send data
		USART_SendData(&USART1Handle, (uint8_t*)Tx_buffer, strlen(Tx_buffer));

		uint8_t b = 0;

	}

//		USART_Handle_t USART_Handle;
//		USART_Handle.pUSARTx = USART1;
//		Murtaza_USART_Init(&USART_Handle);
//uint16_t x=1000;
//uint32_t *clk = 0x40021034;
//*clk |= ( 1 << 0 );
//
// 	while(1)
// 	{
// 		// wait while button is pressed
// 		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );
// 		delay();
//
//
//
// 		//USART1Handle.pUSARTx = USART2;
//
// 		USART_Handle.pUSARTx->BRR = 0x1a0;
// 		USART_write_char(&USART_Handle, 'a');
//
// 		uint8_t b = 0;
// 			x = x + 50;
// 			//delay();
//
// 			//if( x == 200000)
// 	}

}


