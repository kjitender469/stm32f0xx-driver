/*
 * I2C_Master_Tx_testing.c
 *
 *  Created on: 02-Jul-2020
 *      Author: Jitender Kumar
 */


#include "stm32f030xx.h"
#include <string.h>
#include <stdio.h>


/*
 * PB8 -> I2C1_SCL
 * PB9 -> I2C1_SDA
 *
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 * */


#define MASTER_ADDRESS 		0x65
#define SLAVE_ADDR			0x68

I2C_Handle_t I2C1Handle;

uint8_t data[] = "WeDFDFDFRGu\n";
uint8_t data_m = 'J';

void ref_I2C(void);
void I2C_Write(I2C_Handle_t I2Cx, uint8_t slaveAddress, uint8_t val);
void I2C_Write8(I2C_Handle_t I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t val);


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


/*
 * I2C1 GPIO pins configuration
 * */
void I2C1_GPIO_Init(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 1; 		// For PB8 and PB9
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//I2CPins.pGPIOx->AFR[0] = 1;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

/*
 * I2C1 peripheral configuration
 * */
void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MASTER_ADDRESS;
	I2C1Handle.I2C_Config.I2C_SCLTiming = 0x2000090E;
	//I2C1Handle.I2C_Config.I2C_SCLTiming = 0x0000020B;		// Murtaza's Value
	//I2C1Handle.I2C_Config.I2C_SCLTiming = 0x00B01A4B;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIO_Btn;
	// For GPIO Button
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_Btn);
}

int main(void)
{
	GPIO_ButtonInit();

	// Initialize the GPIO pins for I2C communication
	I2C1_GPIO_Init();

	// Initialize the I2C peripheral configuration
	I2C1_Init();
	//ref_I2C();

	// enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// send data when button is pressed

	while(1)
	{
		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );

		delay();

		// send data to slave
		I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDR);

		// Murtaza's
		data_m = 'J';
		//I2C1Handle.pI2Cx = I2C1;
		//I2C_Write(I2C1Handle, SLAVE_ADDR, data_m);

		//data = 1;
		//I2C_Write(I2C1Handle, SLAVE_ADDR, data);
		//I2C_Write8(I2C1Handle, SLAVE_ADDR, 42, data);
	}

	return 0;
}



void ref_I2C()
{
	//i2c init
	RCC->AHBENR |= (uint32_t) 0x00040000;
	RCC->APB1ENR |= (uint32_t)0x00200000;

	GPIOB->MODER = (GPIOB->MODER & 0xffff0fff) | 0x0000a000;
	GPIOB->PUPDR = (GPIOB->PUPDR & 0xffff0fff) | 0x00005000;
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & 0xffff0fff);
	GPIOB->OTYPER  |= 0x00c0;
	GPIOB->AFR[0] = (GPIOB->AFR[0] & 0x00ffffff) | 0x11000000;


	I2C1->CR1 &= 0xfffffffe;
	I2C1->TIMINGR = 0x0000020B;
	I2C1->CR1 |= 0x00000001;
	I2C1->OAR1=0x00008000;
	I2C1->OAR2=0;
	I2C1->CR1 |= 0x00000001;
	//init end

}

void I2C_Write(I2C_Handle_t I2Cx, uint8_t slaveAddress, uint8_t val){

	//Wait until I2C isn't busy
	//while( ( I2Cx.pI2Cx->ISR ) )
	while((I2Cx.pI2Cx->ISR &= 0x00008000) != 0){};//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);
	//set tranfer handler to set transfer
	//I2C_TransferHandling(I2Cx, (slaveAddress << 1) , 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	I2Cx.pI2Cx->CR2 = (I2Cx.pI2Cx->CR2 & (~0x03ff67ff)) | ((slaveAddress << 1 ) | 1<<16 | 0x00002000 );

	//Ensure the transmit interrupted flag is set
	while((I2Cx.pI2Cx->ISR &= 0x00000002) == 0);//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);
	//Send the data wish to write
	I2Cx.pI2Cx->TXDR =  val;
	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
 while((I2Cx.pI2Cx->ISR &= 0x0000020) == 0); //while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);
	//Clear the stop flag for the next potential transfer or Receive
 I2Cx.pI2Cx->ICR = 0x00000020; //  I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);
	//small delay
	//delay_ms(1);
}

void I2C_Write8(I2C_Handle_t I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t val){

	//Wait until I2C isn't busy
while((I2Cx.pI2Cx->ISR &= 0x00008000) != 0){}; //	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == SET);

	//set tranfer handler to write 1
//	I2C_TransferHandling(I2Cx, (slaveAddress << 1), 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
I2Cx.pI2Cx->CR2 = (I2Cx.pI2Cx->CR2 & (~0x03ff67ff)) | ((slaveAddress << 1 ) | 1<<16 | 0x01000000 | 0x00002000 );

	//Ensure the transmit interrupted flag is set
	while((I2Cx.pI2Cx->ISR &= 0x00000002) == 0); //	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register we wish to write to
	//I2C_SendData(I2Cx, reg);
	I2Cx.pI2Cx->TXDR=reg;
	//Ensure that the transfer complete reload flag is
	//set, essentially a standard TC flag
	while((I2Cx.pI2Cx->ISR &= 0x00000080) == 0); //	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TCR) == RESET);
	//any start or stop conditions
//	I2C_TransferHandling(I2Cx,(slaveAddress << 1), 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
//	I2C_TransferHandling(I2Cx,(slaveAddress << 1), nbytes, I2C_AutoEnd_Mode, I2C_No_StartStop);
	I2Cx.pI2Cx->CR2 = (I2Cx.pI2Cx->CR2 & (~0x03ff67ff)) | ((slaveAddress << 1 ) | 1<<16 | 0x02000000 | 0x00000000 );

	//Again, wait until the transmit interrupted flag is set
	while((I2Cx.pI2Cx->ISR &= 0x00000002) == 0); //	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	//I2C_SendData(I2Cx, val);
	I2Cx.pI2Cx->TXDR = val;
	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
 while((I2Cx.pI2Cx->ISR &= 0x0000020) == 0); //while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for the next potential transfer
 I2Cx.pI2Cx->ICR = 0x00000020; //  I2C_ClearFlag(I2Cx, I2C_FLAG_STOPF);
	//delay_ms(2);
}

