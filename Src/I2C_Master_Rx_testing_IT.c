/*
 * I2C_Master_Rx_testing_IT.c
 *
 *  Created on: 13-Aug-2020
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

// Received data buffer
uint8_t data_buffer[32];
uint8_t RxComplete = RESET;

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
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 1; 		// For PB6 and PB7
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
	uint8_t commandCode;

	uint8_t length;

	GPIO_ButtonInit();

	// Initialize the GPIO pins for I2C communication
	I2C1_GPIO_Init();

	// Initialize the I2C peripheral configuration
	I2C1_Init();

	// I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1, ENABLE);

	// enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// send data when button is pressed

	while(1)
	{
		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );

		delay();

		commandCode = 0x52;

		// send data to slave
		while( I2C_MasterSendData_IT(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

		while( I2C_MasterReceiveData_IT(&I2C1Handle, data_buffer, 23, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY );

		// wait till Rx is completed
		while( RxComplete != SET );

		data_buffer[24] = '\0';

		RxComplete = RESET;
	}

	return 0;
}


void I2C1_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	if ( event == I2C_EV_TX_CMPLT )
	{
		printf("Tx is Complete\n");
	}else if ( event == I2C_EV_RX_CMPLT )
	{
		printf("Rx is Complete\n");
		RxComplete = SET;

	}else if ( event == I2C_ERROR_NACKF )
	{
		printf("Error : Ack Failure");
		// In master ack failure happens when slave fails to send ack for byte send from the master.
		I2C_CloseSendData(pI2CHandle);

		// generate the stop condition to release the Bus
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Hang in infinite loop
		while(1);

	}
}



