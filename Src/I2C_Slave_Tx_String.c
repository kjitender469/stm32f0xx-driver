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

// Transmit data buffer
uint8_t Tx_buffer[32] = "STM32 Slave Mode Testing";

uint8_t command_code = 0;
uint8_t count = 0;

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = SLAVE_ADDR;
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

	// Configure the I2C Slave address in I2C_OA1
	//I2C_SlaveAddressConfig(I2C1, SLAVE_ADDR);

	// Enable GCEN Bit
	I2C_GCEN_Control(I2C1, ENABLE);

	// Enable SBC Bit
	I2C_SBC_Control(I2C1, ENABLE);

	// Disable Clock Stretching
	I2C_NOSTRETCH_Control(I2C1, DISABLE);

	// I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1, ENABLE);

	// Enabling Interrupt for slave communication events handling.
	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	// enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// send data when button is pressed

	while(1)
	{
		// Hang in Infinite loop
	}
}


void I2C1_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	if( event == I2C_EV_DATA_REQ )
	{
		// Master wants some data. Slave should send it.
		if( command_code == 0x51 )
		{
			// Send the length information of data to Master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buffer));

		}else if( command_code == 0x52 )
		{
			// Send the data to Master.
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buffer[count++]);
		}

	}else if( event == I2C_EV_DATA_RCV )
	{
		// Data is waiting for the slave to read. Slave has to read it.
		command_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if( event == I2C_ERROR_NACKF )
	{
		// This happens only during slave transmitting.
		// Master has sent the NACK so slave should understand that master doesn't need more data.
		command_code = 0xff;		// Invalidating the command_code
		count = 0;
	}else if( event == I2C_EV_STOP )
	{
		// This happens only during slave reception
		// Master has ended the I2C communication with slave.
	}
}



