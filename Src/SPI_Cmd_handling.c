/*
 * SPI_Cmd_handling.c
 *
 *  Created on: 20-Jun-2020
 *      Author: Jitender Kuamr
 */


/*
 * PA6 			--> MISO
 * PA7 			--> MOSI
 * PA5 			--> SCLK
 * PA4 			--> NSS
 * ALT fn mode 	--> AF0
 * */

/*
 * PB14 		--> MISO
 * PB15 		--> MOSI
 * PB13 		--> SCLK
 * PB12 		--> NSS
 * ALT fn mode 	--> AF0
 * */


#include "stm32f030xx.h"
//#include "stm32f030xx_spi_driver.h"
//#include "stm32f030xx_gpio_driver.h"
#include <string.h>

// Commands codes
#define	COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define	COMMAND_ID_READ			0x54

#define LED_ON		0
#define LED_OFF		1

// Arduino Analog Pins
#define ANALOG_PIN_0		0
#define ANALOG_PIN_1		1
#define ANALOG_PIN_2		2
#define ANALOG_PIN_3		3
#define ANALOG_PIN_4		4
#define ANALOG_PIN_5		5

// Arduino LED
#define LED_PIN		9


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

/*
 * SPI1 GPIO pins configuration
 * */
void SPI2_GPIO_Init(void)
{
	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 0;			// Configure in AF0
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//

	// SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI_Pins);

	// MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI_Pins);

	// MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI_Pins);

	// NSS
	//SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI_Pins);
}

/*
 * SPI1 peripheral configuration
 * */
void SPI2_Init(void)
{
	SPI_Handle_t SPI1_handle;
	SPI1_handle.pSPIx = SPI2;
	SPI1_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1_handle.SPI_Config.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV4;	// generate clock of 4MHz
	SPI1_handle.SPI_Config.SPI_DS = SPI_DS_8BITS;
	SPI1_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI1_handle.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;	// Hardware slave management is enable for NSS pin.

	SPI_Init(&SPI1_handle);
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

uint8_t SPI_VerifyResponse(uint8_t AckByte){
	if(AckByte == 0xF5)
	{
		// ACK
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI2_GPIO_Init();

	SPI2_Init();

	//SPI1_NSSP_Config(SPI1, ENABLE);

	//SPI_TXDMAEN(SPI1, ENABLE);

	//SPI_FRXTH(SPI1, ENABLE);

	//SPI_LSBFIRST(SPI1, ENABLE);


	// this makes NSS signal internally high and avoid MODF(Mode Fault) error
	// SPI_SSI_Config(SPI1, ENABLE);

	// Enable the SSOE(Slave Select Output Enable)
	/*
	 * Making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1, NSS will be pulled to LOW.
	 * when SPE=0, NSS will be pulled to HIGH.
	 * */
	SPI_SSOE_Config(SPI2, ENABLE);

	while(1)
	{
		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );
		delay();

		// enable SPI1 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// CMD_LED_CTRL <pin no> <value>
		uint8_t command_code = COMMAND_LED_CTRL;
		uint8_t Ack_Byte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &command_code, 1);

		// send some dummy bits(1 byte) to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		SPI_ReceiveData(SPI2, &Ack_Byte, 1);

		if(SPI_VerifyResponse(Ack_Byte))
		{
			// Send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}


		// CMD_SENSOR_READ <analog pin number> <>

		// wait while button is pressed
		while( ! GPIO_ReadFromInuptPin(GPIOA, GPIO_PIN_NO_0) );
		delay();

		command_code = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &command_code, 1);

		// send some dummy bits(1 byte) to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		SPI_ReceiveData(SPI2, &Ack_Byte, 1);

		if(SPI_VerifyResponse(Ack_Byte))
		{
			// Send arguments
			args[0] = ANALOG_PIN_0;
			// send arguments
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with data
			delay();

			// send some dummy bits(1 byte) to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			// Read the sensor data sent by slave now
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		// check SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		// disable SPI1 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
